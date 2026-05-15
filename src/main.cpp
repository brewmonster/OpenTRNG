#include <iomanip>
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <queue>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <numeric>
#include <filesystem>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <pthread.h>

#include <libcamera/libcamera.h>

#include "Estimator.hpp"
#include "EntropySource.hpp"
#include "HashDigester.hpp"
#include "OutputPort.hpp"

using namespace libcamera;
using namespace std::chrono_literals;

#define NUM_BACKGROUND_THREADS 2

class TRNGApp {

private:

    std::vector<std::unique_ptr<EntropySource>> sources;
    std::unique_ptr<CameraManager> cm;

	std::vector<uint8_t>* hashedFrameBuffer = new std::vector<uint8_t>;
	
    OutputPort output;

    std::vector<std::thread> cameraThreads;
    std::thread hashThread;
	std::thread writeThread;
	OutputFlags flags;
	unsigned int hashNum = 0;

    std::atomic<bool> running{false};

	std::vector<std::vector<uint8_t>> lastHashes;
	std::vector<uint8_t>* lastHashPtr;
	std::atomic_bool newHash;

	size_t HASH_LENGTH;

	OutputSignal outputSignal;

    std::vector<uint8_t> hashBuffer(const std::vector<uint8_t>& hash) {
        HashDigester digester(EVP_sha256());
        digester.update(hash.data(), hash.size());
        return digester.finalize();
    }
	
	bool test = false;


public:

    TRNGApp() {}

    ~TRNGApp() {
        running = false;
        for (auto& t : cameraThreads) if (t.joinable()) t.join();
        if (hashThread.joinable()) hashThread.join();
		if (writeThread.joinable()) writeThread.join();
        if (cm) cm->stop();
    }
	
	void showEntropy(const uint8_t* data, const size_t length){
		auto result = Estimator::estimateMarkov(data, length);
		std::cout << "\033[2K" << std::dec << length << " entries with: " << "\tp_max: " << result.p_max 
			<< ", H_min: " << result.h_min;
	}
	
	// 
    std::vector<std::vector<uint8_t>> processFrameQueue() {
		auto& source 	= sources[outputSignal.sourceID]; // query who sent the last frame
		auto  frame  	= source->getNextFrame(); // take newest frame delivered from a source
		auto  chunkSize = source->getChunkSize(HASH_LENGTH);

		if (frame->empty()) return std::vector<std::vector<uint8_t>>();

		 // outputs the current hash + entropy estimations
		if (flags.verbose) {
			std::cout <<"\033[4A[Source: " << outputSignal.sourceID << "] output:\n";
			showEntropy(frame->data(), frame->size());
			std::cout << " before Hash" << std::endl;
		}

		std::vector<std::vector<uint8_t>> hashes = {};

		// Putting a 5.2MB frame is a bit overkill for one 256 bit hash, so 
		// resegment the data appropriate to the chunk size.
        if (!frame->empty() && frame->data()[10] != uint8_t(0)) { 		// ensure data is not empty or 0s
			size_t len = frame->size() - chunkSize; 					// ensuring not reading past the last chunk.

			for (size_t i = 0; i < len; i+= chunkSize){
				std::vector<uint8_t> data(frame->data()+i, frame->data() + i + chunkSize); // new vectors of size chunkSize

				hashes.push_back(hashBuffer(data));
				lastHashPtr = &hashes.back();
				newHash = true;
			}
        }
		frame.reset();
		return hashes;
    }
	

    int start(int argc, char* argv[]) {
		
        // ------------------------ CLI PARSING ------------------------
        OutputFlags _flags;
        for (int i = 1; i < argc; ++i) {
            std::string arg(argv[i]);
            if      (arg == "-s" || arg == "--serial")  _flags.forceSerial 	= true;
            else if (arg == "-n" || arg == "--no-input")_flags.noPrompt    	= true;
			else if (arg == "-v" || arg == "--verbose") _flags.verbose    	= true;
			else if (arg == "-h" || arg == "--hashes") 	_flags.pseudo    	= false;
			else if (arg == "-l" || arg == "--logging") _flags.logging   	= true;
			else if (arg == "-lb" || arg == "--logging-binary") {_flags.logging = true; _flags.binary_output = true;}  
			else if (arg == "-f" || arg == "--force-calibrate") _flags.force_calibrate  = true;
            else std::cerr << "Unknown argument: " << arg << std::endl;
        }
		
		flags = _flags;
        if (!output.init(flags)) return -1;

        // ------------------------ CAMERA MANAGER ------------------------
        cm = std::make_unique<CameraManager>();
        cm->start();

        auto cameras = cm->cameras();
        if (cameras.empty()) {
            std::cout << "No cameras were identified on the system." << std::endl;
            cm->stop();
            return EXIT_FAILURE;
        }

        unsigned int numCameras = static_cast<unsigned int>(cameras.size()); 
        std::cout << numCameras << " camera(s) found." << std::endl;

        for (size_t i = 0; i < numCameras; ++i) {
            std::shared_ptr<Camera> camera = cm->get(cameras[i]->id());
            sources.emplace_back(std::make_unique<EntropySource>(camera, &running, &outputSignal, i));
            if (sources[i]->init(numCameras * 2) != 0) return -1;
        }

        // ------------------------ THREAD ASSIGNMENT ------------------------

        unsigned int numThreads = std::thread::hardware_concurrency();
        std::cout << numThreads << " hardware threads available." << std::endl;

        running = true;

        if (numThreads >= numCameras + NUM_BACKGROUND_THREADS) {
            std::cout << "Assigning one worker thread per source + one hash/output thread." << std::endl;
            for (size_t i = 0; i < numCameras; ++i) {
                cameraThreads.emplace_back([this, i]() {
                    sources[i]->workingLoop();
                });
            }
        } else {
            unsigned int workerCount = std::max(1u, numThreads - 1);
            std::cout << "Constrained: " << workerCount
                      << " worker thread(s) across " << numCameras << " sources." << std::endl;

            std::vector<size_t> order(numCameras);
            std::iota(order.begin(), order.end(), 0);
            std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
                return sources[a]->size() > sources[b]->size();
            });

            unsigned int slots = std::min(workerCount, numCameras);
            std::vector<std::vector<size_t>> groups(slots);
            for (size_t i = 0; i < numCameras; ++i)
                groups[i % slots].push_back(order[i]);

            for (auto& group : groups) {
                cameraThreads.emplace_back([this, group]() {
                    while (running)
                        for (size_t idx : group) {
							sources[idx]->workingLoop();
						}
                });
            }
        }
		
		for (size_t i = 0; i < sources.size(); ++i) {
			sources[i]->calibrate(600000, flags.force_calibrate); // call calibrate after worker thread started
		}
		

        // Hash + output TCPServer::writeData() polls for connections inline,
        // so no extra threads are needed for accept or send.
        hashThread = std::thread([this, flgs = flags]() {
			pthread_setname_np(pthread_self(), "hash generation thread");
			std::cout << "Hash Thread PID: " << syscall(SYS_gettid) << std::endl;
			std::cout << "\n\n\n";
			
			// enter blank sample data to test hash size given different algorithms output different sizes
			std::vector<uint8_t> hash = hashBuffer(std::vector<uint8_t>(1000)); // little trick since const T& allows literals parsed.

			HASH_LENGTH = hash.size();
			hashedFrameBuffer->resize(HASH_LENGTH);

			while (running) {
				{
					std::unique_lock<std::mutex> lock(outputSignal.mtx);
					outputSignal.cv.wait(lock, [&]{ return outputSignal.sourceID != -1; }); // wait until notified, if wakeup before, still locked if sourceID == -1.
				}
				lastHashes = processFrameQueue();

				if (!lastHashes.empty()) {
					
					for (const auto& lastHash : lastHashes) {
						if (lastHash.empty()) continue;

						if (flags.verbose) {
							hashedFrameBuffer->insert(hashedFrameBuffer->end(), lastHash.data(), lastHash.data() + lastHash.size());
							showEntropy(hashedFrameBuffer->data(),hashedFrameBuffer->size());
							std::cout << " after Hash" << std::endl;
							hashedFrameBuffer->erase(hashedFrameBuffer->begin(),
														 hashedFrameBuffer->begin()
														 + lastHash.size());

							std::cout << "[" << std::to_string(hashNum++) << "] Hash: "
									  << std::hex << std::setfill('0');
							for (const auto& byte : lastHash)
								std::cout << std::setw(2) << static_cast<int>(byte);
							std::cout << std::dec << std::endl;
						}
					}
				}
			}
		});
		
		writeThread = std::thread([this]() {
			pthread_setname_np(pthread_self(), "Network write thread");
			std::cout << "Write Thread PID: " << syscall(SYS_gettid) << std::endl;
			while(running){
				if (!lastHashPtr->empty()) newHash.wait(false);
				if (flags.pseudo){
					std::vector<uint8_t> pseudo_hash = hashBuffer(*lastHashPtr);
					while (!newHash){
						output.writeData(pseudo_hash.data(), pseudo_hash.size(), flags.binary_output);
						pseudo_hash = hashBuffer(pseudo_hash); // keep rehashing.
					}
					continue;
				} 
				output.writeData(lastHashPtr->data(), lastHashPtr->size(), flags.binary_output);
			}
		});

        std::cout << "TRNG started." << std::endl;
        return 0;
    }
};


int main(int argc, char* argv[]) {
    TRNGApp app;
    if (app.start(argc, argv)) {
        std::cerr << "Failed to initialize." << std::endl;
        return -1;
    }

    while (true) std::this_thread::sleep_for(std::chrono::seconds(1));
    return 0;
}
