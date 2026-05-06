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

#include "MarkovEstimator.hpp"
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
    std::queue<std::vector<uint8_t>*> frameQueue;
	std::mutex frm_q_mtx;
	
	
	std::vector<uint8_t>* pre_hash_output_data = new std::vector<uint8_t>;
	std::queue<int>* pre_hash_output_sizes = new std::queue<int>;
	
	std::vector<uint8_t>* post_hash_output_data = new std::vector<uint8_t>;
	
    OutputPort output;

    std::vector<std::thread> cameraThreads;
    std::thread hashThread;
	std::thread writeThread;
	OutputFlags flags;
	unsigned int hashNum = 0;

    std::atomic<bool> running{false};

	std::vector<std::vector<uint8_t>> lastHashes;
	bool newHash;
    std::vector<uint8_t> hashBuffer(std::vector<uint8_t> hash) {
        HashDigester digester(EVP_sha256());
        digester.update(hash.data(), hash.size());
        return digester.finalize();
    }
	
	static constexpr size_t HASH_OUTPUT_BITS   = 256;
	static constexpr size_t CALIBRATE_N_FRAMES = 30;
	static constexpr size_t FRAME_KEEP = 30;
	static constexpr float  SAFETY_MARGIN      = 2.0f;
	
	size_t chunkSize  = 1024;
	
	void calibrateChunkSize(){ // this will take a while if data is large enough.
		auto result = MarkovEstimator::estimate(post_hash_output_data->data(), post_hash_output_data->size());
		chunkSize = result.h_min * post_hash_output_data->size() / (SAFETY_MARGIN * HASH_OUTPUT_BITS);
	}


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
		auto result = MarkovEstimator::estimate(data, length);
		std::cout << "\033[2K" << std::dec << length << " entries with: " << "\tp_max: " << result.p_max 
			<< ", H_min: " << result.h_min;
	}
	
	
    std::vector<std::vector<uint8_t>> processFrameQueue() {
		
		std::unique_lock<std::mutex> lock(frm_q_mtx);
		if (frameQueue.empty()) {
			return std::vector<std::vector<uint8_t>>();
		}
		
        std::vector<uint8_t>* frame = frameQueue.front(); // take newest frame delivered from a source
		frameQueue.pop();
		lock.unlock();
		
		pre_hash_output_data->insert(pre_hash_output_data->end(), frame->data(), frame->data() + frame->size());
		pre_hash_output_sizes->push(frame->size());
		
		if (pre_hash_output_sizes->front())
			pre_hash_output_data->erase(pre_hash_output_data->begin(), pre_hash_output_data->begin() + pre_hash_output_sizes->front());
		
		pre_hash_output_sizes->pop();
		
		if (flags.verbose) { // outputs the current hash + entropy estimations
			std::cout <<"\033[3A";
			showEntropy(pre_hash_output_data->data(), pre_hash_output_data->size());
			std::cout << " before Hash" << std::endl;
		}
		
        if (!frame->empty()) {
			std::vector<std::vector<uint8_t>> hashes;
			
			size_t len = frame->size()-chunkSize; //ensuring not reading past the last chunk.
			for (size_t i = 0; i < len; i+= chunkSize){
				std::vector<uint8_t> data(frame->data()+i, frame->data() + i + chunkSize);  
				hashes.push_back(hashBuffer(data));
			}
			return hashes;
        }
		delete frame;
		return {};
    }
	

    int start(int argc, char* argv[]) {
		
        // ------------------------ CLI PARSING ------------------------
        OutputFlags _flags;
        for (int i = 1; i < argc; ++i) {
            std::string arg(argv[i]);
            if      (arg == "-s" || arg == "--serial")  _flags.forceSerial 	= true;
            else if (arg == "-n" || arg == "--no-input")_flags.noPrompt    	= true;
			else if (arg == "-v" || arg == "--verbose") _flags.verbose    	= true;
			else if (arg == "-p" || arg == "--pseudo") 	_flags.pseudo    	= true;
			else if (arg == "-l" || arg == "--logging") _flags.logging   	= true;
			else if (arg == "-f" || arg == "--force-calibrate") _flags.force_calibrate  = true;



            else std::cerr << "Unknown argument: " << arg << std::endl;
        }
		
		flags = _flags;
		
        // ------------------------ OUTPUT ------------------------
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
            sources.emplace_back(std::make_unique<EntropySource>(camera, &frameQueue, &running, frm_q_mtx));
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
		
		for (size_t i = 0; i<FRAME_KEEP; i++) 
			pre_hash_output_sizes->push(0);

        // Hash + output — TCPServer::writeData() polls for connections inline,
        // so no extra threads are needed for accept or send.
        hashThread = std::thread([this, flgs = flags]() {
			pthread_setname_np(pthread_self(), "hash generation thread");
			std::cout << "Hash Thread PID: " << syscall(SYS_gettid) << std::endl;
			std::cout << "\n\n\n";

			std::vector<uint8_t> hash = hashBuffer(std::vector<uint8_t>(1000));
			post_hash_output_data->resize(FRAME_KEEP * hash.size());
			size_t counter = 0;
			while (running) {
				if (frameQueue.empty()) continue;

				lastHashes = processFrameQueue();
				newHash = true;
				
				if (!((counter++)%CALIBRATE_N_FRAMES))  calibrateChunkSize(); // re-calibrate chunk size every N frames
				
				
				if (!lastHashes.empty()) {
					for (const auto& lastHash : lastHashes) {
						if (lastHash.empty()) continue;

						if (flags.verbose) {
							post_hash_output_data->insert(post_hash_output_data->end(),
														  lastHash.data(),
														  lastHash.data() + lastHash.size());
							showEntropy(post_hash_output_data->data(),
										post_hash_output_data->size());
							std::cout << " after Hash" << std::endl;
							post_hash_output_data->erase(post_hash_output_data->begin(),
														 post_hash_output_data->begin()
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
//				std::vector<uint8_t> current_hash = lastHash;
				newHash = false;
				if (!lastHashes.empty()) {
					for (const auto& lastHash : lastHashes) {
						if (lastHash.empty()) continue;
							output.writeData(lastHash.data(), lastHash.size());
					}
					
					while (!newHash);
//				if (flags.pseudo){
//					while (!newHash){
//						output.writeData(current_hash.data(), current_hash.size());
//						current_hash = hashBuffer(current_hash);
//					}
//				} else {
					
				}
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
