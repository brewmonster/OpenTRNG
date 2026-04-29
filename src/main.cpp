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

    OutputPort output;

    std::vector<std::thread> cameraThreads;
    std::thread hashThread;
	std::thread writeThread;

    std::atomic<bool> running{false};

	std::vector<uint8_t> lastHash;
	bool newHash;
    std::vector<uint8_t> hashBuffer(std::vector<uint8_t> hash) {
        HashDigester digester(EVP_sha256());
        digester.update(hash.data(), hash.size());
        return digester.finalize();
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

    std::vector<uint8_t> processFrameQueue() {
		
		std::unique_lock<std::mutex> lock(frm_q_mtx);
		if (frameQueue.empty()) {
			return std::vector<uint8_t>();
		}
        std::vector<uint8_t>* frame = frameQueue.front();
		frameQueue.pop();
		lock.unlock();
		
        if (frame->size() < frame->capacity() || !frame->empty()) {
			std::vector<uint8_t> hash = hashBuffer(*frame);
			return hash;
        }
		delete frame;
		return {};
    }
	
	void writePRNG(){
		
	}

    int start(int argc, char* argv[]) {
		std::cout  << "RAND_MAX: " << RAND_MAX << std::endl;
        // ------------------------ CLI PARSING ------------------------
        OutputFlags flags;
        for (int i = 1; i < argc; ++i) {
            std::string arg(argv[i]);
            if      (arg == "-s" || arg == "--serial")   flags.forceSerial = true;
            else if (arg == "-n" || arg == "--no-input") flags.noPrompt    = true;
			else if (arg == "-q" || arg == "--quiet") flags.quiet    = true;
			else if (arg == "-p" || arg == "--pseudo") flags.pseudo    = true;


            else std::cerr << "Unknown argument: " << arg << std::endl;
        }

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
                        for (size_t idx : group) sources[idx]->workingLoop();
                });
            }
        }

        // Hash + output — TCPServer::writeData() polls for connections inline,
        // so no extra threads are needed for accept or send.
		bool quiet = flags.quiet;
		bool pseudo = flags.pseudo;
        hashThread = std::thread([this, quiet, pseudo]() {
			pthread_setname_np(pthread_self(), "hash generation thread");
			std::cout << "Hash Thread PID: " << syscall(SYS_gettid) << std::endl;

            while (running) {
                if (frameQueue.empty()) continue;
                lastHash = processFrameQueue();
				newHash = true;
                if (!lastHash.empty()) {
					if (!quiet){
						std::cout << "Hash: " << std::hex << std::setfill('0');
						for (const auto& byte : lastHash)
							std::cout << std::setw(2) << static_cast<int>(byte);
						std::cout << std::dec << std::endl;						
					}
					if (!pseudo)
						output.writeData(lastHash.data(), lastHash.size()); 
                }
            }
        });
		
		if (flags.pseudo)
		writeThread = std::thread([this]() {
			pthread_setname_np(pthread_self(), "Network write thread");
			std::cout << "Write Thread PID: " << syscall(SYS_gettid) << std::endl;
			while(running){
				std::vector<uint8_t> current_hash = lastHash;
				newHash = false;
				while (!newHash){
					output.writeData(current_hash.data(), current_hash.size());
					current_hash = hashBuffer(current_hash);
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
