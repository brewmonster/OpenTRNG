#include <iomanip>
#include <iostream>
#include <fstream>
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
#include <chrono>
#include <ctime>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <filesystem>
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

	std::vector<uint8_t>* entropy_pool = new std::vector<uint8_t>;
	size_t ENTROPY_POOL_SIZE = 100;
	
    OutputPort output;
	std::ofstream verboseLog;
	std::ofstream dataLog;
	std::ofstream calibrationLog;
	std::ofstream performanceLog;

    std::vector<std::thread> cameraThreads;

	OutputFlags flags;
	size_t hashNum = 0;

    std::atomic<bool> running{false};

	std::atomic_bool newHash;
	std::vector<uint8_t> lastHash;
	std::mutex hashMutex;

	size_t HASH_LENGTH = 0;
	float  SAFETY_MARGIN       = 2.0f;

	size_t RUN_TO_N_HASHES = UINT64_MAX;

	OutputSignal outputSignal;

    void writeOutput(const uint8_t* data, size_t length) {
        if (dataLog.is_open()) {
            if (flags.binary_output)
                dataLog.write(reinterpret_cast<const char*>(data), length);
            else {
                dataLog << std::hex << std::setfill('0');
                for (size_t i = 0; i < length; ++i)
                    dataLog << std::setw(2) << static_cast<int>(data[i]);
                dataLog << '\n';
            }
        }
        output.writeData(data, length);
    }

    std::vector<uint8_t> hashBuffer(const std::vector<uint8_t>& frame) {
        HashDigester digester(EVP_sha256());
        digester.update(frame.data(), frame.size());
        return digester.finalize();
    }
	
	bool test = false;


public:

    TRNGApp() {}

    ~TRNGApp() {
        running = false;
        for (auto& t : cameraThreads) if (t.joinable()) t.join();
        if (cm) cm->stop();
        if (verboseLog.is_open()) verboseLog.close();
        if (dataLog.is_open()) dataLog.close();
        if (calibrationLog.is_open()) calibrationLog.close();
        if (performanceLog.is_open()) performanceLog.close();
    }
	
	void showEntropy(Estimator::Result result, size_t length){
		std::cout << "\033[2K" << std::dec << std::setprecision(3) << length 
			<< " entries with: " << "\tp_max: " << std::scientific << result.p_max
			<< ", H_min: " << std::defaultfloat << result.h_min;
		if (verboseLog.is_open())
			verboseLog << std::dec << length << " entries with: \tp_max: " << result.p_max
			           << ", H_min: " << result.h_min;
	}

	void displayHash(std::vector<uint8_t>& hash, Estimator::Result r_raw, Estimator::Result r_pool,
			size_t raw_size, size_t pool_size){
		// Output [Camera source] [hashnumber]: hash
		std::cout << "\033[3A\033[2K" << std::dec << "[Source: " << outputSignal.sourceID << "]"
		<< " [" << std::setfill('0')  << std::setw(5) << hashNum << "] : " << std::hex; 
		if (verboseLog.is_open()) verboseLog << std::dec << "[Source: " << outputSignal.sourceID << "]" << " [" << std::setw(5) << hashNum << "]: " << std::hex;
		
		for (const auto& byte : hash) {
			std::cout << std::setw(2) << static_cast<int>(byte);
			if (verboseLog.is_open()) verboseLog << std::setfill('0') << std::setw(2) << static_cast<int>(byte);
		}
		
		std::cout << std::endl << "\033[2K" << std::string(25, '-') << "\n";
		if (verboseLog.is_open()) verboseLog << "\n"; 

		// show raw entropy (only changes per frame, not per hash)
		std::cout << "[\033[2K";
		showEntropy(r_raw, raw_size);
		std::cout << " before Hash" << std::endl;
		if (verboseLog.is_open()) verboseLog << " before Hash\n";
		
		// show hashed entropy (of entire pool with new hash popped in)
		std::cout << "[\033[2K";
		std::lock_guard<std::mutex> lock(hashMutex);
		showEntropy(r_pool, pool_size);
		std::cout << " after Hash" << std::endl;
		if (verboseLog.is_open()) verboseLog << " after Hash\n";
	}
	
	// Receiver function for every input source. rates are not controlled i.e 60fps will be read 2x as many than 30fps, 
	// so queue is dependent on who writes first.
    void processFrameQueue(std::chrono::_V2::system_clock::time_point& log_t_last) {
		auto& source 	= sources[outputSignal.sourceID]; // query who sent the last frame
		auto  frame  	= source->getNextFrame(); 		  // take newest frame delivered from a source
		auto  r_raw 	= source->getLastResult();
		size_t chunkSize = r_raw.h_min * frame->size() / (SAFETY_MARGIN * HASH_LENGTH);	

		if (frame == nullptr || frame->empty()) return;
		
		Estimator::Result r_pool; // declaring so can lock with asigment

		// Putting a potential 5.2MB frame is a bit overkill for one 256 bit hash, so 
		// resegment the data appropriate to the chunk size.
        if (chunkSize < 1) chunkSize = frame->size()-1; 			// ensure data is not empty or 0s

		size_t len = frame->size() - chunkSize; // ensuring not reading past the last chunk.
		for (size_t i = 0; i < len; i+= chunkSize){
			std::vector<uint8_t> data(frame->data() + i, frame->data() + i + chunkSize); // new vectors of size chunkSize
			auto hash = hashBuffer(data);

			if (hash.empty()) continue;
			hashNum++;
			{
				std::lock_guard<std::mutex> lock(hashMutex);
				// Maintaining the pool's size and shifting in new hashes.
				entropy_pool->insert(entropy_pool->end(), hash.data(), hash.data() + hash.size());
				if (entropy_pool->size() >= ENTROPY_POOL_SIZE * hash.size()){
					entropy_pool->erase((entropy_pool->begin()), (entropy_pool->begin() + hash.size()));
				}
				// estimate current frame pool 
				if (flags.verbose || performanceLog.is_open()){
					r_pool = Estimator::Estimate_Markov(entropy_pool->data(), entropy_pool->size());
				}
			}
			// Outputs the current hash + entropy estimations
			if (flags.verbose) {
				displayHash(hash, r_raw, r_pool, frame->size(), entropy_pool->size());
			}

			if (performanceLog.is_open()){
				auto t_now = std::chrono::high_resolution_clock::now();
				auto t_dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - log_t_last);
				log_t_last = t_now;
				performanceLog << std::right 
				<< std::setw(11) << r_raw.p_max 	<< "|" << std::setw(11) << r_raw.h_min 	<< "|" << std::setw(11) << frame->size() << "|"
				<< std::setw(11) << r_pool.p_max  	<< "|" << std::setw(11) << r_pool.h_min << "|" << std::setw(11) << t_dt << "|"
				<< "\n" << std::flush;
				
			}

			lastHash = hash;
			newHash = true;
			newHash.notify_one();
			if (hashNum >= RUN_TO_N_HASHES) {
				std::cout << "Completed " << hashNum << " Hashes, exitting.." << std::endl;
				frame.reset();
				running = false; 
				return;
			} 
		}
		
		

		if (!flags.verbose){
			std::cout << "\033[1A\033[2K" << std::dec << "[Source: " << outputSignal.sourceID << "]"
						  << " [" << std::setfill('0')  << std::setw(5) << hashNum << "]" << std::endl; 
		}

		frame.reset(); // clear frame since on the heap
		outputSignal.sourceID = -1; 
    }

	void writeWorkingLoop() {
		pthread_setname_np(pthread_self(), "Write thread");
		std::cout << "Write Thread PID: " << "[" << syscall(SYS_gettid) << "]" << std::endl;

		static constexpr size_t BUF_SIZE = 4096; // target buffer size
		std::vector<uint8_t> buf(BUF_SIZE);
		std::vector<uint8_t> zeros(BUF_SIZE, 0);

		EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();

		auto reseed = [&]() {
			std::vector<uint8_t> key(32), nonce(16, 0);
			{
				std::lock_guard<std::mutex> lock(hashMutex);
				auto k = hashBuffer(*entropy_pool);         // 32-byte SHA-256 seed
				std::copy(k.begin(), k.end(), key.begin());
			}
			// Re-initialise ChaCha20 with new key, counter reset to 0
			EVP_EncryptInit_ex(ctx, EVP_chacha20(), nullptr, key.data(), nonce.data());
		};

		while (running) {
			newHash.wait(false);
			newHash = false;
			reseed();

			if (flags.pseudo) {
				int len = 0;
				while (!newHash) {
					// Encrypting zeros with ChaCha20 gives the 
					EVP_EncryptUpdate(ctx, buf.data(), &len, zeros.data(), BUF_SIZE);
					writeOutput(buf.data(), len);
				}
				continue;
			}
			
			// Raw Hash: dangerous to use directly. 
			std::lock_guard<std::mutex> lock(hashMutex);
			writeOutput(entropy_pool->data(), entropy_pool->size());
		}
		EVP_CIPHER_CTX_free(ctx);
		return;
	}

	void hashWorkingLoop(){
		auto now        = std::chrono::steady_clock::now();
		auto wait_time  = now + std::chrono::milliseconds(1000);

		while (std::chrono::steady_clock::now() < wait_time); // wait 3s so the other thread gets output out

		pthread_setname_np(pthread_self(), "Hash generation thread");
		std::cout << "Hash Thread PID:  [" << syscall(SYS_gettid) << "]" << std::endl;

		std::cout << "\n";
		
		// Enter blank sample data to test hash size given different algorithms output different sizes.
		std::vector<uint8_t> hash = hashBuffer(std::vector<uint8_t>(1000)); // little trick since const T& allows literals parsed.

		HASH_LENGTH = hash.size();
		entropy_pool->resize(HASH_LENGTH);

		auto log_t_last = std::chrono::high_resolution_clock::now();
		while (running) {

			// temporary scope to isntantly delete lock after
			{
				std::unique_lock<std::mutex> lock(outputSignal.mtx);
				outputSignal.cv.wait(lock, [&]{ return outputSignal.sourceID != -1; }); // wait until notified, if wakeup before, still locked if sourceID == -1.
			}
			
			processFrameQueue(log_t_last);
		}
	}

    int run(int argc, char* argv[]) {

        // ------------------------ CLI PARSING ------------------------
        OutputFlags _flags;
        for (int i = 1; i < argc; i++) {
            std::string arg(argv[i]);
            if      (arg == "-s"  || arg == "--serial")         _flags.forceSerial        = true;
            else if (arg == "-n"  || arg == "--no-input")        _flags.noPrompt          = true;
            else if (arg == "-v"  || arg == "--verbose")         _flags.verbose           = true;
            else if (arg == "-h"  || arg == "--hashes")          _flags.pseudo            = false; 
            else if (arg == "-l"  || arg == "--logging")         _flags.logging           = true;
            else if (arg == "-lb" || arg == "--logging-binary") { _flags.logging 		  = true; _flags.binary_output = true; }
            else if (arg == "-lv" || arg == "--log-verbose")    { _flags.verbose 		  = true; _flags.log_verbose = true;   }
            else if (arg == "-lc" || arg == "--log-calibration")   _flags.log_calibration = true;
            else if (arg == "-lp" || arg == "--log-performance") _flags.log_performance   = true;
            else if (arg == "-f"  || arg == "--force-calibrate") _flags.force_calibrate   = true;
            else if (arg == "-ch"  || arg == "--collect-hashes") try {RUN_TO_N_HASHES = std::stoi(argv[++i]);} 
			catch (const std::exception& e) {
				std::cerr << "Parsing error: unknown hash length request: " << argv[i] << e.what() << std::endl;
				return 1;
			} 
            else std::cerr << "Unknown argument: " << arg << std::endl;
        }
		
		flags = _flags;
        if (!output.init(flags)) return -1;
		
		if (!flags.pseudo){
			std::cout << "[WARNING]: Outputting Raw hashes, this exposes the seeds, and is far slower." << std::endl;
		}

		if (!flags.verbose){
			std::cout << "[WARNING]: Verbose mode considerably slows performance due to estimation algoirthm" << std::endl;
		}

		// ------------------------ LOG FILE SETUP ------------------------
		
		auto logTimestamp = [&]() {
			auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			char ts[20];
			std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&now));
			return std::string(ts);
		};

		std::filesystem::create_directories("./logs");  // creates all missing dirs

		if (flags.logging) {
			
			auto mode = std::ios::out | std::ios::app;
			std::string suffix = ".log";
			
			if (flags.binary_output){
				mode = std::ios::binary;
				suffix = ".bits";
			} 

			std::string logPath = "./logs/entropy_" + logTimestamp() + suffix;
			dataLog.open(logPath, mode); // start logging file
			if (!dataLog.is_open())
				std::cerr << "Warning: could not open data log file: " << logPath << std::endl;
			else if (!flags.binary_output) {
				time_t now = std::time(nullptr);
				dataLog << "LOGGING BEGINNING AT: " << std::ctime(&now) << "\n";
			}
		}
		
		auto mode = std::ios::out | std::ios::app;
		if (flags.log_verbose) {
			std::string logPath = "./logs/verbose_" + logTimestamp() + ".log";
			verboseLog.open(logPath, mode);
			if (!verboseLog.is_open())
				std::cerr << "Warning: could not open verbose log file: " << logPath << std::endl;
			else
				std::cout << "Verbose log: " << logPath << std::endl;
		}

		if (flags.log_calibration) {
			std::string logPath = "./logs/calibration_" + logTimestamp() + ".log";	
			calibrationLog.open(logPath, mode);
			if (!calibrationLog.is_open())
				std::cerr << "Warning: could not open calibration log file: " << logPath << std::endl;
			else
				std::cout << "Calibration log: " << logPath << std::endl;
		}

		if (flags.log_performance){
			std::string logPath = "./logs/performance_" + logTimestamp() + ".log";
			performanceLog.open(logPath, mode);
			if (!performanceLog.is_open())
				std::cerr << "Warning: could not open performanceLog log file: " << logPath << std::endl;
			else{
				
				std::cout << "Performance log: " << logPath << std::endl;
				
				performanceLog << std::left 
					<< std::setw(15) << "" << std::setw(20) << "RAW" << "|" 
					<< std::setw(9) << "" << std::setw(14) <<"HASHED" << "|" 
					<< std::setw(11) << "" << "|" << "\n";

				performanceLog 
					<< std::setw(11) << " p_max" << std::setw(12) << "| H_min" << std::setw(12) << "| pixels" << "|"
					<< std::setw(11) << " p_max" << std::setw(12) << "| H_min" << std::setw(12) << "| t_dt"
					<< "|\n" << std::flush;
			}
			
		}


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
            sources.emplace_back(std::make_unique<EntropySource>(camera, running, &outputSignal, i, flags));
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
			sources[i]->calibrate(600000, flags.force_calibrate,
			                      calibrationLog.is_open() ? &calibrationLog : nullptr); // call calibrate after worker thread started
		}
		

        // Hash + output TCPServer::writeData() polls for connections inline,
        // so no extra threads are needed for accept or send.
        
		// this thread = write thread
		std::cout << "TRNG started." << std::endl;
        return 0;
    }

};


int main(int argc, char* argv[]) {
    auto app = std::make_unique<TRNGApp>();
	if (app->run(argc, argv)) {
			std::cerr << "Failed to initialize." << std::endl;
			return -1;
		}
	std::thread hashThread(&TRNGApp::hashWorkingLoop, app.get());
	app->writeWorkingLoop(); // this thread


	// upon exit, delete
	if (hashThread.joinable()) hashThread.join();
	app.reset();
	return 0;
}
