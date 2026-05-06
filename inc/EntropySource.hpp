#ifndef ENTROPY_SOURCE_HPP
#define ENTROPY_SOURCE_HPP

#include <iomanip>
#include <iostream>
#include <bitset>
#include <memory>
#include <thread>
#include <vector>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <sys/mman.h> // for accessing dma data
#include <sys/syscall.h>
#include <unistd.h>
#include <pthread.h>

#include <bit> 

#include <libcamera/libcamera.h>


#include "MarkovEstimator.hpp"
#include "EntropyCalibrator.hpp"


using namespace libcamera;
using namespace std::chrono_literals;

enum Mode {
    NONE=-1,
    RAW=0,
    GREYSCALE=1,
    RGB=2,
};

class EntropySource {

    private:
        
        
        Mode format;

        std::shared_ptr<Camera>  camera; 
        std::vector<std::unique_ptr<Request>> requests;
        
        const std::atomic<bool>* running = nullptr;
        
        size_t queueLimit;
        Request* currentRequest = nullptr;
        Request* oldRequest = nullptr;
		
		CalibrationResult calibrationResult{};
		


        Stream* stream = nullptr;
        


        std::map<const int, uint8_t*> bufferMappedData; // Map buffer pointers to mapped data
        
        std::queue<std::vector<uint8_t>*>* queue = nullptr;
		std::mutex& frm_q_mtx;
        uint8_t* processBuffer(FrameBuffer* bufferPtr);
        std::vector<uint8_t> compareBuffers(uint8_t* currentData, uint8_t* oldData, size_t length);
        
        std::queue<Request*> pendingRequests;
        std::mutex pendingMutex;
        std::condition_variable pendingCV;
        
        void processRequest(Request *request);

    public:
    
        EntropySource(std::shared_ptr<Camera> _camera, 
			std::queue<std::vector<uint8_t>*>* _queue, 
			const std::atomic<bool>* _running, 
			std::mutex& _frame_queue_mutex);
        
        EntropySource();

        ~EntropySource();
		
		void calibrate(uint32_t timeout_ms = 30000, bool force_calibrate = false);
        
        void requestComplete(Request *request);
        
        void workingLoop();

        size_t size();
        

        int init(size_t _queue_limit);

};

#endif
