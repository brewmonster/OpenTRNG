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
#include <sys/mman.h> 
#include <sys/syscall.h>
#include <unistd.h>
#include <pthread.h>

#include <bit> 
#include <any>

#include <libcamera/libcamera.h>


#include "Estimator.hpp"
#include "EntropyCalibrator.hpp"
#include "OutputPort.hpp"


using namespace libcamera;
using namespace std::chrono_literals;

enum Mode {
    NONE=-1,
    RAW=0,
    GREYSCALE=1,
    YUV=2,
    RGB=3,
};

struct OutputSignal {
    std::mutex mtx;
    std::condition_variable cv;
    int sourceID = -1; // -1 = none
};


class EntropySource {

    private:
        
        size_t ID;
        Mode format;

        std::shared_ptr<Camera>  camera; 
        std::vector<std::unique_ptr<Request>> requests;
        
        const std::atomic<bool>& running;
        
        size_t queueLimit;
        Request* currentRequest = nullptr;
        Request* oldRequest = nullptr;
		size_t shift_amount = 0; // for rotating the byte values to reduce bias from specific colour wells in RAW/RGB modes
		
        Stream* stream = nullptr;
        
        // Used for tracking entropy over time.
        std::vector<std::vector<uint8_t>> longSampleBuffer; // reduced data, longer time scale, for validation
        Estimator::Result lastResult;


        std::map<const int, uint8_t*> bufferMappedData; // Map buffer pointers to mapped data
        
        std::queue<std::unique_ptr<std::vector<uint8_t>>> frameQueue;
        OutputSignal* outputSignal;
        const OutputFlags flags;
        bool collectingEntropy = false;

        uint8_t* processBuffer(FrameBuffer* bufferPtr);
        std::vector<uint8_t> compareBuffers(uint8_t* currentData, uint8_t* oldData, size_t length);
        
        std::queue<Request*> pendingRequests;
        std::mutex pendingMutex;
        std::condition_variable pendingCV;
        
        void processRequest(Request *request);
        
        void calibrateHmin();


    public:

        size_t CALIBRATE_N_FRAMES  = 10000;
        size_t FRAME_KEEP          = 8;

    
        EntropySource(std::shared_ptr<Camera> _camera, 
        const std::atomic<bool>& _running, 
        OutputSignal* _outputSignal, size_t& _id, OutputFlags _flags);
        
        EntropySource();

        ~EntropySource();
		
		void calibrate(uint32_t timeout_ms = 30000, bool force_calibrate = false, std::ofstream* log = nullptr);

        void requestComplete(Request *request);
        
        void workingLoop();

        size_t size();
        size_t getChunkSize(size_t hashLength);
        Estimator::Result getLastResult();
        std::unique_ptr<std::vector<uint8_t>> getNextFrame(); 
        
        int init(size_t _queue_limit);

};

#endif // 
