#include <iomanip>
#include <iostream>
#include <bitset>
#include <memory>
#include <thread>
#include <vector>

#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <sys/mman.h> // for accessing dma data

#include <libcamera/libcamera.h>
#include "EntropySource.hpp"

using namespace libcamera;
using namespace std::chrono_literals;

class EntropySource {

    private:
        
        std::shared_ptr<Camera>  camera; 
        std::vector<std::unique_ptr<Request>> requests;

        Request* currentRequest = nullptr;
        Request* oldRequest = nullptr;

        Stream* stream = nullptr;

        std::map<FrameBuffer*, uint8_t*> bufferMappedData; // Map buffer pointers to mapped data


        uint8_t* processBuffer(FrameBuffer* bufferPtr);
        void compareBuffers(uint8_t* currentData, uint8_t* oldData, size_t length);
        void requestComplete(Request *request);

    public:

        EntropySource(std::shared_ptr<Camera> _camera);

        ~EntropySource();

        int init();

};