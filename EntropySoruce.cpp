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


uint8_t* EntropySource::processBuffer(FrameBuffer* bufferPtr) { 
    struct dma_buf_sync sync = {0};

    int fd = bufferPtr->planes()[0].fd.get();

    sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ; 
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);

    // Now it is safe to compare bits in mappedData


    uint8_t* mappedData = bufferMappedData[bufferPtr]; // Retrieve the mapped data pointer for this buffer

    // --- STEP 3: END SYNC ---
    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
    return mappedData;
}

void EntropySource::compareBuffers(uint8_t* currentData, uint8_t* oldData, size_t length) {
    // Example comparison: Calculate the sum of absolute differences
    std::stringstream stream;
    for (size_t i = 0; i < length; ++i) {
        uint8_t diff = currentData[i] - oldData[i]; // get difference in each byte and add to sum
        
        stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(diff) << " ";
    }
    std::cout << stream.str() << std::endl;
}

void EntropySource::requestComplete(Request *request) {
    // std::cout << "Request completed with status: " << request->status() << std::endl;

    if (request->status() == Request::RequestCancelled) return;

    // if oldRequest is not nullptr, then use oldRequest buffer as the new request aka flipflopping between buffers
    
    oldRequest = currentRequest; 
    currentRequest = request;    // The frame that just arrived

    if (oldRequest && currentRequest) {
       
        // grab the buffer pointer of the requests
        FrameBuffer* currentBufferPtr = currentRequest->findBuffer(stream);
        FrameBuffer* oldBufferPtr = oldRequest->findBuffer(stream);

        // before 
        uint8_t* currentMappedData = processBuffer(currentBufferPtr);
        uint8_t* oldMappedData = processBuffer(oldBufferPtr);

        compareBuffers(currentMappedData, oldMappedData, currentBufferPtr->planes()[0].length);
    }

    if (oldRequest) {
        oldRequest->reuse(Request::ReuseBuffers);
        camera->queueRequest(oldRequest);
    } else {
        std::cout << "No old request to reuse yet" << std::endl;
        currentRequest->reuse(Request::ReuseBuffers);
        camera->queueRequest(currentRequest);
    }
}


int EntropySource::init(){

    
    // ------------------------ SETTING UP CAMERA CONFIGURATION ------------------------
    
    camera->acquire(); 

    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration( { StreamRole::Raw } );

    ControlList properties = camera->properties();

    StreamConfiguration &streamConfig = config->at(0); 
    streamConfig.pixelFormat = formats::SGBRG10_CSI2P;

    camera->configure(config->get());

    // ------------------------ SETTING UP FRAME BUFFERS ------------------------

    // allocating minimum driver buffers

    FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);  

    for (StreamConfiguration &cfg : *config) {
        int ret = allocator->allocate(cfg.stream());
        if (ret < 0) {
            std::cerr << "Can't allocate buffers" << std::endl;
            return -ENOMEM;
        }

        size_t allocated = allocator->buffers(cfg.stream()).size();
        std::cout << "Allocated " << allocated << " buffers for stream" << std::endl;
    }


    // ------------------------ MMAP BUFFERS IN DMA MEMORY TO VIRTUAL MEMORY SPACE ------------------------

    for (const auto &cfg : *config) {
        const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(cfg.stream());
        for (const auto &buffer : buffers) {
            for (const auto &plane : buffer->planes()) {
                int fd = plane.fd.get();
                size_t size = plane.length;
                uint8_t* mappedData = static_cast<uint8_t*>(mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
                if (mappedData == MAP_FAILED) {
                    std::cerr << "Failed to mmap buffer" << std::endl;
                    return -1;
                }
                // Store mappedData for later use in requestComplete callback
                bufferMappedData[buffer.get()] = mappedData; // setting the buffer pointer as key and mapped data pointer as value in the map
            }
        }
    }
    

    // ------------------------ SETTING UP REQUEST STREAM ------------------------


    stream = streamConfig.stream();
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);

    for (unsigned int i = 0; i < buffers.size(); i++) { // looping through all the buffers and adding them to the request
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request)
        {
            std::cerr << "Can't create request" << std::endl;
            return -ENOMEM;
        }

        const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
        int ret = request->addBuffer(stream, buffer.get());
        if (ret < 0)
        {
            std::cerr << "Can't set buffer for request"
                << std::endl;
            return ret;
        }

        requests.push_back(std::move(request));
    }

    
    camera->requestCompleted.connect(EntropySource::requestComplete);
    
    camera->start();

    oldRequest = requests[0].get(); // set the first request as the current request to start with
    currentRequest = requests[1].get(); // set the second request as the old request to start with
    
    camera->queueRequest(oldRequest);
    

    // ------------------------ STARTING THE DISPLAY WINDOW ------------------------
    
    // todo

    std::cout << "Camera manager started" << std::endl;
    return 0;
}

EntropySource::EntropySource(std::shared_ptr<Camera> _camera){
    this.camera = _camera;
}

EntropySource::~EntropySource(){
    std::cout << "destructor called" <<std::endl;
}


int main()
{
    if (init()) {
        std::cerr << "Failed to initialize camera manager" << std::endl;
        return -1;
    }
    while (1);

    return 0;
}