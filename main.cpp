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

static std::unique_ptr<CameraManager> cm;
static std::vector<std::unique_ptr<Request>> requests;

Request* currentRequest = nullptr;
Request* oldRequest = nullptr;

Stream* stream = nullptr;

int init(){

    // ------------------------ SETTING UP CAMERA MANAGER ------------------------

    cm = std::make_unique<CameraManager>();
    cm->start();
    for (auto const &camera : cm->cameras()) std::cout << camera->id() << std::endl;

    auto cameras = cm->cameras();
    if (cameras.empty()) {
        std::cout << "No cameras were identified on the system."
                << std::endl;
        cm->stop();
        return EXIT_FAILURE;
    }

    for (auto cam : cameras){
        std::shared_ptr<Camera> camera = cm->get(cam->id());

    }

    std::string cameraId = cameras[0]->id();

    camera = cm->get(cameraId);
    /*
    * Note that `camera` may not compare equal to `cameras[0]`.
    * In fact, it might simply be a `nullptr`, as the particular
    * device might have disappeared (and reappeared) in the meantime.
    */
    camera->acquire(); 

    // ------------------------ SETTING UP CAMERA CONFIGURATION ------------------------

    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration( { StreamRole::Raw } );
    StreamConfiguration &streamConfig = config->at(0); 
    streamConfig.pixelFormat = formats::SGBRG10_CSI2P;

    camera->configure(config.get());

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

    
    camera->requestCompleted.connect(requestComplete);
    
    camera->start();

    oldRequest = requests[0].get(); // set the first request as the current request to start with
    currentRequest = requests[1].get(); // set the second request as the old request to start with
    
    camera->queueRequest(oldRequest);
    

    // ------------------------ STARTING THE DISPLAY WINDOW ------------------------
    
    // todo

    std::cout << "Camera manager started" << std::endl;
    return 0;
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