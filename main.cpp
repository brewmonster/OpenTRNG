#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <sys/mman.h> // for accessing dma data

#include <libcamera/libcamera.h>

using namespace libcamera;
using namespace std::chrono_literals;

static std::shared_ptr<Camera> camera;

Request* currentRequest = nullptr;
Request* oldRequest = nullptr;

Stream* stream = nullptr;

static std::map<FrameBuffer*, uint8_t*> bufferMappedData; // Map buffer pointers to mapped data



uint8_t* processBuffer(FrameBuffer* bufferPtr) { 
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

void compareBuffers(uint8_t* currentData, uint8_t* oldData, size_t length) {
    // Example comparison: Calculate the sum of absolute differences
    uint64_t sumAbsDiff = 0;
    for (size_t i = 0; i < length; ++i) {
        sumAbsDiff += currentData[i] ^ oldData[i]; // get difference in each byte and add to sum
    }
    std::cout << "Sum of Absolute Differences: " << sumAbsDiff << std::endl;
}

void requestComplete(Request *request) {
    if (request->status() == Request::RequestCancelled) return;

    // if oldRequest is not nullptr, then use oldRequest buffer as the new request aka flipflopping between buffers
    if (oldRequest) {
        oldRequest->reuse(Request::ReuseBuffers);
        camera->queueRequest(oldRequest);
    } 
    
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
}


int init(){

    // ------------------------ SETTING UP CAMERA MANAGER ------------------------

    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();
    for (auto const &camera : cm->cameras()) std::cout << camera->id() << std::endl;

    auto cameras = cm->cameras();
    if (cameras.empty()) {
        std::cout << "No cameras were identified on the system."
                << std::endl;
        cm->stop();
        return EXIT_FAILURE;
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

    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration( { StreamRole::Viewfinder } );
    StreamConfiguration &streamConfig = config->at(0); 
    streamConfig.pixelFormat = formats::NV12;

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
    std::vector<std::unique_ptr<Request>> requests;

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

    currentRequest = requests[0].get(); // set the first request as the current request to start with

    camera->requestCompleted.connect(requestComplete);

    

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
    camera->start();
    camera->queueRequest(currentRequest);

    return 0;
}