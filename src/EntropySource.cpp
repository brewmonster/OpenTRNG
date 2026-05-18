#include "EntropySource.hpp"

using namespace libcamera;
using namespace std::chrono_literals;


// ------------------------ GETTERS / SETTERS ------------------------

// Theortical max size for each method
size_t EntropySource::size(){
    Span<const FrameBuffer::Plane> planes = currentRequest->findBuffer(stream)->planes();
    if (format == Mode::RAW || format == Mode::GREYSCALE) return planes[0].length / 5; // optimised to use only 1/5 bytes 
    if (format == Mode::RGB) return planes[0].length * 4; // 4 bytes for every 1 pixel (1 pixel is actually 4 raw pixels)
    return 0; 
}

Estimator::Result EntropySource::getLastResult(){
    return lastResult;
}

void EntropySource::calibrateHmin(){
    if (frameQueue.size() <= queueLimit - frameQueue.size()){
        std::cout << "No frames to estimate from" << std::endl;
        return;
    }
    std::vector<uint8_t> data;
        for (size_t i = 0; i < FRAME_KEEP && !frameQueue.empty(); i++){
            std::unique_ptr<std::vector<uint8_t>> frame = std::move(frameQueue.front());
            frameQueue.pop();
            data.insert(data.end(), frame->data(), frame->data() + frame->size()); // collapses the vector into 1 long data frame. warning this may take a lot of data.
        }
	Estimator::Result r = Estimator::Estimate_Markov(data.data(), data.size());
    lastResult = r;
}

// Only time frame can be read is when popping out a previous frame.
std::unique_ptr<std::vector<uint8_t>> EntropySource::getNextFrame() {
    std::lock_guard<std::mutex> lock(outputSignal->mtx); 
    if (frameQueue.empty()) return nullptr;

    std::unique_ptr<std::vector<uint8_t>> frame = std::move(frameQueue.front()); // move the pointer owner.
    frameQueue.pop();
    return frame; 
}

// ------------------------ REQUEST HANDLING ------------------------

uint8_t* EntropySource::processBuffer(FrameBuffer* bufferPtr) { 
    struct dma_buf_sync sync = {0};

    int fd = bufferPtr->planes()[0].fd.get();

    sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ; 
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);


    uint8_t* mappedData = bufferMappedData[fd]; // Retrieve the mapped data pointer for this buffer

    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
    return mappedData;
}

std::vector<uint8_t> EntropySource::compareBuffers(uint8_t* currentData, uint8_t* oldData, size_t length) {
    // Example comparison: Calculate the sum of absolute differences
    std::vector<uint8_t> data;
	
	// Every 5th byte is 2 LSB its from the last 4 pixels, 
	// to reduce bias, rotate byte based on index to set MSBs to different colour wells
    // R10 also uses 5th byte for LSBs, so this method works for both RAW and GREYSCALE modes
    if (format == Mode::RAW || format == Mode::GREYSCALE) {
		data.reserve(length / 5);
		uint8_t last_diff = 0;
		uint8_t byte;
		for (size_t i = 4; i < length - UINT8_MAX; i+=5) { 
			uint8_t diff = currentData[i] - oldData[i+last_diff - last_diff%5]; 
			if (diff != last_diff) { // check if not 0 or 255
				byte = std::rotl(static_cast<uint8_t> (diff - last_diff), shift_amount);
				last_diff = diff;
				data.push_back(byte);
                shift_amount += byte; 
				shift_amount = (shift_amount + i) % 8; // rotate shift amount to rotate which colour well the bits are taken from, to reduce bias   
			}
		}
		return data;
    }


	// RGB is formatted like R:8 B:8 G:8
	// Loops through all RGB values, and takes the 2 LSBs, 
    // packs into 1/4 of size while maintaining entropy
    if (format == Mode::RGB || format == Mode::YUV) {
		data.reserve(length / 4);
		uint8_t bit_start = 0;
		uint8_t current_byte = 0;
		for (size_t i = 0; i < length; i++) { 
            bit_start += 2;
            bit_start %= 8;
            current_byte |= ((static_cast<uint8_t> (currentData[i] - oldData[i]) & 0b00000011) << bit_start); 
            if (bit_start == 0){ // if byte filled, push to vector and start new byte
                data.push_back(current_byte);
                current_byte = 0;   // start of new packing byte
            }
		}
		
		return data; 
	}
    
	return {};
}

// swap requests, process inbound data, send to main to be written
void EntropySource::processRequest(Request *request) {
    // std::cout << "Request completed with status: " << request->status() << std::endl;

    if (request->status() == Request::RequestCancelled) return;
    
	// Flip flop between 2 buffers
    oldRequest = currentRequest; // frame from before
    currentRequest = request;    // The frame that just arrived
	
	// On first run only currentRequest holds a valid request, so can't flipflip yet 
	// preloading the next request, since request processing will almost always take longer than the camera frame rate
    if (oldRequest) { 
        oldRequest->reuse(Request::ReuseBuffers);
        camera->queueRequest(oldRequest);
    } else {
        std::cout << "No old request to reuse yet" << std::endl;
        currentRequest->reuse(Request::ReuseBuffers);
        camera->queueRequest(currentRequest);
    }

    if (oldRequest && currentRequest) {
       
        // Grab the buffer pointer of the requests
        FrameBuffer* currentBufferPtr = currentRequest->findBuffer(stream);
        FrameBuffer* oldBufferPtr     = oldRequest->findBuffer(stream); 

        // Getting virtual memory pointer equiavlent to the dma fd
        uint8_t* currentMappedData = processBuffer(currentBufferPtr);
        uint8_t* oldMappedData     = processBuffer(oldBufferPtr);
        
        auto frame_diff = std::make_unique<std::vector<uint8_t>>(
            compareBuffers(currentMappedData, oldMappedData, 
                currentBufferPtr->planes()[0].length)
            ); 
		
        std::unique_lock<std::mutex> lock(outputSignal->mtx);
        if (collectingEntropy) lock.unlock();
        if (frameQueue.size() < queueLimit){ // ensure requests are bound by the data processing, as to not cause a memory leak
            frameQueue.push(std::move(frame_diff)); // pushing the mapped data pointer into the queue
        }
    }
}


void EntropySource::requestComplete(Request *request) { 
    if (request->status() == Request::RequestCancelled) return;
    {
        std::lock_guard<std::mutex> lock(pendingMutex);
        pendingRequests.push(request);
    }
    pendingCV.notify_one(); // wake the worker thread
}

void EntropySource::workingLoop() {
	pthread_setname_np(pthread_self(), "Camera thread");
	std::cout << "Camera Thread PID: " << syscall(SYS_gettid) << std::endl;
    std::unique_ptr<std::lock_guard<std::mutex>> calibrateLock;
    size_t counter = 0;
    while (running) {
        Request* request;
        {
            std::unique_lock<std::mutex> lock(pendingMutex); // when request completed, process it when not being accesed.
            pendingCV.wait(lock, [this]{ return !pendingRequests.empty() || !running; });
            if (!running) break;
            request = pendingRequests.front();
            pendingRequests.pop();
        }
        processRequest(request);
        
        if (CALIBRATE_N_FRAMES - (counter++) % CALIBRATE_N_FRAMES == FRAME_KEEP){ // recalibrating hmin every N frames
            calibrateLock = std::make_unique<std::lock_guard<std::mutex>>(outputSignal->mtx);
            collectingEntropy = true; // bypass the lock whereas main threads cannot.
            queueLimit += FRAME_KEEP;
            std::cout << "Re-Calibrating chunk size" << std::endl;
        }
        if (collectingEntropy) {
            if (frameQueue.size() >= queueLimit) {
                collectingEntropy = false;
                calibrateHmin();
                calibrateLock.reset();
                queueLimit -= FRAME_KEEP;
                std::cout << "Re-Calibrating complete." << std::endl;
            }
        } else {
            outputSignal->sourceID = ID;
            outputSignal->cv.notify_one();
        }
    }
}
int EntropySource::init(size_t _queue_limit) {
    

    // ------------------------ SETTING UP CAMERA CONFIGURATION ------------------------
    
    queueLimit = _queue_limit;
    
    camera->acquire();
    using RoleList = std::vector<std::pair<StreamRole, PixelFormat>>;
    RoleList roles = {  {StreamRole::Raw, 				formats::SGBRG10_CSI2P}, // Should be compatible on most cameras. Format facilitates highest performance  
                        {StreamRole::Viewfinder, 	    formats::R10},           // Greyscale
                        {StreamRole::Viewfinder, 	    formats::NV12},          // Planar YUV to separate Y from UV
                        {StreamRole::Viewfinder, 		formats::RGB888}         // RGBX8888 more compatible than RGB888 in terms of ISP requests.
                    };

    std::unique_ptr<CameraConfiguration> config;

                                                     
    // Checking if there are compatible formats
    for (int i = 0; const std::pair<StreamRole, PixelFormat>& role : roles){
        config = camera->generateConfiguration( {role.first} );
        if (config == nullptr) continue; // returns nullptr if can't configure
        
        StreamConfiguration& streamConfig = config->at(0); 
        const StreamFormats &formats = streamConfig.formats();
        
        if (flags.verbose) {
            std::cout << "Role: " << role.first << "\n";
            std::cout << "Supported pixel formats:\n";
        }
        for (const PixelFormat pxlFm : formats.pixelformats()){
            if (flags.verbose) std::cout << "  " << pxlFm.toString() << "\n";
            if (pxlFm == role.second){
                streamConfig.pixelFormat = role.second;
                format = static_cast<Mode>(i);   // setting mode for future
                goto formatExit;
            }
        }
        i++;
    }
    std::cerr << "No compatible formats found." << std::endl;
        return -1;
	
formatExit: 

    // // // // // // Uncomment to force specific mode:
    // config = camera->generateConfiguration({StreamRole::Viewfinder});
    StreamConfiguration& streamConfig = config->at(0); 
    // streamConfig.pixelFormat = formats::RGB888;
    // format = Mode::YUV;   // setting mode for future

	// StreamConfiguration& streamConfig = config->at(0); 

    camera->configure(config.get());
    

    // ------------------------ SETTING UP FRAME BUFFERS ------------------------

    // allocating buffers requested by the camera

    FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);
	
	if (streamConfig.bufferCount < 2){
		std::cerr << "Can't allocate enough buffers, Requires 2" << std::endl;
		return -ENOMEM;
	}
	
	
	// allocate buffers for comparing requests
	streamConfig.bufferCount = 2;
	int ret = allocator->allocate(streamConfig.stream());
	if (ret < 0) {
		std::cerr << "Can't allocate buffers" << std::endl;
		return -ENOMEM;
	}

	
	stream = streamConfig.stream();
	
	size_t allocated = allocator->buffers(stream).size();
	std::cout << "Allocated " << allocated << " buffers for stream" << std::endl;
	

    // ------------------------ MMAP BUFFERS IN DMA SPACE TO VIRTUAL MEMORY SPACE ------------------------

    
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);

	for (const auto &buffer : buffers) {	

		// Mode::RAW and Mode::RGB formats only have 1 plane, so can directly mmap the first plane
		if (format == Mode::RAW || format == Mode::RGB){ 
			int fd = buffer->planes()[0].fd.get();
			size_t size = buffer->planes()[0].length;
			uint8_t* mappedData = static_cast<uint8_t*>(mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
			if (mappedData == MAP_FAILED) {
				std::cerr << "Failed to mmap buffer" << std::endl;
				return -1;
			}
			// Store mappedData for later use in requestComplete callback
			bufferMappedData[fd] = mappedData; // setting the buffer pointer as key and mapped data pointer as value in the map
		} 
		
		// planar data needs multiple maps for each plane
		else{
			for (const auto &plane : buffer->planes()) {
				int fd = plane.fd.get();
				size_t size = plane.length;
				uint8_t* mappedData = static_cast<uint8_t*>(mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
				if (mappedData == MAP_FAILED) {
					std::cerr << "Failed to mmap buffer" << std::endl;
					return -1;
				}
				// Store mappedData for later use in requestComplete callback
				bufferMappedData[fd] = mappedData; // setting the buffer pointer as key and mapped data pointer as value in the map
			}
		}
	}
    

    // ------------------------ SETTING UP REQUEST STREAM ------------------------

    for (unsigned int i = 0; i < buffers.size(); i++) { 
        // looping through all the buffers and asigning each one a request stream
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request) {
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
    
    
    camera->requestCompleted.connect(this, &EntropySource::requestComplete);

	libcamera::ControlList startControls(libcamera::controls::controls);
	startControls.set(libcamera::controls::AeEnable,  false); // auto exposure
	startControls.set(libcamera::controls::AwbEnable, false); // auto white balance
	camera->start(&startControls);

	oldRequest      = requests[0].get(); 
	currentRequest  = requests[1].get();

	camera->queueRequest(oldRequest); // start flipflop by queueing just one.

    return 0;
}



void EntropySource::calibrate(uint32_t timeout_ms, bool force_calibrate, std::ofstream* log) {

    EntropyCalibrator cal(camera, stream, flags, log);

    auto frameCallback = [this](std::vector<uint8_t>& accum, size_t nFrames, const SweepList& sweepList) {
    size_t collected = 0;
    auto deadline    = std::chrono::steady_clock::now() + std::chrono::seconds(600); // 10 minutes

    while (collected < nFrames && std::chrono::steady_clock::now() < deadline) {
        if (collected < 2) {
            auto& controls = currentRequest->controls(); 
            for (auto& param : sweepList)
                param.apply(controls, param.current); // lambda pointers to captures get destroyed, so have to reparse .current
        }

        auto frame = getNextFrame();

        if (!frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        accum.insert(accum.end(), frame->begin(), frame->end());
        ++collected;
    }
};
	
    // lastResult = Estimator::Result({0.0, 0.5, 0});
    lastResult = cal.run(frameCallback, timeout_ms);
}

EntropySource::EntropySource(std::shared_ptr<Camera> _camera, 
	const std::atomic<bool>& _running, 
    OutputSignal* _outputSignal, size_t& _id, const OutputFlags _flags)
                        : camera(_camera), running(_running), 
                        outputSignal(_outputSignal), ID(_id), flags(_flags) {}

EntropySource::~EntropySource(){
    camera->stop();
    camera->release();
}

