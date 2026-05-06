#include "EntropySource.hpp"

using namespace libcamera;
using namespace std::chrono_literals;


// expected size of data 
size_t EntropySource::size(){
    Span<const FrameBuffer::Plane> planes = currentRequest->findBuffer(stream)->planes();
    
    
    // PixelFormatInfo::PixelFormatInfo &info = pixelFormatInfo::(format);
    
    if (format == Mode::RAW) return planes[0].length / 5; // optimised to use only 1/5 bytes 
    if (format == Mode::GREYSCALE) return planes[0].length + planes[1].length + planes[2].length; // multiple planes, so total size is the sum of all plane sizes
    if (format == Mode::RGB) return planes[0].length * 4; // 4 bytes for every 1 pixel (1 pixel is actually 4 raw pixels)
    return 0; 
}

uint8_t* EntropySource::processBuffer(FrameBuffer* bufferPtr) { 
    struct dma_buf_sync sync = {0};

    int fd = bufferPtr->planes()[0].fd.get();

    sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ; 
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);


    uint8_t* mappedData = bufferMappedData[fd]; // Retrieve the mapped data pointer for this buffer

    // --- STEP 3: END SYNC ---
    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
    return mappedData;
}

std::vector<uint8_t> EntropySource::compareBuffers(uint8_t* currentData, uint8_t* oldData, size_t length) {
    // Example comparison: Calculate the sum of absolute differences
    // std::stringstream stream;
    std::vector<uint8_t> data;
	int shift_amount = 0;
	
	// every 5th byte is 2 LSB its from the last 4 pixels, 
	// to reduce bias, rotate byte based on index to set MSBs to different colour wells
    if (format == Mode::RAW) {
		data.reserve(length / 5);
		
		for (size_t i = 4; i < length / 5; i+=5) { 
//			data.push_back(std::rotl( static_cast<uint8_t> (currentData[i]), shift_amount));
			uint8_t byte = std::rotl(static_cast<uint8_t> (currentData[i] ^ oldData[i]), shift_amount); 
			if (byte) {
				data.push_back(byte);
				if ((shift_amount+=1)>= 8) shift_amount = 0;
			}
		}
		
		return data;
    }
	
	// Also uses 5th byte for LSBs
	if (format == Mode::GREYSCALE) {
		data.reserve(length / 5);
		for (size_t i = 4; i < length / 5; i+=5) { 
			data.push_back(static_cast<uint8_t>(currentData[i] - oldData[i])); // get difference in each byte and add to sum, wrapping doesnt matter.
		}
		
		return data;
    }
	
	// RGB is formatted like R:8 B:8 G:8 X:8
	// Loops through all RGB values, and takes the low bit data packs into 1/4 of size while maintaining entropy
    if (format == Mode::RGB) {
		data.reserve((length * 3/4 ) / 4);
		uint8_t current_byte = 0;
		uint8_t shift_amount = 0;
		
		for (size_t i = 0; i < length; i++) { 
			if ((i+1)%4){ // if not padding byte
				if (shift_amount+=2 == 8) {
					shift_amount = 0;
					current_byte = 0; // new byte
					data.push_back(current_byte);
				}
				current_byte |= ((static_cast<uint8_t> (currentData[i] - oldData[i]) & 0b00000011) << shift_amount); // taking last 2 bits from 4 bytes and packing into 1 byte
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
    
	// flip flop between 2 buffers
    oldRequest = currentRequest; // frame from before
    currentRequest = request;    // The frame that just arrived
	
	// on first run only currentRequest holds a valid request, so can't flipflip yet 
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
       
        // grab the buffer pointer of the requests
        FrameBuffer* currentBufferPtr = currentRequest->findBuffer(stream);
        FrameBuffer* oldBufferPtr = oldRequest->findBuffer(stream);

        // getting virtual memory pointer equiavlent to the dma fd
        uint8_t* currentMappedData = processBuffer(currentBufferPtr);
        uint8_t* oldMappedData = processBuffer(oldBufferPtr);

        std::vector<uint8_t>* frame_diff = new std::vector<uint8_t>(compareBuffers(currentMappedData, oldMappedData, currentBufferPtr->planes()[0].length)); 
		
		std::lock_guard<std::mutex> lock(frm_q_mtx);
        if (queue->size() < queueLimit){ // ensure requests are bound by the data processing, as to not cause a memory leak
            queue->push(frame_diff); // pushing the mapped data pointer into the queue
        }
    }
	
}

void EntropySource::workingLoop() {
	pthread_setname_np(pthread_self(), "Camera thread");
	std::cout << "Camera Thread PID: " << syscall(SYS_gettid) << std::endl;

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

int EntropySource::init(size_t _queue_limit) {
    
    // ------------------------ SETTING UP CAMERA CONFIGURATION ------------------------
    
    queueLimit = _queue_limit;
    
    camera->acquire();
    
    std::vector<std::pair<StreamRole, PixelFormat>> roles = {   {StreamRole::Raw, 				formats::SGBRG10_CSI2P}, 
                                                                {StreamRole::VideoRecording, 	formats::R10}, //  
                                                                {StreamRole::Viewfinder, 		formats::RGBX8888} 
																// RGBX8888 more compatible than RGB888 in terms of ISP requests.
                                                            };
    std::unique_ptr<CameraConfiguration> config;
    
	
    // Checking if there are compatible formats
    for (int i = 0; const std::pair<StreamRole, PixelFormat>& role : roles){
        
        config = camera->generateConfiguration( {role.first} );
        if (config == nullptr) continue; // returns nullptr if can't configure
        
        StreamConfiguration& streamConfig = config->at(0); 
        const StreamFormats &formats = streamConfig.formats();
        
        for (const PixelFormat pxlFm : formats.pixelformats()){
            if (pxlFm == role.second){
                streamConfig.pixelFormat = role.second;
                format = static_cast<Mode>(i); // setting mode for future
				std::cout << "Mode: " << role.second.toString() << std::endl;
                goto formatExit;
            }
        }
        i++;
    }
    std::cerr << "No compatible formats found." << std::endl;
        return -1;
	
formatExit:

	StreamConfiguration& streamConfig = config->at(0); 

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
    
    
    camera->requestCompleted.connect(this, &EntropySource::requestComplete);

	libcamera::ControlList startControls(libcamera::controls::controls);
	startControls.set(libcamera::controls::AeEnable,  false);
	startControls.set(libcamera::controls::AwbEnable, false);
	camera->start(&startControls);

	oldRequest = requests[0].get();
	currentRequest = requests[1].get();

	camera->queueRequest(oldRequest); // start flipflop

    return 0;
}

void EntropySource::calibrate(uint32_t timeout_ms, bool force_calibrate) {
	
    EntropyCalibrator cal(camera, stream);

    auto frameCallback = [this](std::vector<uint8_t>& accum, size_t nFrames, int32_t exposureUs, float analogueGain) {
        size_t collected = 0;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
		
        while (collected < nFrames && std::chrono::steady_clock::now() < deadline) {
			if (collected < 2) { // first 2 new requests need controls applied
				auto controls = currentRequest->controls();
					controls.set(libcamera::controls::AeEnable,     false);
					controls.set(libcamera::controls::AwbEnable,    false);
					controls.set(libcamera::controls::ExposureTime, exposureUs);
					controls.set(libcamera::controls::AnalogueGain, analogueGain);
			}
            std::vector<uint8_t>* frame = nullptr;

			if (!queue->empty()) { frame = queue->front(); queue->pop(); }
            if (!frame) { 
				std::this_thread::sleep_for(std::chrono::milliseconds(5)); 
				continue;
			}
            accum.insert(accum.end(), frame->begin(), frame->end());
            delete frame;
            ++collected;
        }
    };
	
    cal.run(frameCallback, timeout_ms);
}


EntropySource::EntropySource(std::shared_ptr<Camera> _camera, 
	std::queue<std::vector<uint8_t>*>* _queue, 
	const std::atomic<bool>* _running, 
	std::mutex& _frame_queue_mutex) 
	: camera(_camera), queue(_queue), running(_running), frm_q_mtx(_frame_queue_mutex) {}

EntropySource::~EntropySource(){
    camera->stop();
    camera->release();
}
