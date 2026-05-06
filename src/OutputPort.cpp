#include "OutputPort.hpp"


// ======================== KEY LOADING ========================

std::vector<uint8_t> loadSharedKey(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open())
        throw std::runtime_error("Cannot open key file: " + path);

    std::vector<uint8_t> key(AES_KEY_LEN);
    f.read(reinterpret_cast<char*>(key.data()), AES_KEY_LEN);

    if (static_cast<size_t>(f.gcount()) != AES_KEY_LEN)
        throw std::runtime_error("Key file must be exactly 32 bytes (got "
                                 + std::to_string(f.gcount()) + ")");
    return key;
}


// ======================== AES-256-GCM ENCRYPTION ========================

std::vector<uint8_t> aesGcmEncrypt(const uint8_t* plaintext, size_t length,
                                    const std::vector<uint8_t>& key) {
    std::vector<uint8_t> out(GCM_NONCE_LEN + GCM_TAG_LEN + length);

    uint8_t* nonce      = out.data();
    uint8_t* tag        = out.data() + GCM_NONCE_LEN;
    uint8_t* ciphertext = out.data() + GCM_NONCE_LEN + GCM_TAG_LEN;

    if (RAND_bytes(nonce, GCM_NONCE_LEN) != 1)
        throw std::runtime_error("RAND_bytes failed");

    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (!ctx) throw std::runtime_error("EVP_CIPHER_CTX_new failed");

    int len = 0;
    EVP_EncryptInit_ex(ctx, EVP_aes_256_gcm(), nullptr, nullptr, nullptr);
    EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_SET_IVLEN, GCM_NONCE_LEN, nullptr);
    EVP_EncryptInit_ex(ctx, nullptr, nullptr, key.data(), nonce);
    EVP_EncryptUpdate(ctx, ciphertext, &len, plaintext, static_cast<int>(length));
    EVP_EncryptFinal_ex(ctx, ciphertext + len, &len);
    EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_GET_TAG, GCM_TAG_LEN, tag);

    EVP_CIPHER_CTX_free(ctx);
    return out;
}


// ======================== SERIAL ========================

SerialPort::SerialPort(const std::string& device, speed_t baud)
    : device(device), baud(baud), fd(-1) {}

SerialPort::~SerialPort() {
    running = false;
    queueCV.notify_all();
    if (writeThread.joinable()) writeThread.join();
    close();
}

bool SerialPort::open() {
    fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << device << ": " << strerror(errno) << std::endl;
        return false;
    }

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr error: " << strerror(errno) << std::endl;
        return false;
    }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr error: " << strerror(errno) << std::endl;
        return false;
    }

    running = true;
    writeThread = std::thread(&SerialPort::writerLoop, this);
    std::cout << "Gadget serial open on " << device << std::endl;
	
    return true;
}

void SerialPort::writeData(const uint8_t* data, size_t length) {
    auto buf = std::make_shared<std::vector<uint8_t>>(data, data + length);
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        writeQueue.push(buf);
    }
    queueCV.notify_one();
}

void SerialPort::close() {
    if (fd >= 0) { ::close(fd); fd = -1; }
}

void SerialPort::writerLoop() {
    while (running) {
        std::shared_ptr<std::vector<uint8_t>> buf;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            queueCV.wait(lock, [this]{ return !writeQueue.empty() || !running; });
            if (!running && writeQueue.empty()) break;
            buf = writeQueue.front();
            writeQueue.pop();
        }
        writeAll(buf->data(), buf->size());

    }
}

bool SerialPort::writeAll(const uint8_t* data, size_t length) {
    size_t written = 0;
    while (written < length) {
        ssize_t n = ::write(fd, data + written, length - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            std::cerr << "Serial write error: " << strerror(errno) << std::endl;
            return false;
        }
        written += n;
    }
    return true;
}



// ======================== TCP SERVER ========================

TCPServer::TCPServer(uint16_t port, std::vector<uint8_t> key)
    : port(port), serverFd(-1), clientFd(-1), key(std::move(key)) {}

TCPServer::~TCPServer() {
    close();
}

bool TCPServer::open() {
    serverFd = socket(AF_INET, SOCK_STREAM, 0);
    if (serverFd < 0) {
        std::cerr << "TCP socket error: " << strerror(errno) << std::endl;
        return false;
    }

    // Non-blocking so pollAccept() never stalls the hash thread
    fcntl(serverFd, F_SETFL, O_NONBLOCK);

    int opt = 1;
    setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(port);

    if (bind(serverFd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "TCP bind error: " << strerror(errno) << std::endl;
        return false;
    }

    if (listen(serverFd, 1) < 0) {
        std::cerr << "TCP listen error: " << strerror(errno) << std::endl;
        return false;
    }

    std::cout << "TCP server listening on port " << port
              << " — waiting for client..." << std::endl;
	


    return true;
}

void TCPServer::pollAccept() {
    // Zero-timeout poll — returns immediately with however many fds are ready
    struct pollfd pfd{ serverFd, POLLIN, 0 };
    if (poll(&pfd, 1, 0) <= 0) return; // nothing pending

    sockaddr_in clientAddr{};
    socklen_t len = sizeof(clientAddr);
    int newFd = accept(serverFd, reinterpret_cast<sockaddr*>(&clientAddr), &len);
    if (newFd < 0) return;

    int flag = 1;
    setsockopt(newFd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    std::cout << "Client connected: " << inet_ntoa(clientAddr.sin_addr) << std::endl;

    if (clientFd >= 0) ::close(clientFd);
    clientFd = newFd;
	
}

void TCPServer::writeData(const uint8_t* data, size_t length) {
    // Check for a new or replacement client before every send
    pollAccept();

    if (clientFd < 0) return; // no client yet, discard

    std::vector<uint8_t> encrypted = aesGcmEncrypt(data, length, key);

    uint32_t frameLen = htonl(static_cast<uint32_t>(encrypted.size()));
    std::vector<uint8_t> frame(sizeof(frameLen) + encrypted.size());
    memcpy(frame.data(), &frameLen, sizeof(frameLen));
    memcpy(frame.data() + sizeof(frameLen), encrypted.data(), encrypted.size());

    if (!writeAll(clientFd, frame.data(), frame.size())) {
        std::cout << "Client disconnected, waiting for reconnect..." << std::endl;
        ::close(clientFd);
        clientFd = -1; // removing the client 
    }
}

void TCPServer::close() {
    if (clientFd >= 0) { ::close(clientFd); clientFd = -1; }
    if (serverFd >= 0) { ::close(serverFd); serverFd = -1; }
}

bool TCPServer::writeAll(int fd, const uint8_t* data, size_t length) {
    size_t written = 0;
    while (written < length) {
        ssize_t n = send(fd, data + written, length - written, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false; // EPIPE, ECONNRESET etc.
        }
        written += n;
    }
    return true;
}




// ======================== OUTPUT PORT ========================

bool OutputPort::isGadgetModeAvailable() {
    const std::string udcPath = "/sys/class/udc";
    if (!std::filesystem::exists(udcPath)) return false;
    for (const auto& entry : std::filesystem::directory_iterator(udcPath)) {
        (void)entry;
        return true;
    }
    return false;
}

// noPrompt=true skips the question and returns defaultYes.
bool OutputPort::promptYN(const std::string& question, bool noPrompt, bool defaultYes) {
    if (noPrompt) {
        std::cout << question << " [y/n]: " << (defaultYes ? "y" : "n")
                  << " (--no-input)" << std::endl;
        return defaultYes;
    }
    char answer = '\0';
    while (answer != 'y' && answer != 'n') {
        std::cout << question << " [y/n]: ";
        std::cin >> answer;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        answer = static_cast<char>(std::tolower(answer));
    }
    return answer == 'y';
}

uint16_t OutputPort::promptPort(bool noPrompt) {
    if (noPrompt) {
        std::cout << "TCP port: 7777 (--no-input)" << std::endl;
        return 7777;
    }
    std::string input;
    std::cout << "TCP port [default 7777]: ";
    std::getline(std::cin, input);
    if (input.empty()) return 7777;
    try {
        int p = std::stoi(input);
        if (p > 0 && p <= 65535) return static_cast<uint16_t>(p);
    } catch (...) {}
    std::cout << "Invalid port, using 7777." << std::endl;
    return 7777;
}

bool OutputPort::promptKey(const std::string& path) {
    std::string input;
    std::cout << "AES key must be exactly 32 characters." << std::endl;
    while (true) {
        std::cout << "Enter encryption key: ";
        std::getline(std::cin, input);
        if (input.length() == AES_KEY_LEN) break;
        std::cout << "Invalid length (" << input.length() << " chars), need 32." << std::endl;
    }
    try {
        std::ofstream f(path, std::ios::binary);
		if (!f.is_open())
            throw std::runtime_error("Cannot create key file: " + path);
        f.write(input.data(), AES_KEY_LEN);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
    return true;
}

std::string SerialPort::getMessage(){
	std::string s = "Using Serial connection to: "+device+ "\tat "+std::to_string(baud)+"baud.";
	return s;
}

std::string TCPServer::getMessage(){
	std::string s = "Using TCP connection on port: "+std::to_string(port)+"\twith fd: "+std::to_string(clientFd);
	return s;
}



void OutputPort::startLog(const OutputFlags& flgs){
	try {
		std::ofstream logFile("./log", std::ios::out);
		if (!logFile.is_open())
			throw std::runtime_error("Cannot open log file");
		time_t time;
		std::time(&time);
		logFile << "\033[2JLOGGING BEGINNING AT: " << std::ctime(&time) << std::endl;
		
			
		logFile << sink->getMessage() << std::endl;
		} catch (const std::exception& e) {
			std::cerr << e.what() << std::endl;
			return;
		}
	return;
}


bool OutputPort::writeToFile(const char* data, size_t length){
	try {
		std::ofstream logFile;
		logFile.open("./log", std::ios::app | std::ios::out);
		if (!logFile.is_open())
			throw std::runtime_error("Cannot open log file");
	
		logFile << std::hex << std::setfill('0');

		for (size_t i=0; i < length; i++){
			logFile << std::setw(2) << static_cast<int>(data[i]);
		}
		
		logFile << std::endl;
		
	} catch (const std::exception& e) {
		std::cerr << e.what() << std::endl;
		return false;
	}
	return true;
}

bool OutputPort::init(const OutputFlags& flags) {
	
	
    // ---- Load shared key ----
    std::vector<uint8_t> key;
    while (key.empty()) {
        try {
            key = loadSharedKey("./key");
            std::cout << "[Output] Shared key loaded from /key." << std::endl;
        } catch (const std::runtime_error& e) {
            std::cerr << "[Output] " << e.what() << std::endl;
            if (flags.noPrompt) {
                std::cerr << "[Output] Cannot prompt for key in --no-input mode." << std::endl;
                return false;
            }
            if (!promptYN("/key not found or invalid. Enter a new key?", false))
                return false;
            if (!promptKey("./key"))
                return false;
        }
    }

    // ---- Priority 1: TCP (unless --serial forced) ----
    if (!flags.forceSerial) {
        std::cout << "\n[Output] Network (TCP) is available." << std::endl;
        if (promptYN("Use TCP output? (Uses IPv4)", flags.noPrompt, /*defaultYes=*/true)) {
            uint16_t port = promptPort(flags.noPrompt);
            sink = std::make_unique<TCPServer>(port, key);
            if (sink->open()) {
				if ((this->isLogging = flags.logging)) startLog(flags);;
				return true;
			}
            std::cerr << "[Output] TCP failed to open." << std::endl;
            sink.reset();
        }
    } else {
        std::cout << "\n[Output] --serial flag set, skipping TCP." << std::endl;
    }

    // ---- Priority 2: USB gadget serial ----
    if (isGadgetModeAvailable()) {
        const std::string gadgetDev = "/dev/ttyGS0";
        if (std::filesystem::exists(gadgetDev)) {
            std::cout << "\n[Output] USB gadget serial detected (" << gadgetDev << ")." << std::endl;
            if (promptYN("Use USB gadget serial output?", flags.noPrompt, /*defaultYes=*/true)) {
                sink = std::make_unique<SerialPort>(gadgetDev);
					if ((isLogging = flags.logging)) startLog(flags);
                if (sink->open()) {
					return true;
				}
                std::cerr << "[Output] Gadget serial failed to open." << std::endl;
                sink.reset();
            }
        } else {
            std::cout << "\n[Output] UDC present but " << gadgetDev
                      << " not found — g_serial/g_cdc module may not be loaded." << std::endl;
        }
    } else {
        std::cout << "\n[Output] No USB Device Controller — gadget mode unavailable." << std::endl;
    }

    std::cerr << "\n[Output] No output transport could be established." << std::endl;
    return false;
}

void OutputPort::writeData(const uint8_t* data, size_t length) {
	if (isLogging) writeToFile(reinterpret_cast<const char*> (data), length);
    if (sink) sink->writeData(data, length);
}
