#ifndef OUTPUT_PORT_HPP
#define OUTPUT_PORT_HPP

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <limits>

#include <poll.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include <openssl/evp.h>
#include <openssl/rand.h>
#include <openssl/err.h>

static constexpr size_t AES_KEY_LEN   = 32; // AES-256
static constexpr size_t GCM_NONCE_LEN = 12; // 96-bit nonce, standard for GCM
static constexpr size_t GCM_TAG_LEN   = 16; // 128-bit authentication tag

// CLI flags passed in from main, forwarded into OutputPort::init()
struct OutputFlags {
    bool forceSerial  = false; // -s / --serial
    bool noPrompt     = false; // -n / --no-input  (skip y/n, take defaults)
	bool quiet		  = false; // -q / shuddup
	bool pseudo		  = false; // --p / prng to increase throughput.
};

std::vector<uint8_t> loadSharedKey(const std::string& path = "/key");

std::vector<uint8_t> aesGcmEncrypt(const uint8_t* plaintext, size_t length,
                                    const std::vector<uint8_t>& key);


// ======================== TRANSPORT PARENT CLASS ========================

class DataSink {
public:
    virtual ~DataSink() = default;
    virtual bool open() = 0;
    virtual void writeData(const uint8_t* data, size_t length) = 0;
    virtual void close() = 0;
};


// ======================== SERIAL (IF GADGET MODE EXISTS) ========================

class SerialPort : public DataSink {
public:
    SerialPort(const std::string& device, speed_t baud = B115200);
    ~SerialPort();
    bool open() override;
    void writeData(const uint8_t* data, size_t length) override;
    void close() override;

private:
    std::string device;
    speed_t baud;
    int fd;
    std::atomic<bool> running{false};
    std::thread writeThread;
    std::queue<std::shared_ptr<std::vector<uint8_t>>> writeQueue;
    std::mutex queueMutex;
    std::condition_variable queueCV;

    void writerLoop();
    bool writeAll(const uint8_t* data, size_t length);
};


// ======================== TCP SERVER ========================
// Threadless — runs entirely on the caller's thread (the hash thread).
// writeData() does a zero-timeout poll() on the server socket to pick up
// any pending connection before each send, then writes synchronously.
//
// Wire format per frame:
//   [4-byte total length, big-endian]  — covers nonce + tag + ciphertext
//   [12-byte nonce]
//   [16-byte GCM tag]
//   [N-byte ciphertext]

class TCPServer : public DataSink {
public:
    explicit TCPServer(uint16_t port, std::vector<uint8_t> key);
    ~TCPServer();
    bool open() override;

    // Called from the hash thread. Polls for a new client connection,
    // then encrypts and sends synchronously. If the client has disconnected,
    // the frame is discarded and the server waits for the next connection.
    void writeData(const uint8_t* data, size_t length) override;

    void close() override;

private:
    uint16_t port;
    int serverFd;
    int clientFd;
    std::vector<uint8_t> key;

    // Non-blocking check for a pending connection on serverFd.
    // Replaces clientFd if a new client has connected.
    void pollAccept();

    bool writeAll(int fd, const uint8_t* data, size_t length);
};


// ======================== OUTPUT PORT ========================

class OutputPort {
public:
    OutputPort() = default;
    ~OutputPort() = default;

    // flags come from CLI parsing in main().
    bool init(const OutputFlags& flags);
    void writeData(const uint8_t* data, size_t length);

private:
    std::unique_ptr<DataSink> sink;

    static bool isGadgetModeAvailable();
    static bool promptYN(const std::string& question, bool noPrompt, bool defaultYes = true);
    static uint16_t promptPort(bool noPrompt);
    static bool promptKey(const std::string& path);
};

#endif
