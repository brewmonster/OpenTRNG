#ifndef HASH_DIGESTER_HPP
#define HASH_DIGESTER_HPP

#include <openssl/evp.h>
#include <openssl/md5.h>
#include <cstdint>
#include <vector>
#include <string>

class HashDigester {
private:
    EVP_MD_CTX* mdctx;
    const EVP_MD* md;

public:
    HashDigester(const EVP_MD* algorithm = EVP_sha256());
    ~HashDigester();
    
    void update(const uint8_t* data, size_t length);
    std::vector<uint8_t> finalize();
    void reset();
};

#endif 
