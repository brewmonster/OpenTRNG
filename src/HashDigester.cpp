#include "HashDigester.hpp"
#include <openssl/err.h>
#include <cstring>
#include <stdexcept>

HashDigester::HashDigester(const EVP_MD* algorithm) : md(algorithm) {
    mdctx = EVP_MD_CTX_new();
    if (!mdctx) {
        throw std::runtime_error("Failed to create EVP_MD_CTX");
    }
    
    if (EVP_DigestInit_ex(mdctx, md, nullptr) != 1) {
        EVP_MD_CTX_free(mdctx);
        throw std::runtime_error("Failed to initialize digest context");
    }
}

HashDigester::~HashDigester() {
    if (mdctx) {
        EVP_MD_CTX_free(mdctx);
    }
}

void HashDigester::update(const uint8_t* data, size_t length) {
    if (!mdctx) {
        throw std::runtime_error("Digest context is not initialized");
    }
    
    if (EVP_DigestUpdate(mdctx, data, length) != 1) {
        throw std::runtime_error("Failed to update digest");
    }
}

std::vector<uint8_t> HashDigester::finalize() {
    if (!mdctx) {
        throw std::runtime_error("Digest context is not initialized");
    }
    
    unsigned int digest_length = EVP_MD_size(md);
    std::vector<uint8_t> digest(digest_length);
    unsigned int result_length = 0;
    
    if (EVP_DigestFinal_ex(mdctx, digest.data(), &result_length) != 1) {
        throw std::runtime_error("Failed to finalize digest");
    }
    
    digest.resize(result_length);
    return digest;
}

void HashDigester::reset() {
    if (!mdctx) {
        throw std::runtime_error("Digest context is not initialized");
    }
    
    EVP_MD_CTX_free(mdctx);
    mdctx = EVP_MD_CTX_new();
    if (!mdctx) {
        throw std::runtime_error("Failed to create EVP_MD_CTX during reset");
    }
    
    if (EVP_DigestInit_ex(mdctx, md, nullptr) != 1) {
        throw std::runtime_error("Failed to initialize digest context during reset");
    }
}
