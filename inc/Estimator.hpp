#pragma once
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <bits/stdc++.h>

// NIST SP 800-90B Section 6.3.3 and 6.3.6 implemented by Kai Brewster, 
// Longest Repeated Substring (LRS) Estimate and Markov Estimate
// Inspired by C++ for MSVC implementation by g-g-sakura: 
// https://github.com/g-g-sakura/AnotherEntropyEstimationTool/blob/main/source/EntEstimateLib/ 
class Estimator {
private:

    // Using a map with std::string_view as a key requires a unique hash, 
    // AES or openssl implemenations are too slow as constantly need to be resized.
    // This does not have to be cryptographically secure, just unique.


    
    // FNV-1a rolling hash
    struct ByteSpanHash {
        size_t operator()(std::string_view sv) const {
            size_t hash = 14695981039346656037; 
            for (unsigned char c : sv)
                hash = (hash ^ c) * 1099511628211;   
            return hash;
        }
    };

    std::unordered_map<std::string_view, int, ByteSpanHash> histogram;

public:

    struct Result {
        double p_max;   // estimated probability of most likely symbol sequence
        double h_min;   // final min-entropy in bits per byte  (-log2(p_u))
        int    longest; // longest tuple length with repeated occurrences (v)
    };


    // This function increments length of tuple t (string of sequential bytes), 
    // then iterates all permutations of that tuple length to get every combination in the data, 
    // until there is no more than 1 than one occurance, aka whole data never repeats at that size. 
    static Result Estimate_LRS(const uint8_t* data, size_t n) {
        if (!data || n < 2)
            throw std::invalid_argument("Estimator: data must be non-null and length >= 2");

        
        int     u           = 0;
        int     v           = 0;
        bool    found_u     = false;
        double  p_hat       = 0.0;

        static constexpr int CUTOFF = 35;

        std::unordered_map<std::string_view, int, ByteSpanHash> histogram;
        histogram.reserve(n); // once, before the loop

        for (int t = 1; ; ++t) { // iterate tuple length
            histogram.clear();

            // --- COLLECTING TUPLE FREQUENCY DISTRIBUTION ---

            int max_count = 0;
            const int limit = static_cast<int>(n) - t + 1;
            if (limit <= 0) break;

            for (int i = 0; i < limit; ++i) {
                std::string_view key(reinterpret_cast<const char*>(data + i), t); // key = current tuple aka string of bytes (char) from data[i] to data[i+t-1]
                int count = ++histogram[key]; 
                if (count > max_count)
                    max_count = count;
            }

            // -----------------------------------------------

            // step 1: first u = t where max_count < cutoff (stated at 35)
            if (!found_u) {
                if (max_count < CUTOFF) {
                    u       = t;
                    found_u = true;
                }
                continue; // keep iterating to find u
            }

            // steps 2 / 3 : for t (W) in u -> v, find P_W and update p_hat as highest P_max_W
            // since P_W needs to be collected for t in u -> v, might as well store P_W when iterating from u, 
            // and will find v when max_count transitions from 2->1
            if (found_u) {
                if (max_count >= 2) {
                    auto& W = t; // for t > u, W = t;
                    v = W;  // v iterates as long as some tuple still repeats i.e first value where max_count == 1, W = v+1

                    // sum of (C_i choose 2) -> sum of C_i * (C_i-1) / 2
                    uint64_t numerator = 0;
                    for (const auto& [key, count] : histogram) {
                        if (count >= 2)
                            numerator += (static_cast<uint64_t>(count) * (count - 1)) / 2;
                    }

                    // ((L-W+1) choose 2) -> (L-W+1) * (L-W) / 2
                    const double L_minus_W_plus_1 = static_cast<double>(limit);
                    const double denominator = (L_minus_W_plus_1 * (L_minus_W_plus_1 - 1)) / 2.0;

                    if (denominator > 0.0) {
                        const double P_W  = static_cast<double>(numerator) / denominator;
                        const double P_max_W = std::pow(P_W, (1.0 / static_cast<double>(W))); // P_max_W = P_W^(1/W)

                        if (P_max_W > p_hat) { // track max P_max_W across all t in u -> v
                            p_hat  = P_max_W;
                        }
                    }
                } else {
                    // max_count < 2 — no tuple repeats at this length, done
                    break;
                }
            }
        }
        
        if (!found_u || p_hat <= 0.0)
            throw std::runtime_error("Estimator: insufficient data or no repeated tuples found");

        // step 4: p_u = p_hat + 2.576 * sqrt(p_hat * (1 - p_hat) / (L - 1))
        double p_u = min(p_hat + 2.576 * std::sqrt((p_hat * (1.0 - p_hat)) 
                            / static_cast<double>(n - 1)), 1.0); 

        // step 5: final min-entropy
        const double h_min = -std::log2(p_u); // done

        return Result{
            .p_max   = p_hat,
            .h_min   = h_min,
            .longest = v,
        };
    }

    // Estimate min-entropy from a byte buffer.
    // Requires at least 2 bytes; returns {1.0, 8.0, 0, 0} otherwise.
    static Result Estimate_Markov(const uint8_t* data, size_t n) {

        if (n < 2)
            return { 1.0, 8.0, 0 };

        // 64 bit data will require 8 times less increments / copies
        const uint64_t* words = reinterpret_cast<const uint64_t*>(data); 
        
        static constexpr int    SEQ_LEN = 128;   // path length L
        
        size_t n_bits = n * 8;
        size_t n_words = n / 8;
        std::map<uint8_t, size_t> transition_count = {{0b00, 0}, 
                                                      {0b01, 0},
                                                      {0b10, 0},
                                                      {0b11, 0}
                                                    };

        // Find count of each transition & count of ones (step 1 + 2)
        size_t ones = 0;
        for (size_t w = 0; w < n_words; ++w){
            uint64_t word = words[w];
            for (uint8_t bit = 0; bit < 63; bit+=2){
                uint8_t key = word & 0b11; // last 2 bits = transition state e.g 01: 0->1
                transition_count[key]++;
                word = word >> 2;
            }
            ones += __builtin_popcountll(word);
        }     

        // Initial probabilities
        double P1  = static_cast<double> (ones) / n_bits;
        double P0  = 1.0 - P1;

        // Transition probabilities
        double P00 = static_cast<double> (transition_count[0b00]) / (transition_count[0b01] + transition_count[0b00]);
        double P01 = 1.0 - P00; 
        double P10 = static_cast<double> (transition_count[0b10]) / (transition_count[0b11] + transition_count[0b10]);
        double P11 = 1.0 - P10;

        std::map<uint8_t, double> transition_prob = {{0b00, P00}, 
                                                     {0b01, P01},
                                                     {0b10, P10},
                                                     {0b11, P11}
                                                    };


        // Sequence probabilities
        auto p_s = [transition_prob](uint8_t b){ 
            std::vector<double> p_s;
            uint8_t c_b = b ^ 0b01;

            // x,y can be either 0 or 1 

            // P_x * P_xx^127 (Doesn't change)
            p_s.push_back(pow(transition_prob.at(b), SEQ_LEN-1.0)); 

            // P_x * P_xy^64 * P_yx^63 (Alternating) 
            p_s.push_back(pow(transition_prob.at(c_b), SEQ_LEN/2.0) 
                        * pow(transition_prob.at(c_b^0b11), (SEQ_LEN/2)-1)); 

            // P_x * P_xy * P_yy^126 (Inverts)
            p_s.push_back(transition_prob.at(c_b)                   
                        * pow(transition_prob.at(b^0b11), SEQ_LEN-2.0));
                        
            return *std::max_element(p_s.begin(), p_s.end()); // return highest probability
        };


        double p_hat = std::max(P0 * p_s(0b00), P1 * p_s(0b11));

        // step 5: final min-entropy
        const double h_min = std::min(-std::log2(p_hat)/128, 1.0); // done

        return Result{
            .p_max   = p_hat,
            .h_min   = h_min,
            .longest = SEQ_LEN,
        };
    }
        

};


 