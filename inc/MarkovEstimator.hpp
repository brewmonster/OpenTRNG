#pragma once
#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <stdexcept>

// ============================================================
// NIST SP 800-90B Section 6.3.3 — Markov Estimator
//
// Models first-order byte transition probabilities P(s_j | s_i),
// finds the most probable sequence of length L, and returns
// min-entropy = -log2(p_max) / L  (bits per byte).
// ============================================================

class MarkovEstimator {

    static constexpr int    ALPHA   = 256;   // byte alphabet
    static constexpr int    SEQ_LEN = 128;   // path length L
    static constexpr double EPSILON = 1e-300; // underflow guard

public:

    struct Result {
        double   p_max;        // probability of most likely sequence
        double   h_min;        // min-entropy in bits per byte
        int      seq_len;      // L used
        uint8_t  most_common;  // most probable initial symbol
    };

    // Estimate min-entropy from a byte buffer.
    // Requires at least 2 bytes; returns {1.0, 8.0, 0, 0} otherwise.
    static Result estimate(const uint8_t* data, size_t n) {

        if (n < 2)
            return { 1.0, 8.0, 0, 0 };

        // ── Initial symbol frequencies ────────────────────────
        double init_prob[ALPHA] = {};
        for (size_t i = 0; i < n; ++i)
            init_prob[data[i]] += 1.0;
        for (int i = 0; i < ALPHA; ++i)
            init_prob[i] /= static_cast<double>(n);

        // ── Transition counts T[from][to] ────────────────────
        // Heap-allocate: 256×256 doubles = 512 KiB on stack is too much
        std::vector<double> trans(ALPHA * ALPHA, 0.0);
        auto T = [&](int i, int j) -> double& {
            return trans[i * ALPHA + j];
        };

        for (size_t k = 0; k + 1 < n; ++k)
            T(data[k], data[k + 1]) += 1.0;

        // Normalise rows → conditional probabilities P(j | i)
        for (int i = 0; i < ALPHA; ++i) {
            double row_sum = 0.0;
            for (int j = 0; j < ALPHA; ++j)
                row_sum += T(i, j);
            if (row_sum > 0.0)
                for (int j = 0; j < ALPHA; ++j)
                    T(i, j) /= row_sum;
            else
                // Unseen symbol: uniform fallback
                for (int j = 0; j < ALPHA; ++j)
                    T(i, j) = 1.0 / ALPHA;
        }

        // ── Greedy most-probable path of length L ─────────────
        int L = static_cast<int>(std::min<size_t>(SEQ_LEN, n));

        // Start from the most probable initial symbol
        uint8_t s = static_cast<uint8_t>(
            std::max_element(init_prob, init_prob + ALPHA) - init_prob
        );

        double p_max = init_prob[s] > 0.0 ? init_prob[s] : EPSILON;

        for (int step = 0; step < L - 1; ++step) {
            // Find the best next symbol from s
            const double* row = &trans[s * ALPHA];
            uint8_t best = static_cast<uint8_t>(
                std::max_element(row, row + ALPHA) - row
            );
            p_max *= (row[best] > 0.0 ? row[best] : EPSILON);
            s      = best;

            // Clamp underflow early
            if (p_max < EPSILON) { p_max = EPSILON; break; }
        }

        double h_min = -std::log2(p_max) / static_cast<double>(L);
        if (h_min > 8.0) h_min = 8.0;

        return { p_max, h_min, L, static_cast<uint8_t>(
            std::max_element(init_prob, init_prob + ALPHA) - init_prob
        )};
    }

    // Convenience overload for std::vector
    static Result estimate(const std::vector<uint8_t>& data) {
        return estimate(data.data(), data.size());
    }
};