#pragma once
#include <cstdint>
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <string>
#include <optional>
#include <functional>
#include <libcamera/libcamera.h>
#include "MarkovEstimator.hpp"

// ============================================================
// EntropyCalibrator
//
// Finds the camera exposure + gain settings that maximise
// sensor noise entropy, using the Markov estimator on processed
// frame data from the existing frameQueue.
//
// Phase 1 — Plateau search:
//   Increase exposure in coarse steps. At each step collect
//   frames and measure H_min. Stop when H_min stops improving
//   (plateaus) — that is the sensor noise ceiling.
//
// Phase 2 — Fine trim:
//   Binary search around the plateau exposure to find the
//   smallest exposure that still achieves plateau-level H_min.
//   Smaller exposure = higher framerate = more entropy/second.
//
// Phase 3 — Markov validation + persist.
//
// On subsequent runs, loads cached settings and does a quick
// H_min check before deciding to re-calibrate.
//
// FrameCallback signature:
//   void(std::vector<uint8_t>& accum, size_t nFrames)
// ============================================================

struct CalibrationResult {
    int32_t exposureUs  = 100000;
    float   gain        = 8.0f;
    double  h_min       = 0.0;
    bool    timedOut    = false;
    bool    fromCache   = false;
};

using FrameCallback = std::function<void(std::vector<uint8_t>&, size_t, int32_t, float)>;

class EntropyCalibrator {
public:

    // Exposure bounds and step sizes (microseconds)
    int32_t EXP_START       = 10000;   // 1ms
    int32_t EXP_MAX         = 98000;  // 50ms
    int32_t EXP_STEP_COARSE = 30000;   // 2ms — plateau search
    int32_t EXP_STEP_FINE   = 1000;   // 10ms — trim searc
	
	float ANALOGUE_GAIN_START  = 1.0;  
	float ANALOGUE_GAIN_MAX  = 8.0;  
	float ANALOGUE_GAIN_STEP  = 1.0;
	float ANALOGUE_GAIN_STEP_FINE  = 0.2;
 

    // Frames collected per measurement
    static constexpr size_t  SAMPLE_FRAMES   = 6;
    static constexpr size_t  QUICK_FRAMES    = 3;

    // H_min must improve by at least this to not count as plateaued
    static constexpr double  PLATEAU_MIN_GAIN = 0.05;   // 0.05 bits/byte

    // Consecutive steps without improvement before confirming plateau
    static constexpr int     PLATEAU_CONFIRM  = 2;

    // Early exit if we hit near-theoretical max
    static constexpr double  EARLY_EXIT_H     = 7.5;

    // Cached settings degradation tolerance before re-calibrating
    static constexpr double  RECHECK_TOLERANCE = 0.10;  // 10%

    static constexpr const char* SETTINGS_PATH = "./calibration";

    EntropyCalibrator(std::shared_ptr<libcamera::Camera> cam,
						libcamera::Stream*                 strm)
						: camera(cam), stream(strm) {
							
		const libcamera::ControlInfoMap& controls = camera->controls();

		// Check if AnalogueGain is supported
		auto it = controls.find(&libcamera::controls::AnalogueGain);
		if (it != controls.end()) {
			ANALOGUE_GAIN_START = it->second.min().get<float>();
			ANALOGUE_GAIN_MAX = it->second.max().get<float>();
			ANALOGUE_GAIN_STEP = (ANALOGUE_GAIN_MAX - ANALOGUE_GAIN_START) / 8;
			ANALOGUE_GAIN_STEP_FINE = ANALOGUE_GAIN_STEP / 10;
		}

		// Same for ExposureTime
		auto it2 = controls.find(&libcamera::controls::ExposureTime);
		if (it2 != controls.end()) {
			EXP_START = it2->second.min().get<int32_t>();
			EXP_MAX = it2->second.max().get<int32_t>();
			EXP_STEP_COARSE = (EXP_MAX - EXP_START) / 4;
			EXP_STEP_FINE = EXP_STEP_COARSE / 10;
		}
	}

    CalibrationResult run(FrameCallback frameCallback,
                          uint32_t      timeout_ms = 600000,
						  bool 			force_calibrate = false)  {
		
		// ------------------------ LOADING SAVED CALIBRATION ------------------------
		if (!force_calibrate){
			auto cached = loadSettings();
			if (cached) {
				std::cout << "[Calibration] Cached settings found:\n"
						  << "  Exposure : " << cached->exposureUs << " us\n"
						  << "  Gain     : " << cached->gain       << "\n"
						  << "  H_min    : " << cached->h_min      << " bits/byte\n"
						  << "[Calibration] Quick validation...\n";


				std::vector<uint8_t> accum;
				frameCallback(accum, QUICK_FRAMES, cached->exposureUs, cached->gain);
				double checked_h = MarkovEstimator::estimate(accum).h_min;

				double degradation = cached->h_min > 0.0 ?
									(cached->h_min - checked_h) / cached->h_min
									: 1.0;

				std::cout << "  Checked H_min=" << std::fixed << std::setprecision(4)
						  << checked_h << "  degradation=" << degradation * 100.0 << "%\n";

				if (degradation <= RECHECK_TOLERANCE && degradation > 0.0) {
					std::cout << "[Calibration] Cache still valid, skipping sweep.\n\n";
					cached->fromCache = true;
					return *cached;
				}
				std::cout << "[Calibration] Cache degraded, re-calibrating.\n";
			} else {
				std::cout << "[Calibration] No cache found, running full calibration.\n";
			}
		}
		
        return fullCalibration(frameCallback, timeout_ms);
    }


private:

    std::shared_ptr<libcamera::Camera> camera;
    libcamera::Stream*                 stream;
    float                              gain;


    double measure(FrameCallback& cb, size_t nFrames, int32_t exposureUs, float analogueGain) { // call multipel frames and test the total entropy
        std::vector<uint8_t> accum;
        cb(accum, nFrames, exposureUs, analogueGain);
        if (accum.empty()) return 0.0;
        return MarkovEstimator::estimate(accum).h_min;
    }


    CalibrationResult fullCalibration(FrameCallback& frameCallback,
                                      uint32_t       timeout_ms) {
										  
		using Clock = std::chrono::steady_clock;
        auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms / 2);

		
        double  bestH        = 0.0;
		
        float 	bestGain	 = ANALOGUE_GAIN_START;
        int32_t bestExp		 = EXP_START;
		
		std::cout << "\n\n\n\n" << std::endl;
		for (float gain = ANALOGUE_GAIN_START; gain < ANALOGUE_GAIN_MAX; gain += ANALOGUE_GAIN_STEP){
			for (int32_t exposure = EXP_START; exposure < EXP_MAX; exposure += EXP_STEP_COARSE){
				if (Clock::now() >= deadline) {
					std::cout << "[Calibration] Timed out. \n";
					goto CalibrateExit;
				}
				
				double h = measure(frameCallback, SAMPLE_FRAMES, exposure, gain);
				
				if (h > bestH){
					bestH = h;
					bestGain = gain;
					bestExp = exposure;
				}
				std::cout << "\033[4A\n[Calibration] Current iteration:\n"
                  << "\033[2K  Exposure : " << exposure << " us   \t| Best:" << bestExp << "\n"
                  << "\033[2K  Gain     : " << gain       << "  \t\t| Best:" << bestGain << "\n"
                  << "\033[2K  H_min    : " << h      << " bits/byte  \t| Best:" << bestH 
				  << (bestH < 3.0 ? "  [WARNING: low entropy]" : "  [OK]");
				  
				if (bestH >= EARLY_EXIT_H) goto CalibrateExit;
			}
		}
		
		if (bestGain+ANALOGUE_GAIN_STEP < ANALOGUE_GAIN_MAX) ANALOGUE_GAIN_MAX = bestGain+ANALOGUE_GAIN_STEP;
		if (bestExp+EXP_STEP_COARSE < EXP_MAX) EXP_MAX = bestExp+EXP_STEP_COARSE;
		
		for (float gain = bestGain; gain < ANALOGUE_GAIN_MAX; gain += ANALOGUE_GAIN_STEP_FINE){
			for (int32_t exposure = bestExp; exposure < EXP_MAX; exposure += EXP_STEP_FINE){
				if (Clock::now() >= deadline) {
						std::cout << "[Calibration] Timed out. \n";
						goto CalibrateExit;
					}
				double h = measure(frameCallback, QUICK_FRAMES, exposure, bestGain);
				if (h > bestH){
					bestH = h;
					bestGain = gain;
					bestExp = exposure;
				}
				std::cout << "\033[4A\n[Calibration] Current iteration:\n"
                  << "\033[2K  Exposure : " << exposure << " us   \t| Best:" << bestExp << "\n"
                  << "\033[2K  Gain     : " << gain       << "  \t\t| Best:" << bestGain << "\n"
                  << "\033[2K  H_min    : " << h      << " bits/byte  \t| Best:" << bestH  
				  << (bestH < 3.0 ? "  [WARNING: low entropy]" : "  [OK]");
			}
		}
		
CalibrateExit:

        CalibrationResult result;
        result.gain = bestGain;
		result.exposureUs = bestExp;
		result.h_min = bestH;
   
        std::cout << "\033[4A\n[Calibration] Complete:\n"
                  << "  Exposure : " << result.exposureUs << " us\n"
                  << "  Gain     : " << result.gain       << "\n"
                  << "  H_min    : " << result.h_min      << " bits/byte"
				  << (bestH < 3.0 ? "  [WARNING: low entropy]" : "  [OK]\n\n");

        saveSettings(result);
        return result;
    }


    // ── Persistence ───────────────────────────────────────────
    void saveSettings(const CalibrationResult& r) {
        std::ofstream f(SETTINGS_PATH);
        if (!f.is_open()) {
            std::cerr << "[Calibration] Could not write to "
                      << SETTINGS_PATH << "\n";
            return;
        }
        auto ts = std::chrono::duration_cast<std::chrono::seconds>(
                      std::chrono::system_clock::now().time_since_epoch()).count();
        f << r.exposureUs << "\n"
          << r.gain       << "\n"
          << r.h_min      << "\n"
          << ts           << "\n";
        std::cout << "[Calibration] Settings saved to " << SETTINGS_PATH << "\n";
    }

    std::optional<CalibrationResult> loadSettings() {
        std::ifstream f(SETTINGS_PATH);
        if (!f.is_open()) return std::nullopt;

        CalibrationResult r;
        int64_t ts = 0;
        if (!(f >> r.exposureUs >> r.gain >> r.h_min >> ts)) {
            std::cerr << "[Calibration] Settings file malformed.\n";
            return std::nullopt;
        }

        bool valid = r.exposureUs >  0
                  && r.exposureUs <= 1000000
                  && r.gain       >= 1.0f
                  && r.gain       <= 16.0f
                  && r.h_min      >  0.0
                  && r.h_min      <= 8.0;

        if (!valid) {
            std::cerr << "[Calibration] Settings file out of range.\n";
            return std::nullopt;
        }

        auto now = std::chrono::duration_cast<std::chrono::seconds>(
                       std::chrono::system_clock::now().time_since_epoch()).count();
        if (now - ts > 86400)
            std::cout << "[Calibration] Warning: cached settings are "
                      << (now - ts) / 3600 << " hours old.\n";

        return r;
    }
};
