#pragma once
#include <cstdint>
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <string>
#include <optional>
#include <functional>
#include <memory>
#include <libcamera/libcamera.h>
#include "Estimator.hpp"
#include "OutputPort.hpp"

struct SweepParam {
    double      min{};
    double      max{};
    double      current{};
    double      step{};
    double      best{};
    int         confirmCount{};

    const libcamera::ControlId*                          id{};
    std::function<void(libcamera::ControlList&, double)> apply;
};

template<typename T>
SweepParam makeSweepParam(const libcamera::Control<T>* ctrl) {
    SweepParam p;
    p.id    = ctrl;
    p.apply = [ctrl](libcamera::ControlList& controls, double val) {
        controls.set(*ctrl, static_cast<T>(val));
    };
    return p;
}

using SweepList        = std::vector<SweepParam>;
using FrameCallback    = std::function<void(std::vector<uint8_t>&, size_t, SweepList&)>;
using EstimationMethod = std::function<Estimator::Result(const uint8_t*, size_t)>;

class EntropyCalibrator {
public:
    int PLATEAU_STEP_START      = 4;
    int PLATEAU_STEP_REDUCTION  = 4;
    int PLATEAU_STEP_MIN_SCALE  = 64;
    int PLATEAU_CONFIRM_STEPS   = 2;

    static constexpr size_t  SAMPLE_FRAMES     = 6;
    static constexpr size_t  QUICK_FRAMES      = 3;
    static constexpr double  EARLY_EXIT_H      = 7.5;
    static constexpr double  RECHECK_TOLERANCE = 0.10;
    static constexpr const char* SETTINGS_PATH = "./calibration";

    EntropyCalibrator(std::shared_ptr<libcamera::Camera> cam,
                      libcamera::Stream*                 strm,
                      OutputFlags                        _flags)
        : camera(cam), stream(strm), flags(_flags) {}

    double run(FrameCallback frameCallback,
               uint32_t      timeout_ms      = 180000,
               bool          force_calibrate = false)
    {
        // check for saved settings, and if they are still valid. 
        // to avoid having to recalibrate every time.
        if (!force_calibrate) {
            double h_min = 0.0;
            if (loadSettings(h_min)) {
                std::cout << "[Calibration] Cached settings found, validating\n";
                double checked_h   = measure(frameCallback, QUICK_FRAMES, Estimator::Estimate_Markov);
                double degradation = h_min > 0.0 ? (h_min - checked_h) / h_min : 1.0;
                std::cout << "  Checked H_min=" << std::fixed << std::setprecision(4)
                          << checked_h << "  degradation=" << degradation * 100.0 << "%\n";
                if (degradation <= RECHECK_TOLERANCE && degradation > 0.0) {
                    std::cout << "[Calibration] Cache still valid, skipping sweep.\n\n";
                    return checked_h;
                }
                std::cout << "[Calibration] Cache degraded, re-calibrating.\n";
            } else {
                std::cout << "[Calibration] No cache found, running full calibration.\n";
            }
        }
        buildSweepList();
        validateSweepList();
        return fullCalibration(frameCallback, timeout_ms);
    }

private:
    SweepList                          sweepList; // list of parameters to sweep through, ordered by expected impact on entropy (least -> most)
    std::shared_ptr<libcamera::Camera> camera;
    libcamera::Stream*                 stream;
    OutputFlags                        flags;

    // Ordered in least -> most entropy impact for reducing saturation of pixels.
    // Assuming start of sweep will be near fully saturated.
    void buildSweepList() {
        sweepList.clear();
        sweepList.push_back(makeSweepParam(
            static_cast<const libcamera::Control<float>*>(&libcamera::controls::DigitalGain)));
        sweepList.push_back(makeSweepParam(
            static_cast<const libcamera::Control<int32_t>*>(&libcamera::controls::ExposureTime)));
        sweepList.push_back(makeSweepParam(
            static_cast<const libcamera::Control<float>*>(&libcamera::controls::AnalogueGain)));
    }

    void validateSweepList() {
        const libcamera::ControlInfoMap& camControls = camera->controls();
        std::vector<size_t> toRemove;

        // Initialize sweep parameters and check control availability
        for (size_t i = 0; i < sweepList.size(); ++i) {
            auto& p      = sweepList[i];
            bool  remove = false;
            try {
                auto it = camControls.find(p.id);
                if (it == camControls.end())
                    throw std::runtime_error("Control not found");

                auto& bounds = it->second;
                p.min     = bounds.min().get<double>();
                p.max     = bounds.max().get<double>();

                if (p.min == p.max) { // Check control has a range of values to sweep through

                    std::cout << "Parameter " << p.id->name() << " has no range, skipping.\n";
                    remove = true;
                } else {
                    p.step    = (p.max - p.min) / PLATEAU_STEP_START;
                    p.current = p.max;
                    p.best    = p.max;
                }
            } catch (const std::exception&) {
                std::cout << "Cannot find " << p.id->name() << " parameter, skipping.\n";
                remove = true;
            }
            if (remove) toRemove.push_back(i);
        }
        // Remove unsupported parameters in reverse order to avoid invalidating indices
        for (auto it = toRemove.rbegin(); it != toRemove.rend(); ++it)
            sweepList.erase(sweepList.begin() + static_cast<std::ptrdiff_t>(*it));

        if (sweepList.empty())
            throw std::runtime_error("No parameters to calibrate.");
    }

    void outputProgress(const std::string& message,
                        std::chrono::steady_clock::time_point timeStarted,
                        double h     = 0.0,
                        double bestH = 0.0) {

        auto time_taken = std::chrono::steady_clock::now() - timeStarted;
        auto elapsed    = std::chrono::duration_cast<std::chrono::seconds>(time_taken).count();

        std::cout << "\033[5A\033[2K" << message << "\n";

        for (const auto& p : sweepList)
            std::cout << "\033[2K  " << p.id->name() << " : " << p.current << "\n";
        
        std::string h_string = "";
        if (h > 0.0) h_string += "H_min : " + std::to_string(h);
        
        std::cout << "\033[2K  " << h_string << " \t| Best: " << bestH << "bits/byte";
        std::cout << (bestH < 3.0 ? "  [WARNING: low entropy]" : "  [Entropy OK]") << "\n";
        std::cout << "\033[2K  Ran for: " << elapsed << "s\n" << std::flush;
    }
    // sample N frames and return the min-entropy estimate, applying the current sweep parameters
    double measure(FrameCallback& cb, size_t nFrames, EstimationMethod method) {
        std::vector<uint8_t> accum;
        cb(accum, nFrames, sweepList);
        if (accum.empty()) return 0.0;
        return method(accum.data(), accum.size()).h_min;
    }
    // Perform full calibration by sweeping through all parameters backwards, 
    // to find optimal settings that maximise min-entropy. 
    // If sensor is low ISO, Max settings will likely be best, so start there
    // and work downwards to find the point of diminishing returns for each parameter.
    // High ISO sensors might have to step down multiple parameters.
    double fullCalibration(FrameCallback& frameCallback, uint32_t timeout_ms) {
        using Clock = std::chrono::steady_clock;
        auto start    = Clock::now();
        auto deadline = start + std::chrono::milliseconds(timeout_ms / 2);

        double bestH = 0.0;
        std::cout << "\n\n\n\n\n";

        for (auto& p : sweepList) { // for each parameter
            bool paramDone = false;

            while (!paramDone && p.current > p.min) {

                if (Clock::now() >= deadline) {
                    std::cout << "[Calibration] Timed out.\n";
                    goto CalibrateExit;
                }

                double h = measure(frameCallback, SAMPLE_FRAMES, &Estimator::Estimate_Markov);

                if (h > bestH) { // new best parameter setup
                    bestH     = h;
                    p.best    = p.current;
                    p.current -= p.step;
                } else {
                    if (p.current >= p.best - p.step * PLATEAU_CONFIRM_STEPS) {
                        p.current -= p.step;
                    } else {
                        p.current += p.step * PLATEAU_STEP_MIN_SCALE;
                        p.step    /= PLATEAU_STEP_REDUCTION;
                        if (p.step < (p.max - p.min) / PLATEAU_STEP_MIN_SCALE) {
                            std::cout << "Parameter " << p.id->name()
                                      << " plateaued at " << p.best << ".\n";
                            p.current = p.best;
                            paramDone = true;
                        }
                    }
                }

                outputProgress("Calibrating...", start, h, bestH);
                if (bestH >= EARLY_EXIT_H) goto CalibrateExit;
            }
        }

CalibrateExit:
        outputProgress("Calibration complete.", start, 0.0, bestH);
        saveSettings(bestH);
        return bestH;
    }

    void saveSettings(double bestH) {
        std::ofstream f(SETTINGS_PATH);
        if (!f.is_open()) {
            std::cerr << "[Calibration] Could not write to " << SETTINGS_PATH << "\n";
            return;
        }
        auto ts = std::chrono::duration_cast<std::chrono::seconds>(
                      std::chrono::system_clock::now().time_since_epoch()).count();
        for (const auto& p : sweepList)
            f << p.id->name() << " " << p.best << "\n";
        f << "H_min "     << bestH << "\n"
          << "timestamp " << ts    << std::endl;
    }

    bool loadSettings(double& bestH) {
        std::ifstream f(SETTINGS_PATH);
        if (!f.is_open()) return false;

        int64_t ts = 0;
        std::string line;
        // read settings from file. Format is "parameter_name value", one per line.
        while (std::getline(f, line)) {
            std::istringstream iss(line);
            std::string key;
            double      value;
            if (!(iss >> key >> value)) {
                std::cerr << "[Calibration] Settings file malformed.\n";
                return false;
            }

            auto it = std::find_if(sweepList.begin(), sweepList.end(),
                                   [&](const auto& p) { return p.id->name() == key; });
            if (it != sweepList.end()) {
                it->best = value;
            } else if (key == "H_min") {
                bestH = value;
            } else if (key == "timestamp") {
                ts = static_cast<int64_t>(value);
            } else {
                std::cerr << "[Calibration] Unknown key in settings: " << key << "\n";
                return false;
            }
        }
        // check values are in bounds
        for (const auto& p : sweepList) {
            if (p.best < p.min || p.best > p.max) {
                std::cerr << "[Calibration] Parameter " << p.id->name()
                          << " out of bounds in settings file.\n";
                return false;
            }
        }

        auto now = std::chrono::duration_cast<std::chrono::seconds>(
                       std::chrono::system_clock::now().time_since_epoch()).count();
        if (ts > 0 && now - ts > 86400)
            std::cout << "[Calibration] Warning: cached settings are "
                      << (now - ts) / 3600 << " hours old.\n";

        return true;
    }
};