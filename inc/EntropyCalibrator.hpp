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
    double      min;
    double      max;
    double      current;
    double      step;
    double      best = 0.0;
    int         confirmCount;

    const libcamera::ControlId* id{};
    std::function<void(libcamera::ControlList&, double)> apply;
};


template<typename T>
SweepParam makeSweepParam(const libcamera::Control<T>* ctrl) {
    SweepParam p;
    p.id    = ctrl;
    p.apply = [ctrl](libcamera::ControlList& controls, double current) {
        controls.set(*ctrl, static_cast<T>(current));
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
                    const OutputFlags                    _flags,
                      std::ofstream*                     _logstream)
        : camera(cam), stream(strm), flags(_flags), logStream(_logstream) {}

    Estimator::Result run(FrameCallback frameCallback,
               uint32_t      timeout_ms      = 180000,
               bool          force_calibrate = false)
    {
        buildSweepList();
        validateSweepList();

        // check for saved settings, and if they are still valid. 
        // to avoid having to recalibrate every time.
        if (!force_calibrate) {
            double h_min = 0.0;
            if (loadSettings(h_min)) {
                tee() << "[Calibration] Cached settings found, validating\n";
                Estimator::Result checked = measure(frameCallback, QUICK_FRAMES, Estimator::Estimate_Markov);
                double checked_h   = checked.h_min;
                double degradation = h_min > 0.0 ? (h_min - checked_h) / h_min : 1.0;
                tee() << "   Checked H_min=" << std::fixed << std::setprecision(4)
                      << checked_h << "  degradation=" << degradation * 100.0 << "%\n";
                if (degradation <= RECHECK_TOLERANCE) { 
                    tee() << "[Calibration] Cache still valid, skipping sweep.\n";
                    return checked;
                }
                tee() << "[Calibration] Cache degraded, re-calibrating.\n";
            } else {
                tee() << "[Calibration] No cache found, running full calibration.\n";
            }
        }
        
        return fullCalibration(frameCallback, timeout_ms);
    }

private:
    SweepList                          sweepList; // list of parameters to sweep through, ordered by expected impact on entropy (least -> most)
    std::shared_ptr<libcamera::Camera> camera;
    libcamera::Stream*                 stream;
    const OutputFlags                  flags;
    std::ostream*                      logStream;

    // Write to stdout and also write a plain-text copy to logStream.
    struct Tee {
        std::ostream* log;
        template<typename T>
        Tee& operator<<(const T& v) { std::cout << v; if (log) *log << v; return *this; }
        Tee& operator<<(std::ostream& (*f)(std::ostream&)) {
            f(std::cout); if (log) f(*log); return *this;
        }
    };
    Tee tee() { return Tee{logStream}; }

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
                                // Runtime type handling
                switch (p.id->type()) {

                    case libcamera::ControlTypeInteger32:
                        p.min = static_cast<double>(bounds.min().get<int32_t>());
                        p.max = static_cast<double>(bounds.max().get<int32_t>());
                        break;

                    case libcamera::ControlTypeFloat:
                        p.min = static_cast<double>(bounds.min().get<float>());
                        p.max = static_cast<double>(bounds.max().get<float>());
                        break;

                    default:
                        break;
                }

                if (p.min == p.max) { // Check control has a range of values to sweep through
                    tee() << "Parameter " << p.id->name() << " has no range, skipping.\n";
                    remove = true;
                } else {
                    p.step    = (p.max - p.min) / PLATEAU_STEP_START;
                    p.current = p.max;
                    p.best    = p.max;
                }
            } catch (const std::exception&) {
                tee() << "Cannot find " << p.id->name() << " parameter, skipping.\n";
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
        std::string h_string = h > 0.0 ? "H_min : " + std::to_string(h) : "";
        std::cout << "\033[2K  " << h_string << " \t| Best: " << bestH << " bits/bit";
        std::cout << (bestH < 0.4 ? "  [WARNING: low entropy]" : "  [Entropy OK]") << "\n";
        std::cout << "\033[2K  Ran for: " << elapsed << "s\n" << std::flush;

        // Log file: plain structured line per step
        if (logStream) {
            *logStream << "[" << elapsed << "s] " << message;
            for (const auto& p : sweepList)
                *logStream << "  " << p.id->name() << "=" << p.current;
            if (h > 0.0) *logStream << "  H_min=" << std::fixed << std::setprecision(4) << h;
            *logStream << "  best=" << bestH
                       << (bestH < 0.4 ? "  [WARNING: low entropy]" : "  [Entropy OK]")
                       << "\n" << std::flush;
        }
    }
    // sample N frames and return the min-entropy estimate, applying the current sweep parameters
    Estimator::Result measure(FrameCallback& cb, size_t nFrames, EstimationMethod method) {
        std::vector<uint8_t> accum;
        cb(accum, nFrames, sweepList);
        if (accum.empty()) return Estimator::Result({0.0, 0.0, 0});
        return method(accum.data(), accum.size());
    }
    // Perform full calibration by sweeping through all parameters backwards, 
    // to find optimal settings that maximise min-entropy. 
    // If sensor is low ISO, Max settings will likely be best, so start there
    // and work downwards to find the point of diminishing returns for each parameter.
    // High ISO sensors might have to step down multiple parameters.
    Estimator::Result fullCalibration(FrameCallback& frameCallback, uint32_t timeout_ms) {
        using Clock = std::chrono::steady_clock;
        auto start    = Clock::now();
        auto deadline = start + std::chrono::milliseconds(timeout_ms / 2);

        double bestH = 0.0;
        
        if (flags.verbose) std::cout << "\n\n\n\n\n";

        Estimator::Result return_result;

        for (auto& p : sweepList) { // for each parameter
            bool paramDone = false;


            while (!paramDone && p.current > p.min) {

                if (Clock::now() >= deadline) {
                    tee() << "[Calibration] Timed out.\n";
                    goto CalibrateExit;
                }

                auto param_start = Clock::now();
                auto r = measure(frameCallback, SAMPLE_FRAMES, &Estimator::Estimate_Markov);
                double h = r.h_min;

                if (h > bestH) { // new best parameter setup
                    bestH     = h;
                    p.best    = p.current;
                    p.current -= p.step;
                    return_result = r;

                } else {
                    
                    // if steps away from best == PLATEAU_CONFIRM_STEPS, then it has plateaued
                    // so go back to best and decrease step.
                    if (p.current >= p.best - (p.step * PLATEAU_CONFIRM_STEPS)) {
                        p.current -= p.step;

                    } else {

                        // step back to the best known point, and reduce step size for finer search
                        p.current += p.step * PLATEAU_STEP_MIN_SCALE;
                        p.step    /= PLATEAU_STEP_REDUCTION;

                        if (p.step < (p.max - p.min) / PLATEAU_STEP_MIN_SCALE) { // if sweep converged
                            tee() << "Parameter " << p.id->name()
                                  << " plateaued at " << p.best << ".\n";
                            p.current = p.best;
                            paramDone = true;
                        }
                    }
                }
                if (flags.verbose) outputProgress("Calibrating...", param_start, h, bestH);
                if (bestH >= EARLY_EXIT_H) goto CalibrateExit;
            }
        }

CalibrateExit:
        outputProgress("Calibration complete.", start, 0.0, bestH);
        saveSettings(bestH);
        return return_result;
    }

    void saveSettings(double bestH) {
        std::ofstream f(SETTINGS_PATH);
        if (!f.is_open()) {
            tee() << "[Calibration] Could not write to " << SETTINGS_PATH << "\n";
            return;
        }
        tee() << "[Calibration] Saving settings to " << SETTINGS_PATH
              << "  H_min=" << std::fixed << std::setprecision(4) << bestH << "\n";
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

        // Read settings from file. Format is "<parameter_name> <value>", one per line.
        while (std::getline(f, line)) {
            std::istringstream iss(line);
            std::string key;
            double      value;
            if (!(iss >> key >> value)) {
                tee() << "[Calibration] Settings file malformed.\n";
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
                tee() << "[Calibration] Unknown key in settings: " << key << "\n";
                return false;
            }
        }
        // check values are in bounds
        for (const auto& p : sweepList) {
            if (p.best < p.min || p.best > p.max) {
                tee() << "[Calibration] Parameter " << p.id->name()
                      << " out of bounds in settings file.\n";
                return false;
            }
        }

        auto now = std::chrono::duration_cast<std::chrono::seconds>(
                       std::chrono::system_clock::now().time_since_epoch()).count();
        if (ts > 0 && now - ts > 86400)
            tee() << "[Calibration] Warning: cached settings are "
                  << (now - ts) / 3600 << " hours old.\n";

        return true;
    }
};