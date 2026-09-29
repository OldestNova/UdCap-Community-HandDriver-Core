#pragma once
#include <array>
#include <cmath>
#include <stdexcept>

// Channels use the CMD_ANGLE indices. Thumb: distal, intermediate, proximal
// pitch, proximal yaw; then DIP/PIP/MCP/splay for each finger; thumb roll at 20.
// Middle splay (11) has only a constant offset, matching official v0.1.8.6.
struct HandResponseCurve {
    double outputMin = 0;
    double outputMax = 0;
    double balance = 0.5;
    double offset = 0; // Degrees, applied after the curve, before protocol axes.
};
using HandResponseSettings = std::array<HandResponseCurve, 21>;

inline constexpr std::array<double, 21> handResponseMin{
    -60,-60,-65,-20, -80,-100,-80,0, -80,-100,-80,0,
    -80,-100,-80,0, -74,-92,-75,0,0};
inline constexpr std::array<double, 21> handResponseMax{
    0,0,0,15, 0,0,0,25, 0,0,0,0, 0,0,0,15, 0,0,0,35,47};

inline HandResponseSettings defaultHandResponse() {
    HandResponseSettings settings{};
    for (std::size_t i = 0; i < settings.size(); ++i)
        settings[i] = {handResponseMin[i], handResponseMax[i], 0.5, 0};
    return settings;
}

inline void validateHandResponse(const HandResponseSettings &settings) {
    for (std::size_t i = 0; i < settings.size(); ++i) {
        const auto &c = settings[i];
        if (!std::isfinite(c.outputMin) || !std::isfinite(c.outputMax) ||
            !std::isfinite(c.balance) || !std::isfinite(c.offset) ||
            c.outputMin < -180 || c.outputMax > 180 || c.outputMin > c.outputMax ||
            c.balance < 0 || c.balance > 1 || std::abs(c.offset) > 45 ||
            (i == 11 && (c.outputMin != 0 || c.outputMax != 0 || c.balance != 0.5)))
            throw std::invalid_argument("Invalid hand response curve");
    }
}

inline double evaluateHandResponse(double value, double inputMin, double inputMax,
                                    const HandResponseCurve &c) {
    // Preserve default identity exactly; the official algorithm otherwise uses
    // two linear segments and extrapolates outside the calibrated endpoints.
    if (std::abs(inputMax - inputMin) < 10) return 0;
    if (c.balance == 0.5 && c.outputMin == inputMin && c.outputMax == inputMax)
        return value + c.offset;
    const double balance = inputMin < 0 ? 1 - c.balance : c.balance;
    const double mid = (inputMin + inputMax) * 0.5;
    const double outMid = c.outputMin + (c.outputMax - c.outputMin) * balance;
    return c.offset + (value <= mid
        ? c.outputMin + (value - inputMin) * (outMid - c.outputMin) / (mid - inputMin)
        : outMid + (value - mid) * (c.outputMax - outMid) / (inputMax - mid));
}

inline std::array<double, 28> applyHandResponse(std::array<double, 28> angles,
                                               const HandResponseSettings &settings) {
    for (std::size_t i = 0; i < settings.size(); ++i)
        angles[i] = i == 11 ? angles[i] + settings[i].offset
            : evaluateHandResponse(angles[i], handResponseMin[i], handResponseMax[i], settings[i]);
    return angles;
}
