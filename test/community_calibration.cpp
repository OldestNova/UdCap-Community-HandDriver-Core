#include <CommunityV1Calibration.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

using Calibration = CommunityV1Calibration;
using Time = Calibration::Clock::time_point;
using Values = Calibration::Values;

static Values filled(double value) {
    Values result{};
    result.fill(value);
    return result;
}

struct Simulation {
    Calibration calibration;
    Time now{};
    Calibration::Bounds bounds{};
    Simulation() { calibration.reset(filled(100), filled(900)); }
    void step(const Values &raw, double dt = 0.02) {
        now += std::chrono::duration_cast<Calibration::Clock::duration>(std::chrono::duration<double>(dt));
        bounds = calibration.updateBounds(raw, now);
    }
    void hold(double value, double duration) {
        for (int n = 0; n < std::lround(duration * 50); ++n) step(filled(value));
    }
    double normalized(double value, int channel = 0) const {
        return (value - bounds.low[channel]) / (bounds.high[channel] - bounds.low[channel]) * 100.0;
    }
    double endpointError(double low, double high, int channel = 0) const {
        return std::max(std::abs(normalized(low, channel)), std::abs(normalized(high, channel) - 100.0));
    }
};

static double gesture(double t, double period = 8.0) {
    const double phase = std::fmod(t, period) / period;
    if (phase < 0.25) return 0;
    if (phase < 0.5) return (phase - 0.25) * 4;
    if (phase < 0.75) return 1;
    return (1 - phase) * 4;
}

static int failures = 0;
static void check(const std::string &name, double error, double limit) {
    std::cout << name << ": " << error << " (limit " << limit << ")\n";
    if (!std::isfinite(error) || error > limit) ++failures;
}

int main() {
    // Continuous frames, not a time jump followed by a handful of endpoint
    // samples. Independent signs, phases and gains exercise all 12 channels.
    Simulation continuous;
    double last20MinutesError = 0;
    for (int n = 1; n <= 3600 * 50; ++n) {
        const double t = n / 50.0;
        Values raw{}, low{}, high{};
        for (int c = 0; c < 12; ++c) {
            const double shift = (c % 2 ? -1 : 1) * 1200 * t / 3600;
            const double finalGain = c % 3 == 0 ? 0.3 : c % 3 == 1 ? 2.0 : 1.0;
            const double span = 800 * (1 + (finalGain - 1) * t / 3600);
            low[c] = 500 + shift - span / 2;
            high[c] = low[c] + span;
            raw[c] = low[c] + span * gesture(t + c * 0.4) + 0.3 * std::sin(t * 13 + c);
        }
        continuous.step(raw);
        if (t > 2400)
            for (int c = 0; c < 12; ++c)
                last20MinutesError = std::max(last20MinutesError, continuous.endpointError(low[c], high[c], c));
    }
    check("60 min offset +/-1200, gain 0.3..2: final 20 min worst endpoint %", last20MinutesError, 3.0);

    Simulation noPause;
    double noPauseError = 0;
    for (int n = 1; n <= 3600 * 50; ++n) {
        const double t = n / 50.0, shift = -t / 3;
        const double span = 800 - 500 * t / 3600;
        const double low = 500 + shift - span / 2;
        noPause.step(filled(low + span * (0.5 - 0.5 * std::cos(t * 0.7853981633974483))));
        if (t > 2400) noPauseError = std::max(noPauseError, noPause.endpointError(low, low + span));
    }
    check("60 min continuous movement with no endpoint pauses: endpoint %", noPauseError, 3.0);

    Simulation sparse;
    double sparseError = 0;
    for (int n = 1; n <= 3600 * 50; ++n) {
        const double t = n / 50.0, shift = t / 6;
        sparse.step(filled(100 + shift + 800 * gesture(t, 240)));
        if (t > 2400 && (std::fmod(t, 240) < 55 || (std::fmod(t, 240) > 125 && std::fmod(t, 240) < 175)))
            sparseError = std::max(sparseError, sparse.endpointError(100 + shift, 900 + shift));
    }
    check("60 min, opposite endpoints 120 s apart: endpoint %", sparseError, 4.0);

    // The hand returns to open but does not stop at the other endpoint. A
    // smooth partial excursion cannot falsely confirm a new high plateau.
    Simulation oneEnd;
    double oneEndError = 0;
    for (int n = 1; n <= 3600 * 50; ++n) {
        const double t = n / 50.0, phase = std::fmod(t, 10);
        const double pose = phase < 2 ? 0 : 0.5 * std::sin((phase - 2) / 8 * 3.141592653589793);
        const double shift = t / 3;
        oneEnd.step(filled(100 + shift + 800 * pose));
        if (t > 2400 && phase > 1 && phase < 2)
            oneEndError = std::max(oneEndError, std::abs(oneEnd.normalized(100 + shift)));
    }
    check("60 min open/partial motion without closed hold: open error %", oneEndError, 2.0);

    // A stable center plus realistic small noise must not collapse a range.
    Simulation stationary;
    for (int n = 0; n < 3600 * 50; ++n)
        stationary.step(filled(500 + 2 * std::sin(n * 0.11)));
    check("60 min stationary noise: bound change", std::max(std::abs(stationary.bounds.low[0] - 100),
        std::abs(stationary.bounds.high[0] - 900)), 1e-8);

    Simulation heldCenter, heldOpen;
    for (int n = 1; n <= 3600 * 50; ++n) {
        const double shift = n / 150.0;
        heldCenter.step(filled(500 + shift));
        heldOpen.step(filled(100 + shift));
    }
    check("Static middle slowly drifts through old high: no false endpoint",
        heldCenter.endpointError(100, 900), 1e-8);
    check("Known open hold with 60 min continuous offset: endpoint %",
        heldOpen.endpointError(1300, 2100), 1.0);

    Simulation partial;
    for (int n = 0; n < 120 * 50; ++n) partial.step(filled(300 + 400 * gesture(n / 50.0)));
    check("2 min partial gestures: bound change", partial.endpointError(100, 900), 1e-8);

    for (const auto &ends : {std::pair<double,double>{700,1500}, {-500,300}, {-400,2000}, {300,700}}) {
        Simulation resumed;
        resumed.hold(500, 1200); // genuine hold, continuous input, twenty minutes
        const double duration = ends.second - ends.first < 600 ? 48 : 32;
        for (int n = 0; n < duration * 50; ++n)
            resumed.step(filled(ends.first + (ends.second - ends.first) * gesture(n / 50.0)));
        check("After 20 min hold: recover [" + std::to_string(ends.first) + "," +
              std::to_string(ends.second) + "] endpoint %", resumed.endpointError(ends.first, ends.second), 2.0);
    }

    Simulation spike;
    spike.hold(500, 3);
    for (int n = 0; n < 100; ++n) {
        spike.hold(3000, 0.06); // beyond median rejection, but too brief to settle
        spike.hold(500, 0.4);
    }
    check("Repeated 60 ms spikes: bound change", spike.endpointError(100, 900), 1e-8);
    spike.step(filled(std::numeric_limits<double>::quiet_NaN()));
    spike.step(filled(std::numeric_limits<double>::infinity()));
    spike.hold(500, 1);
    check("Non-finite frames: bound change", spike.endpointError(100, 900), 1e-8);

    Simulation gap;
    gap.hold(150, 0.35); // leave a pending smoothed update
    const auto held = gap.bounds;
    gap.step(filled(950), 1800);
    check("Disconnect gap must not integrate offline time",
        std::max(std::abs(gap.bounds.low[0] - held.low[0]), std::abs(gap.bounds.high[0] - held.high[0])), 0);
    const auto duplicate = gap.calibration.updateBounds(filled(3000), gap.now);
    const auto backwards = gap.calibration.updateBounds(filled(-3000), gap.now - std::chrono::seconds(1));
    check("Duplicate/backwards clock must not update",
        std::max(std::abs(duplicate.low[0] - held.low[0]), std::abs(backwards.high[0] - held.high[0])), 0);

    Calibration weak;
    auto low = filled(100), high = filled(900);
    high[0] = 105;
    high[1] = std::numeric_limits<double>::quiet_NaN();
    weak.reset(low, high);
    Values result{};
    for (int n = 0; n < 6000; ++n)
        result = weak.update(filled(100 + 800 * gesture(n / 50.0)), Time{} + std::chrono::milliseconds(n * 20));
    check("Weak/invalid channel stays disabled", std::max(std::abs(result[0]), std::abs(result[1])), 0);

    for (int fps : {20, 50, 100, 200}) {
        Simulation rate;
        for (int n = 0; n < 20 * fps; ++n) rate.step(filled(150 + 800 * gesture(n / double(fps))), 1.0 / fps);
        check("Shift recovery at " + std::to_string(fps) + " FPS %", rate.endpointError(150, 950), 1.0);
    }
    return failures ? 1 : 0;
}
