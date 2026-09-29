#include "CommunityV1Calibration.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double minimumSpan = 10.0;
constexpr double maxFrameGapSeconds = 2.0;
constexpr double settleSeconds = 0.18;
constexpr double settleBandRatio = 0.005;
constexpr double nearbyEndpointRatio = 0.10;
constexpr double otherEndpointFreshSeconds = 15.0;
constexpr double evidenceLifetimeSeconds = 300.0;
constexpr double turnExcursionRatio = 0.12;
constexpr double fullExcursionRatio = 0.75;
constexpr double narrowExcursionRatio = 0.10;
constexpr double narrowEvidenceSeconds = 30.0;
constexpr double rangeStaleSeconds = 180.0;
constexpr double outputSmoothingSeconds = 0.25;

double seconds(CommunityV1Calibration::Clock::time_point a,
               CommunityV1Calibration::Clock::time_point b) {
    return std::chrono::duration<double>(a - b).count();
}

double median(std::array<double, 5> samples) {
    std::sort(samples.begin(), samples.end());
    return samples[2];
}
}

void CommunityV1Calibration::forgetObservations(Channel &c) {
    c.sampleCount = c.sampleIndex = 0;
    c.hasVelocity = false;
    c.hasMotion = false;
    c.velocity = 0;
    c.hasSettledReference = c.settledApplied = c.hasTurn = c.turnSettled = false;
    c.lowAnchored = c.highAnchored = c.hasNarrowPair = false;
    c.low = {};
    c.high = {};
    c.direction = 0;
}

void CommunityV1Calibration::reset(const Values &low, const Values &high) {
    for (std::size_t i = 0; i < channelCount; ++i) {
        auto &c = channels[i];
        c = Channel{};
        c.workingLow = c.targetLow = std::isfinite(low[i]) ? low[i] : 0.0;
        c.workingHigh = c.targetHigh = std::isfinite(high[i]) ? high[i] : c.workingLow;
        c.usable = std::isfinite(low[i]) && std::isfinite(high[i]) &&
                   high[i] - low[i] >= minimumSpan;
    }
    initialized = true;
    hasFrameTime = false;
}

void CommunityV1Calibration::recordTurn(Channel &c, bool low, double value, Clock::time_point now) {
    auto &e = low ? c.low : c.high;
    const double span = c.targetHigh - c.targetLow;
    // Separate visits must agree. A persistent jump can be relearned, but a
    // single excursion or a plateau alone cannot establish a different gain.
    if (e.count && seconds(now, e.at) <= evidenceLifetimeSeconds &&
        std::abs(e.value - value) <= 0.15 * span) {
        e.count = std::min(e.count + 1, 100u);
    } else {
        e.count = 1;
        c.hasNarrowPair = false;
    }
    e.value = value;
    e.at = now;
    if (!c.low.count || !c.high.count) return;
    if (seconds(now, c.low.at) > evidenceLifetimeSeconds ||
        seconds(now, c.high.at) > evidenceLifetimeSeconds) {
        c.low.count = c.high.count = 0;
        c.hasNarrowPair = false;
        return;
    }

    const double observedSpan = c.high.value - c.low.value;
    if (observedSpan < minimumSpan || observedSpan < narrowExcursionRatio * span) {
        c.hasNarrowPair = false;
        return;
    }
    bool accept;
    if (observedSpan >= fullExcursionRatio * span) {
        const unsigned visits = observedSpan > 2.0 * span ? 3u : 2u;
        accept = c.low.count >= visits && c.high.count >= visits;
        c.hasNarrowPair = false;
    } else {
        // A smaller gesture and reduced sensor gain are not identifiable from
        // raw data alone. Only relearn a narrow range after the old range has
        // gone unused for minutes AND stable, repeated excursions persist.
        if (!c.hasNarrowPair) {
            c.hasNarrowPair = true;
            c.narrowPairSince = now;
        }
        accept = c.low.count >= 4 && c.high.count >= 4 &&
                 seconds(now, c.narrowPairSince) >= narrowEvidenceSeconds &&
                 seconds(now, c.rangeConfirmedAt) >= rangeStaleSeconds;
    }
    if (accept) {
        c.targetLow = c.low.value;
        c.targetHigh = c.high.value;
        c.lowAnchored = c.highAnchored = true;
        c.lowAnchoredAt = c.low.at;
        c.highAnchoredAt = c.high.at;
        c.rangeConfirmedAt = now;
        c.hasNarrowPair = false;
        c.low.count = c.high.count = 0;
    }
}

CommunityV1Calibration::Bounds CommunityV1Calibration::updateBounds(const Values &raw, Clock::time_point now) {
    Bounds bounds{};
    if (!initialized) return bounds;
    double dt = hasFrameTime ? seconds(now, lastFrameTime) : 0.0;
    const bool advance = !hasFrameTime || dt > 0.0;
    if (advance) {
        lastFrameTime = now;
        hasFrameTime = true;
    }
    const bool gap = dt > maxFrameGapSeconds;
    if (gap) dt = 0.0;
    const double smooth = 1.0 - std::exp(-std::max(0.0, dt) / outputSmoothingSeconds);
    for (std::size_t i = 0; i < channelCount; ++i) {
        auto &c = channels[i];
        if (!c.hasRangeTime) {
            c.rangeConfirmedAt = now;
            c.hasRangeTime = true;
        }
        if (advance && (gap || !std::isfinite(raw[i]))) {
            forgetObservations(c);
            c.targetLow = c.workingLow;
            c.targetHigh = c.workingHigh;
        }
        if (advance && c.usable && std::isfinite(raw[i])) {
            c.samples[c.sampleIndex] = raw[i];
            c.sampleIndex = (c.sampleIndex + 1) % c.samples.size();
            c.sampleCount = std::min(c.sampleCount + 1, c.samples.size());
            if (c.sampleCount == c.samples.size()) {
                const double value = median(c.samples);
                const double span = c.targetHigh - c.targetLow;
                const double quietBand = std::max(1.0, settleBandRatio * span);
                if (!c.hasVelocity) c.observationsStartedAt = now;
                if (c.hasVelocity && dt > 0.0)
                    c.velocity += (1.0 - std::exp(-dt / 0.12)) * ((value - c.previousFiltered) / dt - c.velocity);
                c.previousFiltered = value;
                c.hasVelocity = true;
                // A slow gesture can remain inside the quiet band for many
                // frames. Reject its sustained slope instead of chasing it
                // across the entire range as if it were thermal offset.
                const bool moving = std::abs(c.velocity) > std::max(3.0, span * 0.003);
                if (moving) {
                    c.hasMotion = true;
                    c.lastMotionAt = now;
                }
                if (!c.hasSettledReference || moving || std::abs(value - c.settledReference) > quietBand) {
                    c.settledReference = value;
                    c.settledSince = now;
                    c.settledApplied = false;
                    c.hasSettledReference = true;
                }
                const bool settled = seconds(now, c.settledSince) >= settleSeconds;
                if (settled) {
                    // A static middle pose drifting into an old endpoint must
                    // not be relabelled as open/closed. Require recent motion,
                    // an endpoint at startup, or continuity of that same end.
                    const bool recentArrival = seconds(now, c.observationsStartedAt) < 1.0 ||
                        (c.hasMotion && seconds(now, c.lastMotionAt) < 3.0);
                    const bool nearLow = std::abs(value - c.targetLow) <= nearbyEndpointRatio * span &&
                        (recentArrival || (c.lowAnchored && seconds(now, c.lowAnchoredAt) < 2.0));
                    const bool nearHigh = std::abs(value - c.targetHigh) <= nearbyEndpointRatio * span &&
                        (recentArrival || (c.highAnchored && seconds(now, c.highAnchoredAt) < 2.0));
                    const double gain = c.settledApplied ? 1.0 - std::exp(-dt / 3.0) : 0.85;
                    if (nearLow && !nearHigh) {
                        double change = gain * (value - c.targetLow);
                        if (!c.highAnchored || seconds(now, c.highAnchoredAt) > otherEndpointFreshSeconds) {
                            c.targetHigh += change; // only one end known: track offset, hold gain
                            // Transport earlier observations by the same offset;
                            // otherwise sparse visits mix different thermal states.
                            c.low.value += change;
                            c.high.value += change;
                        } else {
                            change = std::min(change, span - minimumSpan);
                        }
                        c.targetLow += change;
                        c.lowAnchored = true;
                        c.lowAnchoredAt = now;
                        c.settledApplied = true;
                    } else if (nearHigh && !nearLow) {
                        double change = gain * (value - c.targetHigh);
                        if (!c.lowAnchored || seconds(now, c.lowAnchoredAt) > otherEndpointFreshSeconds) {
                            c.targetLow += change;
                            c.low.value += change;
                            c.high.value += change;
                        } else {
                            change = std::max(change, minimumSpan - span);
                        }
                        c.targetHigh += change;
                        c.highAnchored = true;
                        c.highAnchoredAt = now;
                        c.settledApplied = true;
                    }
                }

                if (!c.hasTurn) {
                    c.hasTurn = true;
                    c.turnValue = value;
                    c.direction = 0;
                    c.turnSettled = settled;
                    c.turnStartedAt = now;
                } else {
                    const double distance = std::max(2.0, turnExcursionRatio * span);
                    const double delta = value - c.turnValue;
                    if ((c.direction >= 0 && delta < -distance) ||
                        (c.direction <= 0 && delta > distance)) {
                        const bool low = delta > 0.0;
                        // Smooth movements need not stop at either end. A
                        // sustained leg plus a real reversal is also evidence;
                        // short sensor spikes cannot confirm the opposite end.
                        if (c.turnSettled || (c.direction != 0 && seconds(now, c.turnStartedAt) >= 0.35))
                            recordTurn(c, low, c.turnValue, now);
                        c.direction = low ? 1 : -1;
                        c.turnValue = value;
                        c.turnSettled = false;
                        c.turnStartedAt = now;
                    } else if ((c.direction > 0 && value > c.turnValue) ||
                               (c.direction < 0 && value < c.turnValue)) {
                        c.turnValue = value;
                        c.turnSettled = false;
                    }
                    if (settled && std::abs(value - c.turnValue) <= quietBand) {
                        c.turnSettled = true;
                        // Retain the current plateau, not the oldest sample
                        // from a long hold before its temperature changed.
                        c.turnValue = value;
                    }
                }
            }
            c.workingLow += smooth * (c.targetLow - c.workingLow);
            c.workingHigh += smooth * (c.targetHigh - c.workingHigh);
        }
        bounds.low[i] = c.workingLow;
        bounds.high[i] = c.workingHigh;
    }
    return bounds;
}

CommunityV1Calibration::Values CommunityV1Calibration::update(const Values &raw, Clock::time_point now) {
    Values normalized{};
    const auto bounds = updateBounds(raw, now);
    for (std::size_t i = 0; i < channelCount; ++i) {
        const double span = bounds.high[i] - bounds.low[i];
        if (span >= minimumSpan && std::isfinite(raw[i]))
            normalized[i] = (raw[i] - bounds.low[i]) / span * 100.0;
    }
    return normalized;
}
