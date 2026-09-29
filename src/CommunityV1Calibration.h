#ifndef UDCAP_COMMUNITY_V1_CALIBRATION_H
#define UDCAP_COMMUNITY_V1_CALIBRATION_H

#include <array>
#include <chrono>
#include <cstddef>

// Online endpoint/offset tracking for the twelve raw finger channels. No gesture
// sequence or initial-midpoint constraint. Unobserved gain is held, never inferred
// by decaying bounds while the hand is stationary.
class CommunityV1Calibration {
public:
    static constexpr std::size_t channelCount = 12;
    using Values = std::array<double, channelCount>;
    using Clock = std::chrono::steady_clock;
    struct Bounds {
        Values low{};
        Values high{};
    };

    void reset(const Values &low, const Values &high);
    // Refreshes observed extrema while retaining the official calibration's
    // twelve-channel low/high convention for downstream normalization.
    Bounds updateBounds(const Values &raw, Clock::time_point now = Clock::now());
    // Kept for callers that use the standalone normalization result.
    Values update(const Values &raw, Clock::time_point now = Clock::now());

private:
    struct Channel {
        double workingLow = 0;
        double workingHigh = 0;
        double targetLow = 0;
        double targetHigh = 0;
        bool usable = false;
        std::array<double, 5> samples{};
        std::size_t sampleCount = 0;
        std::size_t sampleIndex = 0;
        bool hasVelocity = false;
        double previousFiltered = 0;
        double velocity = 0;
        Clock::time_point observationsStartedAt{}, lastMotionAt{};
        bool hasMotion = false;
        bool hasSettledReference = false;
        double settledReference = 0;
        Clock::time_point settledSince{};
        bool settledApplied = false;
        bool lowAnchored = false;
        bool highAnchored = false;
        Clock::time_point lowAnchoredAt{}, highAnchoredAt{};
        // Alternating excursions confirm real extrema. A plateau can update a
        // nearby endpoint sooner, but cannot by itself establish a new range.
        bool hasTurn = false;
        int direction = 0;
        double turnValue = 0;
        bool turnSettled = false;
        Clock::time_point turnStartedAt{};
        struct Evidence {
            double value = 0;
            unsigned count = 0;
            Clock::time_point at{};
        } low, high;
        bool hasNarrowPair = false;
        Clock::time_point narrowPairSince{};
        bool hasRangeTime = false;
        Clock::time_point rangeConfirmedAt{};
    };

    static void forgetObservations(Channel &channel);
    static void recordTurn(Channel &channel, bool low, double value, Clock::time_point now);

    std::array<Channel, channelCount> channels{};
    bool initialized = false;
    bool hasFrameTime = false;
    Clock::time_point lastFrameTime{};
};

#endif
