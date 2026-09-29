#include <AR1Linear.hpp>
#include <OfficialCalibration.h>

#include <array>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>

ImportAR1Linear(AR1LinearAA)
ImportAR1Linear(AR1LinearAD)
ImportAR1Linear(AR1LinearAE)
ImportAR1Linear(AR1LinearAG)

static bool matches(const std::vector<double> &actual, const std::array<double, 9> &expected) {
    constexpr std::array<std::size_t, 9> indices{0, 1, 2, 3, 7, 11, 15, 19, 20};
    if (actual.size() != 24) return false;
    for (std::size_t i = 0; i < indices.size(); ++i) {
        if (std::abs(actual[indices[i]] - expected[i]) > 1e-6) return false;
    }
    return actual[23] == -1.0;
}

int main(int argc, char **argv) {
    // Reference values obtained from the installed official v0.1.8.6 AR1_linear.dll.
    // These cover thumb pitch/yaw/roll as well as the index, ring and little splay.
    const std::vector<double> motion{0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110};
    const std::vector<double> spread{0, 0, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0};
    const std::array<AR1Linear *, 4> models{
        AR1LinearAA().get(), AR1LinearAD().get(), AR1LinearAE().get(), AR1LinearAG().get()
    };
    const std::array<std::array<double, 9>, 4> motionExpected{{
        {{0, -51.2, 23.5, -26.5, 0, 0, 0, 0, 6.75}},
        {{-4.5, 0, 0, 5, 0, 0, 0, 0, 15}},
        {{0, 0, 4.3, 15.7, 0, 0, 0, 0, -2.85}},
        {{0, 0, 10, 15, 0, 0, 0, 0, 1}}
    }};
    const std::array<std::array<double, 9>, 4> spreadExpected{{
        {{0, -37.8, 10, -21, 23, 0.24, 13, 35, 0}},
        {{0, 0, 0, 5, 23, 0.24, 13, 35, 10}},
        {{0, 0, 10, 10, 21, -0.72, 15, 35, 0}},
        {{0, -25, 10, 15, 24, 0.96, 15, 35, 1}}
    }};
    for (std::size_t i = 0; i < models.size(); ++i) {
        if (!matches(models[i]->sensor2Angle(motion, {}), motionExpected[i]) ||
            !matches(models[i]->sensor2Angle(spread, {}), spreadExpected[i])) {
            std::cerr << "AR1 model " << i << " differs from the official driver\n";
            return 1;
        }
    }

    if (argc > 1) {
        std::ifstream fixtures(argv[1]);
        if (!fixtures) return 2;
        std::size_t count = 0;
        int model;
        double largest = 0;
        while (fixtures >> model) {
            if (model < 0 || model >= 4) return 2;
            std::vector<double> input(12), expected(24);
            for (auto &value : input) if (!(fixtures >> value)) return 2;
            for (auto &value : expected) if (!(fixtures >> value)) return 2;
            const auto actual = models[model]->sensor2Angle(input, {});
            if (actual.size() != expected.size()) return 1;
            for (std::size_t joint = 0; joint < expected.size(); ++joint) {
                const double error = std::abs(actual[joint] - expected[joint]);
                largest = std::max(largest, error);
                if (!std::isfinite(error) || error > 1e-6) {
                    std::cerr << "Sample " << count << " model " << model << " channel " << joint
                              << ": actual " << actual[joint] << ", official " << expected[joint] << '\n';
                    return 1;
                }
            }
            ++count;
        }
        if (!count) return 2;
        std::cout << "Compared " << count << " complete official poses; max error " << largest << '\n';
    }

    double raw[12]{};
    double low[12]{};
    double high[12]{};
    for (int i = 0; i < 12; ++i) {
        raw[i] = 500;
        low[i] = 100;
        high[i] = 900;
    }
    high[1] = 105; // Official weak-channel rule: fewer than 10 raw units.
    high[2] = 90;  // Reversed endpoints are disabled.
    const auto normalized = udCapV1HandAutoCali(raw, low, high, 100);
    if (normalized.size() != 12 || normalized[0] != 50 || normalized[1] != 0 || normalized[2] != 0) {
        std::cerr << "Official calibration normalization differs\n";
        return 1;
    }
    return 0;
}
