#include "OfficialCalibration.h"

std::vector<double> udCapV1HandAutoCali(const double raw[], const double low[],
                                        const double high[], float scale) {
    std::vector<double> result(12);
    for (int i = 0; i < 12; ++i) {
        const double span = high[i] - low[i];
        if (span < 10.0) {
            result[i] = 0.0;
        } else {
            result[i] = (raw[i] - low[i]) / span * static_cast<double>(scale);
        }
    }
    return result;
}
