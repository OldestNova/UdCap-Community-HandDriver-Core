#ifndef UDCAP_OFFICIAL_CALIBRATION_H
#define UDCAP_OFFICIAL_CALIBRATION_H

#include <vector>

// The current official driver disables a channel when its captured range is
// smaller than 10 raw units, then uses a 0..100 scale at the captured endpoints.
std::vector<double> udCapV1HandAutoCali(const double raw[], const double low[],
                                        const double high[], float scale);

#endif
