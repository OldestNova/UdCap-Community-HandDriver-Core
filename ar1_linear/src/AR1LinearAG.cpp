//
// Created by max_3 on 25-5-13.
//

#include "AR1Linear.hpp"
#include <string>

class AR1LinearAG : AR1Linear {
public:
    std::vector<double> sensor2Angle(std::vector<double> inputSensorData, std::vector<double> preSensorData) override {
        std::vector<double> doubleList;
        double numArray[24]{};
        for (int i = 0; i < inputSensorData.size(); ++i) {
            if (inputSensorData[i] > 120.0) {
                numArray[i] = 120.0;
            } else if (inputSensorData[i] < -20.0) {
                numArray[i] = -20.0;
            } else {
                numArray[i] = inputSensorData[i];
            }
        }
        for (int i = 0; i < std::size(weight); ++i) {
            double num1 = 0.0;
            for (int j = 0; j < std::size(weight[i]); ++j) {
                num1 += weight[i][j] * numArray[j];
            }
            double num2 = static_cast<int>(num1 / 10.0) / 10.0 + w0[i];
            if (i == 0 || i == 1 || i == 5 || i == 8 || i == 11 || i == 14) {
                num2 = std::min(num2, 0.0);
                num2 = std::max(num2, -80.0);
            }
            if (i == 2 && num2 > 10.0)
                num2 = 10.0;
            if (i == 16 && num2 > 0.0)
                num2 = 0.0;
            if (i == 3) {
                if (num2 < 5.0)
                    num2 = 5.0;
                if (num2 > 35.0)
                    num2 = 35.0;
            }
            if ((i == 6 || i == 12 || i == 15) && num2 < 0.0)
                num2 = 0.0;
            if (i == 17 && num2 > 0.0)
                num2 = 0.0;
            if (i == 4 || i == 7 || i == 10 || i == 13) {
                if (num2 > 0.0)
                    num2 = 0.0;
                if (num2 < -100.0)
                    num2 = -100.0;
                doubleList.push_back(num2);
            }
            doubleList.push_back(num2);
        }
        doubleList[20] = 0.6 * doubleList[2] - 5.0;
        if (doubleList[2] > -10.0) {
            doubleList[3] -= 2.5 * (doubleList[2] + 10.0);
            if (doubleList[3] > 30.0)
                doubleList[3] = 30.0;
            doubleList[1] -= 2.0 * (doubleList[2] + 10.0);
        }
        if (doubleList[2] < -10.0) {
            doubleList[3] += 0.35 * (doubleList[2] + 10.0);
            if (doubleList[3] < 20.0)
                doubleList[3] = 20.0;
        }
        if (doubleList[7] > 25.0)
            doubleList[7] = 25.0;
        if (doubleList[15] > 15.0)
            doubleList[15] = 15.0;
        if (doubleList[19] > 35.0)
            doubleList[19] = 35.0;
        if (doubleList[19] > 5.0) {
            doubleList[3] += doubleList[19] - 5.0;
            if (doubleList[3] > 35.0)
                doubleList[3] = 35.0;
        }
        if (doubleList[5] < 0.0) {
            doubleList[7] += 0.15 * doubleList[5];
            if (doubleList[7] < 0.0)
                doubleList[7] = 0.0;
        }
        if (doubleList[13] < 0.0) {
            doubleList[15] += 0.15 * doubleList[13];
            if (doubleList[15] < 0.0)
                doubleList[15] = 0.0;
        }
        if (doubleList[17] < 0.0) {
            doubleList[19] += 0.25 * doubleList[17];
            if (doubleList[19] < 0.0)
                doubleList[19] = 0.0;
        }
        doubleList[11] = 0.24 * doubleList[7] - 0.32 * doubleList[15];
        if (doubleList[6] < -40.0) {
            doubleList[6] = 1.25 * doubleList[6] + 10.0;
            if (doubleList[6] < -80.0)
                doubleList[6] = -80.0;
        }
        if (doubleList[10] < -20.0) {
            doubleList[10] = 1.2 * doubleList[10] + 4.0;
            if (doubleList[10] < -80.0)
                doubleList[10] = -80.0;
        }
        if (doubleList[9] < -7.0)
            doubleList[13] += 0.2 * (doubleList[9] + 7.0);
        if (doubleList[10] < -5.0)
            doubleList[14] += 0.2 * (doubleList[10] + 5.0);
        if (doubleList[13] < -60.0) {
            doubleList[13] = 0.85 * doubleList[13] - 9.0;
            if (doubleList[13] < -100.0)
                doubleList[13] = -100.0;
        }
        if (doubleList[14] < -60.0) {
            doubleList[14] = 0.85 * doubleList[14] - 9.0;
            if (doubleList[14] < -80.0)
                doubleList[14] = -80.0;
        }
        if (doubleList[17] < -40.0) {
            doubleList[17] = 0.88 * doubleList[17] - 4.8;
            if (doubleList[17] < -100.0)
                doubleList[17] = -100.0;
        }
        if (doubleList[18] < -40.0) {
            doubleList[18] = 0.88 * doubleList[18] - 4.8;
            if (doubleList[18] < -80.0)
                doubleList[18] = -80.0;
        }
        if (doubleList[18] < -5.0) {
            doubleList[19] += 0.8 * (doubleList[18] + 5.0);
            if (doubleList[19] < 0.0)
                doubleList[19] = 0.0;
        }
        if (doubleList[0] < -60.0)
            doubleList[0] = -60.0;
        if (doubleList[1] > 0.0)
            doubleList[1] = 0.0;
        if (doubleList[1] < -60.0)
            doubleList[1] = -60.0;
        if (doubleList[2] < -70.0)
            doubleList[2] = -70.0;
        doubleList[4] = 0.8 * doubleList[5];
        doubleList[8] = 0.8 * doubleList[9];
        doubleList[12] = 0.8 * doubleList[13];
        doubleList[16] = 0.8 * doubleList[17];
        if (doubleList[4] > 0.0)
            doubleList[4] = 0.0;
        if (doubleList[8] > 0.0)
            doubleList[8] = 0.0;
        if (doubleList[12] > 0.0)
            doubleList[12] = 0.0;
        if (doubleList[16] > 0.0)
            doubleList[16] = 0.0;
        double num = -1.0;
        doubleList.push_back(num);
        return doubleList;
    }

private:
    const std::string VERSION = "V 1.4.4";
    const double weight[19][12] = {
        {
            -150.0,
            45.0,
            30.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            -10.0,
            0.0,
            10.0,
            0.0
        },
        {
            -30.0,
            -75.0,
            15.0,
            0.0,
            0.0,
            8.0,
            0.0,
            0.0,
            8.0,
            10.0,
            15.0,
            10.0
        },
        {
            45.0,
            -105.0,
            -50.0,
            10.0,
            0.0,
            0.0,
            10.0,
            0.0,
            0.0,
            0.0,
            10.0,
            10.0
        },
        {

        },
        {
            0.0,
            -5.0,
            0.0,
            -105.0,
            0.0,
            5.0,
            0.0,
            0.0,
            -5.0,
            -5.0,
            0.0,
            -5.0
        },
        {
            0.0,
            0.0,
            -2.0,
            -75.0,
            10.0,
            -10.0,
            0.0,
            0.0,
            0.0,
            -2.0,
            0.0,
            -2.0
        },
        {
            0.0,
            -3.0,
            0.0,
            0.0,
            35.0,
            0.0,
            0.0,
            -5.0,
            0.0,
            0.0,
            -4.0,
            -3.0
        },
        {
            0.0,
            -5.0,
            0.0,
            0.0,
            0.0,
            0.0,
            -115.0,
            0.0,
            15.0,
            -5.0,
            0.0,
            -5.0
        },
        {
            0.0,
            -5.0,
            0.0,
            0.0,
            5.0,
            0.0,
            -70.0,
            0.0,
            -15.0,
            -2.0,
            0.0,
            -2.0
        },
        {

        },

        {
            0.0,
            -5.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            -140.0,
            0.0,
            0.0
        },
        {
            0.0,
            -5.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            -100.0,
            -10.0,
            0.0
        },
        {
            0.0,
            0.0,
            0.0,
            0.0,
            -3.0,
            0.0,
            0.0,
            40.0,
            0.0,
            0.0,
            -20.0,
            0.0
        },
        {
            0.0,
            -12.0,
            0.0,
            0.0,
            0.0,
            -5.0,
            0.0,
            0.0,
            0.0,
            -20.0,
            30.0,
            -160.0
        },
        {
            0.0,
            -5.0,
            0.0,
            0.0,
            0.0,
            -5.0,
            0.0,
            0.0,
            0.0,
            -15.0,
            28.0,
            -110.0
        },
        {
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            40.0,
            0.0
        },
        {
            2.5,
            -52.5,
            5.0,
            0.0,
            0.0,
            4.0,
            0.0,
            0.0,
            4.0,
            2.5,
            5.0,
            5.0
        },
        {

        },
        {

        }
    };
    const double w0[19] = {
        0.0,
        0.0,
        10.0,
        30.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -10.0,
        0.0,
        0.0
    };
};

ExportAR1Linear(AR1LinearAG)
