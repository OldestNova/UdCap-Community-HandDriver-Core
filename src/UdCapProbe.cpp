//
// Created by max_3 on 2025/5/11.
//

#include <iostream>
#include <utility>
#include "UdCapProbe.h"

UdCapProbe::UdCapProbe(std::shared_ptr<PortAccessor> portAccessor): portAccessor(std::move(portAccessor)) {

}

UdCapProbe::~UdCapProbe() = default;

UdCapProbeType UdCapProbe::probe() {
    try {
        this->udCapSerial = "";
        portAccessor->openPort();
        portAccessor->setBaudRate(115200);
        portAccessor->setReadSize(32);
        std::vector<uint8_t> tmp;
        uint8_t chance = 6;
        while (chance--) {
            std::vector<uint8_t> read = portAccessor->readData();
            if (read.empty()) {
                continue;
            }
            for (auto r: read) {
                if (r == '\r' || r == '\n') {
                    chance = 0;
                    break;
                }
                tmp.push_back(r);
            }
            if (chance == 5) {
                std::string s = "AT+NAME?\r\n";
                portAccessor->writeData(s);
            }
        }
        if (tmp.empty()) {
            this->udCapSerial = "";
            return UDCAP_PROBE_FAILURE;
        }
        std::string tmpStr(tmp.begin(), tmp.end());
        size_t udPrefix = tmpStr.find("UD");
        if (udPrefix == std::string::npos) {
            this->udCapSerial = "";
            return UDCAP_PROBE_FAILURE;
        }
        this->udCapSerial = tmpStr.substr(udPrefix);
        return UDCAP_PROBE_HAND_V1;
    } catch (std::runtime_error& err) {
        this->udCapSerial = "";
        return UDCAP_PROBE_FAILURE;
    }
    this->udCapSerial = "";
    return UDCAP_PROBE_FAILURE;
}

std::string UdCapProbe::getUDCapSerial() {
    return this->udCapSerial;
}