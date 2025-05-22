//
// Created by max_3 on 2025/5/11.
//

#include <utility>
#include "UdCapProbe.h"

UdCapProbe::UdCapProbe(std::shared_ptr<PortAccessor> portAccessor): portAccessor(std::move(portAccessor)) {

}

UdCapProbe::~UdCapProbe() = default;

UdCapProbeType UdCapProbe::probe() {
    std::function<void()> unlisten = [](){};
    try {
        this->udCapSerial = "";
        portAccessor->openPort();
        portAccessor->setBaudRate(115200);
        portAccessor->setReadSize(8);
        std::mutex mutex;
        std::condition_variable condition;
        std::string uds = "";
        unlisten = portAccessor->addOnceRawDataCallback([this, &uds, &condition](const std::vector<uint8_t>& data) {
            for (uint8_t b: data) {
                if (this->stateMachine == UDCAP_PROBE_STATE_NONE) {
                    if (static_cast<char>(b) == 'U') {
                        this->stateMachine = UDCAP_PROBE_STATE_GET_HEADER_U;
                        uds.append("U");
                    }
                } else if (this->stateMachine == UDCAP_PROBE_STATE_GET_HEADER_U) {
                    if (static_cast<char>(b) == 'D') {
                        this->stateMachine = UDCAP_PROBE_STATE_GET_HEADER_UD;
                        uds.append("D");
                    } else {
                        this->stateMachine = UDCAP_PROBE_STATE_NONE;
                        uds = "";
                    }
                } else if (this->stateMachine == UDCAP_PROBE_STATE_GET_HEADER_UD) {
                    char byte = static_cast<char>(b);
                    if (byte != '\r') {
                        this->stateMachine = UDCAP_PROBE_STATE_GET_HEADER_UD;
                        std::string sByte = "";
                        sByte += byte;
                        uds.append(sByte);
                    } else {
                        condition.notify_all();
                        return true;
                    }
                }
            }
            return false;
        });
        portAccessor->startContinuousRead();
        std::string s = "AT+NAME?\r\n";
        portAccessor->writeData(s);
        std::unique_lock lk(mutex);
        auto state = condition.wait_for(lk, std::chrono::seconds(2));
        portAccessor->stopContinuousRead();
        if (state == std::cv_status::timeout) {
            this->udCapSerial = "";
            unlisten();
            return UDCAP_PROBE_FAILURE;
        }
        this->udCapSerial = uds;
        return UDCAP_PROBE_HAND_V1;
    //     std::vector<uint8_t> tmp;
    //     uint8_t chance = 6;
    //     while (chance--) {
    //         std::vector<uint8_t> read = portAccessor->readData();
    //         if (read.empty()) {
    //             continue;
    //         }
    //         for (auto r: read) {
    //             if (r == '\r' || r == '\n') {
    //                 chance = 0;
    //                 break;
    //             }
    //             tmp.push_back(r);
    //         }
    //         if (chance == 5) {
    //             std::string s = "AT+NAME?\r\n";
    //             portAccessor->writeData(s);
    //         }
    //     }
    //     if (tmp.empty()) {
    //         this->udCapSerial = "";
    //         return UDCAP_PROBE_FAILURE;
    //     }
    //     std::string tmpStr(tmp.begin(), tmp.end());
    //     size_t udPrefix = tmpStr.find("UD");
    //     if (udPrefix == std::string::npos) {
    //         this->udCapSerial = "";
    //         return UDCAP_PROBE_FAILURE;
    //     }
    //     this->udCapSerial = tmpStr.substr(udPrefix);
    //     return UDCAP_PROBE_HAND_V1;
    } catch (std::runtime_error&) {
        this->udCapSerial = "";
        unlisten();
        if (portAccessor->isOpen()) {
            this->portAccessor->stopContinuousRead();
        }
        return UDCAP_PROBE_FAILURE;
    }
    this->udCapSerial = "";
    unlisten();
    if (portAccessor->isOpen()) {
        this->portAccessor->stopContinuousRead();
    }
    return UDCAP_PROBE_FAILURE;
}

std::string UdCapProbe::getUDCapSerial() {
    return this->udCapSerial;
}