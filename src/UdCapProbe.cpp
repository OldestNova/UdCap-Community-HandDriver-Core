//
// Created by max_3 on 2025/5/11.
//

#include <utility>
#include "UdCapProbe.h"
#include "UdCapSerial.h"

std::vector<std::vector<uint8_t>> UdCapProbePacketRealignmentHelper::processPacket(const std::vector<uint8_t> &data) {
    std::vector<std::vector<uint8_t> > packets;
    for (auto &byte: data) {
        switch (this->stateMachine) {
            case UDCAP_PROBE_STATE_NONE: {
                if (byte == 'U') {
                    this->stateMachine = UDCAP_PROBE_STATE_GET_HEADER_U;
                }
                break;
            }
            case UDCAP_PROBE_STATE_GET_HEADER_U: {
                if (byte == 'D') {
                    this->stateMachine = UDCAP_PROBE_STATE_GET_HEADER_UD;
                    packetBuffer.clear();
                    packetBuffer.push_back('U');
                    packetBuffer.push_back('D');
                } else {
                    this->stateMachine = byte == 'U' ? UDCAP_PROBE_STATE_GET_HEADER_U
                                                     : UDCAP_PROBE_STATE_NONE;
                }
                break;
            }
            case UDCAP_PROBE_STATE_GET_HEADER_UD: {
                if (byte == '\r' || byte == '\n') {
                    const std::string serial(packetBuffer.begin(), packetBuffer.end());
                    if (isUdCapReceiverSerial(serial) || isUdCapCombinedV1ReceiverSerial(serial)) {
                        packets.push_back(packetBuffer);
                    }
                    packetBuffer.clear();
                    this->stateMachine = UDCAP_PROBE_STATE_NONE;
                } else if ((byte != ',' && (byte < '0' || (byte > '9' && byte < 'A') || byte > 'Z')) ||
                           packetBuffer.size() >= 8 + 32) {
                    packetBuffer.clear();
                    this->stateMachine = byte == 'U' ? UDCAP_PROBE_STATE_GET_HEADER_U
                                                     : UDCAP_PROBE_STATE_NONE;
                } else {
                    packetBuffer.push_back(byte);
                }
                break;
            }
        }
    }
    return packets;
}

UdCapProbe::UdCapProbe(std::shared_ptr<PortAccessor> portAccessor): portAccessor(std::move(portAccessor)) {

}

UdCapProbe::~UdCapProbe() = default;

UdCapProbeType UdCapProbe::probe() {
    std::function<void()> unlistenData;
    std::unique_ptr<PacketRealignmentHelper> previousHelper;
    bool helperChanged = false;
    bool startRequested = false;
    const bool openedHere = !portAccessor->isOpen();
    auto cleanup = [&]() {
        if (unlistenData) unlistenData();
        if (startRequested) {
            portAccessor->stopContinuousRead();
            startRequested = false;
        }
        if (helperChanged) {
            portAccessor->setPacketRealignmentHelper(std::move(previousHelper));
            helperChanged = false;
        }
    };
    try {
        this->udCapSerial = "";
        portAccessor->openPort();
        portAccessor->setBaudRate(115200);
        portAccessor->setReadSize(8);
        std::mutex mutex;
        std::condition_variable condition;
        std::string response;
        std::size_t responseCount = 0;
        previousHelper = portAccessor->popPacketRealignmentHelper();
        helperChanged = true;
        portAccessor->setPacketRealignmentHelper(std::make_unique<UdCapProbePacketRealignmentHelper>());
        unlistenData = portAccessor->addDataCallback([&](std::shared_ptr<std::vector<uint8_t>> data) {
            {
                std::lock_guard lk(mutex);
                response.assign(data->begin(), data->end());
                ++responseCount;
            }
            condition.notify_all();
        });
        startRequested = true;
        portAccessor->startContinuousRead();
        bool succeeded = false;
        std::vector<std::string> seenResponses;
        for (int attempt = 0; attempt < 3 && !succeeded; ++attempt) {
            std::size_t previousCount;
            {
                std::lock_guard lk(mutex);
                previousCount = responseCount;
            }
            portAccessor->writeData("AT+NAME?\r\n");
            std::unique_lock lk(mutex);
            if (!condition.wait_for(lk, std::chrono::milliseconds(1500),
                                    [&] { return responseCount > previousCount; })) {
                // A non-UdCap serial adapter is common in the port list. Do
                // not spend three full timeouts on it; the UI retries later.
                if (attempt == 0) break;
                continue;
            }
            if (std::find(seenResponses.begin(), seenResponses.end(), response) != seenResponses.end()) {
                this->udCapSerial = response;
                succeeded = true;
            } else {
                seenResponses.push_back(response);
            }
        }
        cleanup();
        if (!succeeded) {
            this->udCapSerial.clear();
            if (openedHere) portAccessor->closePort();
            return UDCAP_PROBE_FAILURE;
        }
        return isUdCapCombinedV1ReceiverSerial(this->udCapSerial)
                   ? UDP_PROBE_HAND_V1_COMBINED_V1 : UDCAP_PROBE_HAND_V1;
    } catch (const std::exception &) {
        try { cleanup(); } catch (const std::exception &) {}
        this->udCapSerial.clear();
        if (openedHere) {
            try { portAccessor->closePort(); } catch (const std::exception &) {}
        }
        return UDCAP_PROBE_FAILURE;
    }
}

std::string UdCapProbe::getUDCapSerial() {
    return this->udCapSerial;
}
