//
// Created by max_3 on 2025/5/11.
//

#include "UdCapV1Core.h"

uint8_t calculateCRC(const std::vector<uint8_t>& data, bool ignoreLast) {
    uint8_t crc = 0;
    size_t lastIndex = ignoreLast ? data.size() - 1 : data.size();
    for (size_t i = 2; i < data.size() - 1; i++) {
        crc += data[i];
    }
    return crc;
}

std::vector<uint8_t> decodeXOR(std::vector<uint8_t> data) {
    std::vector<uint8_t> result;
    for (auto& byte : data) {
        auto u = byte ^ 1;
        u ^= 128;
        result.push_back(u);
    }
    return result;
}

std::vector<std::vector<uint8_t>> UdCapHandV1PacketRealignmentHelper::processPacket(const std::vector<uint8_t> &data) {
    std::vector<std::vector<uint8_t>> packets;
    for (auto& byte : data) {
        switch (this->stateMachine) {
            case NOOP: {
                if (byte == 170) {
                    this->stateMachine = GET_170;
                }
                break;
            }
            case GET_170: {
                if (byte == 85) {
                    this->stateMachine = PACKET_HEADER_ADDRESS;
                    packetBuffer.clear();
                    packetBuffer.push_back(170);
                    packetBuffer.push_back(85);
                } else {
                    this->stateMachine = NOOP;
                }
                break;
            }
            case PACKET_HEADER_ADDRESS: {
                packetBuffer.push_back(byte);
                this->stateMachine = PACKET_HEADER_COMMAND_TYPE;
                break;
            }
            case PACKET_HEADER_COMMAND_TYPE: {
                packetBuffer.push_back(byte);
                this->stateMachine = PACKET_HEADER_DATA_TYPE;
                break;
            }
            case PACKET_HEADER_DATA_TYPE: {
                packetBuffer.push_back(byte);
                this->stateMachine = PACKET_HEADER_GET_LENGTH;
                break;
            }
            case PACKET_HEADER_GET_LENGTH: {
                packetBuffer.push_back(byte);
                this->remainedLength = (uint32_t) byte;
                this->stateMachine = PACKET_TRANSFER;
                break;
            }
            case PACKET_TRANSFER: {
                packetBuffer.push_back(byte);
                this->remainedLength--;
                if (this->remainedLength == 0) {
                    this->stateMachine = PACKET_CRC;
                }
                break;
            }
            case PACKET_CRC: {
                packetBuffer.push_back(byte);
                uint8_t crc = calculateCRC(packetBuffer, true);
                this->stateMachine = NOOP;
                if (crc == packetBuffer.back()) {
                    packets.push_back(packetBuffer);
                }
                packetBuffer.clear();
                break;
            }
        }
    }
    return packets;
}

UdCapV1Core::UdCapV1Core(std::shared_ptr<PortAccessor> portAccessor):
        portAccessor(std::move(portAccessor)) {
    this->portAccessor->openPort();
    this->portAccessor->setBaudRate(115200);
    if (!this->portAccessor->hasPacketRealignmentHelper()) {
        std::unique_ptr<UdCapHandV1PacketRealignmentHelper> packetRealignmentHelper = std::make_unique<UdCapHandV1PacketRealignmentHelper>();
        this->portAccessor->setPacketRealignmentHelper(std::move(packetRealignmentHelper));
    }
    this->unlistenPortCallback = this->portAccessor->addDataCallback([&](const std::vector<uint8_t> &data) {
        parsePacket(data);
    });
    this->portAccessor->startContinuousRead();
}

UdCapV1Core::~UdCapV1Core() {
    this->mcuStopData();
    this->unlistenPortCallback();
    this->portAccessor->stopContinuousRead();
}

std::function<void()> UdCapV1Core::listen(const std::function<void(const UdCapV1MCUPacket &)>& callback) {
    std::lock_guard guard(callbackMutex);
    listenCallbacks.push_back(callback);
    return [this, callback]() {
        // Remove the callback from the list
        listenCallbacks.erase(std::remove_if(listenCallbacks.begin(), listenCallbacks.end(),
                                           [&callback](const std::function<void(const UdCapV1MCUPacket&)>& item) {
                                               return item.target_type() == callback.target_type();
                                           }), listenCallbacks.end());
    };
}

void UdCapV1Core::callListenCallback(const UdCapV1MCUPacket &packet) {
    std::lock_guard guard(callbackMutex);
    for (const auto& callback : listenCallbacks) {
        callback(packet);
    }
}

void UdCapV1Core::sendCommand(uint8_t humanAddress, CommandType commandType, const std::vector<uint8_t> &data) {
    std::vector<uint8_t> packet;
    packet.push_back(85);
    packet.push_back(170);
    packet.push_back(humanAddress); // humanAddress
    packet.push_back((uint8_t)commandType); // commandType
    packet.push_back((uint8_t)data.size()); // dataLength
    packet.insert(packet.end(), data.begin(), data.end());
    uint8_t crc = calculateCRC(packet, false);
    packet.push_back(crc);
    this->portAccessor->writeData(packet);
}

std::string UdCapV1Core::getUDCapSerial() const {
    return udCapSerial;
}

void UdCapV1Core::parsePacket(const std::vector<uint8_t> & packetBuffer) {
    uint8_t dataLength = packetBuffer[5];
    std::vector<uint8_t> data;
    for (size_t i = 6; i < 6 + dataLength; i++) {
        data.push_back(packetBuffer[i]);
    }
    if (packetBuffer[3] == (uint8_t)CommandType::CMD_LINK_STATE) {
        auto deData = decodeXOR(data);
        short linkState = deData[0];
        UdCapV1MCUPacket packet {};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_LINK_STATE;
        if (linkState == 0) {
            packet.linkState = LinkState::LINK_STATE_NOT_CONNECT;

            if (packet.linkState != lastConnState) {
                callListenCallback(packet);
            }
            lastConnState = packet.linkState;

            if (initState == UD_INIT_STATE_INIT) {
                mcuStopData();
                initState = UD_INIT_STATE_NOT_INIT;
            }

        } else if (linkState == 1) {
            packet.linkState = LinkState::LINK_STATE_CONNECTED;

            if (packet.linkState != lastConnState) {
                callListenCallback(packet);
            }
            lastConnState = packet.linkState;

            // 自动获取序列号
            if (initState == UD_INIT_STATE_NOT_INIT) {
                initState = UD_INIT_STATE_INIT;
//                mcuGetSerialNum();
                mcuStartData();
            }
        } else {
            packet.linkState = LinkState::LINK_STATE_UNKNOWN;

            if (packet.linkState != lastConnState) {
                callListenCallback(packet);
            }
            lastConnState = packet.linkState;
        }
        if (this->udCapSerial.empty()) {
            UdCapV1MCUPacket packetSerial{};
            packetSerial.address = packetBuffer[2];
            packetSerial.commandType = CommandType::CMD_SERIAL;
            this->udCapSerial = std::string(deData.begin() + 1, deData.end());
            packetSerial.deviceSerialNum = this->udCapSerial;
            if (*(deData.end() - 1) == 'L') {
                this->target = UD_TARGET_LEFT_HAND;
            } else if (*(deData.end() - 1) == 'R') {
                this->target = UD_TARGET_RIGHT_HAND;
            } else {
                this->target = UD_TARGET_UNKNOWN;
            }
            if (packetSerial.deviceSerialNum.find("AB") != std::string::npos) {
                packetSerial.isEnterprise = true;
                isEnterprise = true;
                initState = UD_INIT_STATE_INIT;
                callListenCallback(packetSerial);
            } else if (packetSerial.deviceSerialNum.find("AC") != std::string::npos) {
                packetSerial.isEnterprise = false;
                isEnterprise = false;
                initState = UD_INIT_STATE_INIT;
                callListenCallback(packetSerial);
            }
        }
    } else if (packetBuffer[3] == (uint8_t)CommandType::CMD_SET_CHANNEL_DONE) {
        UdCapV1MCUPacket packet {};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_SET_CHANNEL_DONE;
        callListenCallback(packet);
    } else if (packetBuffer[3] == (uint8_t)CommandType::CMD_GET_CHANNEL) {
        UdCapV1MCUPacket packet {};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_GET_CHANNEL;
        auto deData = decodeXOR(data);
        packet.channel = deData[0];
        callListenCallback(packet);
    } else if (packetBuffer[3] == (uint8_t)CommandType::CMD_FW_VERSION) {
        UdCapV1MCUPacket packet {};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_FW_VERSION;
        auto deData = decodeXOR(data);
        packet.fwVersion = std::string(deData.begin(), deData.end());
        callListenCallback(packet);
    } else if (packetBuffer[3] == (uint8_t)CommandType::CMD_BATTERY) {
        UdCapV1MCUPacket packet {};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_BATTERY;
        auto deData = decodeXOR(data);
        auto udata = (int16_t) ((deData[0] << 8) | deData[1]);
        int batt = ((udata <= 1980) ? 1 : ((udata <= 2090) ? 2 : ((udata <= 2150) ? 3 : ((udata > 2300) ? 5 : 4))));
        if (batt > lastBattery)
        {
            batt = ((udata <= 2010) ? 1 : ((udata <= 2120) ? 2 : ((udata <= 2180) ? 3 : ((udata > 2320) ? 5 : 4))));
        }
        lastBattery = batt;
        packet.battery = batt;
        callListenCallback(packet);
    } else if (packetBuffer[3] == (uint8_t)CommandType::CMD_SET_CHANNEL) {
        UdCapV1MCUPacket packet {};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_SET_CHANNEL;
        auto deData = decodeXOR(data);
        uint16_t result = deData[1];
        packet.channel = result;
        callListenCallback(packet);
    } else if (packetBuffer[3] == (uint8_t)CommandType::CMD_SERIAL) {
        {
            UdCapV1MCUPacket packet{};
            packet.address = packetBuffer[2];
            packet.commandType = CommandType::CMD_SERIAL;
            auto deData = decodeXOR(data);
            packet.deviceSerialNum = std::string(deData.begin(), deData.end());
            if (packet.deviceSerialNum.find("AB") != std::string::npos) {
                packet.isEnterprise = true;
                isEnterprise = true;
                initState = UD_INIT_STATE_INIT;
                callListenCallback(packet);
            } else if (packet.deviceSerialNum.find("AC") != std::string::npos) {
                packet.isEnterprise = false;
                isEnterprise = false;
                initState = UD_INIT_STATE_INIT;
                callListenCallback(packet);
            }
        }
        mcuStartData();
    } else if (packetBuffer[3] == (uint8_t)CommandType::CMD_DATA) {
        UdCapV1MCUPacket packet {};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_DATA;
        auto deData = decodeXOR(data);
        // std::vector<uint8_t> deData to std::vector<int16_t>
        std::vector<int16_t> iData;
        for (size_t i = 0; i < deData.size(); i += 2) {
            auto value = (int16_t) ((deData[i] << 8) | deData[i + 1]);
            iData.push_back(value);
        }
        packet.angle = iData;
        callListenCallback(packet);
    }
}

UdTarget UdCapV1Core::getTarget() const {
    return target;
}

void UdCapV1Core::mcuStopData() {
    if (initState == UD_INIT_STATE_INIT) {
        sendCommand(1, CommandType::CMD_STOP_DATA, {1});
    }
}
void UdCapV1Core::mcuStartData() {
    if (initState == UD_INIT_STATE_INIT) {
        std::vector<uint8_t> s;
        s.push_back(2);
        if (isEnterprise) {
            s.push_back(66);
            s.push_back(86);
        } else {
            s.push_back(65);
            s.push_back(67);
        }
        sendCommand(1, CommandType::CMD_DATA, s);
    }
}
void UdCapV1Core::mcuGetSerialNum() {
    sendCommand(1, CommandType::CMD_SERIAL, {1});
}

std::string UdCapV1Core::fromLinkStateToString(LinkState state) {
    switch (state) {
        case LINK_STATE_UNKNOWN:
            return "Unknown";
        case LINK_STATE_NOT_CONNECT:
            return "Not Connected";
        case LINK_STATE_CONNECTED:
            return "Connected";
        default:
            return "Invalid State";
    }
}