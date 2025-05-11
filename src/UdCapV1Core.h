//
// Created by max_3 on 2025/5/11.
//

#ifndef SERIALREBORN_UDCAPV1CORE_H
#define SERIALREBORN_UDCAPV1CORE_H

#include <map>
#include "PortAccessor.h"

enum UdCapV1StateMachine {
    NOOP,
    GET_170,
//    PACKET_HEADER_DEVICE_ID,
    PACKET_HEADER_ADDRESS,
    PACKET_HEADER_COMMAND_TYPE,
    PACKET_HEADER_DATA_TYPE,
    PACKET_HEADER_GET_LENGTH,
    PACKET_TRANSFER,
    PACKET_CRC
};

enum CommandType {
    CMD_DATA = 1,
    CMD_BATTERY = 5,
    CMD_SERIAL = 6,
    CMD_LINK_STATE = 7,
    CMD_SET_CHANNEL = 8,
    CMD_GET_CHANNEL = 9,
    CMD_SET_CHANNEL_DONE = 10,
    CMD_FW_VERSION = 11,
    CMD_STOP_DATA = 160
};

enum LinkState {
    LINK_STATE_UNKNOWN = -1,
    LINK_STATE_NOT_CONNECT = 0,
    LINK_STATE_CONNECTED = 1
};

struct UdCapV1MCUPacket {
    uint8_t address;
    CommandType commandType;
    LinkState linkState;
    uint16_t channel;
    std::string fwVersion;
    uint16_t battery;
    std::string deviceSerialNum;
    bool isEnterprise;
    std::vector<int16_t> angle;
};

enum UdTarget {
    UD_TARGET_UNKNOWN = 0,
    UD_TARGET_LEFT_HAND = 1,
    UD_TARGET_RIGHT_HAND = 2,
};

enum UdInitState {
    UD_INIT_STATE_UNKNOWN = -1,
    UD_INIT_STATE_NOT_INIT = 0,
    UD_INIT_STATE_PRE_INIT = 1,
    UD_INIT_STATE_INIT = 2,
};
class UdCapHandV1PacketRealignmentHelper : public PacketRealignmentHelper {
public:
    UdCapHandV1PacketRealignmentHelper() = default;
    ~UdCapHandV1PacketRealignmentHelper() override = default;
    std::vector<std::vector<uint8_t>> processPacket(const std::vector<uint8_t>& data) override;
private:
    std::vector<uint8_t> packetBuffer;
    std::vector<std::vector<uint8_t>> processedPacket;
    uint32_t remainedLength = 0;
    UdCapV1StateMachine stateMachine = NOOP;
};


class UdCapV1Core {
public:
    UdCapV1Core(std::shared_ptr<PortAccessor> portAccessor);
    ~UdCapV1Core();

    void sendCommand(uint8_t humanAddress, CommandType commandType, const std::vector<uint8_t> &data);
    std::function<void()> listen(const std::function<void(const UdCapV1MCUPacket &)>& callback);
    static std::string fromLinkStateToString(LinkState state);
    UdTarget getTarget() const;
    std::string getUDCapSerial() const;
    void mcuStopData();
    void mcuStartData();
    void mcuGetSerialNum();
private:
    void parsePacket(const std::vector<uint8_t> &);
    void callListenCallback(const UdCapV1MCUPacket &packet);
    std::function<void()> unlistenPortCallback;
    std::shared_ptr<PortAccessor> portAccessor;
    std::string udCapSerial;
    UdTarget target = UD_TARGET_UNKNOWN;
    std::vector<std::function<void(const UdCapV1MCUPacket &)>> listenCallbacks;
    std::mutex callbackMutex;
    uint16_t lastBattery = 0;
    bool isEnterprise = false;
    UdInitState initState = UD_INIT_STATE_NOT_INIT;
    LinkState lastConnState = LINK_STATE_UNKNOWN;
};


#endif //SERIALREBORN_UDCAPV1CORE_H
