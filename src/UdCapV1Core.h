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
    CMD_ANGLE = -1,
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
    std::vector<double> result;
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


struct UdCapV1HandCaliFist {
    bool captured;
    double h4;
    double h5;
    double h6;
    double h7;
    double h9;
    double h10;
    double h12;
    double h13;
    double h15;
};
struct UdCapV1HandCaliAdduction {
    bool captured;
    double n1;
    double n2;
    double n3;
    double n4;
    double n5;
    double n6;
    double n7;
    double n8;
    double n9;
    double n10;
    double n11;
    double n12;
    double n13;
    double n14;
    double n15;
};
struct UdCapV1HandCaliProtract {
    bool captured;
    double h8;
    double h11;
    double h14;
};
struct UdCapV1HandData {
    float f0;
    float f1;
    float f2;
    float f3;
    float f4;
    float f5;
    float f6;
    float f7;
    float f8;
    float f9;
    float f10;
    float f11;
    float f12;
    float f13;
    float f14;
    float f15;
    float f16;
    float f17;
    float f18;
};

struct UdCapV1InputData {
    float joyX;
    float joyY;

    bool joyButton;
    bool aButton;
    bool bButton;
    bool menuButton;
};

enum UdCapV1HandCaliStat {
    UDCAP_V1_HAND_CALI_STAT_AUTO = -1,
    UDCAP_V1_HAND_CALI_STAT_NONE = 0,
//    UDCAP_V1_HAND_CALI_STAT_FIST = 1,
//    UDCAP_V1_HAND_CALI_STAT_ADDUCTION = 2,
//    UDCAP_V1_HAND_CALI_STAT_PROTRACT = 3,
    UDCAP_V1_HAND_CALI_STAT_COMPLETED = 4
};
enum UdCapV1HandCaliType {
    UDCAP_V1_HAND_CALI_TYPE_ALL = 0,
    UDCAP_V1_HAND_CALI_TYPE_FIST = 1,
    UDCAP_V1_HAND_CALI_TYPE_ADDUCTION = 2,
    UDCAP_V1_HAND_CALI_TYPE_PROTRACT = 3,
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

    void runCalibration();
    void captureCalibrationData(UdCapV1HandCaliType type);
    void clearCalibrationData(UdCapV1HandCaliType type);
    void completeCalibrationData();
    UdCapV1HandCaliStat getCalibrationStatus() const;
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
    UdCapV1HandCaliStat caliStat = UDCAP_V1_HAND_CALI_STAT_NONE;
    UdCapV1HandCaliFist caliFist;
    UdCapV1HandCaliAdduction caliAdduction;
    UdCapV1HandCaliProtract caliProtract;
    UdCapV1HandData lastAngle {};
    float xCenterData = 1850.0;
    float yCenterData = 1850.0;
    float xMaxData = 3750.0;
    float xMinData = 620.0;
    float yMaxData = 3750.0;
    float yMinData = 620.0;
    float deadZone = 0.15;
    bool thumbOn = true;
};


#endif //SERIALREBORN_UDCAPV1CORE_H
