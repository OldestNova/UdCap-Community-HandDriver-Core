//
// Created by max_3 on 2025/5/11.
//

#ifndef SERIALREBORN_UDCAPV1CORE_H
#define SERIALREBORN_UDCAPV1CORE_H

#include <map>
#include <chrono>
#include <PortAccessor.h>
#include <queue>

#define Default_xCenterData 1850.0;
#define Default_yCenterData 1850.0;
#define Default_xMaxData 3750.0;
#define Default_xMinData 620.0;
#define Default_yMaxData 3750.0;
#define Default_yMinData 620.0;

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
    CMD_SKELETON_QUATERNION = -5,
    CMD_INPUT_BUTTON = -4,
    CMD_INPUT_JOYSTICK = -3,
    CMD_ANGLE = -2,
    CMD_READY = -1,
    CMD_DATA = 1,
    CMD_VIBRATION = 5,
    CMD_BATTERY = 5,
    CMD_SERIAL = 6,
    CMD_LINK_STATE = 7,
    CMD_SET_CHANNEL = 8,
    CMD_GET_CHANNEL = 9,
    CMD_SET_CHANNEL_DONE = 10,
    CMD_FW_VERSION = 11,
    CMD_STOP_DATA = 160
};

struct UdCapV1JoystickData {
    float joyX;
    float joyY;
};

struct UdCapV1ButtonData {
    bool btnA;
    bool btnB;
    bool btnMenu;
    bool btnJoyStick;
    bool btnPower;
};

enum UdTarget {
    UD_TARGET_UNKNOWN = 0,
    UD_TARGET_LEFT_HAND = 1,
    UD_TARGET_RIGHT_HAND = 2,
};

enum UdState {
    UD_INIT_STATE_INIT = 0,
    UD_INIT_STATE_NOT_CONNECT = 1,
    UD_INIT_STATE_CONNECTED = 2,
    UD_INIT_STATE_LINKED = 3
};

struct BoneQuaternion {
    float x;
    float y;
    float z;
    float w;
};

struct FingerQuaternion {
    BoneQuaternion proximal;
    BoneQuaternion intermediate;
    BoneQuaternion distal;
};

struct HandQuaternion {
    FingerQuaternion thumbFinger;
    FingerQuaternion indexFinger;
    FingerQuaternion middleFinger;
    FingerQuaternion ringFinger;
    FingerQuaternion littleFinger;
};

struct UdCapV1MCUPacket {
    uint8_t address;
    CommandType commandType;
    UdState udState;
    uint16_t channel;
    uint16_t channelResult;
    std::string fwVersion;
    uint16_t battery;
    std::string deviceSerialNum;
    bool isReady;
    bool isEnterprise;
    std::vector<int16_t> angle;
    std::vector<double> result;
    UdCapV1JoystickData joystickData {};
    UdCapV1ButtonData button {};
    HandQuaternion skeletonQuaternion {};
};

class UdCapHandV1PacketRealignmentHelper : public PacketRealignmentHelper {
public:
    UdCapHandV1PacketRealignmentHelper() = default;

    ~UdCapHandV1PacketRealignmentHelper() override = default;

    std::vector<std::vector<uint8_t> > processPacket(const std::vector<uint8_t> &data) override;

private:
    std::vector<uint8_t> packetBuffer;
    std::vector<std::vector<uint8_t> > processedPacket;
    uint32_t remainedLength = 0;
    volatile UdCapV1StateMachine stateMachine = NOOP;
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

enum UdCapV1HandCaliStat {
    UDCAP_V1_HAND_CALI_STAT_AUTO = -1,
    UDCAP_V1_HAND_CALI_STAT_NONE = 0,
    //    UDCAP_V1_HAND_CALI_STAT_FIST = 1,
    //    UDCAP_V1_HAND_CALI_STAT_ADDUCTION = 2,
    //    UDCAP_V1_HAND_CALI_STAT_PROTRACT = 3,
    UDCAP_V1_HAND_CALI_STAT_COMPLETED = 4
};

enum UdCapV1JoystickCaliStat {
    UDCAP_V1_JOYSTICK_CALI_STAT_OK = 0,
    UDCAP_V1_JOYSTICK_CALI_STAT_CAPTURE_CENTER = 1,
    UDCAP_V1_JOYSTICK_CALI_STAT_CAPTURE_ZONE = 2,
};

enum UdCapV1HandCaliType {
    UDCAP_V1_HAND_CALI_TYPE_ALL = 0,
    UDCAP_V1_HAND_CALI_TYPE_FIST = 1,
    UDCAP_V1_HAND_CALI_TYPE_ADDUCTION = 2,
    UDCAP_V1_HAND_CALI_TYPE_PROTRACT = 3,
};

enum UdCapV1JoystickCaliType {
    UDCAP_V1_JOYSTICK_CALI_TYPE_ALL = 0,
    UDCAP_V1_JOYSTICK_CALI_TYPE_CENTER = 1,
    UDCAP_V1_JOYSTICK_CALI_TYPE_ZONE = 2,
};

enum UdCapV1DeviceCaliType {
    UDCAP_V1_DEVICE_CALI_TYPE_HAND = 0,
    UDCAP_V1_DEVICE_CALI_TYPE_JOYSTICK = 1,
};

class UdCapV1Core {
public:
    UdCapV1Core(std::shared_ptr<PortAccessor> portAccessor);

    ~UdCapV1Core();

    void sendCommand(uint8_t humanAddress, CommandType commandType, const std::vector<uint8_t> &data);

    std::function<void()> listen(const std::function<void(const UdCapV1MCUPacket &)> &callback);

    static std::string fromUdStateToString(UdState state);

    UdTarget getTarget() const;

    std::string getUDCapSerial() const;

    BoneQuaternion eulerToQuaternion(double pitch, double yaw, double roll);

    void mcuStopData();

    void mcuStartData();

    void mcuGetSerialNum();

    void mcuGetChannel();

    void mcuSetChannel(uint8_t channel);

    void mcuReset();

    void mcuSendVibration(int index, float second, int strength);

    void runCalibration(UdCapV1DeviceCaliType type);

    void captureCalibrationData(UdCapV1HandCaliType type);

    void clearCalibrationData(UdCapV1HandCaliType type);

    void completeCalibration(UdCapV1DeviceCaliType type);

    void captureJoystickData(UdCapV1JoystickCaliType type);

    void clearJoystickData(UdCapV1JoystickCaliType type);

    UdCapV1HandCaliStat getHandCalibrationStatus() const;

    UdCapV1JoystickCaliStat getJoystickCalibrationStatus() const;

    // TODO
    bool loadPref();

    bool loadPref(std::string path);

    bool savePref();

    bool savePref(std::string path);

private:
    void parsePacket(const std::vector<uint8_t> &);

    void callListenCallback(const UdCapV1MCUPacket &packet);
    std::atomic_uint32_t callbackFd = 0;
    std::thread eventLoop;
    std::atomic_bool eventLoopRunning;
    std::queue<UdCapV1MCUPacket> packetQueue;
    std::mutex eventLoopMutex;
    std::condition_variable eventCondition;
    std::function<void()> unlistenPortCallback;
    std::shared_ptr<PortAccessor> portAccessor;
    std::string udCapSerial;
    volatile UdTarget target = UD_TARGET_UNKNOWN;
    std::map<uint32_t, std::function<void(const UdCapV1MCUPacket &)> > listenCallbacks;
    std::mutex callbackMutex;
    uint16_t lastBattery = 0;
    bool isEnterprise = false;
    volatile UdState udState = UD_INIT_STATE_INIT;
    volatile UdCapV1HandCaliStat caliStat = UDCAP_V1_HAND_CALI_STAT_NONE;
    UdCapV1HandCaliFist caliFist;
    UdCapV1HandCaliAdduction caliAdduction;
    UdCapV1HandCaliProtract caliProtract;
    UdCapV1HandData lastAngle{};
    volatile UdCapV1JoystickCaliStat joystickCaliStat = UDCAP_V1_JOYSTICK_CALI_STAT_OK;
    float xCenterData = 1850.0;
    float yCenterData = 1850.0;
    float xMaxData = 3750.0;
    float xMinData = 620.0;
    float yMaxData = 3750.0;
    float yMinData = 620.0;
    float deadZone = 0.15;
    bool thumbOn = true;
    bool isSettingChannel = false;
    float thumbFix[3] = { 0.1f, 0.3f, 1.2f };

    std::chrono::system_clock::time_point powerButtonTimeout;
    bool powerBtnPressed = false;

    int count = 0;
    double filterCount[23][9]{};
    const double SIGNDATA[2][23]{
        {
            -47.0, -25.0, -10.0, 20.0, -40.0, -70.0, -50.0, 0.0, 0.0, 0.0,
            0.0, -5.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 20.0,
            -5.0, 0.0, 0.0
        },
        {
            0.0, 0.0, -45.0, 15.0, -18.0, -40.0, -90.0, 0.0, -70.0, -110.0,
            -90.0, 0.0, -70.0, -110.0, -90.0, 0.0, -70.0, -110.0, -90.0, 0.0,
            -15.0, 0.0, 0.0
        }
    };
};


#endif //SERIALREBORN_UDCAPV1CORE_H
