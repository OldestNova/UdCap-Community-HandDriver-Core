//
// Created by max_3 on 2025/5/11.
//

#ifndef SERIALREBORN_UDCAPV1CORE_H
#define SERIALREBORN_UDCAPV1CORE_H

#include <map>
#include <optional>
#include <atomic>
#include <chrono>
#include <array>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <PortAccessor.h>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include "CommunityV1Calibration.h"
#include "HandResponse.h"

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
    CMD_PORT_CONNECTION_STATE = -6,
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
    CMD_PAIRING = 12,
    CMD_PAIRING_BOTH = 30,
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
    bool btnTrigger;
    bool btnGrip;
    bool btnTrackpad;
    float trigger;
    float grip;
    float trackpad;
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

struct BoneRotation {
    float x;
    float y;
    float z;
};

struct BoneQuaternion {
    float x;
    float y;
    float z;
    float w;
};

struct FingerRotation {
    BoneRotation proximal;
    BoneRotation intermediate;
    BoneRotation distal;
};

struct FingerQuaternion {
    BoneQuaternion proximal;
    BoneQuaternion intermediate;
    BoneQuaternion distal;
};

struct HandRotation {
    FingerRotation thumbFinger;
    FingerRotation indexFinger;
    FingerRotation middleFinger;
    FingerRotation ringFinger;
    FingerRotation littleFinger;
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
    uint16_t batteryRaw;
    // Signed receiver RSSI, in dBm. Zero means telemetry is unavailable.
    int16_t rssiDbm;
    std::string deviceSerialNum;
    bool isReady;
    bool isEnterprise;
    std::array<int16_t, 19> angle;
    std::array<double, 28> result;
    // Snapshot accompanying CMD_ANGLE. Scalar protocols project supported axes;
    // skeleton protocols apply all local XYZ offsets before their own mapping.
    HandRotation angleBoneOffset{};
    UdTarget angleHand = UD_TARGET_UNKNOWN;
    UdCapV1JoystickData joystickData {};
    UdCapV1ButtonData button {};
    HandQuaternion skeletonQuaternion {};
    bool pairing;
    PortAccessorConnectionState portConnectionState;
};

class UdCapHandV1PacketRealignmentHelper : public PacketRealignmentHelper {
public:
    UdCapHandV1PacketRealignmentHelper() = default;

    ~UdCapHandV1PacketRealignmentHelper() override = default;

    std::vector<std::vector<uint8_t> > processPacket(const std::vector<uint8_t> &data) override;

private:
    std::vector<uint8_t> packetBuffer;
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

enum class UdCapV1Algorithm {
    DEFAULT = 0,
    ORIGINAL_V1 = 1,
    COUMMUNITY_V1 = 20
};

enum HandBone {
    HAND_BONE_THUMB_PROXIMAL = 0,
    HAND_BONE_THUMB_INTERMEDIATE = 1,
    HAND_BONE_THUMB_DISTAL = 2,
    HAND_BONE_INDEX_PROXIMAL = 3,
    HAND_BONE_INDEX_INTERMEDIATE = 4,
    HAND_BONE_INDEX_DISTAL = 5,
    HAND_BONE_MIDDLE_PROXIMAL = 6,
    HAND_BONE_MIDDLE_INTERMEDIATE = 7,
    HAND_BONE_MIDDLE_DISTAL = 8,
    HAND_BONE_RING_PROXIMAL = 9,
    HAND_BONE_RING_INTERMEDIATE = 10,
    HAND_BONE_RING_DISTAL = 11,
    HAND_BONE_LITTLE_PROXIMAL = 12,
    HAND_BONE_LITTLE_INTERMEDIATE = 13,
    HAND_BONE_LITTLE_DISTAL = 14
};

struct UdCapV1LastCaliData {
    bool hasLast;
    UdCapV1HandCaliFist caliFist;
    UdCapV1HandCaliAdduction caliAdduction;
    UdCapV1HandCaliProtract caliProtract;
};

class UdCapV1Core {
public:
    UdCapV1Core(std::shared_ptr<PortAccessor> portAccessor);

    ~UdCapV1Core();

    void sendCommand(uint8_t humanAddress, CommandType commandType, const std::vector<uint8_t> &data);

    std::function<void()> listen(const std::function<void(std::shared_ptr<UdCapV1MCUPacket>)> &callback);

    static std::string fromUdStateToString(UdState state);

    UdTarget getTarget() const;

    void setFallbackTarget(UdTarget hand);

    UdState getState() const;

    // The glove reports a five-step battery indicator and its raw ADC value.
    // Newer receivers append signed RSSI as the third battery-packet byte.
    std::optional<std::pair<uint16_t, uint16_t>> getBattery() const;
    std::optional<int16_t> getRssiDbm() const;

    std::string getUDCapSerial() const;

    static BoneQuaternion eulerToQuaternion(double pitch, double yaw, double roll);

    static HandQuaternion officialHandPose(const std::array<double, 28> &angles,
                                           UdTarget hand, const HandRotation &offset,
                                           const std::array<float, 3> &thumbFix);

    void mcuStopData();

    void stopStreamingForShutdown();

    bool waitForPendingWrites(std::chrono::milliseconds timeout) const;

    void mcuStartData();

    void mcuResumeData();

    void mcuGetSerialNum();

    void mcuGetChannel();

    void mcuSetChannel(uint8_t channel);

    void mcuReset();

    void mcuStartPairing();

    void mcuStopPairing();

    // New receivers can pair both gloves through one receiver (60 s by default).
    void mcuStartPairingBoth(uint8_t timeoutSeconds = 60);

    void mcuStopPairingBoth();

    void mcuGetFirmwareVersion();

    void mcuGetLinkState();

    void mcuSendVibration(int index, float second, int strength);

    void runCalibration(UdCapV1DeviceCaliType type);

    void captureCalibrationData(UdCapV1HandCaliType type);

    void clearCalibrationData(UdCapV1HandCaliType type);

    void completeCalibration(UdCapV1DeviceCaliType type);

    void tryRestoreHandCalibration();

    void setAlgorithm(UdCapV1Algorithm algorithm);

    UdCapV1Algorithm getAlgorithm() const;

    void captureJoystickData(UdCapV1JoystickCaliType type);

    void clearJoystickData(UdCapV1JoystickCaliType type);

    UdCapV1HandCaliStat getHandCalibrationStatus() const;

    UdCapV1JoystickCaliStat getJoystickCalibrationStatus() const;

    // Virtual Buttons
    void setTriggerButtonMin(float value);
    void setTriggerButtonMax(float value);
    float getTriggerButtonMin() const;
    float getTriggerButtonMax() const;
    void setGripButtonMin(float value);
    void setGripButtonMax(float value);
    float getGripButtonMin() const;
    float getGripButtonMax() const;
    void setTrackpadButtonMin(float value);
    void setTrackpadButtonMax(float value);
    float getTrackpadButtonMin() const;
    float getTrackpadButtonMax() const;

    void setHandOffset(HandBone bone, float v);
    void setHandOffset(HandRotation r);
    HandRotation getHandOffset() const;
    void setHandResponse(const HandResponseSettings &settings);
    HandResponseSettings getHandResponse() const;

    bool loadPref();

    bool loadPref(std::string path);

    bool savePref();

    bool savePref(std::string path);

private:
    void parsePacket(const std::vector<uint8_t> &);

    void callListenCallback(UdCapV1MCUPacket packet);
    std::atomic_uint32_t callbackFd = 0;
    std::thread eventLoop;
    std::atomic_bool eventLoopRunning;
    std::queue<UdCapV1MCUPacket> packetQueue;
    std::mutex eventLoopMutex;
    std::condition_variable eventCondition;
    std::function<void()> unlistenPortCallback;
    std::function<void()> unlistenPortConnectionStateCallback;
    std::shared_ptr<PortAccessor> portAccessor;
    mutable std::recursive_mutex stateMutex;
    std::string udCapSerial;
    std::atomic<UdTarget> target = UD_TARGET_UNKNOWN;
    struct CallbackSlot {
        std::function<void(std::shared_ptr<UdCapV1MCUPacket>)> callback;
        std::recursive_mutex executionMutex;
        bool active = true;
    };
    std::map<uint32_t, std::shared_ptr<CallbackSlot>> listenCallbacks;
    std::mutex callbackMutex;
    uint16_t lastBattery = 0;
    uint16_t lastBatteryRaw = 0;
    int16_t lastRssiDbm = 0;
    std::atomic_bool isEnterprise = false;
    std::atomic<UdState> udState = UD_INIT_STATE_INIT;
    std::recursive_mutex streamingMutex;
    bool streamingShutdown = false;
    std::atomic<UdCapV1HandCaliStat> caliStat = UDCAP_V1_HAND_CALI_STAT_NONE;
    UdCapV1HandCaliFist caliFist;
    UdCapV1HandCaliAdduction caliAdduction;
    UdCapV1HandCaliProtract caliProtract;
    UdCapV1Algorithm algorithm = UdCapV1Algorithm::DEFAULT;
    CommunityV1Calibration communityCalibration;
    bool communityCalibrationNeedsReset = true;
    UdCapV1HandData lastAngle{};
    UdCapV1LastCaliData lastCaliData;
    std::atomic<UdCapV1JoystickCaliStat> joystickCaliStat = UDCAP_V1_JOYSTICK_CALI_STAT_OK;
    float xCenterData = 1850.0;
    float yCenterData = 1850.0;
    float xMaxData = 3750.0;
    float xMinData = 620.0;
    float yMaxData = 3750.0;
    float yMinData = 620.0;
    float deadZone = 0.15;
    float triggerMax = 0.5f;
    float triggerMin = 0.25f;
    float gripMax = 1.0f;
    float gripMin = 0.5f;
    float trackpadMax = 1.0f;
    float trackpadMin = 0.5f;
    HandRotation handOffset{};
    HandResponseSettings handResponse = defaultHandResponse();
    bool thumbOn = true;
    // Official thumb root Y, Z and X gains, respectively.
    std::array<float, 3> thumbFix{0.3f, 0.3f, 1.2f};

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
