//
// Created by max_3 on 2025/5/11.
//

#include "UdCapV1Core.h"
#include "CorePref.h"
#include "OfficialCalibration.h"
#include "UdCapSerial.h"
#include <AR1Linear.hpp>
#include <filesystem>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>

#define M_PI 3.14159265358979323846
ImportAR1Linear(AR1LinearAA)
ImportAR1Linear(AR1LinearAD)
ImportAR1Linear(AR1LinearAE)
ImportAR1Linear(AR1LinearAG)

uint8_t calculateCRC(const std::vector<uint8_t> &data, bool ignoreLast) {
    uint8_t crc = 0;
    size_t lastIndex = ignoreLast ? data.size() - 1 : data.size();
    for (size_t i = 2; i < lastIndex; i++) {
        crc += data[i];
    }
    return crc;
}

std::vector<uint8_t> decodeXOR(std::vector<uint8_t> data) {
    std::vector<uint8_t> result;
    for (auto &byte: data) {
        auto u = byte ^ 1;
        u ^= 128;
        result.push_back(u);
    }
    return result;
}

BoneQuaternion UdCapV1Core::eulerToQuaternion(double pitch, double yaw, double roll) {
    const double x = pitch * M_PI / 180.0;
    const double y = yaw * M_PI / 180.0;
    const double z = roll * M_PI / 180.0;
    // Match the official v0.1.8.6 OSCManager.EulerToQuaternion formula:
    // the Unity-axis joint rotation is composed Z * Y * X.
    Eigen::Quaterniond q =
            Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX());
    BoneQuaternion boneQ{};
    boneQ.x = q.x();
    boneQ.y = q.y();
    boneQ.z = q.z();
    boneQ.w = q.w();
    return boneQ;
}

HandQuaternion UdCapV1Core::officialHandPose(const std::array<double, 28> &angles,
                                             UdTarget hand, const HandRotation &offset,
                                             const std::array<float, 3> &thumbFix) {
    const bool left = hand == UD_TARGET_LEFT_HAND;
    const double curl = left ? -1.0 : 1.0;
    const auto make = [](double x, double y, double z, const BoneRotation &o) {
        return eulerToQuaternion(x + o.x, y + o.y, z + o.z);
    };
    HandQuaternion pose{};
    pose.indexFinger.proximal = make(angles[21], -curl * angles[7], curl * angles[6], offset.indexFinger.proximal);
    pose.indexFinger.intermediate = make(0, 0, curl * angles[5], offset.indexFinger.intermediate);
    pose.indexFinger.distal = make(0, 0, curl * angles[4], offset.indexFinger.distal);

    pose.middleFinger.proximal = make(0, angles[11], curl * angles[10], offset.middleFinger.proximal);
    pose.middleFinger.intermediate = make(0, 0, curl * angles[9], offset.middleFinger.intermediate);
    pose.middleFinger.distal = make(0, 0, curl * angles[8], offset.middleFinger.distal);

    pose.ringFinger.proximal = make(0, curl * angles[15], curl * angles[14], offset.ringFinger.proximal);
    pose.ringFinger.intermediate = make(0, 0, curl * angles[13], offset.ringFinger.intermediate);
    pose.ringFinger.distal = make(0, 0, curl * angles[12], offset.ringFinger.distal);

    pose.littleFinger.proximal = make(angles[22], curl * angles[19], curl * angles[18], offset.littleFinger.proximal);
    pose.littleFinger.intermediate = make(0, 0, curl * angles[17], offset.littleFinger.intermediate);
    pose.littleFinger.distal = make(0, 0, curl * angles[16], offset.littleFinger.distal);

    const double thumbSide = left ? 1.0 : -1.0;
    // The official v0.1.8.6 VMC pose uses the same X direction on both
    // hands, and mirrors only the thumb Y/Z axes.
    pose.thumbFinger.proximal = make(angles[3] * thumbFix[2],
                                     thumbSide * angles[2] * thumbFix[0],
                                     -thumbSide * angles[20] * thumbFix[1],
                                     offset.thumbFinger.proximal);
    pose.thumbFinger.intermediate = make(0, thumbSide * angles[1], 0, offset.thumbFinger.intermediate);
    pose.thumbFinger.distal = make(0, thumbSide * angles[0], 0, offset.thumbFinger.distal);
    return pose;
}

std::vector<std::vector<uint8_t> > UdCapHandV1PacketRealignmentHelper::processPacket(const std::vector<uint8_t> &data) {
    std::vector<std::vector<uint8_t> > packets;
    for (auto &byte: data) {
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
                    this->stateMachine = byte == 170 ? GET_170 : NOOP;
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
                this->stateMachine = remainedLength == 0 ? PACKET_CRC : PACKET_TRANSFER;
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

UdCapV1Core::UdCapV1Core(std::shared_ptr<PortAccessor> portAccessor): eventLoopRunning(true), portAccessor(std::move(portAccessor)) {
    lastCaliData.hasLast = false;
    try {
        this->portAccessor->openPort();
        this->portAccessor->setBaudRate(115200);
        // Probing uses 8-byte reads; restore the normal packet-sized buffer.
        this->portAccessor->setReadSize(64);
        if (!this->portAccessor->hasPacketRealignmentHelper()) {
            this->portAccessor->setPacketRealignmentHelper(std::make_unique<UdCapHandV1PacketRealignmentHelper>());
        }
        unlistenPortConnectionStateCallback = this->portAccessor->addConnectionCallback([this](PortAccessorConnectionState state) {
            std::lock_guard stateGuard(stateMutex);
            if (state == PORT_ACCESSOR_CONNECTION_STATE_DISCONNECTED) {
                lastBattery = 0;
                lastBatteryRaw = 0;
                lastRssiDbm = 0;
                if (udState != UD_INIT_STATE_INIT) {
                    udState = UD_INIT_STATE_NOT_CONNECT;
                    if (caliStat == UDCAP_V1_HAND_CALI_STAT_COMPLETED) {
                        caliStat = UDCAP_V1_HAND_CALI_STAT_NONE;
                    }
                    UdCapV1MCUPacket linkPacket{};
                    linkPacket.address = 1;
                    linkPacket.commandType = CommandType::CMD_LINK_STATE;
                    linkPacket.udState = UD_INIT_STATE_NOT_CONNECT;
                    callListenCallback(linkPacket);
                }
            }
            UdCapV1MCUPacket packet{};
            packet.address = 0;
            packet.commandType = CommandType::CMD_PORT_CONNECTION_STATE;
            packet.portConnectionState = state;
            callListenCallback(packet);
            if (state == PORT_ACCESSOR_CONNECTION_STATE_CONNECTED) mcuGetLinkState();
        });
        unlistenPortCallback = this->portAccessor->addDataCallback([this](std::shared_ptr<std::vector<uint8_t>> data) {
            parsePacket(*data);
        });
        eventLoop = std::thread([this]() {
            while (true) {
                UdCapV1MCUPacket packet;
                {
                    std::unique_lock lk(eventLoopMutex);
                    eventCondition.wait(lk, [this] { return !eventLoopRunning.load() || !packetQueue.empty(); });
                    if (!eventLoopRunning.load()) break;
                    packet = std::move(packetQueue.front());
                    packetQueue.pop();
                }
                std::shared_ptr<UdCapV1MCUPacket> packetPtr = std::make_shared<UdCapV1MCUPacket>(packet);
                std::vector<std::shared_ptr<CallbackSlot>> callbacks;
                {
                    std::lock_guard guard(callbackMutex);
                    callbacks.reserve(listenCallbacks.size());
                    for (const auto &[id, slot] : listenCallbacks) callbacks.push_back(slot);
                }
                for (const auto &slot : callbacks) {
                    std::lock_guard guard(slot->executionMutex);
                    if (slot->active) slot->callback(packetPtr);
                }
            }
        });
        this->portAccessor->startContinuousRead();
    } catch (...) {
        if (unlistenPortCallback) unlistenPortCallback();
        if (unlistenPortConnectionStateCallback) unlistenPortConnectionStateCallback();
        this->portAccessor->stopContinuousRead();
        eventLoopRunning.store(false);
        eventCondition.notify_all();
        if (eventLoop.joinable()) eventLoop.join();
        throw;
    }
}

UdCapV1Core::~UdCapV1Core() {
    try { stopStreamingForShutdown(); } catch (const std::exception &) {}
    if (this->unlistenPortCallback) this->unlistenPortCallback();
    if (this->unlistenPortConnectionStateCallback) this->unlistenPortConnectionStateCallback();
    this->portAccessor->stopContinuousRead();
    this->eventLoopRunning.store(false);
    this->eventCondition.notify_all();
    if (this->eventLoop.joinable()) this->eventLoop.join();
}

std::function<void()> UdCapV1Core::listen(const std::function<void(std::shared_ptr<UdCapV1MCUPacket>)> &callback) {
    auto slot = std::make_shared<CallbackSlot>();
    slot->callback = callback;
    uint32_t fd;
    {
        std::lock_guard guard(callbackMutex);
        fd = callbackFd.fetch_add(1);
        listenCallbacks[fd] = slot;
    }
    return [this, fd]() {
        std::shared_ptr<CallbackSlot> slot;
        {
            std::lock_guard guard(callbackMutex);
            const auto it = listenCallbacks.find(fd);
            if (it == listenCallbacks.end()) return;
            slot = std::move(it->second);
            listenCallbacks.erase(it);
        }
        // A callback may already be in the current event-loop snapshot.
        // Wait for it to finish before the owner can destroy captured state.
        std::lock_guard guard(slot->executionMutex);
        slot->active = false;
    };
}

void UdCapV1Core::callListenCallback(UdCapV1MCUPacket packet) {
    {
        std::lock_guard guard(eventLoopMutex);
        packetQueue.push(std::move(packet));
    }
    eventCondition.notify_one();
}

void UdCapV1Core::sendCommand(uint8_t humanAddress, CommandType commandType, const std::vector<uint8_t> &data) {
    if (data.size() > 255) throw std::invalid_argument("Command payload is too large");
    std::vector<uint8_t> packet;
    packet.push_back(85);
    packet.push_back(170);
    packet.push_back(humanAddress); // humanAddress
    packet.push_back((uint8_t) commandType); // commandType
    packet.push_back((uint8_t) data.size()); // dataLength
    packet.insert(packet.end(), data.begin(), data.end());
    uint8_t crc = calculateCRC(packet, false);
    packet.push_back(crc);
    this->portAccessor->writeData(packet);
}

std::string UdCapV1Core::getUDCapSerial() const {
    std::lock_guard guard(stateMutex);
    return udCapSerial;
}

void UdCapV1Core::parsePacket(const std::vector<uint8_t> &packetBuffer) {
    std::lock_guard guard(stateMutex);
    if (packetBuffer.size() < 7 || packetBuffer[0] != 170 || packetBuffer[1] != 85 ||
        packetBuffer.size() != 7u + packetBuffer[5] || calculateCRC(packetBuffer, true) != packetBuffer.back()) {
        return; // Invalid packet
    }
    uint8_t dataLength = packetBuffer[5];
    std::vector<uint8_t> data(packetBuffer.begin() + 6, packetBuffer.begin() + 6 + dataLength);
    auto acceptGloveSerial = [this, &packetBuffer](const std::string &serial) {
        if (!isUdCapGloveSerial(serial)) return;
        if (udCapSerial == serial) return;
        udCapSerial = serial;
        target = serial.back() == 'L' ? UD_TARGET_LEFT_HAND : UD_TARGET_RIGHT_HAND;
        isEnterprise = serial.substr(0, 2) == "AB";
        UdCapV1MCUPacket packet{};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_SERIAL;
        packet.deviceSerialNum = serial;
        packet.isEnterprise = isEnterprise;
        callListenCallback(packet);
    };
    if (packetBuffer[3] == (uint8_t) CommandType::CMD_LINK_STATE) {
        if (data.empty()) return;
        auto deData = decodeXOR(data);
        if (deData.size() > 1)
            acceptGloveSerial(std::string(deData.begin() + 1, deData.end()));
        short linkState = deData[0];
        UdCapV1MCUPacket packet{};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_LINK_STATE;
        if (linkState == 0) {
            lastBattery = 0;
            lastBatteryRaw = 0;
            lastRssiDbm = 0;
            packet.udState = UdState::UD_INIT_STATE_NOT_CONNECT;

            if (udState != UdState::UD_INIT_STATE_NOT_CONNECT) {
                callListenCallback(packet);
            }
            if (udState != UdState::UD_INIT_STATE_NOT_CONNECT && udState != UdState::UD_INIT_STATE_INIT) {
                udState = packet.udState;
                mcuStopData();
            }
            udState = UdState::UD_INIT_STATE_NOT_CONNECT;
            if (caliStat == UDCAP_V1_HAND_CALI_STAT_COMPLETED) caliStat = UDCAP_V1_HAND_CALI_STAT_NONE;
        } else if (linkState == 1) {
            packet.udState = UdState::UD_INIT_STATE_CONNECTED;

            if (udState != UdState::UD_INIT_STATE_CONNECTED && udState != UdState::UD_INIT_STATE_LINKED) {
                callListenCallback(packet);
            }
            if (udState == UdState::UD_INIT_STATE_NOT_CONNECT || udState == UdState::UD_INIT_STATE_INIT) {
                udState = UdState::UD_INIT_STATE_CONNECTED;
                mcuStartData();
            }
        }
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_SET_CHANNEL_DONE) {
        UdCapV1MCUPacket packet{};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_SET_CHANNEL_DONE;
        callListenCallback(packet);
        mcuGetChannel();
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_GET_CHANNEL) {
        if (data.empty()) return;
        UdCapV1MCUPacket packet{};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_GET_CHANNEL;
        auto deData = decodeXOR(data);
        packet.channel = deData[0];
        callListenCallback(packet);
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_FW_VERSION) {
        UdCapV1MCUPacket packet{};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_FW_VERSION;
        auto deData = decodeXOR(data);
        packet.fwVersion = std::string(deData.begin(), deData.end());
        callListenCallback(packet);
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_BATTERY) {
        if (data.size() < 3) return;
        UdCapV1MCUPacket packet{};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_BATTERY;
        auto deData = decodeXOR(data);
        auto udata = (int16_t) ((deData[0] << 8) | deData[1]);
        if (udata <= 0) return;
        // The current receiver appends signed RSSI and uses these voltage bands.
        int batt = ((udata <= 2180) ? 1 : ((udata <= 2290) ? 2 : ((udata <= 2350) ? 3 : ((udata > 2500) ? 5 : 4))));
        if (batt > lastBattery) {
            batt = ((udata <= 2220) ? 1 : ((udata <= 2330) ? 2 : ((udata <= 2390) ? 3 : ((udata > 2540) ? 5 : 4))));
        }
        lastBattery = batt;
        lastBatteryRaw = static_cast<uint16_t>(udata);
        lastRssiDbm = static_cast<int8_t>(deData[2]);
        packet.battery = batt;
        packet.batteryRaw = lastBatteryRaw;
        packet.rssiDbm = lastRssiDbm;
        callListenCallback(packet);
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_SERIAL) {
        const auto deData = decodeXOR(data);
        acceptGloveSerial(std::string(deData.begin(), deData.end()));
        mcuStartData();
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_SET_CHANNEL) {
        if (data.size() < 2) return;
        UdCapV1MCUPacket packet{};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_SET_CHANNEL;
        auto deData = decodeXOR(data);
        packet.channel = deData[0];
        packet.channelResult = deData[1];
        callListenCallback(packet);
        // The receiver's result code is not its channel. Read back the actual
        // channel after every response, including failed changes.
        mcuGetChannel();
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_PAIRING ||
               packetBuffer[3] == (uint8_t) CommandType::CMD_PAIRING_BOTH) {
        if (data.empty()) return;
        UdCapV1MCUPacket packet{};
        packet.address = packetBuffer[2];
        packet.commandType = static_cast<CommandType>(packetBuffer[3]);
        auto deData = decodeXOR(data);
        int pairResult = deData[0];
        if (pairResult == 10) {
            packet.pairing = true;
            callListenCallback(packet);
        } else if (pairResult == 11) {
            packet.pairing = false;
            callListenCallback(packet);
        }
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_DATA) {
        if (data.size() < 24 || data.size() > 38 || data.size() % 2 != 0) return;
        if (udState == UD_INIT_STATE_INIT) {
            mcuGetLinkState();
            return;
        }
        UdCapV1MCUPacket packet{};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_DATA;
        auto deData = decodeXOR(data);
        // std::vector<uint8_t> deData to std::vector<int16_t>
        std::array<int16_t, 19> iData{};
        for (size_t i = 0; i < deData.size(); i += 2) {
            auto value = (int16_t) ((deData[i] << 8) | deData[i + 1]);
            iData[i / 2] = value;
        }
        packet.angle = iData;
        callListenCallback(packet);
        // Start Calc
        memset(&lastAngle, 0, sizeof(lastAngle));
        bool hasController = false;
        if (deData.size() >= 32) {
            lastAngle.f0 = (((float) (iData[12])) - 1000.0) / 1000.0;
            lastAngle.f1 = (((float) (iData[13])) - 1000.0) / 1000.0;
            lastAngle.f2 = (((float) (iData[14])) - 1000.0) / 1000.0;
            lastAngle.f3 = (((float) (iData[15])) - 1000.0) / 1000.0;
        }
        lastAngle.f4 = (float) (iData[0]);
        lastAngle.f5 = (float) (iData[1]);
        lastAngle.f6 = (float) (iData[2]);
        lastAngle.f7 = (float) (iData[3]);
        lastAngle.f8 = (float) (iData[4]);
        lastAngle.f9 = (float) (iData[5]);
        lastAngle.f10 = (float) (iData[6]);
        lastAngle.f11 = (float) (iData[7]);
        lastAngle.f12 = (float) (iData[8]);
        lastAngle.f13 = (float) (iData[9]);
        lastAngle.f14 = (float) (iData[10]);
        lastAngle.f15 = (float) (iData[11]);
        if (deData.size() >= 38) {
            lastAngle.f16 = (float) (iData[16]);
            lastAngle.f17 = (float) (iData[17]);
            lastAngle.f18 = (float) (iData[18]);
            hasController = true;
        }

        if (caliStat == UDCAP_V1_HAND_CALI_STAT_COMPLETED) {
            double f[12]{
                lastAngle.f4,
                lastAngle.f5,
                lastAngle.f6,
                lastAngle.f7,
                lastAngle.f8,
                lastAngle.f9,
                lastAngle.f10,
                lastAngle.f11,
                lastAngle.f12,
                lastAngle.f13,
                lastAngle.f14,
                lastAngle.f15
            };
            double array2[12]{
                caliAdduction.n4,
                caliAdduction.n5,
                caliAdduction.n6,
                caliAdduction.n7,
                caliAdduction.n8,
                caliAdduction.n9,
                caliAdduction.n10,
                caliAdduction.n11,
                caliAdduction.n12,
                caliAdduction.n13,
                caliAdduction.n14,
                caliAdduction.n15
            };
            double array3[12]{
                caliFist.h4,
                caliFist.h5,
                caliFist.h6,
                caliFist.h7,
                caliProtract.h8,
                caliFist.h9,
                caliFist.h10,
                caliProtract.h11,
                caliFist.h12,
                caliFist.h13,
                caliProtract.h14,
                caliFist.h15
            };
            float nn = 100.0;
            if (algorithm == UdCapV1Algorithm::COUMMUNITY_V1) {
                CommunityV1Calibration::Values raw{};
                if (communityCalibrationNeedsReset) {
                    CommunityV1Calibration::Values low{};
                    CommunityV1Calibration::Values high{};
                    std::copy_n(array2, 12, low.begin());
                    std::copy_n(array3, 12, high.begin());
                    communityCalibration.reset(low, high);
                    communityCalibrationNeedsReset = false;
                }
                std::copy_n(f, 12, raw.begin());
                const auto bounds = communityCalibration.updateBounds(raw);
                std::copy(bounds.low.begin(), bounds.low.end(), array2);
                std::copy(bounds.high.begin(), bounds.high.end(), array3);
            }
            // Both algorithms use the same official low/high normalization and
            // the same model-specific AR1 angle calculation. Community mode
            // only changes the bounds passed to this stage.
            const std::vector<double> array4 = udCapV1HandAutoCali(f, array2, array3, nn);
            double a4 = array4[0];
            double a5 = array4[1];
            double a6 = array4[2];
            double a7 = array4[3];
            double a8 = array4[4];
            double a9 = array4[5];
            double a10 = array4[6];
            double a11 = array4[7];
            double a12 = array4[8];
            double a13 = array4[9];
            double a14 = array4[10];
            double a15 = array4[11];
            std::vector<double> array6
            {
                a4,
                a5,
                a6,
                a7,
                a8,
                a9,
                a10,
                a11,
                a12,
                a13,
                a14,
                a15
            };
            std::string serialNum = getUDCapSerial();
            std::vector<double> list;
            if (serialNum.empty()) {
                list = AR1LinearAA()->sensor2Angle(array6, {});
            } else if (serialNum.size() > 5 && serialNum[5] == 'A') {
                list = AR1LinearAA()->sensor2Angle(array6, {});
            } else if (serialNum.size() > 5 && serialNum[5] == 'B') {
                list = AR1LinearAD()->sensor2Angle(array6, {});
            } else if (serialNum.size() > 5 && serialNum[5] != 'C' && serialNum[5] != 'D') {
                list = AR1LinearAG()->sensor2Angle(array6, {});
            } else {
                list = AR1LinearAE()->sensor2Angle(array6, {});
            }
            if (count < 8) {
                for (int k = 0; k < 23; k++) {
                    filterCount[k][count] = list[k];
                }
                count++;
            } else {
                for (int l = 0; l < 23; l++) {
                    double array7[23]{};
                    for (int m = 0; m < 8; m++) {
                        filterCount[l][m] = filterCount[l][m + 1];
                        array7[l] += 1.0 * filterCount[l][m];
                    }
                    if (list[23] == -1.0 || list[23] == 2.0) {
                        filterCount[l][8] = list[l];
                    } else {
                        filterCount[l][8] = 0.8 * SIGNDATA[static_cast<int>(list[23])][l] + 0.2 * list[l];
                    }
                    list[l] = 2.0 * list[l] + array7[l];
                    list[l] = static_cast<int>(list[l]);
                    list[l] /= 10.0;
                }
            }

            std::array<double, 28> _calibrationDataC{};
            for (int n = 0; n < list.size(); n++) {
                _calibrationDataC[n] = list[n];
            }
            _calibrationDataC[23] = static_cast<float>(list[23]);
            _calibrationDataC[24] = lastAngle.f0;
            _calibrationDataC[25] = lastAngle.f1;
            _calibrationDataC[26] = lastAngle.f2;
            _calibrationDataC[27] = lastAngle.f3;
            _calibrationDataC = applyHandResponse(_calibrationDataC, handResponse);
            if (udState == UdState::UD_INIT_STATE_LINKED) {
                {
                    UdCapV1MCUPacket dataPacket{};
                    dataPacket.address = packetBuffer[2];
                    dataPacket.commandType = CommandType::CMD_ANGLE;
                    dataPacket.result = _calibrationDataC;
                    dataPacket.angleBoneOffset = handOffset;
                    dataPacket.angleHand = target.load();
                    callListenCallback(dataPacket);
                }
                {
                    UdCapV1MCUPacket packetSkeleton{};
                    packetSkeleton.address = packetBuffer[2];
                    packetSkeleton.commandType = CommandType::CMD_SKELETON_QUATERNION;

                    packetSkeleton.skeletonQuaternion = officialHandPose(
                        _calibrationDataC, target.load(), handOffset, thumbFix);

                    callListenCallback(packetSkeleton);
                }
                {
                    if (joystickCaliStat == UDCAP_V1_JOYSTICK_CALI_STAT_CAPTURE_ZONE) {
                        if (lastAngle.f16 > 600.0) {
                            xMinData = std::min(xMinData, lastAngle.f16);
                        }
                        if (lastAngle.f16 < 3800.0) {
                            xMaxData = std::max(xMaxData, lastAngle.f16);
                        }
                        if (lastAngle.f17 > 600.0) {
                            yMinData = std::min(yMinData, lastAngle.f17);
                        }
                        if (lastAngle.f17 < 3800.0) {
                            yMaxData = std::max(yMaxData, lastAngle.f17);
                        }
                    } else if (joystickCaliStat == UDCAP_V1_JOYSTICK_CALI_STAT_OK) {
                        float num = (lastAngle.f16 - xCenterData) / ((xMaxData - xMinData) / 2.0);
                        float num2 = (lastAngle.f17 - yCenterData) / ((yMaxData - yMinData) / 2.0);
                        if (num > 1.0) {
                            num = 1.0;
                        }
                        if (num < -1.0) {
                            num = -1.0;
                        }
                        if (num2 > 1.0) {
                            num2 = 1.0;
                        }
                        if (num2 < -1.0) {
                            num2 = -1.0;
                        }
                        if (num <= deadZone && num > 0.0 - deadZone && num2 <= deadZone && num2 > 0.0 - deadZone) {
                            num = 0.0;
                            num2 = 0.0;
                        }
                        UdCapV1MCUPacket joyStickPacket{};
                        joyStickPacket.address = packetBuffer[2];
                        joyStickPacket.commandType = CommandType::CMD_INPUT_JOYSTICK;
                        joyStickPacket.joystickData.joyX = num;
                        joyStickPacket.joystickData.joyY = num2;
                        if (!hasController) {
                            joyStickPacket.joystickData.joyX = 0.0;
                            joyStickPacket.joystickData.joyY = 0.0;
                        }
                        callListenCallback(joyStickPacket);
                    }
                }
                {
                    UdCapV1MCUPacket packetInputBtn{};
                    packetInputBtn.address = packetBuffer[2];
                    packetInputBtn.commandType = CommandType::CMD_INPUT_BUTTON;
                    uint16_t num3 = static_cast<uint16_t>(lastAngle.f18);
                    constexpr uint16_t A_BIT     = 0x0001; // 0000 0000 0000 0001
                    constexpr uint16_t B_BIT     = 0x0002; // 0000 0000 0000 0010
                    constexpr uint16_t JOY_BIT   = 0x0004; // 0000 0000 0000 0100
                    constexpr uint16_t POWER_BIT = 0x0008; // 0000 0000 0000 1000
                    packetInputBtn.button.btnA = num3 & A_BIT;
                    packetInputBtn.button.btnB = num3 & B_BIT;
                    if (packetInputBtn.button.btnA && packetInputBtn.button.btnB) {
                        packetInputBtn.button.btnMenu = true;
                        packetInputBtn.button.btnA = false;
                        packetInputBtn.button.btnB = false;
                    }
                    packetInputBtn.button.btnJoyStick = num3 & JOY_BIT;
                    bool isPower = num3 & POWER_BIT;
                    if (powerBtnPressed) {
                        if (!isPower) {
                            std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - powerButtonTimeout);
                            if (ms.count() <= 1500) {
                                packetInputBtn.button.btnPower = true;
                            }
                            powerBtnPressed = false;
                        }
                    } else {
                        if (isPower) {
                            powerButtonTimeout = std::chrono::system_clock::now();
                            powerBtnPressed = true;
                        }
                    }
                    if (packetInputBtn.button.btnA && packetInputBtn.button.btnB) {
                        packetInputBtn.button.trigger = 0.0;
                        packetInputBtn.button.btnTrigger = false;
                    } else {
                        float triggerFinger = abs(_calibrationDataC[5]);
                        if (triggerFinger > triggerMin * 100.0f && triggerFinger < triggerMax * 100.0f) {
                            float value = (triggerFinger - triggerMin * 100.0f) / (triggerMax * 100.0f - triggerMin * 100.0f);
                            packetInputBtn.button.trigger = value;
                            packetInputBtn.button.btnTrigger = false;
                        } else if (triggerFinger >= triggerMax * 100.0f) {
                            packetInputBtn.button.trigger = 1.0f;
                            packetInputBtn.button.btnTrigger = true;
                        } else if (triggerFinger <= triggerMin * 100.0f) {
                            packetInputBtn.button.trigger = 0.0f;
                            packetInputBtn.button.btnTrigger = false;
                        }
                    }
                    float gripFinger1 = abs(_calibrationDataC[9]);
                    float gripFinger2 = abs(_calibrationDataC[13]);
                    if (gripFinger1 >= gripMin * 100.0f && gripFinger2 >= gripMin * 100.0f) {
                        float value = (gripFinger1 >= gripMax * 100.0f || gripFinger2 >= gripMax * 100.0f) ? 1.0f : std::max((gripFinger1 - gripMin * 100.0f) / (gripMax * 100.0f - gripMin * 100.0f), (gripFinger2 - gripMin * 100.0f) / (gripMax * 100.0f - gripMin * 100.0f));
                        packetInputBtn.button.grip = value;
                        packetInputBtn.button.btnGrip = true;
                    } else {
                        packetInputBtn.button.grip = 0.0f;
                        packetInputBtn.button.btnGrip = false;
                    }
                    float trackpadFinger = abs(_calibrationDataC[0]);
                    if (trackpadFinger >= trackpadMin * 60.0f) {
                        float value = trackpadFinger >= trackpadMax * 60.0f ? 1.0f : (trackpadFinger - trackpadMin * 60.0f) / (trackpadMax * 60.0f - trackpadMin * 60.0f);
                        packetInputBtn.button.trackpad = value;
                        packetInputBtn.button.btnTrackpad = true;
                    } else {
                        packetInputBtn.button.trackpad = 0.0f;
                        packetInputBtn.button.btnTrackpad = false;
                    }
                    callListenCallback(packetInputBtn);
                }
            }
        }
        if (udState == UdState::UD_INIT_STATE_CONNECTED) {
            udState = UdState::UD_INIT_STATE_LINKED;
            UdCapV1MCUPacket linkedPacket{};
            linkedPacket.address = packetBuffer[2];
            linkedPacket.commandType = CommandType::CMD_LINK_STATE;
            linkedPacket.udState = UdState::UD_INIT_STATE_LINKED;
            callListenCallback(linkedPacket);
        }
    }
}

UdTarget UdCapV1Core::getTarget() const {
    return target;
}

void UdCapV1Core::setFallbackTarget(UdTarget hand) {
    if (hand != UD_TARGET_LEFT_HAND && hand != UD_TARGET_RIGHT_HAND) return;
    UdTarget unknown = UD_TARGET_UNKNOWN;
    target.compare_exchange_strong(unknown, hand);
}

UdState UdCapV1Core::getState() const {
    return udState.load();
}

std::optional<std::pair<uint16_t, uint16_t>> UdCapV1Core::getBattery() const {
    std::lock_guard guard(stateMutex);
    if (lastBattery == 0) return std::nullopt;
    return std::pair{lastBattery, lastBatteryRaw};
}

std::optional<int16_t> UdCapV1Core::getRssiDbm() const {
    std::lock_guard guard(stateMutex);
    if (lastRssiDbm == 0) return std::nullopt;
    return lastRssiDbm;
}

void UdCapV1Core::mcuStopData() {
    if (udState != UD_INIT_STATE_INIT) {
        sendCommand(1, CommandType::CMD_STOP_DATA, {1});
    }
}

void UdCapV1Core::stopStreamingForShutdown() {
    std::lock_guard lock(streamingMutex);
    if (streamingShutdown) return;
    streamingShutdown = true;
    if (portAccessor->isOpen())
        sendCommand(1, CommandType::CMD_STOP_DATA, {1});
}

bool UdCapV1Core::waitForPendingWrites(std::chrono::milliseconds timeout) const {
    return portAccessor->waitForPendingWrites(timeout);
}

void UdCapV1Core::mcuStartData() {
    std::lock_guard lock(streamingMutex);
    if (streamingShutdown) return;
    if (udState == UD_INIT_STATE_CONNECTED) mcuResumeData();
}

void UdCapV1Core::mcuResumeData() {
    std::lock_guard lock(streamingMutex);
    if (streamingShutdown) return;
    // A stop may briefly change the glove's link state. The receiver still
    // needs the start packet after the requested delay to resume streaming.
    if (portAccessor->isOpen()) {
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

void UdCapV1Core::mcuGetLinkState() {
    sendCommand(1, CommandType::CMD_LINK_STATE, {1});
}

void UdCapV1Core::mcuGetSerialNum() {
    sendCommand(1, CommandType::CMD_SERIAL, {1});
}

void UdCapV1Core::mcuStartPairing() {
    sendCommand(1, CommandType::CMD_PAIRING, {1});
}

void UdCapV1Core::mcuStopPairing() {
    sendCommand(1, CommandType::CMD_PAIRING, {2});
}

void UdCapV1Core::mcuStartPairingBoth(uint8_t timeoutSeconds) {
    if (timeoutSeconds == 0) throw std::invalid_argument("Pairing timeout must be greater than zero");
    sendCommand(1, CommandType::CMD_PAIRING_BOTH, {1, timeoutSeconds});
}

void UdCapV1Core::mcuStopPairingBoth() {
    sendCommand(1, CommandType::CMD_PAIRING_BOTH, {2, 0});
}

void UdCapV1Core::mcuSendVibration(int index, float second, int strength) {
    if (index != 1 && index != 2) return;
    float rSecond = ((second <= 0.04f) ? 0.04f : ((second > 2.5f) ? 2.5f : second));
    int rStrength = ((strength < 4) ? 4 : ((strength > 10) ? 10 : strength));
    sendCommand(1, CommandType::CMD_VIBRATION, { (uint8_t)index, 0, (uint8_t)(rSecond * 100.0), (uint8_t)rStrength });
}

void UdCapV1Core::setTriggerButtonMin(float value) {
    std::lock_guard guard(stateMutex);
    this->triggerMin = value;
}
void UdCapV1Core::setTriggerButtonMax(float value) {
    std::lock_guard guard(stateMutex);
    this->triggerMax = value;
}
float UdCapV1Core::getTriggerButtonMin() const {
    std::lock_guard guard(stateMutex);
    return triggerMin;
}
float UdCapV1Core::getTriggerButtonMax() const {
    std::lock_guard guard(stateMutex);
    return triggerMax;
}
void UdCapV1Core::setGripButtonMin(float value) {
    std::lock_guard guard(stateMutex);
    this->gripMin = value;
}
void UdCapV1Core::setGripButtonMax(float value) {
    std::lock_guard guard(stateMutex);
    this->gripMax = value;
}
float UdCapV1Core::getGripButtonMin() const {
    std::lock_guard guard(stateMutex);
    return gripMin;
}
float UdCapV1Core::getGripButtonMax() const {
    std::lock_guard guard(stateMutex);
    return gripMax;
}
void UdCapV1Core::setTrackpadButtonMin(float value) {
    std::lock_guard guard(stateMutex);
    this->trackpadMin = value;
}
void UdCapV1Core::setTrackpadButtonMax(float value) {
    std::lock_guard guard(stateMutex);
    this->trackpadMax = value;
}
float UdCapV1Core::getTrackpadButtonMin() const {
    std::lock_guard guard(stateMutex);
    return trackpadMin;
}
float UdCapV1Core::getTrackpadButtonMax() const {
    std::lock_guard guard(stateMutex);
    return trackpadMax;
}

std::string UdCapV1Core::fromUdStateToString(UdState state) {
    switch (state) {
        case UD_INIT_STATE_INIT:
            return "Init";
        case UD_INIT_STATE_NOT_CONNECT:
            return "Not Connect";
        case UD_INIT_STATE_CONNECTED:
            return "Connected";
        case UD_INIT_STATE_LINKED:
            return "Linked";
        default:
            return "Invalid State";
    }
}

void UdCapV1Core::runCalibration(UdCapV1DeviceCaliType type) {
    std::lock_guard guard(stateMutex);
    if (udState != UD_INIT_STATE_LINKED) {
        throw std::runtime_error("Core not initialized");
    }
    if (type == UdCapV1DeviceCaliType::UDCAP_V1_DEVICE_CALI_TYPE_HAND) {
        if (caliStat != UDCAP_V1_HAND_CALI_STAT_NONE) {
            throw std::runtime_error("Calibration already called");
        }
        caliStat = UDCAP_V1_HAND_CALI_STAT_AUTO;
        communityCalibrationNeedsReset = true;
        memset(&caliFist, 0, sizeof(caliFist));
        memset(&caliAdduction, 0, sizeof(caliAdduction));
        memset(&caliProtract, 0, sizeof(caliProtract));
        caliFist.captured = false;
        caliAdduction.captured = false;
        caliProtract.captured = false;
        UdCapV1MCUPacket caliPacket {};
        caliPacket.address = 1;
        caliPacket.commandType = CommandType::CMD_READY;
        caliPacket.isReady = false;
        callListenCallback(caliPacket);
    } else if (type == UdCapV1DeviceCaliType::UDCAP_V1_DEVICE_CALI_TYPE_JOYSTICK) {
        joystickCaliStat = UdCapV1JoystickCaliStat::UDCAP_V1_JOYSTICK_CALI_STAT_CAPTURE_CENTER;
    }
}

void UdCapV1Core::captureCalibrationData(UdCapV1HandCaliType type) {
    std::lock_guard guard(stateMutex);
    if (udState != UD_INIT_STATE_LINKED) {
        throw std::runtime_error("Core not initialized");
    }
    if (caliStat == UDCAP_V1_HAND_CALI_STAT_NONE) {
        throw std::runtime_error("Calibration not started");
    }
    if (caliStat == UDCAP_V1_HAND_CALI_STAT_COMPLETED) {
        throw std::runtime_error("Calibration already completed");
    }
    if (type == UDCAP_V1_HAND_CALI_TYPE_ALL) {
        throw std::runtime_error("Invalid calibration type");
    } else if (type == UDCAP_V1_HAND_CALI_TYPE_FIST) {
        // TODO improve accuracy
        caliFist.captured = true;
        caliFist.h4 = lastAngle.f4;
        caliFist.h5 = lastAngle.f5;
        caliFist.h6 = lastAngle.f6;
        caliFist.h7 = lastAngle.f7;
        caliFist.h9 = lastAngle.f9;
        caliFist.h10 = lastAngle.f10;
        caliFist.h12 = lastAngle.f12;
        caliFist.h13 = lastAngle.f13;
        caliFist.h15 = lastAngle.f15;
    } else if (type == UDCAP_V1_HAND_CALI_TYPE_ADDUCTION) {
        // TODO improve accuracy
        caliAdduction.captured = true;
        caliAdduction.n1 = lastAngle.f1;
        caliAdduction.n2 = lastAngle.f2;
        caliAdduction.n3 = lastAngle.f3;
        caliAdduction.n4 = lastAngle.f4;
        caliAdduction.n5 = lastAngle.f5;
        caliAdduction.n6 = lastAngle.f6;
        caliAdduction.n7 = lastAngle.f7;
        caliAdduction.n8 = lastAngle.f8;
        caliAdduction.n9 = lastAngle.f9;
        caliAdduction.n10 = lastAngle.f10;
        caliAdduction.n11 = lastAngle.f11;
        caliAdduction.n12 = lastAngle.f12;
        caliAdduction.n13 = lastAngle.f13;
        caliAdduction.n14 = lastAngle.f14;
        caliAdduction.n15 = lastAngle.f15;
    } else if (type == UDCAP_V1_HAND_CALI_TYPE_PROTRACT) {
        // TODO improve accuracy
        caliProtract.captured = true;
        caliProtract.h8 = lastAngle.f8;
        caliProtract.h11 = lastAngle.f11;
        caliProtract.h14 = lastAngle.f14;
    }
}

void UdCapV1Core::clearCalibrationData(UdCapV1HandCaliType type) {
    std::lock_guard guard(stateMutex);
    communityCalibrationNeedsReset = true;
    if (udState != UD_INIT_STATE_LINKED) {
        throw std::runtime_error("Core not initialized");
    }
        if (caliStat == UDCAP_V1_HAND_CALI_STAT_COMPLETED) {
            if (type != UDCAP_V1_HAND_CALI_TYPE_ALL) {
            throw std::runtime_error("Can not clear single calibration data after completed");
            } else {
                caliStat = UDCAP_V1_HAND_CALI_STAT_NONE;
                lastCaliData.hasLast = false;
            memset(&caliFist, 0, sizeof(caliFist));
            memset(&caliAdduction, 0, sizeof(caliAdduction));
            memset(&caliProtract, 0, sizeof(caliProtract));
            caliFist.captured = false;
            caliAdduction.captured = false;
            caliProtract.captured = false;
            UdCapV1MCUPacket caliPacket {};
            caliPacket.address = 1;
            caliPacket.commandType = CommandType::CMD_READY;
            caliPacket.isReady = false;
            callListenCallback(caliPacket);
            return;
        }
    }
    if (type == UDCAP_V1_HAND_CALI_TYPE_ALL) {
        lastCaliData.hasLast = false;
        memset(&caliFist, 0, sizeof(caliFist));
        memset(&caliAdduction, 0, sizeof(caliAdduction));
        memset(&caliProtract, 0, sizeof(caliProtract));
        caliFist.captured = false;
        caliAdduction.captured = false;
        caliProtract.captured = false;
    } else if (type == UDCAP_V1_HAND_CALI_TYPE_FIST) {
        memset(&caliFist, 0, sizeof(caliFist));
        caliFist.captured = false;
    } else if (type == UDCAP_V1_HAND_CALI_TYPE_ADDUCTION) {
        memset(&caliAdduction, 0, sizeof(caliAdduction));
        caliAdduction.captured = false;
    } else if (type == UDCAP_V1_HAND_CALI_TYPE_PROTRACT) {
        memset(&caliProtract, 0, sizeof(caliProtract));
        caliProtract.captured = false;
    }
}

void UdCapV1Core::completeCalibration(UdCapV1DeviceCaliType type) {
    std::lock_guard guard(stateMutex);
    if (udState != UD_INIT_STATE_LINKED) {
        throw std::runtime_error("Core not initialized");
    }
    if (type == UdCapV1DeviceCaliType::UDCAP_V1_DEVICE_CALI_TYPE_HAND) {
        if (caliStat == UDCAP_V1_HAND_CALI_STAT_COMPLETED) {
            throw std::runtime_error("Calibration already completed");
        }
        if (caliStat == UDCAP_V1_HAND_CALI_STAT_NONE) {
            throw std::runtime_error("Calibration not started");
        }
        if (!caliFist.captured) {
            throw std::runtime_error("Fist calibration not captured");
        }
        if (!caliAdduction.captured) {
            throw std::runtime_error("Adduction calibration not captured");
        }
        if (!caliProtract.captured) {
            throw std::runtime_error("Protract calibration not captured");
        }
        double f[12]{
            lastAngle.f4,
            lastAngle.f5,
            lastAngle.f6,
            lastAngle.f7,
            lastAngle.f8,
            lastAngle.f9,
            lastAngle.f10,
            lastAngle.f11,
            lastAngle.f12,
            lastAngle.f13,
            lastAngle.f14,
            lastAngle.f15
        };
        double array2[12]{
            caliAdduction.n4,
            caliAdduction.n5,
            caliAdduction.n6,
            caliAdduction.n7,
            caliAdduction.n8,
            caliAdduction.n9,
            caliAdduction.n10,
            caliAdduction.n11,
            caliAdduction.n12,
            caliAdduction.n13,
            caliAdduction.n14,
            caliAdduction.n15
        };
        double array3[12]{
            caliFist.h4,
            caliFist.h5,
            caliFist.h6,
            caliFist.h7,
            caliProtract.h8,
            caliFist.h9,
            caliFist.h10,
            caliProtract.h11,
            caliFist.h12,
            caliFist.h13,
            caliProtract.h14,
            caliFist.h15
        };
        // The device's original calibration accepts a usable movement on any
        // finger channel; requiring every sensor to exceed the threshold makes
        // an otherwise working glove fail because of one weak channel.
        bool calibrationSuccess = false;
        for (int i = 0; i < 12; i++) {
            if (std::abs(array3[i] - array2[i]) > 25.0) {
                calibrationSuccess = true;
                break;
            }
        }
        if (!calibrationSuccess) {
            caliStat = UDCAP_V1_HAND_CALI_STAT_NONE;
            UdCapV1MCUPacket caliPacket {};
            caliPacket.address = 1;
            caliPacket.commandType = CommandType::CMD_READY;
            caliPacket.isReady = false;
            callListenCallback(caliPacket);
            throw std::runtime_error("Calibration failed");
        }
        lastCaliData.caliFist = caliFist;
        lastCaliData.caliAdduction = caliAdduction;
        lastCaliData.caliProtract = caliProtract;
        lastCaliData.hasLast = true;
        communityCalibrationNeedsReset = true;
        caliStat = UDCAP_V1_HAND_CALI_STAT_COMPLETED;
        UdCapV1MCUPacket caliPacket {};
        caliPacket.address = 1;
        caliPacket.commandType = CommandType::CMD_READY;
        caliPacket.isReady = true;
        callListenCallback(caliPacket);
    } else if (type == UdCapV1DeviceCaliType::UDCAP_V1_DEVICE_CALI_TYPE_JOYSTICK) {
        joystickCaliStat = UdCapV1JoystickCaliStat::UDCAP_V1_JOYSTICK_CALI_STAT_OK;
    }
}

void UdCapV1Core::tryRestoreHandCalibration() {
    std::lock_guard guard(stateMutex);
    if (udState != UD_INIT_STATE_LINKED) {
        throw std::runtime_error("Core not initialized");
    }
    if (caliStat == UDCAP_V1_HAND_CALI_STAT_COMPLETED) {
        return;
    }
    if (caliStat != UDCAP_V1_HAND_CALI_STAT_NONE) {
        throw std::runtime_error("Calibration already in progress");
    }
    if (lastCaliData.hasLast) {
        caliStat = UDCAP_V1_HAND_CALI_STAT_COMPLETED;
        caliFist = lastCaliData.caliFist;
        caliAdduction = lastCaliData.caliAdduction;
        caliProtract = lastCaliData.caliProtract;
        communityCalibrationNeedsReset = true;
        UdCapV1MCUPacket caliPacket {};
        caliPacket.address = 1;
        caliPacket.commandType = CommandType::CMD_READY;
        caliPacket.isReady = true;
        callListenCallback(caliPacket);
    }
}

void UdCapV1Core::captureJoystickData(UdCapV1JoystickCaliType type) {
    std::lock_guard guard(stateMutex);
    if (udState != UD_INIT_STATE_LINKED) {
        throw std::runtime_error("Core not initialized");
    }
    if (joystickCaliStat == UdCapV1JoystickCaliStat::UDCAP_V1_JOYSTICK_CALI_STAT_OK) {
        throw std::runtime_error("Not in calibration mode");
    }
    if (joystickCaliStat == UdCapV1JoystickCaliStat::UDCAP_V1_JOYSTICK_CALI_STAT_CAPTURE_CENTER) {
        if (type == UdCapV1JoystickCaliType::UDCAP_V1_JOYSTICK_CALI_TYPE_CENTER) {
            xCenterData = lastAngle.f16;
            yCenterData = lastAngle.f17;
            joystickCaliStat = UdCapV1JoystickCaliStat::UDCAP_V1_JOYSTICK_CALI_STAT_CAPTURE_ZONE;
        } else {
            throw std::runtime_error("Error on calibrate type with ZONE, now calibrating CENTER");
        }
    } else if (joystickCaliStat == UdCapV1JoystickCaliStat::UDCAP_V1_JOYSTICK_CALI_STAT_CAPTURE_ZONE) {
        if (type == UdCapV1JoystickCaliType::UDCAP_V1_JOYSTICK_CALI_TYPE_ZONE) {
            std::cout << "Joystick zone calibration is running on data callback automatically." << std::endl;
        } else {
            throw std::runtime_error("Error on calibrate type with CENTER, now calibrating ZONE");
        }
    }
}

void UdCapV1Core::clearJoystickData(UdCapV1JoystickCaliType type) {
    std::lock_guard guard(stateMutex);
    if (udState != UD_INIT_STATE_LINKED) {
        throw std::runtime_error("Core not initialized");
    }
    if (joystickCaliStat == UdCapV1JoystickCaliStat::UDCAP_V1_JOYSTICK_CALI_STAT_OK) {
        throw std::runtime_error("Not in calibration mode");
    }
    if (type == UDCAP_V1_JOYSTICK_CALI_TYPE_ALL) {
        xCenterData = Default_xCenterData;
        yCenterData = Default_yCenterData;
        xMinData = Default_xMinData;
        yMinData = Default_yMinData;
        xMaxData = Default_xMaxData;
        yMaxData = Default_yMaxData;
    } else if (type == UDCAP_V1_JOYSTICK_CALI_TYPE_CENTER) {
        xCenterData = Default_xCenterData;
        yCenterData = Default_yCenterData;
    } else if (type == UDCAP_V1_JOYSTICK_CALI_TYPE_ZONE) {
        xMinData = Default_xMinData;
        yMinData = Default_yMinData;
        xMaxData = Default_xMaxData;
        yMaxData = Default_yMaxData;
    }
    joystickCaliStat = UdCapV1JoystickCaliStat::UDCAP_V1_JOYSTICK_CALI_STAT_OK;
}

UdCapV1HandCaliStat UdCapV1Core::getHandCalibrationStatus() const {
    std::lock_guard guard(stateMutex);
    return caliStat;
}

void UdCapV1Core::setAlgorithm(UdCapV1Algorithm value) {
    if (value != UdCapV1Algorithm::DEFAULT &&
        value != UdCapV1Algorithm::ORIGINAL_V1 &&
        value != UdCapV1Algorithm::COUMMUNITY_V1) {
        throw std::invalid_argument("Unknown hand calibration algorithm");
    }
    std::lock_guard guard(stateMutex);
    if (algorithm != value) {
        algorithm = value;
        communityCalibrationNeedsReset = true;
    }
}

UdCapV1Algorithm UdCapV1Core::getAlgorithm() const {
    std::lock_guard guard(stateMutex);
    return algorithm;
}

UdCapV1JoystickCaliStat UdCapV1Core::getJoystickCalibrationStatus() const {
    std::lock_guard guard(stateMutex);
    return joystickCaliStat;
}

void UdCapV1Core::mcuSetChannel(uint8_t channel) {
    sendCommand(1, CommandType::CMD_SET_CHANNEL, { channel });
}

void UdCapV1Core::mcuGetChannel() {
    sendCommand(1, CommandType::CMD_GET_CHANNEL, { 1 });
}

void UdCapV1Core::mcuReset() {
    sendCommand(1, (CommandType)10, { 1 });
}

void UdCapV1Core::mcuGetFirmwareVersion() {
    sendCommand(1, CommandType::CMD_FW_VERSION, { 1 });
}

void UdCapV1Core::setHandOffset(HandBone bone, float v) {
    std::lock_guard guard(stateMutex);
    if (!std::isfinite(v) || v < -45.0f || v > 45.0f) throw std::runtime_error("Hand offset value must be between -45 and 45 degrees");
    switch (bone) {
        case HandBone::HAND_BONE_THUMB_PROXIMAL:
            handOffset.thumbFinger.proximal.x = v;
            break;
        case HandBone::HAND_BONE_THUMB_INTERMEDIATE:
            handOffset.thumbFinger.intermediate.x = v;
            break;
        case HandBone::HAND_BONE_THUMB_DISTAL:
            handOffset.thumbFinger.distal.x = v;
            break;
        case HandBone::HAND_BONE_INDEX_PROXIMAL:
            handOffset.indexFinger.proximal.x = v;
            break;
        case HandBone::HAND_BONE_INDEX_INTERMEDIATE:
            handOffset.indexFinger.intermediate.x = v;
            break;
        case HandBone::HAND_BONE_INDEX_DISTAL:
            handOffset.indexFinger.distal.x = v;
            break;
        case HandBone::HAND_BONE_MIDDLE_PROXIMAL:
            handOffset.middleFinger.proximal.x = v;
            break;
        case HandBone::HAND_BONE_MIDDLE_INTERMEDIATE:
            handOffset.middleFinger.intermediate.x = v;
            break;
        case HandBone::HAND_BONE_MIDDLE_DISTAL:
            handOffset.middleFinger.distal.x = v;
            break;
        case HandBone::HAND_BONE_RING_PROXIMAL:
            handOffset.ringFinger.proximal.x = v;
            break;
        case HandBone::HAND_BONE_RING_INTERMEDIATE:
            handOffset.ringFinger.intermediate.x = v;
            break;
        case HandBone::HAND_BONE_RING_DISTAL:
            handOffset.ringFinger.distal.x = v;
            break;
        case HandBone::HAND_BONE_LITTLE_PROXIMAL:
            handOffset.littleFinger.proximal.x = v;
            break;
        case HandBone::HAND_BONE_LITTLE_INTERMEDIATE:
            handOffset.littleFinger.intermediate.x = v;
            break;
        case HandBone::HAND_BONE_LITTLE_DISTAL:
            handOffset.littleFinger.distal.x = v;
            break;
    }
}

void UdCapV1Core::setHandOffset(HandRotation r) {
    std::lock_guard guard(stateMutex);
    const FingerRotation fingers[]{r.thumbFinger, r.indexFinger, r.middleFinger, r.ringFinger, r.littleFinger};
    for (const auto &finger : fingers) {
        const BoneRotation bones[]{finger.proximal, finger.intermediate, finger.distal};
        for (const auto &bone : bones)
            for (float value : {bone.x, bone.y, bone.z})
                if (!std::isfinite(value) || std::abs(value) > 45)
                    throw std::invalid_argument("Hand offsets must be finite and between -45 and 45 degrees");
    }
    handOffset = r;
}

HandRotation UdCapV1Core::getHandOffset() const {
    std::lock_guard guard(stateMutex);
    return handOffset;
}

void UdCapV1Core::setHandResponse(const HandResponseSettings &settings) {
    validateHandResponse(settings);
    std::lock_guard guard(stateMutex);
    handResponse = settings;
}

HandResponseSettings UdCapV1Core::getHandResponse() const {
    std::lock_guard guard(stateMutex);
    return handResponse;
}

bool UdCapV1Core::loadPref() {
    std::lock_guard guard(stateMutex);
    std::filesystem::path dirPath = CorePref::getInstance().getPrefPath();
    if (!std::filesystem::exists(dirPath)) {
        if (!std::filesystem::create_directories(dirPath)) {
            return false;
        }
    }
    std::filesystem::path file = dirPath / (getUDCapSerial() + ".json");
    return loadPref(file.string());
}

bool UdCapV1Core::loadPref(std::string path) {
    std::lock_guard guard(stateMutex);
    std::filesystem::path filePath(path);
    std::filesystem::path parentPath = filePath.parent_path();
    if (!parentPath.empty() && !std::filesystem::exists(parentPath)) {
        return false;
    }
    if (!std::filesystem::exists(path)) {
        return false;
    }
    if (!std::filesystem::is_regular_file(path)) {
        return false;
    }
    // Read the file and parse the JSON content UTF-8
    std::ifstream inFile(path);
    if (!inFile.is_open()) {
        return false;
    }
    try {
        nlohmann::json j = nlohmann::json::parse(inFile);
        inFile.close();
        auto loadedResponse = defaultHandResponse();
        if (j.contains("handResponse")) {
            const auto &curves = j.at("handResponse");
            if (!curves.is_array() || curves.size() != loadedResponse.size()) return false;
            for (std::size_t i = 0; i < loadedResponse.size(); ++i) {
                const auto &c = curves.at(i);
                loadedResponse[i] = {c.at("min").get<double>(), c.at("max").get<double>(),
                    c.at("balance").get<double>(), c.value("offset", 0.0)};
            }
            validateHandResponse(loadedResponse);
        }
        auto loadedOffset = HandRotation{};
        if (j.contains("controller")) {
            auto controller = j["controller"];
            if (controller.contains("xCenterData")) xCenterData = controller["xCenterData"].get<float>();
            if (controller.contains("yCenterData")) yCenterData = controller["yCenterData"].get<float>();
            if (controller.contains("xMaxData")) xMaxData = controller["xMaxData"].get<float>();
            if (controller.contains("xMinData")) xMinData = controller["xMinData"].get<float>();
            if (controller.contains("yMaxData")) yMaxData = controller["yMaxData"].get<float>();
            if (controller.contains("yMinData")) yMinData = controller["yMinData"].get<float>();
            if (controller.contains("deadZone")) deadZone = controller["deadZone"].get<float>();
            if (controller.contains("triggerMax")) triggerMax = controller["triggerMax"].get<float>();
            if (controller.contains("triggerMin")) triggerMin = controller["triggerMin"].get<float>();
            if (controller.contains("gripMax")) gripMax = controller["gripMax"].get<float>();
            if (controller.contains("gripMin")) gripMin = controller["gripMin"].get<float>();
            if (controller.contains("trackpadMax")) trackpadMax = controller["trackpadMax"].get<float>();
            if (controller.contains("trackpadMin")) trackpadMin = controller["trackpadMin"].get<float>();
        }
        if (j.contains("handOffset")) {
            auto controller = j["handOffset"];
            auto thumb = controller["thumbFinger"];
            if (thumb.contains("proximal")) {
                loadedOffset.thumbFinger.proximal.x = thumb["proximal"]["x"].get<float>();
                loadedOffset.thumbFinger.proximal.y = thumb["proximal"]["y"].get<float>();
                loadedOffset.thumbFinger.proximal.z = thumb["proximal"]["z"].get<float>();
            }
            if (thumb.contains("intermediate")) {
                loadedOffset.thumbFinger.intermediate.x = thumb["intermediate"]["x"].get<float>();
                loadedOffset.thumbFinger.intermediate.y = thumb["intermediate"]["y"].get<float>();
                loadedOffset.thumbFinger.intermediate.z = thumb["intermediate"]["z"].get<float>();
            }
            if (thumb.contains("distal")) {
                loadedOffset.thumbFinger.distal.x = thumb["distal"]["x"].get<float>();
                loadedOffset.thumbFinger.distal.y = thumb["distal"]["y"].get<float>();
                loadedOffset.thumbFinger.distal.z = thumb["distal"]["z"].get<float>();
            }
            auto index = controller["indexFinger"];
            if (index.contains("proximal")) {
                loadedOffset.indexFinger.proximal.x = index["proximal"]["x"].get<float>();
                loadedOffset.indexFinger.proximal.y = index["proximal"]["y"].get<float>();
                loadedOffset.indexFinger.proximal.z = index["proximal"]["z"].get<float>();
            }
            if (index.contains("intermediate")) {
                loadedOffset.indexFinger.intermediate.x = index["intermediate"]["x"].get<float>();
                loadedOffset.indexFinger.intermediate.y = index["intermediate"]["y"].get<float>();
                loadedOffset.indexFinger.intermediate.z = index["intermediate"]["z"].get<float>();
            }
            if (index.contains("distal")) {
                loadedOffset.indexFinger.distal.x = index["distal"]["x"].get<float>();
                loadedOffset.indexFinger.distal.y = index["distal"]["y"].get<float>();
                loadedOffset.indexFinger.distal.z = index["distal"]["z"].get<float>();
            }
            auto middle = controller["middleFinger"];
            if (middle.contains("proximal")) {
                loadedOffset.middleFinger.proximal.x = middle["proximal"]["x"].get<float>();
                loadedOffset.middleFinger.proximal.y = middle["proximal"]["y"].get<float>();
                loadedOffset.middleFinger.proximal.z = middle["proximal"]["z"].get<float>();
            }
            if (middle.contains("intermediate")) {
                loadedOffset.middleFinger.intermediate.x = middle["intermediate"]["x"].get<float>();
                loadedOffset.middleFinger.intermediate.y = middle["intermediate"]["y"].get<float>();
                loadedOffset.middleFinger.intermediate.z = middle["intermediate"]["z"].get<float>();
            }
            if (middle.contains("distal")) {
                loadedOffset.middleFinger.distal.x = middle["distal"]["x"].get<float>();
                loadedOffset.middleFinger.distal.y = middle["distal"]["y"].get<float>();
                loadedOffset.middleFinger.distal.z = middle["distal"]["z"].get<float>();
            }
            auto ring = controller["ringFinger"];
            if (ring.contains("proximal")) {
                loadedOffset.ringFinger.proximal.x = ring["proximal"]["x"].get<float>();
                loadedOffset.ringFinger.proximal.y = ring["proximal"]["y"].get<float>();
                loadedOffset.ringFinger.proximal.z = ring["proximal"]["z"].get<float>();
            }
            if (ring.contains("intermediate")) {
                loadedOffset.ringFinger.intermediate.x = ring["intermediate"]["x"].get<float>();
                loadedOffset.ringFinger.intermediate.y = ring["intermediate"]["y"].get<float>();
                loadedOffset.ringFinger.intermediate.z = ring["intermediate"]["z"].get<float>();
            }
            if (ring.contains("distal")) {
                loadedOffset.ringFinger.distal.x = ring["distal"]["x"].get<float>();
                loadedOffset.ringFinger.distal.y = ring["distal"]["y"].get<float>();
                loadedOffset.ringFinger.distal.z = ring["distal"]["z"].get<float>();
            }
            auto little = controller["littleFinger"];
            if (little.contains("proximal")) {
                loadedOffset.littleFinger.proximal.x = little["proximal"]["x"].get<float>();
                loadedOffset.littleFinger.proximal.y = little["proximal"]["y"].get<float>();
                loadedOffset.littleFinger.proximal.z = little["proximal"]["z"].get<float>();
            }
            if (little.contains("intermediate")) {
                loadedOffset.littleFinger.intermediate.x = little["intermediate"]["x"].get<float>();
                loadedOffset.littleFinger.intermediate.y = little["intermediate"]["y"].get<float>();
                loadedOffset.littleFinger.intermediate.z = little["intermediate"]["z"].get<float>();
            }
            if (little.contains("distal")) {
                loadedOffset.littleFinger.distal.x = little["distal"]["x"].get<float>();
                loadedOffset.littleFinger.distal.y = little["distal"]["y"].get<float>();
                loadedOffset.littleFinger.distal.z = little["distal"]["z"].get<float>();
            }
        }
        setHandOffset(loadedOffset);
        handResponse = loadedResponse;
        return true;
    } catch (const std::exception&) {
        if (inFile.is_open()) inFile.close();
        return false;
    }
    if (inFile.is_open()) inFile.close();
    return false;
}

bool UdCapV1Core::savePref() {
    std::lock_guard guard(stateMutex);
    std::filesystem::path dirPath = CorePref::getInstance().getPrefPath();
    if (!std::filesystem::exists(dirPath)) {
        if (!std::filesystem::create_directories(dirPath)) {
            return false;
        }
    }
    std::filesystem::path file = dirPath / (getUDCapSerial() + ".json");
    return savePref(file.string());
}

bool UdCapV1Core::savePref(std::string path) {
    std::lock_guard guard(stateMutex);
    std::filesystem::path filePath(path);
    std::filesystem::path parentPath = filePath.parent_path();
    if (!parentPath.empty() && !std::filesystem::exists(parentPath)) {
        return false;
    }
    std::ofstream outFile(path);
    if (!outFile.is_open()) {
        return false;
    }
    nlohmann::json j;
    j["controller"] = nlohmann::json::object();
    j["controller"]["xCenterData"] = xCenterData;
    j["controller"]["yCenterData"] = yCenterData;
    j["controller"]["xMaxData"] = xMaxData;
    j["controller"]["xMinData"] = xMinData;
    j["controller"]["yMaxData"] = yMaxData;
    j["controller"]["yMinData"] = yMinData;
    j["controller"]["deadZone"] = deadZone;
    j["controller"]["triggerMax"] = triggerMax;
    j["controller"]["triggerMin"] = triggerMin;
    j["controller"]["gripMax"] = gripMax;
    j["controller"]["gripMin"] = gripMin;
    j["controller"]["trackpadMax"] = trackpadMax;
    j["controller"]["trackpadMin"] = trackpadMin;
    j["handOffset"] = nlohmann::json::object();
    j["handOffset"]["thumbFinger"] = nlohmann::json::object();
    j["handOffset"]["thumbFinger"]["proximal"] = nlohmann::json::object();
    j["handOffset"]["thumbFinger"]["proximal"]["x"] = handOffset.thumbFinger.proximal.x;
    j["handOffset"]["thumbFinger"]["proximal"]["y"] = handOffset.thumbFinger.proximal.y;
    j["handOffset"]["thumbFinger"]["proximal"]["z"] = handOffset.thumbFinger.proximal.z;
    j["handOffset"]["thumbFinger"]["intermediate"] = nlohmann::json::object();
    j["handOffset"]["thumbFinger"]["intermediate"]["x"] = handOffset.thumbFinger.intermediate.x;
    j["handOffset"]["thumbFinger"]["intermediate"]["y"] = handOffset.thumbFinger.intermediate.y;
    j["handOffset"]["thumbFinger"]["intermediate"]["z"] = handOffset.thumbFinger.intermediate.z;
    j["handOffset"]["thumbFinger"]["distal"] = nlohmann::json::object();
    j["handOffset"]["thumbFinger"]["distal"]["x"] = handOffset.thumbFinger.distal.x;
    j["handOffset"]["thumbFinger"]["distal"]["y"] = handOffset.thumbFinger.distal.y;
    j["handOffset"]["thumbFinger"]["distal"]["z"] = handOffset.thumbFinger.distal.z;
    j["handOffset"]["indexFinger"] = nlohmann::json::object();
    j["handOffset"]["indexFinger"]["proximal"] = nlohmann::json::object();
    j["handOffset"]["indexFinger"]["proximal"]["x"] = handOffset.indexFinger.proximal.x;
    j["handOffset"]["indexFinger"]["proximal"]["y"] = handOffset.indexFinger.proximal.y;
    j["handOffset"]["indexFinger"]["proximal"]["z"] = handOffset.indexFinger.proximal.z;
    j["handOffset"]["indexFinger"]["intermediate"] = nlohmann::json::object();
    j["handOffset"]["indexFinger"]["intermediate"]["x"] = handOffset.indexFinger.intermediate.x;
    j["handOffset"]["indexFinger"]["intermediate"]["y"] = handOffset.indexFinger.intermediate.y;
    j["handOffset"]["indexFinger"]["intermediate"]["z"] = handOffset.indexFinger.intermediate.z;
    j["handOffset"]["indexFinger"]["distal"] = nlohmann::json::object();
    j["handOffset"]["indexFinger"]["distal"]["x"] = handOffset.indexFinger.distal.x;
    j["handOffset"]["indexFinger"]["distal"]["y"] = handOffset.indexFinger.distal.y;
    j["handOffset"]["indexFinger"]["distal"]["z"] = handOffset.indexFinger.distal.z;
    j["handOffset"]["middleFinger"] = nlohmann::json::object();
    j["handOffset"]["middleFinger"]["proximal"] = nlohmann::json::object();
    j["handOffset"]["middleFinger"]["proximal"]["x"] = handOffset.middleFinger.proximal.x;
    j["handOffset"]["middleFinger"]["proximal"]["y"] = handOffset.middleFinger.proximal.y;
    j["handOffset"]["middleFinger"]["proximal"]["z"] = handOffset.middleFinger.proximal.z;
    j["handOffset"]["middleFinger"]["intermediate"] = nlohmann::json::object();
    j["handOffset"]["middleFinger"]["intermediate"]["x"] = handOffset.middleFinger.intermediate.x;
    j["handOffset"]["middleFinger"]["intermediate"]["y"] = handOffset.middleFinger.intermediate.y;
    j["handOffset"]["middleFinger"]["intermediate"]["z"] = handOffset.middleFinger.intermediate.z;
    j["handOffset"]["middleFinger"]["distal"] = nlohmann::json::object();
    j["handOffset"]["middleFinger"]["distal"]["x"] = handOffset.middleFinger.distal.x;
    j["handOffset"]["middleFinger"]["distal"]["y"] = handOffset.middleFinger.distal.y;
    j["handOffset"]["middleFinger"]["distal"]["z"] = handOffset.middleFinger.distal.z;
    j["handOffset"]["ringFinger"] = nlohmann::json::object();
    j["handOffset"]["ringFinger"]["proximal"] = nlohmann::json::object();
    j["handOffset"]["ringFinger"]["proximal"]["x"] = handOffset.ringFinger.proximal.x;
    j["handOffset"]["ringFinger"]["proximal"]["y"] = handOffset.ringFinger.proximal.y;
    j["handOffset"]["ringFinger"]["proximal"]["z"] = handOffset.ringFinger.proximal.z;
    j["handOffset"]["ringFinger"]["intermediate"] = nlohmann::json::object();
    j["handOffset"]["ringFinger"]["intermediate"]["x"] = handOffset.ringFinger.intermediate.x;
    j["handOffset"]["ringFinger"]["intermediate"]["y"] = handOffset.ringFinger.intermediate.y;
    j["handOffset"]["ringFinger"]["intermediate"]["z"] = handOffset.ringFinger.intermediate.z;
    j["handOffset"]["ringFinger"]["distal"] = nlohmann::json::object();
    j["handOffset"]["ringFinger"]["distal"]["x"] = handOffset.ringFinger.distal.x;
    j["handOffset"]["ringFinger"]["distal"]["y"] = handOffset.ringFinger.distal.y;
    j["handOffset"]["ringFinger"]["distal"]["z"] = handOffset.ringFinger.distal.z;
    j["handOffset"]["littleFinger"] = nlohmann::json::object();
    j["handOffset"]["littleFinger"]["proximal"] = nlohmann::json::object();
    j["handOffset"]["littleFinger"]["proximal"]["x"] = handOffset.littleFinger.proximal.x;
    j["handOffset"]["littleFinger"]["proximal"]["y"] = handOffset.littleFinger.proximal.y;
    j["handOffset"]["littleFinger"]["proximal"]["z"] = handOffset.littleFinger.proximal.z;
    j["handOffset"]["littleFinger"]["intermediate"] = nlohmann::json::object();
    j["handOffset"]["littleFinger"]["intermediate"]["x"] = handOffset.littleFinger.intermediate.x;
    j["handOffset"]["littleFinger"]["intermediate"]["y"] = handOffset.littleFinger.intermediate.y;
    j["handOffset"]["littleFinger"]["intermediate"]["z"] = handOffset.littleFinger.intermediate.z;
    j["handOffset"]["littleFinger"]["distal"] = nlohmann::json::object();
    j["handOffset"]["littleFinger"]["distal"]["x"] = handOffset.littleFinger.distal.x;
    j["handOffset"]["littleFinger"]["distal"]["y"] = handOffset.littleFinger.distal.y;
    j["handOffset"]["littleFinger"]["distal"]["z"] = handOffset.littleFinger.distal.z;
    j["handResponse"] = nlohmann::json::array();
    for (const auto &c : handResponse)
        j["handResponse"].push_back({{"min", c.outputMin}, {"max", c.outputMax},
                                      {"balance", c.balance}, {"offset", c.offset}});
    outFile << j.dump(4); // Pretty print with 4 spaces
    outFile.close();
    return true;
}
