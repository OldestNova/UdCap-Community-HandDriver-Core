//
// Created by max_3 on 2025/5/11.
//

#include "UdCapV1Core.h"
#include <AR1Linear.hpp>
#include <filesystem>
#include <iostream>

ImportAR1Linear(AR1LinearAA)
ImportAR1Linear(AR1LinearAD)
ImportAR1Linear(AR1LinearAE)
ImportAR1Linear(AR1LinearAG)

uint8_t calculateCRC(const std::vector<uint8_t> &data, bool ignoreLast) {
    uint8_t crc = 0;
    size_t lastIndex = ignoreLast ? data.size() - 1 : data.size();
    for (size_t i = 2; i < data.size() - 1; i++) {
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

std::vector<double> udCapV1HandAutoCali(const double f[], const double n[], const double h[], float nn) {
    std::vector<double> array(12);
    for (int i = 0; i < 12; i++) {
        if (h[i] <= n[i]) {
            array[i] = 0.0;
        } else {
            array[i] = (f[i] - n[i]) / (h[i] - n[i]) * (double) nn;
        }
    }
    return array;
}

UdCapV1Core::UdCapV1Core(std::shared_ptr<PortAccessor> portAccessor): eventLoopRunning(true), portAccessor(std::move(portAccessor)) {
    std::thread t([this]() {
        while (eventLoopRunning.load()) {
            if (packetQueue.empty()) {
                std::unique_lock lk(eventLoopMutex);
                eventCondition.wait_for(lk, std::chrono::milliseconds(1000));
                continue;
            }
            UdCapV1MCUPacket packet = packetQueue.front();
            packetQueue.pop();
            for (const auto &callback: listenCallbacks) {
                callback(packet);
            }
        }
    });
    eventLoop = std::move(t);
    this->portAccessor->openPort();
    this->portAccessor->setBaudRate(115200);
    if (!this->portAccessor->hasPacketRealignmentHelper()) {
        std::unique_ptr<UdCapHandV1PacketRealignmentHelper> packetRealignmentHelper = std::make_unique<
            UdCapHandV1PacketRealignmentHelper>();
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
    this->eventLoopRunning.store(false);
    this->eventLoop.join();
}

std::function<void()> UdCapV1Core::listen(const std::function<void(const UdCapV1MCUPacket &)> &callback) {
    std::lock_guard guard(callbackMutex);
    listenCallbacks.push_back(callback);
    return [this, callback]() {
        // Remove the callback from the list
        listenCallbacks.erase(std::remove_if(listenCallbacks.begin(), listenCallbacks.end(),
                                             [&callback](const std::function<void(const UdCapV1MCUPacket &)> &item) {
                                                 return item.target_type() == callback.target_type();
                                             }), listenCallbacks.end());
    };
}

void UdCapV1Core::callListenCallback(const UdCapV1MCUPacket &packet) {
    std::lock_guard guard(callbackMutex);
    packetQueue.push(packet);
    eventCondition.notify_all();
}

void UdCapV1Core::sendCommand(uint8_t humanAddress, CommandType commandType, const std::vector<uint8_t> &data) {
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
    return udCapSerial;
}

void UdCapV1Core::parsePacket(const std::vector<uint8_t> &packetBuffer) {
    uint8_t dataLength = packetBuffer[5];
    std::vector<uint8_t> data;
    for (size_t i = 6; i < 6 + dataLength; i++) {
        data.push_back(packetBuffer[i]);
    }
    if (packetBuffer[3] == (uint8_t) CommandType::CMD_LINK_STATE) {
        auto deData = decodeXOR(data);
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
                callListenCallback(packetSerial);
            } else if (packetSerial.deviceSerialNum.find("AC") != std::string::npos) {
                packetSerial.isEnterprise = false;
                isEnterprise = false;
                callListenCallback(packetSerial);
            }
        }
        short linkState = deData[0];
        UdCapV1MCUPacket packet{};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_LINK_STATE;
        if (linkState == 0) {
            packet.udState = UdState::UD_INIT_STATE_NOT_CONNECT;

            if (packet.udState != udState) {
                callListenCallback(packet);
            }
            if (udState != UdState::UD_INIT_STATE_NOT_CONNECT && udState != UdState::UD_INIT_STATE_INIT) {
                udState = packet.udState;
                mcuStopData();
            }
            udState = packet.udState;
        } else if (linkState == 1) {
            packet.udState = UdState::UD_INIT_STATE_CONNECTED;

            if (packet.udState != udState) {
                callListenCallback(packet);
            }
            if (udState == UdState::UD_INIT_STATE_NOT_CONNECT || udState == UdState::UD_INIT_STATE_INIT) {
                udState = packet.udState;
                mcuStartData();
            }
            udState = packet.udState;
        }
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_SET_CHANNEL_DONE) {
        UdCapV1MCUPacket packet{};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_SET_CHANNEL_DONE;
        callListenCallback(packet);
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_GET_CHANNEL) {
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
        UdCapV1MCUPacket packet{};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_BATTERY;
        auto deData = decodeXOR(data);
        auto udata = (int16_t) ((deData[0] << 8) | deData[1]);
        int batt = ((udata <= 1980) ? 1 : ((udata <= 2090) ? 2 : ((udata <= 2150) ? 3 : ((udata > 2300) ? 5 : 4))));
        if (batt > lastBattery) {
            batt = ((udata <= 2010) ? 1 : ((udata <= 2120) ? 2 : ((udata <= 2180) ? 3 : ((udata > 2320) ? 5 : 4))));
        }
        lastBattery = batt;
        packet.battery = batt;
        callListenCallback(packet);
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_SET_CHANNEL) {
        UdCapV1MCUPacket packet{};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_SET_CHANNEL;
        auto deData = decodeXOR(data);
        uint16_t result = deData[1];
        packet.channel = result;
        callListenCallback(packet);
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_SERIAL) {
        {
            UdCapV1MCUPacket packet{};
            packet.address = packetBuffer[2];
            packet.commandType = CommandType::CMD_SERIAL;
            auto deData = decodeXOR(data);
            packet.deviceSerialNum = std::string(deData.begin(), deData.end());
            if (packet.deviceSerialNum.find("AB") != std::string::npos) {
                packet.isEnterprise = true;
                callListenCallback(packet);
            } else if (packet.deviceSerialNum.find("AC") != std::string::npos) {
                packet.isEnterprise = false;
                callListenCallback(packet);
            }
        }
        mcuStartData();
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_SET_CHANNEL) {
        UdCapV1MCUPacket packet{};
        packet.address = packetBuffer[2];
        packet.commandType = CommandType::CMD_SET_CHANNEL;
        auto deData = decodeXOR(data);
        packet.channel = deData[0];
        packet.channelResult = deData[1];
        callListenCallback(packet);
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_GET_CHANNEL) {
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
    } else if (packetBuffer[3] == (uint8_t) CommandType::CMD_DATA) {
        UdCapV1MCUPacket packet{};
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
        // Start Calc
        memset(&lastAngle, 0, sizeof(lastAngle));
        bool hasController = false;
        if (iData.size() > 12) {
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
        if (iData.size() > 16) {
            lastAngle.f16 = (float) (iData[16]);
            lastAngle.f17 = (float) (iData[17]);
            lastAngle.f18 = (float) (iData[18]);
            hasController = true;
        }

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
            std::vector<double> array4 = udCapV1HandAutoCali(f, array2, array3, nn);
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
            // double array5[12][1];
            // array5[0][0]  = a4;
            // array5[1][0]  = a5;
            // array5[2][0]  = a6;
            // array5[3][0]  = a7;
            // array5[4][0]  = a8;
            // array5[5][0]  = a9;
            // array5[6][0]  = a10;
            // array5[7][0]  = a11;
            // array5[8][0]  = a12;
            // array5[9][0]  = a13;
            // array5[10][0] = a14;
            // array5[11][0] = a15;
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

            //            if (pre_sensor_data == null)
            //            {
            //                pre_sensor_data = new double[12];
            //                pre_sensor_data[0] = Convert.ToDouble(a4);
            //                pre_sensor_data[1] = Convert.ToDouble(a5);
            //                pre_sensor_data[2] = Convert.ToDouble(a6);
            //                pre_sensor_data[3] = Convert.ToDouble(a7);
            //                pre_sensor_data[4] = Convert.ToDouble(a8);
            //                pre_sensor_data[5] = Convert.ToDouble(a9);
            //                pre_sensor_data[6] = Convert.ToDouble(a10);
            //                pre_sensor_data[7] = Convert.ToDouble(a11);
            //                pre_sensor_data[8] = Convert.ToDouble(a12);
            //                pre_sensor_data[9] = Convert.ToDouble(a13);
            //                pre_sensor_data[10] = Convert.ToDouble(a14);
            //                pre_sensor_data[11] = Convert.ToDouble(a15);
            //            }

            std::string serialNum = getUDCapSerial();
            std::vector<double> list;
            if (serialNum.empty()) {
                list = AR1LinearAE()->sensor2Angle(array6, {});
            } else if (serialNum[5] == 'A') {
                list = AR1LinearAA()->sensor2Angle(array6, {});
            } else if (serialNum[5] == 'B') {
                list = AR1LinearAD()->sensor2Angle(array6, {});
            } else if (serialNum[5] != 'C' && serialNum[5] != 'D') {
                list = AR1LinearAG()->sensor2Angle(array6, {});
            } else {
                list = AR1LinearAE()->sensor2Angle(array6, {});
            }
            //            Array.Copy(array6, pre_sensor_data, pre_sensor_data.Length);

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

            double _calibrationDataC[28]{};
            for (int n = 0; n < list.size(); n++) {
                _calibrationDataC[n] = list[n];
            }
            _calibrationDataC[23] = static_cast<float>(list[23]);
            _calibrationDataC[24] = lastAngle.f0;
            _calibrationDataC[25] = lastAngle.f1;
            _calibrationDataC[26] = lastAngle.f2;
            _calibrationDataC[27] = lastAngle.f3;
            if (udState == UdState::UD_INIT_STATE_LINKED) {
                UdCapV1MCUPacket dataPacket{};
                dataPacket.address = packetBuffer[2];
                dataPacket.commandType = CommandType::CMD_ANGLE;
                dataPacket.result = std::vector<
                    double>(_calibrationDataC, _calibrationDataC + std::size(_calibrationDataC));
                callListenCallback(dataPacket);
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
        if (udState == UdState::UD_INIT_STATE_LINKED) {
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
                powerButtonTimeout = std::chrono::system_clock::now();
                powerBtnPressed = true;
            }
            callListenCallback(packetInputBtn);
        }
    }
}

UdTarget UdCapV1Core::getTarget() const {
    return target;
}

void UdCapV1Core::mcuStopData() {
    if (udState != UD_INIT_STATE_INIT) {
        sendCommand(1, CommandType::CMD_STOP_DATA, {1});
    }
}

void UdCapV1Core::mcuStartData() {
    if (udState == UD_INIT_STATE_CONNECTED) {
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

void UdCapV1Core::mcuSendVibration(int index, float second, int strength) {
    if (index != 1 && index != 2) return;
    float rSecond = ((second <= 0.04) ? 0.04 : ((second > 2.5) ? 2.5 : second));
    int rStrength = ((strength < 4) ? 4 : ((strength > 10) ? 10 : strength));
    sendCommand(1, CommandType::CMD_VIBRATION, { (uint8_t)index, 0, (uint8_t)(rSecond * 100.0), (uint8_t)rStrength });
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
    if (udState != UD_INIT_STATE_LINKED) {
        throw std::runtime_error("Core not initialized");
    }
    if (type == UdCapV1DeviceCaliType::UDCAP_V1_DEVICE_CALI_TYPE_HAND) {
        if (caliStat != UDCAP_V1_HAND_CALI_STAT_NONE) {
            throw std::runtime_error("Calibration already called");
        }
        caliStat = UDCAP_V1_HAND_CALI_STAT_AUTO;
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
    if (udState != UD_INIT_STATE_LINKED) {
        throw std::runtime_error("Core not initialized");
    }
    if (caliStat == UDCAP_V1_HAND_CALI_STAT_COMPLETED) {
        if (type != UDCAP_V1_HAND_CALI_TYPE_ALL) {
            throw std::runtime_error("Can not clear single calibration data after completed");
        } else {
            caliStat = UDCAP_V1_HAND_CALI_STAT_NONE;
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
        bool calibrationSuccess = false;
        for (int i = 0; i < 12; i++) {
            if (abs(array3[i] - array2[i]) > 25.0) {
                calibrationSuccess = true;
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

void UdCapV1Core::captureJoystickData(UdCapV1JoystickCaliType type) {
    if (udState != UD_INIT_STATE_LINKED) {
        throw std::runtime_error("Core not initialized");
    }
    if (joystickCaliStat == UdCapV1JoystickCaliStat::UDCAP_V1_JOYSTICK_CALI_STAT_OK) {
        throw std::runtime_error("Not in calibration mode");
    }
    if (joystickCaliStat == UdCapV1JoystickCaliStat::UDCAP_V1_JOYSTICK_CALI_STAT_CAPTURE_CENTER) {
        if (type == UdCapV1JoystickCaliStat::UDCAP_V1_JOYSTICK_CALI_STAT_CAPTURE_CENTER) {
            xCenterData = lastAngle.f16;
            yCenterData = lastAngle.f17;
            joystickCaliStat = UdCapV1JoystickCaliStat::UDCAP_V1_JOYSTICK_CALI_STAT_CAPTURE_ZONE;
        } else {
            throw std::runtime_error("Error on calibrate type with ZONE, now calibrating CENTER");
        }
    } else if (joystickCaliStat == UdCapV1JoystickCaliStat::UDCAP_V1_JOYSTICK_CALI_STAT_CAPTURE_ZONE) {
        if (type == UdCapV1JoystickCaliStat::UDCAP_V1_JOYSTICK_CALI_STAT_CAPTURE_CENTER) {
            std::cout << "Joystick zone calibration is running on data callback automatically." << std::endl;
        } else {
            throw std::runtime_error("Error on calibrate type with CENTER, now calibrating ZONE");
        }
    }
}

void UdCapV1Core::clearJoystickData(UdCapV1JoystickCaliType type) {
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
    return caliStat;
}

UdCapV1JoystickCaliStat UdCapV1Core::getJoystickCalibrationStatus() const {
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

bool UdCapV1Core::loadPref() {
    std::filesystem::path prefDir("prefs");
    std::filesystem::path dirPath = std::filesystem::relative(std::filesystem::current_path(), prefDir);
    if (!std::filesystem::exists(dirPath)) {
        if (!std::filesystem::create_directory(dirPath)) {
            return false;
        }
    }
    std::filesystem::path prefFile(getUDCapSerial() + ".json");
    std::filesystem::path file = std::filesystem::relative(dirPath, prefFile);
    if (!std::filesystem::exists(file)) {
        return false;
    }
    // TODO
    return false;
}

bool UdCapV1Core::loadPref(std::string path) {
    std::filesystem::path filePath(path);
    std::filesystem::path parentPath = filePath.parent_path();
    if (!std::filesystem::exists(parentPath)) {
        return false;
    }
    if (!std::filesystem::exists(filePath)) {
        return false;
    }
    // TODO
    return false;
}

bool UdCapV1Core::savePref() {
    std::filesystem::path prefDir("prefs");
    std::filesystem::path dirPath = std::filesystem::relative(std::filesystem::current_path(), prefDir);
    if (!std::filesystem::exists(dirPath)) {
        if (!std::filesystem::create_directory(dirPath)) {
            return false;
        }
    }
    std::filesystem::path prefFile(getUDCapSerial() + ".json");
    std::filesystem::path file = std::filesystem::relative(dirPath, prefFile);
    if (!std::filesystem::exists(file)) {
        return false;
    }
    // TODO
    return false;
}

bool UdCapV1Core::savePref(std::string path) {
    std::filesystem::path filePath(path);
    std::filesystem::path parentPath = filePath.parent_path();
    if (!std::filesystem::exists(parentPath)) {
        return false;
    }
    // TODO
    return false;
}
