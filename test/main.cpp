#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <SDKDDKVer.h>
#endif

#include <iostream>
#include <UsbEnumerate.h>
#include <PortAccessor.h>
#include <UdCapProbe.h>
#include <UdCapV1Core.h>

int main() {
    UsbEnumerate usbEnum;
    try {
        usbEnum.refresh();
        std::vector<SerialDevice> devices = usbEnum.findPorts([](const SerialDevice &device) {
            return device.vid == 0x1A86 && device.pid == 0x7523;
        });
        for (const auto &device: devices) {
            std::cout << "Found device: " << device.portName << std::endl;
            std::shared_ptr<PortAccessor> portAccessor = std::make_shared<PortAccessor>(device);
            UdCapProbe prober(portAccessor);
            bool ready = false;
            switch (prober.probe()) {
                case UDCAP_PROBE_FAILURE: {
                    std::cout << "  Probe failed." << std::endl;
                    break;
                }
                case UDCAP_PROBE_HAND_V1: {
                    std::cout << "  Probe successful. This is a UdCap Receiver with SN: " << prober.getUDCapSerial() <<
                            std::endl;
                    UdCapV1Core core(portAccessor);
                    auto unlisten = core.listen([&core, &ready](const UdCapV1MCUPacket &data) {
                        if (data.commandType == CMD_LINK_STATE) {
                            std::cout << "  Link State: " << UdCapV1Core::fromUdStateToString(data.udState) <<
                                    std::endl;
                            if (data.udState == UdState::UD_INIT_STATE_LINKED) {
                                std::thread t([&core]() {
                                    try {
                                        std::cout << "Start calibration. " << std::endl;
                                        std::this_thread::sleep_for(std::chrono::seconds(1));
                                        core.runCalibration(UdCapV1DeviceCaliType::UDCAP_V1_DEVICE_CALI_TYPE_HAND);
                                        std::cout << "Start calibration. Fist!" << std::endl;
                                        std::this_thread::sleep_for(std::chrono::seconds(5));
                                        std::cout << "Getting Value Fist!" << std::endl;
                                        core.captureCalibrationData(UdCapV1HandCaliType::UDCAP_V1_HAND_CALI_TYPE_FIST);
                                        std::cout << "Start calibration. Adduction!" << std::endl;
                                        std::this_thread::sleep_for(std::chrono::seconds(5));
                                        std::cout << "Getting Value Adduction!" << std::endl;
                                        core.captureCalibrationData(
                                            UdCapV1HandCaliType::UDCAP_V1_HAND_CALI_TYPE_ADDUCTION);
                                        std::cout << "Start calibration. Protract!" << std::endl;
                                        std::this_thread::sleep_for(std::chrono::seconds(5));
                                        std::cout << "Getting Value Protract!" << std::endl;
                                        core.captureCalibrationData(
                                            UdCapV1HandCaliType::UDCAP_V1_HAND_CALI_TYPE_PROTRACT);
                                        std::cout << "Done!" << std::endl;
                                        core.completeCalibration(UdCapV1DeviceCaliType::UDCAP_V1_DEVICE_CALI_TYPE_HAND);
                                    } catch (std::exception &e) {
                                        std::cerr << e.what() << std::endl;
                                    }
                                });
                                t.detach();
                            }
                        } else if (data.commandType == CMD_SERIAL) {
                            std::cout << "  Serial Num: " << data.deviceSerialNum << std::endl;
                            std::cout << "    Hand: " << (core.getTarget() == UD_TARGET_LEFT_HAND
                                                              ? "Left"
                                                              : (core.getTarget() == UD_TARGET_RIGHT_HAND
                                                                     ? "Right"
                                                                     : "Unknown")) << std::endl;
                            std::cout << "    Type: " << (data.isEnterprise ? "Enterprise" : "Client") << std::endl;
                        } else if (data.commandType == CMD_DATA) {
                            // std::cout << "  Angle: ";
                            // for (const auto &angle: data.angle) {
                            //     std::cout << std::dec << angle << " ";
                            // }
                            // std::cout << std::endl;
                        } else if (data.commandType == CMD_GET_CHANNEL) {
                            std::cout << "    Channel: " << static_cast<unsigned>(data.channel) << std::endl;
                        } else if (data.commandType == CMD_READY) {
                            if (data.isReady) {
                                std::cout << "    Ready." << std::endl;
                                ready = true;
                                // core.mcuSendVibration(1, 2, 10);
                            }
                        } else if (data.commandType == CMD_ANGLE) {
                            std::vector<double> angles = data.result;
                            std::cout << "Angles: ";
                            for (int i = 0; i < angles.size(); i++) {
                                std::cout << angles[i] << ", ";
                            }
                            std::cout << std::endl;
                        } else if (data.commandType == CMD_INPUT_JOYSTICK) {
                            if (ready)
                                std::cout << "Input joystick. X: " << data.joystickData.joyX << " Y: " << data.
                                        joystickData.joyY << std::endl;
                        } else if (data.commandType == CMD_INPUT_BUTTON) {
                            if (ready)
                                std::cout << "Input Button. A: " << data.button.btnA << " B: " << data.button.btnB <<
                                        " Menu: " << data.button.btnMenu << " JoyStick: " << data.button.btnJoyStick <<
                                        " PWR: " << data.button.btnPower << std::endl;
                        }
                    });
                    std::this_thread::sleep_for(std::chrono::seconds(60));
                    unlisten();
                    break;
                }
            }
        }
    } catch (const std::runtime_error &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}
