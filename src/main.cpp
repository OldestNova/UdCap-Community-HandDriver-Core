
#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <SDKDDKVer.h>
#endif

#include <iostream>
#include "UsbEnumerate.h"
#include "PortAccessor.h"
#include "UdCapProbe.h"
#include "UdCapV1Core.h"

int main() {
    UsbEnumerate usbEnum;
    try {
        usbEnum.refresh();
        std::vector<SerialDevice> devices = usbEnum.findPorts([](const SerialDevice & device) {
            return device.vid == 0x1A86 && device.pid == 0x7523;
        });
        for (const auto &device : devices) {
            std::cout << "Found device: " << device.portName << std::endl;
            std::shared_ptr<PortAccessor> portAccessor = std::make_shared<PortAccessor>(device);
            UdCapProbe prober(portAccessor);
            switch (prober.probe()) {
                case UDCAP_PROBE_FAILURE: {
                    std::cout << "  Probe failed." << std::endl;
                    break;
                }
                case UDCAP_PROBE_HAND_V1: {
                    std::cout << "  Probe successful. This is a UdCap Receiver with SN: " << prober.getUDCapSerial() << std::endl;
                    UdCapV1Core core(portAccessor, prober.getUDCapSerial());
                    auto unlisten = core.listen([](const UdCapV1MCUPacket &data) {
                        if (data.commandType == CMD_LINK_STATE) {
                            std::cout << "Link State: " << UdCapV1Core::fromLinkStateToString(data.linkState);
                        } else if (data.commandType == CMD_SERIAL) {
                            std::cout << "Serial Num: " << data.deviceSerialNum;
                        } else if (data.commandType == CMD_DATA) {
                            std::cout << "  Angle: ";
                            for (const auto &angle: data.angle) {
                                std::cout << std::dec << angle << " ";
                            }
                        }
                        std::cout << std::endl;
                    });
                    std::this_thread::sleep_for(std::chrono::seconds(90));
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