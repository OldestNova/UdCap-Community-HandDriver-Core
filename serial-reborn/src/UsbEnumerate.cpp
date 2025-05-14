//
// Created by max_3 on 2025/5/8.
//

#include <iostream>
#include <iomanip>
#include <hidapi.h>
#include <libusbp.hpp>
#include "UsbEnumerate.h"



UsbEnumerate::UsbEnumerate() {

}

UsbEnumerate::~UsbEnumerate() {

}

void UsbEnumerate::refresh() {
    this->devices.clear();
    try {
        auto devices = libusbp::list_connected_devices();
        for (const libusbp::device & device : devices)
        {
            // Print the USB device info.
            uint16_t vendorId = device.get_vendor_id();
            uint16_t productId = device.get_product_id();
            try {
                SerialDevice serialDevice {};
                serialDevice.isHid = false;
                serialDevice.serialNumber = device.get_serial_number();
                serialDevice.vid = vendorId;
                serialDevice.pid = productId;
                std::string port_name;
                bool success = false;
                for (uint32_t i = 0; i < 255; i++) {
                    try {
                        libusbp::serial_port port(device, i, true);
                        port_name = port.get_name();
                        serialDevice.interfaceNumber = i;
                        serialDevice.composite = true;
                        success = true;
                    } catch (const libusbp::error & error) {

                    }
                }
                if (!success)
                {
                    try {
                        libusbp::serial_port port(device, 0, false);
                        serialDevice.interfaceNumber = 0;
                        serialDevice.composite = false;
                        port_name = port.get_name();
                    } catch (const libusbp::error & error) {
                        continue;
                    }
                }
                serialDevice.portName = port_name;
                this->devices.push_back(std::move(serialDevice));
            } catch (const libusbp::error & error) {

            }
        }
    } catch (const libusbp::error & error) {
        std::cerr << "Error listing serial devices: " << error.what() << std::endl;
    }
    auto err = hid_init();
    if (err > 0) {
        std::cerr << "Error listing HID devices" << std::endl;
        return;
    }
    auto hids = hid_enumerate(0, 0);
    struct hid_device_info *curDev;
    curDev = hids;
    while (curDev) {
        SerialDevice serialDevice {};
        serialDevice.isHid = true;
        serialDevice.vid = curDev->vendor_id;
        serialDevice.pid = curDev->product_id;
        std::wstring wsSerialNumber(curDev->serial_number);
        serialDevice.serialNumber = std::string(wsSerialNumber.begin(), wsSerialNumber.end());
        serialDevice.path = curDev->path;
        serialDevice.interfaceNumber = curDev->interface_number;
        serialDevice.busType = static_cast<HidBusType>(curDev->bus_type);
        std::wstring wsManufacturerString(curDev->manufacturer_string);
        serialDevice.manufacturerString = std::string(wsManufacturerString.begin(), wsManufacturerString.end());
        std::wstring wsProductString(curDev->product_string);
        serialDevice.productString = std::string(wsProductString.begin(), wsProductString.end());
        serialDevice.releaseNumber = curDev->release_number;
        serialDevice.usage = curDev->usage;
        serialDevice.usagePage = curDev->usage_page;
        this->devices.push_back(std::move(serialDevice));
        curDev = curDev->next;
    }
}

void UsbEnumerate::printDevices() {
    std::cout << "IsHID\tVID\t\tPID\t\t\tComposite\tSerial Number\tPort Name" << std::endl;
    for (const auto & device : devices) {
        std::cout << device.isHid << "\t\t"
                  << std::hex << std::setw(4) << std::setfill('0') << device.vid << "\t"
                  << std::hex << std::setw(4) << std::setfill('0') << device.pid << "\t\t"
                  << device.composite << "\t\t\t"
                  << device.serialNumber << "\t\t"
                  << device.portName << std::endl;
    }
}

std::vector<SerialDevice> UsbEnumerate::findPorts(std::function<bool(const SerialDevice&)> filter) {
    std::vector<SerialDevice> filteredDevices;
    for (const auto & device : devices) {
        if (filter(device)) {
            filteredDevices.push_back(device);
        }
    }
    return filteredDevices;
}
