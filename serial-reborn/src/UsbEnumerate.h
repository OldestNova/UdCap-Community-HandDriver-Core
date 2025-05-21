//
// Created by max_3 on 2025/5/8.
//

#ifndef SERIALREBORN_USBENUMERATE_H
#define SERIALREBORN_USBENUMERATE_H

#include "SerialCommon.h"
#include <vector>
#include <functional>

enum UsbEnumerateRefreshType {
    USB_ENUMERATE_REFRESH_ALL = 0,
    USB_ENUMERATE_REFRESH_SERIAL = 1,
    USB_ENUMERATE_REFRESH_HID = 2
};

class UsbEnumerate {
public:
    UsbEnumerate();
    ~UsbEnumerate();
    void refresh(UsbEnumerateRefreshType type);
    void printDevices();
    std::vector<SerialDevice> findPorts(std::function<bool(const SerialDevice&)>);
private:
    std::vector<SerialDevice> devices;
};


#endif //SERIALREBORN_USBENUMERATE_H
