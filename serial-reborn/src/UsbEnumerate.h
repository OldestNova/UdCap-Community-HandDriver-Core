//
// Created by max_3 on 2025/5/8.
//

#ifndef SERIALREBORN_USBENUMERATE_H
#define SERIALREBORN_USBENUMERATE_H

#include "SerialCommon.h"
#include <vector>
#include <string>
#include <libusbp.hpp>
#include <functional>

class UsbEnumerate {
public:
    UsbEnumerate();
    ~UsbEnumerate();
    void refresh();
    void printDevices();
    std::vector<SerialDevice> findPorts(std::function<bool(const SerialDevice&)>);
private:
    std::vector<SerialDevice> devices;
};


#endif //SERIALREBORN_USBENUMERATE_H
