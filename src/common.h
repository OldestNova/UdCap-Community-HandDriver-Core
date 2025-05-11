//
// Created by max_3 on 2025/5/8.
//

#ifndef SERIALREBORN_COMMON_H
#define SERIALREBORN_COMMON_H
#include <cstdint>
#include <string>
#include <memory>
#include "hidapi.h"


#ifdef _WIN32
#define __export __declspec(dllexport) extern "C"
#else
#define __export
#endif

struct SerialDevice {
    bool isHid;
    // Common
    uint16_t vid;
    uint16_t pid;
    uint8_t interfaceNumber;
    std::string serialNumber;
    // Serial
    std::string portName;
    bool composite;
    // HID
    hid_bus_type busType;
    std::string path;
    std::string manufacturerString;
    std::string productString;
    uint8_t releaseNumber;
    uint8_t usage;
    uint8_t usagePage;
};

#endif //SERIALREBORN_COMMON_H
