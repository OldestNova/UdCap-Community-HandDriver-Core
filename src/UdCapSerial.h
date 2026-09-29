#ifndef UDCAP_SERIAL_H
#define UDCAP_SERIAL_H

#include <algorithm>
#include <string_view>

inline bool isUdCapSerialText(std::string_view serial) {
    if (serial.size() < 6 || serial.size() > 32 ||
        (serial.back() != 'L' && serial.back() != 'R')) return false;
    return std::all_of(serial.begin(), serial.end(), [](char c) {
        return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
    });
}

inline bool isUdCapReceiverSerial(std::string_view serial) {
    return isUdCapSerialText(serial) && serial.substr(0, 3) == "UDX";
}

inline bool isUdCapGloveSerial(std::string_view serial) {
    return isUdCapSerialText(serial) &&
           (serial.substr(0, 2) == "AB" || serial.substr(0, 2) == "AC" ||
            serial.substr(0, 2) == "UD");
}

#endif // UDCAP_SERIAL_H
