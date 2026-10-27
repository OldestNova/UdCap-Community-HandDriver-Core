#ifndef SERIALREBORN_SERIALPORTENUMERATION_H
#define SERIALREBORN_SERIALPORTENUMERATION_H

#include "SerialCommon.h"
#include <algorithm>
#include <optional>
#include <utility>
#include <vector>

namespace serial_reborn::detail {

// ResolvePortName returns nullopt when an interface has no serial port.
template<typename ResolvePortName>
std::vector<SerialDevice> enumerateSerialPortsForDevice(
    uint16_t vendorId, uint16_t productId, const std::string &serialNumber,
    bool firstSerialPortOnly, ResolvePortName &&resolvePortName) {
    std::vector<SerialDevice> ports;
    auto appendPort = [&](uint8_t interfaceNumber, bool composite) {
        auto name = resolvePortName(interfaceNumber, composite);
        if (!name || name->empty()) return false;
        // A USB serial number belongs to the physical device, not each COM port.
        if (std::any_of(ports.begin(), ports.end(), [&](const SerialDevice &port) {
                return port.portName == *name;
            })) return true;
        SerialDevice port{};
        port.vid = vendorId;
        port.pid = productId;
        port.serialNumber = serialNumber;
        port.interfaceNumber = interfaceNumber;
        port.composite = composite;
        port.portName = std::move(*name);
        ports.push_back(std::move(port));
        return true;
    };

    if (vendorId == 0x1A86 && productId == 0x7524) {
        // Combined V1 has two independent ports, even in the fast scan mode.
        // Query the composite interfaces explicitly: a non-composite lookup
        // can silently fall back to MI00 in libusbp on Windows.
        appendPort(0x00, true);
        appendPort(0x02, true);
        return ports;
    }

    if (firstSerialPortOnly && appendPort(0, false)) return ports;
    for (unsigned int interfaceNumber = 0; interfaceNumber <= 0xFF; ++interfaceNumber) {
        if (appendPort(static_cast<uint8_t>(interfaceNumber), true) && firstSerialPortOnly) break;
    }
    if (ports.empty() && !firstSerialPortOnly) appendPort(0, false);
    return ports;
}

} // namespace serial_reborn::detail

#endif // SERIALREBORN_SERIALPORTENUMERATION_H
