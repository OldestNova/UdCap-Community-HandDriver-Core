#include <SerialPortEnumeration.h>
#include <iostream>
#include <map>

struct FakePorts {
    using Interface = std::pair<uint8_t, bool>;
    std::map<Interface, std::string> names;
    std::vector<Interface> lookups;

    std::optional<std::string> operator()(uint8_t number, bool composite) {
        const Interface key{number, composite};
        lookups.push_back(key);
        const auto found = names.find(key);
        if (found == names.end()) return std::nullopt;
        return found->second;
    }
};

int main() {
    using serial_reborn::detail::enumerateSerialPortsForDevice;
    for (const bool firstOnly : {false, true}) {
        FakePorts available{{{{0, true}, "COM10"}, {{2, true}, "COM11"}, {{0, false}, "COM10"}}};
        const auto ports = enumerateSerialPortsForDevice(0x1A86, 0x7524, "SHARED", firstOnly, available);
        if (ports.size() != 2 || ports[0].portName != "COM10" || ports[1].portName != "COM11" ||
            ports[0].interfaceNumber != 0 || ports[1].interfaceNumber != 2 ||
            !ports[0].composite || !ports[1].composite || ports[0].isHid || ports[1].isHid ||
            ports[0].vid != 0x1A86 || ports[1].pid != 0x7524 ||
            ports[0].serialNumber != "SHARED" || ports[1].serialNumber != "SHARED" ||
            available.lookups != std::vector<FakePorts::Interface>{{0, true}, {2, true}}) {
            std::cerr << "Combined receiver lost a COM port or interface identity\n";
            return 1;
        }
    }

    FakePorts onlyMi02{{{{2, true}, "COM11"}}};
    const auto remaining = enumerateSerialPortsForDevice(0x1A86, 0x7524, "", true, onlyMi02);
    if (remaining.size() != 1 || remaining[0].interfaceNumber != 2) {
        std::cerr << "Missing MI00 prevented enumeration of MI02\n";
        return 1;
    }
    FakePorts absent;
    if (!enumerateSerialPortsForDevice(0x1A86, 0x7524, "", true, absent).empty()) {
        std::cerr << "Absent combined interfaces produced a port\n";
        return 1;
    }

    FakePorts legacy{{{{0, false}, "COM3"}}};
    const auto legacyPorts = enumerateSerialPortsForDevice(0x1A86, 0x7523, "LEGACY", true, legacy);
    if (legacyPorts.size() != 1 || legacyPorts[0].portName != "COM3" || legacyPorts[0].composite ||
        legacy.lookups != std::vector<FakePorts::Interface>{{0, false}}) {
        std::cerr << "Legacy receiver fast scan regressed\n";
        return 1;
    }
    FakePorts legacyFallback{{{{0, false}, "COM3"}}};
    const auto fallback = enumerateSerialPortsForDevice(0x1A86, 0x7523, "LEGACY", false, legacyFallback);
    if (fallback.size() != 1 || fallback[0].portName != "COM3" || fallback[0].composite) {
        std::cerr << "Non-composite serial port fallback regressed\n";
        return 1;
    }

    for (const bool firstOnly : {false, true}) {
        FakePorts generic{{{{0, true}, "COM20"}, {{2, true}, "COM21"}, {{255, true}, "COM22"}}};
        const auto ports = enumerateSerialPortsForDevice(0x1234, 0x7524, "OTHER", firstOnly, generic);
        if (ports.size() != (firstOnly ? 1 : 3) || ports[0].portName != "COM20" ||
            (!firstOnly && ports.back().interfaceNumber != 255)) {
            std::cerr << "Generic serial enumeration policy regressed\n";
            return 1;
        }
    }
    FakePorts aliases{{{{0, true}, "COM10"}, {{2, true}, "COM10"}}};
    if (enumerateSerialPortsForDevice(0x1A86, 0x7524, "SHARED", true, aliases).size() != 1) {
        std::cerr << "The same COM port was enumerated twice\n";
        return 1;
    }
    return 0;
}
