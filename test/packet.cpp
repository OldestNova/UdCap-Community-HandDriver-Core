#include <UdCapV1Core.h>
#include <UdCapProbe.h>
#include <UdCapSerial.h>
#include <CorePref.h>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

static std::vector<uint8_t> makePacket(const std::vector<uint8_t> &payload) {
    std::vector<uint8_t> packet{170, 85, 1, 7, 0, static_cast<uint8_t>(payload.size())};
    packet.insert(packet.end(), payload.begin(), payload.end());
    uint8_t crc = 0;
    for (size_t i = 2; i < packet.size(); ++i) crc += packet[i];
    packet.push_back(crc);
    return packet;
}

int main() {
    UdCapHandV1PacketRealignmentHelper helper;
    const auto empty = makePacket({});
    const auto normal = makePacket({1, 2, 3});
    std::vector<uint8_t> input = empty;
    input.insert(input.end(), normal.begin(), normal.end());
    const auto packets = helper.processPacket(input);
    if (packets.size() != 2 || packets[0] != empty || packets[1] != normal) {
        std::cerr << "Zero-length packet disrupted framing\n";
        return 1;
    }

    UdCapHandV1PacketRealignmentHelper fragmented;
    auto first = fragmented.processPacket({170});
    std::vector<uint8_t> remainder{170};
    remainder.insert(remainder.end(), normal.begin() + 1, normal.end());
    auto second = fragmented.processPacket(remainder);
    if (!first.empty() || second.size() != 1 || second[0] != normal) {
        std::cerr << "Fragmented packet or repeated header was not recovered\n";
        return 1;
    }

    auto corrupt = normal;
    corrupt.back() ^= 1;
    if (!fragmented.processPacket(corrupt).empty()) {
        std::cerr << "Invalid CRC was accepted\n";
        return 1;
    }

    if (CorePref::getInstance().getPrefPath() != std::filesystem::current_path() / "prefs") {
        std::cerr << "Default preference directory is incorrect\n";
        return 1;
    }

    UdCapProbePacketRealignmentHelper nameParser;
    if (!nameParser.processPacket({'n', 'o', 'i', 's', 'e', 'U', 'D', 'X', 'S'}).empty()) {
        std::cerr << "Incomplete receiver name was accepted\n";
        return 1;
    }
    const auto namePackets = nameParser.processPacket({'T', '4', '0', '0', '0', '4', '9', 'L', '\r', '\n'});
    const std::vector<uint8_t> expectedName{'U', 'D', 'X', 'S', 'T', '4', '0', '0', '0', '4', '9', 'L'};
    if (namePackets.size() != 1 || namePackets.front() != expectedName) {
        std::cerr << "Fragmented receiver name was not recovered\n";
        return 1;
    }
    if (!nameParser.processPacket({'U', 'D', 'X', 'S', 'T', '4', '0', '0', '0', '4', '9', 'X', '\r'}).empty() ||
        !nameParser.processPacket({'U', 'D', 'X', 'S', 'T', '4', 0, '0', '0', '4', '9', 'L', '\r'}).empty() ||
        !nameParser.processPacket({'U', 'D', 'X', 'S', 'T', '4', '0', '0', '0', '4', '9', 'L', 'X', '\r'}).empty() ||
        !isUdCapGloveSerial("ACSTDE4FG540004L") ||
        !isUdCapGloveSerial("UDXST400049L") ||
        isUdCapGloveSerial("ACSTDE4FG540004X")) {
        std::cerr << "Malformed device name was accepted\n";
        return 1;
    }
    const auto recoveredName = nameParser.processPacket({'U', 'U', 'D', 'X', 'S', 'T', '4', '0', '0', '0', '4', '9', 'R', '\r'});
    if (recoveredName.size() != 1 || recoveredName.front().back() != 'R') {
        std::cerr << "Receiver name parser did not resynchronize\n";
        return 1;
    }

    const std::vector<std::string> combinedNames{
        "UDXST1L,AHST000019D",
        "UDXST1R,AHST000019D",
        "UDXST1L,AH" + std::string(30, '9')
    };
    for (const auto &name : combinedNames) {
        if (!isUdCapCombinedV1ReceiverSerial(name) || isUdCapReceiverSerial(name)) {
            std::cerr << "Combined receiver name was not distinguished from a legacy name\n";
            return 1;
        }
        const std::vector<uint8_t> expected(name.begin(), name.end());
        const std::string response = "AT+NAME?\r\n+NAME:" + name + "\r\nOK\r\n";
        const std::vector<uint8_t> bytes(response.begin(), response.end());
        // Include splits before/after the comma and inside the device serial.
        for (std::size_t split = 0; split <= bytes.size(); ++split) {
            UdCapProbePacketRealignmentHelper parser;
            auto actual = parser.processPacket(std::vector<uint8_t>(bytes.begin(), bytes.begin() + split));
            const auto tail = parser.processPacket(std::vector<uint8_t>(bytes.begin() + split, bytes.end()));
            actual.insert(actual.end(), tail.begin(), tail.end());
            if (actual.size() != 1 || actual.front() != expected) {
                std::cerr << "Fragmented combined receiver name was not preserved\n";
                return 1;
            }
        }
        UdCapProbePacketRealignmentHelper byteParser;
        std::vector<std::vector<uint8_t>> actual;
        for (const auto byte : bytes) {
            const auto packets = byteParser.processPacket({byte});
            actual.insert(actual.end(), packets.begin(), packets.end());
        }
        if (actual.size() != 1 || actual.front() != expected) {
            std::cerr << "Byte-by-byte combined receiver name was not recovered\n";
            return 1;
        }
    }
    const std::vector<std::string> invalidCombinedNames{
        "UDXST1X,AHST000019D", "UDXST2L,AHST000019D", "UDXST1L,",
        "UDXST1L,AHST", "UDXST1L,ACST000019D", "UDXST1L,AHST000019d",
        "UDXST1L,AHST000019D,EXTRA", "UDXST1L,AHST 000019D",
        "UDXST1R,AH" + std::string(31, '9'), "UDX" + std::string(29, '0') + "L"
    };
    for (const auto &name : invalidCombinedNames) {
        UdCapProbePacketRealignmentHelper parser;
        const std::string response = name + "\r\n";
        if (isUdCapCombinedV1ReceiverSerial(name) ||
            !parser.processPacket(std::vector<uint8_t>(response.begin(), response.end())).empty()) {
            std::cerr << "Malformed combined or oversized legacy receiver name was accepted\n";
            return 1;
        }
        const std::string legacy = "UDXST400049L\r\n";
        const auto recovered = parser.processPacket(std::vector<uint8_t>(legacy.begin(), legacy.end()));
        if (recovered.size() != 1 || recovered.front() != expectedName) {
            std::cerr << "Receiver name parser did not recover after a malformed combined name\n";
            return 1;
        }
    }
    if (isUdCapCombinedV1ReceiverSerial("UDXST400049L")) {
        std::cerr << "Legacy receiver name was classified as combined\n";
        return 1;
    }
    return 0;
}
