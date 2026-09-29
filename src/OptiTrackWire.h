#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

// Versioned local IPC frame. The sender refreshes the complete pose on every
// skeleton packet, so late subscribers and temporary queue drops recover.
namespace udcap::optitrack {
inline constexpr char endpoint[] = "ipc:///tmp/udcap-community";
inline constexpr std::uint32_t magic = 0x4344554fu; // "OUDC" in little-endian
inline constexpr std::uint16_t version = 1;

#pragma pack(push, 1)
struct Quaternion {
    float x, y, z, w;
};
struct Frame {
    std::uint32_t frameMagic = magic;
    std::uint16_t frameVersion = version;
    std::uint8_t hand = 0; // 1 left, 2 right
    std::uint8_t batteryLevel = 0; // 0 unavailable, otherwise 1..5
    std::int16_t rssiDbm = INT16_MIN; // unavailable until a battery telemetry packet arrives
    std::uint16_t batteryRaw = 0;
    char serial[32]{};
    std::uint64_t sequence = 0;
    std::array<Quaternion, 15> joints{}; // thumb, index, middle, ring, little; MCP/PIP/DIP
};
#pragma pack(pop)
static_assert(std::is_trivially_copyable_v<Frame>);
static_assert(sizeof(Frame) == 292);
} // namespace udcap::optitrack
