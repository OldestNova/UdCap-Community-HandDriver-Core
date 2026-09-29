#include <PortAccessor.h>
#include <UdCapV1Core.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

static std::vector<uint8_t> makePacket(uint8_t command, const std::string &payload) {
    std::vector<uint8_t> packet{170, 85, 1, command, 0, static_cast<uint8_t>(payload.size())};
    for (unsigned char byte : payload) packet.push_back(byte ^ 129);
    uint8_t crc = 0;
    for (std::size_t i = 2; i < packet.size(); ++i) crc += packet[i];
    packet.push_back(crc);
    return packet;
}

int main() {
    auto port = std::make_shared<PortAccessor>();
    int opens = 0;
    int baudRate = 0;
    std::vector<std::vector<uint8_t>> writes;
    port->bindSerialCallbacks({
        [&] { ++opens; return true; },
        [] {},
        [&](int baud) { baudRate = baud; },
        [&](const uint8_t *data, std::size_t size) {
            writes.emplace_back(data, data + size);
            return size;
        }
    });

    std::mutex mutex;
    std::condition_variable condition;
    bool gotVersion = false;
    {
        UdCapV1Core core(port);
        if (opens != 1 || baudRate != 115200) {
            std::cerr << "Manual serial initialization failed\n";
            return 1;
        }
        auto unlisten = core.listen([&](const std::shared_ptr<UdCapV1MCUPacket> &packet) {
            if (packet->commandType == CMD_FW_VERSION && packet->fwVersion == "1.2.3") {
                std::lock_guard lock(mutex);
                gotVersion = true;
                condition.notify_all();
            }
        });
        port->notifyConnectionState(PORT_ACCESSOR_CONNECTION_STATE_CONNECTED);
        if (writes.empty() || writes.back().size() < 4 || writes.back()[3] != CMD_LINK_STATE) {
            std::cerr << "Connected event did not transmit a link-state command\n";
            return 1;
        }
        const auto frame = makePacket(CMD_FW_VERSION, "1.2.3");
        port->injectReceivedData(frame.data(), 3);
        port->injectReceivedData(frame.data() + 3, frame.size() - 3);
        std::unique_lock lock(mutex);
        if (!condition.wait_for(lock, std::chrono::seconds(2), [&] { return gotVersion; })) {
            std::cerr << "Fragmented manual serial packet did not reach Core\n";
            return 1;
        }
        lock.unlock();
        unlisten();
    }

    port->notifyConnectionState(PORT_ACCESSOR_CONNECTION_STATE_DISCONNECTED);
    if (port->isOpen()) {
        std::cerr << "Disconnect did not close the manual transport\n";
        return 1;
    }
    return 0;
}
