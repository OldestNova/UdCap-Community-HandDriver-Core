#ifndef UDCAP_MANUAL_PORT_ACCESSOR_H
#define UDCAP_MANUAL_PORT_ACCESSOR_H

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

// The manual build intentionally does not include SerialReborn, Boost, USB or HID headers.
class PacketRealignmentHelper {
public:
    virtual ~PacketRealignmentHelper() = default;
    PacketRealignmentHelper(const PacketRealignmentHelper &) = delete;
    PacketRealignmentHelper &operator=(const PacketRealignmentHelper &) = delete;
    virtual std::vector<std::vector<uint8_t>> processPacket(const std::vector<uint8_t> &data) = 0;

protected:
    PacketRealignmentHelper() = default;
};

enum PortAccessorConnectionState {
    PORT_ACCESSOR_CONNECTION_STATE_DISCONNECTED,
    PORT_ACCESSOR_CONNECTION_STATE_CONNECTED
};

struct ManualSerialDriverCallbacks {
    // Optional. A missing open callback means the transport is already available.
    std::function<bool()> open;
    std::function<void()> close;
    std::function<void(int baudRate)> setBaudRate;
    // Required for transmitting commands. Return the number of bytes accepted.
    std::function<std::size_t(const uint8_t *data, std::size_t size)> write;
    // Optional. If writes are queued by the host, wait for that queue to drain.
    std::function<bool(std::chrono::milliseconds timeout)> waitForPendingWrites;
};

class PortAccessor {
public:
    PortAccessor() = default;
    ~PortAccessor();
    PortAccessor(const PortAccessor &) = delete;
    PortAccessor &operator=(const PortAccessor &) = delete;

    // Bind before constructing UdCapV1Core or calling UdCapProbe::probe().
    void bindSerialCallbacks(ManualSerialDriverCallbacks callbacks);
    void openPort();
    void closePort();
    bool isOpen() const;
    void setBaudRate(int baudRate);
    void setReadSize(std::size_t size);
    void writeData(const std::vector<uint8_t> &data);
    void writeData(const std::string &data);
    bool waitForPendingWrites(std::chrono::milliseconds timeout) const;

    void setPacketRealignmentHelper(std::unique_ptr<PacketRealignmentHelper> helper);
    bool hasPacketRealignmentHelper() const;
    std::unique_ptr<PacketRealignmentHelper> popPacketRealignmentHelper();
    std::function<void()> addDataCallback(std::function<void(std::shared_ptr<std::vector<uint8_t>>)> callback);
    std::function<void()> addConnectionCallback(std::function<void(PortAccessorConnectionState)> callback);
    void startContinuousRead();
    void stopContinuousRead();

    // Feed raw bytes in order, including arbitrary serial read boundaries. Callbacks run
    // synchronously on the caller's thread, so do not call this from an interrupt handler.
    void injectReceivedData(const uint8_t *data, std::size_t size);
    void injectReceivedData(const std::vector<uint8_t> &data);
    // Report physical disconnect/reconnect after binding; reconnect permits writes again.
    void notifyConnectionState(PortAccessorConnectionState state);

private:
    template <typename Callback> struct CallbackSlot {
        explicit CallbackSlot(Callback value) : callback(std::move(value)) {}
        Callback callback;
        std::recursive_mutex mutex;
        bool active = true;
    };
    using DataCallback = std::function<void(std::shared_ptr<std::vector<uint8_t>>)>;
    using ConnectionCallback = std::function<void(PortAccessorConnectionState)>;

    mutable std::mutex stateMutex;
    ManualSerialDriverCallbacks serialCallbacks;
    std::atomic_bool opened{false};
    std::atomic_int readRequests{0};
    std::size_t readSize = 64;

    mutable std::mutex helperMutex;
    std::unique_ptr<PacketRealignmentHelper> packetRealignmentHelper;
    std::mutex callbackMutex;
    std::uint64_t nextCallbackId = 0;
    std::map<std::uint64_t, std::shared_ptr<CallbackSlot<DataCallback>>> dataCallbacks;
    std::map<std::uint64_t, std::shared_ptr<CallbackSlot<ConnectionCallback>>> connectionCallbacks;
};

#endif
