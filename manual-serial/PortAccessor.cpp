#include "PortAccessor.h"

#include <stdexcept>
#include <utility>

PortAccessor::~PortAccessor() {
    try { closePort(); } catch (...) {}
}

void PortAccessor::bindSerialCallbacks(ManualSerialDriverCallbacks callbacks) {
    std::lock_guard lock(stateMutex);
    if (opened) throw std::logic_error("Bind serial callbacks before opening the port");
    serialCallbacks = std::move(callbacks);
}

void PortAccessor::openPort() {
    ManualSerialDriverCallbacks callbacks;
    {
        std::lock_guard lock(stateMutex);
        if (opened) return;
        callbacks = serialCallbacks;
    }
    if (callbacks.open && !callbacks.open()) throw std::runtime_error("Manual serial open failed");
    opened = true;
}

void PortAccessor::closePort() {
    if (!opened.exchange(false)) return;
    readRequests = 0;
    std::function<void()> close;
    {
        std::lock_guard lock(stateMutex);
        close = serialCallbacks.close;
    }
    if (close) close();
    notifyConnectionState(PORT_ACCESSOR_CONNECTION_STATE_DISCONNECTED);
}

bool PortAccessor::isOpen() const { return opened; }

void PortAccessor::setBaudRate(int baudRate) {
    std::function<void(int)> configure;
    {
        std::lock_guard lock(stateMutex);
        configure = serialCallbacks.setBaudRate;
    }
    if (configure) configure(baudRate);
}

void PortAccessor::setReadSize(std::size_t size) {
    std::lock_guard lock(stateMutex);
    readSize = size;
}

void PortAccessor::writeData(const std::vector<uint8_t> &data) {
    if (!opened) throw std::runtime_error("Manual serial port is closed");
    std::function<std::size_t(const uint8_t *, std::size_t)> write;
    {
        std::lock_guard lock(stateMutex);
        write = serialCallbacks.write;
    }
    if (!write) throw std::logic_error("Manual serial write callback is not bound");
    if (!data.empty() && write(data.data(), data.size()) != data.size())
        throw std::runtime_error("Manual serial write was incomplete");
}

void PortAccessor::writeData(const std::string &data) {
    writeData(std::vector<uint8_t>(data.begin(), data.end()));
}

bool PortAccessor::waitForPendingWrites(std::chrono::milliseconds timeout) const {
    std::function<bool(std::chrono::milliseconds)> wait;
    {
        std::lock_guard lock(stateMutex);
        wait = serialCallbacks.waitForPendingWrites;
    }
    return !wait || wait(timeout);
}

void PortAccessor::setPacketRealignmentHelper(std::unique_ptr<PacketRealignmentHelper> helper) {
    std::lock_guard lock(helperMutex);
    packetRealignmentHelper = std::move(helper);
}

bool PortAccessor::hasPacketRealignmentHelper() const {
    std::lock_guard lock(helperMutex);
    return static_cast<bool>(packetRealignmentHelper);
}

std::unique_ptr<PacketRealignmentHelper> PortAccessor::popPacketRealignmentHelper() {
    std::lock_guard lock(helperMutex);
    return std::move(packetRealignmentHelper);
}

std::function<void()> PortAccessor::addDataCallback(DataCallback callback) {
    auto slot = std::make_shared<CallbackSlot<DataCallback>>(std::move(callback));
    std::uint64_t id;
    {
        std::lock_guard lock(callbackMutex);
        id = ++nextCallbackId;
        dataCallbacks.emplace(id, slot);
    }
    return [this, id, slot] {
        std::lock_guard executionLock(slot->mutex);
        slot->active = false;
        std::lock_guard callbackLock(callbackMutex);
        dataCallbacks.erase(id);
    };
}

std::function<void()> PortAccessor::addConnectionCallback(ConnectionCallback callback) {
    auto slot = std::make_shared<CallbackSlot<ConnectionCallback>>(std::move(callback));
    std::uint64_t id;
    {
        std::lock_guard lock(callbackMutex);
        id = ++nextCallbackId;
        connectionCallbacks.emplace(id, slot);
    }
    return [this, id, slot] {
        std::lock_guard executionLock(slot->mutex);
        slot->active = false;
        std::lock_guard callbackLock(callbackMutex);
        connectionCallbacks.erase(id);
    };
}

void PortAccessor::startContinuousRead() { ++readRequests; }

void PortAccessor::stopContinuousRead() {
    int count = readRequests.load();
    while (count > 0 && !readRequests.compare_exchange_weak(count, count - 1)) {}
}

void PortAccessor::injectReceivedData(const uint8_t *data, std::size_t size) {
    if (!size || !opened || readRequests <= 0) return;
    if (!data) throw std::invalid_argument("Manual serial data pointer is null");
    std::vector<std::vector<uint8_t>> packets;
    {
        std::lock_guard lock(helperMutex);
        std::vector<uint8_t> chunk(data, data + size);
        packets = packetRealignmentHelper ? packetRealignmentHelper->processPacket(chunk)
                                          : std::vector<std::vector<uint8_t>>{std::move(chunk)};
    }
    std::vector<std::shared_ptr<CallbackSlot<DataCallback>>> listeners;
    {
        std::lock_guard lock(callbackMutex);
        for (const auto &entry : dataCallbacks) listeners.push_back(entry.second);
    }
    for (auto &packet : packets) {
        auto payload = std::make_shared<std::vector<uint8_t>>(std::move(packet));
        for (const auto &slot : listeners) {
            std::lock_guard executionLock(slot->mutex);
            if (slot->active) slot->callback(payload);
        }
    }
}

void PortAccessor::injectReceivedData(const std::vector<uint8_t> &data) {
    injectReceivedData(data.data(), data.size());
}

void PortAccessor::notifyConnectionState(PortAccessorConnectionState state) {
    opened = state == PORT_ACCESSOR_CONNECTION_STATE_CONNECTED;
    std::vector<std::shared_ptr<CallbackSlot<ConnectionCallback>>> listeners;
    {
        std::lock_guard lock(callbackMutex);
        for (const auto &entry : connectionCallbacks) listeners.push_back(entry.second);
    }
    for (const auto &slot : listeners) {
        std::lock_guard executionLock(slot->mutex);
        if (slot->active) slot->callback(state);
    }
}
