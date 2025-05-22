//
// Created by max_3 on 2025/5/11.
//

#include "PortAccessor.h"
#include <hidapi.h>
#include <iomanip>
#include <iostream>

PortAccessor::PortAccessor(const SerialDevice &port): serialDevice(port), io() {
    if (!port.isHid) {
        serialPort = std::make_unique<DeadlineSocket<boost::asio::serial_port> >(io);
    }
    std::thread rxThread([this]() {
        while (eventLoopRunning.load()) {
            if (rxQueue.empty()) {
                std::unique_lock lk(rxQueueMutex);
                rxQueueCondition.wait_for(lk, std::chrono::milliseconds(100));
                continue;
            }
            try {
                ReceiveDataItem item = rxQueue.front();
                std::vector<uint8_t> packet = item.byteData;
                rxQueue.pop();
                std::lock_guard guard(callbackMutex);
                if (item.isOnceRaw) {
                    std::vector<uint32_t> removable;
                    for (auto &pair: this->onceRawDataCallbacks) {
                        if (pair.second(packet)) {
                            removable.push_back(pair.first);
                        }
                    }
                    for (auto &fd: removable) {
                        this->onceRawDataCallbacks.erase(fd);
                    }
                } else {
                    if (printRxTxToStdOut) {
                        std::cout << "Receive: ";
                        for (uint8_t b: packet) {
                            std::cout << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(b) << " ";
                        }
                        std::cout << std::endl;
                    }
                    for (auto &pair: this->dataCallbacks) {
                        pair.second(packet);
                    }
                }
            } catch (std::exception &e) {
            }
        }
    });
    rxThread.detach();
    rxEventThread = std::move(rxThread);

    std::thread txThread([this]() {
        while (eventLoopRunning.load()) {
            if (txQueue.empty()) {
                std::unique_lock lk(txQueueMutex);
                txQueueCondition.wait_for(lk, std::chrono::milliseconds(100));
                continue;
            }
            try {
                SendDataItem packet = txQueue.front();
                txQueue.pop();
                uint16_t delay = txEventLoopSendDelay.load();
                if (delay > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
                }
                if (packet.isString) {
                    if (printRxTxToStdOut) {
                        std::cout << "Send(String): ";
                        std::cout << packet.strData;
                        std::cout << std::endl;
                    }
                    writeDataToDevice(boost::asio::buffer((packet.strData)));
                } else {
                    if (printRxTxToStdOut) {
                        std::cout << "Send(Hex): ";
                        for (uint8_t b: packet.byteData) {
                            std::cout << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(b) << " ";
                        }
                        std::cout << std::endl;
                    }
                    writeDataToDevice(boost::asio::buffer((packet.byteData)));
                }
            } catch (std::exception &e) {
            }
        }
    });
    txThread.detach();
    txEventThread = std::move(txThread);
}

PortAccessor::~PortAccessor() {
    eventLoopRunning.store(false);
    if (continuousReadRunning.load()) {
        continuousReadRunning.store(false);
        if (continuousReadThread.joinable()) {
            continuousReadThread.join();
        }
    }
    closePort();
}

void PortAccessor::openPort() {
    if (isOpen()) return;
    if (serialDevice.isHid) {
        std::wstring wideSerialNumber =
                std::wstring(serialDevice.serialNumber.begin(), serialDevice.serialNumber.end());
        if (wideSerialNumber.empty()) {
            hidDevice = hid_open(serialDevice.vid, serialDevice.pid, nullptr);
        } else {
            hidDevice = hid_open(serialDevice.vid, serialDevice.pid, wideSerialNumber.c_str());
        }
        if (!hidDevice) {
            throw std::runtime_error("Failed to open HID device");
        }
    } else {
        serialPort->socket().open(serialDevice.portName);
        size_t waitOpen = 5;
        while (!serialPort->socket().is_open()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (--waitOpen == 0) {
                throw std::runtime_error("Failed to open serial port");
            }
        }
    }
    isOpenFlag = true;
}

bool PortAccessor::isOpen() const {
    return isOpenFlag;
}

void PortAccessor::closePort() {
    if (!isOpen()) {
        return;
    }
    if (serialDevice.isHid) {
        if (hidDevice) {
            hid_close(hidDevice);
            hidDevice = nullptr;
        }
    } else {
        serialPort->socket().close();
    }
    isOpenFlag = false;
}

void PortAccessor::setBaudRate(int baudRate) {
    if (serialDevice.isHid) {
        throw std::runtime_error("Cannot set baud rate for HID device");
    } else {
        serialPort->socket().set_option(boost::asio::serial_port_base::baud_rate(baudRate));
    }
}

void PortAccessor::setStopBit(StopBit stopBit) {
    if (serialDevice.isHid) {
        throw std::runtime_error("Cannot set stop bit for HID device");
    } else {
        switch (stopBit) {
            case StopBit::STOPBIT_1:
                serialPort->socket().set_option(
                    boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
                break;
            case StopBit::STOPBIT_2:
                serialPort->socket().set_option(
                    boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::two));
                break;
            case StopBit::STOPBIT_1_5:
                serialPort->socket().set_option(
                    boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::onepointfive));
                break;
            default:
                throw std::runtime_error("Invalid stop bit value");
        }
    }
}

void PortAccessor::setParity(Parity parity) {
    if (serialDevice.isHid) {
        throw std::runtime_error("Cannot set parity for HID device");
    } else {
        switch (parity) {
            case P_NONE:
                serialPort->socket().set_option(
                    boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                break;
            case P_EVEN:
                serialPort->socket().set_option(
                    boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::even));
                break;
            case P_ODD:
                serialPort->socket().set_option(
                    boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::odd));
                break;
            default:
                throw std::runtime_error("Invalid parity value");
        }
    }
}

void PortAccessor::setFlowControl(FlowControl flowControl) {
    if (serialDevice.isHid) {
        throw std::runtime_error("Cannot set flow control for HID device");
    } else {
        switch (flowControl) {
            case FC_NONE:
                serialPort->socket().set_option(
                    boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
                break;
            case FC_HARDWARE:
                serialPort->socket().set_option(
                    boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::hardware));
                break;
            case FC_SOFTWARE:
                serialPort->socket().set_option(
                    boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::software));
                break;
            default:
                throw std::runtime_error("Invalid flow control value");
        }
    }
}

void PortAccessor::setDataBits(int dataBits) {
    if (serialDevice.isHid) {
        throw std::runtime_error("Cannot set data bits for HID device");
    } else {
        if (dataBits < 5 || dataBits > 8) {
            throw std::runtime_error("Invalid data bits value");
        }
        serialPort->socket().set_option(boost::asio::serial_port_base::character_size(dataBits));
    }
}

void PortAccessor::setReadSize(size_t size) {
    readSize = size;
}

void PortAccessor::setTimeout(size_t timeout) {
    this->timeout = timeout;
}

std::vector<uint8_t> PortAccessor::readData() {
    if (isOpen()) {
        if (continuousReadRunning.load()) {
            throw std::runtime_error("Continuous read is running, cannot read data");
        }
        std::vector<uint8_t> buffer(readSize);
        if (serialDevice.isHid) {
            size_t transfered = hid_read_timeout(hidDevice, buffer.data(), buffer.size(), (int) timeout);
            buffer.resize(transfered);
        } else {
            uint8_t read = 4;
            while (read > 0) {
                try {
                    size_t transfered = serialPort->read(boost::asio::buffer(buffer),
                                                         std::chrono::milliseconds(this->timeout));
                    if (transfered == 0) {
                        read = read - 1;
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        continue;
                    }
                    buffer.resize(transfered);
                    read = 0;
                } catch (const boost::system::system_error &e) {
                    read = read - 1;
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
        }
        if (printRxTxToStdOut) {
            std::cout << "Receive: ";
            for (uint8_t b: buffer) {
                std::cout << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(b) << " ";
            }
            std::cout << std::endl;
        }
        return buffer;
    } else {
        throw std::runtime_error("Port is not open");
    }
}

void PortAccessor::writeData(const std::vector<uint8_t> &data) {
    if (isOpen()) {
        SendDataItem item;
        item.isString = false;
        item.byteData = data;
        txQueue.push(item);
        txQueueCondition.notify_all();
    } else {
        throw std::runtime_error("Port is not open");
    }
}

void PortAccessor::writeData(const std::string &data) {
    if (isOpen()) {
        SendDataItem item;
        item.isString = true;
        item.strData = data;
        txQueue.push(item);
        txQueueCondition.notify_all();
    } else {
        throw std::runtime_error("Port is not open");
    }
}

void PortAccessor::writeDataToDevice(const boost::asio::const_buffer &buffer) {
    if (isOpen()) {
        if (serialDevice.isHid) {
            hid_write(hidDevice, static_cast<const unsigned char *>(buffer.data()), buffer.size());
        } else {
            serialPort->write(buffer, std::chrono::milliseconds(this->timeout));
        }
    }
}


void PortAccessor::stopContinuousRead() {
    if (isOpen()) {
        if (continuousReadStartCount.load() > 1) {
            continuousReadStartCount.fetch_sub(1);
            return;
        }
        continuousReadStartCount.fetch_sub(1);
        continuousReadRunning.store(false);
        if (continuousReadThread.joinable()) {
            continuousReadThread.join();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else {
        throw std::runtime_error("Port is not open");
    }
}

void PortAccessor::setPacketRealignmentHelper(std::unique_ptr<PacketRealignmentHelper> helper) {
    if (continuousReadRunning.load()) {
        throw std::runtime_error("Cannot set packet realignment helper while continuous read is running");
    }
    packetRealignmentHelper = std::move(helper);
}

void PortAccessor::startContinuousRead() {
    if (isOpen()) {
        if (continuousReadStartCount.load() > 0) {
            continuousReadStartCount.fetch_add(1);
            return;
        }
        continuousReadStartCount.fetch_add(1);
        continuousReadRunning.store(true);
        continuousReadThread = std::thread([this] {
            while (continuousReadRunning.load()) {
                try {
                    std::vector<uint8_t> data(this->readSize);
                    try {
                        if (serialDevice.isHid) {
                            size_t size = hid_read_timeout(hidDevice, data.data(), this->readSize, (int) this->timeout);
                            data.resize(size);
                        } else {
                            size_t size = this->serialPort->read(boost::asio::buffer(data),
                                                                 std::chrono::milliseconds(this->timeout));
                            data.resize(size);
                        }
                    } catch (const boost::system::system_error &e) {
                        continue;
                    }
                    if (data.empty()) {
                        continue;
                    }

                    if (!onceRawDataCallbacks.empty()) {
                        ReceiveDataItem item{};
                        item.isOnceRaw = true;
                        item.byteData = data;
                        rxQueue.push(item);
                        rxQueueCondition.notify_all();
                    }

                    if (packetRealignmentHelper) {
                        std::vector<std::vector<uint8_t>> newPacket = packetRealignmentHelper->processPacket(data);
                        if (newPacket.empty()) {
                            continue;
                        } else {
                            for (const std::vector<uint8_t> &packet: newPacket) {
                                if (packet.empty()) {
                                    continue;
                                }
                                ReceiveDataItem item{};
                                item.isOnceRaw = false;
                                item.byteData = packet;
                                rxQueue.push(item);
                                rxQueueCondition.notify_all();
                            }
                        }
                    } else {
                        ReceiveDataItem item{};
                        item.isOnceRaw = false;
                        item.byteData = data;
                        rxQueue.push(item);
                        rxQueueCondition.notify_all();
                    }
                } catch (std::exception &e) {
                }
            }
        });
    } else {
        throw std::runtime_error("Port is not open");
    }
}

std::function<void()> PortAccessor::addDataCallback(const std::function<void(const std::vector<uint8_t> &)> &callback) {
    std::lock_guard guard(callbackMutex);
    uint32_t fd = callbackFd.fetch_add(1);
    dataCallbacks[fd] = callback;
    return [this, fd]() {
        dataCallbacks.erase(fd);
    };
}

std::function<void()> PortAccessor::addOnceRawDataCallback(
    const std::function<bool(const std::vector<uint8_t> &)> &callback) {
    std::lock_guard guard(callbackMutex);
    uint32_t fd = callbackFd.fetch_add(1);
    onceRawDataCallbacks[fd] = callback;
    return [this, fd]() {
        onceRawDataCallbacks.erase(fd);
    };
}

std::shared_ptr<PortAccessor> PortAccessor::create(const SerialDevice &port) {
    return std::make_shared<PortAccessor>(port);
}

bool PortAccessor::hasPacketRealignmentHelper() const {
    return packetRealignmentHelper != nullptr;
}

void PortAccessor::setWriteDelay(const uint64_t delay) {
    txEventLoopSendDelay.store(delay);
}

void PortAccessor::setPrintRxTxToStdOut(bool enable) {
    printRxTxToStdOut = enable;
}
