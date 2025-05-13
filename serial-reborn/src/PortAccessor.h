//
// Created by max_3 on 2025/5/11.
//

#ifndef SERIALREBORN_PORTACCESSOR_H
#define SERIALREBORN_PORTACCESSOR_H

#include "SerialCommon.h"
#include <boost/asio.hpp>
#include "BoostHelper.hpp"
#include <vector>
#include <functional>
#include <thread>

enum StopBit {
    STOPBIT_1,
    STOPBIT_1_5,
    STOPBIT_2
};

enum Parity {
    P_NONE, P_EVEN, P_ODD
};

enum FlowControl {
    FC_NONE, FC_SOFTWARE, FC_HARDWARE
};

class PacketRealignmentHelper {
public:
    PacketRealignmentHelper() = default;
    virtual ~PacketRealignmentHelper() = default;
    PacketRealignmentHelper(const PacketRealignmentHelper&) = delete;
    virtual std::vector<std::vector<uint8_t>> processPacket(const std::vector<uint8_t>& data) = 0;
};

class PortAccessor {
public:
    explicit PortAccessor(const SerialDevice& port);
    ~PortAccessor();
    PortAccessor(const PortAccessor&) = delete;
    PortAccessor& operator=(const PortAccessor&) = delete;
    static std::shared_ptr<PortAccessor> create(const SerialDevice& port);
    void openPort();
    void closePort();
    bool isOpen() const;
    void setBaudRate(int baudRate);
    void setStopBit(StopBit stopBit);
    void setParity(Parity parity);
    void setFlowControl(FlowControl flowControl);
    void setDataBits(int dataBits);
    void setReadSize(size_t size);
    void setTimeout(size_t timeout);
    std::vector<uint8_t> readData();
    size_t writeData(const std::vector<uint8_t>&);
    size_t writeData(const std::string&);

    void setPacketRealignmentHelper(std::unique_ptr<PacketRealignmentHelper> helper);
    bool hasPacketRealignmentHelper() const;
    std::function<void()> addDataCallback(const std::function<void(const std::vector<uint8_t> &)> &);
    void startContinuousRead();
    void stopContinuousRead();
private:
    SerialDevice serialDevice;
    boost::asio::io_context io;
    std::unique_ptr<DeadlineSocket<boost::asio::serial_port>> serialPort;
    hid_device* hidDevice = nullptr;
    size_t readSize = 64;
    size_t timeout = 500;
    bool isOpenFlag = false;
    std::thread continuousReadThread;
    std::mutex callbackMutex;
    std::atomic_bool continuousReadRunning = false;
    std::atomic_int32_t continuousReadStartCount = 0;
    std::unique_ptr<PacketRealignmentHelper> packetRealignmentHelper;
    std::vector<std::function<void(const std::vector<uint8_t> &)>> dataCallbacks;
};



#endif //SERIALREBORN_PORTACCESSOR_H
