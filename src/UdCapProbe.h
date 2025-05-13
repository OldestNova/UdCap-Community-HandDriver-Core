//
// Created by max_3 on 2025/5/11.
//

#ifndef SERIALREBORN_UDCAPPROBE_H
#define SERIALREBORN_UDCAPPROBE_H

#include <SerialCommon.h>
#include <PortAccessor.h>

enum UdCapProbeType {
    UDCAP_PROBE_FAILURE = 0,
    UDCAP_PROBE_HAND_V1 = 1
};

class UdCapProbe {
public:
    explicit UdCapProbe(std::shared_ptr<PortAccessor> portAccessor);
    ~UdCapProbe();

    UdCapProbeType probe();
    std::string getUDCapSerial();
private:
    std::shared_ptr<PortAccessor> portAccessor;
    std::string udCapSerial;
};


#endif //SERIALREBORN_UDCAPPROBE_H
