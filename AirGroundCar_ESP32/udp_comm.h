#ifndef UDP_COMM_H
#define UDP_COMM_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include "mission_manager.h"

class UdpComm {
public:
    void begin();
    void update(MissionManager& mission);

    bool online() const;
    bool consumeStatusRequest();

    void sendToGroundStation(const String& message);
    void sendToUav(const String& message);
    void notifyMissionStarted(const MissionManager& mission);
    void notifyPoint(const MissionManager& mission, const String& point);
    void notifyMissionDone(const MissionManager& mission);

private:
    WiFiUDP udp_;
    bool udpStarted_ = false;
    bool statusRequested_ = false;
    uint32_t carCommandSequence_ = 1000;

    void updateWifi();
    void receivePackets(MissionManager& mission);
    void handleCommand(const String& packet,
                       const IPAddress& remoteIp,
                       uint16_t remotePort,
                       MissionManager& mission);

    void reply(const IPAddress& ip,
               uint16_t port,
               const String& message);
    static String tokenAt(const String& text, int index);
};

#endif
