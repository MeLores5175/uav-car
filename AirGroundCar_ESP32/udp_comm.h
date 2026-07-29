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

    // 收到地面站命令后，记住实际来源地址。
    // 后续 HB/TEL/EVT 均发回这个地址，避免电脑 DHCP 地址变化导致“ACK 能收到、遥测收不到”。
    IPAddress groundStationIp_;
    uint16_t groundStationPort_ = 0;
    bool groundStationKnown_ = false;

    void updateWifi();
    void receivePackets(MissionManager& mission);
    void handleCommand(const String& packet,
                       const IPAddress& remoteIp,
                       uint16_t remotePort,
                       MissionManager& mission);

    void reply(const IPAddress& ip,
               uint16_t port,
               const String& message);
    void rememberGroundStation(const IPAddress& ip, uint16_t port);
    static String tokenAt(const String& text, int index);
};

#endif
