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
    void sendCarTelemetry(const String& message);
    void notifyMissionStarted(const MissionManager& mission);
    void notifyPoint(const MissionManager& mission, const String& point);
    void notifyMissionDone(const MissionManager& mission);

private:
    WiFiUDP udp_;
    bool udpStarted_ = false;
    bool statusRequested_ = false;
    uint32_t carCommandSequence_ = 1000;

    // 地面站 IP 可通过收到的合法命令动态学习，解决电脑 DHCP 地址变化问题。
    IPAddress groundStationIp_;
    uint16_t groundStationPort_ = 0;
    bool groundStationKnown_ = false;

    // 小车 -> 无人机 START 可靠发送状态。
    bool startPending_ = false;
    String startCommandId_;
    String startRunId_;
    String startPacket_;
    uint8_t startSendCount_ = 0;
    uint32_t lastStartSendMs_ = 0;

    void updateWifi();
    void receivePackets(MissionManager& mission);
    void handlePacket(const String& packet,
                      const IPAddress& remoteIp,
                      uint16_t remotePort,
                      MissionManager& mission);
    void handleCommand(const String& packet,
                       const IPAddress& remoteIp,
                       uint16_t remotePort,
                       MissionManager& mission);
    void handleUavReply(const String& packet,
                        const IPAddress& remoteIp,
                        uint16_t remotePort);
    void updateStartRetry();

    void reply(const IPAddress& ip,
               uint16_t port,
               const String& message);
    void rememberGroundStation(const IPAddress& ip, uint16_t port);
    static String tokenAt(const String& text, int index);
};

#endif
