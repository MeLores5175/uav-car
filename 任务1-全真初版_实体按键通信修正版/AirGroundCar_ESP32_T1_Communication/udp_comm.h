#ifndef UDP_COMM_H
#define UDP_COMM_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include "car_config.h"
#include "mission_manager.h"

class UdpComm {
public:
    void begin();
    void update(MissionManager& mission);

    bool online() const;
    bool consumeStatusRequest();

    void sendToGroundStation(const String& message);
    void sendToUav(const String& message);
    void sendCarTelemetry(const String& message,
                          bool sendToUav = true);

    void notifyCarReady(const MissionManager& mission);
    void notifyCountdownStarted(const MissionManager& mission,
                                uint32_t countdownMs);
    void notifyCountdownCancelled();
    void notifyMissionStarted(const MissionManager& mission);
    void notifyPoint(const MissionManager& mission, const String& point);
    void notifyMissionDone(const MissionManager& mission);
    void notifyLocalStopped(const MissionManager& mission);

private:
    struct CachedReply {
        bool valid = false;
        String key;
        String response;
    };

    WiFiUDP udp_;
    bool udpStarted_ = false;
    bool statusRequested_ = false;
    uint32_t carCommandSequence_ = 1000;

    IPAddress groundStationIp_;
    uint16_t groundStationPort_ = 0;
    bool groundStationKnown_ = false;

    bool startPending_ = false;
    String startCommandId_;
    String startRunId_;
    String startPacket_;
    uint8_t startSendCount_ = 0;
    uint32_t lastStartSendMs_ = 0;

    CachedReply commandCache_[CarConfig::COMMAND_DEDUP_CACHE_SIZE];
    uint8_t commandCacheCursor_ = 0;

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
    void replyAndCache(const IPAddress& ip,
                       uint16_t port,
                       const String& cmdId,
                       const String& action,
                       const String& message);
    bool findCachedReply(const IPAddress& ip,
                         uint16_t port,
                         const String& cmdId,
                         const String& action,
                         String& response) const;
    String commandKey(const IPAddress& ip,
                      uint16_t port,
                      const String& cmdId,
                      const String& action) const;

    void rememberGroundStation(const IPAddress& ip, uint16_t port);
    void sendCriticalToGroundStation(const String& message);
    void sendCriticalToBoth(const String& message);
    static String tokenAt(const String& text, int index);
};

#endif
