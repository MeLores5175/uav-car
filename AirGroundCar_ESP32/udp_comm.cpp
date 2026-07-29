#include "udp_comm.h"
#include "mission_config.h"

void UdpComm::begin()
{
    if (!CarConfig::ENABLE_WIFI_UDP) {
        Serial.println("[UDP] disabled by configuration");
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(CarConfig::WIFI_SSID, CarConfig::WIFI_PASSWORD);
    Serial.println("[UDP] Wi-Fi connecting...");
}

void UdpComm::update(MissionManager& mission)
{
    if (!CarConfig::ENABLE_WIFI_UDP) {
        return;
    }

    updateWifi();
    receivePackets(mission);
}

bool UdpComm::online() const
{
    return udpStarted_ && WiFi.status() == WL_CONNECTED;
}

bool UdpComm::consumeStatusRequest()
{
    const bool requested = statusRequested_;
    statusRequested_ = false;
    return requested;
}

void UdpComm::updateWifi()
{
    if (WiFi.status() != WL_CONNECTED) {
        udpStarted_ = false;
        return;
    }

    if (!udpStarted_) {
        udpStarted_ = udp_.begin(CarConfig::CAR_PORT) == 1;
        if (udpStarted_) {
            Serial.printf("[UDP] listening on %s:%u\n",
                          WiFi.localIP().toString().c_str(),
                          CarConfig::CAR_PORT);
        }
    }
}

void UdpComm::receivePackets(MissionManager& mission)
{
    if (!udpStarted_) {
        return;
    }

    int packetSize = udp_.parsePacket();
    while (packetSize > 0) {
        char buffer[256];
        const int readLength = udp_.read(buffer, sizeof(buffer) - 1);
        if (readLength > 0) {
            buffer[readLength] = '\0';
            String packet(buffer);
            packet.trim();
            handleCommand(packet,
                          udp_.remoteIP(),
                          udp_.remotePort(),
                          mission);
        }
        packetSize = udp_.parsePacket();
    }
}

void UdpComm::handleCommand(const String& packet,
                            const IPAddress& remoteIp,
                            uint16_t remotePort,
                            MissionManager& mission)
{
    if (!packet.startsWith("CMD:")) {
        reply(remoteIp, remotePort, "ERR:0000:UNKNOWN:BAD_FORMAT");
        return;
    }

    const String cmdId = tokenAt(packet, 1);
    const String action = tokenAt(packet, 2);

    if (action == "PING") {
        reply(remoteIp, remotePort,
              "ACK:" + cmdId + ":PING:OK:CAR");
        return;
    }

    if (action == "STATUS") {
        statusRequested_ = true;
        reply(remoteIp, remotePort,
              "ACK:" + cmdId + ":STATUS:OK:CAR");
        return;
    }

    if (action == "MODE") {
        const String task = tokenAt(packet, 3);
        MissionId id = MissionId::NONE;
        if (task == "T1") {
            id = MissionId::TASK_1;
        } else if (task == "T2") {
            id = MissionId::TASK_2;
        }

        if (id == MissionId::NONE) {
            reply(remoteIp, remotePort,
                  "ERR:" + cmdId + ":MODE:BAD_TASK");
        } else if (!mission.selectMission(id)) {
            reply(remoteIp, remotePort,
                  "ERR:" + cmdId + ":MODE:BUSY");
        } else {
            reply(remoteIp, remotePort,
                  "ACK:" + cmdId + ":MODE:OK:" + task);
            sendToGroundStation("EVT:CAR_READY:" + task);
        }
        return;
    }

    if (action == "ARM") {
        const String runId = tokenAt(packet, 3);
        if (mission.arm(runId)) {
            reply(remoteIp, remotePort,
                  "ACK:" + cmdId + ":ARM:OK:" + runId);
        } else {
            reply(remoteIp, remotePort,
                  "ERR:" + cmdId + ":ARM:NOT_READY");
        }
        return;
    }

    if (action == "START") {
        if (!CarConfig::ALLOW_REMOTE_START) {
            reply(remoteIp, remotePort,
                  "ERR:" + cmdId +
                  ":START:LOCAL_BUTTON_REQUIRED");
        } else if (mission.remoteStart()) {
            reply(remoteIp, remotePort,
                  "ACK:" + cmdId + ":START:OK:" + mission.runId());
        } else {
            reply(remoteIp, remotePort,
                  "ERR:" + cmdId + ":START:NOT_READY");
        }
        return;
    }

    if (action == "RESET") {
        mission.reset();
        reply(remoteIp, remotePort,
              "ACK:" + cmdId + ":RESET:OK");
        return;
    }

    reply(remoteIp, remotePort,
          "ERR:" + cmdId + ":" + action + ":UNKNOWN_CMD");
}

void UdpComm::reply(const IPAddress& ip,
                    uint16_t port,
                    const String& message)
{
    if (!udpStarted_) {
        return;
    }

    udp_.beginPacket(ip, port);
    udp_.print(message);
    udp_.endPacket();
}

void UdpComm::sendToGroundStation(const String& message)
{
    Serial.println(message);
    reply(CarConfig::GS_IP, CarConfig::GS_PORT, message);
}

void UdpComm::sendToUav(const String& message)
{
    Serial.println(message);
    reply(CarConfig::UAV_IP, CarConfig::UAV_PORT, message);
}

void UdpComm::notifyMissionStarted(const MissionManager& mission)
{
    const String task = mission.profile().name;
    const String runId = mission.runId();
    const String commandId = String(++carCommandSequence_);

    sendToUav("CMD:" + commandId + ":START:" + runId + ":" + task);
    sendToGroundStation("EVT:MISSION_START:" + runId + ":" + task);

    // TODO：下一版增加 UAV ACK 超时重发和命令去重。
}

void UdpComm::notifyPoint(const MissionManager& mission, const String& point)
{
    sendToGroundStation("EVT:CAR_POINT:" + mission.runId() + ":" + point);
    sendToUav("EVT:CAR_POINT:" + mission.runId() + ":" + point);
}

void UdpComm::notifyMissionDone(const MissionManager& mission)
{
    sendToGroundStation("EVT:MISSION_DONE:" + mission.runId());
}

String UdpComm::tokenAt(const String& text, int index)
{
    int tokenStart = 0;
    int tokenIndex = 0;

    for (int i = 0; i <= text.length(); ++i) {
        if (i == text.length() || text.charAt(i) == ':') {
            if (tokenIndex == index) {
                return text.substring(tokenStart, i);
            }
            tokenStart = i + 1;
            ++tokenIndex;
        }
    }
    return "";
}
