#include "udp_comm.h"
#include "mission_config.h"

void UdpComm::begin()
{
    if (!CarConfig::ENABLE_WIFI_UDP) {
        Serial.println("[UDP] disabled by configuration");
        return;
    }

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);

    const bool staticIpOk = WiFi.config(CarConfig::CAR_IP,
                                        CarConfig::WIFI_GATEWAY,
                                        CarConfig::WIFI_SUBNET,
                                        CarConfig::WIFI_DNS1,
                                        CarConfig::WIFI_DNS2);
    Serial.printf("[UDP] static IP %s: %s\n",
                  CarConfig::CAR_IP.toString().c_str(),
                  staticIpOk ? "configured" : "FAILED");

    WiFi.begin(CarConfig::WIFI_SSID, CarConfig::WIFI_PASSWORD);
    Serial.printf("[UDP] Wi-Fi connecting to %s...\n", CarConfig::WIFI_SSID);
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
        if (udpStarted_) {
            udp_.stop();
            udpStarted_ = false;
            Serial.println("[UDP] Wi-Fi disconnected, UDP socket stopped");
        }
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

    // 地面站的发送 socket 固定绑定在 8889，但电脑 IP 可能由 DHCP 分配。
    // 只要收到合法 CMD，就把后续异步遥测和事件发回实际来源端点。
    rememberGroundStation(remoteIp, remotePort);

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
        const String runId = tokenAt(packet, 3);

        if (!CarConfig::ALLOW_REMOTE_START) {
            reply(remoteIp, remotePort,
                  "ERR:" + cmdId +
                  ":START:LOCAL_BUTTON_REQUIRED");
            return;
        }

        if (runId.length() == 0) {
            reply(remoteIp, remotePort,
                  "ERR:" + cmdId + ":START:MISSING_RUN_ID");
            return;
        }

        // 地面站当前直接发送 START。
        // 在仿真模式下，收到 START 后自动完成 ARM。
        if (mission.state() == MissionState::READY) {
            if (!mission.arm(runId)) {
                reply(remoteIp, remotePort,
                      "ERR:" + cmdId + ":START:ARM_FAILED");
                return;
            }
        }

        // 兼容地面站命令重发：
        // 同一个 run_id 已经运行时，再次回复成功，但不重复启动。
        if (mission.state() == MissionState::RUNNING &&
            mission.runId() == runId) {
            reply(remoteIp, remotePort,
                  "ACK:" + cmdId + ":START:OK:" + runId);
            return;
        }

        if (mission.state() != MissionState::ARMED) {
            reply(remoteIp, remotePort,
                  "ERR:" + cmdId + ":START:NOT_READY");
            return;
        }

        if (mission.remoteStart()) {
            reply(remoteIp, remotePort,
                  "ACK:" + cmdId + ":START:OK:" + mission.runId());
        } else {
            reply(remoteIp, remotePort,
                  "ERR:" + cmdId + ":START:FAILED");
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
    if (groundStationKnown_) {
        reply(groundStationIp_, groundStationPort_, message);
        return;
    }

    // 上电后、尚未收到地面站命令前使用配置中的回退地址。
    reply(CarConfig::GS_IP, CarConfig::GS_PORT, message);
}

void UdpComm::rememberGroundStation(const IPAddress& ip, uint16_t port)
{
    if (!groundStationKnown_ || !(groundStationIp_ == ip) || groundStationPort_ != port) {
        groundStationIp_ = ip;
        groundStationPort_ = port;
        groundStationKnown_ = true;
        Serial.printf("[UDP] ground station learned: %s:%u\n",
                      groundStationIp_.toString().c_str(),
                      groundStationPort_);
    }
}

void UdpComm::sendToUav(const String& message)
{
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
