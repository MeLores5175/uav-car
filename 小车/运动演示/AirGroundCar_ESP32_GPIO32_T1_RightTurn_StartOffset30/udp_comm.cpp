#include "udp_comm.h"
#include "mission_config.h"

namespace {
constexpr uint32_t START_RETRY_INTERVAL_MS = 300;
constexpr uint8_t START_MAX_SEND_COUNT = 10;
}

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
    updateStartRetry();
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
        char buffer[512];
        const int readLength = udp_.read(buffer, sizeof(buffer) - 1);
        if (readLength > 0) {
            buffer[readLength] = '\0';
            String packet(buffer);
            packet.trim();
            handlePacket(packet,
                         udp_.remoteIP(),
                         udp_.remotePort(),
                         mission);
        }
        packetSize = udp_.parsePacket();
    }
}

void UdpComm::handlePacket(const String& packet,
                           const IPAddress& remoteIp,
                           uint16_t remotePort,
                           MissionManager& mission)
{
    if (packet.startsWith("CMD:")) {
        handleCommand(packet, remoteIp, remotePort, mission);
        return;
    }

    if (packet.startsWith("ACK:") || packet.startsWith("ERR:")) {
        handleUavReply(packet, remoteIp, remotePort);
        return;
    }

    reply(remoteIp, remotePort, "ERR:0000:UNKNOWN:BAD_FORMAT");
}

void UdpComm::handleCommand(const String& packet,
                            const IPAddress& remoteIp,
                            uint16_t remotePort,
                            MissionManager& mission)
{
    // 小车控制命令来自地面站，收到合法 CMD 后动态记住其地址。
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
                  "ERR:" + cmdId + ":START:LOCAL_BUTTON_REQUIRED");
            return;
        }

        if (runId.length() == 0) {
            reply(remoteIp, remotePort,
                  "ERR:" + cmdId + ":START:MISSING_RUN_ID");
            return;
        }

        // 地面站只向小车发送 START。小车先进入 RUNNING，再可靠通知无人机起飞。
        if (mission.state() == MissionState::READY) {
            if (!mission.arm(runId)) {
                reply(remoteIp, remotePort,
                      "ERR:" + cmdId + ":START:ARM_FAILED");
                return;
            }
        }

        // 兼容地面站 UDP 重发，同一 run_id 不重复启动。
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
        startPending_ = false;
        reply(remoteIp, remotePort,
              "ACK:" + cmdId + ":RESET:OK");
        return;
    }

    reply(remoteIp, remotePort,
          "ERR:" + cmdId + ":" + action + ":UNKNOWN_CMD");
}

void UdpComm::handleUavReply(const String& packet,
                             const IPAddress& remoteIp,
                             uint16_t remotePort)
{
    // 只接受协议约定的无人机地址和端口返回的 ACK/ERR。
    if (!(remoteIp == CarConfig::UAV_IP) || remotePort != CarConfig::UAV_PORT) {
        Serial.printf("[UDP] ignored reply from %s:%u: %s\n",
                      remoteIp.toString().c_str(),
                      remotePort,
                      packet.c_str());
        return;
    }

    const String prefix = tokenAt(packet, 0);
    const String cmdId = tokenAt(packet, 1);
    const String action = tokenAt(packet, 2);
    const String result = tokenAt(packet, 3);

    if (!startPending_ || cmdId != startCommandId_ || action != "START") {
        Serial.printf("[UDP] UAV reply: %s\n", packet.c_str());
        return;
    }

    if (prefix == "ACK" && result == "OK") {
        startPending_ = false;
        Serial.printf("[UDP] UAV START acknowledged: %s\n", packet.c_str());
        return;
    }

    // 明确的格式/模式错误不再盲目重发；NOT_READY 等瞬态错误仍按定时器重试。
    if (prefix == "ERR" &&
        (result == "BAD_FORMAT" || result == "BAD_TASK" || result == "MODE_MISMATCH")) {
        startPending_ = false;
        Serial.printf("[UDP][ERROR] UAV rejected START permanently: %s\n", packet.c_str());
        sendToGroundStation("EVT:UAV_START_FAILED:" + startRunId_ + ":" + result);
        return;
    }

    Serial.printf("[UDP] UAV START not ready, will retry: %s\n", packet.c_str());
}

void UdpComm::updateStartRetry()
{
    if (!startPending_ || !udpStarted_) {
        return;
    }

    const uint32_t nowMs = millis();
    if (startSendCount_ > 0 &&
        nowMs - lastStartSendMs_ < START_RETRY_INTERVAL_MS) {
        return;
    }

    if (startSendCount_ >= START_MAX_SEND_COUNT) {
        startPending_ = false;
        Serial.printf("[UDP][ERROR] UAV START ACK timeout, run=%s\n",
                      startRunId_.c_str());
        sendToGroundStation("EVT:UAV_START_FAILED:" + startRunId_ + ":ACK_TIMEOUT");
        return;
    }

    sendToUav(startPacket_);
    ++startSendCount_;
    lastStartSendMs_ = nowMs;
    Serial.printf("[UDP] UAV START send %u/%u: %s\n",
                  startSendCount_,
                  START_MAX_SEND_COUNT,
                  startPacket_.c_str());
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

void UdpComm::sendCarTelemetry(const String& message)
{
    // 同一份 TEL:CAR 同时供无人机控制和地面站显示使用。
    sendToUav(message);
    sendToGroundStation(message);
}

void UdpComm::notifyMissionStarted(const MissionManager& mission)
{
    const String task = mission.profile().name;
    const String runId = mission.runId();
    const String commandId = String(++carCommandSequence_);

    startCommandId_ = commandId;
    startRunId_ = runId;
    startPacket_ = "CMD:" + commandId + ":START:" + runId + ":" + task;
    startSendCount_ = 0;
    lastStartSendMs_ = 0;
    startPending_ = true;

    // 地面站计时从小车实际开始运动时开始；无人机 START 由上面的可靠重发完成。
    sendToGroundStation("EVT:MISSION_START:" + runId + ":" + task);
    updateStartRetry();
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
