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

    if (!CarConfig::USE_DHCP) {
        const bool staticIpOk = WiFi.config(
            CarConfig::CAR_STATIC_IP,
            CarConfig::WIFI_GATEWAY,
            CarConfig::WIFI_SUBNET,
            CarConfig::WIFI_DNS1,
            CarConfig::WIFI_DNS2);
        Serial.printf("[UDP] static IP %s: %s\n",
                      CarConfig::CAR_STATIC_IP.toString().c_str(),
                      staticIpOk ? "configured" : "FAILED");
    } else {
        Serial.println(
            "[UDP] DHCP enabled; router reservation is recommended for CAR");
    }

    WiFi.begin(CarConfig::WIFI_SSID, CarConfig::WIFI_PASSWORD);
    Serial.printf("[UDP] Wi-Fi connecting to %s...\n",
                  CarConfig::WIFI_SSID);
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
            Serial.println("[UDP] Wi-Fi disconnected, socket stopped");
        }
        return;
    }

    if (!udpStarted_) {
        udpStarted_ = udp_.begin(CarConfig::CAR_PORT) == 1;
        if (udpStarted_) {
            Serial.printf("[UDP] listening on %s:%u\n",
                          WiFi.localIP().toString().c_str(),
                          CarConfig::CAR_PORT);
            Serial.printf("[UDP] UAV target=%s:%u, GS fallback=%s:%u\n",
                          CarConfig::UAV_IP.toString().c_str(),
                          CarConfig::UAV_PORT,
                          CarConfig::GS_FALLBACK_IP.toString().c_str(),
                          CarConfig::GS_PORT);
        } else {
            Serial.println("[UDP][ERROR] socket bind failed");
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
        char buffer[768];
        const int readLength =
            udp_.read(buffer, sizeof(buffer) - 1);
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

    if (packet.startsWith("ACK:") ||
        packet.startsWith("ERR:")) {
        handleUavReply(packet, remoteIp, remotePort);
        return;
    }

    reply(remoteIp, remotePort,
          "ERR:0000:UNKNOWN:BAD_FORMAT");
}

void UdpComm::handleCommand(const String& packet,
                            const IPAddress& remoteIp,
                            uint16_t remotePort,
                            MissionManager& mission)
{
    rememberGroundStation(remoteIp, remotePort);

    const String cmdId = tokenAt(packet, 1);
    const String action = tokenAt(packet, 2);

    if (cmdId.length() == 0 || action.length() == 0) {
        reply(remoteIp, remotePort,
              "ERR:0000:UNKNOWN:BAD_FORMAT");
        return;
    }

    String cached;
    if (findCachedReply(remoteIp, remotePort,
                        cmdId, action, cached)) {
        reply(remoteIp, remotePort, cached);
        return;
    }

    if (action == "PING") {
        replyAndCache(
            remoteIp, remotePort, cmdId, action,
            "ACK:" + cmdId + ":PING:OK:CAR");
        return;
    }

    if (action == "STATUS") {
        statusRequested_ = true;
        replyAndCache(
            remoteIp, remotePort, cmdId, action,
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

        String response;
        if (id == MissionId::NONE) {
            response =
                "ERR:" + cmdId + ":MODE:BAD_TASK";
        } else if (!mission.selectMission(id)) {
            response =
                "ERR:" + cmdId + ":MODE:BUSY";
        } else {
            startPending_ = false;
            response =
                "ACK:" + cmdId + ":MODE:OK:" + task;
        }

        replyAndCache(remoteIp, remotePort,
                      cmdId, action, response);

        if (response.startsWith("ACK:")) {
            notifyCarReady(mission);
        }
        return;
    }

    if (action == "START") {
        // 正式流程已经改为实体按键启动；MODE 只选择模式。
        const String response =
            "ERR:" + cmdId +
            ":START:LOCAL_BUTTON_REQUIRED";
        replyAndCache(remoteIp, remotePort,
                      cmdId, action, response);
        return;
    }

    if (action == "RESET") {
        mission.reset();
        startPending_ = false;
        const String response =
            "ACK:" + cmdId + ":RESET:OK";
        replyAndCache(remoteIp, remotePort,
                      cmdId, action, response);
        return;
    }

    const String response =
        "ERR:" + cmdId + ":" + action + ":UNKNOWN_CMD";
    replyAndCache(remoteIp, remotePort,
                  cmdId, action, response);
}

void UdpComm::handleUavReply(const String& packet,
                             const IPAddress& remoteIp,
                             uint16_t remotePort)
{
    if (!(remoteIp == CarConfig::UAV_IP) ||
        remotePort != CarConfig::UAV_PORT) {
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

    if (!startPending_ ||
        cmdId != startCommandId_ ||
        action != "START") {
        Serial.printf("[UDP] UAV reply: %s\n",
                      packet.c_str());
        return;
    }

    if (prefix == "ACK" && result == "OK") {
        startPending_ = false;
        Serial.printf("[UDP] UAV START acknowledged: %s\n",
                      packet.c_str());
        sendCriticalToGroundStation(
            "EVT:UAV_START_ACK:" + startRunId_);
        return;
    }

    if (prefix == "ERR" &&
        (result == "BAD_FORMAT" ||
         result == "BAD_TASK" ||
         result == "MODE_MISMATCH")) {
        startPending_ = false;
        Serial.printf(
            "[UDP][ERROR] UAV rejected START: %s\n",
            packet.c_str());
        sendCriticalToGroundStation(
            "EVT:UAV_START_FAILED:" +
            startRunId_ + ":" + result);
        return;
    }

    Serial.printf(
        "[UDP] UAV START not ready, will retry: %s\n",
        packet.c_str());
}

void UdpComm::updateStartRetry()
{
    if (!startPending_ || !udpStarted_) {
        return;
    }

    const uint32_t nowMs = millis();
    if (startSendCount_ > 0 &&
        nowMs - lastStartSendMs_ <
            CarConfig::UAV_START_RETRY_INTERVAL_MS) {
        return;
    }

    if (startSendCount_ >=
        CarConfig::UAV_START_MAX_SEND_COUNT) {
        startPending_ = false;
        Serial.printf(
            "[UDP][ERROR] UAV START ACK timeout, run=%s\n",
            startRunId_.c_str());
        sendCriticalToGroundStation(
            "EVT:UAV_START_FAILED:" +
            startRunId_ + ":ACK_TIMEOUT");
        return;
    }

    sendToUav(startPacket_);
    ++startSendCount_;
    lastStartSendMs_ = nowMs;
    Serial.printf("[UDP] UAV START send %u/%u: %s\n",
                  startSendCount_,
                  CarConfig::UAV_START_MAX_SEND_COUNT,
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

String UdpComm::commandKey(const IPAddress& ip,
                           uint16_t port,
                           const String& cmdId,
                           const String& action) const
{
    return ip.toString() + ":" + String(port) + ":" +
           cmdId + ":" + action;
}

bool UdpComm::findCachedReply(const IPAddress& ip,
                              uint16_t port,
                              const String& cmdId,
                              const String& action,
                              String& response) const
{
    const String key =
        commandKey(ip, port, cmdId, action);
    for (uint8_t i = 0;
         i < CarConfig::COMMAND_DEDUP_CACHE_SIZE;
         ++i) {
        if (commandCache_[i].valid &&
            commandCache_[i].key == key) {
            response = commandCache_[i].response;
            return true;
        }
    }
    return false;
}

void UdpComm::replyAndCache(const IPAddress& ip,
                            uint16_t port,
                            const String& cmdId,
                            const String& action,
                            const String& message)
{
    reply(ip, port, message);

    CachedReply& slot =
        commandCache_[commandCacheCursor_];
    slot.valid = true;
    slot.key = commandKey(
        ip, port, cmdId, action);
    slot.response = message;

    commandCacheCursor_ =
        (commandCacheCursor_ + 1) %
        CarConfig::COMMAND_DEDUP_CACHE_SIZE;
}

void UdpComm::sendToGroundStation(
    const String& message)
{
    if (groundStationKnown_) {
        reply(groundStationIp_,
              groundStationPort_,
              message);
        return;
    }

    reply(CarConfig::GS_FALLBACK_IP,
          CarConfig::GS_PORT,
          message);
}

void UdpComm::rememberGroundStation(
    const IPAddress& ip,
    uint16_t port)
{
    if (!groundStationKnown_ ||
        !(groundStationIp_ == ip) ||
        groundStationPort_ != port) {
        groundStationIp_ = ip;
        groundStationPort_ = port;
        groundStationKnown_ = true;
        Serial.printf(
            "[UDP] ground station learned: %s:%u\n",
            groundStationIp_.toString().c_str(),
            groundStationPort_);
    }
}

void UdpComm::sendToUav(const String& message)
{
    reply(CarConfig::UAV_IP,
          CarConfig::UAV_PORT,
          message);
}

void UdpComm::sendCarTelemetry(
    const String& message,
    bool sendToUavTarget)
{
    if (sendToUavTarget) {
        sendToUav(message);
    }
    sendToGroundStation(message);
}

void UdpComm::sendCriticalToGroundStation(
    const String& message)
{
    for (uint8_t i = 0;
         i < CarConfig::CRITICAL_EVENT_REPEAT_COUNT;
         ++i) {
        sendToGroundStation(message);
    }
}

void UdpComm::sendCriticalToBoth(
    const String& message)
{
    for (uint8_t i = 0;
         i < CarConfig::CRITICAL_EVENT_REPEAT_COUNT;
         ++i) {
        sendToGroundStation(message);
        sendToUav(message);
    }
}

void UdpComm::notifyCarReady(
    const MissionManager& mission)
{
    sendCriticalToGroundStation(
        "EVT:CAR_READY:" +
        String(mission.profile().name));
}

void UdpComm::notifyCountdownStarted(
    const MissionManager& mission,
    uint32_t countdownMs)
{
    sendCriticalToGroundStation(
        "EVT:CAR_COUNTDOWN:" +
        mission.runId() + ":" +
        String(mission.profile().name) + ":" +
        String((countdownMs + 999) / 1000));
}

void UdpComm::notifyCountdownCancelled()
{
    sendCriticalToGroundStation(
        "EVT:CAR_COUNTDOWN_CANCELLED");
}

void UdpComm::notifyMissionStarted(
    const MissionManager& mission)
{
    const String task = mission.profile().name;
    const String runId = mission.runId();
    const String commandId =
        String(++carCommandSequence_);

    startCommandId_ = commandId;
    startRunId_ = runId;
    startPacket_ =
        "CMD:" + commandId +
        ":START:" + runId + ":" + task;
    startSendCount_ = 0;
    lastStartSendMs_ = 0;
    startPending_ = true;

    sendCriticalToGroundStation(
        "EVT:MISSION_START:" +
        runId + ":" + task);
    updateStartRetry();
}

void UdpComm::notifyPoint(
    const MissionManager& mission,
    const String& point)
{
    sendCriticalToBoth(
        "EVT:CAR_POINT:" +
        mission.runId() + ":" + point);
}

void UdpComm::notifyMissionDone(
    const MissionManager& mission)
{
    sendCriticalToGroundStation(
        "EVT:MISSION_DONE:" +
        mission.runId());
}

void UdpComm::notifyLocalStopped(
    const MissionManager& mission)
{
    sendCriticalToGroundStation(
        "EVT:CAR_LOCAL_STOP:" +
        mission.runId());
}

String UdpComm::tokenAt(
    const String& text,
    int index)
{
    int tokenStart = 0;
    int tokenIndex = 0;

    for (int i = 0;
         i <= text.length();
         ++i) {
        if (i == text.length() ||
            text.charAt(i) == ':') {
            if (tokenIndex == index) {
                return text.substring(
                    tokenStart, i);
            }
            tokenStart = i + 1;
            ++tokenIndex;
        }
    }
    return "";
}
