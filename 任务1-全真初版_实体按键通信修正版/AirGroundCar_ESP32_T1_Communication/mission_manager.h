#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

#include "car_types.h"

class MissionManager {
public:
    void begin();
    void reset();

    bool selectMission(MissionId id);
    bool arm(const String& runId);
    bool localStart();
    bool remoteStart();

    // 实体按键流程：READY -> COUNTDOWN -> RUNNING。
    bool beginCountdown(const String& runId,
                        uint32_t nowMs,
                        uint32_t durationMs);
    bool updateCountdown(uint32_t nowMs);
    bool cancelCountdown();

    void finish();
    void fault();

    bool isRunning() const;
    bool isCountdown() const;
    MissionId missionId() const;
    MissionState state() const;
    const MissionProfile& profile() const;
    const String& runId() const;
    uint32_t startTimeMs() const;
    uint32_t countdownRemainingMs(uint32_t nowMs) const;

private:
    MissionId missionId_ = MissionId::NONE;
    MissionState state_ = MissionState::IDLE;
    const MissionProfile* profile_ = nullptr;
    String runId_ = "-";
    uint32_t startTimeMs_ = 0;
    uint32_t countdownStartMs_ = 0;
    uint32_t countdownDurationMs_ = 0;
};

#endif
