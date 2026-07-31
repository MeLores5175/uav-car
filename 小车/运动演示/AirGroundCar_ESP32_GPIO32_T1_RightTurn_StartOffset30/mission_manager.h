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
    void finish();
    void fault();

    bool isRunning() const;
    MissionId missionId() const;
    MissionState state() const;
    const MissionProfile& profile() const;
    const String& runId() const;
    uint32_t startTimeMs() const;

private:
    MissionId missionId_ = MissionId::NONE;
    MissionState state_ = MissionState::IDLE;
    const MissionProfile* profile_ = nullptr;
    String runId_ = "-";
    uint32_t startTimeMs_ = 0;
};

#endif
