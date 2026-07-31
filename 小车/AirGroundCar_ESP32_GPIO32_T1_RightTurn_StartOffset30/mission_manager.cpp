#include "mission_manager.h"
#include "mission_config.h"

void MissionManager::begin()
{
    reset();
}

void MissionManager::reset()
{
    missionId_ = MissionId::NONE;
    profile_ = &MissionConfig::getProfile(MissionId::NONE);
    state_ = MissionState::IDLE;
    runId_ = "-";
    startTimeMs_ = 0;
}

bool MissionManager::selectMission(MissionId id)
{
    if (state_ == MissionState::RUNNING || state_ == MissionState::ARMED) {
        return false;
    }
    if (id != MissionId::TASK_1 && id != MissionId::TASK_2) {
        return false;
    }

    missionId_ = id;
    profile_ = &MissionConfig::getProfile(id);
    state_ = MissionState::READY;
    runId_ = "-";
    return true;
}

bool MissionManager::arm(const String& runId)
{
    if (state_ != MissionState::READY || runId.length() == 0) {
        return false;
    }

    runId_ = runId;
    state_ = MissionState::ARMED;
    return true;
}

bool MissionManager::localStart()
{
    if (state_ != MissionState::ARMED) {
        return false;
    }

    startTimeMs_ = millis();
    state_ = MissionState::RUNNING;
    return true;
}

bool MissionManager::remoteStart()
{
    return localStart();
}

void MissionManager::finish()
{
    if (state_ == MissionState::RUNNING) {
        state_ = MissionState::FINISHED;
    }
}

void MissionManager::fault()
{
    state_ = MissionState::FAULT;
}

bool MissionManager::isRunning() const
{
    return state_ == MissionState::RUNNING;
}

MissionId MissionManager::missionId() const
{
    return missionId_;
}

MissionState MissionManager::state() const
{
    return state_;
}

const MissionProfile& MissionManager::profile() const
{
    return *profile_;
}

const String& MissionManager::runId() const
{
    return runId_;
}

uint32_t MissionManager::startTimeMs() const
{
    return startTimeMs_;
}
