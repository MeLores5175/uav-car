#include "speed_planner.h"
#include <math.h>

void SpeedPlanner::begin()
{
    reset();
}

void SpeedPlanner::reset()
{
    command_ = {0.0f, 0.0f};
}

float SpeedPlanner::approach(float current,
                             float target,
                             float riseRate,
                             float fallRate,
                             float dtSeconds)
{
    const float delta = target - current;
    if (delta > 0.0f) {
        return current + fminf(delta, riseRate * dtSeconds);
    }
    if (delta < 0.0f) {
        return current + fmaxf(delta, -fallRate * dtSeconds);
    }
    return current;
}

MotionCommand SpeedPlanner::update(const MotionCommand& requested,
                                   const MissionProfile& profile,
                                   float dtSeconds)
{
    command_.linearCmS = approach(command_.linearCmS,
                                  requested.linearCmS,
                                  profile.maxAccelCmS2,
                                  profile.maxDecelCmS2,
                                  dtSeconds);

    command_.angularRadS = approach(command_.angularRadS,
                                    requested.angularRadS,
                                    profile.maxAngularAccelRadS2,
                                    profile.maxAngularAccelRadS2,
                                    dtSeconds);
    return command_;
}

WheelState SpeedPlanner::wheelTarget(float wheelTrackCm) const
{
    WheelState target;
    target.leftSpeedCmS =
        command_.linearCmS - command_.angularRadS * wheelTrackCm * 0.5f;
    target.rightSpeedCmS =
        command_.linearCmS + command_.angularRadS * wheelTrackCm * 0.5f;
    return target;
}

const MotionCommand& SpeedPlanner::command() const
{
    return command_;
}
