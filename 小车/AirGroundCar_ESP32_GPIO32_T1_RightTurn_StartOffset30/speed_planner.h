#ifndef SPEED_PLANNER_H
#define SPEED_PLANNER_H

#include "car_types.h"

class SpeedPlanner {
public:
    void begin();
    void reset();

    MotionCommand update(const MotionCommand& requested,
                         const MissionProfile& profile,
                         float dtSeconds);

    WheelState wheelTarget(float wheelTrackCm) const;
    const MotionCommand& command() const;

private:
    MotionCommand command_{0.0f, 0.0f};

    static float approach(float current,
                          float target,
                          float riseRate,
                          float fallRate,
                          float dtSeconds);
};

#endif
