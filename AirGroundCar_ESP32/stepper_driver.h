#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include "car_types.h"

class StepperDriver {
public:
    bool begin();
    void reset();
    WheelState update(const WheelState& target, float dtSeconds);
    void stop();

    const WheelState& wheelState() const;
    bool hasAlarm() const;

private:
    WheelState command_{0.0f, 0.0f};
};

#endif
