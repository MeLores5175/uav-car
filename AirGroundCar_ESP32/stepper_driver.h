#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include "car_types.h"

struct StepperMotorStatus {
    float targetSpeedCmS = 0.0f;
    uint32_t pulseFrequencyHz = 0;
    bool forward = true;
    bool enabled = false;
    bool pulseGeneratorAttached = false;
    bool pulseOutputRunning = false;
};

struct StepperDriverStatus {
    bool begun = false;
    bool configValid = false;
    bool pulseGeneratorReady = false;
    StepperMotorStatus left;
    StepperMotorStatus right;
};

class StepperDriver {
public:
    bool begin();
    void reset();
    WheelState update(const WheelState& target, float dtSeconds);
    void stop();

    const WheelState& wheelState() const;
    const StepperDriverStatus& status() const;
    bool hasAlarm() const;

private:
    WheelState command_{0.0f, 0.0f};
    StepperDriverStatus status_;
};

#endif
