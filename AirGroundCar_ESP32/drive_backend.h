#ifndef DRIVE_BACKEND_H
#define DRIVE_BACKEND_H

#include "car_types.h"
#include "vehicle_sim.h"
#include "stepper_driver.h"

class DriveBackend {
public:
    bool begin(DriveBackendType type);
    void reset();
    WheelState update(const WheelState& target, float dtSeconds);
    void stop();

    DriveBackendType type() const;
    const WheelState& wheelState() const;
    bool hasAlarm() const;

private:
    DriveBackendType type_ = DriveBackendType::SIMULATION;
    VehicleSim simulator_;
    StepperDriver stepper_;
    WheelState wheels_{0.0f, 0.0f};
};

#endif
