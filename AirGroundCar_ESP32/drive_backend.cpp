#include "drive_backend.h"

bool DriveBackend::begin(DriveBackendType type)
{
    type_ = type;
    simulator_.begin();
    const bool stepperOk = stepper_.begin();
    reset();
    return type_ == DriveBackendType::SIMULATION || stepperOk;
}

void DriveBackend::reset()
{
    simulator_.reset();
    stepper_.reset();
    wheels_ = {0.0f, 0.0f};
}

WheelState DriveBackend::update(const WheelState& target, float dtSeconds)
{
    if (type_ == DriveBackendType::SIMULATION) {
        wheels_ = simulator_.update(target, dtSeconds);
    } else {
        wheels_ = stepper_.update(target, dtSeconds);
    }
    return wheels_;
}

void DriveBackend::stop()
{
    simulator_.reset();
    stepper_.stop();
    wheels_ = {0.0f, 0.0f};
}

DriveBackendType DriveBackend::type() const
{
    return type_;
}

const WheelState& DriveBackend::wheelState() const
{
    return wheels_;
}

bool DriveBackend::hasAlarm() const
{
    return type_ == DriveBackendType::STEPPER && stepper_.hasAlarm();
}
