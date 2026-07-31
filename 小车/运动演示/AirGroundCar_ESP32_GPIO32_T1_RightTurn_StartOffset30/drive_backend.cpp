#include "drive_backend.h"

bool DriveBackend::begin(DriveBackendType type)
{
    type_ = type;
    simulator_.begin();

    if (type_ == DriveBackendType::SIMULATION) {
        simulator_.reset();
        wheels_ = {0.0f, 0.0f};
        return true;
    }

    const bool stepperOk = stepper_.begin();
    wheels_ = {0.0f, 0.0f};
    return stepperOk;
}

void DriveBackend::reset()
{
    if (type_ == DriveBackendType::SIMULATION) {
        simulator_.reset();
    } else {
        stepper_.reset();
    }
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
    if (type_ == DriveBackendType::SIMULATION) {
        simulator_.reset();
    } else {
        stepper_.stop();
    }
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

bool DriveBackend::isReady() const
{
    if (type_ == DriveBackendType::SIMULATION) {
        return true;
    }

    const StepperDriverStatus& status = stepper_.status();
    return status.begun &&
           status.configValid &&
           status.pulseGeneratorReady;
}

const StepperDriverStatus& DriveBackend::stepperStatus() const
{
    return stepper_.status();
}

bool DriveBackend::hasAlarm() const
{
    return type_ == DriveBackendType::STEPPER && stepper_.hasAlarm();
}
