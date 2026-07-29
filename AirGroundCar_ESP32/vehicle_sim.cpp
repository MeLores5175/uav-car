#include "vehicle_sim.h"

void VehicleSim::begin()
{
    reset();
}

void VehicleSim::reset()
{
    wheels_ = {0.0f, 0.0f};
}

float VehicleSim::lag(float current,
                      float target,
                      float timeConstant,
                      float dtSeconds)
{
    if (timeConstant <= 0.0f) {
        return target;
    }

    float alpha = dtSeconds / timeConstant;
    if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    if (alpha < 0.0f) {
        alpha = 0.0f;
    }
    return current + alpha * (target - current);
}

WheelState VehicleSim::update(const WheelState& target, float dtSeconds)
{
    wheels_.leftSpeedCmS =
        lag(wheels_.leftSpeedCmS, target.leftSpeedCmS, 0.15f, dtSeconds);
    wheels_.rightSpeedCmS =
        lag(wheels_.rightSpeedCmS, target.rightSpeedCmS, 0.15f, dtSeconds);
    return wheels_;
}

const WheelState& VehicleSim::wheelState() const
{
    return wheels_;
}
