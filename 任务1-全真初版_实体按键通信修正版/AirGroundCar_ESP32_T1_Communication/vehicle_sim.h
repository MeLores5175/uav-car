#ifndef VEHICLE_SIM_H
#define VEHICLE_SIM_H

#include "car_types.h"

class VehicleSim {
public:
    void begin();
    void reset();
    WheelState update(const WheelState& target, float dtSeconds);
    const WheelState& wheelState() const;

private:
    WheelState wheels_{0.0f, 0.0f};

    static float lag(float current,
                     float target,
                     float timeConstant,
                     float dtSeconds);
};

#endif
