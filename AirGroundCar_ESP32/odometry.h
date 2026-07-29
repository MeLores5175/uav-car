#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "car_types.h"

class Odometry {
public:
    void begin(float xCm, float yCm, float yawRad);
    void reset(float xCm, float yCm, float yawRad);

    void update(float leftSpeedCmS,
                float rightSpeedCmS,
                float wheelTrackCm,
                float dtSeconds);

    const Pose2D& pose() const;
    float distanceCm() const;

private:
    Pose2D pose_{0.0f, 0.0f, 0.0f};
    float distanceCm_ = 0.0f;

    static float normalize(float angleRad);
};

#endif
