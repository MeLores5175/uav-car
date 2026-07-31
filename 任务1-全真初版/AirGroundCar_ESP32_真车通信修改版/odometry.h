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

    // 后轮轴中心 O 的位姿。
    const Pose2D& pose() const;

    // 根据 O 的位姿推算板中心 P 的位姿。
    Pose2D boardPose(float boardToRearCm) const;

    // 后轮轴中心平均累计里程，以及左右轮各自累计里程。
    float distanceCm() const;
    float leftDistanceCm() const;
    float rightDistanceCm() const;

private:
    Pose2D pose_{0.0f, 0.0f, 0.0f};
    float distanceCm_ = 0.0f;
    float leftDistanceCm_ = 0.0f;
    float rightDistanceCm_ = 0.0f;

    static float normalize(float angleRad);
};

#endif
