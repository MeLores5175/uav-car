#include "odometry.h"
#include <math.h>

void Odometry::begin(float xCm, float yCm, float yawRad)
{
    reset(xCm, yCm, yawRad);
}

void Odometry::reset(float xCm, float yCm, float yawRad)
{
    pose_ = {xCm, yCm, yawRad};
    distanceCm_ = 0.0f;
    leftDistanceCm_ = 0.0f;
    rightDistanceCm_ = 0.0f;
}

float Odometry::normalize(float angleRad)
{
    while (angleRad > PI) {
        angleRad -= 2.0f * PI;
    }
    while (angleRad < -PI) {
        angleRad += 2.0f * PI;
    }
    return angleRad;
}

void Odometry::update(float leftSpeedCmS,
                      float rightSpeedCmS,
                      float wheelTrackCm,
                      float dtSeconds)
{
    if (wheelTrackCm <= 0.0f || dtSeconds <= 0.0f) {
        return;
    }

    const float linear = (leftSpeedCmS + rightSpeedCmS) * 0.5f;
    const float angular =
        (rightSpeedCmS - leftSpeedCmS) / wheelTrackCm;
    const float deltaYaw = angular * dtSeconds;
    const float midYaw = pose_.yawRad + deltaYaw * 0.5f;

    pose_.xCm += linear * cosf(midYaw) * dtSeconds;
    pose_.yCm += linear * sinf(midYaw) * dtSeconds;
    pose_.yawRad = normalize(pose_.yawRad + deltaYaw);

    distanceCm_ += fabsf(linear) * dtSeconds;
    leftDistanceCm_ += fabsf(leftSpeedCmS) * dtSeconds;
    rightDistanceCm_ += fabsf(rightSpeedCmS) * dtSeconds;
}

const Pose2D& Odometry::pose() const
{
    return pose_;
}

Pose2D Odometry::boardPose(float boardToRearCm) const
{
    Pose2D board;
    board.xCm = pose_.xCm + boardToRearCm * cosf(pose_.yawRad);
    board.yCm = pose_.yCm + boardToRearCm * sinf(pose_.yawRad);
    board.yawRad = pose_.yawRad;
    return board;
}

float Odometry::distanceCm() const
{
    return distanceCm_;
}

float Odometry::leftDistanceCm() const
{
    return leftDistanceCm_;
}

float Odometry::rightDistanceCm() const
{
    return rightDistanceCm_;
}
