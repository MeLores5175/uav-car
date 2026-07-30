#include "board_path_model.h"
#include "mission_config.h"
#include <math.h>

float BoardPathModel::clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

BoardReference BoardPathModel::reference(RouteSegment segment,
                                         float segmentProgressCm,
                                         float boardSpeedCmS)
{
    BoardReference ref{
        CarConfig::START_BOARD_X_CM,
        CarConfig::START_BOARD_Y_CM,
        0.0f,
        0.0f
    };

    const float straight = CarConfig::STRAIGHT_LENGTH_CM;
    const float radius = CarConfig::BOARD_PATH_RADIUS_CM;
    const float arcLength = PI * radius;

    switch (segment) {
        case RouteSegment::STRAIGHT_AB: {
            const float s = clamp(segmentProgressCm, 0.0f, straight);
            ref.xCm = CarConfig::START_BOARD_X_CM;
            ref.yCm = CarConfig::START_BOARD_Y_CM + s;
            ref.vxCmS = 0.0f;
            ref.vyCmS = boardSpeedCmS;
            break;
        }

        case RouteSegment::ARC_BC: {
            const float s = clamp(segmentProgressCm, 0.0f, arcLength);
            const float phi = PI - s / radius;
            const float centerX =
                CarConfig::START_BOARD_X_CM + radius;
            const float centerY =
                CarConfig::START_BOARD_Y_CM + straight;

            ref.xCm = centerX + radius * cosf(phi);
            ref.yCm = centerY + radius * sinf(phi);
            ref.vxCmS = boardSpeedCmS * sinf(phi);
            ref.vyCmS = -boardSpeedCmS * cosf(phi);
            break;
        }

        case RouteSegment::STRAIGHT_CD: {
            const float s = clamp(segmentProgressCm, 0.0f, straight);
            ref.xCm =
                CarConfig::START_BOARD_X_CM + 2.0f * radius;
            ref.yCm =
                CarConfig::START_BOARD_Y_CM + straight - s;
            ref.vxCmS = 0.0f;
            ref.vyCmS = -boardSpeedCmS;
            break;
        }

        case RouteSegment::ARC_DA: {
            const float s = clamp(segmentProgressCm, 0.0f, arcLength);
            const float phi = -s / radius;
            const float centerX =
                CarConfig::START_BOARD_X_CM + radius;
            const float centerY = CarConfig::START_BOARD_Y_CM;

            ref.xCm = centerX + radius * cosf(phi);
            ref.yCm = centerY + radius * sinf(phi);
            ref.vxCmS = boardSpeedCmS * sinf(phi);
            ref.vyCmS = -boardSpeedCmS * cosf(phi);
            break;
        }

        case RouteSegment::COMPLETE:
        case RouteSegment::WAITING:
        default:
            // 等待或完成后，参考点固定在 A，参考速度为零。
            break;
    }

    return ref;
}

MotionCommand BoardPathModel::inverseKinematics(
    const Pose2D& rearPose,
    const Pose2D& actualBoardPose,
    const BoardReference& reference,
    float boardToRearCm,
    float positionKp)
{
    MotionCommand command{0.0f, 0.0f};
    if (boardToRearCm <= 0.0f) {
        return command;
    }

    // 参考速度 + 板中心位置误差反馈。
    const float ux = reference.vxCmS +
        positionKp * (reference.xCm - actualBoardPose.xCm);
    const float uy = reference.vyCmS +
        positionKp * (reference.yCm - actualBoardPose.yCm);

    const float c = cosf(rearPose.yawRad);
    const float s = sinf(rearPose.yawRad);

    // P 点速度：u = v*e_heading + a*omega*e_left。
    // 对该关系求逆，得到后轮轴中心线速度和车体角速度。
    command.linearCmS = c * ux + s * uy;
    command.angularRadS = (-s * ux + c * uy) / boardToRearCm;
    return command;
}
