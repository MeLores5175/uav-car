#ifndef BOARD_PATH_MODEL_H
#define BOARD_PATH_MODEL_H

#include "car_types.h"

// 只处理“板中心沿轨迹”的几何参考和逆运动学。
// 路段切换、任务状态和速度约束仍由原有模块负责。
class BoardPathModel {
public:
    static BoardReference reference(RouteSegment segment,
                                    float segmentProgressCm,
                                    float boardSpeedCmS);

    static MotionCommand inverseKinematics(
        const Pose2D& rearPose,
        const Pose2D& actualBoardPose,
        const BoardReference& reference,
        float boardToRearCm,
        float positionKp);

private:
    static float clamp(float value, float minimum, float maximum);
};

#endif
