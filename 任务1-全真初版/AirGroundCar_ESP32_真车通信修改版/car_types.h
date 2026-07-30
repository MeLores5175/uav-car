#ifndef CAR_TYPES_H
#define CAR_TYPES_H

#include <Arduino.h>

enum class MissionId : uint8_t {
    NONE = 0,
    TASK_1 = 1,
    TASK_2 = 2
};

enum class MissionState : uint8_t {
    IDLE = 0,
    READY,
    ARMED,
    RUNNING,
    FINISHED,
    FAULT
};

enum class RouteSegment : uint8_t {
    WAITING = 0,
    STRAIGHT_AB,
    ARC_BC,
    STRAIGHT_CD,
    ARC_DA,
    COMPLETE
};

enum class DriveBackendType : uint8_t {
    SIMULATION = 0,
    STEPPER
};

// 后轮轴中心 O 的车体速度命令。
struct MotionCommand {
    float linearCmS;
    float angularRadS;
};

// 左右后轮各自的线速度。
struct WheelState {
    float leftSpeedCmS;
    float rightSpeedCmS;
};

// 二维位姿。odometry 内部保存的是后轮轴中心 O 的位姿。
struct Pose2D {
    float xCm;
    float yCm;
    float yawRad;
};

// 板中心 P 的参考位置和参考速度。
struct BoardReference {
    float xCm;
    float yCm;
    float vxCmS;
    float vyCmS;
};

// 建模阶段串口输出的一条弧线参数样本。
struct ArcSample {
    float progress01;
    float leftSpeedCmS;
    float rightSpeedCmS;
};

struct MissionProfile {
    MissionId id;
    const char* name;
    float cruiseSpeedCmS;
    float maxAccelCmS2;
    float maxDecelCmS2;
    float maxAngularAccelRadS2;
};

#endif
