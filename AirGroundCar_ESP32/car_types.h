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

struct MotionCommand {
    float linearCmS;
    float angularRadS;
};

struct WheelState {
    float leftSpeedCmS;
    float rightSpeedCmS;
};

struct Pose2D {
    float xCm;
    float yCm;
    float yawRad;
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
