#include "mission_config.h"
#include <math.h>

namespace {

float max5(float a, float b, float c, float d, float e)
{
    return fmaxf(a, fmaxf(b, fmaxf(c, fmaxf(d, e))));
}

const MissionProfile TASK1{
    MissionId::TASK_1,
    "T1",
    max5(CarConfig::T1_SPEED_APPROACH_A_CM_S,
         CarConfig::T1_SPEED_AB_CM_S,
         CarConfig::T1_SPEED_BC_CM_S,
         CarConfig::T1_SPEED_CD_CM_S,
         CarConfig::T1_SPEED_DA_CM_S),
    CarConfig::T1_MAX_ACCEL_CM_S2,
    CarConfig::T1_MAX_DECEL_CM_S2,
    CarConfig::T1_MAX_ANGULAR_ACCEL_RAD_S2
};

const MissionProfile TASK2{
    MissionId::TASK_2,
    "T2",
    max5(CarConfig::T2_SPEED_APPROACH_A_CM_S,
         CarConfig::T2_SPEED_AB_CM_S,
         CarConfig::T2_SPEED_BC_CM_S,
         CarConfig::T2_SPEED_CD_CM_S,
         CarConfig::T2_SPEED_DA_CM_S),
    CarConfig::T2_MAX_ACCEL_CM_S2,
    CarConfig::T2_MAX_DECEL_CM_S2,
    CarConfig::T2_MAX_ANGULAR_ACCEL_RAD_S2
};

const MissionProfile IDLE{
    MissionId::NONE,
    "NONE",
    0.0f,
    10.0f,
    10.0f,
    1.0f
};

float task1Speed(RouteSegment segment, float routeProgressCm)
{
    if (segment == RouteSegment::STRAIGHT_AB &&
        routeProgressCm < CarConfig::START_LINE_TO_BOARD_CENTER_CM) {
        return CarConfig::T1_SPEED_APPROACH_A_CM_S;
    }

    switch (segment) {
        case RouteSegment::STRAIGHT_AB:
            return CarConfig::T1_SPEED_AB_CM_S;
        case RouteSegment::ARC_BC:
            return CarConfig::T1_SPEED_BC_CM_S;
        case RouteSegment::STRAIGHT_CD:
            return CarConfig::T1_SPEED_CD_CM_S;
        case RouteSegment::ARC_DA:
            return CarConfig::T1_SPEED_DA_CM_S;
        default:
            return 0.0f;
    }
}

float task2Speed(RouteSegment segment, float routeProgressCm)
{
    if (segment == RouteSegment::STRAIGHT_AB &&
        routeProgressCm < CarConfig::START_LINE_TO_BOARD_CENTER_CM) {
        return CarConfig::T2_SPEED_APPROACH_A_CM_S;
    }

    switch (segment) {
        case RouteSegment::STRAIGHT_AB:
            return CarConfig::T2_SPEED_AB_CM_S;
        case RouteSegment::ARC_BC:
            return CarConfig::T2_SPEED_BC_CM_S;
        case RouteSegment::STRAIGHT_CD:
            return CarConfig::T2_SPEED_CD_CM_S;
        case RouteSegment::ARC_DA:
            return CarConfig::T2_SPEED_DA_CM_S;
        default:
            return 0.0f;
    }
}

}  // namespace

namespace MissionConfig {

const MissionProfile& getProfile(MissionId id)
{
    if (id == MissionId::TASK_1) {
        return TASK1;
    }
    if (id == MissionId::TASK_2) {
        return TASK2;
    }
    return IDLE;
}

float segmentCruiseSpeedCmS(MissionId id,
                            RouteSegment segment,
                            float routeProgressCm)
{
    if (id == MissionId::TASK_1) {
        return task1Speed(segment, routeProgressCm);
    }
    if (id == MissionId::TASK_2) {
        return task2Speed(segment, routeProgressCm);
    }
    return 0.0f;
}

}  // namespace MissionConfig
