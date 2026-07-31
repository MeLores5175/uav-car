#include "telemetry.h"
#include "mission_config.h"
#include <math.h>

void Telemetry::begin()
{
    telemetrySequence_ = 0;
}

void Telemetry::publish(uint32_t nowMs,
                        const MissionManager& mission,
                        const RouteController& route,
                        const MotionCommand& motion,
                        const DriveBackend& drive,
                        const Odometry& odometry,
                        UdpComm& udp)
{
    const Pose2D& rearPose = odometry.pose();
    const Pose2D boardPose =
        odometry.boardPose(CarConfig::BOARD_TO_REAR_CM);
    const BoardReference& reference = route.reference();
    const WheelState& wheels = drive.wheelState();
    const StepperDriverStatus& stepper = drive.stepperStatus();

    const uint32_t elapsedMs = mission.isRunning()
        ? nowMs - mission.startTimeMs()
        : 0;
    const uint32_t countdownMs =
        mission.countdownRemainingMs(nowMs);

    const float referenceSpeed = sqrtf(
        reference.vxCmS * reference.vxCmS +
        reference.vyCmS * reference.vyCmS);

    // 根据后轮轴中心 v/omega 计算平台中心 P 在场地坐标中的速度。
    const float c = cosf(rearPose.yawRad);
    const float s = sinf(rearPose.yawRad);
    const float boardVx =
        motion.linearCmS * c -
        CarConfig::BOARD_TO_REAR_CM * motion.angularRadS * s;
    const float boardVy =
        motion.linearCmS * s +
        CarConfig::BOARD_TO_REAR_CM * motion.angularRadS * c;
    const float boardSpeed =
        sqrtf(boardVx * boardVx + boardVy * boardVy);

    // 当前没有独立巡线传感器，此字段仅表示运动驱动链路正常。
    const bool lineDetected = drive.isReady() && !drive.hasAlarm();

    // path_s_cm 必须从 A 轨迹点开始，不包含起点后方 30 cm 的进场段，
    // 这样与 UAV gateway 的 150 cm + 半圆 + 150 cm + 半圆模型一致。
    char json[1500];
    snprintf(json, sizeof(json),
             "TEL:CAR:{\"proto\":\"1.2\","
             "\"seq\":%lu,\"time_ms\":%lu,\"countdown_ms\":%lu,"
             "\"run\":\"%s\",\"task\":%u,"
             "\"state\":\"%s\",\"segment\":\"%s\","
             "\"x_cm\":%.2f,\"y_cm\":%.2f,"
             "\"speed_cm_s\":%.2f,\"vx_cm_s\":%.2f,\"vy_cm_s\":%.2f,"
             "\"yaw_deg\":%.2f,"
             "\"path_s_cm\":%.2f,\"route_s_cm\":%.2f,"
             "\"line_detected\":%s,"
             "\"battery\":-1,\"error\":%u,"
             "\"rear_x_cm\":%.2f,\"rear_y_cm\":%.2f,"
             "\"ref_x_cm\":%.2f,\"ref_y_cm\":%.2f,"
             "\"board_error_cm\":%.3f,"
             "\"segment_progress\":%.5f,"
             "\"angular_rad_s\":%.4f,"
             "\"ref_speed_cm_s\":%.2f,"
             "\"left_cm_s\":%.2f,\"right_cm_s\":%.2f,"
             "\"distance_cm\":%.2f,"
             "\"position_source\":\"open_loop_step_command_board_center\","
             "\"backend\":\"%s\",\"driver_ready\":%s,"
             "\"stepper_config_valid\":%s,"
             "\"stepper_pulse_generator_ready\":%s,"
             "\"driver_alarm\":%s}",
             (unsigned long)++telemetrySequence_,
             (unsigned long)elapsedMs,
             (unsigned long)countdownMs,
             mission.runId().c_str(),
             (unsigned)mission.missionId(),
             missionStateName(mission.state()),
             routeSegmentName(route.segment()),
             boardPose.xCm,
             boardPose.yCm,
             boardSpeed,
             boardVx,
             boardVy,
             rearPose.yawRad * 180.0f / PI,
             route.trackProgressCm(),
             route.routeProgressCm(),
             lineDetected ? "true" : "false",
             drive.hasAlarm() ? 1U : 0U,
             rearPose.xCm,
             rearPose.yCm,
             reference.xCm,
             reference.yCm,
             route.boardErrorCm(),
             route.segmentProgress01(),
             motion.angularRadS,
             referenceSpeed,
             wheels.leftSpeedCmS,
             wheels.rightSpeedCmS,
             odometry.distanceCm(),
             backendName(drive.type()),
             drive.isReady() ? "true" : "false",
             stepper.configValid ? "true" : "false",
             stepper.pulseGeneratorReady ? "true" : "false",
             drive.hasAlarm() ? "true" : "false");

    udp.sendCarTelemetry(
        String(json),
        mission.missionId() != MissionId::NONE);
}

void Telemetry::publishHeartbeat(uint32_t sequence,
                                 const MissionManager& mission,
                                 UdpComm& udp)
{
    const String message =
        "HB:CAR:" + String(sequence) + ":" +
        missionStateName(mission.state());
    udp.sendToGroundStation(message);
}

const char* Telemetry::missionStateName(MissionState state)
{
    switch (state) {
        case MissionState::IDLE: return "IDLE";
        case MissionState::READY: return "READY";
        case MissionState::ARMED: return "ARMED";
        case MissionState::COUNTDOWN: return "COUNTDOWN";
        case MissionState::RUNNING: return "RUNNING";
        case MissionState::FINISHED: return "FINISHED";
        case MissionState::FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

const char* Telemetry::routeSegmentName(RouteSegment segment)
{
    switch (segment) {
        case RouteSegment::STRAIGHT_AB: return "AB";
        case RouteSegment::ARC_BC: return "BC";
        case RouteSegment::STRAIGHT_CD: return "CD";
        case RouteSegment::ARC_DA: return "DA";
        case RouteSegment::WAITING:
        case RouteSegment::COMPLETE:
        default:
            return "UNKNOWN";
    }
}

const char* Telemetry::backendName(DriveBackendType type)
{
    return type == DriveBackendType::SIMULATION
        ? "simulation"
        : "stepper";
}
