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
    const float referenceSpeed = sqrtf(
        reference.vxCmS * reference.vxCmS +
        reference.vyCmS * reference.vyCmS);

    // 当前版本没有独立巡线传感器接口。
    // 这里表示运动控制链路可用；接入真实巡线模块后应替换为真实检测状态。
    const bool lineDetected = drive.isReady() && !drive.hasAlarm();

    // x_cm/y_cm 是 FIELD 场地坐标中的板中心；path_s_cm 从 A 点开始累计。
    char json[1200];
    snprintf(json, sizeof(json),
             "TEL:CAR:{\"seq\":%lu,\"time_ms\":%lu,"
             "\"run\":\"%s\",\"task\":%u,"
             "\"state\":\"%s\",\"segment\":\"%s\","
             "\"x_cm\":%.2f,\"y_cm\":%.2f,"
             "\"speed_cm_s\":%.2f,\"yaw_deg\":%.2f,"
             "\"path_s_cm\":%.2f,\"line_detected\":%s,"
             "\"battery\":-1,\"error\":%u,"
             "\"rear_x_cm\":%.2f,\"rear_y_cm\":%.2f,"
             "\"ref_x_cm\":%.2f,\"ref_y_cm\":%.2f,"
             "\"board_error_cm\":%.3f,"
             "\"segment_progress\":%.5f,"
             "\"angular_rad_s\":%.4f,"
             "\"ref_speed_cm_s\":%.2f,"
             "\"left_cm_s\":%.2f,\"right_cm_s\":%.2f,"
             "\"distance_cm\":%.2f,"
             "\"position_source\":\"wheel_odometry_board_center\","
             "\"backend\":\"%s\",\"driver_ready\":%s,"
             "\"stepper_config_valid\":%s,"
             "\"stepper_pulse_generator_ready\":%s,"
             "\"driver_alarm\":%s}",
             (unsigned long)++telemetrySequence_,
             (unsigned long)elapsedMs,
             mission.runId().c_str(),
             (unsigned)mission.missionId(),
             missionStateName(mission.state()),
             routeSegmentName(route.segment()),
             boardPose.xCm,
             boardPose.yCm,
             motion.linearCmS,
             rearPose.yawRad * 180.0f / PI,
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

    udp.sendCarTelemetry(String(json));
}

void Telemetry::publishHeartbeat(uint32_t sequence,
                                 const MissionManager& mission,
                                 UdpComm& udp)
{
    String message = "HB:CAR:" + String(sequence) + ":" +
                     missionStateName(mission.state());
    udp.sendToGroundStation(message);
}

const char* Telemetry::missionStateName(MissionState state)
{
    switch (state) {
        case MissionState::IDLE: return "IDLE";
        case MissionState::READY: return "READY";
        case MissionState::ARMED: return "ARMED";
        case MissionState::RUNNING: return "RUNNING";
        case MissionState::FINISHED: return "FINISHED";
        case MissionState::FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

const char* Telemetry::routeSegmentName(RouteSegment segment)
{
    // 必须严格使用无人机协议接受的 AB/BC/CD/DA/UNKNOWN。
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
