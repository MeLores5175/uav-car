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
    const uint32_t elapsedMs = mission.isRunning()
        ? nowMs - mission.startTimeMs()
        : 0;
    const float referenceSpeed = sqrtf(
        reference.vxCmS * reference.vxCmS +
        reference.vyCmS * reference.vyCmS);

    // x_cm / y_cm 统一表示板中心；rear_x_cm / rear_y_cm 表示后轮轴中心。
    char json[1024];
    snprintf(json, sizeof(json),
             "TEL:CAR:{\"seq\":%lu,\"time_ms\":%lu,"
             "\"run\":\"%s\",\"task\":%u,"
             "\"state\":\"%s\",\"segment\":\"%s\","
             "\"x_cm\":%.2f,\"y_cm\":%.2f,"
             "\"rear_x_cm\":%.2f,\"rear_y_cm\":%.2f,"
             "\"ref_x_cm\":%.2f,\"ref_y_cm\":%.2f,"
             "\"board_error_cm\":%.3f,"
             "\"route_progress_cm\":%.2f,"
             "\"segment_progress\":%.5f,"
             "\"yaw_deg\":%.2f,"
             "\"speed_cm_s\":%.2f,\"angular_rad_s\":%.4f,"
             "\"ref_speed_cm_s\":%.2f,"
             "\"left_cm_s\":%.2f,\"right_cm_s\":%.2f,"
             "\"distance_cm\":%.2f,"
             "\"left_distance_cm\":%.2f,"
             "\"right_distance_cm\":%.2f,"
             "\"position_source\":\"wheel_odometry_board_center\","
             "\"backend\":\"%s\",\"driver_alarm\":%s,"
             "\"error\":0}",
             (unsigned long)++telemetrySequence_,
             (unsigned long)elapsedMs,
             mission.runId().c_str(),
             (unsigned)mission.missionId(),
             missionStateName(mission.state()),
             routeSegmentName(route.segment()),
             boardPose.xCm,
             boardPose.yCm,
             rearPose.xCm,
             rearPose.yCm,
             reference.xCm,
             reference.yCm,
             route.boardErrorCm(),
             route.routeProgressCm(),
             route.segmentProgress01(),
             rearPose.yawRad * 180.0f / PI,
             motion.linearCmS,
             motion.angularRadS,
             referenceSpeed,
             wheels.leftSpeedCmS,
             wheels.rightSpeedCmS,
             odometry.distanceCm(),
             odometry.leftDistanceCm(),
             odometry.rightDistanceCm(),
             backendName(drive.type()),
             drive.hasAlarm() ? "true" : "false");

    udp.sendToGroundStation(String(json));
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
    switch (segment) {
        case RouteSegment::WAITING: return "WAITING";
        case RouteSegment::STRAIGHT_AB: return "STRAIGHT_AB";
        case RouteSegment::ARC_BC: return "ARC_BC";
        case RouteSegment::STRAIGHT_CD: return "STRAIGHT_CD";
        case RouteSegment::ARC_DA: return "ARC_DA";
        case RouteSegment::COMPLETE: return "COMPLETE";
        default: return "UNKNOWN";
    }
}

const char* Telemetry::backendName(DriveBackendType type)
{
    return type == DriveBackendType::SIMULATION
        ? "simulation"
        : "stepper";
}
