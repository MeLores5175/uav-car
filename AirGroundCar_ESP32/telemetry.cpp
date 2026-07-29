#include "telemetry.h"
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
    const Pose2D& pose = odometry.pose();
    const WheelState& wheels = drive.wheelState();
    const uint32_t elapsedMs = mission.isRunning()
        ? nowMs - mission.startTimeMs()
        : 0;

    char json[512];
    snprintf(json, sizeof(json),
             "TEL:CAR:{\"seq\":%lu,\"time_ms\":%lu,"
             "\"run\":\"%s\",\"task\":%u,"
             "\"state\":\"%s\",\"segment\":\"%s\","
             "\"x_cm\":%.2f,\"y_cm\":%.2f,"
             "\"yaw_deg\":%.2f,\"speed_cm_s\":%.2f,"
             "\"left_cm_s\":%.2f,\"right_cm_s\":%.2f,"
             "\"distance_cm\":%.2f,"
             "\"position_source\":\"command_odometry\","
             "\"backend\":\"%s\",\"driver_alarm\":%s,"
             "\"error\":0}",
             (unsigned long)++telemetrySequence_,
             (unsigned long)elapsedMs,
             mission.runId().c_str(),
             (unsigned)mission.missionId(),
             missionStateName(mission.state()),
             routeSegmentName(route.segment()),
             pose.xCm,
             pose.yCm,
             pose.yawRad * 180.0f / PI,
             motion.linearCmS,
             wheels.leftSpeedCmS,
             wheels.rightSpeedCmS,
             odometry.distanceCm(),
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
