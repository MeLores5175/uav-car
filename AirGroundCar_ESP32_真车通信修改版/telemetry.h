#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "mission_manager.h"
#include "route_controller.h"
#include "drive_backend.h"
#include "odometry.h"
#include "udp_comm.h"

class Telemetry {
public:
    void begin();

    void publish(uint32_t nowMs,
                 const MissionManager& mission,
                 const RouteController& route,
                 const MotionCommand& motion,
                 const DriveBackend& drive,
                 const Odometry& odometry,
                 UdpComm& udp);

    void publishHeartbeat(uint32_t sequence,
                          const MissionManager& mission,
                          UdpComm& udp);

private:
    uint32_t telemetrySequence_ = 0;

    static const char* missionStateName(MissionState state);
    static const char* routeSegmentName(RouteSegment segment);
    static const char* backendName(DriveBackendType type);
};

#endif
