#ifndef ROUTE_CONTROLLER_H
#define ROUTE_CONTROLLER_H

#include "car_types.h"

class RouteController {
public:
    void begin();
    void reset();
    void start();

    MotionCommand update(const Pose2D& rearPose,
                         const Pose2D& boardPose,
                         const MissionProfile& profile,
                         float dtSeconds);

    RouteSegment segment() const;
    bool isComplete() const;

    float routeProgressCm() const;
    float segmentProgress01() const;
    float boardErrorCm() const;
    const BoardReference& reference() const;

    // 路段切换时返回 B/C/D/A_FINISH，一次读取后清空。
    bool consumePointEvent(String& pointName);

private:
    RouteSegment segment_ = RouteSegment::WAITING;
    float routeProgressCm_ = 0.0f;
    float segmentProgress01_ = 0.0f;
    float boardErrorCm_ = 0.0f;
    float referenceSpeedCmS_ = 0.0f;
    BoardReference currentReference_{0.0f, 0.0f, 0.0f, 0.0f};
    String pendingPointEvent_;

    void setSegment(RouteSegment next, const char* pointEvent = nullptr);
    static float approach(float current, float target,
                          float accel, float decel,
                          float dtSeconds);
    float localSegmentProgressCm(float endAB,
                                 float endBC,
                                 float endCD) const;
};

#endif
