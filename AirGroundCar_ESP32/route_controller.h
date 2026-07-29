#ifndef ROUTE_CONTROLLER_H
#define ROUTE_CONTROLLER_H

#include "car_types.h"

class RouteController {
public:
    void begin();
    void reset();
    void start(float currentDistanceCm);

    MotionCommand update(float currentDistanceCm,
                         const MissionProfile& profile);

    RouteSegment segment() const;
    bool isComplete() const;
    float routeProgressCm(float currentDistanceCm) const;

    // 路段切换时返回 B/C/D/A_FINISH，一次读取后清空。
    bool consumePointEvent(String& pointName);

private:
    RouteSegment segment_ = RouteSegment::WAITING;
    float startDistanceCm_ = 0.0f;
    String pendingPointEvent_;

    void setSegment(RouteSegment next, const char* pointEvent = nullptr);
};

#endif
