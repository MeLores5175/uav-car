#include "route_controller.h"
#include "mission_config.h"

void RouteController::begin()
{
    reset();
}

void RouteController::reset()
{
    segment_ = RouteSegment::WAITING;
    startDistanceCm_ = 0.0f;
    pendingPointEvent_ = "";
}

void RouteController::start(float currentDistanceCm)
{
    startDistanceCm_ = currentDistanceCm;
    pendingPointEvent_ = "";
    segment_ = RouteSegment::STRAIGHT_AB;
}

MotionCommand RouteController::update(float currentDistanceCm,
                                      const MissionProfile& profile)
{
    const float progress = routeProgressCm(currentDistanceCm);
    const float arcLength = PI * CarConfig::TRACK_RADIUS_CM;
    const float endAB = CarConfig::STRAIGHT_LENGTH_CM;
    const float endBC = endAB + arcLength;
    const float endCD = endBC + CarConfig::STRAIGHT_LENGTH_CM;
    const float endDA = endCD + arcLength;

    if (segment_ == RouteSegment::STRAIGHT_AB && progress >= endAB) {
        setSegment(RouteSegment::ARC_BC, "B");
    }
    if (segment_ == RouteSegment::ARC_BC && progress >= endBC) {
        setSegment(RouteSegment::STRAIGHT_CD, "C");
    }
    if (segment_ == RouteSegment::STRAIGHT_CD && progress >= endCD) {
        setSegment(RouteSegment::ARC_DA, "D");
    }
    if (segment_ == RouteSegment::ARC_DA && progress >= endDA) {
        setSegment(RouteSegment::COMPLETE, "A_FINISH");
    }

    MotionCommand command{0.0f, 0.0f};
    if (segment_ == RouteSegment::STRAIGHT_AB ||
        segment_ == RouteSegment::STRAIGHT_CD) {
        command.linearCmS = profile.cruiseSpeedCmS;
        command.angularRadS = 0.0f;
    } else if (segment_ == RouteSegment::ARC_BC ||
               segment_ == RouteSegment::ARC_DA) {
        command.linearCmS = profile.cruiseSpeedCmS;
        command.angularRadS =
            -profile.cruiseSpeedCmS / CarConfig::TRACK_RADIUS_CM;
    }

    return command;
}

RouteSegment RouteController::segment() const
{
    return segment_;
}

bool RouteController::isComplete() const
{
    return segment_ == RouteSegment::COMPLETE;
}

float RouteController::routeProgressCm(float currentDistanceCm) const
{
    const float progress = currentDistanceCm - startDistanceCm_;
    return progress > 0.0f ? progress : 0.0f;
}

bool RouteController::consumePointEvent(String& pointName)
{
    if (pendingPointEvent_.length() == 0) {
        return false;
    }

    pointName = pendingPointEvent_;
    pendingPointEvent_ = "";
    return true;
}

void RouteController::setSegment(RouteSegment next, const char* pointEvent)
{
    segment_ = next;
    if (pointEvent != nullptr) {
        pendingPointEvent_ = pointEvent;
    }
}
