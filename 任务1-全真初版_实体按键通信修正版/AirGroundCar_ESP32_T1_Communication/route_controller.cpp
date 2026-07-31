#include "route_controller.h"
#include "board_path_model.h"
#include "mission_config.h"
#include <math.h>

void RouteController::begin()
{
    reset();
}

void RouteController::reset()
{
    segment_ = RouteSegment::WAITING;
    routeProgressCm_ = 0.0f;
    segmentProgress01_ = 0.0f;
    boardErrorCm_ = 0.0f;
    referenceSpeedCmS_ = 0.0f;
    currentReference_ = {
        CarConfig::START_BOARD_X_CM,
        CarConfig::START_BOARD_Y_CM,
        0.0f,
        0.0f
    };
    pendingPointEvent_ = "";
}

void RouteController::start()
{
    routeProgressCm_ = 0.0f;
    segmentProgress01_ = 0.0f;
    boardErrorCm_ = 0.0f;
    referenceSpeedCmS_ = 0.0f;
    pendingPointEvent_ = "";
    segment_ = RouteSegment::STRAIGHT_AB;
}

MotionCommand RouteController::update(const Pose2D& rearPose,
                                      const Pose2D& boardPose,
                                      const MissionProfile& profile,
                                      float dtSeconds)
{
    MotionCommand zero{0.0f, 0.0f};
    if (segment_ == RouteSegment::WAITING || dtSeconds <= 0.0f) {
        return zero;
    }

    const float radius = CarConfig::BOARD_PATH_RADIUS_CM;
    const float arcLength = PI * radius;

    // 初始中心在出发线后方30 cm：
    // 第一次直线为180 cm，对侧直线仍为150 cm。
    const float firstStraight =
        CarConfig::FIRST_STRAIGHT_LENGTH_CM;
    const float trackStraight =
        CarConfig::TRACK_STRAIGHT_LENGTH_CM;

    const float endAB = firstStraight;
    const float endBC = endAB + arcLength;
    const float endCD = endBC + trackStraight;
    const float endDA = endCD + arcLength;

    // 建模阶段按“板中心参考路径长度”推进，不再使用后轮轴累计里程。
    // 参考轨迹自身也采用加减速，避免一启动就以巡航速度向前跑，
    // 导致参考点与仿真小车之间产生较大的初始距离误差。
    if (segment_ != RouteSegment::COMPLETE) {
        const float remaining = fmaxf(endDA - routeProgressCm_, 0.0f);
        const float decel = fmaxf(profile.maxDecelCmS2, 0.001f);
        const float stoppingLimitedSpeed = sqrtf(2.0f * decel * remaining);
        const float configuredSegmentSpeed =
            MissionConfig::segmentCruiseSpeedCmS(
                profile.id, segment_, routeProgressCm_);
        const float targetReferenceSpeed =
            fminf(configuredSegmentSpeed, stoppingLimitedSpeed);

        referenceSpeedCmS_ = approach(referenceSpeedCmS_,
                                      targetReferenceSpeed,
                                      profile.maxAccelCmS2,
                                      profile.maxDecelCmS2,
                                      dtSeconds);

        routeProgressCm_ += referenceSpeedCmS_ * dtSeconds;
        if (routeProgressCm_ > endDA ||
            (remaining < 0.02f && referenceSpeedCmS_ < 0.1f)) {
            routeProgressCm_ = endDA;
            referenceSpeedCmS_ = 0.0f;
        }
    }

    if (segment_ == RouteSegment::STRAIGHT_AB &&
        routeProgressCm_ >= endAB) {
        setSegment(RouteSegment::ARC_BC, "B");
    }
    if (segment_ == RouteSegment::ARC_BC &&
        routeProgressCm_ >= endBC) {
        setSegment(RouteSegment::STRAIGHT_CD, "C");
    }
    if (segment_ == RouteSegment::STRAIGHT_CD &&
        routeProgressCm_ >= endCD) {
        setSegment(RouteSegment::ARC_DA, "D");
    }
    if (segment_ == RouteSegment::ARC_DA &&
        routeProgressCm_ >= endDA) {
        setSegment(RouteSegment::COMPLETE, "A_FINISH");
    }

    const float localProgress =
        localSegmentProgressCm(endAB, endBC, endCD);

    float segmentLength = 1.0f;
    if (segment_ == RouteSegment::STRAIGHT_AB) {
        segmentLength = firstStraight;
    } else if (segment_ == RouteSegment::STRAIGHT_CD) {
        segmentLength = trackStraight;
    } else if (segment_ == RouteSegment::ARC_BC ||
               segment_ == RouteSegment::ARC_DA) {
        segmentLength = arcLength;
    }

    segmentProgress01_ = segment_ == RouteSegment::COMPLETE
        ? 1.0f
        : constrain(localProgress / segmentLength, 0.0f, 1.0f);

    currentReference_ = BoardPathModel::reference(
        segment_,
        localProgress,
        referenceSpeedCmS_);

    const float ex = currentReference_.xCm - boardPose.xCm;
    const float ey = currentReference_.yCm - boardPose.yCm;
    boardErrorCm_ = sqrtf(ex * ex + ey * ey);

    if (segment_ == RouteSegment::COMPLETE &&
        boardErrorCm_ <=
            CarConfig::ROUTE_COMPLETE_POSITION_TOLERANCE_CM) {
        return zero;
    }

    return BoardPathModel::inverseKinematics(
        rearPose,
        boardPose,
        currentReference_,
        CarConfig::BOARD_TO_REAR_CM,
        CarConfig::BOARD_POSITION_KP);
}

RouteSegment RouteController::segment() const
{
    return segment_;
}

bool RouteController::isComplete() const
{
    return segment_ == RouteSegment::COMPLETE;
}

float RouteController::routeProgressCm() const
{
    return routeProgressCm_;
}

float RouteController::trackProgressCm() const
{
    const float trackTotal =
        2.0f * CarConfig::TRACK_STRAIGHT_LENGTH_CM +
        2.0f * PI * CarConfig::BOARD_PATH_RADIUS_CM;
    const float fromA =
        routeProgressCm_ - CarConfig::START_LINE_TO_BOARD_CENTER_CM;
    return constrain(fromA, 0.0f, trackTotal);
}

float RouteController::segmentProgress01() const
{
    return segmentProgress01_;
}

float RouteController::boardErrorCm() const
{
    return boardErrorCm_;
}

const BoardReference& RouteController::reference() const
{
    return currentReference_;
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

float RouteController::approach(float current,
                                float target,
                                float accel,
                                float decel,
                                float dtSeconds)
{
    const float delta = target - current;
    if (delta > 0.0f) {
        return current + fminf(delta, accel * dtSeconds);
    }
    if (delta < 0.0f) {
        return current + fmaxf(delta, -decel * dtSeconds);
    }
    return current;
}

float RouteController::localSegmentProgressCm(float endAB,
                                              float endBC,
                                              float endCD) const
{
    switch (segment_) {
        case RouteSegment::STRAIGHT_AB:
            return routeProgressCm_;
        case RouteSegment::ARC_BC:
            return routeProgressCm_ - endAB;
        case RouteSegment::STRAIGHT_CD:
            return routeProgressCm_ - endBC;
        case RouteSegment::ARC_DA:
            return routeProgressCm_ - endCD;
        case RouteSegment::COMPLETE:
        case RouteSegment::WAITING:
        default:
            return 0.0f;
    }
}
