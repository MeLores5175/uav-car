#include "car_types.h"
#include "mission_config.h"
#include "mission_manager.h"
#include "route_controller.h"
#include "speed_planner.h"
#include "drive_backend.h"
#include "odometry.h"
#include "udp_comm.h"
#include "telemetry.h"

MissionManager missionManager;
RouteController routeController;
SpeedPlanner speedPlanner;
DriveBackend driveBackend;
Odometry odometry;
UdpComm udpComm;
Telemetry telemetry;

uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastArcRecordMs = 0;
uint32_t lastMotionDebugMs = 0;
uint32_t heartbeatSequence = 0;
uint32_t localRunSequence = 1;
int32_t lastCountdownSecond = -1;

MissionState previousMissionState =
    MissionState::IDLE;
bool driveBackendReady = false;

static const char* missionStateName(
    MissionState state)
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

static void resetOdometryToStart()
{
    const float yaw = CarConfig::START_YAW_RAD;
    const float rearX =
        CarConfig::START_BOARD_X_CM -
        CarConfig::BOARD_TO_REAR_CM * cosf(yaw);
    const float rearY =
        CarConfig::START_BOARD_Y_CM -
        CarConfig::BOARD_TO_REAR_CM * sinf(yaw);

    odometry.reset(rearX, rearY, yaw);
}

static void resetMotionModules()
{
    routeController.reset();
    speedPlanner.reset();
    driveBackend.reset();
    resetOdometryToStart();
    lastArcRecordMs = 0;
}

static String nextLocalRunId()
{
    char buffer[16];
    snprintf(buffer, sizeof(buffer),
             "R%03lu",
             static_cast<unsigned long>(
                 localRunSequence++ % 1000));
    return String(buffer);
}

static void handleMissionStateChange()
{
    const MissionState current =
        missionManager.state();
    if (current == previousMissionState) {
        return;
    }

    Serial.printf("[MISSION] %s -> %s\n",
                  missionStateName(previousMissionState),
                  missionStateName(current));

    if (current == MissionState::READY) {
        resetMotionModules();
        Serial.printf(
            "[MISSION] mode selected: %s; "
            "waiting GPIO%u\n",
            missionManager.profile().name,
            CarConfig::LOCAL_START_PIN);
    } else if (current == MissionState::COUNTDOWN) {
        resetMotionModules();
        lastCountdownSecond = -1;
    } else if (current == MissionState::RUNNING) {
        resetMotionModules();
        routeController.start();

        if (CarConfig::ENABLE_WIFI_UDP) {
            udpComm.notifyMissionStarted(
                missionManager);
        }

        Serial.printf(
            "[MISSION] actual movement started: "
            "run=%s task=%s\n",
            missionManager.runId().c_str(),
            missionManager.profile().name);
    } else if (current == MissionState::IDLE) {
        resetMotionModules();
        Serial.println("[MISSION] reset to idle");
    } else if (current == MissionState::FINISHED) {
        driveBackend.reset();

        if (CarConfig::ENABLE_WIFI_UDP) {
            udpComm.notifyMissionDone(
                missionManager);
        }

        Serial.println(
            "[MISSION] finished; motors disabled; "
            "send RESET/MODE before next run");
    } else if (current == MissionState::FAULT) {
        driveBackend.reset();
        Serial.println(
            "[MISSION][ERROR] fault; motors disabled");
    }

    previousMissionState = current;
}

static void cancelOrStopByButton()
{
    if (!CarConfig::ALLOW_BUTTON_CANCEL_OR_STOP) {
        Serial.println(
            "[BUTTON] cancel/stop disabled in config");
        return;
    }

    if (missionManager.isCountdown()) {
        if (missionManager.cancelCountdown()) {
            resetMotionModules();
            udpComm.notifyCountdownCancelled();
            Serial.println(
                "[BUTTON] countdown cancelled; "
                "mode remains READY");
        }
        return;
    }

    if (missionManager.isRunning()) {
        const String runId =
            missionManager.runId();
        udpComm.notifyLocalStopped(
            missionManager);
        missionManager.reset();
        resetMotionModules();
        Serial.printf(
            "[BUTTON] local test stop: %s\n",
            runId.c_str());
    }
}

static void handleButtonPressed()
{
    if (missionManager.isCountdown() ||
        missionManager.isRunning()) {
        cancelOrStopByButton();
        return;
    }

    if (missionManager.state() !=
        MissionState::READY) {
        Serial.println(
            "[BUTTON] ignored: select MODE:T1/T2 "
            "from ground station first");
        return;
    }

    if (!driveBackendReady ||
        !driveBackend.isReady()) {
        missionManager.fault();
        Serial.println(
            "[BUTTON][ERROR] drive backend not ready");
        return;
    }

    if (CarConfig::REQUIRE_NETWORK_BEFORE_BUTTON &&
        !udpComm.online()) {
        Serial.println(
            "[BUTTON][ERROR] Wi-Fi/UDP is offline");
        return;
    }

    const String runId = nextLocalRunId();
    const uint32_t nowMs = millis();

    if (!missionManager.beginCountdown(
            runId,
            nowMs,
            CarConfig::START_COUNTDOWN_MS)) {
        Serial.println(
            "[BUTTON][ERROR] cannot start countdown");
        return;
    }

    udpComm.notifyCountdownStarted(
        missionManager,
        CarConfig::START_COUNTDOWN_MS);

    Serial.printf(
        "[BUTTON] accepted: run=%s task=%s; "
        "actual start in %lu seconds\n",
        runId.c_str(),
        missionManager.profile().name,
        static_cast<unsigned long>(
            (CarConfig::START_COUNTDOWN_MS + 999) /
            1000));
}

static void updateLocalStartButton()
{
    static bool initialized = false;
    static bool lastRawPressed = false;
    static bool stablePressed = false;
    static uint32_t rawChangedMs = 0;

    const uint32_t nowMs = millis();
    const bool rawPressed =
        digitalRead(CarConfig::LOCAL_START_PIN) == LOW;

    if (!initialized) {
        initialized = true;
        lastRawPressed = rawPressed;
        stablePressed = rawPressed;
        rawChangedMs = nowMs;
        return;
    }

    if (rawPressed != lastRawPressed) {
        lastRawPressed = rawPressed;
        rawChangedMs = nowMs;
    }

    if (rawPressed == stablePressed ||
        nowMs - rawChangedMs <
            CarConfig::LOCAL_BUTTON_DEBOUNCE_MS) {
        return;
    }

    stablePressed = rawPressed;
    if (stablePressed) {
        handleButtonPressed();
    }
}

static void updateCountdown(uint32_t nowMs)
{
    if (!missionManager.isCountdown()) {
        return;
    }

    const uint32_t remainingMs =
        missionManager.countdownRemainingMs(nowMs);
    const int32_t remainingSecond =
        static_cast<int32_t>(
            (remainingMs + 999) / 1000);

    if (remainingSecond != lastCountdownSecond) {
        lastCountdownSecond = remainingSecond;
        Serial.printf(
            "[COUNTDOWN] %ld\n",
            static_cast<long>(remainingSecond));
    }

    missionManager.updateCountdown(nowMs);
}

static void recordArcSample(
    uint32_t nowMs,
    const WheelState& target)
{
    if (!CarConfig::ARC_MODEL_GENERATION_MODE) {
        return;
    }

    const RouteSegment segment =
        routeController.segment();
    const bool inArc =
        segment == RouteSegment::ARC_BC ||
        segment == RouteSegment::ARC_DA;

    if (!inArc ||
        nowMs - lastArcRecordMs <
            CarConfig::ARC_RECORD_PERIOD_MS) {
        return;
    }

    lastArcRecordMs = nowMs;
    Serial.printf(
        "ARC_SAMPLE,%u,%.5f,%.4f,%.4f\n",
        static_cast<unsigned>(segment),
        routeController.segmentProgress01(),
        target.leftSpeedCmS,
        target.rightSpeedCmS);
}

void setup()
{
    Serial.begin(115200);
    delay(300);

    missionManager.begin();
    routeController.begin();
    speedPlanner.begin();

    const DriveBackendType backendType =
        CarConfig::USE_SIMULATION
            ? DriveBackendType::SIMULATION
            : DriveBackendType::STEPPER;

    driveBackendReady =
        driveBackend.begin(backendType);

    odometry.begin(0.0f, 0.0f, 0.0f);
    resetOdometryToStart();
    telemetry.begin();
    udpComm.begin();

    pinMode(CarConfig::LOCAL_START_PIN,
            INPUT_PULLUP);

    previousMissionState =
        missionManager.state();
    lastControlMs = millis();
    lastTelemetryMs = lastControlMs;
    lastHeartbeatMs = lastControlMs;

    Serial.println();
    Serial.println(
        "AirGroundCar ESP32 task1 communication build");
    Serial.println(
        "Flow: GS MODE -> CAR READY -> GPIO32 -> "
        "10s countdown -> CAR motion + UAV START");

    Serial.printf(
        "[CONFIG] T1 speeds: preA=%.1f AB=%.1f "
        "BC=%.1f CD=%.1f DA=%.1f cm/s\n",
        CarConfig::T1_SPEED_APPROACH_A_CM_S,
        CarConfig::T1_SPEED_AB_CM_S,
        CarConfig::T1_SPEED_BC_CM_S,
        CarConfig::T1_SPEED_CD_CM_S,
        CarConfig::T1_SPEED_DA_CM_S);

    if (!driveBackendReady) {
        Serial.println(
            "[DRIVE][ERROR] initialization failed");
        missionManager.fault();
        previousMissionState =
            missionManager.state();
    } else {
        Serial.printf(
            "[DRIVE] ready: pulses/rev=%.0f, "
            "max=%luHz\n",
            CarConfig::STEPPER_PULSES_PER_REV,
            static_cast<unsigned long>(
                CarConfig::STEPPER_MAX_PULSE_HZ));
    }

    Serial.printf(
        "[WAIT] listening for CMD:<id>:MODE:T1/T2 "
        "on UDP %u\n",
        CarConfig::CAR_PORT);
}

void loop()
{
    udpComm.update(missionManager);
    updateLocalStartButton();

    const uint32_t nowMs = millis();
    updateCountdown(nowMs);
    handleMissionStateChange();

    if (nowMs - lastControlMs >=
        CarConfig::CONTROL_PERIOD_MS) {
        const float dtSeconds =
            (nowMs - lastControlMs) / 1000.0f;
        lastControlMs = nowMs;

        MotionCommand requested{0.0f, 0.0f};

        if (missionManager.isRunning()) {
            const Pose2D boardPose =
                odometry.boardPose(
                    CarConfig::BOARD_TO_REAR_CM);

            requested = routeController.update(
                odometry.pose(),
                boardPose,
                missionManager.profile(),
                dtSeconds);
        }

        const MotionCommand planned =
            speedPlanner.update(
                requested,
                missionManager.profile(),
                dtSeconds);

        const WheelState target =
            speedPlanner.wheelTarget(
                CarConfig::WHEEL_TRACK_CM);

        recordArcSample(nowMs, target);

        const WheelState actual =
            driveBackend.update(
                target, dtSeconds);

        if (missionManager.isRunning() &&
            nowMs - lastMotionDebugMs >=
                CarConfig::MOTION_DEBUG_PERIOD_MS) {
            lastMotionDebugMs = nowMs;

            const StepperDriverStatus& stepper =
                driveBackend.stepperStatus();

            Serial.printf(
                "[MOTION] task=%s segment=%u "
                "track_s=%.1f route_s=%.1f "
                "v=%.2f w=%.3f "
                "L=%.2f(%luHz) R=%.2f(%luHz)\n",
                missionManager.profile().name,
                static_cast<unsigned>(
                    routeController.segment()),
                routeController.trackProgressCm(),
                routeController.routeProgressCm(),
                planned.linearCmS,
                planned.angularRadS,
                target.leftSpeedCmS,
                static_cast<unsigned long>(
                    stepper.left.pulseFrequencyHz),
                target.rightSpeedCmS,
                static_cast<unsigned long>(
                    stepper.right.pulseFrequencyHz));
        }

        odometry.update(
            actual.leftSpeedCmS,
            actual.rightSpeedCmS,
            CarConfig::WHEEL_TRACK_CM,
            dtSeconds);

        String point;
        if (routeController.consumePointEvent(point)) {
            Serial.printf("[ROUTE] point %s\n",
                          point.c_str());
            udpComm.notifyPoint(
                missionManager, point);
        }

        if (missionManager.isRunning() &&
            routeController.isComplete() &&
            routeController.boardErrorCm() <=
                CarConfig::
                ROUTE_COMPLETE_POSITION_TOLERANCE_CM &&
            fabsf(planned.linearCmS) < 0.05f &&
            fabsf(planned.angularRadS) < 0.005f) {
            missionManager.finish();
        }
    }

    handleMissionStateChange();

    if (CarConfig::ENABLE_WIFI_UDP) {
        if (udpComm.consumeStatusRequest() ||
            nowMs - lastTelemetryMs >=
                CarConfig::TELEMETRY_PERIOD_MS) {
            lastTelemetryMs = nowMs;
            telemetry.publish(
                nowMs,
                missionManager,
                routeController,
                speedPlanner.command(),
                driveBackend,
                odometry,
                udpComm);
        }

        if (nowMs - lastHeartbeatMs >=
            CarConfig::HEARTBEAT_PERIOD_MS) {
            lastHeartbeatMs = nowMs;
            telemetry.publishHeartbeat(
                ++heartbeatSequence,
                missionManager,
                udpComm);
        }
    }
}
