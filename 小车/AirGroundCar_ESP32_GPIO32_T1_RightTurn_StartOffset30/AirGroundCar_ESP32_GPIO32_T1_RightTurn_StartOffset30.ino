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
uint32_t heartbeatSequence = 0;
uint32_t lastMotionDebugMs = 0;
MissionState previousMissionState = MissionState::IDLE;
bool driveBackendReady = false;

// 配置中的 A 点是板中心坐标，里程计内部保存的是后轮轴中心坐标。
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

static void handleMissionStateChange()
{
    const MissionState current = missionManager.state();
    if (current == previousMissionState) {
        return;
    }

    if (current == MissionState::RUNNING) {
        resetMotionModules();
        routeController.start();
        if (CarConfig::ENABLE_WIFI_UDP) {
            udpComm.notifyMissionStarted(missionManager);
        }
        Serial.println("[MISSION] running: task1 route started");
        if (CarConfig::ARC_MODEL_GENERATION_MODE) {
            Serial.println(
                "ARC_SAMPLE_FORMAT,segment,progress01,left_cm_s,right_cm_s");
        }
    } else if (current == MissionState::IDLE) {
        resetMotionModules();
        Serial.println("[MISSION] reset to idle");
    } else if (current == MissionState::FINISHED) {
        // 本地测试完成后直接关闭STEP与EN，避免长时间保持励磁发热。
        driveBackend.reset();
        if (CarConfig::ENABLE_WIFI_UDP) {
            udpComm.notifyMissionDone(missionManager);
        }
        Serial.println(
            "[MISSION] finished; motors disabled; "
            "press GPIO32 once to run task1 again");
    }

    previousMissionState = current;
}

static bool prepareLocalTask1()
{
    if (!driveBackendReady || !driveBackend.isReady()) {
        missionManager.fault();
        Serial.println("[LOCAL][ERROR] stepper backend is not ready; button start disabled");
        return false;
    }

    missionManager.reset();
    resetMotionModules();

    if (!missionManager.selectMission(MissionId::TASK_1)) {
        missionManager.fault();
        Serial.println("[LOCAL][ERROR] failed to select task1");
        return false;
    }

    if (!missionManager.arm("LOCAL_T1")) {
        missionManager.fault();
        Serial.println("[LOCAL][ERROR] failed to arm task1");
        return false;
    }

    previousMissionState = missionManager.state();
    Serial.println("[LOCAL] task1 selected and armed");
    Serial.printf(
        "[LOCAL] GPIO%u button: first press=start, second press=stop\n",
        CarConfig::LOCAL_START_PIN);
    return true;
}


static bool rearmLocalTask1()
{
    // reset() 会立即停止两路STEP并关闭电机使能，
    // 适合当前本地底盘测试阶段。
    driveBackend.reset();
    routeController.reset();
    speedPlanner.reset();
    resetOdometryToStart();

    missionManager.reset();
    previousMissionState = missionManager.state();

    return prepareLocalTask1();
}

static void stopLocalTask1()
{
    if (!missionManager.isRunning()) {
        return;
    }

    // 先停脉冲，再重置状态，避免等待下一控制周期。
    driveBackend.reset();
    routeController.reset();
    speedPlanner.reset();
    resetOdometryToStart();

    missionManager.reset();
    previousMissionState = missionManager.state();

    if (prepareLocalTask1()) {
        Serial.println(
            "[BUTTON] task1 stopped; system re-armed, "
            "press GPIO32 again to restart from A");
    } else {
        Serial.println("[BUTTON][ERROR] stopped, but re-arm failed");
    }
}

static void processSerialCommand()
{
    while (Serial.available() > 0) {
        const char command = (char)Serial.read();

        switch (command) {
            case 'R':
            case 'r':
                if (missionManager.isRunning()) {
                    Serial.println("[CMD] rearm rejected while running");
                } else {
                    Serial.println(rearmLocalTask1()
                        ? "[CMD] task1 re-armed"
                        : "[CMD] task1 re-arm failed");
                }
                break;

            case 'S':
            case 's':
                Serial.println(missionManager.localStart()
                    ? "[CMD] serial start accepted"
                    : "[CMD] serial start rejected");
                break;

            case 'X':
            case 'x':
                if (missionManager.isRunning()) {
                    stopLocalTask1();
                } else {
                    driveBackend.reset();
                    Serial.println("[CMD] motors already stopped");
                }
                break;

            case '\r':
            case '\n':
                break;

            default:
                Serial.println(
                    "[CMD] GPIO32: start/stop; "
                    "S=start, X=stop, R=rearm");
                break;
        }
    }
}

static void updateLocalStartButton()
{
    if (!CarConfig::USE_LOCAL_START_BUTTON) {
        return;
    }

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
        nowMs - rawChangedMs < CarConfig::LOCAL_BUTTON_DEBOUNCE_MS) {
        return;
    }

    stablePressed = rawPressed;

    // 只在稳定的按下沿触发一次，松开不会触发。
    if (!stablePressed) {
        return;
    }

    if (missionManager.isRunning()) {
        stopLocalTask1();
        return;
    }

    if (!driveBackendReady || !driveBackend.isReady()) {
        missionManager.fault();
        Serial.println("[BUTTON][ERROR] drive backend not ready");
        return;
    }

    if (missionManager.state() != MissionState::ARMED) {
        if (!rearmLocalTask1()) {
            Serial.printf(
                "[BUTTON][ERROR] cannot re-arm from state=%u\n",
                (unsigned)missionManager.state());
            return;
        }
    }

    if (missionManager.localStart()) {
        Serial.println(
            "[BUTTON] accepted; task1 starting now; "
            "press GPIO32 again to stop");
    } else {
        Serial.println("[BUTTON] start rejected");
    }
}

static void recordArcSample(uint32_t nowMs, const WheelState& target)
{
    if (!CarConfig::ARC_MODEL_GENERATION_MODE) {
        return;
    }

    const RouteSegment segment = routeController.segment();
    const bool inArc =
        segment == RouteSegment::ARC_BC ||
        segment == RouteSegment::ARC_DA;

    if (!inArc ||
        nowMs - lastArcRecordMs < CarConfig::ARC_RECORD_PERIOD_MS) {
        return;
    }

    lastArcRecordMs = nowMs;
    Serial.printf("ARC_SAMPLE,%u,%.5f,%.4f,%.4f\n",
                  (unsigned)segment,
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

    const DriveBackendType backendType = CarConfig::USE_SIMULATION
        ? DriveBackendType::SIMULATION
        : DriveBackendType::STEPPER;
    driveBackendReady = driveBackend.begin(backendType);

    odometry.begin(0.0f, 0.0f, 0.0f);
    resetOdometryToStart();
    telemetry.begin();
    udpComm.begin();

    if (CarConfig::USE_LOCAL_START_BUTTON) {
        pinMode(CarConfig::LOCAL_START_PIN, INPUT_PULLUP);
    }

    previousMissionState = missionManager.state();
    lastControlMs = millis();
    lastTelemetryMs = lastControlMs;
    lastHeartbeatMs = lastControlMs;

    Serial.println();
    Serial.println("AirGroundCar ESP32 GPIO32 T1 right-turn + 30cm start-offset test");
    Serial.println("Wi-Fi/UDP disabled; T1; right-turn; center starts 30cm behind line");
    Serial.println(CarConfig::USE_SIMULATION
        ? "backend=SIMULATION"
        : "backend=STEPPER (LEDC pulse driver)");

    if (!driveBackendReady) {
        Serial.println("[DRIVE][ERROR] backend initialization failed; motor output is disabled");

        if (backendType == DriveBackendType::STEPPER) {
            const StepperDriverStatus& status = driveBackend.stepperStatus();
            Serial.printf(
                "[DRIVE][STATUS] begun=%s config_valid=%s pulse_generator_ready=%s "
                "left_attached=%s right_attached=%s\n",
                status.begun ? "true" : "false",
                status.configValid ? "true" : "false",
                status.pulseGeneratorReady ? "true" : "false",
                status.left.pulseGeneratorAttached ? "true" : "false",
                status.right.pulseGeneratorAttached ? "true" : "false");

            if (!status.configValid) {
                Serial.printf(
                    "[DRIVE][ERROR] invalid stepper calibration: "
                    "wheel_diameter_cm=%.3f, pulses_per_rev=%.3f; "
                    "set STEPPER_PULSES_PER_REV > 0 after calibration\n",
                    CarConfig::WHEEL_DIAMETER_CM,
                    CarConfig::STEPPER_PULSES_PER_REV);
            }

            if (!status.pulseGeneratorReady) {
                Serial.println(
                    "[DRIVE][ERROR] LEDC pulse generator attach failed; "
                    "check ESP32 Arduino core compatibility and STEP pins");
            }
        }
    } else {
        Serial.println("[DRIVE] backend initialization ready");
        if (backendType == DriveBackendType::STEPPER) {
            Serial.printf(
                "[DRIVE] pulses_per_rev=%.0f max_pulse_hz=%lu "
                "left_forward_dir=%s right_forward_dir=%s\n",
                CarConfig::STEPPER_PULSES_PER_REV,
                static_cast<unsigned long>(CarConfig::STEPPER_MAX_PULSE_HZ),
                CarConfig::LEFT_FORWARD_DIR_LEVEL == HIGH ? "HIGH" : "LOW",
                CarConfig::RIGHT_FORWARD_DIR_LEVEL == HIGH ? "HIGH" : "LOW");
        }
    }

    Serial.printf("start board=(%.2f, %.2f), rear offset=%.2f cm\n",
                  CarConfig::START_BOARD_X_CM,
                  CarConfig::START_BOARD_Y_CM,
                  CarConfig::BOARD_TO_REAR_CM);

    if (driveBackendReady) {
        prepareLocalTask1();
    } else {
        missionManager.fault();
        previousMissionState = missionManager.state();
        Serial.println("[LOCAL][ERROR] initialization failed; task1 not armed");
    }
}

void loop()
{
    processSerialCommand();
    updateLocalStartButton();
    udpComm.update(missionManager);
    handleMissionStateChange();

    const uint32_t nowMs = millis();

    if (nowMs - lastControlMs >= CarConfig::CONTROL_PERIOD_MS) {
        const float dtSeconds =
            (nowMs - lastControlMs) / 1000.0f;
        lastControlMs = nowMs;

        MotionCommand requested{0.0f, 0.0f};
        if (missionManager.isRunning()) {
            const Pose2D boardPose =
                odometry.boardPose(CarConfig::BOARD_TO_REAR_CM);

            requested = routeController.update(
                odometry.pose(),
                boardPose,
                missionManager.profile(),
                dtSeconds);
        }

        const MotionCommand planned = speedPlanner.update(
            requested,
            missionManager.profile(),
            dtSeconds);

        const WheelState target = speedPlanner.wheelTarget(
            CarConfig::WHEEL_TRACK_CM);

        recordArcSample(nowMs, target);

        const WheelState actual = driveBackend.update(target, dtSeconds);

        // 每500 ms同时打印逻辑轮速和实际STEP频率。
        // segment=2/4时左右频率必须明显不同。
        if (missionManager.isRunning() &&
            nowMs - lastMotionDebugMs >= 500) {
            lastMotionDebugMs = nowMs;

            const StepperDriverStatus& stepper =
                driveBackend.stepperStatus();

            Serial.printf(
                "[MOTION] segment=%u path_s=%.1f "
                "v=%.2f w=%.3f "
                "left=%.2fcm/s(%luHz) "
                "right=%.2fcm/s(%luHz)\n",
                static_cast<unsigned>(routeController.segment()),
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

        odometry.update(actual.leftSpeedCmS,
                        actual.rightSpeedCmS,
                        CarConfig::WHEEL_TRACK_CM,
                        dtSeconds);

        String point;
        if (routeController.consumePointEvent(point)) {
            Serial.printf("[ROUTE] point %s\n", point.c_str());
            if (CarConfig::ENABLE_WIFI_UDP) {
                udpComm.notifyPoint(missionManager, point);
            }
        }

        if (missionManager.isRunning() &&
            routeController.isComplete() &&
            routeController.boardErrorCm() <=
                CarConfig::ROUTE_COMPLETE_POSITION_TOLERANCE_CM &&
            fabsf(planned.linearCmS) < 0.05f &&
            fabsf(planned.angularRadS) < 0.005f) {
            missionManager.finish();
        }
    }

    handleMissionStateChange();

    if (CarConfig::ENABLE_WIFI_UDP) {
        if (udpComm.consumeStatusRequest() ||
            nowMs - lastTelemetryMs >= CarConfig::TELEMETRY_PERIOD_MS) {
            lastTelemetryMs = nowMs;
            telemetry.publish(nowMs,
                              missionManager,
                              routeController,
                              speedPlanner.command(),
                              driveBackend,
                              odometry,
                              udpComm);
        }

        if (nowMs - lastHeartbeatMs >= CarConfig::HEARTBEAT_PERIOD_MS) {
            lastHeartbeatMs = nowMs;
            telemetry.publishHeartbeat(++heartbeatSequence,
                                       missionManager,
                                       udpComm);
        }
    }
}
