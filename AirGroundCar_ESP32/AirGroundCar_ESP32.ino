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
uint32_t heartbeatSequence = 0;
MissionState previousMissionState = MissionState::IDLE;

static void resetMotionModules()
{
    routeController.reset();
    speedPlanner.reset();
    driveBackend.reset();
    odometry.reset(CarConfig::START_X_CM,
                   CarConfig::START_Y_CM,
                   CarConfig::START_YAW_RAD);
}

static void handleMissionStateChange()
{
    const MissionState current = missionManager.state();
    if (current == previousMissionState) {
        return;
    }

    if (current == MissionState::RUNNING) {
        resetMotionModules();
        routeController.start(odometry.distanceCm());
        udpComm.notifyMissionStarted(missionManager);
        Serial.println("[MISSION] running");
    } else if (current == MissionState::IDLE) {
        resetMotionModules();
        Serial.println("[MISSION] reset to idle");
    } else if (current == MissionState::FINISHED) {
        driveBackend.stop();
        udpComm.notifyMissionDone(missionManager);
        Serial.println("[MISSION] finished");
    }

    previousMissionState = current;
}

static void processSerialCommand()
{
    while (Serial.available() > 0) {
        const char command = (char)Serial.read();

        switch (command) {
            case '1':
                Serial.println(missionManager.selectMission(MissionId::TASK_1)
                    ? "[CMD] MODE T1 -> READY"
                    : "[CMD] MODE T1 rejected");
                break;

            case '2':
                Serial.println(missionManager.selectMission(MissionId::TASK_2)
                    ? "[CMD] MODE T2 -> READY"
                    : "[CMD] MODE T2 rejected");
                break;

            case 'A':
            case 'a':
                Serial.println(missionManager.arm("R001")
                    ? "[CMD] ARMED R001"
                    : "[CMD] ARM rejected");
                break;

            case 'S':
            case 's':
                Serial.println(missionManager.localStart()
                    ? "[CMD] local start accepted"
                    : "[CMD] local start rejected");
                break;

            case '0':
                missionManager.reset();
                break;

            case 'R':
            case 'r':
                resetMotionModules();
                Serial.println("[CMD] motion modules reset");
                break;

            case 'P':
            case 'p':
                telemetry.publish(millis(),
                                  missionManager,
                                  routeController,
                                  speedPlanner.command(),
                                  driveBackend,
                                  odometry,
                                  udpComm);
                break;

            case '\r':
            case '\n':
                break;

            default:
                Serial.printf("[CMD] unknown: %c\n", command);
                break;
        }
    }
}

static void updateLocalStartButton()
{
    if (!CarConfig::USE_LOCAL_START_BUTTON) {
        return;
    }

    // 初版只做最简单的低电平检测，下一版再加入完整消抖。
    static bool previousPressed = false;
    const bool pressed = digitalRead(CarConfig::LOCAL_START_PIN) == LOW;
    if (pressed && !previousPressed) {
        missionManager.localStart();
    }
    previousPressed = pressed;
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
    driveBackend.begin(backendType);

    odometry.begin(CarConfig::START_X_CM,
                   CarConfig::START_Y_CM,
                   CarConfig::START_YAW_RAD);
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
    Serial.println("AirGroundCar ESP32 initial architecture");
    Serial.println("1=T1, 2=T2, A=arm, S=local start, 0=reset, P=status");
    Serial.println(CarConfig::USE_SIMULATION
        ? "backend=SIMULATION"
        : "backend=STEPPER (driver is still a stub)");
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
            requested = routeController.update(
                odometry.distanceCm(),
                missionManager.profile());
        }

        const MotionCommand planned = speedPlanner.update(
            requested,
            missionManager.profile(),
            dtSeconds);

        const WheelState target = speedPlanner.wheelTarget(
            CarConfig::WHEEL_TRACK_CM);
        const WheelState actual = driveBackend.update(target, dtSeconds);

        odometry.update(actual.leftSpeedCmS,
                        actual.rightSpeedCmS,
                        CarConfig::WHEEL_TRACK_CM,
                        dtSeconds);

        String point;
        if (routeController.consumePointEvent(point)) {
            Serial.printf("[ROUTE] point %s\n", point.c_str());
            udpComm.notifyPoint(missionManager, point);
        }

        if (missionManager.isRunning() &&
            routeController.isComplete() &&
            fabsf(planned.linearCmS) < 0.05f) {
            missionManager.finish();
        }
    }

    handleMissionStateChange();

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
