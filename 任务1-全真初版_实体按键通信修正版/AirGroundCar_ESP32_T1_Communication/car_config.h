#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#include <Arduino.h>
#include <IPAddress.h>

namespace CarConfig {

// ============================================================================
// 只需要修改这个文件
// Wi-Fi、目标 IP/端口、10 秒延时、每段速度、机械参数和电机参数都集中在这里。
// ============================================================================

// -------------------- 编译/运行模式 --------------------
constexpr bool USE_SIMULATION = false;
constexpr bool ENABLE_WIFI_UDP = true;

// ESP32 本机默认使用 DHCP。路由器应按 ESP32 Wi-Fi MAC 地址保留 .102。
// 监听本机 IP 不需要写死地址，udp.begin(CAR_PORT) 会监听当前 DHCP 地址。
constexpr bool USE_DHCP = true;

// 实车联调时建议保持 true：未连接局域网时按键不会进入 10 秒倒计时。
constexpr bool REQUIRE_NETWORK_BEFORE_BUTTON = true;

// 测试阶段允许再次按 GPIO32 取消倒计时或停止小车。
// 正式封箱前若不希望按键承担停止功能，可改为 false。
constexpr bool ALLOW_BUTTON_CANCEL_OR_STOP = true;

// -------------------- Wi-Fi --------------------
static const char WIFI_SSID[] = "TP-LINK_B592";
static const char WIFI_PASSWORD[] = "dsnbdsnb789";

// 发送目标地址。小车自己的地址不在这里写死；建议路由器保留为 192.168.77.102。
static const IPAddress UAV_IP(192, 168, 77, 103);
static const IPAddress GS_FALLBACK_IP(192, 168, 77, 104);

// 仅 USE_DHCP=false 时使用。
static const IPAddress CAR_STATIC_IP(192, 168, 77, 102);
static const IPAddress WIFI_GATEWAY(192, 168, 77, 1);
static const IPAddress WIFI_SUBNET(255, 255, 255, 0);
static const IPAddress WIFI_DNS1(192, 168, 77, 1);
static const IPAddress WIFI_DNS2(8, 8, 8, 8);

constexpr uint16_t GS_PORT = 8889;
constexpr uint16_t UAV_PORT = 8888;
constexpr uint16_t CAR_PORT = 8890;

// -------------------- 通信频率与可靠性 --------------------
constexpr uint32_t TELEMETRY_PERIOD_MS = 50;   // 20 Hz，同时发给 UAV 和 GS
constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;
constexpr uint32_t UAV_START_RETRY_INTERVAL_MS = 300;
constexpr uint8_t UAV_START_MAX_SEND_COUNT = 20;  // 最长约 6 秒
constexpr uint8_t COMMAND_DEDUP_CACHE_SIZE = 12;
constexpr uint8_t CRITICAL_EVENT_REPEAT_COUNT = 3;

// -------------------- 实体按键启动 --------------------
constexpr uint8_t LOCAL_START_PIN = 32;
constexpr uint32_t LOCAL_BUTTON_DEBOUNCE_MS = 50;

// 收到 MODE 并回传 READY 后，队员按键；小车等待 10 秒才真正运动并通知无人机 START。
constexpr uint32_t START_COUNTDOWN_MS = 10000;

// -------------------- 控制周期 --------------------
constexpr uint32_t CONTROL_PERIOD_MS = 20;
constexpr uint32_t MOTION_DEBUG_PERIOD_MS = 500;

// -------------------- 场地与起点 --------------------
// 平台/车体中心轨迹半圆半径。
constexpr float BOARD_PATH_RADIUS_CM = 75.0f;

// A-B、C-D 黑线切点之间的直线长度。
constexpr float TRACK_STRAIGHT_LENGTH_CM = 150.0f;

// 测试要求是车头放在 A 横线上；车体中心在横线后方 30 cm。
constexpr float START_LINE_TO_BOARD_CENTER_CM = 30.0f;
constexpr float FIRST_STRAIGHT_LENGTH_CM =
    START_LINE_TO_BOARD_CENTER_CM + TRACK_STRAIGHT_LENGTH_CM;

// 场地左下角为 (0,0)，X 向右、Y 向上。
// A 轨迹点中心坐标为 (150,200)，实际启动中心为 (150,170)。
constexpr float START_BOARD_X_CM = 150.0f;
constexpr float START_BOARD_Y_CM = 170.0f;
constexpr float START_LINE_BOARD_Y_CM =
    START_BOARD_Y_CM + START_LINE_TO_BOARD_CENTER_CM;
constexpr float TOP_TANGENT_BOARD_Y_CM =
    START_LINE_BOARD_Y_CM + TRACK_STRAIGHT_LENGTH_CM;
constexpr float START_YAW_RAD = PI * 0.5f;

// -------------------- 任务 1 分段中心速度 --------------------
// 当前默认全部保持已验证的 16 cm/s；以后只改这里即可独立调速。
constexpr float T1_SPEED_APPROACH_A_CM_S = 16.0f; // 起点中心到 A：前 30 cm
constexpr float T1_SPEED_AB_CM_S = 8.0f;
constexpr float T1_SPEED_BC_CM_S = 16.0f;
constexpr float T1_SPEED_CD_CM_S = 16.0f;
constexpr float T1_SPEED_DA_CM_S = 16.0f;

constexpr float T1_MAX_ACCEL_CM_S2 = 15.0f;
constexpr float T1_MAX_DECEL_CM_S2 = 15.0f;
constexpr float T1_MAX_ANGULAR_ACCEL_RAD_S2 = 1.5f;

// -------------------- 任务 2 预留分段速度 --------------------
// 当前只完善任务 1；任务 2 先保留同一路线和较低默认速度。
constexpr float T2_SPEED_APPROACH_A_CM_S = 13.0f;
constexpr float T2_SPEED_AB_CM_S = 13.0f;
constexpr float T2_SPEED_BC_CM_S = 13.0f;
constexpr float T2_SPEED_CD_CM_S = 13.0f;
constexpr float T2_SPEED_DA_CM_S = 13.0f;

constexpr float T2_MAX_ACCEL_CM_S2 = 12.0f;
constexpr float T2_MAX_DECEL_CM_S2 = 12.0f;
constexpr float T2_MAX_ANGULAR_ACCEL_RAD_S2 = 1.2f;

// -------------------- 机械参数 --------------------
constexpr float WHEEL_TRACK_CM = 62.85f;
constexpr float WHEEL_DIAMETER_CM = 6.37f;
constexpr float WHEEL_RADIUS_CM = WHEEL_DIAMETER_CM * 0.5f;
constexpr float WHEEL_WIDTH_CM = 1.35f;
constexpr float WHEELBASE_CM = 53.75f;
constexpr float BOARD_TO_REAR_CM = 27.50f;
constexpr float BOARD_TO_FRONT_CM = 30.0f;

// -------------------- 步进电机参数 --------------------
constexpr float STEPPER_PULSES_PER_REV = 3200.0f;
constexpr uint32_t STEPPER_MAX_PULSE_HZ = 5000;

// 已经实测并修正的逻辑左右轮映射。
constexpr uint8_t LEFT_STP = 19;
constexpr uint8_t LEFT_DIR = 21;
constexpr uint8_t LEFT_EN = 18;
constexpr uint8_t RIGHT_STP = 26;
constexpr uint8_t RIGHT_DIR = 27;
constexpr uint8_t RIGHT_EN = 25;

constexpr uint8_t STEPPER_EN_ENABLE_LEVEL = LOW;
constexpr uint8_t STEPPER_EN_DISABLE_LEVEL = HIGH;
constexpr uint8_t LEFT_FORWARD_DIR_LEVEL = LOW;
constexpr uint8_t RIGHT_FORWARD_DIR_LEVEL = HIGH;

// -------------------- 路径控制参数 --------------------
constexpr float BOARD_POSITION_KP = 1.0f;
constexpr float ROUTE_COMPLETE_POSITION_TOLERANCE_CM = 0.5f;

// 建模/调试开关。
constexpr bool ARC_MODEL_GENERATION_MODE = false;
constexpr uint32_t ARC_RECORD_PERIOD_MS = 50;
constexpr float SIM_WHEEL_TIME_CONSTANT_S = 0.0f;

}  // namespace CarConfig

#endif
