#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

#include <Arduino.h>
#include <IPAddress.h>
#include "car_types.h"

namespace CarConfig {

// 当前阶段使用仿真后端。接入真实步进驱动后再改为 false。
constexpr bool USE_SIMULATION = false;

constexpr bool ENABLE_WIFI_UDP = false;
constexpr bool ALLOW_REMOTE_START = false;

constexpr uint32_t CONTROL_PERIOD_MS = 20;
constexpr uint32_t TELEMETRY_PERIOD_MS = 50;  // 20 Hz，满足无人机 0.5 s 超时要求
constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;

// ==================== 标准跑道参数 ====================
// 75 cm 表示车体/平台中心 P 沿跑道运行时的半圆半径。
constexpr float BOARD_PATH_RADIUS_CM = 75.0f;

// 跑道上下两个半圆切点之间的有效直线距离，仍为 150 cm。
constexpr float TRACK_STRAIGHT_LENGTH_CM = 150.0f;

// 实车摆放要求：车头位于出发线时，车体中心位于线后方 30 cm。
constexpr float START_LINE_TO_BOARD_CENTER_CM = 30.0f;

// 因此第一次入圆前，中心需要行驶 30 + 150 = 180 cm。
constexpr float FIRST_STRAIGHT_LENGTH_CM =
    TRACK_STRAIGHT_LENGTH_CM + START_LINE_TO_BOARD_CENTER_CM;

// 初始坐标定义为车体中心的实际起点，即出发线后方 30 cm。
// 原出发线 A 的中心坐标为 START_BOARD_Y_CM + 30 cm。
constexpr float START_BOARD_X_CM = 150.0f;
constexpr float START_BOARD_Y_CM = 170.0f;
constexpr float START_LINE_BOARD_Y_CM =
    START_BOARD_Y_CM + START_LINE_TO_BOARD_CENTER_CM;
constexpr float TOP_TANGENT_BOARD_Y_CM =
    START_LINE_BOARD_Y_CM + TRACK_STRAIGHT_LENGTH_CM;

constexpr float START_YAW_RAD = PI * 0.5f;

// ==================== 实测机械参数 ====================
// 左右后轮中心距。
constexpr float WHEEL_TRACK_CM = 62.85f;

// 后轮直径、半径和轮宽。
constexpr float WHEEL_DIAMETER_CM = 6.37f;
constexpr float WHEEL_RADIUS_CM = WHEEL_DIAMETER_CM * 0.5f;
constexpr float WHEEL_WIDTH_CM = 1.35f;

// 前轮中心到后轮轴中心的轴距。
constexpr float WHEELBASE_CM = 53.75f;

// 板中心位于后轮轴中心前方 27.5 cm。
constexpr float BOARD_TO_REAR_CM = 27.50f;
constexpr float BOARD_TO_FRONT_CM = 30.0f;

// ==================== Stepper driver ====================
// 2026-07-30 实测：左右车轮均为 3200 个 STEP 脉冲转一圈。
constexpr float STEPPER_PULSES_PER_REV = 3200.0f;

// 实测升频至 8000 Hz 未见异常；任务按当前轮径和速度约需 3450 Hz。
// 首次整车测试保留 5000 Hz 软件安全上限。
constexpr uint32_t STEPPER_MAX_PULSE_HZ = 5000;

// 2026-07-30 入圆实测修正后的逻辑左右轮映射：
// 当前路径模型的正角速度定义遵循标准差速模型。
// 上一版入圆实际向左，说明两路物理轮在逻辑层中对调。
// 因此将 19/21/18 作为逻辑左轮，将 26/27/25 作为逻辑右轮。
// 两路的实测前进DIR电平保持不变。
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

// ==================== 板中心建模参数 ====================
// 板中心位置误差的运动学反馈增益，不是电机 PID。
constexpr float BOARD_POSITION_KP = 1.0f;

// true：在线求解并通过串口输出 ARC_SAMPLE，供后续固化参数表。
// false：保留接口，后续在生成 arc_profile 后切换到查表模式。
constexpr bool ARC_MODEL_GENERATION_MODE = false;
constexpr uint32_t ARC_RECORD_PERIOD_MS = 50;

// 纯运动学验证先设为 0；公式验证通过后可改回 0.15。
constexpr float SIM_WHEEL_TIME_CONSTANT_S = 0.0f;

// 完成一圈后，板中心进入该误差范围才允许停止位置修正。
constexpr float ROUTE_COMPLETE_POSITION_TOLERANCE_CM = 0.5f;

// ==================== 本地按键任务1测试 ====================
// 使用外接常开按键：GPIO32 -- 按键 -- GND。
// GPIO32 使用内部上拉，按下时读取 LOW；无需额外上拉电阻。
// 上电时请保持按键松开，程序完成初始化后按下一次启动任务1。
constexpr bool USE_LOCAL_START_BUTTON = true;
constexpr uint8_t LOCAL_START_PIN = 32;
constexpr uint32_t LOCAL_BUTTON_DEBOUNCE_MS = 50;

extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;
extern const IPAddress GS_IP;
extern const IPAddress UAV_IP;
extern const IPAddress CAR_IP;
extern const IPAddress WIFI_GATEWAY;
extern const IPAddress WIFI_SUBNET;
extern const IPAddress WIFI_DNS1;
extern const IPAddress WIFI_DNS2;

constexpr uint16_t GS_PORT = 8889;
constexpr uint16_t UAV_PORT = 8888;
constexpr uint16_t CAR_PORT = 8890;

}  // namespace CarConfig

namespace MissionConfig {
const MissionProfile& getProfile(MissionId id);
}

#endif
