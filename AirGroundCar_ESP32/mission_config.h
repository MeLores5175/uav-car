#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

#include <Arduino.h>
#include <IPAddress.h>
#include "car_types.h"

namespace CarConfig {

// 当前阶段使用仿真后端。接入真实步进驱动后再改为 false。
constexpr bool USE_SIMULATION = true;

constexpr bool ENABLE_WIFI_UDP = true;
constexpr bool ALLOW_REMOTE_START = USE_SIMULATION;

constexpr uint32_t CONTROL_PERIOD_MS = 20;
constexpr uint32_t TELEMETRY_PERIOD_MS = 200;
constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;

// ==================== 标准跑道参数 ====================
// 75 cm 表示板中心沿黑线运行时的半圆半径。
constexpr float BOARD_PATH_RADIUS_CM = 75.0f;
constexpr float STRAIGHT_LENGTH_CM = 150.0f;

// A 点表示板中心 P 的起始位置，而不是后轮轴中心 O。
constexpr float START_BOARD_X_CM = 150.0f;
constexpr float START_BOARD_Y_CM = 200.0f;
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
constexpr float BOARD_TO_FRONT_CM = 26.25f;

// 仍为占位值：必须在电机、驱动器细分和传动比确定后重新填写。
constexpr float STEPPER_PULSES_PER_REV = 3200.0f;

// ==================== 板中心建模参数 ====================
// 板中心位置误差的运动学反馈增益，不是电机 PID。
constexpr float BOARD_POSITION_KP = 1.0f;

// true：在线求解并通过串口输出 ARC_SAMPLE，供后续固化参数表。
// false：保留接口，后续在生成 arc_profile 后切换到查表模式。
constexpr bool ARC_MODEL_GENERATION_MODE = true;
constexpr uint32_t ARC_RECORD_PERIOD_MS = 50;

// 纯运动学验证先设为 0；公式验证通过后可改回 0.15。
constexpr float SIM_WHEEL_TIME_CONSTANT_S = 0.0f;

// 完成一圈后，板中心进入该误差范围才允许停止位置修正。
constexpr float ROUTE_COMPLETE_POSITION_TOLERANCE_CM = 0.5f;

// 以后接实体按键时再开启并修改引脚。
constexpr bool USE_LOCAL_START_BUTTON = false;
constexpr uint8_t LOCAL_START_PIN = 27;

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
