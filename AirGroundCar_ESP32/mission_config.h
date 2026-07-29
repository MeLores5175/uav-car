#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

#include <Arduino.h>
#include <IPAddress.h>
#include "car_types.h"

namespace CarConfig {

// 当前阶段默认使用仿真后端，底盘完成后再改为 false。
constexpr bool USE_SIMULATION = true;

// 当前 UDP 代码已经搭好，但默认关闭，避免 Wi-Fi 参数未填写时反复连接。
constexpr bool ENABLE_WIFI_UDP = false;
constexpr bool ALLOW_REMOTE_START = false;

constexpr uint32_t CONTROL_PERIOD_MS = 20;
constexpr uint32_t TELEMETRY_PERIOD_MS = 200;
constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;

// 标准跑道参数。
constexpr float TRACK_RADIUS_CM = 75.0f;
constexpr float STRAIGHT_LENGTH_CM = 150.0f;

// 场地坐标系中的 A 点和初始航向。
constexpr float START_X_CM = 150.0f;
constexpr float START_Y_CM = 200.0f;
constexpr float START_YAW_RAD = PI * 0.5f;

// 机械参数均为占位值，底盘完成后必须实测。
constexpr float WHEEL_TRACK_CM = 35.0f;
constexpr float WHEEL_DIAMETER_CM = 10.0f;
constexpr float STEPPER_PULSES_PER_REV = 3200.0f;

// 以后接实体按键时再开启并修改引脚。
constexpr bool USE_LOCAL_START_BUTTON = false;
constexpr uint8_t LOCAL_START_PIN = 27;

extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;
extern const IPAddress GS_IP;
extern const IPAddress UAV_IP;
extern const IPAddress CAR_IP;

constexpr uint16_t GS_PORT = 8889;
constexpr uint16_t UAV_PORT = 8888;
constexpr uint16_t CAR_PORT = 8890;

}  // namespace CarConfig

namespace MissionConfig {
const MissionProfile& getProfile(MissionId id);
}

#endif
