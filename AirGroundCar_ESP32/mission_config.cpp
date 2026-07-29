#include "mission_config.h"

namespace CarConfig {

const char* WIFI_SSID = "TP-LINK_B592";
const char* WIFI_PASSWORD = "dsnbdsnb789";

// 地面站当前 config.json 使用的小车地址为 192.168.77.103:8890。
// ESP32 必须真正配置该静态地址，不能只声明 CAR_IP 后继续使用 DHCP。
const IPAddress GS_IP(192, 168, 77, 104);      // 仅作为首次收到命令前的回退地址
const IPAddress UAV_IP(192, 168, 168, 101);
const IPAddress CAR_IP(192, 168, 77, 103);
const IPAddress WIFI_GATEWAY(192, 168, 77, 1);
const IPAddress WIFI_SUBNET(255, 255, 255, 0);
const IPAddress WIFI_DNS1(192, 168, 77, 1);
const IPAddress WIFI_DNS2(8, 8, 8, 8);

}  // namespace CarConfig

namespace {

const MissionProfile TASK1{
    MissionId::TASK_1,
    "T1",
    16.0f,
    15.0f,
    15.0f,
    1.5f
};

const MissionProfile TASK2{
    MissionId::TASK_2,
    "T2",
    13.0f,
    12.0f,
    12.0f,
    1.2f
};

const MissionProfile IDLE{
    MissionId::NONE,
    "NONE",
    0.0f,
    10.0f,
    10.0f,
    1.0f
};

}  // namespace

namespace MissionConfig {

const MissionProfile& getProfile(MissionId id)
{
    if (id == MissionId::TASK_1) {
        return TASK1;
    }
    if (id == MissionId::TASK_2) {
        return TASK2;
    }
    return IDLE;
}

}  // namespace MissionConfig
