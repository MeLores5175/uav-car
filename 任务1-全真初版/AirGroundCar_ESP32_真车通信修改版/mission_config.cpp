#include "mission_config.h"

namespace CarConfig {

const char* WIFI_SSID = "TP-LINK_B592";
const char* WIFI_PASSWORD = "dsnbdsnb789";

// 当前联调网段：地面站 192.168.77.104、无人机 192.168.77.103、小车 192.168.77.102。
// 三者必须在同一子网；现场地址变化时只改这里和地面站设置页/无人机 launch 参数。
const IPAddress GS_IP(192, 168, 77, 104);      // 尚未学习到地面站地址时的回退值
const IPAddress UAV_IP(192, 168, 77, 103);
const IPAddress CAR_IP(192, 168, 77, 102);
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
