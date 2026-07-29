#include "mission_config.h"

namespace CarConfig {

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const IPAddress GS_IP(192, 168, 151, 101);
const IPAddress UAV_IP(192, 168, 151, 102);
const IPAddress CAR_IP(192, 168, 151, 103);

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
