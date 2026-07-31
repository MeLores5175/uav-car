#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

#include "car_config.h"
#include "car_types.h"

namespace MissionConfig {

const MissionProfile& getProfile(MissionId id);

// 根据任务和当前路段返回独立配置的中心巡航速度。
// routeProgressCm 用于把起点后方 30 cm 与正式 AB 段分开设置。
float segmentCruiseSpeedCmS(MissionId id,
                            RouteSegment segment,
                            float routeProgressCm);

}  // namespace MissionConfig

#endif
