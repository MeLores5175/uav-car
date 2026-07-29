#include "stepper_driver.h"

bool StepperDriver::begin()
{
    reset();

    // TODO：底盘完成后在这里初始化左右 PUL / DIR / ENA / ALM。
    // 当前文件只是固定接口，尚未产生真实步进脉冲。
    return true;
}

void StepperDriver::reset()
{
    command_ = {0.0f, 0.0f};
}

WheelState StepperDriver::update(const WheelState& target, float dtSeconds)
{
    (void)dtSeconds;
    command_ = target;

    // TODO：将 cm/s 换算为 PUL 频率并输出。
    return command_;
}

void StepperDriver::stop()
{
    command_ = {0.0f, 0.0f};
    // TODO：停止 PUL 输出。
}

const WheelState& StepperDriver::wheelState() const
{
    return command_;
}

bool StepperDriver::hasAlarm() const
{
    // TODO：接入闭环驱动器 ALM 后返回真实状态。
    return false;
}
