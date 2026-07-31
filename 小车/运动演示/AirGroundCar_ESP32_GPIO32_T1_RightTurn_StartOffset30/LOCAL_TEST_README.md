# GPIO32 本地任务1实车测试版

## 已确认参数

- 每圈脉冲数：3200 pulse/rev
- 逻辑左轮：STEP19 / DIR21 / EN18，DIR=LOW 前进
- 逻辑右轮：STEP26 / DIR27 / EN25，DIR=HIGH 前进
- 左右轮使用独立 LEDC 定时器：
  - 左轮 channel 0，初始 997 Hz
  - 右轮 channel 2，初始 1009 Hz
- STEP 软件上限：5000 Hz
- Wi-Fi/UDP：关闭
- 默认任务：任务1

## 按键

外接常开按键：

GPIO32 ---- 按键 ---- GND

程序使用 INPUT_PULLUP。

- 上电后第一次按下：从 A 点开始运行任务1
- 运行中再次按下：立即停止，并重新 ARM
- 停止后再次按下：从 A 点重新开始
- 正常跑完后再次按下：重新运行任务1

## 路线

1. 起始中心位于出发线后30 cm，第一段直线180 cm
2. BC 半圆，中心路径半径75 cm，向右入圆
3. CD 对侧直线150 cm
4. DA 半圆，板中心半径 75 cm
5. 回到 A 后停止并关闭电机使能

## 串口

波特率 115200。

关键输出：

- segment=1：AB 直线
- segment=2：BC 半圆
- segment=3：CD 直线
- segment=4：DA 半圆

弯道时会显示左右轮目标速度和 STEP 频率，例如：

[MOTION] segment=2 ... left=21.xxcm/s(34xxHz) right=8.xxcm/s(13xxHz)

如果 segment=2/4 的左右频率不同，但实车仍不转弯，再检查机械轮胎打滑或轮距参数。
