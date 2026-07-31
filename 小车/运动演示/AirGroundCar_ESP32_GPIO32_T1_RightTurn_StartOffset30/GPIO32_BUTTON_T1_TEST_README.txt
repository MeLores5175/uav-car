AirGroundCar ESP32 外接按键任务1实车测试版

功能：
1. 不连接 Wi-Fi，不接收地面站或无人机命令。
2. 上电后自动选择任务1并进入 ARMED 状态。
3. 按下 GPIO32 外接按键一次，开始运行完整任务1路线。
4. 按键带 50 ms 软件消抖，一次按下只触发一次。
5. 任务结束后不会再次启动；需要重新测试时按 ESP32 EN/RST 复位。

外接按键接线：
  GPIO32 ---- 常开按键 ---- GND

程序使用 INPUT_PULLUP：
- 松开：GPIO32 = HIGH
- 按下：GPIO32 = LOW
- 不需要外接上拉电阻

已标定参数：
- 左右轮每圈脉冲数：3200 pulse/rev
- STEP26/DIR27/EN25 通道：前进 DIR=HIGH
- STEP19/DIR21/EN18 通道：前进 DIR=LOW
- 软件最大脉冲频率：5000 Hz

使用：
1. 打开文件夹内 AirGroundCar_ESP32_GPIO32_Button_T1_Test.ino。
2. 烧录到 ESP32 Dev Module。
3. 打开串口监视器，115200 波特率。
4. 上电时保持按键松开。
5. 看到“task1 selected and armed”后按一下外接按键。

串口备用命令：
- S：不按按钮，直接从串口启动。
- R：任务未运行时重新选择并 ARM 任务1。
