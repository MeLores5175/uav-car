# 本版关键修正

1. `STEPPER_PULSES_PER_REV = 3200`。
2. 物理左轮：STEP26 / DIR27 / EN25，DIR=HIGH前进。
3. 物理右轮：STEP19 / DIR21 / EN18，DIR=LOW前进。
4. 左右STEP不再以相同参数挂接到LEDC：
   - 左轮：channel 0，初始997 Hz
   - 右轮：channel 2，初始1009 Hz
5. 这样可避免Arduino-ESP32 3.x自动让两路共享同一个LEDC timer，
   从而保证弯道时可输出不同频率。

上电串口应看到类似：
`[LEDC] independent attach check: left=997 Hz right=1009 Hz (channels 0/2)`

若这里左右读数相同，不要落地测试。
