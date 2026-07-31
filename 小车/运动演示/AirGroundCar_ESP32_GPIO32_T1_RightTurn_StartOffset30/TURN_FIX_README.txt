本版本修正内容：
1. 将 STEP19/DIR21/EN18 归为物理左轮，前进 DIR=LOW。
2. 将 STEP26/DIR27/EN25 归为物理右轮，前进 DIR=HIGH。
3. 保持 3200 pulse/rev、5000 Hz、GPIO32 按键启动任务1。
4. 串口每500ms输出 [MOTION]，进入弯道后 left/right 应不同。

任务1路线：
AB直线150cm -> BC半圆(R=75cm) -> CD直线150cm -> DA半圆(R=75cm)。
第一次弯道约在启动后10秒、前进150cm时开始。
