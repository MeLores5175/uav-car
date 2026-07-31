# 陆空协同无人机系统 UDP 通信协议 V1.1

> 适用对象：HTML 地面站、无人机端常驻 BOOT/UDP 节点、无人机 ROS 任务节点、ESP32 小车端程序。  
> 设计目标：控制命令尽量短、便于人工调试；状态与位置数据使用 JSON，便于 HTML/JavaScript 解析；关键命令具备 ACK、重发和去重机制。
> 本次修订：在 `TEL:CAR` 中增加 `segment`（当前赛道区段）和 `path_s_cm`（从 A 点开始的累计行进里程），供无人机进行 C-D 段控制、区段进度计算和交叉校验。

---

## 1. 本版最终确定内容

1. 地面站可以选择 **任务 1** 或 **任务 2**。
2. 无人机和小车都采用任务 1、任务 2 两套行为模式。
3. 地面站选择任务后：
   - 向无人机发送对应任务的 `BOOT` 命令，由无人机常驻脚本启动相应 ROS 节点；
   - 向小车发送对应任务的 `MODE` 命令，使 ESP32 切换到对应行为模式。
4. 控制命令不使用大段 JSON，采用短文本格式。
5. 无人机和小车的位置、速度、状态等连续数据使用 JSON。
6. 正式任务中，地面站的 START 按钮只向小车发送启动命令；小车开始运动后，再可靠地通知无人机开始任务。
7. 地面站提供 `LAND` 安全命令，使无人机立即中止当前任务并在当前位置受控降落。

---

## 2. 网络结构

三台设备接入同一个不连接外网的局域网。

| 设备 | 示例 IP | UDP 监听端口 | 说明 |
|---|---|---:|---|
| 地面站 GS | `192.168.151.101` | `8889` | 接收无人机、小车状态与事件 |
| 无人机 UAV | `192.168.151.102` | `8888` | 接收 BOOT、PING、STATUS、LAND 等命令 |
| 小车 CAR | `192.168.151.103` | `8890` | 接收 MODE、PING、STATUS、START 等命令 |

以上地址均为建议默认值，最终必须写入独立配置文件，不应散落硬编码在多个程序中。

---

## 3. 报文分类

协议分为五类报文：

| 前缀 | 类型 | 是否需要 ACK | 用途 |
|---|---|---:|---|
| `CMD` | 控制命令 | 是 | BOOT、MODE、START、LAND、RESET、PING、STATUS |
| `ACK` | 成功确认 | 否 | 确认命令已收到或已执行 |
| `ERR` | 错误回复 | 否 | 命令格式错误、状态不允许、节点启动失败等 |
| `EVT` | 关键事件 | 关键事件建议确认 | 到达 B/D 点、任务开始、投放完成、降落完成等 |
| `HB` / `TEL` | 心跳与遥测 | 否 | 在线状态、位置、速度、电量、任务状态 |

控制报文保持短小；只有 `TEL` 状态遥测使用 JSON。

---

## 4. 控制命令格式

### 4.1 正式格式

```text
CMD:<cmd_id>:<action>[:arg1][:arg2]...
```

示例：

```text
CMD:0001:PING
CMD:0002:BOOT:T1
CMD:0003:MODE:T1
CMD:0004:START:R001
CMD:0005:LAND:R001
```

字段说明：

| 字段 | 说明 |
|---|---|
| `CMD` | 固定命令前缀 |
| `cmd_id` | 四位或更多位命令编号，用于 ACK 匹配、重发和去重 |
| `action` | 命令名称 |
| `arg` | 可选参数，例如任务号、运行编号 |

因此，最简单的命令确实可以写成 `CMD:START`，但正式版本建议使用：

```text
CMD:0042:START:R001
```

它只比 `CMD:START` 多几个字符，却能解决 UDP 重发时的重复执行问题。

### 4.2 兼容旧仓库格式

为了兼容原仓库已有代码，接收端在过渡阶段可以同时接受：

```text
CMD:START
CMD:LAND
CMD:PING
CMD:STATUS
```

正式比赛版本统一使用带 `cmd_id` 的 V1.1 格式。

---

## 5. ACK 与 ERR 格式

### 5.1 成功确认

```text
ACK:<cmd_id>:<action>:<result>[:detail]
```

示例：

```text
ACK:0001:PING:OK:UAV
ACK:0002:BOOT:ACCEPTED:T1
ACK:0003:MODE:OK:T1
ACK:0004:START:OK:R001
ACK:0005:LAND:ACCEPTED:R001
```

`ACCEPTED` 表示命令已经接收，动作仍在进行；`OK` 表示动作或设置已经完成。

### 5.2 错误回复

```text
ERR:<cmd_id>:<action>:<error_code>[:detail]
```

示例：

```text
ERR:0002:BOOT:ALREADY_RUNNING
ERR:0003:MODE:BAD_TASK
ERR:0004:START:NOT_READY
ERR:0005:LAND:FCU_DISCONNECTED
```

建议错误码：

| 错误码 | 含义 |
|---|---|
| `BAD_FORMAT` | 报文格式错误 |
| `UNKNOWN_CMD` | 不支持该命令 |
| `BAD_TASK` | 任务号不是 T1 或 T2 |
| `NOT_READY` | 当前设备未准备完成 |
| `BUSY` | 正在执行其他动作 |
| `ALREADY_RUNNING` | 任务已经运行 |
| `FCU_DISCONNECTED` | 无人机飞控未连接 |
| `ROS_START_FAILED` | ROS 节点启动失败 |
| `MODE_MISMATCH` | 小车和无人机任务模式不一致 |

---

## 6. 地面站发送给无人机的命令

| 命令 | 示例 | 作用 |
|---|---|---|
| `PING` | `CMD:0001:PING` | 检查无人机常驻 UDP/BOOT 节点是否在线 |
| `STATUS` | `CMD:0002:STATUS` | 请求无人机立即返回一次完整状态 |
| `BOOT` | `CMD:0003:BOOT:T1` | 启动任务 1 对应 ROS 节点 |
| `BOOT` | `CMD:0004:BOOT:T2` | 启动任务 2 对应 ROS 节点 |
| `LAND` | `CMD:0005:LAND:R001` | 中止任务并在当前位置受控降落 |
| `RESET` | `CMD:0006:RESET` | 无人机落地后复位任务状态 |
| `SERVO` | `CMD:0007:SERVO:LOCK` / `CMD:0008:SERVO:RELEASE` | 仅起飞前锁止 A30 或释放 A90 |
| `STOP_NODES` | `CMD:0007:STOP_NODES` | 调试时停止任务 ROS 节点，不用于飞行中 |

### 6.1 BOOT 命令语义

无人机开机后先运行一个常驻脚本，例如：

```text
uav_supervisor.py
```

收到：

```text
CMD:0003:BOOT:T1
```

后执行：

1. 记录当前任务模式为 T1；
2. 启动任务 1 所需的 ROS 节点；
3. 检查 ROS Master、MAVROS、飞控连接、视觉节点和任务状态机；
4. 立即回复：

```text
ACK:0003:BOOT:ACCEPTED:T1
```

5. 准备完成后主动上报：

```text
EVT:UAV_BOOT_READY:T1
```

若启动失败：

```text
ERR:0003:BOOT:ROS_START_FAILED:vision_node
```

`BOOT` 只启动程序和完成自检，不允许直接起飞。

### 6.2 LAND 命令语义

```text
CMD:0005:LAND:R001
```

定义为：

> 立即锁存安全中止状态，停止当前任务逻辑，取消投放、伴飞、返航或再次起飞等动作，在飞控可控的前提下保持当前位置并受控下降，落地后解锁。

无人机收到后必须首先回复：

```text
ACK:0005:LAND:ACCEPTED:R001
```

随后依次上报：

```text
EVT:UAV_ABORTING:R001
EVT:UAV_LANDING:R001
EVT:UAV_LANDED:R001
```

若无人机已经稳定停在小车平台上，收到 `LAND` 后应保持落地并禁止再次起飞。

`LAND` 不是直接停桨命令，不设置普通网页 `KILL` 按钮。

---

### 6.3 起飞前舵机命令

```text
CMD:<cmd_id>:SERVO:LOCK
CMD:<cmd_id>:SERVO:RELEASE
```

仅允许地面站在无人机未解锁且 FSM 为 `WAIT_START` 时使用。`LOCK` 对应串口 `A30`，`RELEASE` 对应串口 `A90`。任务开始后，投放只能由 FSM 自动触发。

成功回复：

```text
ACK:<cmd_id>:SERVO:OK:LOCK
ACK:<cmd_id>:SERVO:OK:RELEASE
```

## 7. 地面站发送给小车的命令

| 命令 | 示例 | 作用 |
|---|---|---|
| `PING` | `CMD:0101:PING` | 检查 ESP32 UDP 通信是否在线 |
| `STATUS` | `CMD:0102:STATUS` | 请求小车立即返回完整状态 |
| `MODE` | `CMD:0103:MODE:T1` | 切换为任务 1 行为模式 |
| `MODE` | `CMD:0104:MODE:T2` | 切换为任务 2 行为模式 |
| `START` | `CMD:0105:START:R001` | 启动本轮循线任务 |
| `RESET` | `CMD:0106:RESET` | 小车复位到等待状态 |

小车任务模式由 `MODE:T1` 或 `MODE:T2` 决定。两种模式的具体速度曲线、转弯控制和平台配合行为由小车程序实现，通信协议只负责传递模式。

---

## 8. 正式 START 流程

为了满足“小车启动后通知无人机起飞”的协同关系，正式模式采用以下流程：

```text
地面站选择任务 T1 或 T2
        │
        ├── GS → UAV：CMD:<id>:BOOT:T1/T2
        └── GS → CAR：CMD:<id>:MODE:T1/T2
                    │
等待 UAV_BOOT_READY 和 CAR_READY
                    │
地面站点击 START
                    │
        GS → CAR：CMD:<id>:START:R001
                    │
小车开始循线，并向无人机发送：
        CAR → UAV：CMD:<id>:START:R001:T1/T2
                    │
无人机回复：
        UAV → CAR：ACK:<id>:START:OK:R001
                    │
小车向地面站报告：
        CAR → GS：EVT:MISSION_START:R001:T1/T2
```

这样地面站界面仍然只有一个 START 按钮，但无人机的正式起飞触发来自小车。

### 8.1 调试模式

调试时允许地面站直接向无人机发送：

```text
CMD:<id>:START:R001:T1
```

但正式比赛界面应隐藏或禁用“直接启动无人机”功能。

---

## 9. 小车发送给无人机的关键命令

### 9.1 任务启动

```text
CMD:1001:START:R001:T1
```

或：

```text
CMD:1001:START:R001:T2
```

无人机回复：

```text
ACK:1001:START:OK:R001
```

小车未收到 ACK 时，按可靠性规则重发。

### 9.2 小车关键点事件

小车经过关键点时，可同时向无人机和地面站发送：

```text
EVT:CAR_POINT:R001:B
EVT:CAR_POINT:R001:C
EVT:CAR_POINT:R001:D
EVT:CAR_POINT:R001:A_FINISH
```

这类事件用于地面站显示和判断：

- 无人机是否在 B 点前建立伴飞；
- 投放是否在 D 点前完成；
- 动态降落是否在 D 点前完成。

---

## 10. 心跳格式

无人机和小车每 1 秒发送一次短心跳：

```text
HB:UAV:<seq>:<state>
HB:CAR:<seq>:<state>
```

示例：

```text
HB:UAV:152:READY
HB:CAR:086:READY
HB:UAV:153:FOLLOW
HB:CAR:087:RUNNING
```

地面站判断建议：

| 未收到数据时间 | 显示状态 |
|---:|---|
| 小于 1.5 秒 | 在线，绿色 |
| 1.5～3 秒 | 通信延迟，黄色 |
| 大于 3 秒 | 离线，红色 |

`PING` 用于人工检查，`HB` 用于持续在线判断。

---

## 11. 状态与遥测格式

位置、速度、电量和详细状态允许使用较长的 JSON 报文。

### 11.1 无人机遥测

格式：

```text
TEL:UAV:{json}
```

示例：

```json
TEL:UAV:{"seq":520,"time_ms":18430,"run":"R001","task":1,"boot":"READY","state":"FOLLOW","safety":"NORMAL","armed":true,"fcu":true,"mode":"OFFBOARD","x_cm":183.5,"y_cm":276.2,"z_cm":149.6,"vx_cm_s":26.4,"vy_cm_s":1.8,"vz_cm_s":0.3,"yaw_deg":89.4,"battery":78,"target_locked":true,"error":0}
```

建议发送频率：

- 位置、速度：5～10 Hz；
- 电量、完整状态：1～2 Hz；
- 状态发生改变时立即发送事件。

### 11.2 小车遥测

格式：

```text
TEL:CAR:{json}
```

示例：

```json
TEL:CAR:{"seq":310,"time_ms":18390,"run":"R001","task":2,"state":"RUNNING","x_cm":318.5,"y_cm":305.2,"speed_cm_s":12.0,"yaw_deg":270.0,"point":"C","segment":"CD","path_s_cm":512.4,"line_detected":true,"battery":82,"error":0}
```

新增字段定义：

| 字段 | 类型 | 单位/取值 | 是否必需 | 含义 |
|---|---|---|---:|---|
| `segment` | string | `AB`、`BC`、`CD`、`DA`、`UNKNOWN` | 是 | 小车当前连续所在的赛道区段 |
| `path_s_cm` | number | cm，`>=0` | 是 | 本轮任务中从 A 点开始沿黑线顺时针累计行进的里程 |

`segment` 约定：

- `AB`：A 点到 B 点；
- `BC`：B 点到 C 点；
- `CD`：C 点到 D 点；
- `DA`：D 点到 A 点；
- `UNKNOWN`：当前无法可靠判断区段。无人机收到该值时不得开始动态下降。

`path_s_cm` 约定：

1. 每轮任务开始时清零；
2. 从 A 点开始沿顺时针轨迹单调增加；
3. 单位为厘米，不在一圈中途回零；
4. 无人机可根据赛道几何模型由该字段计算 `segment_progress` 和 `lap_progress`；
5. 无人机应将由 `path_s_cm` 计算出的区段与小车上报的 `segment` 进行交叉校验。若两者明显不一致，应将区段视为 `UNKNOWN`，继续伴飞但禁止进入动态下降。

`point` 与 `segment` 的含义不同：

- `point` 表示最近经过或当前触发的关键点，主要用于地面站显示和事件记录；
- `segment` 表示当前持续所在路段，主要用于无人机实时控制。

例如：

```json
{"point":"C","segment":"CD"}
```

表示小车已经经过 C 点，目前正在 C-D 段运行。

建议小车以 20～50 Hz 向无人机发送 `TEL:CAR`；地面站显示可以使用相同报文或较低发送频率。

### 11.3 统一单位与坐标系

| 物理量 | 单位 |
|---|---|
| 场地位置 `x_cm`、`y_cm` | cm |
| 小车累计里程 `path_s_cm` | cm |
| 无人机高度 `z_cm` | cm |
| 速度 | cm/s |
| 航向角 `yaw_deg` | degree，范围建议 `0～360` |
| 时间 `time_ms` | 本轮任务开始后的毫秒数 |
| 电量 `battery` | 0～100 的百分数 |

场地坐标统一定义为：

- 场地左下角为 `(0, 0)`；
- 水平方向向右为 X 正方向，范围 `0～400 cm`；
- 竖直方向向上为 Y 正方向，范围 `0～500 cm`。

`TEL:UAV` 和 `TEL:CAR` 中的 `x_cm`、`y_cm` 必须使用上述场地坐标。无人机端必须在发送 `TEL:UAV` 前，将 MAVROS/Faster-LIO 本地坐标转换为场地坐标，不能直接把原始本地坐标作为 `x_cm`、`y_cm` 发送。协议不要求每包重复增加 `frame` 或 `origin` 字段。

---

## 12. 状态名称建议

### 12.1 无人机 BOOT 状态

```text
STOPPED
STARTING
READY
FAILED
```

### 12.2 无人机任务 1 状态

```text
WAIT_START
TAKEOFF
HOVER
SEARCH_CAR
FOLLOW
PREPARE_DROP
DROPPING
DROP_DONE
RETURN_HOME
LAND_HOME
DONE
```

### 12.3 无人机任务 2 状态

```text
WAIT_START
TAKEOFF
SEARCH_CAR
APPROACH_CAR
LAND_ON_CAR
ON_CAR
TAKEOFF_FROM_CAR
RETURN_HOME
LAND_HOME
DONE
```

### 12.4 无人机安全状态

```text
NORMAL
WARNING
ABORTING
LANDING
LANDED
FAULT
```

### 12.5 小车状态

```text
IDLE
READY
RUNNING
PASS_B
PASS_C
PASS_D
RETURN_A
FINISHED
LINE_LOST
FAULT
```

---

## 13. 关键事件格式

关键事件采用短文本，不必使用 JSON。

```text
EVT:<event>[:arg1][:arg2]...
```

推荐事件：

```text
EVT:UAV_BOOT_READY:T1
EVT:CAR_READY:T1
EVT:MISSION_START:R001:T1
EVT:CAR_POINT:R001:B
EVT:CAR_POINT:R001:D
EVT:UAV_FOLLOW_ESTABLISHED:R001
EVT:UAV_DROP_DONE:R001
EVT:UAV_LAND_ON_CAR:R001
EVT:UAV_TAKEOFF_FROM_CAR:R001
EVT:UAV_LANDING:R001
EVT:UAV_LANDED:R001
EVT:MISSION_DONE:R001
```

关键事件建议连续发送 2～3 次，或增加事件 ACK。

---

## 14. UDP 可靠性规则

UDP 本身不保证到达，因此规定如下。

### 14.1 必须确认的命令

```text
BOOT
MODE
START
LAND
RESET
STOP_NODES
```

### 14.2 重发参数建议

普通关键命令：

```text
等待 ACK：300 ms
最大重发：3 次
重发间隔：300 ms
```

安全 LAND 命令：

```text
等待 ACK：150 ms
最大重发：5 次
重发间隔：150 ms
```

### 14.3 去重

每个接收端至少缓存最近 32 条：

```text
发送源 IP + cmd_id + action
```

收到重复命令时：

1. 不再次执行动作；
2. 重新发送原 ACK。

这样即使 `START` 或 `LAND` 因为 ACK 丢失被重发，也不会重复触发。

### 14.4 遥测数据

`HB` 和 `TEL` 不需要 ACK。新数据直接覆盖旧数据，丢失一两帧不影响任务执行。

---

## 15. 地面站界面操作流程

### 15.1 任务准备

1. 选择任务 1 或任务 2；
2. 地面站向无人机发送 `BOOT:T1/T2`；
3. 地面站向小车发送 `MODE:T1/T2`；
4. 等待无人机 `UAV_BOOT_READY`；
5. 等待小车 `CAR_READY`；
6. 分别发送 `PING` 或 `STATUS`；
7. 两端均正常后启用 START 按钮。

### 15.2 任务运行

1. 点击 START；
2. 地面站向小车发送 `START:Rxxx`；
3. 小车开始循线并通知无人机；
4. 地面站根据 `EVT:MISSION_START` 开始计时；
5. 无人机和小车持续发送 `HB`、`TEL` 和关键事件。

### 15.3 危险处理

红色按钮文字建议为：

```text
中止任务并原地降落
```

按钮按下后：

```text
GS → UAV：CMD:<id>:LAND:<run_id>
```

界面锁定其他任务控制按钮，直到收到：

```text
EVT:UAV_LANDED:<run_id>
```

为防止误触，可采用按住 1 秒确认，但不能设置过多弹窗影响紧急操作。

---

## 16. 一次任务 1 的完整示例

```text
GS  → UAV  CMD:0001:BOOT:T1
UAV → GS   ACK:0001:BOOT:ACCEPTED:T1

GS  → CAR  CMD:0002:MODE:T1
CAR → GS   ACK:0002:MODE:OK:T1

UAV → GS   EVT:UAV_BOOT_READY:T1
CAR → GS   EVT:CAR_READY:T1

GS  → UAV  CMD:0003:PING
UAV → GS   ACK:0003:PING:OK:UAV

GS  → CAR  CMD:0004:PING
CAR → GS   ACK:0004:PING:OK:CAR

GS  → CAR  CMD:0005:START:R001
CAR → GS   ACK:0005:START:OK:R001

CAR → UAV  CMD:1001:START:R001:T1
UAV → CAR  ACK:1001:START:OK:R001
CAR → GS   EVT:MISSION_START:R001:T1

CAR → GS   EVT:CAR_POINT:R001:B
UAV → GS   EVT:UAV_FOLLOW_ESTABLISHED:R001
UAV → GS   EVT:UAV_DROP_DONE:R001
CAR → GS   EVT:CAR_POINT:R001:D
UAV → GS   EVT:UAV_LANDED:R001
CAR → GS   EVT:CAR_POINT:R001:A_FINISH
CAR → GS   EVT:MISSION_DONE:R001
```

---

## 17. 一次任务 2 的完整示例

```text
GS  → UAV  CMD:0101:BOOT:T2
UAV → GS   ACK:0101:BOOT:ACCEPTED:T2

GS  → CAR  CMD:0102:MODE:T2
CAR → GS   ACK:0102:MODE:OK:T2

UAV → GS   EVT:UAV_BOOT_READY:T2
CAR → GS   EVT:CAR_READY:T2

GS  → CAR  CMD:0103:START:R002
CAR → GS   ACK:0103:START:OK:R002

CAR → UAV  CMD:1101:START:R002:T2
UAV → CAR  ACK:1101:START:OK:R002
CAR → GS   EVT:MISSION_START:R002:T2

CAR → GS   EVT:CAR_POINT:R002:B
UAV → GS   EVT:UAV_LAND_ON_CAR:R002
UAV → GS   EVT:UAV_TAKEOFF_FROM_CAR:R002
UAV → GS   EVT:UAV_LANDED:R002
CAR → GS   EVT:CAR_POINT:R002:A_FINISH
CAR → GS   EVT:MISSION_DONE:R002
```

---

## 18. 兼容现有仓库的改造建议

参考仓库当前使用的可读协议包括：

```text
CMD:START
CMD:LAND
CMD:PING
CMD:STATUS
ACK:<CMD>:...
ERR:<CMD>:...
STATUS:...
VISION:IMAGE:{json}
```

V1.1 保留 `CMD / ACK / ERR / STATUS / VISION` 的整体思路，只做以下扩展：

1. 在控制命令中增加短 `cmd_id`；
2. 增加小车设备和 `MODE:T1/T2`；
3. 增加 `run_id`，区分每一轮测试；
4. 增加 `HB`、`TEL` 和 `EVT`；
5. 明确正式 START 由小车转发给无人机；
6. 明确 `LAND` 为受控安全降落，不等于直接停桨。

建议接收程序先同时兼容旧格式和 V1.1，等地面站、小车和无人机全部迁移完成后，再关闭旧格式。

对于旧版不含 `segment` 或 `path_s_cm` 的 `TEL:CAR`：

- 可以用于通信调试、位置显示和普通伴飞；
- 不得用于任务二的动态下降判定；
- 正式比赛版本中，任务二必须同时收到有效 `segment` 和 `path_s_cm`。

---

## 19. 最小可用命令集合

第一版开发只需先实现以下命令：

### 地面站到无人机

```text
CMD:<id>:PING
CMD:<id>:STATUS
CMD:<id>:BOOT:T1
CMD:<id>:BOOT:T2
CMD:<id>:LAND:<run_id>
CMD:<id>:RESET
```

### 地面站到小车

```text
CMD:<id>:PING
CMD:<id>:STATUS
CMD:<id>:MODE:T1
CMD:<id>:MODE:T2
CMD:<id>:START:<run_id>
CMD:<id>:RESET
```

### 小车到无人机

```text
CMD:<id>:START:<run_id>:T1
CMD:<id>:START:<run_id>:T2
```

### 无人机、小车到地面站

```text
HB:...
TEL:...:{json}
EVT:...
ACK:...
ERR:...
```

这套最小集合已经可以支持：任务选择、无人机节点启动、小车模式切换、通信检查、一键 START、状态显示和危险 LAND。

---

## 20. 协议版本

状态 JSON 中可增加：

```json
"proto": "1.1"
```

设备收到无法支持的协议或命令时返回：

```text
ERR:<cmd_id>:<action>:UNSUPPORTED_VERSION
```

后续增加字段时，应允许接收端忽略不认识的 JSON 字段，不应因新增字段导致旧程序崩溃。
