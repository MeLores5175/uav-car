# Task 1 Independent Takeoff Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give `TAKEOFF` and `DROP_HOVER` independent horizontal/vertical control parameters and require a broad safety gate after the fixed three-second hover without restarting its timer.

**Architecture:** Extend the existing velocity-only point controller with an optional vertical proportional gain, then add one takeoff-specific wrapper used by both takeoff states. Move the inline `DROP_HOVER` behavior into a focused handler that keeps the existing timer monotonic and transitions only when the new broad exit thresholds are satisfied.

**Tech Stack:** Python 3, ROS1 `rospy`, MAVROS `PositionTarget`, YAML, `unittest`

## Global Constraints

- Modify only UAV-side files; never modify `AirGroundCar_ESP32_真车通信修改版/`.
- Horizontal takeoff profile: `Kp=0.80`, speed `0.35 m/s`, acceleration `0.35 m/s²`, jerk `0.60 m/s³`.
- Vertical takeoff profile: `Kp=0.90`, speed `0.55 m/s`, acceleration `0.75 m/s²`.
- Hover exit gate: XY error `0.30 m`, Z error `0.18 m`, measured 3D speed `0.35 m/s`.
- Do not reset or restart the fixed three-second hover timer.
- Do not change `WAIT_FCU`, the eight-second soft-timeout branch, PX4 parameters, moving-target control, dynamic landing, or real-car code.
- Do not add measured-overspeed warnings or active overspeed protection.
- The workspace is not a Git repository, so use verification checkpoints instead of commit steps.

---

### Task 1: Add independent takeoff parameters and controller wrapper

**Files:**
- Modify: `config/air_ground_mission.yaml:15-33`
- Modify: `scripts/air_ground_mission_fsm.py:1414-1450`
- Modify: `scripts/air_ground_mission_fsm.py:1558-1565`
- Modify: `tests/test_air_ground_mission_velocity_control.py`

**Interfaces:**
- Extends: `publish_point_control(..., vertical_kp=None)`
- Produces: `publish_takeoff_point_control(dt) -> tuple[float, float, float] | None`
- Consumes: all seven takeoff controller parameters from `self.takeoff_cfg`

- [ ] **Step 1: Add failing tests for the YAML values and takeoff wrapper**

Append:

```python
class TakeoffControlParameterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_fsm_module()

    def test_takeoff_yaml_contains_independent_balanced_xy_and_fast_z_values(self):
        config = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "air_ground_mission.yaml"
        ).read_text(encoding="utf-8")
        for line in (
            "horizontal_position_kp: 0.80",
            "max_horizontal_speed_mps: 0.35",
            "max_horizontal_accel_mps2: 0.35",
            "max_horizontal_jerk_mps3: 0.60",
            "vertical_position_kp: 0.90",
            "max_vertical_speed_mps: 0.55",
            "max_vertical_accel_mps2: 0.75",
            "hover_exit_xy_tolerance_m: 0.30",
            "hover_exit_z_tolerance_m: 0.18",
            "hover_exit_speed_tolerance_mps: 0.35",
        ):
            self.assertIn(line, config)

    def test_takeoff_wrapper_passes_only_takeoff_control_parameters(self):
        fsm = self.module.AirGroundMissionFSM.__new__(
            self.module.AirGroundMissionFSM
        )
        fsm.home_x = 1.0
        fsm.home_y = 2.0
        fsm.home_z = 3.0
        fsm.cruise_height = 1.2
        fsm.takeoff_cfg = {
            "horizontal_position_kp": 0.80,
            "max_horizontal_speed_mps": 0.35,
            "max_horizontal_accel_mps2": 0.35,
            "max_horizontal_jerk_mps3": 0.60,
            "vertical_position_kp": 0.90,
            "max_vertical_speed_mps": 0.55,
            "max_vertical_accel_mps2": 0.75,
        }
        calls = []

        def publish_spy(*args, **kwargs):
            calls.append((args, kwargs))
            return (0.1, 0.1, 0.1)

        fsm.publish_point_control = publish_spy
        result = fsm.publish_takeoff_point_control(0.02)

        self.assertEqual((0.1, 0.1, 0.1), result)
        args, kwargs = calls[0]
        self.assertEqual(
            (1.0, 2.0, 4.2, 0.02, 0.35, 0.35, 0.80),
            args,
        )
        self.assertEqual(0.60, kwargs["max_jerk"])
        self.assertEqual(0.55, kwargs["max_vz"])
        self.assertEqual(0.75, kwargs["max_az"])
        self.assertEqual(0.90, kwargs["vertical_kp"])

    def test_point_control_uses_explicit_vertical_kp_override(self):
        fsm = _make_control_fsm(self.module)
        fsm.current_pose.pose.position.z = 0.0
        fsm.current_vel = [0.0, 0.0, 0.0]
        fsm.cmd_xy = [0.0, 0.0]
        fsm.cmd_acc_xy = [0.0, 0.0]
        fsm.cmd_vz = 0.0
        fsm.publish_point_control(
            0.0,
            0.0,
            1.0,
            0.10,
            1.0,
            1.0,
            1.0,
            max_jerk=1.0,
            max_vz=1.0,
            max_az=10.0,
            vertical_kp=0.50,
        )
        self.assertAlmostEqual(
            0.50, fsm.raw_pub.messages[-1].velocity.z
        )
```

- [ ] **Step 2: Run the tests and verify the intended failures**

Run:

```powershell
python -m unittest tests.test_air_ground_mission_velocity_control.TakeoffControlParameterTest -v
```

Expected:

- YAML test fails because the independent keys are absent;
- wrapper test errors because `publish_takeoff_point_control` is absent;
- vertical override test errors because `publish_point_control` does not accept `vertical_kp`.

- [ ] **Step 3: Add the ten YAML values under `mission.takeoff`**

Insert after `soft_timeout_z_error_m`:

```yaml
    # 起飞和任务一固定悬停专用水平控制参数
    horizontal_position_kp: 0.80
    max_horizontal_speed_mps: 0.35
    max_horizontal_accel_mps2: 0.35
    max_horizontal_jerk_mps3: 0.60

    # 快速档垂直起飞控制参数
    vertical_position_kp: 0.90
    max_vertical_speed_mps: 0.55
    max_vertical_accel_mps2: 0.75

    # 固定悬停3秒完成后的宽松退出安全门槛
    hover_exit_xy_tolerance_m: 0.30
    hover_exit_z_tolerance_m: 0.18
    hover_exit_speed_tolerance_mps: 0.35
```

- [ ] **Step 4: Extend the point controller with a vertical gain override**

Change the signature to:

```python
    def publish_point_control(
        self, tx, ty, tz, dt, max_speed, max_accel, kp,
        max_jerk=None, max_vz=None, max_az=None, vertical_kp=None
    ):
```

Replace the current vertical gain assignment with:

```python
        z_kp = (
            safe_float(vertical_kp, 0.90)
            if vertical_kp is not None
            else safe_float(self.follow_cfg.get("vertical_kp", 0.90), 0.90)
        )
```

All existing callers omit the new final argument and therefore preserve their
current behavior.

- [ ] **Step 5: Add the takeoff-specific wrapper and use it in `handle_takeoff`**

Add immediately before `handle_takeoff()`:

```python
    def publish_takeoff_point_control(self, dt):
        target_z = self.home_z + self.cruise_height
        return self.publish_point_control(
            self.home_x,
            self.home_y,
            target_z,
            dt,
            safe_float(
                self.takeoff_cfg.get("max_horizontal_speed_mps", 0.35),
                0.35,
            ),
            safe_float(
                self.takeoff_cfg.get("max_horizontal_accel_mps2", 0.35),
                0.35,
            ),
            safe_float(
                self.takeoff_cfg.get("horizontal_position_kp", 0.80),
                0.80,
            ),
            max_jerk=safe_float(
                self.takeoff_cfg.get("max_horizontal_jerk_mps3", 0.60),
                0.60,
            ),
            max_vz=safe_float(
                self.takeoff_cfg.get("max_vertical_speed_mps", 0.55),
                0.55,
            ),
            max_az=safe_float(
                self.takeoff_cfg.get("max_vertical_accel_mps2", 0.75),
                0.75,
            ),
            vertical_kp=safe_float(
                self.takeoff_cfg.get("vertical_position_kp", 0.90),
                0.90,
            ),
        )
```

Replace the initial target/point-control block in `handle_takeoff()` with:

```python
        metrics = self.publish_takeoff_point_control(dt)
```

Leave `try_offboard_and_arm()`, the first stable gate, eight-second timeout,
soft-timeout acceptance, and emergency-land branch byte-for-byte unchanged.

- [ ] **Step 6: Run the focused tests**

Run:

```powershell
python -m unittest tests.test_air_ground_mission_velocity_control.TakeoffControlParameterTest -v
```

Expected: three tests pass.

### Task 2: Add the non-resetting hover exit safety gate

**Files:**
- Modify: `scripts/air_ground_mission_fsm.py:2637-2662`
- Modify: `tests/test_air_ground_mission_velocity_control.py`

**Interfaces:**
- Produces: `handle_drop_hover(dt) -> None`
- Consumes: `publish_takeoff_point_control(dt)` metrics `(xy_error, z_error, measured_3d_speed)`
- Consumes: `drop_hover_time_s` and the three `hover_exit_*` thresholds from `takeoff_cfg`

- [ ] **Step 1: Add failing hover-state tests**

Append:

```python
def _make_drop_hover_fsm(module, elapsed, metrics):
    fsm = module.AirGroundMissionFSM.__new__(module.AirGroundMissionFSM)
    fsm.takeoff_cfg = {
        "drop_hover_time_s": 3.0,
        "hover_exit_xy_tolerance_m": 0.30,
        "hover_exit_z_tolerance_m": 0.18,
        "hover_exit_speed_tolerance_mps": 0.35,
    }
    fsm.state_enter_time = object()
    fsm.state_elapsed = lambda: elapsed
    fsm.publish_takeoff_point_control = lambda _dt: metrics
    fsm.transitions = []
    fsm.enter_state = lambda state: fsm.transitions.append(state)
    return fsm


class DropHoverExitGateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_fsm_module()

    def test_hover_does_not_exit_before_three_seconds(self):
        fsm = _make_drop_hover_fsm(
            self.module, 2.99, (0.0, 0.0, 0.0)
        )
        fsm.handle_drop_hover(0.02)
        self.assertEqual([], fsm.transitions)

    def test_hover_waits_after_three_seconds_when_gate_is_unsafe(self):
        fsm = _make_drop_hover_fsm(
            self.module, 3.50, (0.31, 0.10, 0.20)
        )
        original_enter_time = fsm.state_enter_time
        fsm.handle_drop_hover(0.02)
        self.assertEqual([], fsm.transitions)
        self.assertIs(original_enter_time, fsm.state_enter_time)

    def test_hover_exits_immediately_when_gate_recovers(self):
        fsm = _make_drop_hover_fsm(
            self.module, 5.00, (0.20, 0.12, 0.25)
        )
        fsm.handle_drop_hover(0.02)
        self.assertEqual(["INTERCEPT"], fsm.transitions)
```

- [ ] **Step 2: Run the hover tests and verify they fail**

Run:

```powershell
python -m unittest tests.test_air_ground_mission_velocity_control.DropHoverExitGateTest -v
```

Expected: three tests error because `handle_drop_hover` does not exist.

- [ ] **Step 3: Implement the focused hover handler**

Add beside `handle_takeoff()`:

```python
    def handle_drop_hover(self, dt):
        metrics = self.publish_takeoff_point_control(dt)
        if metrics is None:
            return
        required_hover = safe_float(
            self.takeoff_cfg.get("drop_hover_time_s", 3.0), 3.0
        )
        if self.state_elapsed() < required_hover:
            return

        xy_err, z_err, speed = metrics
        exit_safe = (
            xy_err <= safe_float(
                self.takeoff_cfg.get(
                    "hover_exit_xy_tolerance_m", 0.30
                ),
                0.30,
            )
            and z_err <= safe_float(
                self.takeoff_cfg.get(
                    "hover_exit_z_tolerance_m", 0.18
                ),
                0.18,
            )
            and speed <= safe_float(
                self.takeoff_cfg.get(
                    "hover_exit_speed_tolerance_mps", 0.35
                ),
                0.35,
            )
        )
        if exit_safe:
            self.enter_state("INTERCEPT")
            return

        rospy.logwarn_throttle(
            1.0,
            "Task1 hover timer done; wait for safe intercept exit: "
            "xy_err=%.2f z_err=%.2f speed=%.2f",
            xy_err,
            z_err,
            speed,
        )
```

This function never assigns `state_enter_time` and never invokes a stable
timer, so the completed three-second timer remains completed while waiting.

- [ ] **Step 4: Route `DROP_HOVER` through the new handler**

Replace the inline `DROP_HOVER` block in `spin()` with:

```python
            elif self.fsm_state == "DROP_HOVER":
                self.handle_drop_hover(dt)
```

- [ ] **Step 5: Run the hover and takeoff tests**

Run:

```powershell
python -m unittest tests.test_air_ground_mission_velocity_control.TakeoffControlParameterTest tests.test_air_ground_mission_velocity_control.DropHoverExitGateTest -v
```

Expected: six tests pass.

### Task 3: Regression and scope verification

**Files:**
- Verify: `scripts/air_ground_mission_fsm.py`
- Verify: `config/air_ground_mission.yaml`
- Verify: `tests/test_air_ground_mission_velocity_control.py`
- Verify untouched: `AirGroundCar_ESP32_真车通信修改版/`

**Interfaces:**
- Consumes: completed Tasks 1-2
- Produces: verified UAV-only takeoff change

- [ ] **Step 1: Add source-level scope assertions**

Append:

```python
class TakeoffScopeRegressionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (
            Path(__file__).resolve().parents[1]
            / "scripts"
            / "air_ground_mission_fsm.py"
        ).read_text(encoding="utf-8")

    def test_drop_hover_spin_path_uses_focused_handler(self):
        self.assertIn(
            'elif self.fsm_state == "DROP_HOVER":\n'
            "                self.handle_drop_hover(dt)",
            self.source,
        )

    def test_existing_takeoff_soft_timeout_branch_remains(self):
        self.assertIn(
            'self.takeoff_cfg.get("max_wait_s", 8.0)', self.source
        )
        self.assertIn(
            'self.takeoff_cfg.get("soft_timeout_z_error_m", 0.22)',
            self.source,
        )
        self.assertIn(
            'self.request_emergency_land("takeoff timeout")',
            self.source,
        )

    def test_wait_fcu_still_uses_existing_position_velocity_prestream(self):
        self.assertIn(
            "self.publish_position_velocity_yaw(\n"
            "                    self.home_x, self.home_y, target_z,",
            self.source,
        )
```

- [ ] **Step 2: Run the complete unit suite**

Run:

```powershell
python -m unittest discover -s tests -v
```

Expected: all pre-existing 17 tests plus the 9 new takeoff tests pass.

- [ ] **Step 3: Run Python syntax checks**

Run:

```powershell
python -m py_compile scripts/air_ground_mission_fsm.py tests/test_air_ground_mission_velocity_control.py tests/test_platform_vision_cache.py
```

Expected: exit code 0 with no output.

- [ ] **Step 4: Parse and validate the YAML with the WSL PyYAML environment**

Run:

```powershell
wsl.exe -e python3 -c "import yaml; from pathlib import Path; cfg=yaml.safe_load(Path('/mnt/c/Users/kqjaj/Desktop/任务1-全真初版/config/air_ground_mission.yaml').read_text(encoding='utf-8')); t=cfg['mission']['takeoff']; expected={'horizontal_position_kp':0.80,'max_horizontal_speed_mps':0.35,'max_horizontal_accel_mps2':0.35,'max_horizontal_jerk_mps3':0.60,'vertical_position_kp':0.90,'max_vertical_speed_mps':0.55,'max_vertical_accel_mps2':0.75,'hover_exit_xy_tolerance_m':0.30,'hover_exit_z_tolerance_m':0.18,'hover_exit_speed_tolerance_mps':0.35}; assert all(float(t[k])==v for k,v in expected.items()); print('takeoff yaml validated')"
```

Expected:

```text
takeoff yaml validated
```

- [ ] **Step 5: Audit unchanged scope**

Run:

```powershell
$carTouched = @(
    Get-ChildItem -Recurse -File 'AirGroundCar_ESP32_真车通信修改版' |
        Where-Object { $_.LastWriteTime -ge [datetime]'2026-07-31T00:00:00' }
)
if ($carTouched.Count -ne 0) {
    $carTouched.FullName
    throw 'real-car files have current-date writes'
}
"real-car current-date writes: $($carTouched.Count)"
```

Expected:

```text
real-car current-date writes: 0
```

- [ ] **Step 6: Record the touched-file checkpoint**

Report:

```text
Changed:
- scripts/air_ground_mission_fsm.py
- config/air_ground_mission.yaml
- tests/test_air_ground_mission_velocity_control.py
- docs/superpowers/specs/2026-07-31-task1-takeoff-control-design.md
- docs/superpowers/plans/2026-07-31-task1-takeoff-control.md

Unchanged:
- AirGroundCar_ESP32_真车通信修改版/**
- WAIT_FCU control path
- takeoff soft-timeout conditions
- moving-target and dynamic-landing control algorithms
```

