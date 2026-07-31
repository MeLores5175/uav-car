# Task 1 UAV ROS Feedforward+PD Velocity Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the task-1 drop mission compute and bound the complete XY velocity setpoint in ROS using car-velocity feedforward plus a PD position outer loop, while preserving dynamic-landing-specific control behavior.

**Architecture:** Add strict loading for four drop-mission control profiles, isolate the controller mathematics in pure functions, and route the drop mission through a velocity-only MAVROS setpoint path. Move the current position+velocity behavior into a dedicated dynamic-landing function with no runtime mode switch.

**Tech Stack:** Python 3, ROS1 `rospy`, MAVROS `PositionTarget`, YAML, `unittest`

## Global Constraints

- Modify only UAV-side files; do not modify `AirGroundCar_ESP32_真车通信修改版/`.
- Do not add an integral term to the ROS position outer loop.
- Do not modify PX4 parameters.
- Preserve takeoff, post-drop braking/climb, return-home, landing, estimator, vision, drop-window, and actuator-ACK behavior.
- Apply the new controller only to `INTERCEPT`, `FOLLOW_DROP`, `DROP_DESCENT`, `DROP_ALIGN`, and `DROP_WAIT_ACK`.
- Preserve the current control behavior of `FOLLOW_CD`, `DYNAMIC_DESCENT`, `PLATFORM_DISARM`, and `PLATFORM_TAKEOFF`.
- The four `mission.follow.velocity_profiles` groups and all required fields are mandatory; invalid configuration must abort node initialization.
- The workspace is not a Git repository, so commit steps are replaced by explicit verification checkpoints.

---

### Task 1: Strict profile loading and configuration

**Files:**
- Modify: `scripts/air_ground_mission_fsm.py:32-75`
- Modify: `scripts/air_ground_mission_fsm.py:161-170`
- Modify: `config/air_ground_mission.yaml:43-87`
- Modify: `config/air_ground_mission.yaml:136-149`
- Create: `tests/test_air_ground_mission_velocity_control.py`

**Interfaces:**
- Produces: `load_follow_velocity_profiles(follow_cfg: dict) -> dict[str, dict[str, float]]`
- Produces: `FOLLOW_VELOCITY_PROFILE_NAMES`
- Produces: `FOLLOW_VELOCITY_PROFILE_FIELDS`
- Consumes: `mission.follow.velocity_profiles` from YAML

- [ ] **Step 1: Create a ROS-stubbed test loader and failing validation tests**

Create `tests/test_air_ground_mission_velocity_control.py` with these imports, stubs, loader, fixture, and tests:

```python
import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path


def _install_module(name, **attributes):
    module = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


class _PositionTarget:
    FRAME_LOCAL_NED = 1
    IGNORE_PX = 1
    IGNORE_PY = 2
    IGNORE_PZ = 4
    IGNORE_VX = 8
    IGNORE_VY = 16
    IGNORE_VZ = 32
    IGNORE_AFX = 64
    IGNORE_AFY = 128
    IGNORE_AFZ = 256
    FORCE = 512
    IGNORE_YAW = 1024
    IGNORE_YAW_RATE = 2048

    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None)
        self.coordinate_frame = None
        self.position = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.velocity = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.yaw = 0.0


class _RosTime:
    @classmethod
    def now(cls):
        return types.SimpleNamespace(to_sec=lambda: 10.0)


def _load_fsm_module():
    rospy = _install_module(
        "rospy",
        Time=_RosTime,
        logfatal=lambda *_args, **_kwargs: None,
        logwarn_throttle=lambda *_args, **_kwargs: None,
    )
    geometry_msg = _install_module(
        "geometry_msgs.msg",
        PoseStamped=type("PoseStamped", (), {}),
        TwistStamped=type("TwistStamped", (), {}),
    )
    _install_module("geometry_msgs", msg=geometry_msg)
    sensor_msg = _install_module("sensor_msgs.msg", Range=type("Range", (), {}))
    _install_module("sensor_msgs", msg=sensor_msg)
    std_msg = _install_module(
        "std_msgs.msg",
        Bool=type("Bool", (), {}),
        String=type("String", (), {}),
    )
    _install_module("std_msgs", msg=std_msg)
    mavros_msg = _install_module(
        "mavros_msgs.msg",
        State=type("State", (), {}),
        ExtendedState=type("ExtendedState", (), {}),
        PositionTarget=_PositionTarget,
    )
    _install_module("mavros_msgs", msg=mavros_msg)
    mavros_srv = _install_module(
        "mavros_msgs.srv",
        CommandBool=type("CommandBool", (), {}),
        SetMode=type("SetMode", (), {}),
    )
    sys.modules["mavros_msgs.srv"] = mavros_srv
    transformations = _install_module(
        "tf.transformations",
        euler_from_quaternion=lambda _value: (0.0, 0.0, 0.0),
    )
    _install_module("tf", transformations=transformations)

    source = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "air_ground_mission_fsm.py"
    )
    spec = importlib.util.spec_from_file_location(
        "air_ground_mission_fsm_under_test", source
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _valid_profiles():
    return {
        "intercept": {
            "velocity_feedforward_gain": 1.00,
            "position_kp": 0.95,
            "relative_velocity_kd": 0.30,
            "max_horizontal_speed_mps": 0.75,
            "max_correction_speed_mps": 0.35,
            "max_horizontal_accel_mps2": 0.55,
            "max_horizontal_jerk_mps3": 0.90,
        },
        "follow": {
            "velocity_feedforward_gain": 1.00,
            "position_kp": 0.75,
            "relative_velocity_kd": 0.25,
            "max_horizontal_speed_mps": 0.75,
            "max_correction_speed_mps": 0.25,
            "max_horizontal_accel_mps2": 0.45,
            "max_horizontal_jerk_mps3": 0.75,
        },
        "drop_descent": {
            "velocity_feedforward_gain": 1.00,
            "position_kp": 0.65,
            "relative_velocity_kd": 0.25,
            "max_horizontal_speed_mps": 0.75,
            "max_correction_speed_mps": 0.18,
            "max_horizontal_accel_mps2": 0.30,
            "max_horizontal_jerk_mps3": 0.55,
        },
        "drop_align": {
            "velocity_feedforward_gain": 1.00,
            "position_kp": 0.55,
            "relative_velocity_kd": 0.30,
            "max_horizontal_speed_mps": 0.75,
            "max_correction_speed_mps": 0.12,
            "max_horizontal_accel_mps2": 0.22,
            "max_horizontal_jerk_mps3": 0.40,
        },
    }


class FollowVelocityProfileValidationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_fsm_module()

    def test_valid_profiles_are_converted_to_floats(self):
        loaded = self.module.load_follow_velocity_profiles(
            {"velocity_profiles": _valid_profiles()}
        )
        self.assertEqual(set(_valid_profiles()), set(loaded))
        self.assertTrue(
            all(
                isinstance(value, float)
                for profile in loaded.values()
                for value in profile.values()
            )
        )

    def test_missing_profile_is_rejected(self):
        profiles = _valid_profiles()
        del profiles["drop_align"]
        with self.assertRaisesRegex(ValueError, "drop_align"):
            self.module.load_follow_velocity_profiles(
                {"velocity_profiles": profiles}
            )

    def test_non_finite_field_is_rejected(self):
        profiles = _valid_profiles()
        profiles["follow"]["position_kp"] = math.nan
        with self.assertRaisesRegex(ValueError, "follow.position_kp"):
            self.module.load_follow_velocity_profiles(
                {"velocity_profiles": profiles}
            )

    def test_non_positive_limit_is_rejected(self):
        profiles = _valid_profiles()
        profiles["intercept"]["max_horizontal_speed_mps"] = 0.0
        with self.assertRaisesRegex(ValueError, "max_horizontal_speed_mps"):
            self.module.load_follow_velocity_profiles(
                {"velocity_profiles": profiles}
            )

    def test_correction_limit_cannot_exceed_total_limit(self):
        profiles = _valid_profiles()
        profiles["follow"]["max_correction_speed_mps"] = 0.80
        with self.assertRaisesRegex(ValueError, "max_correction_speed_mps"):
            self.module.load_follow_velocity_profiles(
                {"velocity_profiles": profiles}
            )
```

- [ ] **Step 2: Run the tests and verify they fail**

Run:

```powershell
python -m unittest tests.test_air_ground_mission_velocity_control.FollowVelocityProfileValidationTest -v
```

Expected: all five tests error or fail because `load_follow_velocity_profiles` does not exist.

- [ ] **Step 3: Add strict profile constants and loader**

Add after `safe_int()` in `scripts/air_ground_mission_fsm.py`:

```python
FOLLOW_VELOCITY_PROFILE_NAMES = (
    "intercept",
    "follow",
    "drop_descent",
    "drop_align",
)

FOLLOW_VELOCITY_PROFILE_FIELDS = (
    "velocity_feedforward_gain",
    "position_kp",
    "relative_velocity_kd",
    "max_horizontal_speed_mps",
    "max_correction_speed_mps",
    "max_horizontal_accel_mps2",
    "max_horizontal_jerk_mps3",
)


def load_follow_velocity_profiles(follow_cfg):
    profiles_cfg = follow_cfg.get("velocity_profiles")
    if not isinstance(profiles_cfg, dict):
        raise ValueError("mission.follow.velocity_profiles must be a mapping")

    profiles = {}
    for profile_name in FOLLOW_VELOCITY_PROFILE_NAMES:
        source = profiles_cfg.get(profile_name)
        if not isinstance(source, dict):
            raise ValueError(
                "mission.follow.velocity_profiles.%s must be a mapping"
                % profile_name
            )

        profile = {}
        for field_name in FOLLOW_VELOCITY_PROFILE_FIELDS:
            if field_name not in source:
                raise ValueError(
                    "missing mission.follow.velocity_profiles.%s.%s"
                    % (profile_name, field_name)
                )
            value = source[field_name]
            if isinstance(value, bool):
                raise ValueError(
                    "%s.%s must be a finite number"
                    % (profile_name, field_name)
                )
            try:
                number = float(value)
            except (TypeError, ValueError):
                raise ValueError(
                    "%s.%s must be a finite number"
                    % (profile_name, field_name)
                )
            if not math.isfinite(number):
                raise ValueError(
                    "%s.%s must be a finite number"
                    % (profile_name, field_name)
                )
            profile[field_name] = number

        for field_name in (
            "velocity_feedforward_gain",
            "position_kp",
            "relative_velocity_kd",
        ):
            if profile[field_name] < 0.0:
                raise ValueError(
                    "%s.%s must be non-negative"
                    % (profile_name, field_name)
                )

        for field_name in (
            "max_horizontal_speed_mps",
            "max_correction_speed_mps",
            "max_horizontal_accel_mps2",
            "max_horizontal_jerk_mps3",
        ):
            if profile[field_name] <= 0.0:
                raise ValueError(
                    "%s.%s must be greater than zero"
                    % (profile_name, field_name)
                )

        if (
            profile["max_correction_speed_mps"]
            > profile["max_horizontal_speed_mps"]
        ):
            raise ValueError(
                "%s.max_correction_speed_mps cannot exceed "
                "max_horizontal_speed_mps" % profile_name
            )
        profiles[profile_name] = profile

    return profiles
```

In `AirGroundMissionFSM.__init__`, immediately after assigning `self.follow_cfg`, add:

```python
        try:
            self.follow_velocity_profiles = load_follow_velocity_profiles(
                self.follow_cfg
            )
        except ValueError as exc:
            rospy.logfatal("Invalid follow velocity profiles: %s", str(exc))
            raise
```

- [ ] **Step 4: Replace the YAML runtime-mode settings with four mandatory profiles**

Remove `velocity_setpoint_mode`, the global `position_kp`,
`relative_velocity_kd`, and `drop.descent_horizontal_speed_scale`.
Keep the global feedforward/speed/acceleration/jerk values because takeoff
and dynamic-landing-specific control still use them. Add under `mission.follow`:

```yaml
    # 投掷任务移动目标控制：ROS计算完整XY速度，只向PX4发布XY速度设定值。
    velocity_profiles:
      intercept:
        velocity_feedforward_gain: 1.00
        position_kp: 0.95
        relative_velocity_kd: 0.30
        max_horizontal_speed_mps: 0.75
        max_correction_speed_mps: 0.35
        max_horizontal_accel_mps2: 0.55
        max_horizontal_jerk_mps3: 0.90
      follow:
        velocity_feedforward_gain: 1.00
        position_kp: 0.75
        relative_velocity_kd: 0.25
        max_horizontal_speed_mps: 0.75
        max_correction_speed_mps: 0.25
        max_horizontal_accel_mps2: 0.45
        max_horizontal_jerk_mps3: 0.75
      drop_descent:
        velocity_feedforward_gain: 1.00
        position_kp: 0.65
        relative_velocity_kd: 0.25
        max_horizontal_speed_mps: 0.75
        max_correction_speed_mps: 0.18
        max_horizontal_accel_mps2: 0.30
        max_horizontal_jerk_mps3: 0.55
      drop_align:
        velocity_feedforward_gain: 1.00
        position_kp: 0.55
        relative_velocity_kd: 0.30
        max_horizontal_speed_mps: 0.75
        max_correction_speed_mps: 0.12
        max_horizontal_accel_mps2: 0.22
        max_horizontal_jerk_mps3: 0.40
```

- [ ] **Step 5: Run validation tests**

Run:

```powershell
python -m unittest tests.test_air_ground_mission_velocity_control.FollowVelocityProfileValidationTest -v
```

Expected: five tests pass.

### Task 2: Pure feedforward+PD command calculation

**Files:**
- Modify: `scripts/air_ground_mission_fsm.py:75-137`
- Modify: `tests/test_air_ground_mission_velocity_control.py`

**Interfaces:**
- Consumes: one validated profile from `load_follow_velocity_profiles()`
- Produces: `calculate_follow_xy_velocity(profile, car_vx, car_vy, ex, ey, evx, evy) -> dict[str, float]`
- Produces fields: `raw_correction_vx`, `raw_correction_vy`, `correction_vx`, `correction_vy`, `prelimit_vx`, `prelimit_vy`, `desired_vx`, `desired_vy`

- [ ] **Step 1: Add failing controller-math tests**

Append:

```python
class FollowVelocityMathTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_fsm_module()
        cls.profile = _valid_profiles()["follow"]

    def test_zero_errors_produce_car_velocity_feedforward(self):
        result = self.module.calculate_follow_xy_velocity(
            self.profile, 0.40, -0.20, 0.0, 0.0, 0.0, 0.0
        )
        self.assertAlmostEqual(0.40, result["desired_vx"])
        self.assertAlmostEqual(-0.20, result["desired_vy"])

    def test_position_and_relative_velocity_form_pd_correction(self):
        result = self.module.calculate_follow_xy_velocity(
            self.profile, 0.10, 0.0, 0.10, 0.0, 0.04, 0.0
        )
        expected_correction = 0.75 * 0.10 + 0.25 * 0.04
        self.assertAlmostEqual(expected_correction, result["correction_vx"])
        self.assertAlmostEqual(0.10 + expected_correction, result["desired_vx"])

    def test_correction_vector_is_limited(self):
        result = self.module.calculate_follow_xy_velocity(
            self.profile, 0.0, 0.0, 10.0, 10.0, 0.0, 0.0
        )
        correction_speed = math.hypot(
            result["correction_vx"], result["correction_vy"]
        )
        self.assertLessEqual(
            correction_speed,
            self.profile["max_correction_speed_mps"] + 1e-12,
        )

    def test_complete_velocity_vector_is_limited(self):
        result = self.module.calculate_follow_xy_velocity(
            self.profile, 0.70, 0.70, 10.0, 10.0, 0.0, 0.0
        )
        command_speed = math.hypot(
            result["desired_vx"], result["desired_vy"]
        )
        self.assertLessEqual(
            command_speed,
            self.profile["max_horizontal_speed_mps"] + 1e-12,
        )
```

- [ ] **Step 2: Run the math tests and verify they fail**

Run:

```powershell
python -m unittest tests.test_air_ground_mission_velocity_control.FollowVelocityMathTest -v
```

Expected: four tests error because `calculate_follow_xy_velocity` does not exist.

- [ ] **Step 3: Implement the pure controller calculation**

Add after `load_follow_velocity_profiles()`:

```python
def calculate_follow_xy_velocity(
    profile, car_vx, car_vy, ex, ey, evx, evy
):
    ff_gain = profile["velocity_feedforward_gain"]
    kp = profile["position_kp"]
    kd = profile["relative_velocity_kd"]

    raw_correction_vx = kp * ex + kd * evx
    raw_correction_vy = kp * ey + kd * evy
    correction_vx, correction_vy = limit_xy(
        raw_correction_vx,
        raw_correction_vy,
        profile["max_correction_speed_mps"],
    )

    prelimit_vx = ff_gain * car_vx + correction_vx
    prelimit_vy = ff_gain * car_vy + correction_vy
    desired_vx, desired_vy = limit_xy(
        prelimit_vx,
        prelimit_vy,
        profile["max_horizontal_speed_mps"],
    )
    return {
        "raw_correction_vx": raw_correction_vx,
        "raw_correction_vy": raw_correction_vy,
        "correction_vx": correction_vx,
        "correction_vy": correction_vy,
        "prelimit_vx": prelimit_vx,
        "prelimit_vy": prelimit_vy,
        "desired_vx": desired_vx,
        "desired_vy": desired_vy,
    }
```

- [ ] **Step 4: Run profile and controller-math tests**

Run:

```powershell
python -m unittest tests.test_air_ground_mission_velocity_control -v
```

Expected: nine tests pass.

### Task 3: Integrate the drop controller and isolate dynamic landing

**Files:**
- Modify: `scripts/air_ground_mission_fsm.py:1084-1170`
- Modify: `scripts/air_ground_mission_fsm.py:1379-1623`
- Modify: `scripts/air_ground_mission_fsm.py:1750-1970`
- Modify: `tests/test_air_ground_mission_velocity_control.py`

**Interfaces:**
- Consumes: `self.follow_velocity_profiles[profile_name]`
- Produces: `publish_drop_follow_control(car, target_z, dt, profile_name, target_offset_xy=None, max_vz_override=None, max_az_override=None)`
- Produces: `publish_dynamic_follow_control(car, target_z, dt, speed_scale=1.0, target_offset_xy=None, max_vz_override=None, max_az_override=None)`
- Preserves: metrics keys used by stability, descent, alignment, status, and telemetry code

- [ ] **Step 1: Add failing integration tests for final limits and MAVROS mask**

Append a publisher spy and a focused FSM fixture:

```python
class _PublisherSpy:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _make_control_fsm(module):
    fsm = module.AirGroundMissionFSM.__new__(module.AirGroundMissionFSM)
    fsm.follow_velocity_profiles = module.load_follow_velocity_profiles(
        {"velocity_profiles": _valid_profiles()}
    )
    fsm.follow_cfg = {
        "vertical_kp": 0.90,
        "max_vertical_speed_mps": 0.45,
        "max_vertical_accel_mps2": 0.60,
        "velocity_feedforward_gain": 1.0,
        "max_horizontal_speed_mps": 0.75,
        "max_horizontal_accel_mps2": 0.55,
        "max_horizontal_jerk_mps3": 0.90,
    }
    fsm.current_pose = types.SimpleNamespace(
        pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=1.0)
        )
    )
    fsm.current_vel = [0.0, 0.0, 0.0]
    fsm.cmd_xy = [0.74, 0.0]
    fsm.cmd_acc_xy = [0.55, 0.0]
    fsm.cmd_vz = 0.0
    fsm.fixed_yaw = 0.0
    fsm.current_yaw = 0.0
    fsm.raw_pub = _PublisherSpy()
    fsm.last_follow_metrics = {}
    fsm.last_follow_metrics_time = None
    fsm.car_target_xy = lambda car: (car["x"], car["y"])
    return fsm


class FollowVelocityIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_fsm_module()

    def test_drop_control_publishes_velocity_only_and_caps_final_command(self):
        fsm = _make_control_fsm(self.module)
        car = {"x": 10.0, "y": 10.0, "vx": 1.0, "vy": 1.0}
        metrics = fsm.publish_drop_follow_control(
            car, 1.0, 0.02, "follow"
        )

        message = fsm.raw_pub.messages[-1]
        self.assertTrue(message.type_mask & _PositionTarget.IGNORE_PX)
        self.assertTrue(message.type_mask & _PositionTarget.IGNORE_PY)
        self.assertFalse(message.type_mask & _PositionTarget.IGNORE_VX)
        self.assertFalse(message.type_mask & _PositionTarget.IGNORE_VY)
        self.assertLessEqual(
            math.hypot(message.velocity.x, message.velocity.y),
            0.75 + 1e-12,
        )
        self.assertEqual("follow", metrics["control_profile"])

    def test_dynamic_control_keeps_xy_position_fields_active(self):
        fsm = _make_control_fsm(self.module)
        car = {"x": 1.0, "y": 2.0, "vx": 0.2, "vy": 0.0}
        fsm.publish_dynamic_follow_control(car, 1.0, 0.02)

        message = fsm.raw_pub.messages[-1]
        self.assertFalse(message.type_mask & _PositionTarget.IGNORE_PX)
        self.assertFalse(message.type_mask & _PositionTarget.IGNORE_PY)
        self.assertAlmostEqual(1.0, message.position.x)
        self.assertAlmostEqual(2.0, message.position.y)
```

- [ ] **Step 2: Run integration tests and verify they fail**

Run:

```powershell
python -m unittest tests.test_air_ground_mission_velocity_control.FollowVelocityIntegrationTest -v
```

Expected: two tests error because the two explicit controller methods do not exist.

- [ ] **Step 3: Replace the runtime-mode branch with two explicit controller methods**

Rename the current function to `publish_dynamic_follow_control()`. Delete the
`velocity_setpoint_mode` lookup and the `legacy_outer_pd` branch. Its horizontal
command calculation must be exactly:

```python
        ff = safe_float(
            self.follow_cfg.get("velocity_feedforward_gain", 1.0), 1.0
        )
        desired_vx = ff * car["vx"]
        desired_vy = ff * car["vy"]
        max_speed = safe_float(
            self.follow_cfg.get("max_horizontal_speed_mps", 0.75), 0.75
        ) * speed_scale
        desired_vx, desired_vy = limit_xy(
            desired_vx, desired_vy, max_speed
        )
```

Keep its existing acceleration/jerk, vertical control, position+velocity
publisher, and metrics behavior unchanged.

Add `publish_drop_follow_control()` beside it. The horizontal portion must be:

```python
        profile = self.follow_velocity_profiles[profile_name]
        control = calculate_follow_xy_velocity(
            profile,
            car["vx"],
            car["vy"],
            ex,
            ey,
            evx,
            evy,
        )

        previous_cmd_xy = list(self.cmd_xy)
        self.update_xy_command(
            control["desired_vx"],
            control["desired_vy"],
            dt,
            profile["max_horizontal_accel_mps2"],
            profile["max_horizontal_jerk_mps3"],
        )
        limited_vx, limited_vy = limit_xy(
            self.cmd_xy[0],
            self.cmd_xy[1],
            profile["max_horizontal_speed_mps"],
        )
        self.cmd_xy = [limited_vx, limited_vy]
        dt_eff = max(float(dt), 0.01)
        self.cmd_acc_xy = [
            (self.cmd_xy[0] - previous_cmd_xy[0]) / dt_eff,
            (self.cmd_xy[1] - previous_cmd_xy[1]) / dt_eff,
        ]
```

Reuse the existing vertical command calculation, but publish with:

```python
        self.publish_velocity_yaw(
            self.cmd_xy[0],
            self.cmd_xy[1],
            self.cmd_vz,
            self.fixed_yaw,
        )
```

Keep all existing metrics and add:

```python
            "control_profile": profile_name,
            "raw_correction_vx": control["raw_correction_vx"],
            "raw_correction_vy": control["raw_correction_vy"],
            "correction_vx": control["correction_vx"],
            "correction_vy": control["correction_vy"],
            "prelimit_total_vx": control["prelimit_vx"],
            "prelimit_total_vy": control["prelimit_vy"],
            "desired_vx": control["desired_vx"],
            "desired_vy": control["desired_vy"],
            "measured_horizontal_speed": norm2(
                self.current_vel[0], self.current_vel[1]
            ),
            "command_speed_limit": profile["max_horizontal_speed_mps"],
```

Warn without changing state when measured horizontal speed exceeds the command
limit by more than `0.05 m/s`:

```python
        measured_speed = norm2(self.current_vel[0], self.current_vel[1])
        if measured_speed > profile["max_horizontal_speed_mps"] + 0.05:
            rospy.logwarn_throttle(
                1.0,
                "Measured XY speed %.3f exceeds %s command limit %.3f",
                measured_speed,
                profile_name,
                profile["max_horizontal_speed_mps"],
            )
```

- [ ] **Step 4: Route drop and dynamic-landing states explicitly**

Use these exact mappings:

```python
# Drop mission and shared intercept
handle_intercept       -> publish_drop_follow_control(..., "intercept")
handle_follow_drop     -> publish_drop_follow_control(..., "follow")
handle_drop_descent    -> publish_drop_follow_control(..., "drop_descent")
handle_drop_align      -> publish_drop_follow_control(..., "drop_align")
handle_drop_wait_ack   -> publish_drop_follow_control(..., "drop_align")

# Dynamic-landing-specific states; preserve current speed_scale values
handle_follow_cd         -> publish_dynamic_follow_control(..., speed_scale=1.0)
handle_dynamic_descent   -> publish_dynamic_follow_control(..., speed_scale=0.75)
handle_platform_disarm   -> publish_dynamic_follow_control(..., speed_scale=0.55)
handle_platform_takeoff  -> publish_dynamic_follow_control(..., speed_scale=0.85)
```

Remove the `speed_scale` argument from all calls to
`publish_drop_follow_control()`. In particular, `DROP_DESCENT` must use the
`drop_descent` profile instead of scaling the car velocity feedforward.

- [ ] **Step 5: Run all new controller tests**

Run:

```powershell
python -m unittest tests.test_air_ground_mission_velocity_control -v
```

Expected: eleven tests pass.

### Task 4: Regression verification and scope audit

**Files:**
- Verify: `scripts/air_ground_mission_fsm.py`
- Verify: `config/air_ground_mission.yaml`
- Verify: `tests/test_air_ground_mission_velocity_control.py`
- Verify untouched: `AirGroundCar_ESP32_真车通信修改版/`

**Interfaces:**
- Consumes: completed implementation from Tasks 1-3
- Produces: verified UAV-only change set

- [ ] **Step 1: Add source/config regression assertions**

Append:

```python
class FollowVelocitySourceRegressionTest(unittest.TestCase):
    def test_runtime_mode_switches_are_removed(self):
        root = Path(__file__).resolve().parents[1]
        source = (
            root / "scripts" / "air_ground_mission_fsm.py"
        ).read_text(encoding="utf-8")
        config = (
            root / "config" / "air_ground_mission.yaml"
        ).read_text(encoding="utf-8")
        self.assertNotIn("velocity_setpoint_mode", source)
        self.assertNotIn("legacy_outer_pd", source)
        self.assertNotIn("velocity_setpoint_mode:", config)
        self.assertNotIn("descent_horizontal_speed_scale:", config)

    def test_dynamic_handlers_use_the_dedicated_controller(self):
        root = Path(__file__).resolve().parents[1]
        source = (
            root / "scripts" / "air_ground_mission_fsm.py"
        ).read_text(encoding="utf-8")
        for scale in ("0.75", "0.55", "0.85"):
            self.assertIn(
                "publish_dynamic_follow_control", source
            )
            self.assertIn("speed_scale=%s" % scale, source)
```

- [ ] **Step 2: Run all unit tests**

Run:

```powershell
python -m unittest discover -s tests -v
```

Expected: the existing vision-cache test and all thirteen new control tests pass.

- [ ] **Step 3: Run Python syntax checks**

Run:

```powershell
python -m py_compile scripts/air_ground_mission_fsm.py tests/test_air_ground_mission_velocity_control.py
```

Expected: exit code 0 with no output.

- [ ] **Step 4: Validate the YAML syntax and required profiles**

Run:

```powershell
python -c "import yaml; from pathlib import Path; cfg=yaml.safe_load(Path('config/air_ground_mission.yaml').read_text(encoding='utf-8')); p=cfg['mission']['follow']['velocity_profiles']; assert set(p)=={'intercept','follow','drop_descent','drop_align'}; print('profiles:', ', '.join(sorted(p)))"
```

Expected:

```text
profiles: drop_align, drop_descent, follow, intercept
```

- [ ] **Step 5: Audit that no real-car file changed**

Run:

```powershell
Get-ChildItem -Recurse -File 'AirGroundCar_ESP32_真车通信修改版' |
    Select-Object FullName, LastWriteTime
```

Expected: inspection shows no file from that directory was edited during this implementation. Because the workspace is not a Git repository, use the edit-tool history and explicit touched-file list as the authoritative scope record.

- [ ] **Step 6: Record the final verification checkpoint**

Report:

```text
Changed:
- scripts/air_ground_mission_fsm.py
- config/air_ground_mission.yaml
- tests/test_air_ground_mission_velocity_control.py
- docs/superpowers/specs/2026-07-31-task1-uav-velocity-control-design.md
- docs/superpowers/plans/2026-07-31-task1-uav-velocity-control.md

Unchanged:
- AirGroundCar_ESP32_真车通信修改版/**
```
