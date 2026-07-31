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
    _install_module("yaml")
    _install_module(
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

    def test_follow_config_must_be_a_mapping(self):
        with self.assertRaisesRegex(ValueError, "mission.follow"):
            self.module.load_follow_velocity_profiles(None)

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


class FollowVelocityRoutingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (
            Path(__file__).resolve().parents[1]
            / "scripts"
            / "air_ground_mission_fsm.py"
        ).read_text(encoding="utf-8")

    def test_drop_states_use_explicit_profiles(self):
        self.assertEqual(
            5, self.source.count("self.publish_drop_follow_control(")
        )
        for profile_name in (
            '"intercept"',
            '"follow"',
            '"drop_descent"',
            '"drop_align"',
        ):
            self.assertIn(profile_name, self.source)

    def test_dynamic_states_use_dedicated_controller(self):
        self.assertEqual(
            4, self.source.count("self.publish_dynamic_follow_control(")
        )
        for speed_scale in ("0.75", "0.55", "0.85"):
            self.assertIn("speed_scale=%s" % speed_scale, self.source)

    def test_old_runtime_follow_entry_point_is_not_called(self):
        self.assertNotIn("self.publish_follow_control(", self.source)


class FollowVelocityConfigRegressionTest(unittest.TestCase):
    def test_runtime_mode_and_descent_feedforward_scale_are_removed(self):
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


if __name__ == "__main__":
    unittest.main()
