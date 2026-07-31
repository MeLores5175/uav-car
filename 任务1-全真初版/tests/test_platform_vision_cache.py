import importlib.util
import sys
import types
import unittest
from pathlib import Path


class _RosTime:
    current = 10.0

    @classmethod
    def now(cls):
        return types.SimpleNamespace(to_sec=lambda: cls.current)


class _TrtLogger:
    WARNING = 1

    def __init__(self, *_args, **_kwargs):
        pass


def _install_module(name, **attributes):
    module = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


def _load_platform_vision_module():
    _install_module("cv2")
    _install_module("numpy", generic=type("generic", (), {}))
    _install_module("rospy", Time=_RosTime)
    _install_module("tensorrt", Logger=_TrtLogger)

    geometry_msg = _install_module(
        "geometry_msgs.msg", PoseStamped=type("PoseStamped", (), {})
    )
    _install_module("geometry_msgs", msg=geometry_msg)
    nav_msg = _install_module("nav_msgs.msg", Odometry=type("Odometry", (), {}))
    _install_module("nav_msgs", msg=nav_msg)
    sensor_msg = _install_module("sensor_msgs.msg", Range=type("Range", (), {}))
    _install_module("sensor_msgs", msg=sensor_msg)
    std_msg = _install_module(
        "std_msgs.msg",
        Bool=type("Bool", (), {}),
        String=type("String", (), {}),
    )
    _install_module("std_msgs", msg=std_msg)

    transformations = _install_module(
        "tf.transformations",
        euler_from_quaternion=lambda _value: (0.0, 0.0, 0.0),
    )
    _install_module("tf", transformations=transformations)

    pycuda = _install_module("pycuda")
    pycuda_autoinit = _install_module("pycuda.autoinit")
    pycuda_driver = _install_module("pycuda.driver")
    pycuda.autoinit = pycuda_autoinit
    pycuda.driver = pycuda_driver
    _install_module("pupil_apriltags", Detector=type("Detector", (), {}))

    source = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "platform_vision_node.py"
    )
    spec = importlib.util.spec_from_file_location(
        "platform_vision_node_under_test", source
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class _PredictorSpy:
    def __init__(self):
        self.calls = []
        self.initialized = True
        self.vx = 0.0
        self.vy = 0.0

    def update(self, x, y, stamp, alpha):
        self.calls.append((x, y, stamp, alpha))


class PlatformVisionCacheTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_platform_vision_module()

    def test_cached_measurement_updates_predictor_only_once_with_measurement_stamp(self):
        node = self.module.PlatformVisionNode.__new__(
            self.module.PlatformVisionNode
        )
        node.last_tag_result = None
        node.last_yolo_result = {
            "stamp": 10.0,
            "detected": True,
            "confidence": 0.9,
            "local_position_valid": True,
            "target_x_local": 1.0,
            "target_y_local": 2.0,
        }
        node.last_tag_good_stamp = 0.0
        node.last_selected_source = "NONE"
        node.source_stable_count = 0
        node.frame_id = 1
        node.tag_confirm_frames = 3
        node.tag_family = "tag25h9"
        node.apply_tag_offset = False
        node.tag_to_platform_forward_m = 0.0
        node.tag_to_platform_left_m = 0.0
        node.last_predictor_measurement_stamp = {
            "YOLO": None,
            "APRILTAG": None,
        }
        node.predictor = _PredictorSpy()
        node.candidate_valid = (
            lambda _result, source, _now: source == "YOLO"
        )
        node.gate_candidate = (
            lambda _result, source, _stamp: (source == "YOLO", 0.0)
        )

        _RosTime.current = 10.05
        node.select_result("TRACK", 10.05)
        _RosTime.current = 10.08
        node.select_result("TRACK", 10.08)

        self.assertEqual(1, len(node.predictor.calls))
        self.assertEqual(10.0, node.predictor.calls[0][2])


if __name__ == "__main__":
    unittest.main()
