#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
platform_vision_node.py

FS-J310 下视相机移动平台融合视觉节点（ROS1）。

职责：
1. YOLOv8s TensorRT：远距离发现官方 50 cm 靶标、目标丢失后的重捕获。
2. Tag25h9：近距离精确定位、投放/动态降落阶段的精修。
3. 使用图像采集时刻附近的 UAV 位姿，把机体系相对测量转换到 local 坐标。
4. 发布统一 /uav/platform_vision JSON，供任务 FSM 融合小车遥测与视觉修正。

当前版本的重要约束：
- AprilTag 暂时按“Tag 中心就是平台中心”处理。
- Tag 到平台中心的真实偏置尚未标定，apply_tag_offset 默认必须为 false。
- YOLO 和 AprilTag 共用同一台 /dev/video0 下视相机，禁止另外启动其他占用相机的测试脚本。
"""

import json
import math
import os
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy
import tensorrt as trt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion

try:
    import pycuda.autoinit  # noqa: F401
    import pycuda.driver as cuda
except ImportError as exc:
    raise RuntimeError(
        "缺少 PyCUDA。Jetson 可执行：sudo apt install python3-pycuda"
    ) from exc

try:
    from pupil_apriltags import Detector
except ImportError as exc:
    raise RuntimeError(
        "缺少 pupil_apriltags。请先按文档安装。"
    ) from exc


TRT_LOGGER = trt.Logger(trt.Logger.WARNING)


def clamp(value, low, high):
    return max(low, min(high, value))


def safe_float(value, default=0.0):
    try:
        result = float(value)
        return result if math.isfinite(result) else float(default)
    except Exception:
        return float(default)


def safe_int(value, default=0):
    try:
        return int(value)
    except Exception:
        return int(default)


def json_safe(value):
    """递归替换 NaN/Inf 和 NumPy 标量，保证 JSON 可序列化。"""
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def normalize_mode(value):
    mode = str(value).strip().upper()
    aliases = {
        "0": "OFF",
        "DISABLED": "OFF",
        "IDLE": "OFF",
        "1": "SEARCH",
        "ACQUIRE": "SEARCH",
        "INTERCEPT": "SEARCH",
        "2": "TRACK",
        "FOLLOW": "TRACK",
        "3": "PRECISION",
        "ALIGN": "PRECISION",
        "LAND": "PRECISION",
        "DESCENT": "PRECISION",
    }
    mode = aliases.get(mode, mode)
    return mode if mode in {"OFF", "SEARCH", "TRACK", "PRECISION"} else "SEARCH"


def quaternion_to_yaw(q):
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return float(yaw)


@dataclass
class PoseSample:
    stamp: float
    x: float
    y: float
    z: float
    yaw: float


class PoseHistory:
    """保存最近 UAV 位姿，按图像采集时间查找最邻近样本。"""

    def __init__(self, history_s=2.0):
        self.history_s = max(0.5, float(history_s))
        self.samples = deque()
        self.lock = threading.Lock()

    def add(self, sample: PoseSample):
        with self.lock:
            self.samples.append(sample)
            cutoff = sample.stamp - self.history_s
            while self.samples and self.samples[0].stamp < cutoff:
                self.samples.popleft()

    def nearest(self, stamp_sec: float, max_delta_s: float):
        with self.lock:
            if not self.samples:
                return None, float("inf")
            sample = min(self.samples, key=lambda item: abs(item.stamp - stamp_sec))
            delta = abs(sample.stamp - stamp_sec)
            if delta > max_delta_s:
                return None, delta
            return sample, delta


class TargetPredictor:
    """
    仅用于视觉来源切换、残差门控和短时预测。

    正式“小车遥测 + 视觉”融合仍由任务 FSM 完成。
    """

    def __init__(self, max_speed_mps=1.0):
        self.initialized = False
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.stamp = 0.0
        self.max_speed = max(0.1, float(max_speed_mps))

    def reset(self):
        self.__init__(self.max_speed)

    def predict(self, stamp_sec):
        if not self.initialized:
            return None
        dt = clamp(float(stamp_sec) - self.stamp, 0.0, 0.40)
        return self.x + self.vx * dt, self.y + self.vy * dt

    def residual(self, x, y, stamp_sec):
        predicted = self.predict(stamp_sec)
        if predicted is None:
            return 0.0
        return math.hypot(float(x) - predicted[0], float(y) - predicted[1])

    def update(self, x, y, stamp_sec, alpha):
        x = float(x)
        y = float(y)
        stamp_sec = float(stamp_sec)
        alpha = clamp(float(alpha), 0.05, 1.0)

        if not self.initialized:
            self.initialized = True
            self.x = x
            self.y = y
            self.vx = 0.0
            self.vy = 0.0
            self.stamp = stamp_sec
            return

        dt = clamp(stamp_sec - self.stamp, 0.02, 0.30)
        pred_x = self.x + self.vx * dt
        pred_y = self.y + self.vy * dt
        residual_x = x - pred_x
        residual_y = y - pred_y

        new_x = pred_x + alpha * residual_x
        new_y = pred_y + alpha * residual_y

        beta = 0.06 if alpha >= 0.6 else 0.025
        new_vx = self.vx + beta * residual_x / dt
        new_vy = self.vy + beta * residual_y / dt

        speed = math.hypot(new_vx, new_vy)
        if speed > self.max_speed:
            scale = self.max_speed / max(speed, 1e-6)
            new_vx *= scale
            new_vy *= scale

        self.x = new_x
        self.y = new_y
        self.vx = new_vx
        self.vy = new_vy
        self.stamp = stamp_sec


class TensorRTBackend:
    """固定输入 YOLOv8 Detect Engine，兼容 TensorRT 8/10 API。"""

    def __init__(self, engine_path):
        self.engine_path = os.path.expanduser(engine_path)
        if not os.path.isfile(self.engine_path):
            raise FileNotFoundError("Engine 不存在：%s" % self.engine_path)

        with open(self.engine_path, "rb") as stream:
            runtime = trt.Runtime(TRT_LOGGER)
            self.engine = runtime.deserialize_cuda_engine(stream.read())

        if self.engine is None:
            raise RuntimeError("TensorRT Engine 反序列化失败")

        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError("TensorRT execution context 创建失败")

        self.stream = cuda.Stream()
        self.inputs = []
        self.outputs = []
        self.bindings = []
        self.is_tensor_api = hasattr(self.engine, "num_io_tensors")

        if self.is_tensor_api:
            self._allocate_tensor_api()
        else:
            self._allocate_binding_api()

        if len(self.inputs) != 1:
            raise RuntimeError("只支持单输入 Engine，实际输入数=%d" % len(self.inputs))

        self.input_shape = tuple(int(v) for v in self.inputs[0]["shape"])
        if len(self.input_shape) != 4 or any(v <= 0 for v in self.input_shape):
            raise RuntimeError("Engine 必须为固定 NCHW 输入，实际=%s" % (self.input_shape,))

        self.input_h = self.input_shape[2]
        self.input_w = self.input_shape[3]

        rospy.logwarn(
            "TensorRT loaded: version=%s engine=%s input=%s outputs=%s",
            trt.__version__,
            self.engine_path,
            self.input_shape,
            [item["shape"] for item in self.outputs],
        )

    @staticmethod
    def _make_buffer(name, shape, dtype, index):
        shape = tuple(int(v) for v in shape)
        if any(v < 0 for v in shape):
            raise RuntimeError("当前节点不支持动态 Engine：%s shape=%s" % (name, shape))
        np_dtype = trt.nptype(dtype)
        size = int(trt.volume(shape))
        host = cuda.pagelocked_empty(size, np_dtype)
        device = cuda.mem_alloc(host.nbytes)
        return {
            "name": name,
            "shape": shape,
            "dtype": np_dtype,
            "host": host,
            "device": device,
            "index": index,
        }

    def _allocate_binding_api(self):
        self.bindings = [0] * self.engine.num_bindings
        for index in range(self.engine.num_bindings):
            name = self.engine.get_binding_name(index)
            shape = self.engine.get_binding_shape(index)
            dtype = self.engine.get_binding_dtype(index)
            item = self._make_buffer(name, shape, dtype, index)
            self.bindings[index] = int(item["device"])
            if self.engine.binding_is_input(index):
                self.inputs.append(item)
            else:
                self.outputs.append(item)

    def _allocate_tensor_api(self):
        for index in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(index)
            shape = self.engine.get_tensor_shape(name)
            dtype = self.engine.get_tensor_dtype(name)
            mode = self.engine.get_tensor_mode(name)
            item = self._make_buffer(name, shape, dtype, index)
            self.context.set_tensor_address(name, int(item["device"]))
            if mode == trt.TensorIOMode.INPUT:
                self.inputs.append(item)
            else:
                self.outputs.append(item)

    def infer(self, input_tensor):
        input_item = self.inputs[0]
        flat = np.ascontiguousarray(
            input_tensor.astype(input_item["dtype"], copy=False)
        ).ravel()
        if flat.size != input_item["host"].size:
            raise RuntimeError(
                "TensorRT 输入尺寸不匹配：got=%d expected=%d"
                % (flat.size, input_item["host"].size)
            )

        np.copyto(input_item["host"], flat)
        cuda.memcpy_htod_async(
            input_item["device"], input_item["host"], self.stream
        )

        if self.is_tensor_api:
            ok = self.context.execute_async_v3(stream_handle=self.stream.handle)
        else:
            ok = self.context.execute_async_v2(
                bindings=self.bindings, stream_handle=self.stream.handle
            )

        if not ok:
            raise RuntimeError("TensorRT 推理执行失败")

        for item in self.outputs:
            cuda.memcpy_dtoh_async(item["host"], item["device"], self.stream)

        self.stream.synchronize()
        return [
            np.array(item["host"], copy=True).reshape(item["shape"])
            for item in self.outputs
        ]


def letterbox(image, target_h, target_w, color=(114, 114, 114)):
    source_h, source_w = image.shape[:2]
    ratio = min(target_w / source_w, target_h / source_h)
    new_w = int(round(source_w * ratio))
    new_h = int(round(source_h * ratio))

    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    pad_w = target_w - new_w
    pad_h = target_h - new_h
    left = int(round(pad_w / 2.0 - 0.1))
    right = int(round(pad_w / 2.0 + 0.1))
    top = int(round(pad_h / 2.0 - 0.1))
    bottom = int(round(pad_h / 2.0 + 0.1))

    output = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
    )
    return output, ratio, float(left), float(top)


def preprocess_yolo(frame, input_h, input_w):
    image, ratio, pad_x, pad_y = letterbox(frame, input_h, input_w)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    tensor = image.transpose(2, 0, 1)
    tensor = np.ascontiguousarray(tensor, dtype=np.float32) / 255.0
    return tensor[None], ratio, pad_x, pad_y


def xywh_to_xyxy(boxes):
    output = np.empty_like(boxes)
    output[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.0
    output[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.0
    output[:, 2] = boxes[:, 0] + boxes[:, 2] / 2.0
    output[:, 3] = boxes[:, 1] + boxes[:, 3] / 2.0
    return output


def nms(boxes, scores, iou_threshold):
    if len(boxes) == 0:
        return []

    order = scores.argsort()[::-1]
    keep = []

    while order.size > 0:
        index = int(order[0])
        keep.append(index)
        if order.size == 1:
            break

        rest = order[1:]
        xx1 = np.maximum(boxes[index, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[index, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[index, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[index, 3], boxes[rest, 3])

        intersection = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        area_a = max(0.0, boxes[index, 2] - boxes[index, 0]) * max(
            0.0, boxes[index, 3] - boxes[index, 1]
        )
        area_b = np.maximum(0.0, boxes[rest, 2] - boxes[rest, 0]) * np.maximum(
            0.0, boxes[rest, 3] - boxes[rest, 1]
        )
        iou = intersection / np.maximum(area_a + area_b - intersection, 1e-7)
        order = rest[iou <= iou_threshold]

    return keep


def decode_yolo(
    output,
    frame_shape,
    ratio,
    pad_x,
    pad_y,
    conf_threshold,
    iou_threshold,
    class_id,
):
    prediction = np.squeeze(output)
    if prediction.ndim != 2:
        raise RuntimeError("YOLO 输出维度异常：%s" % (output.shape,))

    if prediction.shape[0] < prediction.shape[1]:
        prediction = prediction.T

    if prediction.shape[1] < 5:
        raise RuntimeError("YOLO 输出通道不足：%s" % (prediction.shape,))

    boxes_xywh = prediction[:, :4]
    class_scores = prediction[:, 4:]

    if class_id >= class_scores.shape[1]:
        raise RuntimeError(
            "yolo_class_id=%d 超出模型类别数=%d"
            % (class_id, class_scores.shape[1])
        )

    scores = class_scores[:, class_id]
    mask = scores >= conf_threshold
    boxes_xywh = boxes_xywh[mask]
    scores = scores[mask]

    if len(boxes_xywh) == 0:
        return []

    boxes = xywh_to_xyxy(boxes_xywh)
    boxes[:, [0, 2]] -= pad_x
    boxes[:, [1, 3]] -= pad_y
    boxes /= ratio

    frame_h, frame_w = frame_shape
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, frame_w - 1)
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, frame_h - 1)

    kept = nms(boxes, scores, iou_threshold)
    results = []
    for index in kept:
        x1, y1, x2, y2 = boxes[index]
        results.append(
            {
                "box": [float(x1), float(y1), float(x2), float(y2)],
                "confidence": float(scores[index]),
                "class_id": int(class_id),
            }
        )

    results.sort(key=lambda item: item["confidence"], reverse=True)
    return results


class PlatformVisionNode:
    FSM_MODE_MAP = {
        "INTERCEPT": "SEARCH",
        "FOLLOW_DROP": "TRACK",
        "POST_DROP_FOLLOW": "TRACK",
        "FOLLOW_CD": "TRACK",
        "PLATFORM_TAKEOFF": "TRACK",
        "DROP_DESCENT": "PRECISION",
        "DROP_ALIGN": "PRECISION",
        "DROP_WAIT_ACK": "PRECISION",
        "DYNAMIC_DESCENT": "PRECISION",
        "PLATFORM_DISARM": "PRECISION",
    }

    def __init__(self):
        rospy.init_node("platform_vision_node", anonymous=False)

        # ---------- 相机 ----------
        self.camera_index = int(rospy.get_param("~camera_index", 0))
        self.width = int(rospy.get_param("~width", 640))
        self.height = int(rospy.get_param("~height", 480))
        self.camera_fps = float(rospy.get_param("~fps", 30.0))
        self.auto_exposure = float(rospy.get_param("~auto_exposure", 3))
        self.gain = float(rospy.get_param("~gain", 56))
        self.disabled_read_rate_hz = float(
            rospy.get_param("~disabled_read_rate_hz", 3.0)
        )

        self.image_center_u = float(
            rospy.get_param("~image_center_u", self.width * 0.5)
        )
        self.image_center_v = float(
            rospy.get_param("~image_center_v", self.height * 0.5)
        )
        self.image_down_to_forward_sign = float(
            rospy.get_param("~image_down_to_body_forward_sign", -1.0)
        )
        self.image_right_to_left_sign = float(
            rospy.get_param("~image_right_to_body_left_sign", -1.0)
        )

        # ---------- YOLO ----------
        self.engine_path = rospy.get_param(
            "~engine_path", "/home/nvidia/models/targer/best_fp16.engine"
        )
        self.yolo_class_id = int(rospy.get_param("~yolo_class_id", 0))
        self.yolo_class_name = str(
            rospy.get_param("~yolo_class_name", "targer")
        )
        self.yolo_conf_threshold = float(
            rospy.get_param("~yolo_confidence", 0.25)
        )
        self.yolo_iou_threshold = float(rospy.get_param("~yolo_iou", 0.45))
        self.platform_size_m = float(rospy.get_param("~platform_size_m", 0.50))
        self.yolo_cache_s = float(rospy.get_param("~yolo_cache_s", 0.30))
        self.yolo_residual_gate_m = float(
            rospy.get_param("~yolo_residual_gate_m", 0.50)
        )

        # ---------- AprilTag ----------
        self.tag_family = str(rospy.get_param("~tag_family", "tag25h9"))
        self.tag_id = int(rospy.get_param("~tag_id", 0))
        self.tag_size_m = float(rospy.get_param("~tag_size_m", 0.10))
        self.tag_min_margin = float(rospy.get_param("~tag_min_margin", 20.0))
        self.tag_max_hamming = int(rospy.get_param("~tag_max_hamming", 0))
        self.tag_confirm_frames = int(
            rospy.get_param("~tag_confirm_frames", 3)
        )
        self.tag_quad_decimate = float(
            rospy.get_param("~tag_quad_decimate", 1.0)
        )
        self.tag_nthreads = int(rospy.get_param("~tag_nthreads", 2))
        self.tag_cache_s = float(rospy.get_param("~tag_cache_s", 0.15))
        self.tag_hold_time_s = float(
            rospy.get_param("~tag_hold_time_s", 0.15)
        )
        self.tag_fallback_time_s = float(
            rospy.get_param("~tag_fallback_time_s", 0.40)
        )
        self.tag_residual_gate_m = float(
            rospy.get_param("~apriltag_residual_gate_m", 0.35)
        )

        # 当前默认关闭。后续标定后再启用。
        self.apply_tag_offset = bool(
            rospy.get_param("~apply_tag_offset", False)
        )
        self.tag_to_platform_forward_m = float(
            rospy.get_param("~tag_to_platform_forward_m", 0.0)
        )
        self.tag_to_platform_left_m = float(
            rospy.get_param("~tag_to_platform_left_m", 0.0)
        )

        if self.apply_tag_offset:
            rospy.logwarn(
                "apply_tag_offset=true：当前实现只支持固定机体系偏置。"
                "若偏置应随小车/Tag航向旋转，请先完成方向标定再修改 offset 函数。"
            )

        # ---------- 各模式推理频率 ----------
        self.search_yolo_n = max(
            1, int(rospy.get_param("~search_yolo_every_n_frames", 1))
        )
        self.search_tag_n = max(
            1, int(rospy.get_param("~search_tag_every_n_frames", 3))
        )
        self.track_yolo_n = max(
            1, int(rospy.get_param("~track_yolo_every_n_frames", 6))
        )
        self.track_tag_n = max(
            1, int(rospy.get_param("~track_tag_every_n_frames", 1))
        )
        self.precision_yolo_n = max(
            1, int(rospy.get_param("~precision_yolo_every_n_frames", 10))
        )
        self.precision_tag_n = max(
            1, int(rospy.get_param("~precision_tag_every_n_frames", 1))
        )

        # ---------- ROS / 时间同步 ----------
        self.pose_topic = str(
            rospy.get_param("~pose_topic", "/mavros/local_position/pose")
        )
        self.pose_msg_type = str(
            rospy.get_param("~pose_msg_type", "pose_stamped")
        ).strip().lower()
        self.pose_sync_max_delta_s = float(
            rospy.get_param("~pose_sync_max_delta_s", 0.12)
        )
        self.pose_history = PoseHistory(
            float(rospy.get_param("~pose_history_s", 2.0))
        )

        self.range_topic = str(
            rospy.get_param(
                "~range_topic", "/mavros/distance_sensor/rangefinder_pub"
            )
        )
        self.range_m = None
        self.range_stamp = None

        self.publish_rate_hz = float(
            rospy.get_param("~publish_rate_hz", 25.0)
        )
        self.status_rate_hz = float(
            rospy.get_param("~status_rate_hz", 2.0)
        )
        self.publish_raw_results = bool(
            rospy.get_param("~publish_raw_results", True)
        )
        self.show_debug = bool(rospy.get_param("~show_debug", False))
        self.auto_mode_from_fsm = bool(
            rospy.get_param("~auto_mode_from_fsm", True)
        )
        self.explicit_mode_timeout_s = float(
            rospy.get_param("~explicit_mode_timeout_s", 1.0)
        )

        # ---------- 运行状态 ----------
        self.scan_enabled = False
        self.explicit_mode = "SEARCH"
        self.explicit_mode_stamp = 0.0
        self.fsm_state = "UNKNOWN"
        self.frame_id = 0
        self.last_publish = 0.0
        self.last_status_publish = 0.0
        self.last_disabled_read = 0.0
        self.last_yolo_result = None
        self.last_tag_result = None
        self.last_selected_source = "NONE"
        self.source_stable_count = 0
        self.tag_good_count = 0
        self.last_tag_good_stamp = 0.0
        self.predictor = TargetPredictor(
            float(rospy.get_param("~visual_predictor_max_speed_mps", 1.0))
        )
        self.last_predictor_measurement_stamp = {
            "YOLO": None,
            "APRILTAG": None,
        }

        self.fps_value = 0.0
        self.fps_frames = 0
        self.fps_start = time.monotonic()
        self.yolo_ms = 0.0
        self.tag_ms = 0.0

        # ---------- 模型 ----------
        self.trt = TensorRTBackend(self.engine_path)
        self.tag_detector = Detector(
            families=self.tag_family,
            nthreads=max(1, self.tag_nthreads),
            quad_decimate=max(1.0, self.tag_quad_decimate),
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        # ---------- 发布 / 订阅 ----------
        self.result_pub = rospy.Publisher(
            "/uav/platform_vision", String, queue_size=30
        )
        self.yolo_pub = rospy.Publisher(
            "/uav/platform_vision/yolo", String, queue_size=10
        )
        self.tag_pub = rospy.Publisher(
            "/uav/platform_vision/apriltag", String, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            "/uav/platform_vision/status", String, queue_size=5, latch=True
        )

        rospy.Subscriber(
            "/uav/platform_scan_enable", Bool, self.scan_enable_cb, queue_size=5
        )
        rospy.Subscriber(
            "/uav/platform_vision_mode", String, self.mode_cb, queue_size=5
        )
        rospy.Subscriber(
            "/uav/fsm_state", String, self.fsm_state_cb, queue_size=10
        )
        rospy.Subscriber(self.range_topic, Range, self.range_cb, queue_size=20)

        if self.pose_msg_type in {"odom", "odometry"}:
            rospy.Subscriber(
                self.pose_topic, Odometry, self.odom_cb, queue_size=100
            )
        else:
            rospy.Subscriber(
                self.pose_topic, PoseStamped, self.pose_cb, queue_size=100
            )

        self.cap = None
        self.open_camera(force=True)
        rospy.on_shutdown(self.shutdown)

        rospy.logwarn(
            "Platform vision ready. camera=%d engine=%s tag=%s:%d pose=%s(%s)",
            self.camera_index,
            self.engine_path,
            self.tag_family,
            self.tag_id,
            self.pose_topic,
            self.pose_msg_type,
        )

    def scan_enable_cb(self, msg):
        enabled = bool(msg.data)
        if enabled != self.scan_enabled:
            rospy.logwarn("Platform vision scan_enable: %s", enabled)
            self.scan_enabled = enabled
            if not enabled:
                self.last_selected_source = "NONE"
                self.source_stable_count = 0
                self.tag_good_count = 0
                self.predictor.reset()
                self.last_predictor_measurement_stamp = {
                    "YOLO": None,
                    "APRILTAG": None,
                }

    def mode_cb(self, msg):
        self.explicit_mode = normalize_mode(msg.data)
        self.explicit_mode_stamp = rospy.Time.now().to_sec()

    def fsm_state_cb(self, msg):
        self.fsm_state = str(msg.data).strip().upper()

    def pose_cb(self, msg):
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0:
            stamp = rospy.Time.now().to_sec()
        q = msg.pose.orientation
        p = msg.pose.position
        self.pose_history.add(
            PoseSample(stamp, p.x, p.y, p.z, quaternion_to_yaw(q))
        )

    def odom_cb(self, msg):
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0:
            stamp = rospy.Time.now().to_sec()
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        self.pose_history.add(
            PoseSample(stamp, p.x, p.y, p.z, quaternion_to_yaw(q))
        )

    def range_cb(self, msg):
        value = safe_float(msg.range, -1.0)
        if value > 0:
            self.range_m = value
            stamp = msg.header.stamp.to_sec()
            self.range_stamp = stamp if stamp > 0 else rospy.Time.now().to_sec()

    def current_mode(self):
        if not self.scan_enabled:
            return "OFF"

        now = rospy.Time.now().to_sec()
        if now - self.explicit_mode_stamp <= self.explicit_mode_timeout_s:
            return self.explicit_mode

        if self.auto_mode_from_fsm:
            return self.FSM_MODE_MAP.get(self.fsm_state, "OFF")

        return "SEARCH"

    def open_camera(self, force=False):
        if self.cap is not None and self.cap.isOpened() and not force:
            return True

        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass

        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            rospy.logerr("无法打开下视相机 /dev/video%d", self.camera_index)
            return False

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, self.auto_exposure)
        self.cap.set(cv2.CAP_PROP_GAIN, self.gain)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return True

    def mode_intervals(self, mode):
        if mode == "PRECISION":
            return self.precision_yolo_n, self.precision_tag_n
        if mode == "TRACK":
            return self.track_yolo_n, self.track_tag_n
        return self.search_yolo_n, self.search_tag_n

    def relative_to_local(self, forward_m, left_m, capture_stamp):
        pose, delta = self.pose_history.nearest(
            capture_stamp, self.pose_sync_max_delta_s
        )
        if pose is None:
            return None, delta

        dx = math.cos(pose.yaw) * forward_m - math.sin(pose.yaw) * left_m
        dy = math.sin(pose.yaw) * forward_m + math.cos(pose.yaw) * left_m

        return {
            "target_x_local": pose.x + dx,
            "target_y_local": pose.y + dy,
            "uav_x_local": pose.x,
            "uav_y_local": pose.y,
            "uav_z_local": pose.z,
            "uav_yaw": pose.yaw,
        }, delta

    def pixel_to_relative(self, center_u, center_v, meters_per_pixel):
        du = float(center_u) - self.image_center_u
        dv = float(center_v) - self.image_center_v
        forward = self.image_down_to_forward_sign * dv * meters_per_pixel
        left = self.image_right_to_left_sign * du * meters_per_pixel
        return forward, left

    def detect_yolo(self, frame, capture_stamp):
        start = time.perf_counter()
        tensor, ratio, pad_x, pad_y = preprocess_yolo(
            frame, self.trt.input_h, self.trt.input_w
        )
        outputs = self.trt.infer(tensor)
        if not outputs:
            raise RuntimeError("YOLO Engine 无输出")

        detections = decode_yolo(
            outputs[0],
            frame.shape[:2],
            ratio,
            pad_x,
            pad_y,
            self.yolo_conf_threshold,
            self.yolo_iou_threshold,
            self.yolo_class_id,
        )
        self.yolo_ms = (time.perf_counter() - start) * 1000.0

        if not detections:
            result = {
                "stamp": capture_stamp,
                "detected": False,
                "reason": "NO_YOLO_TARGET",
                "source": "YOLO",
            }
            self.last_yolo_result = result
            return result

        best = detections[0]
        x1, y1, x2, y2 = best["box"]
        center_u = 0.5 * (x1 + x2)
        center_v = 0.5 * (y1 + y2)
        box_w = max(1.0, x2 - x1)
        box_h = max(1.0, y2 - y1)

        # 50 cm 正方形粗略尺度。用于搜索/重捕获，不作为最终精密降落尺度。
        apparent_size_px = math.sqrt(box_w * box_h)
        meters_per_pixel = self.platform_size_m / apparent_size_px
        forward_m, left_m = self.pixel_to_relative(
            center_u, center_v, meters_per_pixel
        )
        local_data, pose_delta = self.relative_to_local(
            forward_m, left_m, capture_stamp
        )

        result = {
            "stamp": capture_stamp,
            "detected": True,
            "source": "YOLO",
            "confidence": best["confidence"],
            "center_u": center_u,
            "center_v": center_v,
            "box": best["box"],
            "box_width_px": box_w,
            "box_height_px": box_h,
            "apparent_size_px": apparent_size_px,
            "meters_per_pixel": meters_per_pixel,
            "forward_m": forward_m,
            "left_m": left_m,
            "local_position_valid": local_data is not None,
            "pose_time_offset_s": pose_delta,
        }
        if local_data is not None:
            result.update(local_data)

        self.last_yolo_result = result
        return result

    @staticmethod
    def tag_average_side_px(corners):
        corners = np.asarray(corners, dtype=np.float32)
        lengths = []
        for index in range(4):
            lengths.append(
                float(np.linalg.norm(corners[(index + 1) % 4] - corners[index]))
            )
        return float(np.mean(lengths))

    def detect_tag(self, frame, capture_stamp):
        start = time.perf_counter()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.tag_detector.detect(gray, estimate_tag_pose=False)
        self.tag_ms = (time.perf_counter() - start) * 1000.0

        if self.tag_id >= 0:
            detections = [
                item for item in detections if int(item.tag_id) == self.tag_id
            ]

        if not detections:
            self.tag_good_count = 0
            result = {
                "stamp": capture_stamp,
                "detected": False,
                "reason": "NO_APRILTAG",
                "source": "APRILTAG",
            }
            self.last_tag_result = result
            return result

        detection = max(
            detections, key=lambda item: float(item.decision_margin)
        )
        hamming = int(detection.hamming)
        margin = float(detection.decision_margin)
        quality_ok = (
            hamming <= self.tag_max_hamming and margin >= self.tag_min_margin
        )

        if quality_ok:
            self.tag_good_count += 1
            self.last_tag_good_stamp = capture_stamp
        else:
            self.tag_good_count = 0

        corners = np.asarray(detection.corners, dtype=np.float32)
        center_u = float(detection.center[0])
        center_v = float(detection.center[1])
        side_px = max(1.0, self.tag_average_side_px(corners))
        meters_per_pixel = self.tag_size_m / side_px
        forward_m, left_m = self.pixel_to_relative(
            center_u, center_v, meters_per_pixel
        )

        # 当前默认不启用。真实偏置应在明确 Tag 朝向/小车航向后再标定。
        if self.apply_tag_offset:
            forward_m += self.tag_to_platform_forward_m
            left_m += self.tag_to_platform_left_m

        local_data, pose_delta = self.relative_to_local(
            forward_m, left_m, capture_stamp
        )

        confidence = clamp(
            0.55 + 0.45 * (margin - self.tag_min_margin) / 80.0,
            0.0,
            1.0,
        )
        if not quality_ok:
            confidence = min(confidence, 0.45)

        result = {
            "stamp": capture_stamp,
            "detected": True,
            "quality_ok": quality_ok,
            "source": "APRILTAG",
            "confidence": confidence,
            "tag_family": self.tag_family,
            "tag_id": int(detection.tag_id),
            "tag_decision_margin": margin,
            "tag_hamming": hamming,
            "tag_good_count": self.tag_good_count,
            "center_u": center_u,
            "center_v": center_v,
            "corners": corners.tolist(),
            "tag_size_px": side_px,
            "meters_per_pixel": meters_per_pixel,
            "forward_m": forward_m,
            "left_m": left_m,
            "local_position_valid": local_data is not None,
            "pose_time_offset_s": pose_delta,
            "offset_applied": self.apply_tag_offset,
            "tag_to_platform_forward_m": self.tag_to_platform_forward_m,
            "tag_to_platform_left_m": self.tag_to_platform_left_m,
        }
        if local_data is not None:
            result.update(local_data)

        self.last_tag_result = result
        return result

    def candidate_valid(self, result, source, now_sec):
        if not result or not bool(result.get("detected", False)):
            return False

        age = now_sec - safe_float(result.get("stamp", 0.0))
        if source == "APRILTAG":
            if age > self.tag_cache_s:
                return False
            return (
                bool(result.get("quality_ok", False))
                and safe_int(result.get("tag_good_count", 0))
                >= self.tag_confirm_frames
            )

        if age > self.yolo_cache_s:
            return False
        return (
            safe_float(result.get("confidence", 0.0))
            >= self.yolo_conf_threshold
        )

    def result_residual(self, result, stamp_sec):
        if not result or not result.get("local_position_valid", False):
            return 0.0
        return self.predictor.residual(
            result["target_x_local"],
            result["target_y_local"],
            stamp_sec,
        )

    def gate_candidate(self, result, source, stamp_sec):
        if result is None:
            return False, float("inf")

        if not result.get("local_position_valid", False):
            # 没有同步位姿时仍允许输出机体系测量，但不能做 local 残差门控。
            return True, 0.0

        residual = self.result_residual(result, stamp_sec)
        gate = (
            self.tag_residual_gate_m
            if source == "APRILTAG"
            else self.yolo_residual_gate_m
        )

        if not self.predictor.initialized:
            return True, residual

        return residual <= gate, residual

    def select_result(self, mode, capture_stamp):
        now = rospy.Time.now().to_sec()
        tag_valid = self.candidate_valid(
            self.last_tag_result, "APRILTAG", now
        )
        yolo_valid = self.candidate_valid(
            self.last_yolo_result, "YOLO", now
        )

        tag_gate_ok, tag_residual = self.gate_candidate(
            self.last_tag_result, "APRILTAG", capture_stamp
        )
        yolo_gate_ok, yolo_residual = self.gate_candidate(
            self.last_yolo_result, "YOLO", capture_stamp
        )
        tag_valid = tag_valid and tag_gate_ok
        yolo_valid = yolo_valid and yolo_gate_ok

        selected = None
        source = "NONE"
        predicted = False
        residual = 0.0

        if tag_valid:
            selected = dict(self.last_tag_result)
            source = "APRILTAG"
            residual = tag_residual
        else:
            tag_loss_age = (
                now - self.last_tag_good_stamp
                if self.last_tag_good_stamp > 0
                else float("inf")
            )

            if (
                self.last_selected_source in {"APRILTAG", "PREDICTED"}
                and tag_loss_age <= self.tag_hold_time_s
                and self.predictor.initialized
            ):
                predicted_xy = self.predictor.predict(capture_stamp)
                if predicted_xy is not None:
                    selected = {
                        "stamp": capture_stamp,
                        "detected": True,
                        "confidence": 0.60,
                        "local_position_valid": True,
                        "target_x_local": predicted_xy[0],
                        "target_y_local": predicted_xy[1],
                        "forward_m": None,
                        "left_m": None,
                        "center_u": None,
                        "center_v": None,
                    }
                    source = "PREDICTED"
                    predicted = True
            elif yolo_valid:
                selected = dict(self.last_yolo_result)
                source = "YOLO"
                residual = yolo_residual
            elif (
                self.last_selected_source in {"APRILTAG", "PREDICTED"}
                and tag_loss_age <= self.tag_fallback_time_s
                and self.predictor.initialized
            ):
                predicted_xy = self.predictor.predict(capture_stamp)
                if predicted_xy is not None:
                    selected = {
                        "stamp": capture_stamp,
                        "detected": True,
                        "confidence": 0.35,
                        "local_position_valid": True,
                        "target_x_local": predicted_xy[0],
                        "target_y_local": predicted_xy[1],
                        "forward_m": None,
                        "left_m": None,
                        "center_u": None,
                        "center_v": None,
                    }
                    source = "PREDICTED"
                    predicted = True

        if selected is None:
            self.last_selected_source = "NONE"
            self.source_stable_count = 0
            return {
                "target": "landing_platform",
                "stamp": capture_stamp,
                "publish_stamp": now,
                "frame_id": self.frame_id,
                "mode": mode,
                "detected": False,
                "stable": False,
                "stable_count": 0,
                "source": "NONE",
                "confidence": 0.0,
                "reason": "NO_VALID_SOURCE",
                "yolo_detected": bool(
                    self.last_yolo_result
                    and self.last_yolo_result.get("detected", False)
                ),
                "tag_detected": bool(
                    self.last_tag_result
                    and self.last_tag_result.get("detected", False)
                ),
            }

        if source == self.last_selected_source:
            self.source_stable_count += 1
        else:
            self.source_stable_count = 1
        self.last_selected_source = source

        if (
            not predicted
            and selected.get("local_position_valid", False)
        ):
            measurement_stamp = safe_float(
                selected.get("stamp", capture_stamp), capture_stamp
            )
            alpha = (
                0.78
                if source == "APRILTAG"
                else 0.35
            )
            alpha *= clamp(
                safe_float(selected.get("confidence", 0.0)),
                0.40,
                1.0,
            )
            if (
                self.last_predictor_measurement_stamp.get(source)
                != measurement_stamp
            ):
                self.predictor.update(
                    selected["target_x_local"],
                    selected["target_y_local"],
                    measurement_stamp,
                    alpha,
                )
                self.last_predictor_measurement_stamp[source] = measurement_stamp

        confidence = safe_float(selected.get("confidence", 0.0))
        stable_required = (
            self.tag_confirm_frames if source == "APRILTAG" else 2
        )
        stable = (
            source in {"APRILTAG", "YOLO"}
            and self.source_stable_count >= stable_required
        )

        if source == "APRILTAG" and confidence >= 0.80:
            quality = "HIGH"
        elif source == "YOLO" and confidence >= 0.60:
            quality = "MEDIUM"
        elif source == "PREDICTED":
            quality = "PREDICTED"
        else:
            quality = "LOW"

        payload = {
            "target": "landing_platform",
            "stamp": safe_float(selected.get("stamp", capture_stamp)),
            "publish_stamp": now,
            "frame_id": self.frame_id,
            "mode": mode,
            "detected": True,
            "stable": stable,
            "stable_count": self.source_stable_count,
            "source": source,
            "quality": quality,
            "confidence": confidence,
            "center_u": selected.get("center_u"),
            "center_v": selected.get("center_v"),
            "forward_m": selected.get("forward_m"),
            "left_m": selected.get("left_m"),
            "target_x_local": selected.get("target_x_local"),
            "target_y_local": selected.get("target_y_local"),
            "local_position_valid": bool(
                selected.get("local_position_valid", False)
            ),
            "pose_time_offset_s": selected.get("pose_time_offset_s"),
            "measurement_age_s": max(
                0.0, now - safe_float(selected.get("stamp", now))
            ),
            "fusion_residual_m": residual,
            "predictor_vx_mps": self.predictor.vx
            if self.predictor.initialized
            else None,
            "predictor_vy_mps": self.predictor.vy
            if self.predictor.initialized
            else None,
            "yolo_detected": bool(
                self.last_yolo_result
                and self.last_yolo_result.get("detected", False)
            ),
            "yolo_confidence": (
                self.last_yolo_result.get("confidence", 0.0)
                if self.last_yolo_result
                else 0.0
            ),
            "tag_detected": bool(
                self.last_tag_result
                and self.last_tag_result.get("detected", False)
            ),
            "tag_family": self.tag_family,
            "tag_id": (
                self.last_tag_result.get("tag_id")
                if self.last_tag_result
                else None
            ),
            "tag_decision_margin": (
                self.last_tag_result.get("tag_decision_margin")
                if self.last_tag_result
                else None
            ),
            "tag_hamming": (
                self.last_tag_result.get("tag_hamming")
                if self.last_tag_result
                else None
            ),
            "tag_size_px": (
                self.last_tag_result.get("tag_size_px")
                if self.last_tag_result
                else None
            ),
            "offset_applied": bool(self.apply_tag_offset),
            "tag_to_platform_forward_m": self.tag_to_platform_forward_m,
            "tag_to_platform_left_m": self.tag_to_platform_left_m,
        }
        return payload

    def publish_json(self, publisher, payload):
        publisher.publish(
            String(
                data=json.dumps(
                    json_safe(payload),
                    ensure_ascii=False,
                    separators=(",", ":"),
                    allow_nan=False,
                )
            )
        )

    def publish_raw(self):
        if not self.publish_raw_results:
            return
        if self.last_yolo_result is not None:
            payload = dict(self.last_yolo_result)
            payload["frame_id"] = self.frame_id
            self.publish_json(self.yolo_pub, payload)
        if self.last_tag_result is not None:
            payload = dict(self.last_tag_result)
            payload["frame_id"] = self.frame_id
            self.publish_json(self.tag_pub, payload)

    def publish_status(self, mode):
        now = rospy.Time.now().to_sec()
        if now - self.last_status_publish < 1.0 / max(
            self.status_rate_hz, 0.2
        ):
            return
        self.last_status_publish = now

        payload = {
            "stamp": now,
            "camera_open": bool(self.cap and self.cap.isOpened()),
            "camera_index": self.camera_index,
            "resolution": [self.width, self.height],
            "scan_enabled": self.scan_enabled,
            "mode": mode,
            "fsm_state": self.fsm_state,
            "selected_source": self.last_selected_source,
            "fps": self.fps_value,
            "yolo_ms": self.yolo_ms,
            "tag_ms": self.tag_ms,
            "engine_path": self.engine_path,
            "tag_family": self.tag_family,
            "tag_id": self.tag_id,
            "pose_topic": self.pose_topic,
            "pose_msg_type": self.pose_msg_type,
            "apply_tag_offset": self.apply_tag_offset,
        }
        self.publish_json(self.status_pub, payload)

    def draw_debug(self, frame, payload):
        canvas = frame.copy()
        cv2.drawMarker(
            canvas,
            (int(self.image_center_u), int(self.image_center_v)),
            (255, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=28,
            thickness=2,
        )

        if self.last_yolo_result and self.last_yolo_result.get(
            "detected", False
        ):
            x1, y1, x2, y2 = [
                int(round(v)) for v in self.last_yolo_result["box"]
            ]
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (255, 128, 0), 2)

        if self.last_tag_result and self.last_tag_result.get(
            "detected", False
        ):
            corners = np.asarray(
                self.last_tag_result["corners"], dtype=np.int32
            )
            for index in range(4):
                p1 = tuple(corners[index])
                p2 = tuple(corners[(index + 1) % 4])
                cv2.line(canvas, p1, p2, (0, 255, 0), 2)

        center_u = payload.get("center_u")
        center_v = payload.get("center_v")
        if center_u is not None and center_v is not None:
            cv2.circle(
                canvas,
                (int(round(center_u)), int(round(center_v))),
                6,
                (0, 0, 255),
                -1,
            )

        cv2.putText(
            canvas,
            "%s %s conf=%.2f stable=%d"
            % (
                payload.get("mode"),
                payload.get("source"),
                safe_float(payload.get("confidence", 0.0)),
                safe_int(payload.get("stable_count", 0)),
            ),
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "FPS=%.1f YOLO=%.1fms TAG=%.1fms"
            % (self.fps_value, self.yolo_ms, self.tag_ms),
            (10, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.imshow("platform_vision_node", canvas)
        cv2.waitKey(1)

    def shutdown(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        if self.show_debug:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

    def spin(self):
        loop_rate = rospy.Rate(max(self.camera_fps, self.publish_rate_hz, 5.0))

        # 相机预热。
        for _ in range(20):
            if self.cap is not None and self.cap.isOpened():
                self.cap.read()

        # TensorRT 预热。
        dummy = np.zeros(self.trt.input_shape, dtype=np.float32)
        for _ in range(3):
            self.trt.infer(dummy)

        while not rospy.is_shutdown():
            mode = self.current_mode()
            now_wall = time.monotonic()

            if self.cap is None or not self.cap.isOpened():
                self.open_camera(force=True)
                self.publish_status(mode)
                loop_rate.sleep()
                continue

            if mode == "OFF":
                if now_wall - self.last_disabled_read < 1.0 / max(
                    self.disabled_read_rate_hz, 0.5
                ):
                    self.publish_status(mode)
                    loop_rate.sleep()
                    continue
                self.last_disabled_read = now_wall

            ok, frame = self.cap.read()
            capture_stamp = rospy.Time.now().to_sec()

            if not ok or frame is None:
                rospy.logwarn_throttle(
                    1.0, "下视相机读取失败，尝试重新打开。"
                )
                self.open_camera(force=True)
                self.publish_status(mode)
                loop_rate.sleep()
                continue

            self.frame_id += 1
            self.fps_frames += 1
            fps_elapsed = now_wall - self.fps_start
            if fps_elapsed >= 1.0:
                self.fps_value = self.fps_frames / fps_elapsed
                self.fps_frames = 0
                self.fps_start = now_wall

            if mode == "OFF":
                if capture_stamp - self.last_publish >= 0.5:
                    self.last_publish = capture_stamp
                    payload = {
                        "target": "landing_platform",
                        "stamp": capture_stamp,
                        "publish_stamp": capture_stamp,
                        "frame_id": self.frame_id,
                        "mode": "OFF",
                        "detected": False,
                        "stable": False,
                        "stable_count": 0,
                        "source": "NONE",
                        "confidence": 0.0,
                        "reason": "VISION_DISABLED",
                    }
                    self.publish_json(self.result_pub, payload)
                self.publish_status(mode)
                loop_rate.sleep()
                continue

            yolo_interval, tag_interval = self.mode_intervals(mode)

            try:
                if self.frame_id % yolo_interval == 0:
                    self.detect_yolo(frame, capture_stamp)
                if self.frame_id % tag_interval == 0:
                    self.detect_tag(frame, capture_stamp)
            except Exception as exc:
                rospy.logerr_throttle(
                    1.0, "视觉推理异常：%s", str(exc)
                )

            payload = self.select_result(mode, capture_stamp)

            if capture_stamp - self.last_publish >= 1.0 / max(
                self.publish_rate_hz, 1.0
            ):
                self.last_publish = capture_stamp
                self.publish_json(self.result_pub, payload)
                self.publish_raw()

            self.publish_status(mode)

            if self.show_debug:
                self.draw_debug(frame, payload)

            loop_rate.sleep()


if __name__ == "__main__":
    try:
        PlatformVisionNode().spin()
    except rospy.ROSInterruptException:
        pass
