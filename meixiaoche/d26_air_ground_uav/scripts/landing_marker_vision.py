#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""下视相机识别小车平台的 50cm/30cm 同心圆与十字中心。"""

import json
import math
import os
import time

import cv2
import numpy as np
import rospy
import yaml
from std_msgs.msg import Bool, String


def clamp(value, low, high):
    return max(low, min(high, value))


def safe_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


class MotionStableTracker:
    """允许目标移动，用匀速预测残差而不是静态像素跳变判稳。"""

    def __init__(self, max_residual_px, max_diameter_change_ratio):
        self.max_residual_px = float(max_residual_px)
        self.max_diameter_change_ratio = float(max_diameter_change_ratio)
        self.last_center = None
        self.last_velocity = np.zeros(2, dtype=np.float32)
        self.last_diameter = None
        self.last_time = None
        self.count = 0

    def reset(self):
        self.last_center = None
        self.last_velocity[:] = 0.0
        self.last_diameter = None
        self.last_time = None
        self.count = 0

    def update(self, center, diameter, now_sec):
        center = np.asarray(center, dtype=np.float32)
        diameter = float(diameter)
        if self.last_center is None or self.last_time is None:
            self.last_center = center
            self.last_diameter = diameter
            self.last_time = float(now_sec)
            self.count = 1
            return self.count, 0.0

        dt = clamp(float(now_sec) - self.last_time, 0.01, 0.20)
        predicted = self.last_center + self.last_velocity * dt
        residual = float(np.linalg.norm(center - predicted))
        diameter_change = abs(diameter - self.last_diameter) / max(self.last_diameter, 1.0)

        if residual <= self.max_residual_px and diameter_change <= self.max_diameter_change_ratio:
            self.count += 1
        else:
            self.count = 1

        measured_velocity = (center - self.last_center) / dt
        self.last_velocity = 0.65 * self.last_velocity + 0.35 * measured_velocity
        self.last_center = center
        self.last_diameter = diameter
        self.last_time = float(now_sec)
        return self.count, residual


class LandingMarkerVision:
    def __init__(self):
        rospy.init_node("landing_marker_vision", anonymous=False)
        default_yaml = os.path.expanduser(
            "~/catkin_ws/src/d26_air_ground_uav/config/air_ground_mission.yaml"
        )
        config_path = rospy.get_param("~mission_yaml", default_yaml)
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        vcfg = cfg.get("vision", {})

        self.camera_index = int(rospy.get_param("~camera_index", vcfg.get("camera_index", 0)))
        self.width = int(rospy.get_param("~width", vcfg.get("width", 640)))
        self.height = int(rospy.get_param("~height", vcfg.get("height", 480)))
        self.fps = int(rospy.get_param("~fps", vcfg.get("fps", 30)))
        self.show_debug = bool(rospy.get_param("~show_debug", vcfg.get("show_debug", False)))
        self.publish_rate_hz = safe_float(
            rospy.get_param("~publish_rate_hz", vcfg.get("publish_rate_hz", 25.0)), 25.0
        )
        self.disabled_read_rate_hz = safe_float(
            rospy.get_param("~disabled_read_rate_hz", vcfg.get("disabled_read_rate_hz", 3.0)), 3.0
        )

        self.outer_diameter_m = safe_float(vcfg.get("outer_circle_diameter_m", 0.50), 0.50)
        self.ratio_target = safe_float(vcfg.get("inner_outer_ratio", 0.60), 0.60)
        self.ratio_tolerance = safe_float(vcfg.get("ratio_tolerance", 0.18), 0.18)
        self.min_outer_diameter_px = safe_float(vcfg.get("min_outer_diameter_px", 55), 55)
        self.max_outer_diameter_px = safe_float(vcfg.get("max_outer_diameter_px", 620), 620)
        self.min_contour_area_px = safe_float(vcfg.get("min_contour_area_px", 600), 600)
        self.min_circularity = safe_float(vcfg.get("min_circularity", 0.52), 0.52)
        self.max_pair_center_error_ratio = safe_float(
            vcfg.get("max_pair_center_error_ratio", 0.18), 0.18
        )
        self.min_publish_confidence = safe_float(
            vcfg.get("min_publish_confidence", 0.45), 0.45
        )

        self.adaptive_block_size = int(vcfg.get("adaptive_block_size", 41))
        if self.adaptive_block_size % 2 == 0:
            self.adaptive_block_size += 1
        self.adaptive_block_size = max(3, self.adaptive_block_size)
        self.adaptive_c = int(vcfg.get("adaptive_c", 7))
        self.morphology_kernel = max(1, int(vcfg.get("morphology_kernel", 3)))

        self.release_u = safe_float(vcfg.get("release_u", -1.0), -1.0)
        self.release_v = safe_float(vcfg.get("release_v", -1.0), -1.0)
        if self.release_u < 0:
            self.release_u = self.width * 0.5
        if self.release_v < 0:
            self.release_v = self.height * 0.5

        self.image_down_to_forward = safe_float(
            vcfg.get("image_down_to_body_forward_sign", -1.0), -1.0
        )
        self.image_right_to_left = safe_float(
            vcfg.get("image_right_to_body_left_sign", -1.0), -1.0
        )
        self.camera_offset_forward = safe_float(
            vcfg.get("camera_offset_forward_m", 0.0), 0.0
        )
        self.camera_offset_left = safe_float(vcfg.get("camera_offset_left_m", 0.0), 0.0)
        self.focal_length_px = safe_float(vcfg.get("focal_length_px", 520.0), 520.0)

        self.scan_enabled = False
        self.cap = None
        self.last_open_try = 0.0
        self.last_publish = 0.0
        self.last_disabled_read = 0.0
        self.frame_id = 0

        self.tracker = MotionStableTracker(
            safe_float(vcfg.get("max_prediction_residual_px", 42.0), 42.0),
            safe_float(vcfg.get("max_diameter_change_ratio", 0.28), 0.28),
        )

        self.result_pub = rospy.Publisher("/uav/platform_vision", String, queue_size=20)
        rospy.Subscriber(
            "/uav/platform_scan_enable", Bool, self.scan_enable_cb, queue_size=5
        )
        rospy.on_shutdown(self.shutdown)
        self.open_camera(force=True)
        rospy.logwarn("Landing marker vision ready. camera=%d", self.camera_index)

    def scan_enable_cb(self, msg):
        enabled = bool(msg.data)
        if enabled != self.scan_enabled:
            self.scan_enabled = enabled
            if not enabled:
                self.tracker.reset()

    def open_camera(self, force=False):
        now = time.time()
        if not force and now - self.last_open_try < 2.0:
            return False
        self.last_open_try = now
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            rospy.logerr_throttle(2.0, "Cannot open down camera index %d", self.camera_index)
            return False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return True

    def publish_result(self, detected, **kwargs):
        payload = {
            "target": "landing_platform",
            "detected": bool(detected),
            "stamp": rospy.Time.now().to_sec(),
            "frame_id": self.frame_id,
            "confidence": 0.0,
            "stable_count": 0,
        }
        payload.update(kwargs)
        self.result_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

    def preprocess(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        binary = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            self.adaptive_block_size,
            self.adaptive_c,
        )
        kernel = np.ones((self.morphology_kernel, self.morphology_kernel), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
        return gray, binary

    def circle_candidates(self, binary):
        result = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(result) == 3:
            _, contours, hierarchy = result
        else:
            contours, hierarchy = result
        candidates = []
        if hierarchy is None:
            return candidates
        hierarchy = hierarchy[0]
        for idx, contour in enumerate(contours):
            area = float(cv2.contourArea(contour))
            if area < self.min_contour_area_px:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter < 1.0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < self.min_circularity:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            diameter = 2.0 * radius
            if diameter < 18.0 or diameter > self.max_outer_diameter_px * 1.15:
                continue
            rect = cv2.minAreaRect(contour)
            rw, rh = rect[1]
            if min(rw, rh) < 1.0:
                continue
            aspect = min(rw, rh) / max(rw, rh)
            if aspect < 0.48:
                continue
            fill_ratio = area / max(math.pi * radius * radius, 1.0)
            if fill_ratio < 0.15 or fill_ratio > 1.10:
                continue
            candidates.append({
                "idx": idx,
                "center": np.array([cx, cy], dtype=np.float32),
                "diameter": diameter,
                "radius": radius,
                "area": area,
                "circularity": circularity,
                "aspect": aspect,
                "fill_ratio": fill_ratio,
                "parent": int(hierarchy[idx][3]),
                "child": int(hierarchy[idx][2]),
                "contour": contour,
            })
        return candidates

    def choose_pair(self, candidates, frame_shape):
        h, w = frame_shape[:2]
        image_center = np.array([self.release_u, self.release_v], dtype=np.float32)
        best = None
        best_score = -1e9
        for outer in candidates:
            if not (
                self.min_outer_diameter_px <= outer["diameter"] <= self.max_outer_diameter_px
            ):
                continue
            for inner in candidates:
                if inner is outer or inner["diameter"] >= outer["diameter"]:
                    continue
                ratio = inner["diameter"] / max(outer["diameter"], 1.0)
                ratio_error = abs(ratio - self.ratio_target)
                if ratio_error > self.ratio_tolerance:
                    continue
                center_error_px = float(np.linalg.norm(inner["center"] - outer["center"]))
                center_error_ratio = center_error_px / max(outer["radius"], 1.0)
                if center_error_ratio > self.max_pair_center_error_ratio:
                    continue

                nested_bonus = 0.18 if (
                    inner["parent"] == outer["idx"] or outer["parent"] == inner["idx"]
                ) else 0.0
                center_distance = float(np.linalg.norm(outer["center"] - image_center))
                center_penalty = center_distance / max(math.sqrt(w * w + h * h), 1.0)
                ratio_score = 1.0 - ratio_error / max(self.ratio_tolerance, 1e-6)
                concentric_score = 1.0 - center_error_ratio / max(
                    self.max_pair_center_error_ratio, 1e-6
                )
                shape_score = clamp(
                    0.45 * outer["circularity"] +
                    0.25 * inner["circularity"] +
                    0.15 * outer["aspect"] +
                    0.15 * inner["aspect"],
                    0.0,
                    1.0,
                )
                score = (
                    0.38 * ratio_score + 0.32 * concentric_score +
                    0.30 * shape_score + nested_bonus - 0.18 * center_penalty
                )
                if score > best_score:
                    best_score = score
                    best = {
                        "outer": outer,
                        "inner": inner,
                        "center": 0.70 * outer["center"] + 0.30 * inner["center"],
                        "diameter": outer["diameter"],
                        "raw_score": score,
                        "ratio": ratio,
                        "center_error_px": center_error_px,
                    }
        return best

    def fallback_outer(self, candidates, frame_shape):
        h, w = frame_shape[:2]
        image_center = np.array([self.release_u, self.release_v], dtype=np.float32)
        best = None
        best_score = -1e9
        for outer in candidates:
            if not (
                self.min_outer_diameter_px <= outer["diameter"] <= self.max_outer_diameter_px
            ):
                continue
            center_distance = float(np.linalg.norm(outer["center"] - image_center))
            center_penalty = center_distance / max(math.sqrt(w * w + h * h), 1.0)
            score = (
                0.50 * outer["circularity"] +
                0.30 * outer["aspect"] +
                0.20 * clamp(outer["fill_ratio"], 0.0, 1.0) -
                0.25 * center_penalty
            )
            if score > best_score:
                best_score = score
                best = {
                    "outer": outer,
                    "inner": None,
                    "center": outer["center"],
                    "diameter": outer["diameter"],
                    "raw_score": score * 0.58,
                    "ratio": None,
                    "center_error_px": None,
                }
        return best

    def detect(self, frame):
        _, binary = self.preprocess(frame)
        candidates = self.circle_candidates(binary)
        selected = self.choose_pair(candidates, frame.shape)
        paired = selected is not None
        if selected is None:
            selected = self.fallback_outer(candidates, frame.shape)
        if selected is None:
            self.tracker.reset()
            return None, binary

        center = selected["center"]
        diameter = float(selected["diameter"])
        stable_count, residual = self.tracker.update(center, diameter, time.time())
        raw_score = float(selected["raw_score"])
        confidence = clamp(raw_score, 0.0, 1.0)
        confidence *= clamp(0.60 + 0.08 * stable_count, 0.60, 1.0)
        if not paired:
            confidence = min(confidence, 0.62)

        meters_per_pixel = self.outer_diameter_m / max(diameter, 1.0)
        du = float(center[0] - self.release_u)
        dv = float(center[1] - self.release_v)
        forward_m = self.image_down_to_forward * dv * meters_per_pixel
        left_m = self.image_right_to_left * du * meters_per_pixel
        forward_m += self.camera_offset_forward
        left_m += self.camera_offset_left
        estimated_height_m = self.outer_diameter_m * self.focal_length_px / max(diameter, 1.0)

        return {
            "detected": confidence >= self.min_publish_confidence,
            "center_u": float(center[0]),
            "center_v": float(center[1]),
            "outer_diameter_px": diameter,
            "meters_per_pixel": meters_per_pixel,
            "forward_m": forward_m,
            "left_m": left_m,
            "estimated_height_m": estimated_height_m,
            "confidence": confidence,
            "stable_count": stable_count,
            "prediction_residual_px": residual,
            "paired_circles": paired,
            "inner_outer_ratio": selected["ratio"],
            "pair_center_error_px": selected["center_error_px"],
            "outer_contour": selected["outer"]["contour"],
            "inner_contour": selected["inner"]["contour"] if selected["inner"] else None,
        }, binary

    def draw_debug(self, frame, result, binary):
        canvas = frame.copy()
        cv2.drawMarker(
            canvas,
            (int(self.release_u), int(self.release_v)),
            (0, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=2,
        )
        if result is not None:
            cv2.drawContours(canvas, [result["outer_contour"]], -1, (0, 255, 0), 2)
            if result["inner_contour"] is not None:
                cv2.drawContours(canvas, [result["inner_contour"]], -1, (255, 0, 0), 2)
            c = (int(result["center_u"]), int(result["center_v"]))
            cv2.circle(canvas, c, 5, (0, 0, 255), -1)
            text = "F %.2f L %.2f conf %.2f stable %d" % (
                result["forward_m"], result["left_m"],
                result["confidence"], result["stable_count"]
            )
            cv2.putText(canvas, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        cv2.imshow("landing_marker", canvas)
        cv2.imshow("landing_marker_binary", binary)
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
        loop_rate = rospy.Rate(max(self.fps, self.publish_rate_hz, 5.0))
        while not rospy.is_shutdown():
            now = time.time()
            if self.cap is None or not self.cap.isOpened():
                self.open_camera()
                loop_rate.sleep()
                continue

            if not self.scan_enabled:
                if now - self.last_disabled_read < 1.0 / max(self.disabled_read_rate_hz, 0.5):
                    loop_rate.sleep()
                    continue
                self.last_disabled_read = now

            ok, frame = self.cap.read()
            if not ok or frame is None:
                rospy.logwarn_throttle(1.0, "Down camera read failed; reopening.")
                self.open_camera()
                loop_rate.sleep()
                continue

            self.frame_id += 1
            if not self.scan_enabled:
                loop_rate.sleep()
                continue

            result, binary = self.detect(frame)
            if now - self.last_publish >= 1.0 / max(self.publish_rate_hz, 1.0):
                self.last_publish = now
                if result is None:
                    self.publish_result(False, reason="NO_MARKER")
                else:
                    publish_data = dict(result)
                    publish_data.pop("outer_contour", None)
                    publish_data.pop("inner_contour", None)
                    self.publish_result(**publish_data)

            if self.show_debug:
                self.draw_debug(frame, result, binary)

            loop_rate.sleep()


if __name__ == "__main__":
    try:
        LandingMarkerVision().spin()
    except rospy.ROSInterruptException:
        pass
