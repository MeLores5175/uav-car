#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线检查 D26 FIELD/H/local/假小车赛道映射，不需要启动 ROS。"""
import argparse
import math
from pathlib import Path
import yaml


def f(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "yaml_path", nargs="?",
        default=str(Path.home() / "catkin_ws/src/d26_air_ground_uav/config/air_ground_mission.yaml"),
    )
    args = parser.parse_args()
    cfg = yaml.safe_load(Path(args.yaml_path).expanduser().read_text(encoding="utf-8")) or {}
    field = cfg.get("udp_protocol", {}).get("field_transform", {})
    track = cfg.get("car_udp", {}).get("track", {})

    hx = f(field.get("h_field_x_cm", 75.0)) / 100.0
    hy = f(field.get("h_field_y_cm", 75.0)) / 100.0
    theta = math.radians(f(field.get("local_x_to_field_yaw_deg", 0.0)))
    c, s = math.cos(theta), math.sin(theta)

    mode = str(track.get("coordinate_mode", "local")).lower()
    if mode == "field":
        axf = f(track.get("a_field_x_cm", 150.0)) / 100.0
        ayf = f(track.get("a_field_y_cm", 200.0)) / 100.0
        dx, dy = axf - hx, ayf - hy
        ax, ay = c * dx + s * dy, -s * dx + c * dy
        yaw = math.radians(f(track.get("ab_field_yaw_deg", 90.0))) - theta
    else:
        ax, ay = f(track.get("a_x_m", 0.75)), f(track.get("a_y_m", 1.25))
        yaw = math.radians(f(track.get("ab_yaw_deg", 90.0)))

    length = f(track.get("straight_length_m", 1.50))
    radius = f(track.get("radius_m", 0.75))

    def local(u, v):
        fx, fy = math.cos(yaw), math.sin(yaw)
        rx, ry = math.sin(yaw), -math.cos(yaw)
        return ax + fx * u + rx * v, ay + fy * u + ry * v

    def field_xy(x, y):
        return hx + c * x - s * y, hy + s * x + c * y

    local_points = {
        "H": (0.0, 0.0),
        "A": local(0.0, 0.0),
        "B": local(length, 0.0),
        "C": local(length, 2.0 * radius),
        "D": local(0.0, 2.0 * radius),
    }

    print(f"coordinate_mode={mode}")
    print(f"H FIELD=({hx*100:.1f}, {hy*100:.1f}) cm")
    print(f"local +X -> FIELD yaw={math.degrees(theta):.1f} deg")
    print(f"A local relative H=({ax:.3f}, {ay:.3f}) m")
    print(f"AB local yaw={math.degrees(yaw):.1f} deg")
    print("Landmarks:")
    for name, (x, y) in local_points.items():
        xf, yf = field_xy(x, y)
        print(f"  {name}: local=({x:.3f},{y:.3f})m  FIELD=({xf*100:.1f},{yf*100:.1f})cm")

    expected = {"H": (75,75), "A": (150,200), "B": (150,350), "C": (300,350), "D": (300,200)}
    okay = True
    for name, want in expected.items():
        got = field_xy(*local_points[name])
        got_cm = (got[0] * 100.0, got[1] * 100.0)
        if abs(got_cm[0]-want[0]) > 0.5 or abs(got_cm[1]-want[1]) > 0.5:
            okay = False
            print(f"ERROR {name}: expected {want}, got ({got_cm[0]:.1f},{got_cm[1]:.1f})")
    print("RESULT: OK" if okay else "RESULT: MISMATCH - DO NOT FLY")
    raise SystemExit(0 if okay else 2)


if __name__ == "__main__":
    main()
