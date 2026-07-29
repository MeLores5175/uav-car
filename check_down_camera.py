#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FS-J310 下视 USB 相机画面测试

运行：
    python3 check_down_camera.py

指定相机编号：
    python3 check_down_camera.py 0
    python3 check_down_camera.py 1

按键：
    Q / ESC：退出
    S：保存当前画面
"""

import sys
import time
from pathlib import Path

import cv2


def main():
    # 默认打开 /dev/video0，也可以通过命令行指定编号
    camera_index = int(sys.argv[1]) if len(sys.argv) > 1 else 0

    print("=" * 60)
    print("FS-J310 下视相机测试")
    print(f"准备打开：/dev/video{camera_index}")
    print("按 Q 或 ESC 退出，按 S 保存截图")
    print("=" * 60)

    # Linux/Jetson 上明确使用 V4L2
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)

    if not cap.isOpened():
        print(f"[错误] 无法打开 /dev/video{camera_index}")
        print("请执行以下命令查看相机设备：")
        print("    ls -l /dev/video*")
        return

    # 设置 MJPG，通常可提高 USB 相机帧率
    cap.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*"MJPG")
    )

    # 使用之前 animal_detect_pc 中的分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # 沿用之前验证过的曝光和增益设置
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
    cap.set(cv2.CAP_PROP_GAIN, 56)

    # 减少缓存，尽量显示最新画面
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))

    fourcc_text = "".join(
        chr((actual_fourcc >> 8 * i) & 0xFF)
        for i in range(4)
    )

    print(f"[成功] 已打开 /dev/video{camera_index}")
    print(f"实际分辨率：{actual_width} × {actual_height}")
    print(f"相机报告帧率：{actual_fps:.1f} FPS")
    print(f"视频格式：{fourcc_text}")

    save_dir = Path.home() / "down_camera_snapshots"
    save_dir.mkdir(parents=True, exist_ok=True)

    frame_count = 0
    fps_value = 0.0
    fps_start_time = time.time()

    window_name = "FS-J310 Downward Camera"

    try:
        while True:
            ret, frame = cap.read()

            if not ret or frame is None:
                print("[警告] 未读取到相机画面")
                time.sleep(0.05)
                continue

            height, width = frame.shape[:2]

            # 图像几何中心
            center_u = width // 2
            center_v = height // 2

            # 绘制中心十字，用于确认相机方向和对准位置
            cross_length = 30
            cv2.line(
                frame,
                (center_u - cross_length, center_v),
                (center_u + cross_length, center_v),
                (0, 255, 0),
                2,
            )
            cv2.line(
                frame,
                (center_u, center_v - cross_length),
                (center_u, center_v + cross_length),
                (0, 255, 0),
                2,
            )
            cv2.circle(
                frame,
                (center_u, center_v),
                5,
                (0, 0, 255),
                -1,
            )

            # 计算实际显示帧率
            frame_count += 1
            now = time.time()
            elapsed = now - fps_start_time

            if elapsed >= 1.0:
                fps_value = frame_count / elapsed
                frame_count = 0
                fps_start_time = now

            cv2.putText(
                frame,
                f"/dev/video{camera_index}  {width}x{height}",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                frame,
                f"FPS: {fps_value:.1f}",
                (10, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                frame,
                f"Center: ({center_u}, {center_v})",
                (10, 84),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                frame,
                "Q/ESC: quit   S: screenshot",
                (10, height - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
            )

            cv2.imshow(window_name, frame)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), ord("Q"), 27):
                print("正在退出相机测试……")
                break

            if key in (ord("s"), ord("S")):
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                save_path = save_dir / f"down_camera_{timestamp}.jpg"

                if cv2.imwrite(str(save_path), frame):
                    print(f"[截图已保存] {save_path}")
                else:
                    print("[错误] 截图保存失败")

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在退出……")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("相机已释放。")


if __name__ == "__main__":
    main()