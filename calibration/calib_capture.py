#!/usr/bin/env python3
"""Capture frames from /cam0/image_raw to PNG for offline calibration.

Runs on the drone. No OpenCV needed -- PIL only.

  python3 calib_capture.py [--interval 1.0] [--count 40] [--out ~/calib_frames]
"""
import argparse, os, sys, time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from PIL import Image as PILImage

# encoding -> (PIL mode, channels, channel slice to keep as RGB)
MODES = {
    "rgb8":   ("RGB",  3, (0, 1, 2)),
    "bgr8":   ("RGB",  3, (2, 1, 0)),
    "rgba8":  ("RGB",  4, (0, 1, 2)),
    "bgra8":  ("RGB",  4, (2, 1, 0)),
    "mono8":  ("L",    1, (0,)),
}


class Capture(Node):
    def __init__(self, out_dir, interval, count):
        super().__init__("calib_capture")
        self.out_dir, self.interval, self.target = out_dir, interval, count
        self.n = 0
        self.last = 0.0
        self.create_subscription(Image, "/cam0/image_raw", self.cb, qos_profile_sensor_data)
        self.get_logger().info(f"saving to {out_dir}, every {interval}s, target {count}")

    def cb(self, msg):
        now = time.time()
        if now - self.last < self.interval:
            return
        self.last = now

        if msg.encoding not in MODES:
            self.get_logger().error(f"unhandled encoding {msg.encoding!r}")
            rclpy.shutdown()
            return
        mode, ch, order = MODES[msg.encoding]

        buf = bytes(msg.data)
        if ch == 1:
            img = PILImage.frombytes("L", (msg.width, msg.height), buf)
        else:
            src = PILImage.frombytes("RGBA" if ch == 4 else "RGB", (msg.width, msg.height), buf)
            bands = src.split()
            img = PILImage.merge("RGB", tuple(bands[i] for i in order))

        path = os.path.join(self.out_dir, f"frame_{self.n:03d}.png")
        img.save(path)
        self.n += 1
        print(f"  [{self.n:2d}/{self.target}] {path}  ({msg.width}x{msg.height} {msg.encoding})", flush=True)
        if self.n >= self.target:
            print("done", flush=True)
            rclpy.shutdown()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--count", type=int, default=40)
    p.add_argument("--out", default=os.path.expanduser("~/calib_frames"))
    a = p.parse_args()
    os.makedirs(a.out, exist_ok=True)

    rclpy.init()
    node = Capture(a.out, a.interval, a.count)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    print(f"captured {node.n} frames to {a.out}")


if __name__ == "__main__":
    main()
