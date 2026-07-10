#!/usr/bin/env python3
"""Save a raw frame from /cam0/image_raw whenever std_msgs/Empty lands on /calib/snap.

Runs on the drone. PIL only, no OpenCV.
"""
import argparse, os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Int32
from PIL import Image as PILImage

MODES = {
    "rgb8":  (3, (0, 1, 2)),
    "bgr8":  (3, (2, 1, 0)),
    "rgba8": (4, (0, 1, 2)),
    "bgra8": (4, (2, 1, 0)),
    "mono8": (1, (0,)),
}


class Snap(Node):
    def __init__(self, out_dir):
        super().__init__("calib_snap")
        self.out_dir = out_dir
        self.latest = None
        # resume from whatever is already on disk so a restart never overwrites
        self.n = len([f for f in os.listdir(out_dir) if f.endswith(".png")])
        self.create_subscription(Image, "/cam0/image_raw", self._img, qos_profile_sensor_data)
        self.create_subscription(Empty, "/calib/snap", self._snap, 10)
        # publish the REAL saved count so the live view can show ground truth
        self.count_pub = self.create_publisher(Int32, "/calib/count", 10)
        self.create_timer(0.5, self._pub_count)
        print(f"ready — saving to {out_dir} on /calib/snap (starting at {self.n})", flush=True)

    def _pub_count(self):
        m = Int32(); m.data = self.n
        self.count_pub.publish(m)

    def _img(self, msg):
        self.latest = msg

    def _snap(self, _):
        # One bad frame must never kill the node — that is what silently lost
        # a whole capture run. Catch everything, keep serving.
        try:
            msg = self.latest
            if msg is None:
                print("snap ignored: no frame yet", flush=True)
                return
            if msg.encoding not in MODES:
                print(f"snap ignored: unhandled encoding {msg.encoding}", flush=True)
                return
            ch, order = MODES[msg.encoding]
            buf = bytes(msg.data)
            if ch == 1:
                img = PILImage.frombytes("L", (msg.width, msg.height), buf)
            else:
                src = PILImage.frombytes("RGBA" if ch == 4 else "RGB", (msg.width, msg.height), buf)
                b = src.split()
                img = PILImage.merge("RGB", tuple(b[i] for i in order))
            path = os.path.join(self.out_dir, f"frame_{self.n:03d}.png")
            img.save(path)
            self.n += 1
            self._pub_count()
            print(f"saved {path}  ({self.n} total)", flush=True)
        except Exception as e:
            print(f"snap error (node stays up): {e!r}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.expanduser("~/calib_frames"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = Snap(a.out)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    print(f"captured {node.n} frames")


if __name__ == "__main__":
    main()
