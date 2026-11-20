#!/usr/bin/env python3
"""Camera server for guided calibration. Runs on the CM5 (stdlib + numpy + cv2 for JPEG/PNG).

Reads raw mono frames from rpicam-vid (yuv420, Y plane only) and serves:
  GET /preview.jpg   latest frame, downscaled JPEG; header X-Seq = frame number
  GET /snap?seq=N    full-resolution PNG of frame N (kept in a ring buffer), also saved to --out

The camera must be free: stop dexi.service first (camera_ros holds it).

  python3 cam_server.py [--width 1280 --height 800 --fps 30 --shutter 0 --port 8098]
"""
import argparse, collections, os, subprocess, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import cv2
import numpy as np

RING = 90


class Cam:
    def __init__(self, a):
        self.w, self.h = a.width, a.height
        self.ring = collections.OrderedDict()
        self.seq = 0
        self.lock = threading.Lock()
        self.out = a.out
        os.makedirs(self.out, exist_ok=True)
        cmd = ["rpicam-vid", "-n", "-t", "0", "--width", str(self.w), "--height", str(self.h),
               "--framerate", str(a.fps), "--codec", "yuv420", "--flush", "-o", "-"]
        if a.mode:
            cmd += ["--mode", a.mode]  # match the sensor mode the drone's camera_ros uses
        if a.shutter:
            cmd += ["--shutter", str(a.shutter), "--gain", str(a.gain)]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        threading.Thread(target=self._reader, daemon=True).start()

    def _read_exact(self, n):
        buf = bytearray(n)
        mv, got = memoryview(buf), 0
        while got < n:
            k = self.proc.stdout.readinto(mv[got:])
            if not k:
                raise EOFError("rpicam-vid exited")
            got += k
        return buf

    def _reader(self):
        ysz = self.w * self.h
        frame_sz = ysz * 3 // 2
        while True:
            buf = self._read_exact(frame_sz)
            y = np.frombuffer(buf, np.uint8, ysz).reshape(self.h, self.w).copy()
            with self.lock:
                self.seq += 1
                self.ring[self.seq] = y
                while len(self.ring) > RING:
                    self.ring.popitem(last=False)

    def latest(self):
        with self.lock:
            if not self.ring:
                return None, None
            s = next(reversed(self.ring))
            return s, self.ring[s]

    def get(self, seq):
        with self.lock:
            return self.ring.get(seq)


CAM = None
PREVIEW_W = 640


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body=b"", ctype="text/plain", hdr=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (hdr or {}).items():
            self.send_header(k, v)
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/preview.jpg":
            s, y = CAM.latest()
            if y is None:
                return self._send(503, b"no frame")
            pw = min(int(parse_qs(u.query).get("w", [PREVIEW_W])[0]), y.shape[1])
            ph = int(round(y.shape[0] * pw / y.shape[1]))
            small = y if pw == y.shape[1] else cv2.resize(y, (pw, ph), interpolation=cv2.INTER_AREA)
            ok, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 80])
            return self._send(200, jpg.tobytes(), "image/jpeg",
                              {"X-Seq": str(s), "X-Full": f"{CAM.w}x{CAM.h}"})
        if u.path == "/snap":
            seq = int(parse_qs(u.query).get("seq", ["0"])[0])
            y = CAM.get(seq)
            if y is None:
                return self._send(404, b"frame gone")
            ok, png = cv2.imencode(".png", y)
            n = len([f for f in os.listdir(CAM.out) if f.endswith(".png")])
            with open(os.path.join(CAM.out, f"frame_{n:03d}.png"), "wb") as f:
                f.write(png.tobytes())
            return self._send(200, png.tobytes(), "image/png")
        self._send(404, b"?")


def main():
    global CAM
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=800)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--mode", default="", help="sensor mode W:H:bits, e.g. 1640:1232:10")
    ap.add_argument("--shutter", type=int, default=0, help="us; 0 = auto exposure")
    ap.add_argument("--gain", type=float, default=4.0)
    ap.add_argument("--port", type=int, default=8098)
    ap.add_argument("--out", default=os.path.expanduser("~/vio_calib_frames"))
    a = ap.parse_args()
    CAM = Cam(a)
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), H)
    print(f"serving {a.width}x{a.height} on :{a.port}, saving to {a.out}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
