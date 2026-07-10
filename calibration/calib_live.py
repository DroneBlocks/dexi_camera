#!/usr/bin/env python3
"""Live checkerboard calibration view. Runs on the Mac; open http://localhost:8099

Subscribes to /cam0/image_raw/compressed over rosbridge for the preview, detects the
board, and when a pose is sharp + still + novel it publishes /calib/snap so the drone
saves the RAW frame. The JPEG preview is never used for the fit.

  python3 calib_live.py --host 192.168.4.1 [--cols 7] [--rows 9] [--target 40]
"""
import argparse, base64, json, math, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from websocket import create_connection

STATE = {"jpeg": None, "lock": threading.Lock()}

GRID = 6
SHARP_MIN = 60.0        # variance of Laplacian on the board region
STILL_MAX = 1.5         # px mean corner motion between consecutive frames
NOVEL_MIN = 0.14        # min Chebyshev distance in (x, y, size, skew) space
SIZE_SPAN_TARGET = 0.22  # want near-vs-far spread (board fill fraction) >= this
SKEW_TARGET = 0.28       # want to reach at least this much tilt

REGION = ["top-left", "top-centre", "top-right",
          "left", "centre", "right",
          "bottom-left", "bottom-centre", "bottom-right"]


def board_params(corners, w, h):
    pts = corners.reshape(-1, 2)
    x = float(pts[:, 0].mean() / w)
    y = float(pts[:, 1].mean() / h)
    area = cv2.contourArea(cv2.convexHull(pts.astype(np.float32)))
    size = float(math.sqrt(area / (w * h)))
    # skew from the board's outer quad: how far the corner angles deviate from 90 deg
    hull = cv2.convexHull(pts.astype(np.float32))
    rect = cv2.minAreaRect(hull)
    (rw, rh) = rect[1]
    ratio = min(rw, rh) / max(rw, rh) if max(rw, rh) > 0 else 1.0
    skew = float(1.0 - ratio)
    return np.array([x, y, size, skew])


class Live:
    def __init__(self, host, pattern, target):
        self.pattern, self.target = pattern, target
        self.accepted = []          # list of param vectors
        self.grid = np.zeros((GRID, GRID), int)
        self.prev_corners = None
        self.n = 0                  # triggers we've fired
        self.saved = 0             # frames the DRONE actually saved (ground truth)
        self.count_seen = False    # have we heard from the snap node at all?
        self.msg = "waiting for board"
        self.ws = create_connection(f"ws://{host}:9090", timeout=15)
        self.ws.send(json.dumps({"op": "advertise", "topic": "/calib/snap", "type": "std_msgs/Empty"}))
        self.ws.send(json.dumps({"op": "subscribe", "topic": "/cam0/image_raw/compressed",
                                 "id": "prev", "throttle_rate": 0, "queue_length": 1}))
        self.ws.send(json.dumps({"op": "subscribe", "topic": "/calib/count",
                                 "id": "cnt", "throttle_rate": 200}))

    def snap(self):
        self.ws.send(json.dumps({"op": "publish", "topic": "/calib/snap", "msg": {}}))

    def novelty(self, p):
        if not self.accepted:
            return 1.0
        return float(min(np.max(np.abs(np.array(self.accepted) - p), axis=1)))

    def hint(self):
        # emptiest 3x3 macro-region of the coverage grid
        macro = self.grid.reshape(3, GRID // 3, 3, GRID // 3).sum(axis=(1, 3))
        i = int(np.argmin(macro))
        return REGION[i]

    def coverage(self):
        # returns (size_cov 0..1, skew_cov 0..1)
        if not self.accepted:
            return 0.0, 0.0
        arr = np.array(self.accepted)
        sizes, skews = arr[:, 2], arr[:, 3]
        size_cov = min(1.0, (sizes.max() - sizes.min()) / SIZE_SPAN_TARGET)
        skew_cov = min(1.0, skews.max() / SKEW_TARGET)
        return float(size_cov), float(skew_cov)

    def redundant_hint(self):
        # when a pose is a duplicate, name the weakest coverage dimension
        size_cov, skew_cov = self.coverage()
        weakest = min((skew_cov, "tilt the board more"),
                      (size_cov, "move closer or farther"),
                      key=lambda t: t[0])
        if weakest[0] < 0.75:
            return weakest[1]
        return f"move to {self.hint()}"

    def run(self):
        while True:
            raw = self.ws.recv()
            m = json.loads(raw)
            topic = m.get("topic")
            if topic == "/calib/count":
                self.saved = int(m["msg"]["data"])
                self.count_seen = True
                continue
            if topic != "/cam0/image_raw/compressed":
                continue
            buf = np.frombuffer(base64.b64decode(m["msg"]["data"]), np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is None:
                continue
            self.process(img)

    def process(self, img):
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCornersSB(gray, self.pattern,
                                                  flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        vis = img.copy()

        # DONE is gated on frames the DRONE actually saved, not triggers fired.
        if not self.count_seen:
            self.msg = "WAITING for snap node on drone — no /calib/count yet"
        elif self.saved >= self.target:
            self.msg = f"DONE — {self.saved} frames saved on drone"
        elif ok:
            cv2.drawChessboardCorners(vis, self.pattern, corners, ok)
            pts = corners.reshape(-1, 2)
            x0, y0 = pts.min(0).astype(int)
            x1, y1 = pts.max(0).astype(int)
            roi = gray[max(y0, 0):y1, max(x0, 0):x1]
            sharp = cv2.Laplacian(roi, cv2.CV_64F).var() if roi.size else 0.0

            motion = 99.0
            if self.prev_corners is not None and self.prev_corners.shape == corners.shape:
                motion = float(np.linalg.norm(pts - self.prev_corners.reshape(-1, 2), axis=1).mean())
            self.prev_corners = corners.copy()

            p = board_params(corners, w, h)
            nov = self.novelty(p)

            # don't outrun the drone: if we've fired well ahead of confirmed
            # saves, pause capturing until saves catch up
            if self.n - self.saved > 2:
                self.msg = f"syncing… fired {self.n}, saved {self.saved}"
            elif sharp < SHARP_MIN:
                self.msg = f"too blurry ({sharp:.0f}) — steady, more light"
            elif motion > STILL_MAX:
                self.msg = f"hold still ({motion:.1f} px)"
            elif nov < NOVEL_MIN:
                self.msg = f"pose already captured — {self.redundant_hint()}"
            else:
                self.snap()
                self.accepted.append(p)
                self.n += 1
                for pt in pts:
                    gx = min(int(pt[0] / w * GRID), GRID - 1)
                    gy = min(int(pt[1] / h * GRID), GRID - 1)
                    self.grid[gy, gx] += 1
                self.msg = f"CAPTURED — saved {self.saved}/{self.target}"
                self.prev_corners = None
        else:
            self.msg = "no board detected"
            self.prev_corners = None

        self.draw(vis, w, h)

    def draw(self, vis, w, h):
        # coverage grid
        overlay = vis.copy()
        for gy in range(GRID):
            for gx in range(GRID):
                x0, y0 = int(gx * w / GRID), int(gy * h / GRID)
                x1, y1 = int((gx + 1) * w / GRID), int((gy + 1) * h / GRID)
                if self.grid[gy, gx] > 0:
                    cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 140, 0), -1)
                cv2.rectangle(vis, (x0, y0), (x1, y1), (60, 60, 60), 1)
        vis = cv2.addWeighted(overlay, 0.22, vis, 0.78, 0)

        empty = int((self.grid == 0).sum())
        bar = int(min(self.saved, self.target) / self.target * (w - 20))
        cv2.rectangle(vis, (10, h - 26), (10 + bar, h - 14), (0, 200, 0), -1)
        cv2.rectangle(vis, (10, h - 26), (w - 10, h - 14), (200, 200, 200), 1)

        colour = (0, 220, 0) if "CAPTURED" in self.msg or "DONE" in self.msg else (0, 200, 255)
        cv2.putText(vis, self.msg, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(vis, self.msg, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 1)
        sub = f"saved {self.saved}/{self.target}   empty cells {empty}/{GRID*GRID}"
        cv2.putText(vis, sub, (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(vis, sub, (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # tilt + size coverage meters (ROS-style, but told to you)
        size_cov, skew_cov = self.coverage()
        for row, (label, cov) in enumerate([("tilt", skew_cov), ("size", size_cov)]):
            by = 64 + row * 16
            cv2.putText(vis, label, (12, by + 9), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
            cv2.putText(vis, label, (12, by + 9), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            x0, x1 = 52, 152
            cv2.rectangle(vis, (x0, by), (x1, by + 10), (120, 120, 120), 1)
            fillc = (0, 200, 0) if cov >= 0.999 else (0, 170, 220)
            cv2.rectangle(vis, (x0, by), (x0 + int(cov * (x1 - x0)), by + 10), fillc, -1)

        vis = cv2.resize(vis, (w * 2, h * 2), interpolation=cv2.INTER_NEAREST)
        okj, jpg = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if okj:
            with STATE["lock"]:
                STATE["jpeg"] = jpg.tobytes()


PAGE = b"""<!doctype html><html><head><meta charset=utf-8>
<title>DEXI calibration</title>
<style>html,body{margin:0;background:#111;height:100%;overflow:hidden}
#v{display:block;max-width:100vw;max-height:100vh;margin:auto;image-rendering:pixelated}</style>
</head><body>
<img id=v>
<script>
// Client-pull: fetch single snapshots back-to-back. No long-lived connection to stall.
const v = document.getElementById('v');
async function loop(){
  while(true){
    try{
      const r = await fetch('/frame.jpg?'+Date.now(), {cache:'no-store'});
      const b = await r.blob();
      const u = URL.createObjectURL(b);
      await new Promise(res=>{ v.onload=()=>{URL.revokeObjectURL(u);res();}; v.onerror=res; v.src=u; });
    }catch(e){ await new Promise(r=>setTimeout(r,300)); }
    await new Promise(r=>setTimeout(r,80));   // ~12 fps
  }
}
loop();
</script>
</body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/frame.jpg"):
            with STATE["lock"]:
                buf = STATE["jpeg"]
            if not buf:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.send_header("Content-Length", str(len(buf)))
            self.end_headers()
            try:
                self.wfile.write(buf)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(PAGE)))
        self.end_headers()
        self.wfile.write(PAGE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.4.1")
    ap.add_argument("--cols", type=int, default=7)
    ap.add_argument("--rows", type=int, default=9)
    ap.add_argument("--target", type=int, default=40)
    ap.add_argument("--port", type=int, default=8099)
    a = ap.parse_args()

    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"open http://localhost:{a.port}  (ctrl-C to stop)")

    Live(a.host, (a.cols, a.rows), a.target).run()


if __name__ == "__main__":
    main()
