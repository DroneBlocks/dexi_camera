#!/usr/bin/env python3
"""Guided ChArUco calibration. Runs on the Mac; open http://localhost:8099

The CM5 runs cam_server.py. This script pulls the preview, detects the ChArUco board, and
draws a ghost outline of the next pose on a mirrored view. Line the board up with the
ghost and hold still: it grabs that exact frame at full resolution and moves to the next
pose. After 6 captures it fits a rough lens model so the ghosts match the real lens.
At the end it solves and writes a ROS camera_info YAML.

  python3 calib_guided.py --host 192.168.68.60            # capture + solve
  python3 calib_guided.py --solve frames/ --name ov9281   # solve existing frames only

Keys in the browser: space = capture now, s = skip pose, b = back one pose, m = mirror.
"""
import argparse, json, math, os, threading, time, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- board

def load_board(path):
    b = json.load(open(path))
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, b["dictionary"]))
    board = cv2.aruco.CharucoBoard((b["squares_x"], b["squares_y"]),
                                   b["square_mm"] / 1000, b["marker_mm"] / 1000, d)
    board.setLegacyPattern(b["legacy_pattern"])
    W, H = b["squares_x"] * b["square_mm"] / 1000, b["squares_y"] * b["square_mm"] / 1000
    return board, W, H


def detect(detector, board, gray):
    cc, ids, _, _ = detector.detectBoard(gray)
    if ids is None or len(ids) < 4:
        return None, None
    return cc.reshape(-1, 2), ids.reshape(-1)


# ---------------------------------------------------------------- poses
# (u, v) = where the board centre should sit (0..1 of the image, camera view, not mirrored)
# z = distance in m, rx = tilt top/bottom, ry = tilt left/right, rz = in-plane spin (deg)
POSES = [
    (0.50, 0.50, 0.40, 0, 0, 0),
    (0.50, 0.50, 0.25, 0, 0, 0),
    (0.50, 0.50, 0.35, 35, 0, 0),
    (0.50, 0.50, 0.35, -35, 0, 0),
    (0.50, 0.50, 0.35, 0, 35, 0),
    (0.50, 0.50, 0.35, 0, -35, 0),
    (0.27, 0.27, 0.33, -20, 20, 0),
    (0.73, 0.27, 0.33, -20, -20, 0),
    (0.73, 0.73, 0.33, 20, -20, 0),
    (0.27, 0.73, 0.33, 20, 20, 0),
    (0.50, 0.20, 0.33, -25, 0, 0),
    (0.88, 0.50, 0.30, 0, -25, 0),
    (0.50, 0.80, 0.33, 25, 0, 0),
    (0.12, 0.50, 0.30, 0, 25, 0),
    (0.10, 0.14, 0.28, -15, 15, 15),
    (0.90, 0.14, 0.28, -15, -15, -15),
    (0.90, 0.86, 0.28, 15, -15, 15),
    (0.10, 0.86, 0.28, 15, 15, -15),
    (0.50, 0.50, 0.55, 0, 0, 30),
]

MATCH_OK = 0.07      # mean corner distance / image diagonal
STILL_PX = 2.0       # preview px of mean corner motion between frames
STILL_FRAMES = 3
SHARP_MIN = 25.0
MIN_CORNERS_FULL = 10


def rot(rx, ry, rz):
    R, _ = cv2.Rodrigues(np.array([math.radians(rx), 0, 0]))
    Ry, _ = cv2.Rodrigues(np.array([0, math.radians(ry), 0]))
    Rz, _ = cv2.Rodrigues(np.array([0, 0, math.radians(rz)]))
    return R @ Ry @ Rz


def ghost_quad(pose, W, H, K, D, size):
    u, v, z, rx, ry, rz = pose
    w, h = size
    obj = np.array([[-W / 2, -H / 2, 0], [W / 2, -H / 2, 0], [W / 2, H / 2, 0], [-W / 2, H / 2, 0]])
    # poses were designed for a ~95 deg lens; scale distance so the board keeps the same
    # size on screen whatever the real focal length turns out to be
    z = z * K[0, 0] / ((w / 2) / math.tan(math.radians(95) / 2))
    ray = np.linalg.inv(K) @ np.array([u * w, v * h, 1.0])
    t = ray / ray[2] * z
    rv, _ = cv2.Rodrigues(rot(rx, ry, rz))
    pts, _ = cv2.projectPoints(obj, rv, t, K, D)
    q = pts.reshape(4, 2)
    if not cv2.isContourConvex(q.astype(np.float32)):
        # distortion model is extrapolating badly here: fall back to plain pinhole
        pts, _ = cv2.projectPoints(obj, rv, t, K, np.zeros(5))
        q = pts.reshape(4, 2)
    return q


def sane_radial(K, D, size):
    """True if r_d = r(1 + k1 r^2 + k2 r^4) keeps increasing out past the image corner."""
    k1, k2 = float(D.ravel()[0]), float(D.ravel()[1])
    rmax = 1.5 * np.hypot(size[0] / 2 / K[0, 0], size[1] / 2 / K[1, 1])
    r = np.linspace(0, rmax, 200)
    rd = r * (1 + k1 * r ** 2 + k2 * r ** 4)
    return bool(np.all(np.diff(rd) > 0))


def live_quad(board, W, H, corners, ids):
    obj = board.getChessboardCorners()[ids][:, :2]
    if len(obj) < 6:
        return None
    Hm, _ = cv2.findHomography(obj, corners)
    if Hm is None:
        return None
    # same corner order as ghost_quad: board origin is top-left in board coords
    q = np.array([[0, 0], [W, 0], [W, H], [0, H]], np.float64)
    return cv2.perspectiveTransform(q.reshape(-1, 1, 2), Hm).reshape(4, 2)


def quad_dist(a, b, diag):
    return min(np.linalg.norm(np.roll(a, k, axis=0) - b, axis=1).mean() for k in range(4)) / diag


def align(a, b):
    """Return a cyclically shifted to best match b (board can be held any way up)."""
    k = min(range(4), key=lambda k: np.linalg.norm(np.roll(a, k, axis=0) - b, axis=1).mean())
    return np.roll(a, k, axis=0)


def hints(live, ghost, w, h, mirror):
    """Short instructions in the coordinates the user SEES (mirrored view)."""
    def disp(q):
        q = q.copy()
        if mirror:
            q[:, 0] = w - q[:, 0]
        return q
    L, G = disp(live), disp(ghost)
    out = []
    dc = (G.mean(0) - L.mean(0)) / np.array([w, h])
    if abs(dc[0]) > 0.04:
        out.append("move " + ("right" if dc[0] > 0 else "left"))
    if abs(dc[1]) > 0.04:
        out.append("move " + ("down" if dc[1] > 0 else "up"))
    ar = cv2.contourArea(G.astype(np.float32)) / max(cv2.contourArea(L.astype(np.float32)), 1)
    if ar > 1.3:
        out.append("closer")
    elif ar < 0.77:
        out.append("farther")

    def sides(q):
        e = [(q[i], q[(i + 1) % 4]) for i in range(4)]
        mid = [((p + r) / 2) for p, r in e]
        ln = [np.linalg.norm(p - r) for p, r in e]
        left = min(range(4), key=lambda i: mid[i][0]); right = max(range(4), key=lambda i: mid[i][0])
        top = min(range(4), key=lambda i: mid[i][1]); bot = max(range(4), key=lambda i: mid[i][1])
        return ln[left] / ln[right], ln[top] / ln[bot]
    gl, gt = sides(G)
    ll, lt = sides(L)
    if ll < gl * 0.88:
        out.append("bring LEFT edge toward camera")
    elif ll > gl * 1.12:
        out.append("bring RIGHT edge toward camera")
    if lt < gt * 0.88:
        out.append("bring TOP edge toward camera")
    elif lt > gt * 1.12:
        out.append("bring BOTTOM edge toward camera")
    return out


# ---------------------------------------------------------------- solve

def collect(board, files):
    det = cv2.aruco.CharucoDetector(board)
    objs, imgs, names, size = [], [], [], None
    for f in files:
        g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        size = g.shape[::-1]
        c, i = detect(det, board, g)
        if c is None or len(i) < MIN_CORNERS_FULL:
            continue
        o, p = board.matchImagePoints(c.reshape(-1, 1, 2), i.reshape(-1, 1))
        objs.append(o.astype(np.float32)); imgs.append(p.astype(np.float32)); names.append(os.path.basename(f))
    return objs, imgs, names, size


def fit(objs, imgs, size, flags):
    rms, K, D, rv, tv = cv2.calibrateCamera(objs, imgs, size, None, None, flags=flags)
    per = []
    for o, p, r, t in zip(objs, imgs, rv, tv):
        q, _ = cv2.projectPoints(o, r, t, K, D)
        per.append(float(np.sqrt(np.mean(np.sum((q.reshape(-1, 2) - p.reshape(-1, 2)) ** 2, 1)))))
    return rms, K, D, per


def holdout_err(objs, imgs, K, D):
    errs = []
    for o, p in zip(objs, imgs):
        ok, r, t = cv2.solvePnP(o, p, K, D)
        q, _ = cv2.projectPoints(o, r, t, K, D)
        errs.append(float(np.sqrt(np.mean(np.sum((q.reshape(-1, 2) - p.reshape(-1, 2)) ** 2, 1)))))
    return float(np.mean(errs)) if errs else float("nan")


def coverage(imgs, size, n=6):
    g = np.zeros((n, n), int)
    for p in imgs:
        for x, y in p.reshape(-1, 2):
            g[min(int(y / size[1] * n), n - 1), min(int(x / size[0] * n), n - 1)] += 1
    return g


def solve(board, frame_dir, name, out_dir):
    files = sorted(os.path.join(frame_dir, f) for f in os.listdir(frame_dir) if f.endswith(".png"))
    objs, imgs, names, size = collect(board, files)
    rep = [f"{len(objs)} usable frames of {len(files)}, image {size[0]}x{size[1]}"]
    # drop frames whose per-view error is far above the rest (bad detection, curled paper, blur)
    if len(objs) >= 8:
        _, _, _, per = fit(objs, imgs, size, cv2.CALIB_FIX_K3)
        lim = max(1.0, 3 * float(np.median(per)))
        keep = [k for k, e in enumerate(per) if e <= lim]
        dropped = [f"{names[k]} ({per[k]:.2f} px)" for k in range(len(objs)) if k not in keep]
        if dropped and len(keep) >= 8:
            rep.append("dropped outliers: " + ", ".join(dropped))
            objs, imgs, names = [objs[k] for k in keep], [imgs[k] for k in keep], [names[k] for k in keep]
    if len(objs) < 8:
        rep.append("FAIL: need at least 8 usable frames")
        return None, "\n".join(rep)
    # radtan (k1, k2, p1, p2; k3 fixed at 0): the model VIO tools expect. The rational
    # model is fitted only as a check: if it beats radtan clearly, the lens needs more terms.
    rat = cv2.CALIB_FIX_K3
    rms_rat, *_ = fit(objs, imgs, size, cv2.CALIB_RATIONAL_MODEL)
    rms, K, D, per = fit(objs, imgs, size, rat)
    hold = [i for i in range(len(objs)) if i % 5 == 4]
    train = [i for i in range(len(objs)) if i % 5 != 4]
    _, Kh, Dh, _ = fit([objs[i] for i in train], [imgs[i] for i in train], size, rat)
    ho = holdout_err([objs[i] for i in hold], [imgs[i] for i in hold], Kh, Dh)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    hfov = math.degrees(2 * math.atan(size[0] / 2 / fx)); vfov = math.degrees(2 * math.atan(size[1] / 2 / fy))
    cov = coverage(imgs, size)
    worst = int(np.argmax(per))
    rep += [
        f"RMS radtan {rms:.3f} px   (rational_polynomial {rms_rat:.3f} px)",
        f"holdout {ho:.3f} px on {len(hold)} frames",
        f"fx {fx:.2f}  fy {fy:.2f}  cx {cx:.2f}  cy {cy:.2f}   fx/fy {fx/fy:.4f}",
        f"FOV (pinhole) {hfov:.1f} x {vfov:.1f} deg",
        f"D (k1, k2, p1, p2, k3=0) {np.round(D.ravel()[:5], 5).tolist()}",
        f"worst frame {names[worst]} {per[worst]:.3f} px",
        "coverage (corners per cell):",
    ] + ["  " + " ".join(f"{v:4d}" for v in row) for row in cov]
    gates = [("RMS < 0.30 px", rms < 0.30), ("holdout < 2x RMS", ho < 2 * rms + 0.05),
             ("fx/fy within 1%", abs(fx / fy - 1) < 0.01), ("no empty cells", int((cov == 0).sum()) == 0),
             ("radtan is enough (rational < 0.05 px better)", rms - rms_rat < 0.05)]
    rep += [("PASS " if ok else "FAIL ") + g for g, ok in gates]
    d5 = D.ravel()[:4].tolist() + [0.0]
    yaml = (f"image_width: {size[0]}\nimage_height: {size[1]}\ncamera_name: {name}\n"
            f"camera_matrix:\n  rows: 3\n  cols: 3\n  data: [{fx:.6f}, 0.0, {cx:.6f}, 0.0, {fy:.6f}, {cy:.6f}, 0.0, 0.0, 1.0]\n"
            f"distortion_model: plumb_bob\ndistortion_coefficients:\n  rows: 1\n  cols: 5\n"
            f"  data: [{', '.join(f'{v:.6f}' for v in d5)}]\n"
            f"rectification_matrix:\n  rows: 3\n  cols: 3\n  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n"
            f"projection_matrix:\n  rows: 3\n  cols: 4\n  data: [{fx:.6f}, 0.0, {cx:.6f}, 0.0, 0.0, {fy:.6f}, {cy:.6f}, 0.0, 0.0, 0.0, 1.0, 0.0]\n")
    path = os.path.join(out_dir, f"{name}_{size[0]}x{size[1]}.yaml")
    open(path, "w").write(yaml)
    open(os.path.join(out_dir, f"{name}_{size[0]}x{size[1]}_report.txt"), "w").write("\n".join(rep) + "\n")
    rep.append(f"wrote {path}")
    return (K, D), "\n".join(rep)


# ---------------------------------------------------------------- live

class Guide:
    def __init__(self, a):
        self.host, self.a = a.host, a
        self.board, self.W, self.H = load_board(a.board)
        self.det = cv2.aruco.CharucoDetector(self.board)
        self.frames = a.frames
        os.makedirs(self.frames, exist_ok=True)
        self.idx = 0
        self.mirror = True
        self.full = None           # full-res (w, h)
        self.K = self.D = None     # full-res model
        self.objs, self.imgs = [], []
        self.still = 0
        self.prev = None
        self.force = False
        self.state = {"msg": "connecting to camera…"}
        self.jpeg = None
        self.lock = threading.Lock()
        self.report = None
        self.solving = False
        old = sorted(os.path.join(self.frames, f) for f in os.listdir(self.frames) if f.endswith(".png"))
        if old:
            self.objs, self.imgs, _, self.full = collect(self.board, old)
            self.idx = min(len(old), len(POSES))
            self.refit()
            print(f"resumed: {len(self.objs)} frames on disk, starting at pose {self.idx + 1}", flush=True)

    def model(self, pw, ph):
        """Camera model at preview size."""
        fw, fh = self.full
        s = pw / fw
        if self.K is None:
            fx = (fw / 2) / math.tan(math.radians(self.a.hfov) / 2)
            K = np.array([[fx, 0, fw / 2], [0, fx, fh / 2], [0, 0, 1.0]])
            D = np.zeros(5)
        else:
            K, D = self.K, self.D
        Ks = K.copy(); Ks[:2] *= s
        return Ks, D

    def refit(self):
        if len(self.objs) < 6:
            return
        try:
            # guide model only: k1, k2 and no tangential, so it can't fold at the edges
            # it hasn't seen yet. The final solve still uses the full rational model.
            _, K, D, _ = fit(self.objs, self.imgs, self.full,
                             cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)
            if 0.3 * self.full[0] < K[0, 0] < 3 * self.full[0]:
                self.K = K
                self.D = D if sane_radial(K, D, self.full) else np.zeros(5)
        except cv2.error:
            pass

    def capture(self, seq):
        try:
            png = urllib.request.urlopen(f"http://{self.host}:8098/snap?seq={seq}", timeout=5).read()
        except Exception as e:
            return f"snap failed: {e}"
        g = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_GRAYSCALE)
        c, i = detect(self.det, self.board, g)
        if c is None or len(i) < MIN_CORNERS_FULL:
            return "full-res frame had too few corners, try again"
        n = len([f for f in os.listdir(self.frames) if f.endswith(".png")])
        open(os.path.join(self.frames, f"frame_{n:03d}.png"), "wb").write(png)
        o, p = self.board.matchImagePoints(c.reshape(-1, 1, 2), i.reshape(-1, 1))
        self.objs.append(o.astype(np.float32)); self.imgs.append(p.astype(np.float32))
        threading.Thread(target=self.refit, daemon=True).start()
        self.idx += 1
        return None

    def finish(self):
        self.solving = True
        _, rep = solve(self.board, self.frames, self.a.name, self.frames)
        self.report = rep
        print(rep, flush=True)
        self.solving = False

    def key(self, k):
        if k == "m":
            self.mirror = not self.mirror
        elif k == "s" and self.idx < len(POSES):
            self.idx += 1
        elif k == "b" and self.idx > 0:
            self.idx -= 1
        elif k == " ":
            self.force = True

    def run(self):
        while True:
            try:
                r = urllib.request.urlopen(f"http://{self.host}:8098/preview.jpg?w=" + str(self.a.preview), timeout=3)
                buf = r.read()
                seq = int(r.headers["X-Seq"])
                self.full = tuple(int(v) for v in r.headers["X-Full"].split("x"))
            except Exception as e:
                with self.lock:
                    self.state = {"msg": f"no camera: {e}"}
                time.sleep(0.5)
                continue
            img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_GRAYSCALE)
            st = self.step(img, seq)
            with self.lock:
                self.jpeg, self.state = buf, st

    def step(self, g, seq):
        ph, pw = g.shape
        diag = math.hypot(pw, ph)
        st = {"w": pw, "h": ph, "mirror": self.mirror, "idx": self.idx, "total": len(POSES),
              "captured": len(self.objs), "fitted": self.K is not None}
        # coverage of captured corners, in preview px
        if self.full:
            s = pw / self.full[0]
            st["cov"] = [[float(x * s), float(y * s)] for p in self.imgs for x, y in p.reshape(-1, 2)[::3]]
        if self.idx >= len(POSES):
            if self.report is None and not self.solving:
                threading.Thread(target=self.finish, daemon=True).start()
            st["msg"] = "solving…" if self.report is None else "done"
            st["report"] = self.report
            return st
        K, D = self.model(pw, ph)
        ghost = ghost_quad(POSES[self.idx], self.W, self.H, K, D, (pw, ph))
        st["ghost"] = ghost.tolist()
        c, i = detect(self.det, self.board, g)
        if c is None:
            self.still, self.prev = 0, None
            st["msg"] = "show the board to the camera"
            return st
        st["corners"] = c.tolist()
        lq = live_quad(self.board, self.W, self.H, c, i)
        if lq is None:
            st["msg"] = "need more of the board in view"
            return st
        lq = align(lq, ghost)
        st["live"] = lq.tolist()
        d = quad_dist(lq, ghost, diag)
        st["score"] = max(0.0, 1 - d / (MATCH_OK * 3))
        motion = 99.0
        if self.prev is not None:
            pi, pc = self.prev
            common, a_i, b_i = np.intersect1d(i, pi, return_indices=True)
            if len(common) >= 4:
                motion = float(np.linalg.norm(c[a_i] - pc[b_i], axis=1).mean())
        self.prev = (i, c)
        x0, y0 = c.min(0).astype(int); x1, y1 = c.max(0).astype(int)
        roi = g[max(y0, 0):y1, max(x0, 0):x1]
        sharp = cv2.Laplacian(roi, cv2.CV_64F).var() if roi.size else 0
        if d < MATCH_OK or self.force:
            if motion < STILL_PX and sharp > SHARP_MIN:
                self.still += 1
            else:
                self.still = 0
            st["hint"] = ["hold still"] if motion >= STILL_PX else (["too blurry, more light"] if sharp <= SHARP_MIN else [])
            if self.still >= STILL_FRAMES or (self.force and motion < STILL_PX * 2):
                self.force = False
                err = self.capture(seq)
                self.still, self.prev = 0, None
                st["flash"] = err is None
                st["msg"] = err or "captured"
                st["idx"] = self.idx
                return st
            st["msg"] = "hold it there…"
        else:
            self.still = 0
            st["hint"] = hints(lq, ghost, pw, ph, self.mirror)
            st["msg"] = "match the outline"
        return st


PAGE = r"""<!doctype html><html><head><meta charset=utf-8><title>DEXI calibration</title>
<meta name=viewport content="width=device-width,initial-scale=1">
<style>
:root{--bg:#f6f7f9;--fg:#14171c;--mut:#5c6470;--ok:#16a34a;--warn:#d97706;--bad:#dc2626;--card:#fff}
body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.4 -apple-system,system-ui,sans-serif}
#wrap{max-width:1300px;margin:0 auto;padding:12px 16px}
#top{display:flex;align-items:baseline;gap:16px;flex-wrap:wrap}
#pose{font-size:28px;font-weight:700}
#msg{font-size:22px;font-weight:600}
#hint{font-size:30px;font-weight:700;color:var(--warn);min-height:42px;margin:4px 0}
#bar{height:8px;background:#e3e6ea;border-radius:4px;overflow:hidden;margin:6px 0 10px}
#bar i{display:block;height:100%;background:var(--ok);width:0}
canvas{max-width:100%;max-height:calc(100vh - 190px);border-radius:8px;background:#000;display:block;margin:0 auto}
#keys{color:var(--mut);margin-top:8px}
pre{background:var(--card);padding:12px;border-radius:8px;border:1px solid #e3e6ea;font-size:13px;overflow:auto}
</style></head><body><div id=wrap>
<div id=top><span id=pose></span><span id=msg></span><span id=fit style="color:var(--mut)"></span></div>
<div id=hint></div><div id=bar><i></i></div>
<canvas id=c></canvas>
<div id=keys>space = capture now · s = skip pose · b = back · m = mirror view</div>
<pre id=rep hidden></pre>
</div><script>
const cv=document.getElementById('c'),cx=cv.getContext('2d');let flashT=0;
const img=new Image();
document.addEventListener('keydown',e=>{const k=e.key===' '?' ':e.key.toLowerCase();
 if(' smb'.includes(k)){e.preventDefault();fetch('/key?k='+encodeURIComponent(k));}});
function poly(q,M,W){cx.beginPath();q.forEach((p,i)=>{const x=M?W-p[0]:p[0];i?cx.lineTo(x,p[1]):cx.moveTo(x,p[1])});cx.closePath();}
async function loop(){
 while(true){
  try{
   const r=await fetch('/frame?'+Date.now(),{cache:'no-store'});
   const st=JSON.parse(r.headers.get('X-State')||'{}');
   const b=await r.blob();
   if(b.size){const u=URL.createObjectURL(b);await new Promise(res=>{img.onload=res;img.onerror=res;img.src=u});URL.revokeObjectURL(u);}
   draw(st);
  }catch(e){await new Promise(r=>setTimeout(r,300));}
  await new Promise(r=>setTimeout(r,40));
 }
}
function draw(st){
 const W=st.w||640,H=st.h||400,M=st.mirror;
 if(cv.width!==W){cv.width=W;cv.height=H;}
 cx.save();if(M){cx.translate(W,0);cx.scale(-1,1);}cx.drawImage(img,0,0,W,H);cx.restore();
 (st.cov||[]).forEach(p=>{cx.fillStyle='rgba(34,197,94,.55)';cx.fillRect((M?W-p[0]:p[0])-1.5,p[1]-1.5,3,3);});
 if(st.ghost){const s=st.score||0;const col=s>.66?'#22c55e':s>.33?'#f59e0b':'#ef4444';
  cx.setLineDash([10,7]);cx.lineWidth=4;cx.strokeStyle=col;poly(st.ghost,M,W);cx.stroke();cx.setLineDash([]);
  cx.fillStyle=col+'22';cx.fill();}
 if(st.live){cx.lineWidth=2;cx.strokeStyle='#38bdf8';poly(st.live,M,W);cx.stroke();
  if(st.ghost){const c=q=>q.reduce((a,p)=>[a[0]+p[0]/4,a[1]+p[1]/4],[0,0]);let a=c(st.live),g=c(st.ghost);
   if(M){a[0]=W-a[0];g[0]=W-g[0];}const dx=g[0]-a[0],dy=g[1]-a[1];
   if(Math.hypot(dx,dy)>15){cx.strokeStyle='#38bdf8';cx.lineWidth=3;cx.beginPath();cx.moveTo(a[0],a[1]);cx.lineTo(g[0],g[1]);cx.stroke();
    const an=Math.atan2(dy,dx);cx.beginPath();cx.moveTo(g[0],g[1]);cx.lineTo(g[0]-14*Math.cos(an-.5),g[1]-14*Math.sin(an-.5));
    cx.lineTo(g[0]-14*Math.cos(an+.5),g[1]-14*Math.sin(an+.5));cx.closePath();cx.fillStyle='#38bdf8';cx.fill();}}}
 (st.corners||[]).forEach(p=>{cx.fillStyle='#38bdf8';cx.fillRect((M?W-p[0]:p[0])-2,p[1]-2,4,4);});
 if(st.flash)flashT=Date.now();
 if(Date.now()-flashT<250){cx.fillStyle='rgba(255,255,255,.5)';cx.fillRect(0,0,W,H);}
 const n=Math.min((st.idx||0)+1,st.total||0);
 document.getElementById('pose').textContent=st.total?(st.idx>=st.total?'All poses done':`Pose ${n} / ${st.total}`):'';
 document.getElementById('msg').textContent=st.msg||'';
 document.getElementById('fit').textContent=st.captured!=null?`${st.captured} captured${st.fitted?' · lens model fitted':''}`:'';
 document.getElementById('hint').textContent=(st.hint||[]).join(' · ');
 document.querySelector('#bar i').style.width=st.total?(100*Math.min(st.idx,st.total)/st.total)+'%':'0';
 const rp=document.getElementById('rep');if(st.report){rp.hidden=false;rp.textContent=st.report;}
}
loop();
</script></body></html>"""

G = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/frame"):
            with G.lock:
                buf, st = G.jpeg or b"", dict(G.state)
            body = buf
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("X-State", json.dumps(st, separators=(",", ":")))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
        elif self.path.startswith("/key"):
            from urllib.parse import urlparse, parse_qs
            G.key(parse_qs(urlparse(self.path).query).get("k", [""])[0])
            body = b"ok"
            self.send_response(200); self.send_header("Content-Length", "2"); self.end_headers()
        else:
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    global G
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="drone IP running cam_server.py")
    ap.add_argument("--board", default=os.path.join(HERE, "charuco_letter.json"))
    ap.add_argument("--frames", default=os.path.join(HERE, "frames"))
    ap.add_argument("--name", default="camera", help="output file prefix")
    ap.add_argument("--hfov", type=float, default=95.0, help="starting guess until the lens model is fitted")
    ap.add_argument("--port", type=int, default=8099)
    ap.add_argument("--preview", type=int, default=1280, help="preview width; detection runs on it")
    ap.add_argument("--solve", metavar="DIR", help="solve existing frames and exit")
    a = ap.parse_args()
    if a.solve:
        board, _, _ = load_board(a.board)
        _, rep = solve(board, a.solve, a.name, a.solve)
        print(rep)
        return
    G = Guide(a)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"open http://localhost:{a.port}", flush=True)
    G.run()


if __name__ == "__main__":
    main()
