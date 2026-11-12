# Camera Intrinsic Calibration

Tools to compute the intrinsic calibration (`camera_info`) for a DEXI CSI camera and
drop the result into [`../config/`](../config/).

The calibration you produce is only valid for the **exact pipeline it was captured
through** — the sensor mode, the ISP crop, the output resolution, *and the image
orientation*. If any of those change (different overlay, different `rotation=`,
different `width`/`height`), recalibrate. A number borrowed from another mode is
simply wrong, however carefully it was measured.

## Why this instead of the ROS `cameracalibrator` GUI

Same solver underneath — both call OpenCV's `cv2.calibrateCamera`. The differences:

- **Runs split across two machines.** The Pi's `cv2` is broken by a numpy version
  conflict, and your laptop usually has no ROS. So the drone does the minimum (saves
  raw frames), and OpenCV runs on your Mac/laptop.
- **Better corner detector** — `findChessboardCornersSB` (2018) instead of the classic
  detector, so subpixel corners are more accurate and more robust to blur/perspective.
- **It tells you what to do next.** A live browser view detects the board, and captures
  automatically only when the pose is sharp, still, and genuinely new. On-screen
  **tilt** and **size** coverage meters plus directive hints ("tilt more", "move to
  bottom-left") replace guessing at the ROS GUI's four sliders.

## What you need

- The checkerboard: **7×9 interior corners, 15 mm squares**
  (`dexi/docs/assets/calib.io_checker_200x150_8x10_15.pdf`). Print at 100 %, tape it to
  something rigid — a curled sheet quietly corrupts the distortion fit.
- On your Mac/laptop: `pip install pymavlink pyserial paramiko websocket-client opencv-python-headless numpy`

## Procedure

### 1. Start the capture node on the drone

Saves the **raw** frame each time it receives a `/calib/snap` trigger, and publishes
the real saved count on `/calib/count`.

```bash
ssh dexi@192.168.4.1              # password: dexi
sudo bash -c 'source /home/dexi/ros2_jazzy/install/setup.bash
source /home/dexi/dexi_ws/install/setup.bash
mkdir -p /home/dexi/calib_frames; rm -f /home/dexi/calib_frames/*.png
cd /home/dexi; HOME=/home/dexi setsid python3 /home/dexi/calib_snap.py </dev/null >/home/dexi/snap.log 2>&1 &'
```

Run it as root. `dexi.service` runs the camera as root, and a node started as the `dexi`
user can't see root's topics, so the live view sits on "WAITING for snap node".

### 2. Start the live view on your laptop, then open the browser

```bash
python3 calib_live.py --host 192.168.4.1 --target 40
# open http://localhost:8099
```

Hold up the board and move it around. The overlay tells you what's blocking each
capture; the 6×6 grid fills in green as corners land in each region; the **tilt** and
**size** meters fill as you vary the angle and distance.

**Goal: all 36 grid cells green, both meters full.** Edge/corner coverage and tilt
variety are what pin down `cx`, `cy`, and the distortion terms — skipping them is how
you get a low-error fit that's still wrong. The counter (`saved N/40`) reflects frames
actually written on the drone, so if the drone stops saving, the counter stops.

### 3. Pull the frames and solve (on your laptop)

```bash
python3 fetch_frames.py 192.168.4.1 /home/dexi/calib_frames ./frames
python3 calibrate.py ./frames --name picam_3_arducam_640
```

The solver prints a coverage grid, RMS reprojection error, per-view error (worst frame
named), a 5-frame holdout check, the intrinsics, the implied FOV, and pass/fail gates.
It writes a ROS `camera_info` YAML.

Trust the fit when: **RMS < 0.30 px**, holdout error similar to fit error, `fx/fy`
within ~1 %, and no empty coverage cells. A large off-center `cx`/`cy` is not
automatically wrong — the IMX708 640×480 mode has a real ~+105 px horizontal offset
from its asymmetric sensor crop; confirm by checking that forcing the principal point
to centre makes RMS noticeably worse.

### 4. Install

Copy the YAML into [`../config/`](../config/), point `camera_info_url` in the platform
launch (`dexi_bringup/launch/dexi_bringup_<platform>.launch.py`) at it — **in both the
`src` and `install` trees** — and restart `dexi.service`. Verify the new `fx` appears
in `/cam0/camera_info`.

## Reference result — Arducam fixed-focus IMX708, upright, 640×480

Produced with this tool (`config/picam_3_arducam_640.yaml`):

```
fx = 635.22   fy = 638.74      cx = 424.12   cy = 232.78
D  = [0.0754, -0.0404, -0.0028, -0.0029, 0]   plumb_bob
RMS 0.256 px   holdout 0.213 px   38 frames   HFOV 53.5°
```

Note this is ~53° HFOV / `fx≈635`, **not** the ~36° / `fx≈978` an older tag-based
estimate suggested — that estimate was captured through a different (more cropped)
sensor mode and did not transfer. Same lesson as the header: calibrate the pipeline you
actually ship.

## Files

| file | runs on | purpose |
|---|---|---|
| `calib_live.py` | laptop | live browser view + auto-capture (client-pull MJPEG, OpenCV detect) |
| `calib_snap.py` | drone | saves raw PNG on `/calib/snap`, publishes `/calib/count` |
| `calibrate.py` | laptop | offline solver → `camera_info` YAML + quality gates |
| `fetch_frames.py` | laptop | sftp the PNGs back |
| `calib_capture.py` | drone | fallback: dumb interval capture, no live view |

## Reference result — IMX708 Wide, upright, 768×432 (sensor_mode 2304:1296)

`config/picam_3_wide_768x432.yaml`. The 120° lens bends too hard for `plumb_bob`
(RMS 0.36 px, fails the gate), so this file uses OpenCV's rational model
(`CALIB_RATIONAL_MODEL`, ROS `rational_polynomial`, 8 coefficients):

```
fx = 336.57   fy = 338.69      cx = 386.22   cy = 210.83
RMS 0.113 px   holdout 0.099 px   40 frames   HFOV ~99°
```

Tape check: 3⅞" tag on the floor, camera 45" up, range from the tag pose = 45.1".
`apriltag_ros` ignores the distortion terms (it runs `solvePnP` on the raw image with
`P` only), so tags near the image edge still carry some pose error.
`calibrate.py` fits `plumb_bob` only; the rational fit was done with the same frames and
`cv2.calibrateCamera(..., flags=cv2.CALIB_RATIONAL_MODEL)`.
