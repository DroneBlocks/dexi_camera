# Guided ChArUco calibration

A faster way to make a `camera_info` file than the checkerboard tools one level up.
The browser shows an outline of where to hold the board next; line the board up with
it and hold still, and the frame is captured automatically. About 3 minutes for 19
poses. The solve drops bad frames and checks the result before writing the YAML.

The drone only streams frames. Detection and the solve run on your Mac/laptop,
because ChArUco needs OpenCV 4.7 or newer and the drone has 4.6.

## What you need

- The board: `charuco_letter.pdf` (6x8 squares, 30 mm squares, 22 mm markers,
  DICT_4X4_50). Print on letter paper at 100 % and check the 100 mm scale bar with a
  ruler. Tape it to something flat and rigid. A streaky print loses markers.
- On your Mac/laptop: `pip install opencv-python numpy` (OpenCV 4.7 or newer).
- Decent Wi-Fi to the drone. The preview is about 45 KB per frame; a weak signal
  makes it laggy.

## Steps

1. On the drone, free the camera and start the frame server in the same sensor mode
   the drone flies with:

   ```bash
   sudo systemctl stop dexi.service
   python3 cam_server.py --width 640 --height 480 --mode 1640:1232:10   # IMX219 example
   ```

   `--mode` matters. A calibration only holds for the sensor mode, crop and output size
   it was captured through, so match what `camera_ros` uses (its log prints
   `Selected sensor format`).

2. On your Mac:

   ```bash
   python3 calib_guided.py --host <drone-ip> --name <camera>_<model> --hfov 62 --preview 640
   ```

   Open http://localhost:8099. `--hfov` is a rough starting guess for the lens; it is
   refitted after 6 captures. Keys: space = capture now, s = skip pose, b = back,
   m = mirror view.

3. When the last pose is captured it solves and writes `<name>_<W>x<H>.yaml` and a
   report into the frames folder. Copy the YAML into `../../config/`.

4. Restart the drone stack: `sudo systemctl start dexi.service`.

Re-solve saved frames without the drone: `python3 calib_guided.py --solve <frames-dir> --name <name>`.

## Reading the report

It prints PASS/FAIL for: RMS under 0.30 px, holdout under 2x RMS, fx/fy within 1 %,
board corners in every part of the image, and whether the simple 5-coefficient model
is enough. All PASS means use it. Results so far:

| camera | resolution | RMS | fx | FOV |
|---|---|---|---|---|
| InnoMaker OV9281 (VIO) | 1280x800 | 0.21 px | 1123.6 | 59 x 39 deg |
| Arducam B0390 (IMX219) | 640x480 | 0.16 px | 514.3 | 64 x 50 deg |

The B0390 matched `picam_2.1_csi.yaml` within about 1 % in focal length, so it is a
drop-in replacement for the Pi Camera v2.1.
