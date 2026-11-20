#!/usr/bin/env python3
"""Generate a letter-size ChArUco calibration target as a true-scale PDF.

The board settings below are the single source of truth: the detector must use the
same squares, square/marker size and dictionary. They are also written to charuco.json.

  python3 make_charuco.py [--out charuco_letter.pdf]
"""
import argparse, json, os

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

BOARD = {
    "squares_x": 6,            # columns of squares
    "squares_y": 8,            # rows of squares
    "square_mm": 30.0,
    "marker_mm": 22.0,
    "dictionary": "DICT_4X4_50",
    "legacy_pattern": False,
    "paper": "US Letter 8.5x11 in, print at 100% / actual size",
}
DPI = 600
PAGE_IN = (8.5, 11.0)


def mm2px(mm):
    return int(round(mm / 25.4 * DPI))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "charuco_letter.pdf"))
    a = ap.parse_args()

    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, BOARD["dictionary"]))
    board = cv2.aruco.CharucoBoard((BOARD["squares_x"], BOARD["squares_y"]),
                                   BOARD["square_mm"] / 1000, BOARD["marker_mm"] / 1000, d)
    board.setLegacyPattern(BOARD["legacy_pattern"])

    bw, bh = mm2px(BOARD["squares_x"] * BOARD["square_mm"]), mm2px(BOARD["squares_y"] * BOARD["square_mm"])
    img = board.generateImage((bw, bh), marginSize=0, borderBits=1)

    pw, ph = int(PAGE_IN[0] * DPI), int(PAGE_IN[1] * DPI)
    page = Image.new("L", (pw, ph), 255)
    ox, oy = (pw - bw) // 2, mm2px(14)
    page.paste(Image.fromarray(img), (ox, oy))

    draw = ImageDraw.Draw(page)
    try:
        font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", mm2px(3.2))
    except OSError:
        font = ImageFont.load_default()
    y = oy + bh + mm2px(5)
    # 100 mm scale bar: measure it after printing
    x0 = ox
    draw.rectangle([x0, y, x0 + mm2px(100), y + mm2px(1.2)], fill=0)
    for t in range(0, 101, 10):
        draw.rectangle([x0 + mm2px(t) - 2, y - mm2px(2), x0 + mm2px(t) + 2, y], fill=0)
    draw.text((x0 + mm2px(104), y - mm2px(2.5)), "100 mm, check with a ruler", font=font, fill=0)
    txt = (f"DEXI ChArUco  {BOARD['squares_x']}x{BOARD['squares_y']} squares  "
           f"square {BOARD['square_mm']:.0f} mm  marker {BOARD['marker_mm']:.0f} mm  {BOARD['dictionary']}  "
           "print at 100%, tape flat to rigid backing")
    draw.text((x0, y + mm2px(4)), txt, font=font, fill=0)

    page.save(a.out, "PDF", resolution=DPI)
    with open(os.path.splitext(a.out)[0] + ".json", "w") as f:
        json.dump(BOARD, f, indent=2)
    print(f"wrote {a.out}  board {bw/DPI*25.4:.0f}x{bh/DPI*25.4:.0f} mm")


if __name__ == "__main__":
    main()
