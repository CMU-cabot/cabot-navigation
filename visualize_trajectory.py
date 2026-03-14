#!/usr/bin/env python3
# Copyright (c) 2026  IBM Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, listribute, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Trajectory visualization tool for cabot test results.
Reads a trajectory_*.csv and generates an MP4 video with animated poses
and direction arrows.

Usage:
    python3 visualize_trajectory.py <trajectory_csv> [output.mp4]
"""

import csv
import sys
import math
import os
from collections import defaultdict

import cv2
import numpy as np

# ── canvas / world parameters ──────────────────────────────────────────────────
XY_MIN: float = -10.0
XY_MAX: float =  10.0
CANVAS_SIZE: int = 800          # square canvas in pixels
MARGIN: int = 40                # pixel margin for axis labels

# ── rendering parameters ───────────────────────────────────────────────────────
FPS: float = 20.0               # output video fps (matches 0.05 s recording step)
TRAIL_FRAMES: int = 60          # number of past frames shown as trail
ROBOT_RADIUS: int = 9           # circle radius for robot (px)
ACTOR_RADIUS: int = 6           # circle radius for actors (px)
ARROW_LEN: int = 20             # direction-arrow length (px)
ARROW_THICKNESS: int = 2
TRAIL_THICKNESS: int = 1

# ── colors (BGR) ───────────────────────────────────────────────────────────────
BG_COLOR     = (245, 245, 245)
GRID_COLOR   = (210, 210, 210)
AXIS_COLOR   = (160, 160, 160)
TEXT_COLOR   = (30, 30, 30)
ROBOT_COLOR  = (200, 70, 20)    # deep blue
CHILD_COLOR  = (30, 150, 255)   # orange

# Palette for adult actors / unnamed actors – enough for 10+ actors
ACTOR_PALETTE = [
    (50,  50,  210),   # red
    (50,  180,  80),   # green
    (190,  50, 190),   # magenta
    (40,  190, 190),   # cyan
    (40,  100, 230),   # orange-red
    (130,  50, 200),   # purple
    (200, 130,  50),   # teal
    (50,  130, 190),   # brown-ish
    (80,  200, 130),   # olive
    (190, 190,  50),   # sky blue
    (150,  80, 150),   # dark green
    (200,  80, 140),   # dark cyan
]


def _draw_size() -> int:
    """Drawable canvas size (excluding margin)."""
    return CANVAS_SIZE - 2 * MARGIN


def world_to_pixel(x: float, y: float):
    """Map world (x, y) → pixel (px, py). Y is flipped (world up = pixel up)."""
    draw = _draw_size()
    scale = draw / (XY_MAX - XY_MIN)
    px = MARGIN + int((x - XY_MIN) * scale)
    py = MARGIN + int((XY_MAX - y) * scale)
    return px, py


def draw_background(img: np.ndarray, title: str = "") -> None:
    """Draw a clean grid background with axis labels."""
    img[:] = BG_COLOR

    # grid lines every 5 units
    for val in range(int(XY_MIN), int(XY_MAX) + 1, 5):
        px, _ = world_to_pixel(val, 0)
        _, py = world_to_pixel(0, val)
        cv2.line(img, (px, MARGIN), (px, CANVAS_SIZE - MARGIN), GRID_COLOR, 1)
        cv2.line(img, (MARGIN, py), (CANVAS_SIZE - MARGIN, py), GRID_COLOR, 1)

    # major axes
    px0, py0 = world_to_pixel(0, 0)
    cv2.line(img, (px0, MARGIN), (px0, CANVAS_SIZE - MARGIN), AXIS_COLOR, 1)
    cv2.line(img, (MARGIN, py0), (CANVAS_SIZE - MARGIN, py0), AXIS_COLOR, 1)

    # border rect
    cv2.rectangle(img,
                  (MARGIN, MARGIN),
                  (CANVAS_SIZE - MARGIN, CANVAS_SIZE - MARGIN),
                  AXIS_COLOR, 1)

    # tick labels every 5 units
    font = cv2.FONT_HERSHEY_SIMPLEX
    for val in range(int(XY_MIN), int(XY_MAX) + 1, 5):
        px, _ = world_to_pixel(val, 0)
        _, py = world_to_pixel(0, val)
        cv2.putText(img, str(val), (px - 10, CANVAS_SIZE - MARGIN + 12),
                    font, 0.28, TEXT_COLOR, 1)
        cv2.putText(img, str(val), (2, py + 4),
                    font, 0.28, TEXT_COLOR, 1)

    # axis labels
    cv2.putText(img, "x", (CANVAS_SIZE - MARGIN + 4, py0 + 4),
                font, 0.4, TEXT_COLOR, 1)
    cv2.putText(img, "y", (px0 + 4, MARGIN - 6),
                font, 0.4, TEXT_COLOR, 1)

    if title:
        cv2.putText(img, title, (MARGIN, MARGIN - 10),
                    font, 0.38, TEXT_COLOR, 1)


def draw_agent(img: np.ndarray, px: int, py: int, yaw: float,
               color, radius: int) -> None:
    """Draw a filled circle with outline and a direction arrow."""
    cv2.circle(img, (px, py), radius, color, -1)
    cv2.circle(img, (px, py), radius, (0, 0, 0), 1)
    ex = int(px + ARROW_LEN * math.cos(yaw))
    ey = int(py - ARROW_LEN * math.sin(yaw))   # flip y
    cv2.arrowedLine(img, (px, py), (ex, ey), color,
                    ARROW_THICKNESS, tipLength=0.35)


def draw_legend(img: np.ndarray, color_map: dict) -> None:
    """Draw a color legend in the top-right corner."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    lx = CANVAS_SIZE - 170
    ly = MARGIN + 5
    line_h = 17
    for i, (name, color) in enumerate(sorted(color_map.items())):
        cy = ly + i * line_h + 6
        cv2.circle(img, (lx + 6, cy), 5, color, -1)
        cv2.circle(img, (lx + 6, cy), 5, (0, 0, 0), 1)
        text = name if len(name) <= 20 else name[:18] + ".."
        cv2.putText(img, text, (lx + 14, cy + 4),
                    font, 0.28, TEXT_COLOR, 1)


def assign_colors(entity_names: list, entity_types: dict) -> dict:
    """Assign a consistent BGR color to every entity."""
    color_map = {}
    actor_palette_idx = 0
    for name in sorted(entity_names):
        if entity_types.get(name) == 'robot':
            color_map[name] = ROBOT_COLOR
        elif 'child' in name.lower():
            color_map[name] = CHILD_COLOR
        else:
            color_map[name] = ACTOR_PALETTE[actor_palette_idx % len(ACTOR_PALETTE)]
            actor_palette_idx += 1
    return color_map


def main() -> None:
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <trajectory_csv> [output.mp4]")
        sys.exit(1)

    csv_path = sys.argv[1]
    out_path = (sys.argv[2] if len(sys.argv) > 2
                else os.path.splitext(csv_path)[0] + '.mp4')

    # ── load CSV ────────────────────────────────────────────────────────────────
    # timestep_data: elapsed_sec -> [entity_dict, ...]
    timestep_data: dict = {}
    entity_types: dict = {}    # name -> 'robot' | 'actor'

    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row['elapsed_sec'])
            if t not in timestep_data:
                timestep_data[t] = []
            timestep_data[t].append({
                'name': row['entity_name'],
                'type': row['entity_type'],
                'x':    float(row['pos_x']),
                'y':    float(row['pos_y']),
                'yaw':  float(row['yaw']),
            })
            entity_types[row['entity_name']] = row['entity_type']

    ordered_times = sorted(timestep_data.keys())

    if not ordered_times:
        print("ERROR: No data found in CSV.")
        sys.exit(1)

    color_map = assign_colors(list(entity_types.keys()), entity_types)
    title = os.path.basename(os.path.splitext(csv_path)[0])

    # ── setup video writer ──────────────────────────────────────────────────────
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(out_path, fourcc, FPS,
                             (CANVAS_SIZE, CANVAS_SIZE))
    if not writer.isOpened():
        print(f"ERROR: Cannot open video writer for {out_path}")
        sys.exit(1)

    # ── render frame by frame ───────────────────────────────────────────────────
    # history[entity_name] = [(px, py), ...] (most recent last)
    history: dict = defaultdict(list)

    total = len(ordered_times)
    print(f"Rendering {total} frames → {out_path}")

    for frame_idx, t in enumerate(ordered_times):
        # base background (grid is redrawn each frame)
        img = np.empty((CANVAS_SIZE, CANVAS_SIZE, 3), dtype=np.uint8)
        draw_background(img, title)

        entities = timestep_data[t]

        # ── update position history ─────────────────────────────────────────────
        current_px: dict = {}
        for e in entities:
            px, py = world_to_pixel(e['x'], e['y'])
            current_px[e['name']] = (px, py, e['yaw'], e['type'])
            history[e['name']].append((px, py))
            if len(history[e['name']]) > TRAIL_FRAMES:
                history[e['name']].pop(0)

        # ── draw trails ─────────────────────────────────────────────────────────
        for name, pts in history.items():
            color = color_map.get(name, (128, 128, 128))
            n = len(pts)
            for i in range(1, n):
                alpha = i / n                       # newest = full color
                c = tuple(int(cv + (200 - cv) * (1.0 - alpha))
                           for cv in color)
                cv2.line(img, pts[i - 1], pts[i], c, TRAIL_THICKNESS)

        # ── draw current pose: actors first, robot on top ───────────────────────
        actors = [(n, v) for n, v in current_px.items() if v[3] != 'robot']
        robots = [(n, v) for n, v in current_px.items() if v[3] == 'robot']

        for name, (px, py, yaw, etype) in actors + robots:
            color = color_map.get(name, (128, 128, 128))
            radius = ROBOT_RADIUS if etype == 'robot' else ACTOR_RADIUS
            draw_agent(img, px, py, yaw, color, radius)

        # ── overlays ────────────────────────────────────────────────────────────
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, f"t = {t:6.2f} s", (MARGIN, MARGIN - 10),
                    font, 0.45, TEXT_COLOR, 1)
        draw_legend(img, color_map)

        # ── progress bar (bottom) ───────────────────────────────────────────────
        bar_w = int((frame_idx + 1) / total * (CANVAS_SIZE - 2 * MARGIN))
        cv2.rectangle(img,
                      (MARGIN, CANVAS_SIZE - MARGIN + 20),
                      (MARGIN + bar_w, CANVAS_SIZE - MARGIN + 26),
                      AXIS_COLOR, -1)

        writer.write(img)

        if (frame_idx + 1) % 200 == 0 or frame_idx + 1 == total:
            print(f"  {frame_idx + 1}/{total} frames")

    writer.release()
    duration = total / FPS
    print(f"Done. Duration: {duration:.1f} s  ({total} frames @ {FPS:.0f} fps)")
    print(f"Saved: {out_path}")


if __name__ == '__main__':
    main()
