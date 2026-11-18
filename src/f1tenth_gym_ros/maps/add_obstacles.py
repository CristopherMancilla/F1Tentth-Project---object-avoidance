#!/usr/bin/env python3
"""Create Sepang_obs_map (Sepang + 5 obstacles) from the original Sepang map.

The original Sepang_map.png / .yaml are not modified.

Usage (from this folder):
    python3 add_obstacles.py
"""
import math
import os

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_PNG = os.path.join(HERE, 'Sepang_map.png')
SRC_YAML = os.path.join(HERE, 'Sepang_map.yaml')
DST_PNG = os.path.join(HERE, 'Sepang_obs_map.png')
DST_YAML = os.path.join(HERE, 'Sepang_obs_map.yaml')

# Map geometry (from Sepang_map.yaml)
RESOLUTION = 0.05944                                   # m per pixel
ORIGIN_X, ORIGIN_Y = -66.15462837870969, -58.7392475352799

# Obstacle size: square of SIZE x SIZE meters
SIZE = 0.5

# Obstacles: center x, center y (map frame, m) and the track direction at
# that point (rad), so the square is aligned with the track.
OBSTACLES = [
    (-9.695, -11.515, -2.9749),   # 1, left side of the track
    (-9.533, -32.315, -0.6690),   # 2, right side
    (19.568, -10.700, 0.1005),    # 3, left side
    (35.260, 11.900, 2.3500),     # 4, right side
    (-31.482, 33.632, -2.7313),   # 5, left side
]


def to_pixel(x, y, height):
    """Map coordinates (m) -> image pixel (column, row). Row 0 is the top."""
    return (x - ORIGIN_X) / RESOLUTION, height - 1 - (y - ORIGIN_Y) / RESOLUTION


def main():
    image = Image.open(SRC_PNG).convert('L')
    draw = ImageDraw.Draw(image)
    half = SIZE / 2.0
    for cx, cy, heading in OBSTACLES:
        tx, ty = math.cos(heading), math.sin(heading)     # along the track
        nx, ny = -math.sin(heading), math.cos(heading)    # across the track
        corners = []
        for a, b in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
            x = cx + a * half * tx + b * half * nx
            y = cy + a * half * ty + b * half * ny
            corners.append(to_pixel(x, y, image.size[1]))
        draw.polygon(corners, fill=0)                     # black = occupied
    image.save(DST_PNG)

    with open(SRC_YAML) as f:
        yaml_text = f.read()
    with open(DST_YAML, 'w') as f:
        f.write(yaml_text.replace('Sepang_map.png', 'Sepang_obs_map.png'))
    print(f'Wrote {DST_PNG} and {DST_YAML} with {len(OBSTACLES)} obstacles')


if __name__ == '__main__':
    main()
