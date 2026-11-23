#!/usr/bin/env python3
"""Create Sepang_obs_map (Sepang + 5 obstacles) from the original Sepang map.

Obstacles can be cylinders (drawn as filled circles, seen from above) or
rectangles aligned with the track. The original Sepang_map.png / .yaml are
not modified.

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

# Obstacles: (shape, center x, center y, track direction, length, width)
#   shape:     'cylinder' or 'rectangle'
#   center:    map frame, m
#   direction: track direction at that point (rad); rectangles are aligned with it
#   length:    size along the track (m); for a cylinder, its diameter
#   width:     size across the track (m); for a cylinder, its diameter
# Every obstacle is 0.4 m from one wall; the free passage is on the other side.
OBSTACLES = [
    ('cylinder', -9.695, -11.515, -2.9749, 0.5, 0.5),    # 1, left,  passage 1.60 m
    ('rectangle', -9.564, -32.354, -0.6690, 1.2, 0.4),   # 2, right, passage 1.64 m
    ('cylinder', 19.578, -10.799, 0.1005, 0.7, 0.7),     # 3, left,  passage 1.42 m
    ('rectangle', 35.331, 11.970, 2.3500, 0.6, 0.3),     # 4, right, passage 1.82 m
    ('rectangle', -31.502, 33.677, -2.7313, 0.8, 0.6),   # 5, left,  passage 1.50 m
]


def to_pixel(x, y, height):
    """Map coordinates (m) -> image pixel (column, row). Row 0 is the top."""
    return (x - ORIGIN_X) / RESOLUTION, height - 1 - (y - ORIGIN_Y) / RESOLUTION


def draw_rectangle(draw, height, cx, cy, heading, length, width):
    tx, ty = math.cos(heading), math.sin(heading)     # along the track
    nx, ny = -math.sin(heading), math.cos(heading)    # across the track
    corners = []
    for a, b in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
        x = cx + a * length / 2.0 * tx + b * width / 2.0 * nx
        y = cy + a * length / 2.0 * ty + b * width / 2.0 * ny
        corners.append(to_pixel(x, y, height))
    draw.polygon(corners, fill=0)                     # black = occupied


def draw_cylinder(draw, height, cx, cy, diameter):
    u, v = to_pixel(cx, cy, height)
    radius_px = diameter / 2.0 / RESOLUTION
    draw.ellipse((u - radius_px, v - radius_px, u + radius_px, v + radius_px), fill=0)


def main():
    image = Image.open(SRC_PNG).convert('L')
    draw = ImageDraw.Draw(image)
    height = image.size[1]
    for shape, cx, cy, heading, length, width in OBSTACLES:
        if shape == 'cylinder':
            draw_cylinder(draw, height, cx, cy, length)
        elif shape == 'rectangle':
            draw_rectangle(draw, height, cx, cy, heading, length, width)
        else:
            raise ValueError(f'Unknown shape: {shape}')
    image.save(DST_PNG)

    with open(SRC_YAML) as f:
        yaml_text = f.read()
    with open(DST_YAML, 'w') as f:
        f.write(yaml_text.replace('Sepang_map.png', 'Sepang_obs_map.png'))
    print(f'Wrote {DST_PNG} and {DST_YAML} with {len(OBSTACLES)} obstacles')


if __name__ == '__main__':
    main()
