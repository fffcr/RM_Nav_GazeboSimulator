#!/usr/bin/env python3
"""Generate corridor.pgm from corridor.sdf geometry for A* global planner."""

import math

# ---- Map config ----------------------------------------------------------
RES = 0.05                 # m/pixel
# Span: x [-5, 10], y [-7, 7]
W, H = 300, 280
ORIGIN_X, ORIGIN_Y = -5.0, -7.0

def world_to_pixel(wx, wy):
    px = int((wx - ORIGIN_X) / RES)
    py = int((wy - ORIGIN_Y) / RES)
    return px, py

# ---- All white (free space) ----------------------------------------------
img = bytearray(b'\xff' * (W * H))

def fill_rect(cx, cy, sx, sy):
    """Fill a rectangle centered at (cx,cy) with size (sx,sy) as occupied."""
    x0, y0 = world_to_pixel(cx - sx / 2, cy - sy / 2)
    x1, y1 = world_to_pixel(cx + sx / 2, cy + sy / 2)
    x0 = max(0, min(W - 1, x0))
    x1 = max(0, min(W - 1, x1))
    y0 = max(0, min(H - 1, y0))
    y1 = max(0, min(H - 1, y1))
    for py in range(y0, y1 + 1):
        row_start = (H - 1 - py) * W  # PGM: first row = max y
        for px in range(x0, x1 + 1):
            img[row_start + px] = 0

# ---- Obstacles from corridor.sdf -----------------------------------------
# box:   pose (3, 2.5, 0.5),  collision 3x5x1 → 2D rect 3m(x) x 5m(y)
fill_rect(3.0, 2.5, 3, 5)
# box_0: pose (3, -3.1, 0.5), collision 3x5x1 → 2D rect 3m(x) x 5m(y)
fill_rect(3.0, -3.1, 3, 5)

# ---- Write PGM (P5 binary) ------------------------------------------------
with open('corridor.pgm', 'wb') as f:
    f.write(f'P5\n{W} {H}\n255\n'.encode())
    f.write(img)

print(f'Generated corridor.pgm ({W}x{H}, {RES} m/pixel)')
print(f'  box:     x[1.5, 4.5] y[0.0, 5.0]')
print(f'  box_0:   x[1.5, 4.5] y[-5.6, -0.6]')
print(f'  gap:     0.6 m at y=[-0.6, 0.0]')
