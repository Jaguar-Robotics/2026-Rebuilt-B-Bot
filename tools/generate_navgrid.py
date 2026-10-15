#!/usr/bin/env python3
"""
Generates src/main/deploy/pathplanner/navgrid.json for the 2026 REBUILT field for THIS robot (30 in tall, cannot fit under the trench).

What it blocks (blue side; the red side is the mirror image):
  * the field walls, padded by half the robot's width
  * the whole trench lanes (y 0..1.57 and 8.07-1.57..8.07) in the x band 3.96..5.18 -> the robot may only cross over the BUMPS
  * the hub (y 3.435..4.635) and the posts, padded by the robot's corner radius
  * the towers on both alliance walls (from the AprilTag layout), padded by the robot's corner radius. The original PathPlanner grid
    is not used any more: it was not symmetric between the two alliances.

The cells are 0.1 m (the original was 0.3 m) so paths hug obstacles less and bend more smoothly.
Edit the numbers below and re-run:  python3 tools/generate_navgrid.py
"""
import json, math, os

FIELD_X, FIELD_Y = 16.54, 8.07
NODE = 0.1

# robot (from deploy/pathplanner/settings.json): 0.978 wide x 0.73 long
HALF_WIDTH = 0.489
CORNER_RADIUS = math.hypot(0.978, 0.73) / 2          # 0.61: the robot may rotate while it drives
STRUCT_PAD = CORNER_RADIUS + 0.16                     # corner radius + 11 cm: clearance used around the hub / trench / posts (the robot may rotate)
WALL_PAD = HALF_WIDTH + 0.01
TOWER_PAD = STRUCT_PAD + 0.10                         # extra room around the towers: the robot used to hug them

# the structure between the alliance zone and the neutral zone (from the fuel-sim field model)
STRUCT_X0, STRUCT_X1 = 3.96, 5.18
TRENCH_LANE = 1.57                                    # trench opening + post, measured from the side wall
HUB_Y0, HUB_Y1 = FIELD_Y / 2 - 0.60, FIELD_Y / 2 + 0.60

# towers on the alliance walls: AprilTags 31/32 (blue, x=0.01, y 3.75..4.18) and 15/16 (red, x=16.53, y 3.89..4.32); the field is rotationally
# symmetric for these. About 1.25 m wide and 1.2 m deep.
TOWER_BLUE = (0.0, 3.30, 1.25, 4.65)

cols = math.ceil(FIELD_X / NODE)
rows = math.ceil(FIELD_Y / NODE)

def rects():
    """(x0, y0, x1, y1, pad) rectangles that are blocked, blue side then mirrored for red."""
    blue = [
        (STRUCT_X0, 0.0, STRUCT_X1, TRENCH_LANE, STRUCT_PAD),                 # trench lane, near wall
        (STRUCT_X0, FIELD_Y - TRENCH_LANE, STRUCT_X1, FIELD_Y, STRUCT_PAD),   # trench lane, far wall
        (STRUCT_X0, HUB_Y0, STRUCT_X1, HUB_Y1, STRUCT_PAD),                   # hub
    ]
    out = list(blue)
    for x0, y0, x1, y1, pad in blue:
        out.append((FIELD_X - x1, y0, FIELD_X - x0, y1, pad))
    tx0, ty0, tx1, ty1 = TOWER_BLUE
    out.append((tx0, ty0, tx1, ty1, TOWER_PAD))                                         # blue tower
    out.append((FIELD_X - tx1, FIELD_Y - ty1, FIELD_X - tx0, FIELD_Y - ty0, TOWER_PAD)) # red tower (rotated 180 degrees)
    return out

def dist_to_rect(px, py, x0, y0, x1, y1):
    dx = max(x0 - px, 0, px - x1)
    dy = max(y0 - py, 0, py - y1)
    return math.hypot(dx, dy)

R = rects()
grid = []
for r in range(rows):
    row = []
    cy = (r + 0.5) * NODE
    for c in range(cols):
        cx = (c + 0.5) * NODE
        blocked = (cx < WALL_PAD or cx > FIELD_X - WALL_PAD or cy < WALL_PAD or cy > FIELD_Y - WALL_PAD)
        if not blocked:
            for x0, y0, x1, y1, pad in R:
                if dist_to_rect(cx, cy, x0, y0, x1, y1) <= pad:
                    blocked = True
                    break
        row.append(blocked)
    grid.append(row)

out = os.path.join(os.path.dirname(__file__), '..', 'src', 'main', 'deploy', 'pathplanner', 'navgrid.json')
with open(out, 'w') as f:
    json.dump({"field_size": {"x": FIELD_X, "y": FIELD_Y}, "nodeSizeMeters": NODE, "grid": grid}, f, separators=(',', ':'))
print(f"wrote {out}: {rows} rows x {cols} cols of {NODE} m, {sum(sum(r) for r in grid)} blocked cells")
step = 4
for r in range(rows - 1, -1, -step):
    print(f"y={r*NODE:4.1f} " + ''.join('#' if grid[r][c] else '.' for c in range(0, cols, step)))
print("x -> 0 .. 16.5 (each char 0.4 m)")
