#!/usr/bin/env python3
"""
Collision map between the hopper floor and the intake (and the sliding hopper), from the CAD, using the same poses the 3D view uses.
Needs: python3 with numpy, scipy, trimesh.   Usage: python3 tools/collision_map.py "<folder with the 4 .glb files>"
"""
import math, sys, glob, os
import numpy as np, trimesh
from scipy.spatial import cKDTree

folder = sys.argv[1] if len(sys.argv) > 1 else "/Users/noah/Downloads/GLB CAD"
M = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])      # CAD (x lateral, y forward, z down) -> robot (x forward, y left, z up)

def load(name, n=15000):
    scene = trimesh.load(os.path.join(folder, name))
    mesh = scene.dump(concatenate=True)
    pts, _ = trimesh.sample.sample_surface(mesh, n, seed=1)
    return pts @ M.T

floor0, intake0, slide0 = load("pivoting hopper bottom plate.glb"), load("intake slapdown.glb"), load("slidy hopper.glb")

# ---- constants (match FourBarConstants / IntakeConstants)
L1 = 0.2939; L3 = 0.066
B0 = (-0.170, 0.151); F0 = (0.143, 0.206)
GDX, GDZ = 0.0854, 0.0248
DOWN, UP = -0.010537, 0.173785
PIV = (0.266, 0.187); SLIDE = 0.27

def floor_pose(link_rot):
    coupler = math.hypot(F0[0] - B0[0], F0[1] - B0[1])
    aF = DOWN * 2 * math.pi
    hx = B0[0] + L1 * math.cos(aF); hz = B0[1] - L1 * math.sin(aF)
    sx, sz = hx + GDX, hz + GDZ
    a = link_rot * 2 * math.pi
    bx, bz = hx - L1 * math.cos(a), hz + L1 * math.sin(a)
    dx, dz = bx - sx, bz - sz; d = max(1e-6, math.hypot(dx, dz))
    along = (L3 * L3 - coupler * coupler + d * d) / (2 * d)
    h = math.sqrt(max(0, L3 * L3 - along * along))
    px, pz = sx + along * dx / d, sz + along * dz / d
    c1 = (px + h * (-dz / d), pz + h * (dx / d)); c2 = (px - h * (-dz / d), pz - h * (dx / d))
    flat = math.copysign(1, (B0[0] - sx) * (F0[1] - sz) - (B0[1] - sz) * (F0[0] - sx))
    side1 = math.copysign(1, dx * (c1[1] - sz) - dz * (c1[0] - sx))
    fx, fz = c1 if side1 == flat else c2
    delta = math.atan2(fz - bz, fx - bx) - math.atan2(F0[1] - B0[1], F0[0] - B0[0])
    rx = B0[0] * math.cos(delta) - B0[1] * math.sin(delta); rz = B0[0] * math.sin(delta) + B0[1] * math.cos(delta)
    return delta, (bx - rx, bz - rz)

def apply_floor(link_rot):
    delta, (tx, tz) = floor_pose(link_rot)
    c, s = math.cos(delta), math.sin(delta)
    p = floor0
    return np.c_[p[:, 0] * c - p[:, 2] * s + tx, p[:, 1], p[:, 0] * s + p[:, 2] * c + tz]

def apply_intake(theta_deg):
    p_ = -math.radians(theta_deg)                         # pitch = -(angle - 0)
    c, s = math.cos(p_), math.sin(p_)
    px, pz = PIV
    rx = px * c + pz * s; rz = -px * s + pz * c
    tx, tz = px - rx, pz - rz
    p = intake0
    return np.c_[p[:, 0] * c + p[:, 2] * s + tx, p[:, 1], -p[:, 0] * s + p[:, 2] * c + tz]

def apply_slide(theta_deg):
    frac = min(1, max(0, theta_deg / 120.0))
    return slide0 + np.array([-SLIDE * frac, 0, 0])

def gap(a, b):
    d, _ = cKDTree(a).query(b, k=1)
    return float(d.min())

links = np.linspace(DOWN, UP, 16)
thetas = list(range(0, 125, 5))
print("link angle (rot) down->up across; intake angle (deg) down the side")
print("legend: X = floor and intake within 1.5 cm (collision), + = within 4 cm, . = clear")
trees = {l: cKDTree(apply_floor(l)) for l in links}                 # one index per floor pose
intakes = {th: apply_intake(th) for th in thetas}
slides = {th: apply_slide(th) for th in thetas}
def gap_to(tree, pts):
    return float(tree.query(pts, k=1)[0].min())
res = {}
print("theta\\link " + " ".join(f"{l:6.3f}" for l in links))
for th in thetas:
    row = []
    for l in links:
        g = min(gap_to(trees[l], intakes[th]), gap_to(trees[l], slides[th]))
        res[(th, l)] = g
        row.append("  X   " if g < 0.015 else ("  +   " if g < 0.04 else "  .   "))
    print(f"{th:5d}      " + " ".join(row))

print("\nclosest gap (m) between floor and intake / slide at a few poses:")
for th, l in [(0, DOWN), (0, UP), (60, DOWN), (60, UP), (120, DOWN), (120, UP)]:
    t = cKDTree(apply_floor(l))
    print(f"  intake {th:3d} deg, floor link {l:+.3f}: floor-intake {gap_to(t, apply_intake(th)):.3f} m, floor-slide {gap_to(t, apply_slide(th)):.3f} m")
