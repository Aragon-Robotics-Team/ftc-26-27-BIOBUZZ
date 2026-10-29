"""Builds the AdvantageScope assets for the sim's tilting hives, from simple shapes instead of the field CAD.

Writes three custom assets:
  Field3d_BIOBUZZSim         the field without the hive trays: tiles, walls, tape, flowers, the hive A-frame,
                             and Pollen / Nectar game-piece models for GamePieceTracker's positions3d
  Robot_BIOBUZZ Hive Red     the red tray (both cells, the arm, the AprilTags), posed by Hive/red/trayPose3d
  Robot_BIOBUZZ Hive Blue    the same in blue, posed by Hive/blue/trayPose3d

Dimensions are the Competition Manual's, as BiobuzzSim (the Roblox sim) has them in Config.lua and FieldBuilder.lua;
the shapes are built the same way here. Inches throughout, written out in meters.

Frames:
  "Roblox" (how the numbers below are written, as in BiobuzzSim): origin at the field center, +X toward the blue
    alliance, +Y up, +Z toward the audience.
  Field model: AdvantageScope's native FTC field frame, +Z up (config rotations []). Logged "center-rotated" poses
    land in it turned -90 deg about Z, so a pose logged at (x, y) is drawn at (y, -x): BiobuzzRobotSim's tray pose
    (0, -12.75 in) is the red hive here at x = -12.75 in, where the stock 2026-2027 field has it.
  Tray model: the tray pose's own frame, origin on the pivot. The logged frame is FieldFrame's (x toward the
    audience, y away from the red wall), so the cells lie along +-x and the arm turns about y (pitch).

Usage: python3 build_sim_field.py [userAssets dir]   (needs numpy and trimesh)
"""
import json
import math
import os
import sys

import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial

IN = 0.0254

# ---------------------------------------------------------------- dimensions (BiobuzzSim Config.lua)
FIELD = 144
TILE = 24
WALL_HEIGHT = 12
WALL_T = 1
ALLIANCE_AREA_WIDTH = 97
ALLIANCE_AREA_DEPTH = 54
TAPE_W = 2
LZ_WIDTH, LZ_DEPTH = 24, 11
RED_LZ_Z, BLUE_LZ_Z = -36, 36
GARDEN_LENGTH, GARDEN_WIDTH = 23, 2

HIVE_X = 12.75           # red at -X, blue at +X
PIVOT = 43.95
FRAME_WIDTH = 49.46      # along X
FRAME_DEPTH = 38.95      # along Z, at the base
ARM_LENGTH = 18.84       # gap between the cells
CELL_W, CELL_H, CELL_SIDE_H, CELL_D = 20, 14, 7.61, 12
CELL_RISE = 5.75         # cell center above the arm axis
TAG_SIZE, TAG_GAP, TAG_FROM_PIVOT = 3.25, 0.5, 14.27
# Tag ids per cell. North (-Z, away from the audience) is red's LEFT and blue's RIGHT (SimHives).
TAG_IDS = {"Red": {"North": 30, "South": 34}, "Blue": {"South": 38, "North": 42}}

FLOWER_WALL_OFFSET = 3.5
FLOWER_TOP = 21.5
FLOWER_BACKSTOP = 1.25
FLOWER_MIDDLE = 3.95
FLOWER_BOTTOM = 0.4
FLOWER_HOLE_R = 2.79 / 2
FLOWER_OPENING_R = 2.0
FLOWER_INNER_R = 2.25
FLOWERS = [("North", -24), ("South", 24), ("West", 24), ("East", -24)]

POLLEN_D, NECTAR_D = 2.8, 3.6

# ---------------------------------------------------------------- colors, 0-255 RGBA
TILE_A, TILE_B, SEAM = (66, 66, 70, 255), (74, 74, 78, 255), (40, 40, 42, 255)
GLASS = (220, 230, 240, 90)
METAL = (150, 150, 155, 255)
DARK = (45, 45, 50, 255)
RED, BLUE = (215, 40, 40, 255), (40, 90, 230, 255)
RED_FILL, BLUE_FILL = (215, 40, 40, 46), (40, 90, 230, 46)
GREEN = (70, 200, 80, 255)
TUBE = (210, 170, 230, 77)
ORANGE = (255, 150, 20, 255)
PURPLE = (120, 60, 200, 255)
YELLOW = (250, 205, 30, 255)
WHITE, BLACK = (245, 245, 245, 255), (10, 10, 10, 255)
POLLEN, NECTAR_RED, NECTAR_BLUE = (255, 214, 0, 255), (230, 30, 30, 255), (30, 80, 235, 255)

# tag36h11 codes and bit layout (AprilRobotics), as in FieldBuilder.lua
TAG36H11 = {30: 60884361568, 31: 12872940643, 32: 19096692507, 33: 21549952597, 34: 26672269255,
            35: 28204941670, 36: 28173392202, 37: 29781966262, 38: 37813287046, 39: 42389124888,
            40: 47235095178, 41: 50168524841, 42: 56410991961, 43: 60007549953, 44: 61056786991,
            45: 67567695709}
TAG_BIT_X = [1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4, 1, 1, 1, 1, 1, 2, 2, 2, 3]
TAG_BIT_Y = [1, 1, 1, 1, 1, 2, 2, 2, 3, 1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4]

# Roblox axes (x, y up, z) to each output frame, as 3x3 matrices applied to column vectors
TO_FIELD = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)  # (x, -z, y)
TO_TRAY = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)    # (z, x, y)


class Model:
    """Meshes in Roblox inches, grouped by color; written out as one mesh per color under one root node."""

    def __init__(self, axes):
        self.axes = axes
        self.parts = {}

    def add(self, color, mesh):
        self.parts.setdefault(color, []).append(mesh)

    def box(self, color, x0, x1, y0, y1, z0, z1):
        """An axis-aligned box from bounds."""
        m = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0])
        m.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
        self.add(color, m)

    def posed_box(self, color, size, transform):
        m = trimesh.creation.box(extents=size)
        m.apply_transform(transform)
        self.add(color, m)

    def beam(self, color, a, b, thickness):
        """A square beam from point a to point b."""
        a, b = np.asarray(a, float), np.asarray(b, float)
        m = trimesh.creation.box(extents=[thickness, thickness, np.linalg.norm(b - a)])
        m.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], b - a))
        m.apply_translation((a + b) / 2)
        self.add(color, m)

    def cylinder_y(self, color, cx, cz, r, y0, y1, sections=24):
        m = trimesh.creation.cylinder(radius=r, height=y1 - y0, sections=sections)
        m.apply_transform(trimesh.transformations.rotation_matrix(-math.pi / 2, [1, 0, 0]))  # its z axis to +y
        m.apply_translation([cx, (y0 + y1) / 2, cz])
        self.add(color, m)

    def ring_y(self, color, cx, cz, r_in, r_out, y0, y1, sections=32, arc=None):
        """An annulus about a vertical axis, or only the part within arc = (center angle, half width) radians,
        angles measured from +X toward +Z."""
        if arc is None:
            m = trimesh.creation.annulus(r_min=r_in, r_max=r_out, height=y1 - y0, sections=sections)
            m.apply_transform(trimesh.transformations.rotation_matrix(-math.pi / 2, [1, 0, 0]))
            m.apply_translation([cx, (y0 + y1) / 2, cz])
            self.add(color, m)
            return
        step = 2 * math.pi / sections
        for i in range(sections):
            a = (i + 0.5) * step
            if abs((a - arc[0] + math.pi) % (2 * math.pi) - math.pi) > arc[1]:
                continue
            seg = 2 * r_out * math.tan(step / 2)
            r_mid = (r_in + r_out) / 2
            t = np.eye(4)
            t[:3, :3] = trimesh.transformations.rotation_matrix(-a, [0, 1, 0])[:3, :3]
            t[:3, 3] = [cx + math.cos(a) * r_mid, (y0 + y1) / 2, cz + math.sin(a) * r_mid]
            self.posed_box(color, [r_out - r_in, y1 - y0, seg], t)

    def export(self, path, root):
        scene = trimesh.Scene(base_frame=root)
        to = np.eye(4)
        to[:3, :3] = self.axes * IN
        for i, (color, meshes) in enumerate(self.parts.items()):
            merged = trimesh.util.concatenate(meshes)
            merged.apply_transform(to)
            merged = trimesh.Trimesh(merged.vertices, merged.faces, process=True)
            rgba = np.array(color, dtype=np.float64) / 255
            merged.visual = trimesh.visual.TextureVisuals(material=PBRMaterial(
                baseColorFactor=rgba, metallicFactor=0.1, roughnessFactor=0.7, doubleSided=True,
                alphaMode="BLEND" if color[3] < 255 else "OPAQUE"))
            _ = merged.vertex_normals
            scene.add_geometry(merged, node_name=f"part_{i}", geom_name=f"mesh_{i}", parent_node_name=root)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        scene.export(path, include_normals=True)
        faces = sum(len(g.faces) for g in scene.geometry.values())
        print(f"{path}: {len(scene.geometry)} meshes, {faces} faces, {os.path.getsize(path) // 1024} KB")


# ---------------------------------------------------------------- field
def build_field():
    f = Model(TO_FIELD)
    half = FIELD / 2
    # tiles, top at y = 0, and the seams between them
    for c in range(6):
        for r in range(6):
            x0, z0 = -half + c * TILE, -half + r * TILE
            f.box(TILE_A if (c + r) % 2 == 0 else TILE_B, x0, x0 + TILE, -0.6, 0, z0, z0 + TILE)
    for i in range(1, 6):
        p = -half + i * TILE
        f.box(SEAM, p - 0.1, p + 0.1, 0, 0.03, -half, half)
        f.box(SEAM, -half, half, 0, 0.03, p - 0.1, p + 0.1)
    f.box(DARK, -half - 2, half + 2, -1.2, -0.6, -half - 2, half + 2)

    # perimeter: clear panels with a metal rail on top, and corner posts
    t = WALL_T
    for x0, x1, z0, z1 in [(-half - t, half + t, -half - t, -half), (-half - t, half + t, half, half + t),
                           (-half - t, -half, -half, half), (half, half + t, -half, half)]:
        f.box(GLASS, x0, x1, 1.0, WALL_HEIGHT, z0, z1)
        f.box(METAL, x0 - 0.5, x1 + 0.5, WALL_HEIGHT, WALL_HEIGHT + 1.25, z0 - 0.5, z1 + 0.5)
        f.box(METAL, x0 - 0.5, x1 + 0.5, -0.6, 1.0, z0 - 0.5, z1 + 0.5)
    for sx in (-1, 1):
        for sz in (-1, 1):
            px, pz = sx * (half + t / 2), sz * (half + t / 2)
            f.box(METAL, px - 1, px + 1, -0.6, WALL_HEIGHT + 1.25, pz - 1, pz + 1)

    # tape: loading zones, gardens, alliance areas
    def strip(color, x0, x1, z0, z1):
        f.box(color, x0, x1, 0.02, 0.07, z0, z1)

    def fill(color, x0, x1, z0, z1):
        f.box(color, x0, x1, 0.01, 0.04, z0, z1)

    for color, tint, wall, zc in [(RED, RED_FILL, -1, RED_LZ_Z), (BLUE, BLUE_FILL, 1, BLUE_LZ_Z)]:
        z0, z1 = zc - LZ_WIDTH / 2, zc + LZ_WIDTH / 2
        inner = wall * (half - LZ_DEPTH)
        xa, xb = sorted((wall * half, inner))
        strip(color, *sorted((inner, inner - wall * TAPE_W)), z0, z1)
        strip(color, xa, xb, z0, z0 + TAPE_W)
        strip(color, xa, xb, z1 - TAPE_W, z1)
        fill(tint, xa, xb, z0, z1)
    strip(RED, -half + 0.5, -half + 0.5 + GARDEN_LENGTH, half - GARDEN_WIDTH, half)
    strip(BLUE, half - 0.5 - GARDEN_LENGTH, half - 0.5, -half, -half + GARDEN_WIDTH)
    aw, ad, out = ALLIANCE_AREA_WIDTH, ALLIANCE_AREA_DEPTH, half + WALL_T
    for color, tint, x0, x1, far in [(RED, RED_FILL, -out - ad, -out, -out - ad),
                                     (BLUE, BLUE_FILL, out, out + ad, out + ad - TAPE_W)]:
        strip(color, far, far + TAPE_W, -aw / 2, aw / 2)
        strip(color, x0, x1, -aw / 2, -aw / 2 + TAPE_W)
        strip(color, x0, x1, aw / 2 - TAPE_W, aw / 2)
        fill(tint, x0, x1, -aw / 2, aw / 2)

    for wall, along in FLOWERS:
        build_flower(f, wall, along)
    build_hive_frame(f)
    return f


def build_flower(f, wall, along):
    half = FIELD / 2
    cx, cz, inward = {
        "North": (along, -half + FLOWER_WALL_OFFSET, (0, 1)),
        "South": (along, half - FLOWER_WALL_OFFSET, (0, -1)),
        "West": (-half + FLOWER_WALL_OFFSET, along, (1, 0)),
        "East": (half - FLOWER_WALL_OFFSET, along, (-1, 0)),
    }[wall]
    wx, wz = -inward[0], -inward[1]  # toward the wall
    wall_angle = math.atan2(wz, wx)
    ring_bottom = FLOWER_TOP - 0.6
    f.ring_y(DARK, cx, cz, FLOWER_HOLE_R, FLOWER_HOLE_R + 1.45, 0, FLOWER_BOTTOM, 16)
    for side in (-1, 1):
        a = wall_angle + side * math.radians(35)
        f.cylinder_y(METAL, cx + math.cos(a) * 2.45, cz + math.sin(a) * 2.45, 0.3, FLOWER_BOTTOM, FLOWER_MIDDLE, 8)
    f.ring_y(DARK, cx, cz, FLOWER_INNER_R, FLOWER_INNER_R + 0.8, FLOWER_MIDDLE, FLOWER_MIDDLE + 0.5, 24)
    f.ring_y(TUBE, cx, cz, FLOWER_INNER_R, FLOWER_INNER_R + 0.4, FLOWER_MIDDLE + 0.25, ring_bottom, 16)
    for i in range(4):
        a = wall_angle + (i + 0.5) * math.pi / 2
        r = FLOWER_INNER_R + 0.35
        f.cylinder_y(GREEN, cx + math.cos(a) * r, cz + math.sin(a) * r, 0.35, FLOWER_MIDDLE, ring_bottom, 10)
    f.ring_y(ORANGE, cx, cz, FLOWER_OPENING_R, FLOWER_OPENING_R + 1.0, ring_bottom, FLOWER_TOP, 24)
    f.ring_y(PURPLE, cx, cz, FLOWER_OPENING_R + 0.4, FLOWER_OPENING_R + 0.9, FLOWER_TOP, FLOWER_TOP + FLOWER_BACKSTOP,
             24, arc=(wall_angle, math.pi * 0.45))
    # wall mount: a plate on the wall and an arm out to the column
    plate_y0, plate_y1 = max(WALL_HEIGHT - 5, FLOWER_MIDDLE + 1), WALL_HEIGHT - 0.5
    d = FLOWER_WALL_OFFSET - 0.15
    px, pz = cx + wx * d, cz + wz * d
    along_z = abs(wx) > 0.5
    hx, hz = (0.15, 2.0) if along_z else (2.0, 0.15)
    f.box(METAL, px - hx, px + hx, plate_y0, plate_y1, pz - hz, pz + hz)
    a0, a1 = FLOWER_INNER_R + 0.2, FLOWER_WALL_OFFSET - 0.3
    mid, length = (a0 + a1) / 2, a1 - a0
    ax, az = cx + wx * mid, cz + wz * mid
    hx, hz = (length / 2, 1.25) if along_z else (1.25, length / 2)
    y = (plate_y0 + plate_y1) / 2
    f.box(METAL, ax - hx, ax + hx, y - 0.5, y + 0.5, az - hz, az + hz)


def build_hive_frame(f):
    """The A-frame both trays hang from: two leg pairs, the crossbar carrying both pivots, the logo banners."""
    hx, hz = FRAME_WIDTH / 2, FRAME_DEPTH / 2
    for sx in (-1, 1):
        x = sx * hx
        f.box(METAL, x - 1, x + 1, 0, 1.5, -hz - 1, hz + 1)
        for sz in (-1, 1):
            f.beam(METAL, (x, 0.75, sz * hz), (x, PIVOT, 0), 1.5)
        f.box(METAL, x - 2, x + 2, PIVOT - 2, PIVOT + 2, -2, 2)
    f.box(METAL, -hx - 2, hx + 2, PIVOT - 1, PIVOT + 1, -1, 1)
    panel_h, tilt = 6.5, math.radians(30)
    for sz in (-1, 1):
        hinge = np.array([0, PIVOT - 1.0, sz * 1.1])
        center = hinge + [0, -math.cos(tilt) * panel_h / 2, sz * math.sin(tilt) * panel_h / 2]
        t = trimesh.transformations.rotation_matrix(-sz * tilt, [1, 0, 0])
        t[:3, 3] = center
        f.posed_box(YELLOW, [28, panel_h, 0.2], t)


# ---------------------------------------------------------------- trays
def tag_grid(tag_id):
    """10 x 10 cells, True = white, rows top to bottom (apriltag_to_image)."""
    code = TAG36H11[tag_id]
    g = [[r in (0, 9) or c in (0, 9) for c in range(10)] for r in range(10)]
    for i in range(36):
        g[TAG_BIT_Y[i] + 1][TAG_BIT_X[i] + 1] = (code >> (35 - i)) & 1 == 1
    return g


def build_tray(alliance):
    """One tray, level, in the pivot's frame (Roblox axes): the arm along Z, the South cell at +Z, each cell open
    away from the pivot. The tray pose's pitch tips it."""
    color = RED if alliance == "Red" else BLUE
    skin = color[:3] + (140,)
    m = Model(TO_TRAY)
    m.box(color, -1.5, 1.5, -1.5, 1.5, -1.5, 1.5)                                  # hub
    m.box(color, -1, 1, -1, 1, -(ARM_LENGTH / 2 + 1), ARM_LENGTH / 2 + 1)          # arm
    offset = ARM_LENGTH / 2 + CELL_D / 2
    W, H, HS, D = CELL_W, CELL_H, CELL_SIDE_H, CELL_D
    roof_rise = H - HS
    for cell, s in (("North", -1), ("South", 1)):
        # cell frame: origin at the cell center, +Z out toward the opening; the North cell is the South one
        # turned 180 deg about Y
        base = trimesh.transformations.rotation_matrix(0 if s > 0 else math.pi, [0, 1, 0])
        base[:3, 3] = [0, CELL_RISE, s * offset]

        def part(col, size, x, y, z, rot=None):
            t = np.eye(4) if rot is None else rot.copy()
            t[:3, 3] = [x, y, z]
            m.posed_box(col, size, base @ t)

        back_z = -(D / 2 - 0.25)
        part(skin, [W, HS, 0.5], 0, -H / 2 + HS / 2, back_z)
        peak = trimesh.convex.convex_hull(np.array(
            [[sx, -H / 2 + HS, back_z + dz] for sx in (-W / 2, W / 2) for dz in (-0.25, 0.25)]
            + [[0, H / 2, back_z + dz] for dz in (-0.25, 0.25)]))
        peak.apply_transform(base)
        m.add(skin, peak)
        part(skin, [W, 0.5, D], 0, -(H / 2 - 0.25), 0)
        part(skin, [0.5, HS, D], -(W / 2 - 0.25), -H / 2 + HS / 2, 0)
        part(skin, [0.5, HS, D], W / 2 - 0.25, -H / 2 + HS / 2, 0)
        roof_len = math.hypot(W / 2, roof_rise)
        roof_ang = math.atan2(roof_rise, W / 2)
        roof_y = -H / 2 + HS + roof_rise / 2
        for sx in (-1, 1):
            part(skin, [roof_len, 0.5, D], sx * W / 4, roof_y, 0,
                 trimesh.transformations.rotation_matrix(-sx * roof_ang, [0, 0, 1]))
        part(color, [W, 0.8, 0.8], 0, -H / 2, D / 2)
        for sx in (-1, 1):
            part(color, [0.8, HS, 0.8], sx * W / 2, -H / 2 + HS / 2, D / 2)
        part(color, [3, CELL_RISE, 3], 0, -(H / 2 + CELL_RISE / 2), -(D / 2 - 2))

        # AprilTags under the floor: four in a row across the cell (along X), centered TAG_FROM_PIVOT along the
        # arm. Ids rise along +X on red and -X on blue, as SimHives lays them out.
        strip = 4 * TAG_SIZE + 3 * TAG_GAP
        tag_z = s * TAG_FROM_PIVOT
        tag_y = CELL_RISE - (H / 2 + 0.3)
        m.box(WHITE, -(strip + 0.5) / 2, (strip + 0.5) / 2, tag_y - 0.05, tag_y + 0.05,
              tag_z - (TAG_SIZE + 0.25) / 2, tag_z + (TAG_SIZE + 0.25) / 2)
        first = TAG_IDS[alliance][cell]
        step = 1 if alliance == "Red" else -1
        cellsz = TAG_SIZE / 10
        for k in range(4):
            tx = step * (k - 1.5) * (TAG_SIZE + TAG_GAP)
            grid = tag_grid(first + k)
            for r in range(10):
                for c in range(10):
                    if not grid[r][c]:
                        x0 = tx - TAG_SIZE / 2 + c * cellsz
                        z0 = tag_z - TAG_SIZE / 2 + r * cellsz
                        m.box(BLACK, x0, x0 + cellsz, tag_y - 0.09, tag_y - 0.05, z0, z0 + cellsz)
    return m


def build_piece(color, diameter):
    m = Model(np.eye(3))
    m.add(color, trimesh.creation.icosphere(subdivisions=3, radius=diameter / 2))
    return m


# ---------------------------------------------------------------- write
def main():
    assets = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        "~/Library/Application Support/AdvantageScope/userAssets")
    field_dir = os.path.join(assets, "Field3d_BIOBUZZSim")
    build_field().export(os.path.join(field_dir, "model.glb"), "BIOBUZZSim")
    pieces = [("Pollen", POLLEN, POLLEN_D), ("Nectar (Blue)", NECTAR_BLUE, NECTAR_D), ("Nectar (Red)", NECTAR_RED, NECTAR_D)]
    for i, (name, color, d) in enumerate(pieces):
        build_piece(color, d).export(os.path.join(field_dir, f"model_{i}.glb"), name.replace(" ", ""))
    stock_driver_stations = [[2.515, -0.803], [2.515, 0.803], [-2.515, 0.803], [-2.515, -0.803]]
    with open(os.path.join(field_dir, "config.json"), "w") as out:
        json.dump({
            "name": "BIOBUZZ Sim (tilting hives)",
            "isFTC": True,
            "coordinateSystem": "center-rotated",
            "rotations": [],
            "position": [0, 0, 0],
            "widthInches": FIELD,
            "heightInches": FIELD,
            "driverStations": stock_driver_stations,
            "gamePieces": [{"name": n, "rotations": [], "position": [0, 0, 0], "stagedObjects": []}
                           for n, _, _ in pieces],
            "aprilTags": [],
        }, out, indent=2)
    for alliance in ("Red", "Blue"):
        robot_dir = os.path.join(assets, f"Robot_BIOBUZZ Hive {alliance}")
        build_tray(alliance).export(os.path.join(robot_dir, "model.glb"), f"Hive{alliance}")
        with open(os.path.join(robot_dir, "config.json"), "w") as out:
            json.dump({"name": f"BIOBUZZ Hive {alliance}", "isFTC": True, "rotations": [], "position": [0, 0, 0],
                       "cameras": [], "components": []}, out, indent=2)


if __name__ == "__main__":
    main()
