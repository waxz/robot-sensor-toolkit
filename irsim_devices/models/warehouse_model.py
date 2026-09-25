"""Generate a realistic warehouse 3D model for LiDAR benchmarking.

Produces:
  warehouse.obj       — triangle mesh for Embree3D
  warehouse_segs.npy  — float32 [N,4] 2D segments for Embree2D
"""

from __future__ import annotations

import math
import os

import numpy as np

# ── Scene dimensions ──────────────────────────────────────────────────────────
W, D, H = 50.0, 30.0, 6.0  # width (X), depth (Y), height (Z)
WALL_T = 0.3


class MeshBuilder:
    """Accumulate box and cylinder primitives into a triangle mesh + 2D segments."""

    def __init__(self):
        self._verts: list[tuple] = []
        self._tris: list[tuple] = []
        self._segs: list[tuple] = []  # (ax,ay,bx,by) floor-plan edges

    # ── Primitives ─────────────────────────────────────────────────────────

    def add_box(self, x0, y0, z0, x1, y1, z1):
        """Add axis-aligned solid box from (x0,y0,z0) to (x1,y1,z1)."""
        b = len(self._verts)
        vs = [
            (x0, y0, z0),
            (x1, y0, z0),
            (x1, y1, z0),
            (x0, y1, z0),
            (x0, y0, z1),
            (x1, y0, z1),
            (x1, y1, z1),
            (x0, y1, z1),
        ]
        self._verts.extend(vs)
        self._tris += [
            (b + 0, b + 2, b + 1),
            (b + 0, b + 3, b + 2),  # -Z face
            (b + 4, b + 5, b + 6),
            (b + 4, b + 6, b + 7),  # +Z face
            (b + 0, b + 1, b + 5),
            (b + 0, b + 5, b + 4),  # -Y face
            (b + 2, b + 3, b + 7),
            (b + 2, b + 7, b + 6),  # +Y face
            (b + 0, b + 4, b + 7),
            (b + 0, b + 7, b + 3),  # -X face
            (b + 1, b + 2, b + 6),
            (b + 1, b + 6, b + 5),  # +X face
        ]
        # 2D silhouette: four perimeter edges projected to XY
        self._segs += [
            (x0, y0, x1, y0),  # south edge
            (x0, y1, x1, y1),  # north edge
            (x0, y0, x0, y1),  # west edge
            (x1, y0, x1, y1),  # east edge
        ]

    def add_cylinder(self, cx, cy, z0, z1, r, n=20):
        """Add vertical solid cylinder."""
        b = len(self._verts)
        angles = [2 * math.pi * i / n for i in range(n)]
        lo = [(cx + r * math.cos(a), cy + r * math.sin(a), z0) for a in angles]
        hi = [(cx + r * math.cos(a), cy + r * math.sin(a), z1) for a in angles]
        cli = b + 2 * n
        chi = b + 2 * n + 1
        self._verts.extend(lo + hi + [(cx, cy, z0), (cx, cy, z1)])
        for i in range(n):
            ni = (i + 1) % n
            # side quad
            self._tris += [(b + i, b + n + i, b + n + ni), (b + i, b + n + ni, b + ni)]
            # caps
            self._tris.append((cli, b + ni, b + i))
            self._tris.append((chi, b + n + i, b + n + ni))
        for i in range(n):
            ni = (i + 1) % n
            self._segs.append((lo[i][0], lo[i][1], lo[ni][0], lo[ni][1]))

    def add_wall_segment(self, ax, ay, bx, by, height=H, thickness=WALL_T):
        """Add a wall along line AB with given height and thickness."""
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return
        nx, ny = -dy / length * thickness / 2, dx / length * thickness / 2
        x0, y0 = ax - nx, ay - ny
        x1, y1 = bx - nx, by - ny
        x2, y2 = bx + nx, by + ny
        x3, y3 = ax + nx, ay + ny
        # Build box approximation
        self.add_box(
            min(x0, x1, x2, x3),
            min(y0, y1, y2, y3),
            0,
            max(x0, x1, x2, x3),
            max(y0, y1, y2, y3),
            height,
        )

    # ── Output ─────────────────────────────────────────────────────────────

    @property
    def vertices(self):
        return np.array(self._verts, dtype=np.float32)

    @property
    def triangles(self):
        return np.array(self._tris, dtype=np.int32)

    @property
    def segments_2d(self):
        return np.array(self._segs, dtype=np.float32)

    def triangle_soup(self):
        v, t = self.vertices, self.triangles
        return v[t]  # [T,3,3] float32

    def save_obj(self, path):
        with open(path, "w") as f:
            f.write("# Realistic warehouse model — irsim-devices Embree benchmark\n")
            f.write(
                f"# {W}m × {D}m × {H}m · {len(self._verts)} verts · {len(self._tris)} tris\n"
            )
            for v in self._verts:
                f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
            for t in self._tris:
                f.write(f"f {t[0] + 1} {t[1] + 1} {t[2] + 1}\n")
        print(
            f"  OBJ  : {len(self._verts):6d} verts, {len(self._tris):6d} tris → {path}"
        )

    def save_segments(self, path):
        segs = self.segments_2d
        np.save(path, segs)
        print(f"  Segs : {len(segs):6d} 2D segments → {path}")


# ── Scene construction ────────────────────────────────────────────────────────


def build_warehouse() -> MeshBuilder:
    """Return a MeshBuilder populated with a realistic 50×30×6 m warehouse."""
    m = MeshBuilder()

    # ── Floor & ceiling slabs ───────────────────────────────────────────
    m.add_box(0, 0, -0.15, W, D, 0.0)
    m.add_box(0, 0, H, W, D, H + 0.2)

    # ── Exterior walls ───────────────────────────────────────────────────
    t = WALL_T
    m.add_box(-t, -t, 0, 0, D + t, H)  # west
    m.add_box(W, -t, 0, W + t, D + t, H)  # east
    m.add_box(0, D, 0, W, D + t, H)  # north

    # South wall with three loading-dock openings (3.0 m wide × 3.5 m tall)
    dock_w, dock_h = 3.0, 3.5
    dock_xs = [5.0, 20.0, 35.0]
    segs = []
    prev = 0.0
    for dx in dock_xs:
        segs.append((prev, dx - dock_w / 2))
        segs.append((dx + dock_w / 2, dx + dock_w / 2))  # placeholder
        prev = dx + dock_w / 2
    # Build south wall panels
    xs = [0.0] + [x for dx in dock_xs for x in (dx - dock_w / 2, dx + dock_w / 2)] + [W]
    for i in range(0, len(xs), 2):
        x0, x1 = xs[i], xs[i + 1]
        if x1 > x0:
            m.add_box(x0, -t, 0, x1, 0, H)
    # Header above each dock door
    for dx in dock_xs:
        m.add_box(dx - dock_w / 2, -t, dock_h, dx + dock_w / 2, 0, H)

    # ── Steel columns (4×3 grid) ─────────────────────────────────────────
    col_xs = [10.0, 20.0, 30.0, 40.0]
    col_ys = [7.5, 15.0, 22.5]
    cs = 0.3  # column half-size
    for cx in col_xs:
        for cy in col_ys:
            m.add_box(cx - cs, cy - cs, 0, cx + cs, cy + cs, H)

    # ── Roof trusses ──────────────────────────────────────────────────────
    bt = 0.15  # beam half-thickness
    for cy in col_ys:
        m.add_box(0, cy - bt, H - 0.5, W, cy + bt, H)
    for cx in col_xs:
        m.add_box(cx - bt, 0, H - 0.5, cx + bt, D, H)

    # ── Shelving racks ────────────────────────────────────────────────────
    # 8 double-sided racks arranged in two rows, 4 bays each
    rl, rw, rh = 12.0, 1.2, 5.0
    shelf_zs = [1.0, 2.0, 3.0, 4.0]
    hw = rw / 2

    rack_origins = [
        # (x_start, y_center)
        (2.0, 4.5),
        (2.0, 11.0),
        (2.0, 18.0),
        (2.0, 25.5),
        (26.0, 4.5),
        (26.0, 11.0),
        (26.0, 18.0),
        (26.0, 25.5),
    ]
    for rx, ry in rack_origins:
        # Upright posts at each end
        m.add_box(rx, ry - hw, 0, rx + 0.08, ry + hw, rh)
        m.add_box(rx + rl - 0.08, ry - hw, 0, rx + rl, ry + hw, rh)
        # Intermediate posts every 2m
        for xi in range(2, int(rl), 2):
            m.add_box(rx + xi - 0.04, ry - hw, 0, rx + xi + 0.04, ry + hw, rh)
        # Shelf boards
        for sz in shelf_zs:
            m.add_box(rx, ry - hw, sz - 0.04, rx + rl, ry + hw, sz)
        # Back panel
        m.add_box(rx, ry - hw, 0, rx + rl, ry - hw + 0.06, rh)
        # Goods on shelves (simplified boxes)
        for sz in shelf_zs:
            for xi in range(0, int(rl) - 1, 2):
                m.add_box(
                    rx + xi + 0.1,
                    ry - hw + 0.1,
                    sz,
                    rx + xi + 1.8,
                    ry + hw - 0.1,
                    sz + 0.7,
                )

    # ── Ground-level pallets & boxes ─────────────────────────────────────
    pallets = [
        (14.0, 3.0),
        (14.0, 6.5),
        (14.0, 10.0),
        (14.0, 20.0),
        (14.0, 24.0),
        (17.0, 4.0),
        (17.0, 8.0),
        (17.0, 14.0),
        (17.0, 21.0),
        (17.0, 27.0),
        (21.0, 5.0),
        (21.0, 13.0),
        (21.0, 22.0),
        (38.0, 3.0),
        (38.0, 11.0),
        (38.0, 18.0),
        (38.0, 25.0),
        (43.0, 6.0),
        (43.0, 20.0),
    ]
    for px, py in pallets:
        m.add_box(px, py, 0.0, px + 1.2, py + 0.8, 0.15)
        bh = 0.6 + 0.05 * ((int(px * 3 + py * 7)) % 9)
        m.add_box(px + 0.1, py + 0.1, 0.15, px + 1.1, py + 0.7, 0.15 + bh)

    # ── Forklifts ─────────────────────────────────────────────────────────
    # Forklift 1
    m.add_box(22.0, 13.0, 0, 23.8, 14.8, 1.4)  # body
    m.add_box(22.0, 13.0, 1.4, 22.25, 14.8, 4.2)  # mast
    m.add_box(21.6, 13.2, 1.8, 22.0, 14.6, 1.95)  # forks
    # Forklift 2 (parked rotated 90°)
    m.add_box(23.5, 6.0, 0, 25.3, 7.8, 1.4)
    m.add_box(25.0, 6.0, 1.4, 25.3, 7.8, 4.2)

    # ── Mezzanine platform ────────────────────────────────────────────────
    m.add_box(44.0, 0.0, 2.5, W, 12.0, 2.7)  # deck slab
    m.add_box(44.0, 0.0, 0.0, 44.25, 12.0, 2.5)  # front support
    m.add_box(44.0, 12.0, 0.0, W, 12.25, 2.5)  # side railing
    # Staircase (10 steps)
    for s in range(10):
        m.add_box(42.5, s * 0.9, s * 0.25, 44.0, s * 0.9 + 0.9, s * 0.25 + 0.05)

    # ── Fire hose / utility pillars (small obstacles in aisles) ──────────
    utility_pts = [(12.5, 1.5), (12.5, 28.5), (24.5, 1.5), (24.5, 28.5)]
    for ux, uy in utility_pts:
        m.add_cylinder(ux, uy, 0, H, 0.15, n=12)

    return m


# ── Entry point ───────────────────────────────────────────────────────────────


def main():
    out_dir = os.path.dirname(os.path.abspath(__file__))
    print("Building warehouse model …")
    m = build_warehouse()
    v, t = m.vertices, m.triangles
    s = m.segments_2d
    print(f"  Vertices  : {len(v):6d}")
    print(f"  Triangles : {len(t):6d}")
    print(f"  2D segs   : {len(s):6d}")
    m.save_obj(os.path.join(out_dir, "warehouse.obj"))
    m.save_segments(os.path.join(out_dir, "warehouse_segs.npy"))
    print("Done.")


if __name__ == "__main__":
    main()
