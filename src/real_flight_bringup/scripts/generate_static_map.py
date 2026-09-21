#!/usr/bin/env python3

import os
import math
import struct
import sys
import zlib
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class Obstacle:
    cx: float
    cy: float
    size_x: float
    size_y: float

    def bounds_xy(self) -> Tuple[float, float, float, float]:
        hx = 0.5 * float(self.size_x)
        hy = 0.5 * float(self.size_y)
        return (self.cx - hx, self.cx + hx, self.cy - hy, self.cy + hy)


def _maybe_load_yaml(path: str) -> Optional[dict]:
    try:
        import yaml  # type: ignore
    except Exception:
        return None

    if not os.path.exists(path):
        return None

    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _default_bounds_from_intention_real() -> Optional[Tuple[float, float, float, float, float, float]]:
    this_dir = os.path.dirname(os.path.abspath(__file__))
    intention_real = os.path.abspath(os.path.join(this_dir, "..", "config", "intention_real.yaml"))
    data = _maybe_load_yaml(intention_real)
    if not isinstance(data, dict):
        return None
    r3 = data.get("R3Bound")
    if not (isinstance(r3, list) and len(r3) == 6):
        return None
    try:
        return (float(r3[0]), float(r3[1]), float(r3[2]), float(r3[3]), float(r3[4]), float(r3[5]))
    except Exception:
        return None


def _arange_inclusive(start: float, stop: float, step: float) -> List[float]:
    if step <= 0:
        raise ValueError("step must be > 0")
    if stop < start:
        return []
    count = int(math.floor((stop - start) / step + 1.0 + 1e-9))
    return [float(start + step * i) for i in range(count)]


def generate_dense_prism_points(
    obs: Obstacle,
    spacing: float,
    z_min: float,
    z_max: float,
    bounds_xy: Tuple[float, float, float, float],
) -> List[Tuple[float, float, float]]:
    
    xmin, xmax, ymin, ymax = bounds_xy
    ox0, ox1, oy0, oy1 = obs.bounds_xy()
    ox0 = max(xmin, ox0)
    ox1 = min(xmax, ox1)
    oy0 = max(ymin, oy0)
    oy1 = min(ymax, oy1)
    if ox1 <= ox0 or oy1 <= oy0:
        return []

    xs = _arange_inclusive(ox0, ox1, spacing)
    ys = _arange_inclusive(oy0, oy1, spacing)
    zs = _arange_inclusive(z_min, z_max, spacing)
    if not xs or not ys or not zs:
        return []

    pts: List[Tuple[float, float, float]] = []
    for z in zs:
        for y in ys:
            for x in xs:
                pts.append((float(x), float(y), float(z)))
    return pts


def write_pcd_xyz_ascii(path: str, points_xyz: Sequence[Tuple[float, float, float]]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    n = int(len(points_xyz))
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z",
            "SIZE 4 4 4",
            "TYPE F F F",
            "COUNT 1 1 1",
            f"WIDTH {n}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {n}",
            "DATA ascii",
        ]
    )

    with open(path, "w", encoding="utf-8") as f:
        f.write(header)
        f.write("\n")
        # Use a stable formatting; keep file size reasonable.
        for x, y, z in points_xyz:
            f.write(f"{float(x):.4f} {float(y):.4f} {float(z):.4f}\n")


def _png_chunk(tag: bytes, data: bytes) -> bytes:
    if len(tag) != 4:
        raise ValueError("PNG chunk tag must be 4 bytes")
    length = struct.pack(">I", len(data))
    crc = struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    return length + tag + data + crc


def write_png_rgb8(path: str, rgb: Sequence[Sequence[Tuple[int, int, int]]]) -> None:
    if not rgb or not rgb[0]:
        raise ValueError("rgb must be a non-empty HxW array")

    h = len(rgb)
    w = len(rgb[0])

    # PNG raw scanlines: each row starts with filter type 0.
    raw_rows = []
    for row in range(h):
        scan = bytearray()
        scan.append(0)
        for col in range(w):
            r, g, b = rgb[row][col]
            scan.extend(bytes((int(r) & 0xFF, int(g) & 0xFF, int(b) & 0xFF)))
        raw_rows.append(bytes(scan))
    raw = b"".join(raw_rows)
    compressed = zlib.compress(raw, level=9)

    signature = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)  # 8-bit, truecolor
    png = (
        signature
        + _png_chunk(b"IHDR", ihdr)
        + _png_chunk(b"IDAT", compressed)
        + _png_chunk(b"IEND", b"")
    )

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "wb") as f:
        f.write(png)


def rasterize_obstacles(
    obstacles: Sequence[Obstacle],
    bounds_xy: Tuple[float, float, float, float],
    meters_per_pixel: float,
) -> List[List[Tuple[int, int, int]]]:
    xmin, xmax, ymin, ymax = bounds_xy
    if meters_per_pixel <= 0:
        raise ValueError("meters_per_pixel must be > 0")

    w = int(math.ceil((xmax - xmin) / meters_per_pixel))
    h = int(math.ceil((ymax - ymin) / meters_per_pixel))
    w = max(1, w)
    h = max(1, h)

    grid = [[0 for _ in range(w)] for _ in range(h)]

    for obs in obstacles:
        ox0, ox1, oy0, oy1 = obs.bounds_xy()
        # clamp
        ox0 = max(xmin, ox0)
        ox1 = min(xmax, ox1)
        oy0 = max(ymin, oy0)
        oy1 = min(ymax, oy1)
        if ox1 <= ox0 or oy1 <= oy0:
            continue

        ix0 = int(math.floor((ox0 - xmin) / meters_per_pixel))
        ix1 = int(math.ceil((ox1 - xmin) / meters_per_pixel))
        iy0 = int(math.floor((oy0 - ymin) / meters_per_pixel))
        iy1 = int(math.ceil((oy1 - ymin) / meters_per_pixel))

        ix0 = max(0, min(w, ix0))
        ix1 = max(0, min(w, ix1))
        iy0 = max(0, min(h, iy0))
        iy1 = max(0, min(h, iy1))
        if ix1 <= ix0 or iy1 <= iy0:
            continue

        for yy in range(iy0, iy1):
            for xx in range(ix0, ix1):
                grid[yy][xx] = 255

    # Make Y-up for a top-down map display.
    grid = list(reversed(grid))

    # Obstacles as black, free space as white.
    rgb: List[List[Tuple[int, int, int]]] = []
    for row in grid:
        rgb_row: List[Tuple[int, int, int]] = []
        for value in row:
            free = 255 - int(value)
            rgb_row.append((free, free, free))
        rgb.append(rgb_row)
    return rgb


def main() -> int:
    # ---- User settings (edit these) ----
    obstacles: List[Obstacle] = [
        # Obstacle(cx, cy, size_x, size_y)
        # Per user request:
        Obstacle(-1.0, 0.0, 0.1, 6.0),   # at (-1,0) size 0.1 x 6
        Obstacle(2.0, -2.5, 6.0, 0.1),   # at (2.0,-2.5) size 6 x 0.1
        Obstacle(1.0, 0.0, 4.0, 0.1),   # at (1.0,0.0) size 4 x 0.1
        Obstacle(3.5, 2.5, 3.0, 0.1),  # at (3.5,2.5) size 3 x 0.1
        Obstacle(4.7, 0.0, 0.1, 6.0),  # at (4.5,0.0) size 0.1 x 6
        Obstacle(2.0, 3.0, 0.1, 1.25),  # at (2.0,3.0) size 0.1 x 1.25
    ]

    # Optional: load obstacles from a YAML file (same schema as before).
    # If set, this will APPEND to the list above.
    obs_yaml_path = ""  # e.g. "/home/liet/obs.yaml"

    # Optional: override bounds (XMIN, XMAX, YMIN, YMAX).
    # If None, it loads from intention_real.yaml (R3Bound) or falls back to [-2, 12, -2, 12].
    bounds_override = None  # e.g. (-2.0, 12.0, -2.0, 12.0)

    # Output paths
    this_dir = os.path.dirname(os.path.abspath(__file__))
    out_pcd = os.path.abspath(os.path.join(this_dir, "..", "config", "maps", "static_map.pcd"))
    out_png = os.path.abspath(os.path.join(this_dir, "..", "config", "maps", "static_map.png"))

    # Density controls
    pcd_spacing = 0.05  # meters
    png_mpp = 0.05      # meters per pixel

    # Fixed height (spec)
    z_min = 0.0
    z_max = 2.0
    # ---- End user settings ----

    if obs_yaml_path:
        data = _maybe_load_yaml(obs_yaml_path)
        if not isinstance(data, dict) or "obstacles" not in data:
            raise ValueError("Invalid obs_yaml_path: expected dict with key 'obstacles'")
        items = data["obstacles"]
        if not isinstance(items, list):
            raise ValueError("Invalid obs_yaml_path: obstacles must be a list")
        for it in items:
            if not isinstance(it, dict):
                raise ValueError("Invalid obs_yaml_path: each obstacle must be a dict")
            obstacles.append(
                Obstacle(
                    float(it["cx"]),
                    float(it["cy"]),
                    float(it["size_x"]),
                    float(it["size_y"]),
                )
            )

    # Spec: obstacle height is fixed to 2m.
    z_min = 0.0
    z_max = 2.0

    r3_default = _default_bounds_from_intention_real()
    if bounds_override is not None:
        xmin, xmax, ymin, ymax = [float(v) for v in bounds_override]
    elif r3_default is not None:
        xmin, xmax, ymin, ymax = r3_default[0], r3_default[1], r3_default[2], r3_default[3]
    else:
        xmin, xmax, ymin, ymax = -2.0, 12.0, -6.0, 6.0

    if xmax <= xmin or ymax <= ymin:
        raise ValueError("Invalid bounds")
    if pcd_spacing <= 0:
        raise ValueError("pcd-spacing must be > 0")
    if png_mpp <= 0:
        raise ValueError("png-mpp must be > 0")

    bounds_xy = (xmin, xmax, ymin, ymax)

    # Build point cloud
    all_points: List[List[Tuple[float, float, float]]] = []
    for obs in obstacles:
        pts = generate_dense_prism_points(
            obs,
            spacing=float(pcd_spacing),
            z_min=z_min,
            z_max=z_max,
            bounds_xy=bounds_xy,
        )
        if len(pts) > 0:
            all_points.append(pts)

    if all_points:
        points_xyz = [pt for pts in all_points for pt in pts]
    else:
        points_xyz = []

    # De-duplicate (optional; keeps file size reasonable for dense grids)
    if points_xyz:
        q = max(float(pcd_spacing), 1e-6)
        seen = set()
        deduped: List[Tuple[float, float, float]] = []
        for x, y, z in points_xyz:
            key = (int(round(x / q)), int(round(y / q)), int(round(z / q)))
            if key in seen:
                continue
            seen.add(key)
            deduped.append((float(x), float(y), float(z)))
        points_xyz = deduped

    write_pcd_xyz_ascii(out_pcd, points_xyz)

    # Rasterize PNG
    rgb = rasterize_obstacles(obstacles, bounds_xy=bounds_xy, meters_per_pixel=float(png_mpp))
    write_png_rgb8(out_png, rgb)

    print("Generated static map:")
    print(f"  obstacles: {len(obstacles)}")
    print(f"  bounds_xy: {bounds_xy}")
    print(f"  z_range:   [{z_min}, {z_max}] m")
    print(f"  pcd:       {out_pcd} (points={int(len(points_xyz))}, spacing={pcd_spacing})")
    print(f"  png:       {out_png} (meters_per_pixel={png_mpp}, size={len(rgb[0])}x{len(rgb)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
