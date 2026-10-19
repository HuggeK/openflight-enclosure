#!/usr/bin/env python3
"""Find the screw and heat-set insert holes in every enclosure STL.

The script slices each STL into cross-sections every STEP mm along X, Y and Z,
collects the small round holes in each slice, joins slices that belong to the
same hole, and fits a straight axis through them. For every hole it records:

  * diameter (measured square to the hole axis)
  * axis direction and tilt
  * depth and whether it is blind or runs through
  * the open end an insert or screw goes in from
  * a size class guessed from the diameter (see CLASSES)

Output: docs/viewer/data/holes.json, read by the 3D viewer in docs/viewer/.

Usage:
    pip install -r tools/insert-map/requirements.txt
    python tools/insert-map/analyze.py            # writes docs/viewer/data/holes.json
    python tools/insert-map/analyze.py --check    # exit 1 if holes.json is out of date
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import trimesh
from scipy.cluster.hierarchy import fcluster, linkage
from shapely.geometry import Point, Polygon

ROOT = Path(__file__).resolve().parents[2]
STL_DIR = ROOT / "stls"
OUT = ROOT / "docs" / "viewer" / "data" / "holes.json"

STEP = 0.2            # slice spacing, mm
MIN_D, MAX_D = 1.4, 5.0   # hole diameters to report, mm (case-screw channels are ~6 mm)
MIN_SLICES = 3        # a hole must show up in at least this many slices

# Size classes by measured diameter (mm). Order matters: first match wins.
CLASSES = [
    # (kind, label, d_min, d_max)
    ("m3-insert", "M3 heat-set insert", 4.15, 4.8),
    ("m25-insert", "M2.5 heat-set insert", 3.15, 3.6),
    ("m2-insert", "M2 heat-set insert", 2.65, 3.15),
    ("pilot", "Self-tapping pilot hole", 1.4, 2.65),
]
CLEARANCE_MAX_LEN = 2.5   # a short hole open at both ends is a clearance hole, not an insert pocket
MIN_DEPTH = 1.5           # shorter blind "holes" are chamfers or slots, not fastener holes

# What each hole is for. Geometry alone cannot tell an IWR mount from an OPS mount, so these
# rules name holes by part and position (enclosure coordinates, mm: X across, Y up, Z out the front).
# First matching rule wins. Keys: group, file (substring of the STL name), kind, tilt_min, tilt_max,
# x, y (inclusive ranges on the entry point). Set "kind"/"label" in "set" to correct a guess.
RULES = [
    dict(group="radar", kind="m3-insert", tilt_max=1, role="Case mount: screw comes from the rear of the shell"),
    dict(group="radar", tilt_min=5, x=[0, 86], role="IWR6843 mount"),
    dict(group="radar", tilt_min=5, x=[86, 200], role="OPS243 mount"),
    dict(group="camera", kind="m3-insert", role="Case mount: screw comes from the rear of the shell"),
    dict(group="camera", file="Sound-Detector", x=[20, 70], role="Camera (OV9281)"),
    dict(group="camera", file="Sound-Detector", x=[100, 180], role="Sound-detector retainer"),
    dict(group="camera", role="Camera (OV9281)"),
    dict(group="screen", kind="m3-insert", tilt_max=5, role="Case mount: screw comes from the rear of the shell"),
    dict(group="screen", kind="m3-insert", tilt_min=5, role="Display panel mount"),
    dict(group="screen", kind="clearance", role="Display mounting screw"),
    dict(group="shell", kind="m25-insert", tilt_min=5, role="UPS / Raspberry Pi stack"),
    dict(group="shell", kind="m25-insert", y=[0, 12], role="Sensor bay (Adafruit breakout)"),
    dict(group="shell", kind="m3-insert", y=[0, 12], role="Adjustable foot (skip with solid feet)"),
    dict(group="adapters", file="x1209", set=dict(kind="clearance", label="Clearance hole (M2.5 screw)"),
         role="Pi / HAT screw"),
]


def apply_rules(group: str, name: str, hole: dict) -> None:
    x, y, _ = hole["entry"]
    for r in RULES:
        if r["group"] != group or r.get("file", "") not in name:
            continue
        if "kind" in r and r["kind"] != hole["kind"]:
            continue
        if hole["tilt"] < r.get("tilt_min", -1) or hole["tilt"] > r.get("tilt_max", 999):
            continue
        if "x" in r and not r["x"][0] <= x <= r["x"][1]:
            continue
        if "y" in r and not r["y"][0] <= y <= r["y"][1]:
            continue
        hole.update(r.get("set", {}))
        hole["role"] = r["role"]
        return


def classify(d: float, length: float, blind: bool) -> tuple[str, str]:
    if not blind and length <= CLEARANCE_MAX_LEN:
        return "clearance", f"Clearance hole (screw passes through, ~M{max(2, round(d - 0.4, 1)):g})"
    for kind, label, lo, hi in CLASSES:
        if lo <= d < hi:
            return kind, label
    return "other", "Other hole"


def _polygons(section):
    if section is None:
        return [], None
    path2d, to_3d = section.to_2D()
    try:
        return list(path2d.polygons_full), to_3d
    except Exception:  # broken slice on a non-watertight mesh
        return [], to_3d


def inside(mesh: trimesh.Trimesh, p: np.ndarray, normal: np.ndarray) -> bool:
    """Point-in-material test that also works on meshes that are not watertight.

    Slices the mesh through p and checks whether p lies inside a filled region of that slice.
    """
    polys, to_3d = _polygons(mesh.section(plane_origin=p, plane_normal=normal))
    if not polys:
        return False
    q = trimesh.transform_points([p], np.linalg.inv(to_3d))[0][:2]
    pt = Point(q)
    return any(poly.contains(pt) for poly in polys)


def scan_axis(mesh: trimesh.Trimesh, axis: int) -> np.ndarray:
    """Return rows of (x, y, z, diameter) for round holes seen in slices along one axis."""
    n = np.zeros(3)
    n[axis] = 1.0
    lo, hi = mesh.bounds[:, axis]
    rows = []
    for h in np.arange(lo + STEP / 2, hi, STEP):
        o = np.zeros(3)
        o[axis] = h
        polys, to_3d = _polygons(mesh.section(plane_origin=o, plane_normal=n))
        for poly in polys:
            for ring in poly.interiors:
                r = Polygon(ring)
                if r.length == 0:
                    continue
                d = 2 * np.sqrt(r.area / np.pi)
                circ = 4 * np.pi * r.area / r.length**2
                if circ > 0.88 and MIN_D * 0.9 <= d <= MAX_D * 1.25:
                    c = trimesh.transform_points([[*r.centroid.coords[0], 0]], to_3d)[0]
                    rows.append([*c, d])
    return np.array(rows) if rows else np.zeros((0, 4))


def true_diameter(mesh, centre, axis):
    """Diameter measured in a slice square to the hole axis."""
    polys, to_3d = _polygons(mesh.section(plane_origin=centre, plane_normal=axis))
    if not polys:
        return None
    q = Point(trimesh.transform_points([centre], np.linalg.inv(to_3d))[0][:2])
    best = None
    for poly in polys:
        for ring in poly.interiors:
            r = Polygon(ring)
            if r.contains(q):
                d = 2 * np.sqrt(r.area / np.pi)
                best = d if best is None else min(best, d)
    return best


def find_holes(mesh: trimesh.Trimesh) -> list[dict]:
    candidates = []
    for axis in range(3):
        pts = scan_axis(mesh, axis)
        if len(pts) < MIN_SLICES:
            continue
        labels = fcluster(linkage(pts[:, :3], "single"), STEP * 3.5, "distance")
        for lab in np.unique(labels):
            P = pts[labels == lab]
            if len(P) < MIN_SLICES:
                continue
            c0 = P[:, :3].mean(0)
            u = np.linalg.svd(P[:, :3] - c0)[2][0]
            # keep the slice axis as the direction for short holes where SVD is noisy
            if len(P) < 6:
                u = np.zeros(3)
                u[axis] = 1.0
            u /= np.linalg.norm(u)
            t = (P[:, :3] - c0) @ u
            candidates.append(dict(c0=c0, u=u, t0=t.min(), t1=t.max(), n=len(P), d_slice=float(np.median(P[:, 3]))))

    # the same hole is found from more than one slicing axis: keep the best-sampled copy
    candidates.sort(key=lambda c: -c["n"])
    kept = []
    for c in candidates:
        mid = c["c0"] + c["u"] * (c["t0"] + c["t1"]) / 2
        if all(np.linalg.norm(mid - (k["c0"] + k["u"] * (k["t0"] + k["t1"]) / 2)) > 3.0 for k in kept):
            kept.append(c)

    holes = []
    for c in kept:
        u, c0 = c["u"], c["c0"]
        a = c0 + u * c["t0"]
        b = c0 + u * c["t1"]
        mid = (a + b) / 2
        d = true_diameter(mesh, mid, u) or c["d_slice"]
        if not (MIN_D <= d <= MAX_D):
            continue
        length = float(np.linalg.norm(b - a)) + STEP
        a_closed = inside(mesh, a - u * 0.6, u)
        b_closed = inside(mesh, b + u * 0.6, u)
        blind = a_closed != b_closed
        if blind:
            entry, into = (b, -u) if a_closed else (a, u)
        else:
            # through hole: report the end that faces the inside of the enclosure (-Z, the back)
            entry, into = (a, u) if a[2] <= b[2] else (b, -u)
        tilt = float(np.degrees(np.arccos(min(1.0, max(abs(into[0]), abs(into[1]), abs(into[2]))))))
        kind, label = classify(d, length, blind)
        if length < MIN_DEPTH and kind != "clearance":
            continue
        holes.append(dict(
            kind=kind,
            label=label,
            d=round(float(d), 2),
            depth=round(length, 1),
            blind=bool(blind),
            entry=[round(float(v), 2) for v in entry],
            dir=[round(float(v), 4) for v in into],
            tilt=round(tilt, 1),
        ))
    holes.sort(key=lambda h: (-h["entry"][1], h["entry"][0], h["entry"][2]))
    return holes


def describe(path: Path) -> dict:
    rel = path.relative_to(ROOT).as_posix()
    parts = path.relative_to(STL_DIR).parts
    group = parts[0]
    status = parts[1] if len(parts) > 2 else "v1"
    mesh = trimesh.load(path, force="mesh")
    holes = find_holes(mesh)
    for h in holes:
        apply_rules(group, path.stem, h)
    return dict(
        path=rel,
        group=group,
        status=status,
        name=path.stem,
        bounds=[[round(float(v), 2) for v in row] for row in mesh.bounds],
        holes=holes,
    )


def build() -> dict:
    files = sorted(STL_DIR.rglob("*.stl"))
    out = []
    for p in files:
        print(f"  {p.relative_to(ROOT)}", file=sys.stderr, flush=True)
        out.append(describe(p))
    return dict(
        generated_by="tools/insert-map/analyze.py",
        units="mm",
        classes=[dict(kind=k, label=l, d_min=lo, d_max=hi) for k, l, lo, hi in CLASSES],
        parts=out,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="exit 1 if holes.json does not match the STLs")
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()

    data = build()
    text = json.dumps(data, indent=1) + "\n"
    if args.check:
        current = args.out.read_text() if args.out.exists() else ""
        if current != text:
            print(f"{args.out.relative_to(ROOT)} is out of date. Run: python tools/insert-map/analyze.py", file=sys.stderr)
            return 1
        print("holes.json is up to date", file=sys.stderr)
        return 0
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text)
    n = sum(len(p["holes"]) for p in data["parts"])
    print(f"wrote {args.out.relative_to(ROOT)}: {len(data['parts'])} parts, {n} holes", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
