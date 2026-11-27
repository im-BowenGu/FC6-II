#!/usr/bin/env python3
"""Generate a single movable-fin plate STL for UKROC 2027.

The geometry is deliberately simple: one trapezoid with a straight hinge
edge at x = 0, matching the planform recorded in the FluidX3D manifest
(root chord 74 mm, tip chord 50 mm, span 100 mm, leading-edge sweep 25 mm,
3 mm plate thickness). Rotating this mesh about the hinge and re-exporting
one STL per state is the staged path to movable-fin CFD documented in
`../cfd/README.md`.

No third-party modules are required.
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

ROOT_CHORD = 0.074
TIP_CHORD = 0.050
SPAN = 0.100
LEADING_EDGE_SWEEP = 0.025
HALF_THICKNESS = 0.0015

# (x, y) outline: hinge/root edge first, then leading edge, tip, trailing edge.
OUTLINE = [
    (0.0, 0.0),
    (LEADING_EDGE_SWEEP, SPAN),
    (LEADING_EDGE_SWEEP + TIP_CHORD, SPAN),
    (ROOT_CHORD, 0.0),
]


def triangle(a, b, c):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    nx, ny, nz = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
    mag = (nx * nx + ny * ny + nz * nz) ** 0.5 or 1.0
    normal = (nx / mag, ny / mag, nz / mag)
    return (normal, a, b, c)


def build_triangles():
    top = [(x, y, HALF_THICKNESS) for x, y in OUTLINE]
    bottom = [(x, y, -HALF_THICKNESS) for x, y in OUTLINE]
    tris = []
    for i in range(1, len(OUTLINE) - 1):
        tris.append(triangle(top[0], top[i], top[i + 1]))
        tris.append(triangle(bottom[0], bottom[i + 1], bottom[i]))
    for i in range(len(OUTLINE)):
        j = (i + 1) % len(OUTLINE)
        tris.append(triangle(top[i], bottom[i], bottom[j]))
        tris.append(triangle(top[i], bottom[j], top[j]))
    return tris


def write_binary_stl(path: Path) -> None:
    tris = build_triangles()
    with path.open("wb") as f:
        f.write(b"UKROC fin plate, hinge at x=0".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        for normal, a, b, c in tris:
            f.write(struct.pack("<12fH", *normal, *a, *b, *c, 0))


if __name__ == "__main__":
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_name("fin_plate.stl")
    write_binary_stl(out)
    print(f"wrote {out} ({len(build_triangles())} triangles)")
