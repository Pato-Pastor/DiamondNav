"""A plan view as a PNG: the baked walkable polygons (navbake --dump) shaded by height,
the routes navbake checked (navbake --paths: green found, red not), and the road
graph (roads.dnr: one-way links drawn thinner). Needs Pillow."""

from __future__ import annotations

import struct
from pathlib import Path

PAD = 20
MAX_SIZE = 4096


def load_polys(path: Path):
    verts, polys = [], []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("v "):
            verts.append(tuple(float(k) for k in line.split()[1:4]))
        elif line.startswith("f "):
            polys.append([verts[int(k) - 1] for k in line.split()[1:]])
    return polys


def load_paths(path: Path):
    out = []
    for line in path.read_text(encoding="utf-8").splitlines():
        cols = line.split("\t")
        out.append((cols[0], int(cols[1]), [tuple(float(k) for k in c.split()) for c in cols[2:]]))
    return out


def load_roads(path: Path):
    d = path.read_bytes()
    _, _, nn, ne = struct.unpack_from("<4I", d, 0)
    nodes = [struct.unpack_from("<3fIHH", d, 16 + i * 20) for i in range(nn)]
    at = 16 + nn * 20
    edges = [struct.unpack_from("<IfBBH", d, at + i * 12) for i in range(ne)]
    out = []
    for i, (x, y, z, first, count, _) in enumerate(nodes):
        for to, _, lanes, back, _ in edges[first:first + count]:
            if to > i or not back:       # each two-way link once
                out.append(((x, y, z), nodes[to][:3], lanes + back, bool(back)))
    return out


def draw(out: Path, polys_obj: Path | None = None, paths_txt: Path | None = None, roads_dnr: Path | None = None,
         scale: float | None = None):
    from PIL import Image, ImageDraw
    polys = load_polys(polys_obj) if polys_obj and polys_obj.exists() else []
    paths = load_paths(paths_txt) if paths_txt and paths_txt.exists() else []
    roads = load_roads(roads_dnr) if roads_dnr and roads_dnr.exists() else []
    pts = [p for pl in polys for p in pl] + [p for _, _, r in paths for p in r] + [p for a, b, _, _ in roads for p in (a, b)]
    if not pts:
        raise SystemExit("nothing to draw")
    x0, x1 = min(p[0] for p in pts), max(p[0] for p in pts)
    y0, y1 = min(p[1] for p in pts), max(p[1] for p in pts)
    z0, z1 = min(p[2] for p in pts), max(p[2] for p in pts)
    if scale is None:
        scale = min(6.0, (MAX_SIZE - 2 * PAD) / max(x1 - x0, y1 - y0, 1.0))
    w, h = int((x1 - x0) * scale) + 2 * PAD, int((y1 - y0) * scale) + 2 * PAD

    def px(p):
        return (PAD + (p[0] - x0) * scale, PAD + (y1 - p[1]) * scale)

    img = Image.new("RGB", (w, h), (24, 26, 30))
    d = ImageDraw.Draw(img)
    # lower floors first, so an upper floor covers what is under it
    for poly in sorted(polys, key=lambda pl: sum(p[2] for p in pl) / len(pl)):
        z = sum(p[2] for p in poly) / len(poly)
        t = (z - z0) / max(z1 - z0, 1e-3)
        c = (int(60 + 120 * t), int(90 + 110 * t), int(130 + 110 * t))
        d.polygon([px(p) for p in poly], fill=c, outline=(30, 40, 55) if scale >= 2 else None)
    for a, b, lanes, two_way in roads:
        d.line([px(a), px(b)], fill=(240, 180, 60) if two_way else (240, 120, 200),
               width=max(1, int(min(lanes, 4) * scale * 0.5)) if scale >= 1 else 1)
    for _, status, route in paths:
        colour = (60, 220, 90) if status == 0 else (230, 60, 60)
        if len(route) > 1:
            d.line([px(p) for p in route], fill=colour, width=3)
        for p in route[:1] + route[-1:]:
            x, y = px(p)
            d.ellipse((x - 4, y - 4, x + 4, y + 4), fill=colour)
    parts = []
    if polys:
        parts.append(f"{len(polys)} walkable polygons, height {z0:.1f} to {z1:.1f} m (lighter is higher)")
    if roads:
        parts.append(f"{len(roads)} road links")
    if paths:
        parts.append(f"{sum(1 for p in paths if p[1] == 0)}/{len(paths)} routes found")
    d.text((PAD, 4), "; ".join(parts), fill=(230, 230, 230))
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out)
