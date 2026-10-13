"""Writers for a tiny made-up San Andreas, in the game's own formats (and
ColAndreas's), so the tools can be tested without a real install."""

from __future__ import annotations

import math
import struct
from pathlib import Path

SECTOR = 2048


def box_tris(lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    c = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    out = []
    for a, b, cc, d in [(4, 5, 6, 7), (3, 2, 1, 0), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]:
        out += [(c[a], c[b], c[cc]), (c[a], c[cc], c[d])]
    return out


def col1(name, lo, hi):
    """A COLL (GTA3/VC style) entry: one box as faces, float vertices, u32 indices."""
    tris = box_tris(lo, hi)
    verts = sorted({p for t in tris for p in t})
    vi = {p: i for i, p in enumerate(verts)}
    body = name.encode()[:21].ljust(22, b"\0") + struct.pack("<H", 0) + struct.pack("<10f", 1, 0, 0, 0, *lo, *hi)
    body += struct.pack("<3I", 0, 0, 0)                                    # spheres, lines, boxes
    body += struct.pack("<I", len(verts)) + b"".join(struct.pack("<3f", *p) for p in verts)
    body += struct.pack("<I", len(tris)) + b"".join(struct.pack("<3I4B", *(vi[p] for p in t), 0, 0, 0, 0) for t in tris)
    return b"COLL" + struct.pack("<I", len(body)) + body


def col2_boxes(name, boxes):
    """A COL2 entry made of boxes only."""
    head_len = 116                                     # offsets count from after "COL2"
    pts = [p for b in boxes for p in b]
    lo = tuple(min(p[i] for p in pts) for i in range(3))
    hi = tuple(max(p[i] for p in pts) for i in range(3))
    head = name.encode()[:21].ljust(22, b"\0") + struct.pack("<H", 0)
    head += struct.pack("<10f", *lo, *hi, 0, 0, 0, 50)
    head += struct.pack("<HHHBBI", 0, len(boxes), 0, 0, 0, 2)
    head += struct.pack("<6I", 0, head_len, 0, 0, 0, 0)
    body = head.ljust(head_len - 4, b"\0") + b"".join(struct.pack("<6f4B", *lo_, *hi_, 0, 0, 0, 0) for lo_, hi_ in boxes)
    return b"COL2" + struct.pack("<I", len(body)) + body


def col3(name, tris):
    """A COL3 entry with faces (int16 vertices, 1/128 m)."""
    verts = sorted({p for t in tris for p in t})
    vi = {p: i for i, p in enumerate(verts)}
    pts = verts
    lo = tuple(min(p[i] for p in pts) for i in range(3))
    hi = tuple(max(p[i] for p in pts) for i in range(3))
    head_len = 116
    o_vert = head_len
    vbytes = b"".join(struct.pack("<3h", *(round(v * 128) for v in p)) for p in verts)
    vbytes = vbytes.ljust((len(vbytes) + 3) // 4 * 4, b"\0")
    o_face = o_vert + len(vbytes)
    fbytes = b"".join(struct.pack("<3H2B", *(vi[p] for p in t), 0, 0) for t in tris)
    head = name.encode()[:21].ljust(22, b"\0") + struct.pack("<H", 0)
    head += struct.pack("<10f", *lo, *hi, 0, 0, 0, 50)
    head += struct.pack("<HHHBBI", 0, 0, len(tris), 0, 0, 2)
    head += struct.pack("<6I", 0, 0, 0, o_vert, o_face, 0)
    head += struct.pack("<3I", 0, 0, 0)
    body = head.ljust(head_len - 4, b"\0") + vbytes + fbytes
    return b"COL3" + struct.pack("<I", len(body)) + body


def dff_with_col(col_entry: bytes) -> bytes:
    """A DFF that is only a clump carrying a collision plugin (enough for the reader)."""
    def chunk(kind, body):
        return struct.pack("<3I", kind, len(body), 0x1803FFFF) + body
    return chunk(0x10, chunk(0x01, struct.pack("<3I", 0, 0, 0)) + chunk(0x03, chunk(0x253F2FA, col_entry)))


def write_img(path: Path, files: dict):
    names = sorted(files)
    cursor = (8 + 32 * len(names) + SECTOR - 1) // SECTOR
    table, blobs = bytearray(), []
    for n in names:
        sectors = (len(files[n]) + SECTOR - 1) // SECTOR
        table += struct.pack("<IHH", cursor, sectors, 0) + n.encode().ljust(24, b"\0")
        blobs.append(files[n].ljust(sectors * SECTOR, b"\0"))
        cursor += sectors
    out = bytearray(b"VER2" + struct.pack("<I", len(names)) + table)
    out = out.ljust(((len(out) + SECTOR - 1) // SECTOR) * SECTOR, b"\0")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(bytes(out) + b"".join(blobs))


def binary_ipl(rows):
    """rows: (x, y, z, qx, qy, qz, qw, model)."""
    head = b"bnry" + struct.pack("<I", len(rows)) + b"\0" * 20 + struct.pack("<I", 76) + b"\0" * 44
    return head + b"".join(struct.pack("<7f3i", *r[:7], r[7], 0, -1) for r in rows)


def nodes_dat(vehicle, links_of, navi=(), navi_of=None, ped=()):
    """A nodes<N>.dat. vehicle and ped: (x, y, z[, flags]); links_of[i]: [(area, node)]
    for node i (vehicle first, then ped); navi: (x, y, area, node, dirx, diry, left
    (lanes along dir), right, width); navi_of[i]: the (area, navi id) of each of node i's links."""
    allnodes = list(vehicle) + list(ped)
    links, navilinks, rows = [], [], []
    for i, n in enumerate(allnodes):
        x, y, z = n[:3]
        flags = n[3] if len(n) > 3 else 0
        mine = links_of.get(i, [])
        rows.append(struct.pack("<2I3hhHHHBBI", 0, 0, round(x * 8), round(y * 8), round(z * 8), 0x7FFE,
                                len(links), 0, i, 16, 1, flags | len(mine)))
        links += mine
        navilinks += (navi_of or {}).get(i, [(0x3F, 0x3FF)] * len(mine))
    out = struct.pack("<5I", len(allnodes), len(vehicle), len(ped), len(navi), len(links)) + b"".join(rows)
    for x, y, area, node, dx, dy, left, right, width in navi:
        out += struct.pack("<hhHHbbI", round(x * 8), round(y * 8), area, node, round(dx * 100), round(dy * 100),
                           width | left << 8 | right << 11)
    out += b"".join(struct.pack("<HH", a, n) for a, n in links)
    out += b"\0" * 768
    out += b"".join(struct.pack("<H", a << 10 | n) for a, n in navilinks)
    out += b"\0" * len(links) * 2
    return out


def cadb(models: dict, placements):
    """ColAndreas's database: models {id: (spheres, boxes, faces)}, placements
    (model, x, y, z, qx, qy, qz, qw) with the rotation ready to use."""
    out = b"cadf" + struct.pack("<HHI", 2, len(models), len(placements))
    for mid, (spheres, boxes, faces) in models.items():
        out += struct.pack("<4H", mid, len(spheres), len(boxes), len(faces))
        out += b"".join(struct.pack("<4f", *s) for s in spheres)
        out += b"".join(struct.pack("<6f", *b) for b in boxes)
        out += b"".join(struct.pack("<9f", *(c for p in f for c in p)) for f in faces)
    out += b"".join(struct.pack("<H7f", *p) for p in placements)
    return out


def zquat_ipl(deg):
    """A rotation about z, as an IPL stores it (conjugated)."""
    s, c = math.sin(math.radians(deg) / 2), math.cos(math.radians(deg) / 2)
    return (0.0, 0.0, -s, c)


def vehicle_dff(frames, col_entry: bytes | None = None) -> bytes:
    """A vehicle DFF: a frame list (name, position, parent index) and, optionally, the
    collision plugin. Frames have no rotation."""
    def chunk(kind, body):
        return struct.pack("<3I", kind, len(body), 0x1803FFFF) + body
    data = struct.pack("<I", len(frames))
    for _name, pos, parent in frames:
        data += struct.pack("<12fiI", 1, 0, 0, 0, 1, 0, 0, 0, 1, *pos, parent, 0)
    exts = b"".join(chunk(0x03, chunk(0x253F2FE, name.encode())) for name, _p, _q in frames)
    body = chunk(0x01, struct.pack("<3I", 0, 0, 0)) + chunk(0x0E, chunk(0x01, data) + exts)
    if col_entry:
        body += chunk(0x03, chunk(0x253F2FA, col_entry))
    return chunk(0x10, body)
