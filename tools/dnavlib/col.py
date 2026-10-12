"""Collision: GTA's COL files (in archives, loose, or inside a model's DFF) and
ColAndreas's colandreas.cadb. Everything comes out as triangles, float32 [n, 3, 3],
in model space."""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np

EMPTY = np.zeros((0, 3, 3), np.float32)


def box(lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    c = np.array([(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                  (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)], dtype=np.float32)
    q = [(4, 5, 6, 7), (3, 2, 1, 0), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    idx = [(a, b, cc) for a, b, cc, d in q] + [(a, cc, d) for a, b, cc, d in q]
    return c[np.array(idx)]


def sphere(centre, r):
    """A sphere as the box round it: what matters to a walker is the footprint."""
    cx, cy, cz = centre
    return box((cx - r, cy - r, cz - r), (cx + r, cy + r, cz + r))


def _join(parts):
    return np.concatenate(parts).astype(np.float32) if parts else EMPTY


# ---------------------------------------------------------------------------
# COL1 .. COL4
# ---------------------------------------------------------------------------

def parse_col_entry(d: bytes, at: int):
    """One COL1/2/3/4 entry at `at`: (name, triangles, next offset). name is None
    when `at` is not the start of an entry."""
    four = d[at:at + 4]
    if len(d) < at + 32:
        return None, None, len(d)
    size = struct.unpack_from("<I", d, at + 4)[0]
    nxt = at + 8 + size
    name = d[at + 8:at + 30].split(b"\0")[0].decode("latin-1").lower()
    parts = []
    if four == b"COLL":
        p = at + 32 + 40                                    # name, id, bounds (r, c, min, max)
        ns = struct.unpack_from("<I", d, p)[0]; p += 4
        for k in range(ns):
            r, cx, cy, cz = struct.unpack_from("<4f", d, p + k * 20)
            parts.append(sphere((cx, cy, cz), r))
        p += ns * 20
        p += 4                                              # "unknown" (lines), always 0
        nb = struct.unpack_from("<I", d, p)[0]; p += 4
        for k in range(nb):
            b = struct.unpack_from("<6f", d, p + k * 28)
            parts.append(box(b[0:3], b[3:6]))
        p += nb * 28
        nv = struct.unpack_from("<I", d, p)[0]; p += 4
        verts = np.frombuffer(d, dtype="<f4", count=nv * 3, offset=p).reshape(nv, 3); p += nv * 12
        nf = struct.unpack_from("<I", d, p)[0]; p += 4
        faces = np.frombuffer(d, dtype="<u4", count=nf * 4, offset=p).reshape(nf, 4)[:, :3]
        if nf:
            parts.append(verts[faces.astype(np.int64)])
    elif four in (b"COL2", b"COL3", b"COL4"):
        base = at + 4                                       # offsets count from here
        h = at + 32 + 40
        nsph, nbox, nface, _, _, _flags = struct.unpack_from("<HHHBBI", d, h)
        o_sph, o_box, _, o_vert, o_face, _ = struct.unpack_from("<6I", d, h + 12)
        if nsph and o_sph:
            for k in range(nsph):
                cx, cy, cz, r = struct.unpack_from("<4f", d, base + o_sph + k * 20)
                parts.append(sphere((cx, cy, cz), r))
        if nbox and o_box:
            for k in range(nbox):
                b = struct.unpack_from("<6f", d, base + o_box + k * 28)
                parts.append(box(b[0:3], b[3:6]))
        if nface and o_face and o_vert:
            faces = np.frombuffer(d, dtype=np.dtype([("i", "<u2", 3), ("m", "u1", 2)]), count=nface,
                                  offset=base + o_face)["i"].astype(np.int64)
            nv = int(faces.max()) + 1
            verts = np.frombuffer(d, dtype="<i2", count=nv * 3, offset=base + o_vert).reshape(nv, 3)
            parts.append((verts.astype(np.float32) / 128.0)[faces])
    else:
        return None, None, len(d)
    return name, _join(parts), nxt


def parse_col_file(d: bytes) -> dict[str, np.ndarray]:
    """Every entry in a COL file (an archive's .col, a loose .col): name -> triangles."""
    out = {}
    at = 0
    while at + 8 <= len(d) and d[at:at + 3] == b"COL":
        name, tris, at = parse_col_entry(d, at)
        if name is not None:
            out.setdefault(name, tris)
    return out


def read_cols(imgs) -> dict[str, np.ndarray]:
    """model name -> its collision, from every .col in the archives."""
    out = {}
    for img in imgs:
        for arch in img.names(".col"):
            for name, tris in parse_col_file(img.read(arch)).items():
                out.setdefault(name, tris)
    return out


# ---------------------------------------------------------------------------
# collision inside a DFF (custom SA-MP models)
# ---------------------------------------------------------------------------

RW_CLUMP = 0x10
RW_EXTENSION = 0x03
RW_COLLISION = 0x253F2FA            # Rockstar's collision plugin: a COL entry


def dff_collision(d: bytes) -> np.ndarray | None:
    """The collision a DFF carries (how custom SA-MP objects get theirs), or None."""
    def walk(start, end):
        at = start
        while at + 12 <= end:
            kind, size = struct.unpack_from("<II", d, at)
            body, stop = at + 12, min(at + 12 + size, end)
            if kind == RW_COLLISION:
                cols = parse_col_file(d[body:stop])
                if cols:
                    return _join(list(cols.values()))
            elif kind in (RW_CLUMP, RW_EXTENSION):
                hit = walk(body, stop)
                if hit is not None:
                    return hit
            at = body + size
        return None
    return walk(0, len(d))


# ---------------------------------------------------------------------------
# ColAndreas's colandreas.cadb
# ---------------------------------------------------------------------------
#
# Read from the file format alone (no ColAndreas code is used):
#   "cadf", u16 version, u16 model count, u32 placement count
#   per model: u16 model id, u16 spheres, u16 boxes, u16 faces, then
#              spheres  (x, y, z, radius)              4 float32 each
#              boxes    (centre x, y, z, half x, y, z) 6 float32 each
#              faces    (three corners, x, y, z)       9 float32 each
#   per placement: u16 model id, float32 x, y, z, float32 qx, qy, qz, qw
# The placements are the stock map without LODs (the 0.3.DL objects' collision
# is in the models, under their SA-MP ids), the rotation ready to use as is.

CADB_MAGIC = b"cadf"


class Cadb:
    def __init__(self, models: dict[int, np.ndarray], placements: list, version: int):
        self.models = models
        self.placements = placements        # (model, x, y, z, qx, qy, qz, qw)
        self.version = version


def read_cadb(path: Path) -> Cadb:
    d = path.read_bytes()
    if d[:4] != CADB_MAGIC:
        raise SystemExit(f"{path} is not a ColAndreas database (no 'cadf' header)")
    version, nmodels, nplace = struct.unpack_from("<HHI", d, 4)
    if version != 2:
        print(f"note: {path.name} is ColAndreas database version {version}; this reader knows version 2")
    at = 12
    models = {}
    for _ in range(nmodels):
        mid, ns, nb, nf = struct.unpack_from("<4H", d, at)
        at += 8
        parts = []
        if ns:
            s = np.frombuffer(d, "<f4", ns * 4, at).reshape(ns, 4)
            parts += [sphere(row[:3], row[3]) for row in s]
            at += ns * 16
        if nb:
            b = np.frombuffer(d, "<f4", nb * 6, at).reshape(nb, 6)
            parts += [box(row[:3] - row[3:], row[:3] + row[3:]) for row in b]
            at += nb * 24
        if nf:
            parts.append(np.frombuffer(d, "<f4", nf * 9, at).reshape(nf, 3, 3))
            at += nf * 36
        models[mid] = _join(parts)
    place = []
    for _ in range(nplace):
        mid = struct.unpack_from("<H", d, at)[0]
        place.append((mid, *struct.unpack_from("<7f", d, at + 2)))
        at += 30
    if at > len(d):
        raise SystemExit(f"{path} is truncated")
    return Cadb(models, place, version)
