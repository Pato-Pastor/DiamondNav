"""A GTA San Andreas install: archives, model names, placed objects, water."""

from __future__ import annotations

import re
import struct
from pathlib import Path

SECTOR = 2048


class Img:
    """An IMG v2 archive (gta3.img, gta_int.img): name -> bytes, read lazily."""

    def __init__(self, path: Path):
        self.path = path
        self.fh = open(path, "rb")
        if self.fh.read(4) != b"VER2":
            raise SystemExit(f"{path} is not an IMG v2 archive (GTA SA's)")
        n = struct.unpack("<I", self.fh.read(4))[0]
        self.entries = {}
        for _ in range(n):
            off, size, size2, raw = struct.unpack("<IHH24s", self.fh.read(32))
            name = raw.split(b"\0")[0].decode("latin-1").lower()
            self.entries[name] = (off, size or size2)

    def names(self, suffix: str):
        return [n for n in self.entries if n.endswith(suffix)]

    def has(self, name: str) -> bool:
        return name.lower() in self.entries

    def close(self):
        self.fh.close()

    def read(self, name: str) -> bytes:
        off, size = self.entries[name.lower()]
        self.fh.seek(off * SECTOR)
        return self.fh.read(size * SECTOR)


def find(root: Path, rel: str) -> Path | None:
    """rel under root, whatever the case of each part (installs differ)."""
    cur = root
    for part in rel.split("/"):
        if not cur.is_dir():
            return None
        hit = next((p for p in cur.iterdir() if p.name.lower() == part.lower()), None)
        if hit is None:
            return None
        cur = hit
    return cur


def open_imgs(gta: Path) -> list[Img]:
    """The game's archives, and SA-MP's (its own objects) when the install has them."""
    imgs = []
    for rel in ("models/gta3.img", "models/gta_int.img", "samp/samp.img", "samp/custom.img"):
        p = find(gta, rel)
        if p is not None:
            imgs.append(Img(p))
    if not imgs:
        raise SystemExit(f"{gta}: no models/gta3.img; is this a GTA San Andreas folder?")
    return imgs


def read_ides(gta: Path) -> dict[int, str]:
    """model id -> lower-case name, from every IDE under data/ (SA-MP's own objects
    included when the install has SAMP/samp.ide, as SA-MP and 0.3.DL installs do)."""
    out = {}
    files = []
    for sub in ("data", "samp", "SAMP"):
        d = find(gta, sub)
        if d is not None:
            files += sorted(p for p in d.rglob("*") if p.suffix.lower() == ".ide")
    for ide in dict.fromkeys(files):
        sect = None
        for line in ide.read_text(encoding="latin-1", errors="ignore").splitlines():
            t = line.split("#")[0].strip()
            low = t.lower()
            if low in ("objs", "tobj", "anim", "cars", "peds", "weap", "hier", "txdp", "2dfx", "path", "tanm"):
                sect = low
            elif low == "end":
                sect = None
            elif t and sect in ("objs", "tobj", "anim"):
                f = [x.strip() for x in t.split(",")]
                try:
                    out.setdefault(int(f[0]), f[1].lower())
                except (ValueError, IndexError):
                    pass
    return out


def read_breakables(gta: Path) -> set[str]:
    """Lower-case names of the objects a car goes through in the game (data/object.dat):
    ones with a collision damage effect (they break, smash or change when hit: fences,
    railings, glass, lamp posts, traffic lights, saplings) and ones light enough to be
    shoved aside (1500 kg or less: bins, cones, boxes, bollards). Fixed objects (mass
    99999) stay solid, and so do locked doors (special collision response 7) and
    anything heavier with no damage effect (containers, rocks), even if it can be
    uprooted."""
    path = find(gta, "data/object.dat")
    if path is None:
        return set()
    out = set()
    for line in path.read_text(encoding="latin-1", errors="ignore").splitlines():
        t = line.split(";")[0].strip()
        if not t or t.startswith("*"):
            continue
        f = [x for x in re.split(r"[,\s]+", t) if x]
        # name, mass, turn mass, air resistance, elasticity, % submerged, uproot limit,
        # collision damage multiplier, collision damage effect, special collision response, ...
        try:
            mass, effect, special = float(f[1]), int(float(f[8])), int(float(f[9]))
        except (ValueError, IndexError):
            continue
        if mass >= 99000.0 or special == 7:
            continue
        if effect != 0 or mass <= 1500.0:
            out.add(f[0].lower())
    return out


def read_ipls(gta: Path, imgs: list[Img]):
    """Every placed object: (model, x, y, z, qx, qy, qz, qw), text and binary IPLs,
    the rotation as stored (conjugated: see world.py)."""
    out = []
    maps = find(gta, "data/maps")
    for p in sorted(maps.rglob("*")) if maps else []:
        if p.suffix.lower() != ".ipl":
            continue
        sect = None
        for line in p.read_text(encoding="latin-1", errors="ignore").splitlines():
            t = line.split("#")[0].strip()
            if t.lower() in ("inst", "cull", "path", "grge", "enex", "pick", "cars", "jump", "tcyc", "auzo", "mult", "occl", "zone"):
                sect = t.lower()
            elif t.lower() == "end":
                sect = None
            elif sect == "inst":
                f = [x.strip() for x in t.split(",")]
                if len(f) >= 10:
                    try:
                        out.append((int(f[0]), *(float(v) for v in f[3:10])))
                    except ValueError:
                        pass
    for img in imgs:
        for name in img.names(".ipl"):
            d = img.read(name)
            if d[:4] != b"bnry":
                continue
            count, at = struct.unpack_from("<I", d, 4)[0], struct.unpack_from("<I", d, 28)[0]
            for k in range(count):
                x, y, z, qx, qy, qz, qw, mdl, _, _ = struct.unpack_from("<7f3i", d, at + k * 40)
                out.append((mdl, x, y, z, qx, qy, qz, qw))
    return out


def read_water(gta: Path):
    """(x0, y0, x1, y1, level) for each quad or triangle in data/water.dat."""
    p = find(gta, "data/water.dat")
    return parse_water(p.read_text(encoding="latin-1", errors="ignore")) if p else []


def parse_water(text: str):
    out = []
    for line in text.splitlines():
        try:
            v = [float(x) for x in line.split()]
        except ValueError:
            continue
        nverts = 4 if len(v) >= 29 else 3 if len(v) >= 22 else 0
        if not nverts:
            continue
        pts = [v[k * 7:k * 7 + 3] for k in range(nverts)]
        xs, ys = [q[0] for q in pts], [q[1] for q in pts]
        out.append((min(xs), min(ys), max(xs), max(ys), sum(q[2] for q in pts) / nverts))
    return out


def node_files(gta: Path | None, extra: list[Path]):
    """name -> bytes for every nodes<N>.dat: from the install's gta3.img, then any
    loose files given (a loose file replaces the archive's of the same name)."""
    import re
    out = {}
    if gta is not None:
        for img in open_imgs(gta):
            for name in img.names(".dat"):
                if re.fullmatch(r"nodes\d+\.dat", name):
                    out[name] = img.read(name)
            img.close()
    for p in extra:
        files = sorted(p.glob("*")) if p.is_dir() else [p]
        for f in files:
            if re.fullmatch(r"nodes\d+\.dat", f.name.lower()):
                out[f.name.lower()] = f.read_bytes()
    return out
