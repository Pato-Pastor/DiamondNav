"""The world as navbake sees it: every colliding triangle, stock map plus custom maps.

Where the stock map comes from, either:
  a GTA San Andreas install   data/**/*.ide and data/maps/**/*.ipl, models/gta3.img
                              and gta_int.img (binary IPLs, COL archives, path nodes),
                              SAMP/samp.img when there (SA-MP's own objects),
                              data/water.dat
  ColAndreas's database       colandreas.cadb: the stock map and SA-MP's objects
                              with their collision, already put together
(given both, the database places the objects and the install adds the path nodes
and water.)

Custom maps on top: the objects your scripts create and the stock buildings they
remove (sampmap.py), with custom models' collision taken from their DFFs (or a
.col of the same name) in the models folders you give.

Writes OUT.tris (navbake's geometry), OUT.car.tris (the same without the objects a
car breaks or knocks over in the game, from data/object.dat: fences, railings, lamp
posts, bins; for the car navmesh and a server's world.col), and when the install's
path nodes are there,
OUT.seeds.txt (points known to be walked or driven: the bake keeps only what is
connected to them, which drops roofs and closed-off spaces) and OUT.water.txt.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import TRIS_MAGIC
from . import col, gta, sampmap

WORLD = 3000.0                      # San Andreas is -3000 .. 3000 on x and y


def quat_matrix(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]], dtype=np.float32)


def stored_quat(rx, ry, rz):
    """An object's rotation (degrees, as CreateObject takes it) as GTA stores it:
    the same conversion as open.mp's GTAQuat. Stored rotations are conjugated, as
    in IPL files."""
    cx, cy, cz = (math.cos(math.radians(a) * -0.5) for a in (rx, ry, rz))
    sx, sy, sz = (math.sin(math.radians(a) * -0.5) for a in (rx, ry, rz))
    return (cx * sy * sz + sx * cy * cz,            # x
            cx * sy * cz - sx * cy * sz,            # y
            cx * cy * sz - sx * sy * cz,            # z
            cx * cy * cz + sx * sy * sz)            # w


def object_matrix(rx, ry, rz):
    qx, qy, qz, qw = stored_quat(rx, ry, rz)
    return quat_matrix(-qx, -qy, -qz, qw)


def removed(model, x, y, z, removals) -> bool:
    for r in removals:
        if (r.model == -1 or r.model == model) and (x - r.x) ** 2 + (y - r.y) ** 2 + (z - r.z) ** 2 <= r.radius ** 2:
            return True
    return False


@dataclass
class Report:
    stock: int = 0
    removed: int = 0
    lods: int = 0
    no_collision: int = 0
    breakable: int = 0
    breakable_models: dict[str, int] = field(default_factory=dict)
    custom: int = 0
    custom_no_collision: list[int] = field(default_factory=list)
    skipped_calls: int = 0
    triangles: int = 0
    seeds: int = 0
    water: int = 0

    def lines(self):
        out = [f"stock objects placed: {self.stock} ({self.removed} removed by your maps, {self.lods} LODs skipped, "
               f"{self.no_collision} without collision)",
               f"breakable stock objects (left out of the car files): {self.breakable}"
               + (" (most placed: " + ", ".join(f"{n} {c}" for n, c in sorted(self.breakable_models.items(),
                                                                               key=lambda kv: -kv[1])[:8]) + ")"
                  if self.breakable_models else ""),
               f"custom objects placed: {self.custom}"]
        if self.custom_no_collision:
            ids = sorted(set(self.custom_no_collision))
            out.append(f"custom objects without collision: {len(self.custom_no_collision)} (models "
                       + ", ".join(map(str, ids[:12])) + (", ..." if len(ids) > 12 else "") + ")")
        if self.skipped_calls:
            out.append(f"map calls skipped (variables or expressions, not numbers): {self.skipped_calls}")
        out.append(f"triangles: {self.triangles}, seeds: {self.seeds}, water areas: {self.water}")
        return out


class Models:
    """Model id -> collision, from wherever the world comes from."""

    def __init__(self, gta_dir: Path | None, cadb: col.Cadb | None, model_dirs: list[Path],
                 simple: dict[int, sampmap.SimpleModel]):
        self.cadb = cadb
        self.simple = simple
        self.model_dirs = model_dirs
        self.names: dict[int, str] = {}
        self.breakables: set[str] = set()
        self.cols: dict[str, np.ndarray] = {}
        self.imgs = []
        if gta_dir is not None:
            self.imgs = gta.open_imgs(gta_dir)
            self.names = gta.read_ides(gta_dir)
            self.breakables = gta.read_breakables(gta_dir)
            self.cols = col.read_cols(self.imgs)
        # Loose .col files in the models folders, by entry name.
        for d in model_dirs:
            for p in sorted(d.rglob("*")):
                if p.suffix.lower() == ".col":
                    for name, tris in col.parse_col_file(p.read_bytes()).items():
                        self.cols[name] = tris
        self.cache: dict[int, np.ndarray | None] = {}

    def _file(self, rel: str) -> Path | None:
        for d in self.model_dirs:
            hit = gta.find(d, rel.replace("\\", "/"))
            if hit is not None and hit.is_file():
                return hit
        return None

    def is_lod(self, model: int) -> bool:
        name = self.names.get(model)
        return name is not None and name.startswith("lod")

    def is_breakable(self, model: int) -> bool:
        name = self.names.get(model)
        return name is not None and name in self.breakables

    def collision(self, model: int):
        if model in self.cache:
            return self.cache[model]
        tris = None
        sm = self.simple.get(model)
        if sm is not None:
            dff = self._file(sm.dff)
            if dff is not None:
                tris = col.dff_collision(dff.read_bytes())
            if tris is None:
                stem = Path(sm.dff.replace("\\", "/")).stem.lower()
                tris = self.cols.get(stem)
        elif self.cadb is not None and model in self.cadb.models:
            tris = self.cadb.models[model]
        else:
            name = self.names.get(model)
            if name is not None:
                tris = self.cols.get(name)
        if tris is not None and not len(tris):
            tris = None
        self.cache[model] = tris
        return tris


def write_tris(path: Path, chunks) -> int:
    tris = np.concatenate(chunks).astype("<f4") if chunks else np.zeros((0, 3, 3), "<f4")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as fh:
        fh.write(TRIS_MAGIC + struct.pack("<I", len(tris)))
        fh.write(tris.tobytes())
    return len(tris)


def read_nodes(files: dict[str, bytes]):
    """Every vehicle and ped path node (x, y, z): ground the game walks or drives."""
    out = []
    for d in files.values():
        n = struct.unpack_from("<I", d, 0)[0]
        arr = np.frombuffer(d, dtype=np.dtype([("pad", "<u4", 2), ("p", "<i2", 3), ("rest", "u1", 14)]),
                            count=n, offset=20)
        out.append(arr["p"].astype(np.float32) / 8.0)
    return np.concatenate(out) if out else np.zeros((0, 3), np.float32)


def build(out: Path, gta_dir: Path | None = None, cadb_path: Path | None = None, maps: list[Path] = (),
          model_dirs: list[Path] = (), area=None, water_dat: Path | None = None) -> Report:
    if gta_dir is None and cadb_path is None:
        raise SystemExit("give a GTA San Andreas folder (--gta) or ColAndreas's database (--colandreas)")
    rep = Report()
    mapdata = sampmap.read_maps(list(maps))
    # artconfig.txt in a models folder declares custom models too.
    for d in model_dirs:
        art = gta.find(d, "artconfig.txt")
        if art is not None:
            mapdata.extend(sampmap.read_map(art))
    rep.skipped_calls = mapdata.skipped
    cadb = col.read_cadb(cadb_path) if cadb_path is not None else None
    models = Models(gta_dir, cadb, list(model_dirs), mapdata.models)

    def inside(x, y):
        if not (-WORLD <= x < WORLD and -WORLD <= y < WORLD):
            return False
        return area is None or (area[0] <= x <= area[2] and area[1] <= y <= area[3])

    chunks, car_chunks = [], []      # everything; what a car can't drive through
    if cadb is not None:
        stock = [(m, x, y, z, quat_matrix(qx, qy, qz, qw)) for m, x, y, z, qx, qy, qz, qw in cadb.placements]
    else:
        stock = [(m, x, y, z, quat_matrix(-qx, -qy, -qz, qw))
                 for m, x, y, z, qx, qy, qz, qw in gta.read_ipls(gta_dir, models.imgs)]
    for mdl, x, y, z, rot in stock:
        if not inside(x, y):
            continue
        if models.is_lod(mdl):
            rep.lods += 1
            continue
        if removed(mdl, x, y, z, mapdata.removals):
            rep.removed += 1
            continue
        tris = models.collision(mdl)
        if tris is None:
            rep.no_collision += 1
            continue
        placed = tris @ rot.T + np.array([x, y, z], np.float32)
        chunks.append(placed)
        if models.is_breakable(mdl):
            rep.breakable += 1
            name = models.names[mdl]
            rep.breakable_models[name] = rep.breakable_models.get(name, 0) + 1
        else:
            car_chunks.append(placed)
        rep.stock += 1

    for o in mapdata.objects:
        if not inside(o.x, o.y):
            continue
        tris = models.collision(o.model)
        if tris is None:
            rep.custom_no_collision.append(o.model)
            continue
        # A script's objects never break or move in SA-MP, whatever the model.
        placed = tris @ object_matrix(o.rx, o.ry, o.rz).T + np.array([o.x, o.y, o.z], np.float32)
        chunks.append(placed)
        car_chunks.append(placed)
        rep.custom += 1

    rep.triangles = write_tris(out.with_suffix(".tris"), chunks)
    write_tris(out.with_suffix(".car.tris"), car_chunks)
    for img in models.imgs:
        img.close()

    seeds_path, water_path = out.with_suffix(".seeds.txt"), out.with_suffix(".water.txt")
    if gta_dir is not None:
        nodes = read_nodes(gta.node_files(gta_dir, []))
        seeds = [f"{px:.2f} {py:.2f} {pz:.2f}" for px, py, pz in nodes if inside(px, py)]
        # Custom maps are where players go: keep what is connected to them too.
        seeds += [f"{o.x:.2f} {o.y:.2f} {o.z:.2f}" for o in mapdata.objects if inside(o.x, o.y)]
        rep.seeds = len(seeds)
        seeds_path.write_text("\n".join(seeds) + "\n", encoding="utf-8")
    elif seeds_path.exists():
        seeds_path.unlink()

    water = []
    if water_dat is not None:
        water = gta.parse_water(water_dat.read_text(encoding="latin-1", errors="ignore"))
    elif gta_dir is not None:
        water = gta.read_water(gta_dir)
    water = [w for w in water if area is None or not (w[2] < area[0] or w[0] > area[2] or w[3] < area[1] or w[1] > area[3])]
    rep.water = len(water)
    if water:
        water_path.write_text("".join("%.2f %.2f %.2f %.2f %.2f\n" % w for w in water), encoding="utf-8")
    elif water_path.exists():
        water_path.unlink()
    return rep
