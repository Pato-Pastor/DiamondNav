"""How each vehicle model drives (vehicles.dnv), read from a GTA San Andreas install.

NPC drivers take their top speed, acceleration, braking, cornering grip and turning
circle from the game's own data/handling.cfg (linked to model ids by
data/vehicles.ide), so a Sultan pulls away, a truck takes its time and a bike turns
tight. Nothing of the game's is copied into DiamondNav: the file is made on your PC
from your install.

The .dnv layout is text, read by src/vehicles.cpp:
  DNVH 1
  MODEL TOP ACCEL BRAKE GRIP RADIUS     one line per model; # starts a comment
TOP in m/s, ACCEL / BRAKE / GRIP (sideways, in corners) in m/s^2, RADIUS (the
tightest turning circle) in metres.
"""

from __future__ import annotations

import math
from pathlib import Path

from . import gta

VEHICLES_MAGIC = "DNVH"
VEHICLES_VERSION = 1

# handling.cfg's car lines (SA): name, then these columns.
MASS, TRACTION, MAX_VELOCITY, ENGINE_ACCEL, BRAKE_DECEL, STEERING_LOCK = 1, 8, 12, 13, 17, 20
# Not cars: boats (%), bikes (!), planes ($), anims (^) add lines of their own.
SECTION_MARKS = "%!$^"
DRIVEN = {"car", "bike", "bmx", "mtruck", "quad"}

# handling.cfg's units are the game's own; these turn them into metres and seconds.
# They are estimates of how the cars feel in game, not taken from anywhere official:
# tune a model with Nav_SetVehicleHandling, or these, if a car feels wrong.
TOP_SCALE = 0.9 / 3.6           # maxVelocity is about km/h; the game tops out a bit below it
ACCEL_SLOPE, ACCEL_BASE = 0.3, -2.6     # engineAcceleration 14 (a truck) -> 1.6 m/s^2, 30 (sports) -> 6.4
GRIP_SCALE = 10.0               # tractionMultiplier 1.0 -> 10 m/s^2 sideways, for a mid-weight car
LIMITS = {"top": (8.0, 75.0), "accel": (1.2, 9.0), "brake": (2.0, 11.0), "grip": (3.0, 12.0), "radius": (3.0, 14.0)}


def wheelbase(kind: str, mass: float) -> float:
    """handling.cfg has no size: a wheelbase guessed from the kind and the weight."""
    if kind in ("bike", "bmx"):
        return 1.45
    if kind == "quad":
        return 1.3
    if mass < 1800:
        return 2.6
    if mass < 3500:
        return 3.0
    if mass < 8000:
        return 4.0
    return 5.5


def mass_grip(mass: float) -> float:
    """Heavier vehicles hold less in a corner than their tyres alone say."""
    if mass < 1500:
        return 1.1
    if mass < 2500:
        return 1.0
    if mass < 5000:
        return 0.85
    return 0.7


def clamp(v: float, key: str) -> float:
    lo, hi = LIMITS[key]
    return max(lo, min(hi, v))


def parse_handling(text: str) -> dict[str, list[float]]:
    """Car lines by name: columns 1..20 (drive and engine type, 15 and 16, are letters: 0)."""
    out = {}
    for raw in text.splitlines():
        line = raw.split(";")[0].strip()
        if not line or line[0] in SECTION_MARKS:
            continue
        t = line.split()
        if len(t) <= STEERING_LOCK:
            continue
        try:
            out[t[0].upper()] = [0.0 if i in (15, 16) else float(t[i]) for i in range(1, STEERING_LOCK + 1)]
        except ValueError:
            continue
    return out


def parse_ide(text: str) -> list[tuple[int, str, str]]:
    """(model id, kind, handling name) for every vehicle in vehicles.ide."""
    out, inside = [], False
    for raw in text.splitlines():
        line = raw.split("#")[0].strip()
        if not line:
            continue
        low = line.lower()
        if low == "cars":
            inside = True
            continue
        if low == "end":
            inside = False
            continue
        if not inside:
            continue
        t = [v.strip() for v in line.split(",")]
        if len(t) < 5:
            continue
        try:
            out.append((int(t[0]), t[3].lower(), t[4].upper()))
        except ValueError:
            continue
    return out


def models(handling_text: str, ide_text: str) -> list[tuple[int, str, dict]]:
    h = parse_handling(handling_text)
    rows = []
    for model, kind, name in parse_ide(ide_text):
        if kind not in DRIVEN or name not in h:
            continue
        c = h[name]                         # c[i - 1] is column i
        mass = c[MASS - 1]
        lock = max(10.0, min(50.0, c[STEERING_LOCK - 1]))
        rows.append((model, name, {
            "top": clamp(c[MAX_VELOCITY - 1] * TOP_SCALE, "top"),
            "accel": clamp(c[ENGINE_ACCEL - 1] * ACCEL_SLOPE + ACCEL_BASE, "accel"),
            "brake": clamp(c[BRAKE_DECEL - 1], "brake"),
            "grip": clamp(c[TRACTION - 1] * GRIP_SCALE * mass_grip(mass), "grip"),
            "radius": clamp(wheelbase(kind, mass) / math.sin(math.radians(lock)), "radius"),
        }))
    return rows


def write_dnv(rows, path: Path) -> int:
    lines = [f"{VEHICLES_MAGIC} {VEHICLES_VERSION}",
             "# model top accel brake grip radius   (m/s, m/s^2, m/s^2, m/s^2, m) - tools/dnav.py vehicles"]
    for model, name, v in sorted(rows):
        lines.append(f"{model} {v['top']:.2f} {v['accel']:.2f} {v['brake']:.2f} {v['grip']:.2f} {v['radius']:.2f}"
                     f"  # {name.lower()}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return len(rows)


def build(out: Path, gta_dir: Path) -> int:
    handling = gta.find(gta_dir, "data/handling.cfg")
    ide = gta.find(gta_dir, "data/vehicles.ide")
    if handling is None or ide is None:
        raise SystemExit(f"{gta_dir}: no data/handling.cfg or data/vehicles.ide (is this a GTA San Andreas folder?)")
    rows = models(handling.read_text(encoding="latin-1"), ide.read_text(encoding="latin-1"))
    if not rows:
        raise SystemExit(f"{gta_dir}: no vehicles found in data/vehicles.ide and data/handling.cfg")
    return write_dnv(rows, out)
