"""How each vehicle model drives (vehicles.dnv), worked out from a GTA San Andreas install.

NPC drivers take how hard they pull away in each gear, their top speed, braking,
cornering grip on and off the road, turning circle and how fast they can start to
turn from the game's own data, so a Sultan launches hard, a Linerunner takes its
time, a bike leans and a Sandking keeps its grip on dirt:

  data/handling.cfg   every column: mass, turn mass, drag, centre of mass,
                      traction (multiplier, loss, front/rear bias), gears, top
                      speed, engine acceleration and inertia, drive (F/R/4),
                      brakes (deceleration, bias), steering lock, suspension
                      (front/rear bias), the handling flags (gear boosts, the
                      speed limit, off-road ability, rear-wheel steering); and
                      the bike lines (lean)
  data/vehicles.ide   which handling each model uses, its kind and wheel size
  data/surface.dat    how well tyres grip each kind of ground (road, dirt, sand)
  models/gta3.img     each model's DFF: where its wheels are (wheelbase, track)
                      and how big it is (its collision bounds)

The game's own vehicle physics are run here, once per model, on your PC: the
gearbox, engine inertia, air drag and tyre grip (with wheelspin) are stepped at the
game's 50 steps a second from a standstill to top speed, on a road and on dirt. What
comes out is a table the component reads; it does no physics of its own. Nothing of
the game's is copied into DiamondNav: the file is made from your install.

The .dnv layout is text, read by src/driving.cpp:
  DNVH 2
  MODEL TOP BRAKE BRAKE_OFF COAST DRAG GRIP GRIP_OFF RADIUS YAW REVERSE
        LENGTH WIDTH HEIGHT WHEELBASE  A0 .. A15  O0 .. O15
TOP and REVERSE in m/s; BRAKE, BRAKE_OFF (on dirt), COAST (rolling with no throttle),
GRIP, GRIP_OFF (sideways, in corners) and A/O (full-throttle acceleration on a road /
on dirt at TOP * i / 15) in m/s^2; DRAG the air's deceleration per (m/s)^2; RADIUS
the tightest circle the car's centre turns, LENGTH WIDTH HEIGHT WHEELBASE in metres;
YAW how fast it can start to turn, in rad/s^2. # starts a comment.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from pathlib import Path

from . import gta

VEHICLES_MAGIC = "DNVH"
VEHICLES_VERSION = 2
CURVE = 16                      # acceleration samples from 0 to top speed

# --- GTA SA's units ----------------------------------------------------------
# The game steps its physics 50 times a second; speeds are metres per step and
# accelerations metres per step per step.
STEP = 50.0                     # steps per second
TO_MS = STEP                    # m/step -> m/s
TO_MS2 = STEP * STEP            # m/step^2 -> m/s^2
GRAVITY = 0.008                 # the game's gravity, m/step^2 (20 m/s^2)
KMH = 1000.0 / 3600.0 / STEP    # handling.cfg's km/h -> m/step
ACCEL_FILE = 1.0 / 2500.0       # handling.cfg's accelerations -> m/step^2
ENGINE_SCALE = 0.4              # the game scales engine acceleration by this when it loads it...
DRIVE_SCALE = 0.4               # ...and again for every step of drive
TRACTION = 0.001                # a wheel's grip per traction multiplier, m/step, before load and surface
GEAR_CUT = 0.05                 # m/step past a gear's top speed where its drive is gone
SPIN_LOSS = 0.2                 # grip lost while a wheel spins at full throttle
WHEEL_FRICTION = 0.9            # what slows a rolling wheel with no throttle (/ mass)
SMOOTH = 0.85                   # the engine's inertia smoothing

# handling.cfg columns (the name is column 0)
(MASS, TURN_MASS, DRAG, COM_X, COM_Y, COM_Z, SUBMERGED, TRACTION_MULT, TRACTION_LOSS, TRACTION_BIAS,
 GEARS, MAX_VELOCITY, ENGINE_ACCEL, ENGINE_INERTIA, DRIVE_TYPE, ENGINE_TYPE, BRAKE_DECEL, BRAKE_BIAS, ABS,
 STEERING_LOCK, SUSP_FORCE, SUSP_DAMPING, SUSP_HIGH_SPEED, SUSP_UPPER, SUSP_LOWER, SUSP_BIAS, ANTI_DIVE,
 SEAT_OFFSET, DAMAGE_MULT, VALUE, MODEL_FLAGS, HANDLING_FLAGS) = range(1, 33)
LETTERS = (DRIVE_TYPE, ENGINE_TYPE)
HEX = (MODEL_FLAGS, HANDLING_FLAGS)

# handling flags
H_1G_BOOST, H_2G_BOOST = 0x1, 0x2
H_STEER_REARWHEELS = 0x20
H_OFFROAD_ABILITY, H_OFFROAD_ABILITY2 = 0x100000, 0x200000
H_USE_MAXSP_LIMIT = 0x1000000
# model flags
M_IS_BIKE = 0x1000000

# Not cars: boats (%), planes ($) and anim groups (^) add lines of their own. Bikes
# (!) add their lean.
DRIVEN = {"car", "bike", "bmx", "mtruck", "quad"}
TWO_WHEELS = {"bike", "bmx"}

# data/surface.dat, used when the install has none: how well each adhesion group
# grips each other one (the wheels are RUBBER).
GROUPS = ("RUBBER", "HARD", "ROAD", "LOOSE", "SAND", "WET")
SURFACE_DEFAULT = {
    ("RUBBER", "RUBBER"): 6.0,
    ("HARD", "RUBBER"): 3.6, ("HARD", "HARD"): 2.0,
    ("ROAD", "RUBBER"): 4.5, ("ROAD", "HARD"): 3.0, ("ROAD", "ROAD"): 6.0,
    ("LOOSE", "RUBBER"): 3.2, ("LOOSE", "HARD"): 3.5, ("LOOSE", "ROAD"): 2.0, ("LOOSE", "LOOSE"): 1.0,
    ("SAND", "RUBBER"): 3.0, ("SAND", "HARD"): 4.0, ("SAND", "ROAD"): 2.0, ("SAND", "LOOSE"): 1.0,
    ("SAND", "SAND"): 1.0,
    ("WET", "RUBBER"): 2.8, ("WET", "HARD"): 2.0, ("WET", "ROAD"): 1.0, ("WET", "LOOSE"): 1.0,
    ("WET", "SAND"): 1.0, ("WET", "WET"): 0.5,
}

# Bounds the results are kept inside, so a broken line can't make a car absurd.
LIMITS = {"top": (3.0, 90.0), "brake": (1.0, 40.0), "grip": (2.0, 40.0), "radius": (2.0, 30.0), "yaw": (0.3, 15.0)}


@dataclass
class Handling:
    name: str
    c: list                     # c[i] is column i (c[0] the name)
    bike: list | None = None    # the bike line's lean columns, when there is one

    def __getitem__(self, i):
        return self.c[i]

    @property
    def hflags(self) -> int:
        return int(self.c[HANDLING_FLAGS])

    @property
    def mflags(self) -> int:
        return int(self.c[MODEL_FLAGS])


@dataclass
class IdeVehicle:
    model: int
    dff: str
    kind: str
    handling: str
    wheel_front: float = 0.7    # wheel diameters, m
    wheel_rear: float = 0.7


@dataclass
class Body:
    """What a vehicle's DFF says about its size."""
    wheels: dict = field(default_factory=dict)      # frame name -> (x, y, z), model space
    lo: tuple | None = None                         # collision bounds
    hi: tuple | None = None


# --- reading the install -----------------------------------------------------

def parse_handling(text: str) -> dict[str, Handling]:
    """Every car line by name, with its bike line when there is one."""
    out, bikes = {}, {}
    for raw in text.splitlines():
        line = raw.split(";")[0].strip()
        if not line or line[0] in "%$^":
            continue
        if line[0] == "!":
            t = line[1:].split()
            try:
                bikes[t[0].upper()] = [float(x) for x in t[1:16]]
            except (ValueError, IndexError):
                pass
            continue
        t = line.split()
        if len(t) <= HANDLING_FLAGS:
            continue
        try:
            c = [t[0].upper()]
            for i in range(1, HANDLING_FLAGS + 1):
                if i in LETTERS:
                    c.append(t[i].upper())
                elif i in HEX:
                    c.append(int(t[i], 16))
                else:
                    c.append(float(t[i]))
        except ValueError:
            continue
        out[c[0]] = Handling(c[0], c)
    for name, b in bikes.items():
        if name in out and len(b) == 15:
            out[name].bike = b
    return out


def parse_ide(text: str) -> list[IdeVehicle]:
    """Every vehicle in vehicles.ide: model id, DFF, kind, handling and wheel size."""
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
            v = IdeVehicle(int(t[0]), t[1].lower(), t[3].lower(), t[4].upper())
        except ValueError:
            continue
        # ..., then the wheel model (a bike's steering angle), front and rear wheel size.
        try:
            v.wheel_front, v.wheel_rear = float(t[12]), float(t[13])
        except (ValueError, IndexError):
            pass
        out.append(v)
    return out


def parse_surface(text: str) -> dict[tuple[str, str], float]:
    """data/surface.dat: rows of adhesive limits, a lower triangle by adhesion group."""
    table = {}
    for raw in text.splitlines():
        line = raw.split(";")[0].strip()
        if not line:
            continue
        t = line.split()
        row = t[0].upper()
        if row not in GROUPS:
            continue
        for col, v in zip(GROUPS, t[1:]):
            try:
                table[(row, col)] = float(v)
            except ValueError:
                pass
    return table


def adhesive(table: dict, a: str, b: str) -> float:
    """How well group a grips group b (the table is a triangle: either order)."""
    for key in ((a, b), (b, a)):
        if key in table:
            return table[key]
    return SURFACE_DEFAULT.get((a, b), SURFACE_DEFAULT.get((b, a), 1.0))


# RenderWare sections in a DFF
RW_STRUCT, RW_EXTENSION, RW_FRAME_LIST, RW_CLUMP = 0x01, 0x03, 0x0E, 0x10
RW_FRAME_NAME = 0x253F2FE
RW_COLLISION = 0x253F2FA


def parse_dff(d: bytes) -> Body:
    """A vehicle DFF's wheel positions (its frames named wheel_*) and collision bounds."""
    body = Body()

    def chunks(start, end):
        at = start
        while at + 12 <= end:
            kind, size = struct.unpack_from("<II", d, at)
            yield kind, at + 12, min(at + 12 + size, end)
            at += 12 + size

    def frames(start, end):
        parts = list(chunks(start, end))
        if not parts or parts[0][0] != RW_STRUCT:
            return
        _, s, _ = parts[0]
        n = struct.unpack_from("<I", d, s)[0]
        mats = []
        for i in range(n):
            f = struct.unpack_from("<12fiI", d, s + 4 + i * 56)
            mats.append((f[0:9], f[9:12], f[12]))
        names = []
        for kind, s2, e2 in parts[1:]:
            if kind != RW_EXTENSION:
                continue
            name = ""
            for k3, s3, e3 in chunks(s2, e2):
                if k3 == RW_FRAME_NAME:
                    name = d[s3:e3].split(b"\0")[0].decode("latin-1").lower()
            names.append(name)
        world = {}

        def place(i):
            if i in world:
                return world[i]
            rot, pos, parent = mats[i]
            if 0 <= parent < len(mats) and parent != i:
                prot, ppos = place(parent)
                # rows are the right, up and at axes: world = pos.x*right + pos.y*up + pos.z*at + parent's
                p = tuple(ppos[k] + pos[0] * prot[k] + pos[1] * prot[3 + k] + pos[2] * prot[6 + k] for k in range(3))
                r = tuple(sum(rot[3 * a + j] * prot[3 * j + k] for j in range(3)) for a in range(3) for k in range(3))
            else:
                p, r = tuple(pos), tuple(rot)
            world[i] = (r, p)
            return world[i]

        for i, name in enumerate(names[:len(mats)]):
            if name.startswith("wheel_"):
                body.wheels[name] = place(i)[1]

    def bounds(start, end):
        four = d[start:start + 4]
        if end - start < 32 + 40:
            return
        p = start + 32
        if four == b"COLL":
            _r, _cx, _cy, _cz, *mm = struct.unpack_from("<10f", d, p)
            body.lo, body.hi = tuple(mm[0:3]), tuple(mm[3:6])
        elif four in (b"COL2", b"COL3", b"COL4"):
            mm = struct.unpack_from("<6f", d, p)
            body.lo, body.hi = tuple(mm[0:3]), tuple(mm[3:6])

    def walk(start, end):
        for kind, s, e in chunks(start, end):
            if kind == RW_FRAME_LIST:
                frames(s, e)
            elif kind == RW_COLLISION:
                bounds(s, e)
            elif kind in (RW_CLUMP, RW_EXTENSION):
                walk(s, e)

    walk(0, len(d))
    return body


# --- the game's physics ------------------------------------------------------

class Gearbox:
    """GTA SA's gearbox: each gear's top speed, and where it changes up and down."""

    def __init__(self, gears: int, max_v: float, reverse_v: float, flags: int, engine: float):
        self.n = n = max(1, int(gears))
        self.max_v, self.flags, self.engine = max_v, flags, engine
        self.top = [0.0] * (n + 1)
        self.up = [0.0] * (n + 1)
        self.down = [0.0] * (n + 1)
        half = 0.5 * max_v / n
        span = max_v - half
        for i in range(1, n + 1):
            self.top[i] = i * span / n + half
            step = self.top[i] - self.top[i - 1]
            if i >= n:
                self.up[i] = max_v
            else:
                self.down[i + 1] = 0.42 * step + self.top[i - 1]
                self.up[i] = 0.6667 * step + self.top[i - 1]
        self.top[0] = reverse_v
        self.up[0] = -0.01
        self.down[0] = reverse_v
        self.down[1] = -0.01

    def drive(self, v: float, state: dict) -> float:
        """Full throttle at forward speed v (m/step): one wheel's drive this step (m/step).
        state carries the gear and the engine's inertia between steps."""
        if v > self.max_v:
            return 0.0
        gear = state["gear"]
        while True:
            if v > self.up[gear] and gear < self.n:
                gear += 1
                continue
            if v >= self.down[gear] or gear <= 1:
                break
            gear -= 1
        state["gear"] = gear
        if self.n == 1:
            mult = 1.0
        else:
            g = 1.0 - (gear - 1) / (self.n - 1)
            boost = 5.0 if self.flags & H_1G_BOOST else 4.0 if self.flags & H_2G_BOOST else 3.0
            mult = 1.0 + boost * g * g
        a = mult * self.engine * DRIVE_SCALE
        # The engine's inertia: how far through the gear the car is, and how fast
        # that changes, hold the drive back, smoothed over a few steps.
        third = self.max_v / self.n / 3.0
        if gear == 1:
            ratio = (third + v) / (third + self.up[1])
        else:
            ratio = (v - self.down[gear]) / max(1e-6, self.up[gear] - self.down[gear])
        hold = min(1.0, max(0.1, 1.0 - (ratio - state["ratio"]) * state["inertia"]))
        state["ratio"] = ratio
        state["smooth"] = hold * (1.0 - SMOOTH) + SMOOTH * state["smooth"]
        a *= state["smooth"]
        if v > self.top[gear]:
            a *= 1.0 - min((v - self.top[gear]) / GEAR_CUT, 1.0)
        return a


@dataclass
class Model:
    """One vehicle model worked out: everything vehicles.dnv carries, in m and s."""
    top: float
    brake: float
    brake_off: float
    coast: float
    drag: float
    grip: float
    grip_off: float
    radius: float
    yaw: float
    reverse: float
    length: float
    width: float
    height: float
    wheelbase: float
    accel: list
    accel_off: list
    zero100: float              # seconds from 0 to 100 km/h on a road (0: never), for the summary


def air_resistance(drag: float) -> float:
    """The game's air resistance factor (per m/step of speed, per step)."""
    return drag / 2000.0 if drag > 0.01 else drag


def flat_speed(h: Handling) -> tuple[float, float, float]:
    """(gearbox top speed, top speed on the flat, reverse) in m/step, as the game sets
    them from the line: the drag that matches a sixth of the engine caps the file's speed."""
    engine = h[ENGINE_ACCEL] * ENGINE_SCALE * ACCEL_FILE
    limit = engine / 6.0
    v = h[MAX_VELOCITY] * KMH
    drag = h[DRAG]
    while v > 0.0:
        v -= 0.01
        if drag >= 0.01:
            if v * v * (drag / 2.0) / 1000.0 <= limit:
                break
        elif (1.0 / (v * v * drag + 1.0) - 1.0) * -v <= limit:
            break
    v = max(v, 0.01)
    two = bool(h.mflags & M_IS_BIKE)
    if h.hflags & H_USE_MAXSP_LIMIT:
        flat = v / 1.2
        return h[MAX_VELOCITY] * KMH, flat, min(-flat / 4.0, -0.2)
    reverse = -0.05 if two else min(-v * 0.3, -0.2)
    return v * 1.2, v, reverse


class Wheels:
    """A vehicle's wheels as the game grips with them: n wheels, half on each axle,
    each holding a share of the load, the front and rear biased by tractionBias."""

    def __init__(self, h: Handling, two: bool, ground: float, offroad: float):
        self.n = 2 if two else 4
        self.per_axle = self.n // 2
        bias = h[TRACTION_BIAS]
        # A wheel's grip: surface x traction x its share of the load (springs at rest
        # carry the car: each of n wheels holds 2/n of the load factor) x the axle's bias.
        base = ground * offroad * h[TRACTION_MULT] * TRACTION * (2.0 / self.n)
        self.front = base * 2.0 * bias
        self.rear = base * (2.0 - 2.0 * bias)
        self.loss = h[TRACTION_LOSS]
        drive = h[DRIVE_TYPE]
        self.driven = []                    # (grip, wheels) per driven axle
        if drive in ("4", "F"):
            self.driven.append((self.front, self.per_axle))
        if drive in ("4", "R"):
            self.driven.append((self.rear, self.per_axle))
        if not self.driven:
            self.driven.append((self.rear, self.per_axle))
        self.share = 4.0 if drive == "4" else 2.0       # the engine's drive is shared this way

    def cornering(self) -> float:
        """The most sideways acceleration both axles hold together (m/step^2). Grip
        follows each wheel's load, so where the weight sits does not matter; the
        weaker axle (tractionBias) slides first."""
        return 2.0 * self.per_axle * min(self.front, self.rear)


def simulate(h: Handling, two: bool, ground: float, offroad: float, max_v: float, reverse_v: float):
    """Full throttle from a standstill until the speed settles: (top m/s, [(v m/s, a m/s^2)])."""
    wheels = Wheels(h, two, ground, offroad)
    engine = h[ENGINE_ACCEL] * ENGINE_SCALE * ACCEL_FILE
    box = Gearbox(h[GEARS], max_v, reverse_v, h.hflags, engine / wheels.share)
    air = air_resistance(h[DRAG])
    # A car standing with its engine running has its inertia settled (the game keeps
    # updating it with no throttle), so it pulls away at once.
    third = max_v / box.n / 3.0
    state = {"gear": 1, "ratio": third / (third + box.up[1]), "smooth": 1.0, "inertia": h[ENGINE_INERTIA]}
    spinning = [False] * len(wheels.driven)
    v, best, still, trace = 0.0, 0.0, 0, []
    for _step in range(int(STEP * 240)):
        thrust = box.drive(v, state)
        dv = 0.0
        for k, (grip, count) in enumerate(wheels.driven):
            hold = grip
            if spinning[k]:
                hold *= wheels.loss * (1.0 - SPIN_LOSS)      # a spinning tyre grips less
            if thrust > hold:
                force = hold if spinning[k] else hold * wheels.loss * (1.0 - SPIN_LOSS)
                spinning[k] = True
            else:
                force = thrust
                spinning[k] = False
            dv += force * count
        before = v
        v += dv
        v *= 1.0 - v * air
        trace.append((before * TO_MS, (v - before) * TO_MS2))
        if v > best + 1e-5:
            best, still = v, 0
        else:
            still += 1
            if still > STEP * 3:
                break
    return best * TO_MS, trace


def curve_from(trace, top: float) -> list[float]:
    """The trace's acceleration at CURVE speeds from 0 to top (m/s^2), never negative."""
    out = []
    j = 0
    for i in range(CURVE):
        want = top * i / (CURVE - 1)
        while j + 1 < len(trace) and trace[j + 1][0] <= want:
            j += 1
        if i == CURVE - 1:
            out.append(0.0)
            continue
        # The steps around this speed, averaged so a gear change doesn't decide it.
        lo, hi = max(0, j - 2), min(len(trace), j + 3)
        a = sum(t[1] for t in trace[lo:hi]) / max(1, hi - lo)
        out.append(max(0.0, a))
    if out[0] <= 0.0 and len(out) > 1:
        out[0] = out[1]
    return out


def time_to(trace, speed: float) -> float:
    for i, (v, _a) in enumerate(trace):
        if v >= speed:
            return i / STEP
    return 0.0


def braking(h: Handling, two: bool, ground: float, offroad: float) -> float:
    """Full brakes (m/s^2): each wheel brakes by the brake bias, and one that brakes
    harder than it grips locks and slides at its grip x tractionLoss."""
    w = Wheels(h, two, ground, offroad)
    b = h[BRAKE_DECEL] * ACCEL_FILE
    total = 0.0
    for grip, bias in ((w.front, 2.0 * h[BRAKE_BIAS]), (w.rear, 2.0 - 2.0 * h[BRAKE_BIAS])):
        each = b * bias
        total += (each if each <= grip else grip * w.loss) * w.per_axle
    return total * TO_MS2


def coasting(h: Handling, two: bool, ground: float) -> float:
    """Rolling with no throttle and no brake (m/s^2, before air drag)."""
    w = Wheels(h, two, ground, 1.0)
    mass = max(h[MASS], 1.0)
    each = WHEEL_FRICTION * 0.6 / (mass + 200.0) if two else WHEEL_FRICTION / mass
    if each > 500.0:
        each *= 0.1
    return (min(each, w.front) + min(each, w.rear)) * w.per_axle * TO_MS2


def work_out(h: Handling, v: IdeVehicle, body: Body, surface: dict) -> Model:
    two = v.kind in TWO_WHEELS or bool(h.mflags & M_IS_BIKE)
    road = adhesive(surface, "ROAD", "RUBBER")
    dirt = adhesive(surface, "LOOSE", "RUBBER")
    offroad = 1.4 if h.hflags & H_OFFROAD_ABILITY2 else 1.15 if h.hflags & H_OFFROAD_ABILITY else 1.0
    max_v, _flat, reverse_v = flat_speed(h)
    top, trace = simulate(h, two, road, 1.0, max_v, reverse_v)
    top_off, trace_off = simulate(h, two, dirt, offroad, max_v, reverse_v)
    accel = curve_from(trace, top)
    # On dirt the same speeds as on the road, so both tables share one axis.
    speeds = [top * i / (CURVE - 1) for i in range(CURVE)]
    accel_off = [min(a, b) if s < top_off else 0.0 for a, b, s in zip(curve_from(trace_off, top), accel, speeds)]

    # Size: the wheels from the DFF, else a guess from the kind and weight.
    wheels = body.wheels
    front = [p for n, p in wheels.items() if n in ("wheel_rf_dummy", "wheel_lf_dummy", "wheel_front")]
    rear = [p for n, p in wheels.items() if n in ("wheel_rb_dummy", "wheel_lb_dummy", "wheel_rear")]
    if front and rear:
        yf = sum(p[1] for p in front) / len(front)
        yr = sum(p[1] for p in rear) / len(rear)
        wheelbase = max(0.5, yf - yr)
        track = max((abs(p[0]) * 2.0 for p in front + rear), default=1.6) if not two else 0.0
        ground_z = min(p[2] for p in front + rear) - max(v.wheel_front, v.wheel_rear) * 0.5
    else:
        wheelbase = guess_wheelbase(v.kind, h[MASS])
        yr = -wheelbase * 0.5
        yf = yr + wheelbase
        track = 0.0 if two else 1.6
        ground_z = None
    if body.lo and body.hi:
        length, width, height = (body.hi[k] - body.lo[k] for k in (1, 0, 2))
        if ground_z is None:
            ground_z = body.lo[2]
    else:
        length, width, height = wheelbase * 1.65, (0.8 if two else 1.9), (1.2 if two else 1.5)
        if ground_z is None:
            ground_z = h[COM_Z] - 0.6

    # Cornering: tyres (the weaker axle), the lean a bike may take, the roll a tall
    # narrow vehicle can take before its inner wheels lift.
    grip = Wheels(h, two, road, 1.0).cornering() * TO_MS2
    grip_off = Wheels(h, two, dirt, offroad).cornering() * TO_MS2
    g = GRAVITY * TO_MS2
    if two and h.bike:
        lean = math.tan(math.radians(min(max(h.bike[4], 5.0), 70.0)))
        grip, grip_off = min(grip, g * lean), min(grip_off, g * lean)
    rise = max(0.2, h[COM_Z] - ground_z)
    if track > 0.0:
        roll = g * (track * 0.5) / rise
        grip, grip_off = min(grip, roll), min(grip_off, roll)

    # Turning circle of the car's centre at full lock (both axles steer when the rear
    # wheels do, which halves it).
    lock = math.radians(min(max(h[STEERING_LOCK], 5.0), 60.0))
    if h.hflags & H_STEER_REARWHEELS:
        bend = wheelbase * 0.5 / math.tan(lock)
        back = (yf + yr) * 0.5
    else:
        bend = wheelbase / math.tan(lock)
        back = yr
    radius = math.hypot(bend, back)

    # Starting to turn: what the front tyres push sideways, at their distance from
    # the centre of mass, against the turn mass (the game's moment of inertia).
    w = Wheels(h, two, road, 1.0)
    lever = max(0.3, yf - h[COM_Y])
    yaw = h[MASS] * w.front * w.per_axle * lever / max(h[TURN_MASS], 1.0) * TO_MS2

    brake, brake_off = braking(h, two, road, 1.0), braking(h, two, dirt, offroad)
    if two:
        # A bike pitches over its front wheel braking (a stoppie) and over its back
        # wheel pulling away (a wheelie) before its tyres give: neither goes past
        # what its wheelbase and the height of its weight allow.
        pitch = g * (wheelbase * 0.5) / rise
        brake, brake_off = min(brake, pitch), min(brake_off, pitch)
        accel = [min(a, pitch) for a in accel]
        accel_off = [min(a, pitch) for a in accel_off]

    return Model(
        top=clamp(top, "top"),
        brake=clamp(brake, "brake"),
        brake_off=clamp(brake_off, "brake"),
        coast=coasting(h, two, road),
        drag=air_resistance(h[DRAG]),
        grip=clamp(grip, "grip"),
        grip_off=clamp(grip_off, "grip"),
        radius=clamp(radius, "radius"),
        yaw=clamp(yaw, "yaw"),
        reverse=abs(reverse_v) * TO_MS,
        length=length, width=width, height=height, wheelbase=wheelbase,
        accel=accel, accel_off=accel_off,
        zero100=time_to(trace, 100.0 / 3.6),
    )


def guess_wheelbase(kind: str, mass: float) -> float:
    """No DFF to read: a wheelbase guessed from the kind and the weight."""
    if kind in TWO_WHEELS:
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


def clamp(v: float, key: str) -> float:
    lo, hi = LIMITS[key]
    return max(lo, min(hi, v))


# --- the file ----------------------------------------------------------------

def models(handling_text: str, ide_text: str, surface_text: str = "", dffs=None) -> list[tuple[int, str, Model]]:
    """Every driven model worked out. dffs(name) gives a model's DFF bytes, or None."""
    h = parse_handling(handling_text)
    surface = parse_surface(surface_text) if surface_text else {}
    rows = []
    for v in parse_ide(ide_text):
        if v.kind not in DRIVEN or v.handling not in h:
            continue
        data = dffs(v.dff) if dffs else None
        body = parse_dff(data) if data else Body()
        rows.append((v.model, v.handling, work_out(h[v.handling], v, body, surface)))
    return rows


def write_dnv(rows, path: Path) -> int:
    lines = [f"{VEHICLES_MAGIC} {VEHICLES_VERSION}",
             "# model top brake brake_off coast drag grip grip_off radius yaw reverse length width height wheelbase"
             f"  accel x{CURVE} (road)  accel x{CURVE} (dirt) - tools/dnav.py vehicles"]
    for model, name, m in sorted(rows, key=lambda r: r[0]):
        head = [m.top, m.brake, m.brake_off, m.coast, m.drag, m.grip, m.grip_off, m.radius, m.yaw, m.reverse,
                m.length, m.width, m.height, m.wheelbase]
        nums = " ".join(f"{x:.2f}" if abs(x) >= 0.01 or x == 0 else f"{x:.6f}" for x in head)
        curve = " ".join(f"{a:.2f}" for a in m.accel + m.accel_off)
        note = f"{name.lower()}, {m.top * 3.6:.0f} km/h" + (f", 0-100 in {m.zero100:.1f} s" if m.zero100 else "")
        lines.append(f"{model} {nums} {curve}  # {note}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return len(rows)


def build(out: Path, gta_dir: Path) -> int:
    handling = gta.find(gta_dir, "data/handling.cfg")
    ide = gta.find(gta_dir, "data/vehicles.ide")
    if handling is None or ide is None:
        raise SystemExit(f"{gta_dir}: no data/handling.cfg or data/vehicles.ide (is this a GTA San Andreas folder?)")
    surface = gta.find(gta_dir, "data/surface.dat")
    img_path = gta.find(gta_dir, "models/gta3.img")
    img = gta.Img(img_path) if img_path is not None else None

    def dffs(name):
        key = name + ".dff"
        return img.read(key) if img is not None and img.has(key) else None

    try:
        rows = models(handling.read_text(encoding="latin-1"), ide.read_text(encoding="latin-1"),
                      surface.read_text(encoding="latin-1") if surface is not None else "", dffs)
    finally:
        if img is not None:
            img.close()
    if not rows:
        raise SystemExit(f"{gta_dir}: no vehicles found in data/vehicles.ide and data/handling.cfg")
    return write_dnv(rows, out)
