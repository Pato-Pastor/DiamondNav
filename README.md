# DiamondNav

Walking paths and driving routes for [open.mp](https://open.mp) NPCs.

- **Walking:** NPCs find their way on foot round walls, up stairs and across
  bridges, on a navmesh baked from the game world's collision (Recast/Detour).
- **Driving:** NPC drivers follow the roads and keep to their lane, on a road graph
  built from GTA's own traffic paths. They drive like their car: GTA's own vehicle
  physics are run offline on each model's handling data (gears, engine, drag,
  tyres, brakes, steering, turn mass, off-road ability) and the size of its model,
  so each pulls away, tops out, brakes, corners and turns in as it does in the
  game, on a road and on dirt. Each road has a speed limit by its class (highway, main road,
  street, minor road, service road), they brake in time for corners and junctions,
  and a driving style (cautious, normal, fast, pursuit) sets how hard they push.
- **Chasing:** an NPC driver chases a player along the roads, and off them across a
  car-sized navmesh when the player leaves the road; close in, it drives straight
  at them when nothing is in the way, keeps up alongside a car that is getting
  away, and follows a player off a bridge or a ledge, through a gap in its railing.
  With the world's collision loaded they see every wall and object (yours too) and
  never drive or fall through one, and a pursuit takes the shortest way, turning
  round or going the wrong way down a one-way street. Several chasers come up either
  side of their target. Drivers steer like cars: they curve into corners, brake for
  them, and back up to turn round in tight spots.
- **Traffic sense:** every driver swerves round, or brakes behind, the vehicles in
  its way (other NPCs' cars, players' cars, parked cars), by each model's own
  collision box and where it is heading.
- **Your maps too:** custom SA-MP maps are baked in: objects your scripts create,
  buildings they remove, and custom models with their own collision. Build the
  world from a GTA San Andreas install or from ColAndreas's database.
- **Never holds up the server:** searches run on their own threads; results come
  back on the game thread through a callback.
- **Optional extra:** a build of open.mp's NPC component in which NPCs can fire from
  cars (drive-bys) and cars look right while NPCs drive them.

```pawn
#include <open.mp>
#include <DiamondNav>

Nav_WalkTo(npcid, x, y, z);                // walk there along the navmesh
Nav_DriveTo(npcid, x, y, z);               // drive the NPC's car there along the roads
Nav_Chase(npcid, playerid);                // chase a player, on or off the road

public OnNavMoveEnd(npcid, reason)
{
    if (reason == NAV_END_ARRIVED) { /* ... */ }
    return 1;
}
```

Licensed under the [Mozilla Public License 2.0](LICENSE). Works with open.mp
v1.6.7.3218 (the 32-bit server, Linux or Windows).

## Install

Download the archive for your server from
[Releases](https://github.com/payrle-dev/DiamondNav/releases) and unpack it into
the server folder:

| From the archive | What it is |
| --- | --- |
| `components/DiamondNav.so` / `.dll` | the component; open.mp loads it at start |
| `qawno/include/DiamondNav.inc` | the include for your scripts |
| `navmesh/` | where the baked files go |
| `navtools/` | `navbake`, `navquery` and `dnav.py`, to bake on your PC |
| `extras/NPCs.so` / `.dll` | optional: the improved NPC component ([below](#the-improved-npc-component)) |

Then bake your world into `navmesh/` (next section). The server log says what it
loaded:

```
[nav] loaded roads.dnr: <nodes> road nodes, <links> links
[nav] loaded world.nav (walking)
[nav] loaded vehicles.dnv: <models> vehicle models
[nav] loaded world.col: <faces> faces
[nav] DiamondNav 1.3.6: <tiles> walking tiles, <tiles> car tiles, <nodes> road nodes, 3 search threads
```

## Baking your world

Baking happens once, on your PC, whenever your maps change. The tools need
Python 3.9+ with numpy (`pip install numpy`, plus `pillow` for previews).

### 1. The world's collision

From a GTA San Andreas install (the 0.3.DL client folder players use is best: it
has SA-MP's own objects in `SAMP/samp.img`):

```sh
python navtools/dnav.py world bake/world --gta "C:/Games/GTA San Andreas"
```

Or from [ColAndreas](https://github.com/Pottus/ColAndreas)'s database, if you
already have one (`scriptfiles/colandreas/ColAndreas.cadb`):

```sh
python navtools/dnav.py world bake/world --colandreas scriptfiles/colandreas/ColAndreas.cadb
```

Give both to get the database's objects and the install's path nodes and water
(the path nodes let the bake drop roofs and closed-off spaces nobody walks on).

### 2. Your maps

Add `--map` for each map file or folder, and `--models` for your custom models:

```sh
python navtools/dnav.py world bake/world --gta "C:/Games/GTA San Andreas" \
    --map gamemodes/maps --map filterscripts/mapping.pwn --map maps/island.map \
    --models models
```

- **Map files** are read for literal `CreateObject`, `CreateDynamicObject`,
  `CreateDynamicObjectEx`, `CreatePlayerObject` and `RemoveBuildingForPlayer` calls
  (`.pwn`, `.inc`, `.txt`), or MTA `.map` files (`<object>` and
  `<removeWorldObject>`), which most map editors export. A call whose arguments
  are variables can't be read; the build counts them.
- **Custom models** (`AddSimpleModel`, in your scripts or in
  `models/artconfig.txt`) get their collision from their DFF, as in the game
  (SA-MP's collision plugin, which 0.3.DL models carry, or Rockstar's), or from a
  `.col` file in the models folder.

The build prints what it placed and what had no collision.

### 3. Bake the navmesh

```sh
navtools/navbake bake/world.tris navmesh/world.nav --seeds bake/world.seeds.txt --water bake/world.water.txt
```

For cars that leave the road (`Nav_Chase`, `NAV_DRIVE_OFFROAD`), bake the same world
again for a car:

```sh
navtools/navbake bake/world.car.tris navmesh/world_car.nav --agent car --water bake/world.water.txt
```

For chases, copy the world's collision itself into `navmesh/` too. Drivers cast
rays against it to see walls and your objects, and to find the ground under a car
that falls; without it they only know what the car navmesh says:

```sh
cp bake/world.car.tris navmesh/world.col
```

`world.car.tris` is `world.tris` without the stock objects a car breaks or knocks
over in the game (data/object.dat: fences, railings, lamp posts, signs, bins,
barriers), so cars go straight through those, on the car navmesh and in a chase,
as a player's car would. Objects your scripts create never break in SA-MP, so they
stay solid.

(`dnav.py world` prints these lines with the files it wrote.) `--area "X0 Y0 X1 Y1"` on `dnav.py world` bakes part of it, which is
quicker for trying things out. A `--routes` file makes navbake check that paths
exist (each line `name<TAB>x y z<TAB>x y z`) and fail if one doesn't.

### 4. The roads

```sh
python navtools/dnav.py roads navmesh/roads.dnr --gta "C:/Games/GTA San Andreas"
```

For a custom map with its own streets, add a roads file with `--roads my_roads.txt`
(`dnav.py roads --help-format` explains it):

```
# a car park off the main road
node gate   1520.0 -1650.0 13.5
node park   1560.0 -1650.0 13.5
road 1500.0,-1650.0,13.5 gate 1 1      # join the nearest stock road node
road gate park 1 0                     # one-way into the car park
remove 1600 -1700 1700 -1600           # this map builds over the stock road here
```

### 5. How each vehicle drives

```sh
python navtools/dnav.py vehicles navmesh/vehicles.dnv --gta "C:/Games/GTA San Andreas"
```

This runs GTA's own vehicle physics for every model, once, on your PC: the gearbox
(gears, the first- and second-gear boosts, the speed-limit flag), the engine's
acceleration and inertia, air drag, the tyres (traction, traction loss, the
front/rear bias, wheelspin), the brakes and their bias, from `data/handling.cfg`
(and its bike lines: how far a bike leans); how well tyres grip a road and dirt
from `data/surface.dat` (with the off-road ability flags); and each model's
wheelbase, track and size from its DFF in `models/gta3.img`. It writes each model's
acceleration at every speed on a road and on dirt, top speed, braking, cornering
grip (the weaker axle, a bike's lean, the roll a tall vehicle takes), turning
circle, how fast it can start to turn (its turn mass), reverse speed and size. A
car with the Infernus's line tops out at 221 km/h, as measured in the game (222),
not handling.cfg's 240.

Without it every car drives as the same average car. To check a model against the
real game, drive it yourself with a script that times your top speed, 0-100 km/h,
braking and cornering next to `Nav_GetVehicleInfo`'s numbers, and if one is off, set
it from your script with `Nav_SetVehicleHandling`.

### Checking a bake

```sh
navtools/navquery navmesh info
navtools/navquery navmesh path  1700 -2490 13.5  1650 -2300 13.5
navtools/navquery navmesh route 1700 -2490 13.5  1480 -1730 13.5
navtools/navquery navmesh path  2350 -1819 12.5  2350 -1852 0.7 --car --ledges
python navtools/dnav.py preview check.png --bake bake/world --roads navmesh/roads.dnr
```

## Scripting

See [`include/DiamondNav.inc`](include/DiamondNav.inc) for every native, and
[`examples/`](examples) for whole scripts (NPCs wandering the city, a taxi that
drives to you).

**Let DiamondNav move the NPC.** `Nav_WalkTo` and `Nav_DriveTo` search, then move
the NPC along the result point by point with open.mp's `NPC_Move`, and call
`OnNavMoveEnd(npcid, reason)` at the end: `NAV_END_ARRIVED`, `NAV_END_NO_PATH`,
`NAV_END_STOPPED` (`Nav_Stop`, or a new move), or `NAV_END_INTERRUPTED` (another
script moved it, it left the driver's seat, died or was destroyed). A driver must
be in seat 0; `carZ` is how high the car's centre sits over the road, and the
default (0.0) takes the model's own: `vehicles.dnv`'s, worked out from its wheels
and springs as the game places it, else a guess from open.mp's model table
(`NAV_VINFO_RIDE`). Off a road route (a straight run at a chase target, or a car
left with no path), the car keeps to the ground under it: the car navmesh, else the
walking navmesh, else the road surface nearby; it never climbs onto a level it has
no ground for.

**How cars drive.** A driver steers like a car: it looks ahead along its path,
turns no tighter than its turning circle (nor, at speed, than its grip holds),
eases into and out of turns as fast as its turn mass lets it, pulls away as its
own gears and engine allow at each speed (less on dirt) and
brakes in time to take each corner, junction or lower limit at the speed it
allows, then to stop at the end. What sets the speed:

- **The road's class**, read from GTA's path data, each with a limit you can change
  with `Nav_SetRoadSpeed` (km/h): `NAV_CLASS_HIGHWAY` 130, `NAV_CLASS_MAIN` 80 (two
  or more lanes each way, or one-way), `NAV_CLASS_STREET` 60, `NAV_CLASS_MINOR` 45
  (GTA's low-traffic roads), `NAV_CLASS_SERVICE` 30 (emergency-only roads) and
  `NAV_CLASS_OFFROAD` 60 (the car navmesh). A custom roads file marks its own with
  `main` / `minor` on a node.
- **The car**, from `vehicles.dnv` or `Nav_SetVehicleHandling`; read any of it with
  `Nav_GetVehicleInfo` (`NAV_VINFO_*`), `Nav_GetVehicleAccel` and
  `Nav_GetVehicleAccelTime`.
- **The style**, `Nav_SetDriveStyle(npcid, style)`: `NAV_STYLE_CAUTIOUS` (under the
  limits, gentle), `NAV_STYLE_NORMAL` (the limits; `Nav_DriveTo`'s default),
  `NAV_STYLE_FAST` (a quarter over, brisk corners, late braking) and
  `NAV_STYLE_PURSUIT` (no limits at all: flat out, cornering on the limit of the
  car's grip and braking as hard as it can; `Nav_Chase`'s default).
- **A cap**, if you pass a `speed` above 0 to `Nav_DriveTo` or `Nav_Chase`
  (in `NPC_Move`'s units: 1.0 is 10 m/s).

`Nav_GetDriveInfo` tells you how a driver is doing: its speed, the speed it is going
for, the road it is on and that road's limit. With the improved NPC component a
car also backs up to turn round in a tight spot.

**Other vehicles.** Every driver sweeps the course it is about to take against
every vehicle in its world, by each model's collision box (`vehicles.dnv`, from its
DFF: its size and where the box sits; else open.mp's model table) turned the way
that vehicle faces, and moved on at its own speed. If it would touch one, it
swerves onto the smallest change of heading that touches nothing, keeping to one
side until it is past; with no way past it brakes to stop short of it, or follows
it at its speed, and after a moment stopped it backs up to make room (with the
improved NPC component). Parked cars, players' cars and other NPCs' all count; a
chaser with `NAV_CHASE_RAM` goes into its target's car on purpose.

**Chase.** `Nav_Chase(npcid, targetid, speed, carZ, flags)` keeps a driver after a
player (or another NPC) until `Nav_Stop`. It follows the roads while the target is
on one, the car navmesh once it leaves them, and drives straight at where the
target is going (as far ahead as the car takes to get there) when it is close and
nothing is in the way: `Nav_IsLineClear` on the car navmesh, or, without one, a
line that runs over road all the way (up to 150 m) or a very short one. With
`world.col` loaded that line must also clear every wall at bumper height and keep
to the ground, and a wall that blocks a straight run sends it to the roads for a
moment. Its road routes are a pursuit's: from wherever the car can get on the road,
turning round if that is shorter (it costs about 35 m), and the wrong way down a
one-way road at a small cost. It aims at where the target is heading on average,
not where it points this instant, so a target swerving about does not make it
brake. With `world.col`, a target below it is driven at straight down a bank or off
a ledge, and a chaser that goes over an edge falls. With the car navmesh as well,
the server finds every ledge a car could drive off at boot (a bridge's open side, a
storm drain's edge: a run-up clear of walls and posts, a drop of 2 to 15 m onto more
of the car navmesh), and a chase's routes use them as shortcuts, so a chaser drives
round to the nearest way down instead of the long way by road. It turns round in a
narrow street with a three-point turn, and backs out and tries the other side when
it is wedged on a post. A chaser left far behind a
moving target gets extra speed (up to 35% at 160 m), as the game's police do. It keeps
up with a target that is moving, pulling up alongside on its left, stops beside
one that stands still, and re-plans as the target moves without pausing. A target
that drops off a bridge or a ledge (up to 30 m down, within 60 m) is followed off
the edge (without the car navmesh's ledges): the car drives to it on its own level and falls, keeping its speed, to
the road below. With `world.col` it finds where the edge is, lines up square to it
and goes through a gap in the railing (the nearest within 12 m), stops falling
forward when it meets a wall, and lands on whatever is under it (`NAV_CHASE_NO_DROP` takes the roads round instead). Several NPCs
after one target come up either side of it instead of in a line;
`NAV_CHASE_RAM` drives into it without slowing. `Nav_GetChaseInfo(npcid, distance,
mode)` says how far the target is and what the chaser is doing (beside it,
straight at it, on a route, running at an edge, in the air). It ends with
`NAV_END_LOST` if the target leaves or changes world. What the NPCs do during the
chase (shoot, give up) is your script's. `NAV_STYLE_PURSUIT` drives flat out:
cornering on the limit of the car's grip and braking as hard as it can.

**Lines.** `Nav_IsLineClear(x1, y1, z1, x2, y2, z2, kind)` says whether a ped
(`NAV_MESH_PED`) or a car (`NAV_MESH_CAR`) can go straight between two points on the
ground: no wall, drop or obstacle. It follows the ground, so it is not an eye-level
line of sight; drivers use the world's collision (`world.col`) for that.

**Or take the points.** `Nav_RequestPath` (walking) and `Nav_RequestRoute`
(driving) return a request id; `OnNavPathResult(requestid, status, points)` follows
on a later tick. Read the points with `Nav_GetPathPoint` (and, for routes, each
point's speed factor with `Nav_GetPathPointSpeed` and road class with
`Nav_GetPathPointClass`) inside that callback.

Road flags: `NAV_ROAD_EMERGENCY` lets a route use emergency-only roads;
`NAV_ROAD_BOATS` searches the water routes instead (for boats).

## The improved NPC component

`extras/NPCs.so` (`NPCs.dll`) is open.mp's own NPC component, built from the
v1.6.7.3218 source with [our patch](npcs/patches) applied. Copy it over
`components/NPCs.so`. Everything else in open.mp stays stock, and DiamondNav works
without it. What it changes:

- **Drive-bys.** `NPC_AimAt` / `NPC_AimAtPlayer` with shooting on now fire from a
  car. Passengers lean out and aim (any bullet weapon but the sniper rifle and
  minigun); drivers fire the Uzi, MP5 and Tec-9 sideways, as in GTA.
- **Passengers ride along**: their position follows the car (stock left them where
  they got in).
- **Driving looks right**: the front wheels steer into turns, straighten when the
  car stops, and the car never rolls. (Sending the car's speed in the game's units
  and pitching it along slopes were ours until open.mp 1.6.7 did both itself.)
- **Cars back up**: an `NPC_Move` drive with a negative speed reverses away from its
  target. DiamondNav uses it for three-point turns, and for a target just behind the
  car; with the stock component it notices, says so once in the log, and turns
  round going forward instead.

It only fits the open.mp release it was built from. To build it yourself or for
another release, see [npcs/README.md](npcs/README.md).

## Building from source

The server is 32-bit, so everything is built 32-bit. CMake fetches the
dependencies (the open.mp SDK, pawn-natives, the Pawn headers, Recast/Detour).

```sh
# Linux (needs g++-multilib, cmake, ninja)
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS=-m32 -DCMAKE_CXX_FLAGS=-m32
cmake --build build
ctest --test-dir build
python3 tools/tests/test_tools.py build

# Windows (Visual Studio 2022 with the Clang tools)
cmake -S . -B build -A Win32 -T ClangCL
cmake --build build --config Release
```

`tests/server/run.py` boots a real open.mp server with the component on a small
made-up map and checks that an NPC walks round a wall and another drives a route.
CI runs all of it on every push.

## Good to know

- There is no physics on the server: DiamondNav drives the car as a driver would
  (speed, turning, braking), but it does not crash, skid or get pushed. Drivers
  don't stop for traffic or players yet; that's up to your script.
- How GTA's path files mark lane direction and median width is read from the files
  as documented by the community, and has not been checked against every road.
  If a car drives on the wrong side somewhere, please open an issue with the place.
- Nothing from GTA is in this repository or the releases: you bake from your own
  copy, and the navmesh holds only walkable floor polygons.

## Credits

Built on [Recast & Detour](https://github.com/recastnavigation/recastnavigation)
(zlib license) and the [open.mp](https://github.com/openmultiplayer/open.mp) SDK
(MPL-2.0). The ColAndreas database reader was written from the file format; no
ColAndreas code is used. See [THIRD_PARTY.md](THIRD_PARTY.md).
