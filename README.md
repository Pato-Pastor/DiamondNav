# DiamondNav

Walking paths and driving routes for [open.mp](https://open.mp) NPCs.

- **Walking:** NPCs find their way on foot round walls, up stairs and across
  bridges, on a navmesh baked from the game world's collision (Recast/Detour).
- **Driving:** NPC drivers follow the roads, keep to their lane, round corners and
  slow down for them, on a road graph built from GTA's own traffic paths.
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
Nav_DriveTo(npcid, x, y, z, 1.2);          // drive the NPC's car there along the roads

public OnNavMoveEnd(npcid, reason)
{
    if (reason == NAV_END_ARRIVED) { /* ... */ }
    return 1;
}
```

Licensed under the [Mozilla Public License 2.0](LICENSE). Works with open.mp
v1.5.8.3079 (the 32-bit server, Linux or Windows).

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
[nav] loaded world.nav
[nav] DiamondNav 1.0.0: <tiles> navmesh tiles, <nodes> road nodes, 3 search threads
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
  `models/artconfig.txt`) get their collision from their DFF, as in the game, or
  from a `.col` file in the models folder.

The build prints what it placed and what had no collision.

### 3. Bake the navmesh

```sh
navtools/navbake bake/world.tris navmesh/world.nav --seeds bake/world.seeds.txt --water bake/world.water.txt
```

(`dnav.py world` prints this line with the files it wrote.) `--area "X0 Y0 X1 Y1"` on `dnav.py world` bakes part of it, which is
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

### Checking a bake

```sh
navtools/navquery navmesh info
navtools/navquery navmesh path  1700 -2490 13.5  1650 -2300 13.5
navtools/navquery navmesh route 1700 -2490 13.5  1480 -1730 13.5
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
be in seat 0; `carZ` is how high the car's centre sits over the road (1.0 for a
car, more for a truck).

**Or take the points.** `Nav_RequestPath` (walking) and `Nav_RequestRoute`
(driving) return a request id; `OnNavPathResult(requestid, status, points)` follows
on a later tick. Read the points with `Nav_GetPathPoint` (and, for routes, each
point's speed factor with `Nav_GetPathPointSpeed`) inside that callback.

Road flags: `NAV_ROAD_EMERGENCY` lets a route use emergency-only roads;
`NAV_ROAD_BOATS` searches the water routes instead (for boats).

## The improved NPC component

`extras/NPCs.so` (`NPCs.dll`) is open.mp's own NPC component, built from the
v1.5.8.3079 source with [our patch](npcs/patches) applied. Copy it over
`components/NPCs.so`. Everything else in open.mp stays stock, and DiamondNav works
without it. What it changes:

- **Drive-bys.** `NPC_AimAt` / `NPC_AimAtPlayer` with shooting on now fire from a
  car. Passengers lean out and aim (any bullet weapon but the sniper rifle and
  minigun); drivers fire the Uzi, MP5 and Tec-9 sideways, as in GTA.
- **Passengers ride along**: their position follows the car (stock left them where
  they got in).
- **Driving looks right**: the car's speed is sent in the units the game expects
  (stock sent it 20 times too small, so cars crawled and jumped for players), cars
  pitch along slopes, and the front wheels steer into turns.

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

- Cars driven by NPCs slide from point to point at the speed you give: there is no
  physics on the server. They don't stop for traffic or players; that's up to your
  script.
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
