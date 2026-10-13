# DiamondNav roadmap

DiamondNav gives NPCs the ability to move well. It never decides which NPCs exist:
spawning traffic, pedestrians or gangs is the gamemode's job. Each item below is a
native (or a few) that a script calls on an NPC it already made.

## 1.0 (done)

- `Nav_WalkTo`: walking paths on a navmesh (a map, or the whole world from GTA).
- `Nav_DriveTo`: road routes on GTA's traffic paths (`roads.dnr`), in the right-hand
  lane, slowing for corners.
- `Nav_RequestPath` / `Nav_RequestRoute`: the same searches, with the points handed
  to the script.

## 1.1: chasing

- **`Nav_Chase(npcid, targetid, speed, flags)`**: a driver follows a player (or
  another NPC) until `Nav_Stop`. The component re-plans as the target moves, and
  keeps driving the old plan until the new one is ready, so the car never pauses.
  - Far away, with the target near a road: a road route, as `Nav_DriveTo`.
  - The target off the roads (a field, a car park, a beach): a path on the **car
    navmesh**, so the car leaves the road where it has to.
  - Close and with a clear straight line: it drives straight at where the target
    will be in half a second.
  - It ends with `OnNavMoveEnd(NAV_END_LOST)` if the target leaves or changes world.
- **Car navmesh**: a second navmesh baked for a car (2 m tall, 1.4 m round, climbs
  0.6 m, slopes up to 35°), from the same world geometry as the walking one.
  `dnav.py world` bakes both; `.nav` files say which kind they are.
- **`Nav_IsLineClear(x1, y1, z1, x2, y2, z2, kind)`**: whether a ped (or a car) can go
  straight from one point to the other, with no wall, drop or obstacle in between.
  Scripts use it to steer, or as a cheap check before shooting. (A true eye-level
  line of sight through windows and fences needs the full collision mesh; that stays
  ColAndreas' job, which a gamemode can use alongside DiamondNav.)

## 1.2: traffic

- **`Nav_Cruise(npcid, speed, flags)`**: a driver drives around the road network on
  its own, picking turns at random (or towards an area), in its lane.
  - Keeps a gap to the vehicle in front: other DiamondNav cars and players' cars.
  - Slows into junctions and gives way to a car already in the junction.
  - Ends only on `Nav_Stop`, or when the NPC leaves the car.
- `Nav_DriveTo` gets the same spacing and junction behaviour.

## 1.3: pedestrians

- **`Nav_Wander(npcid, radius, flags)`**: a ped walks to random points around it,
  preferring sidewalks; the walking navmesh marks road surface as costly (from
  `roads.dnr` at bake time), so peds cross roads instead of walking along them.
- Peds walking with DiamondNav keep apart from each other instead of walking
  through each other.
- `Nav_Flee(npcid, x, y, z)`: run away from a point (a shooting, a crash).
