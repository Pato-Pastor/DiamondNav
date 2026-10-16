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

## 1.1: chasing (done)

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

## 1.1.1-1.1.2 (done)

- Drivers steer like cars: a lookahead point, a turning circle, smooth speed, the
  car navmesh's ground off the road, and backing up to turn round.

## 1.2: driving like a car (done)

- Road classes from GTA's own path data (highway, main, street, minor, service), each
  with a speed limit (`Nav_SetRoadSpeed`).
- Each vehicle's handling from the game's `handling.cfg` (`dnav.py vehicles`):
  top speed, acceleration, braking, grip, turning circle (`Nav_SetVehicleHandling`).
- A speed plan along every path: limits, corners by the car's grip, junctions, and
  braking in time for each.
- Driving styles (`Nav_SetDriveStyle`): cautious, normal, fast, pursuit.
- `Nav_GetDriveInfo`, `Nav_GetPathPointClass`.

## 1.3: the game's own handling (done)

- `dnav.py vehicles` runs GTA's vehicle physics offline on every handling.cfg
  column: the gearbox, engine inertia, drag, tyres and wheelspin, brakes, steering,
  turn mass, off-road ability, bike lean; surface.dat for road and dirt; each
  model's DFF for its wheelbase, track and size. vehicles.dnv version 2 carries an
  acceleration table per model (road and dirt) and the rest.
- Drivers pull away by that table, brake and corner on dirt by the dirt numbers, and
  ease into turns by the turn mass.
- `Nav_GetVehicleInfo`, `Nav_GetVehicleAccel`, `Nav_GetVehicleAccelTime`.
- 1.3.1: built for open.mp 1.6.7 (its SDK, and the NPC patch rebased onto it; open.mp
  now sends a driving NPC's speed in the game's units and pitches the car itself).
- 1.3.2: each car sits at its own height over the road (vehicles.dnv version 3, from
  its wheels and springs; `carZ` 0.0 by default), and cars off a road route keep to
  the ground (car mesh, walking mesh or the road surface) instead of floating.
- 1.3.3: chases. A chaser follows its target off a bridge or a ledge (falling to the
  road below), keeps up alongside a moving target instead of stopping, leads it by
  the time it takes to get there, drives straight along a road up to 150 m without
  a car navmesh, and spreads out with other chasers; `NAV_CHASE_RAM`,
  `NAV_CHASE_NO_DROP`, `Nav_GetChaseInfo`; pursuit brakes flat out. Every driver
  swerves round or stops for vehicles in its way by their collision boxes
  (vehicles.dnv version 4 adds where each box sits). Road routes start and end
  along long stretches of road, not only near a node.
- 1.3.4: chases know the map. `navmesh/world.col` (the bake's world.tris) is loaded
  for rays: chasers check walls and the ground before a straight run, swerve round
  walls and objects, find a railing's gap and line up square to it, and fall without
  passing through anything. Breakable stock objects (object.dat: fences, railings,
  lamp posts, bins) are left out of the car bake and world.col, so cars smash
  through them as in the game. Pursuit routes start wherever the car can join the road
  (turning round if that is shorter) and may go against one-way traffic, and the
  chaser aims at the target's smoothed heading so it stops braking for every swerve.

## 1.4: traffic

- **`Nav_Cruise(npcid, speed, flags)`**: a driver drives around the road network on
  its own, picking turns at random (or towards an area), in its lane.
  - Keeps a gap to the vehicle in front: other DiamondNav cars and players' cars
    (1.3.3's avoidance already swerves round or stops for them; traffic adds lanes).
  - Slows into junctions and gives way to a car already in the junction.
  - Ends only on `Nav_Stop`, or when the NPC leaves the car.
- `Nav_DriveTo` gets the same spacing and junction behaviour.

## 1.5: pedestrians

- **`Nav_Wander(npcid, radius, flags)`**: a ped walks to random points around it,
  preferring sidewalks; the walking navmesh marks road surface as costly (from
  `roads.dnr` at bake time), so peds cross roads instead of walking along them.
- Peds walking with DiamondNav keep apart from each other instead of walking
  through each other.
- `Nav_Flee(npcid, x, y, z)`: run away from a point (a shooting, a crash).
