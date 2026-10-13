// DiamondNav: walking paths and driving routes for open.mp NPCs.
// https://github.com/payrle-dev/DiamondNav  (MPL-2.0)
//
// The DiamondNav component (components/DiamondNav.so or .dll) loads, from the
// server's navmesh/ folder:
//   *.nav       where peds can walk, and where a car fits off the road (navbake
//               --agent ped or car; dnav.py world bakes both)
//   roads.dnr   where cars can drive (built with tools/dnav.py roads)
//   vehicles.dnv how each vehicle model drives (tools/dnav.py vehicles, from the
//               game's handling.cfg; without it every car drives alike)
// and searches paths on its own threads, so a search never holds up the server.
//
// Two ways to use it:
//   - Let it move the NPC: Nav_WalkTo / Nav_DriveTo / Nav_Chase, then OnNavMoveEnd
//     says how it ended. It follows the path point by point with open.mp's NPC_Move.
//   - Ask for the points and move the NPC yourself: Nav_RequestPath /
//     Nav_RequestRoute, then OnNavPathResult hands over the points.
//
// Coordinates are GTA's. Path points are on the ground (a ped's feet, the road
// surface); Nav_WalkTo and Nav_DriveTo take care of the height of a ped or a car.

#if defined _DiamondNav_included
    #endinput
#endif
#define _DiamondNav_included

#define DIAMONDNAV_INCLUDE_VERSION      (10200)     // 1.2.0

// ── Results ──────────────────────────────────────────────────────────────────

// OnNavPathResult's status.
#define NAV_OK                  (0)     // the whole way
#define NAV_PARTIAL             (1)     // walking: as near to the end as the navmesh gets
#define NAV_NO_START            (-1)    // nothing walkable (or no road) near the start
#define NAV_NO_END              (-2)    // nothing walkable (or no road) near the end
#define NAV_NO_PATH             (-3)    // start and end are not connected
#define NAV_NOT_LOADED          (-4)    // no navmesh (or no road graph) was loaded

// OnNavMoveEnd's reason.
#define NAV_END_ARRIVED         (0)     // it got there
#define NAV_END_NO_PATH         (1)     // there was no way there
#define NAV_END_STOPPED         (2)     // Nav_Stop, or a new Nav_WalkTo / Nav_DriveTo
#define NAV_END_INTERRUPTED     (3)     // something else moved it, it left its car (or the
                                        // driver's seat), died or was destroyed
#define NAV_END_LOST            (4)     // Nav_Chase: the target left, or went to another world

// Road flags for Nav_RequestRoute, Nav_DriveTo and Nav_GetNearestRoadPoint.
#define NAV_ROAD_EMERGENCY      (1)     // may use roads only emergency vehicles drive on
#define NAV_ROAD_BOATS          (2)     // a boat: water routes only
// Nav_DriveTo only: across the car navmesh (anywhere a car fits) instead of the roads.
#define NAV_DRIVE_OFFROAD       (4)

// Which navmesh: where peds walk, or where a car fits.
#define NAV_MESH_PED            (0)
#define NAV_MESH_CAR            (1)

// Road classes, each with its own speed limit (Nav_SetRoadSpeed). Read from GTA's own
// path data: its highway flag, the lanes each way and its traffic level.
#define NAV_CLASS_HIGHWAY       (0)     // freeways (default limit 130 km/h)
#define NAV_CLASS_MAIN          (1)     // two or more lanes each way, or one-way (80)
#define NAV_CLASS_STREET        (2)     // one lane each way (60)
#define NAV_CLASS_MINOR         (3)     // GTA's low-traffic roads: back streets, country lanes (45)
#define NAV_CLASS_SERVICE       (4)     // emergency-only roads: aprons, yards, car parks (30)
#define NAV_CLASS_OFFROAD       (5)     // off the roads, on the car navmesh (60)

// How a driver drives (Nav_SetDriveStyle).
#define NAV_STYLE_CAUTIOUS      (0)     // under the limits, gentle corners, early braking
#define NAV_STYLE_NORMAL        (1)     // the limits, comfortable corners (Nav_DriveTo's default)
#define NAV_STYLE_FAST          (2)     // a quarter over the limits, brisk corners, late braking
#define NAV_STYLE_PURSUIT       (3)     // no limits: as fast as the car and the corners allow
                                        // (Nav_Chase's default)
#define NAV_STYLE_DEFAULT       (-1)    // Nav_SetDriveStyle: back to the default

// ── Information ──────────────────────────────────────────────────────────────

// Navmesh tiles loaded (25.6 m squares walking, 64 m for cars); 0 without that navmesh.
native Nav_GetTileCount(kind = NAV_MESH_PED);

// Road nodes loaded; 0 without a road graph.
native Nav_GetRoadNodeCount();

// Searches queued or running.
native Nav_GetPendingCount();

// ── Moving NPCs ──────────────────────────────────────────────────────────────

// Walks (or runs, or sprints) an NPC to a point along the navmesh. moveType is
// open.mp's NPC_MOVE_TYPE_WALK, _JOG or _SPRINT; speed as NPC_Move takes it.
// Returns false when the NPC does not exist or the search queue is full.
native bool:Nav_WalkTo(npcid, Float:x, Float:y, Float:z, moveType = 1, Float:speed = -1.0);

// Drives an NPC's car to a point along the roads, in its lane. The NPC must be the
// driver. It drives like its car (vehicles.dnv) in its style (Nav_SetDriveStyle):
// up to each road's speed limit, slowing for corners and junctions in time, and
// braking to a stop at the end. speed, if above 0, caps it in NPC_MOVE_TYPE_DRIVE's
// units (1.0 is 10 m/s); carZ is how far the car's centre sits above the road
// (about 1.0 for a car; more for a truck or bus).
native bool:Nav_DriveTo(npcid, Float:x, Float:y, Float:z, Float:speed = 0.0, Float:carZ = 1.0, flags = 0);

// Chases a player (or another NPC) with an NPC's car until Nav_Stop. Far off, it
// follows the roads while the target is on them, and the car navmesh once the target
// leaves them; close and with nothing in the way, it drives straight at where the
// target is going, and it stops beside a target that stops. It re-plans as the
// target moves, without pausing. The NPC must be the driver. Ends with
// NAV_END_LOST when the target leaves or changes world. What the NPCs do in the
// chase (shoot, ram, give up) is the script's. It drives in NAV_STYLE_PURSUIT unless
// the NPC has a style of its own; speed, if above 0, caps it (as Nav_DriveTo's).
native bool:Nav_Chase(npcid, targetid, Float:speed = 0.0, Float:carZ = 1.0);

// Stops a Nav_WalkTo, Nav_DriveTo or Nav_Chase where the NPC is (OnNavMoveEnd:
// NAV_END_STOPPED).
native bool:Nav_Stop(npcid);

// Whether a Nav_WalkTo, Nav_DriveTo or Nav_Chase is under way (or searching).
native bool:Nav_IsMoving(npcid);

// ── How cars drive ───────────────────────────────────────────────────────────

// An NPC's driving style (NAV_STYLE_*), kept until the NPC is destroyed; it applies
// at once to a drive or chase under way. NAV_STYLE_DEFAULT clears it, and
// Nav_GetDriveStyle is NAV_STYLE_DEFAULT for an NPC that has none (Nav_DriveTo then
// drives normally, Nav_Chase in pursuit).
native bool:Nav_SetDriveStyle(npcid, style);
native Nav_GetDriveStyle(npcid);

// A road class's speed limit (NAV_CLASS_*), in km/h, for every driver.
native bool:Nav_SetRoadSpeed(roadClass, Float:kmh);
native Float:Nav_GetRoadSpeed(roadClass);

// How a vehicle model drives: top speed (km/h), acceleration and hardest braking
// (m/s^2), the most sideways acceleration it holds in a corner (m/s^2) and its
// tightest turning circle (m). Setting it overrides vehicles.dnv for that model.
// Without either, a model drives as an average car.
native bool:Nav_SetVehicleHandling(modelid, Float:topKmh, Float:accel, Float:brake, Float:grip, Float:radius);
native bool:Nav_GetVehicleHandling(modelid, &Float:topKmh, &Float:accel, &Float:brake, &Float:grip, &Float:radius);

// Vehicle models vehicles.dnv gave handling for; 0 without it.
native Nav_GetVehicleModelCount();

// A driver under way: its speed, the speed it is going for (the road, the next
// corner, the end), the class of road it is on and that road's limit. False when the
// NPC is not driving with DiamondNav.
native bool:Nav_GetDriveInfo(npcid, &Float:speedKmh, &Float:wantKmh, &roadClass, &Float:limitKmh, &bool:reversing);

// ── Searching ────────────────────────────────────────────────────────────────

// Queues a walking path search. Returns the request id OnNavPathResult will carry,
// or 0 when the queue is full.
native Nav_RequestPath(Float:fromX, Float:fromY, Float:fromZ, Float:toX, Float:toY, Float:toZ);

// Queues a driving route search on the road graph (flags: NAV_ROAD_*).
native Nav_RequestRoute(Float:fromX, Float:fromY, Float:fromZ, Float:toX, Float:toY, Float:toZ, flags = 0);

// Point `index` (0 .. points - 1) of the result being handed to OnNavPathResult.
// Only valid during that callback.
native bool:Nav_GetPathPoint(requestid, index, &Float:x, &Float:y, &Float:z);

// A route point's speed, 0.0 .. 1.0 of cruising speed (lower in corners and at
// the end). Walking paths are always 1.0.
native Float:Nav_GetPathPointSpeed(requestid, index);

// A route point's road class (NAV_CLASS_*); -1 for a walking path.
native Nav_GetPathPointClass(requestid, index);

// ── Points ───────────────────────────────────────────────────────────────────

// The nearest point a ped can walk on (or a car fit), within `reach` metres.
native bool:Nav_GetNearestPoint(Float:x, Float:y, Float:z, &Float:outX, &Float:outY, &Float:outZ, Float:reach = 3.0,
    kind = NAV_MESH_PED);

// A random point a ped can walk on (or a car fit), anywhere on that navmesh.
native bool:Nav_GetRandomPoint(&Float:x, &Float:y, &Float:z, kind = NAV_MESH_PED);

// Whether a ped (or a car) can go in a straight line from one point to the other:
// no wall, drop or obstacle in between. Points are on the ground. Use it to steer,
// or as a cheap check before an NPC shoots; it follows the ground, so it does not
// see through windows or over fences (a full line of sight needs the collision
// mesh: ColAndreas). False when either point is off the navmesh.
native bool:Nav_IsLineClear(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, kind = NAV_MESH_PED);

// The nearest road node, within `reach` metres (flags: NAV_ROAD_*).
native bool:Nav_GetNearestRoadPoint(Float:x, Float:y, Float:z, &Float:outX, &Float:outY, &Float:outZ,
    Float:reach = 50.0, flags = 0);

// ── Callbacks ────────────────────────────────────────────────────────────────

// A search finished. Read the points with Nav_GetPathPoint (and
// Nav_GetPathPointSpeed) here; they are gone after the callback returns.
forward OnNavPathResult(requestid, status, points);

// A Nav_WalkTo, Nav_DriveTo or Nav_Chase ended (reason: NAV_END_*).
forward OnNavMoveEnd(npcid, reason);
