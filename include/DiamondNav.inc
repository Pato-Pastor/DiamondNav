// DiamondNav: walking paths and driving routes for open.mp NPCs.
// https://github.com/payrle-dev/DiamondNav  (MPL-2.0)
//
// The DiamondNav component (components/DiamondNav.so or .dll) loads, from the
// server's navmesh/ folder:
//   *.nav       where peds can walk, and where a car fits off the road (navbake
//               --agent ped or car; dnav.py world bakes both)
//   roads.dnr   where cars can drive (built with tools/dnav.py roads)
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

#define DIAMONDNAV_INCLUDE_VERSION      (10100)     // 1.1.0

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

// Drives an NPC's car to a point along the roads, in its lane, slowing for
// corners. The NPC must be the driver. speed is NPC_MOVE_TYPE_DRIVE's (1.0 is
// about 10 m/s) on the straight; carZ is how far the car's centre sits above the
// road (about 1.0 for a car; more for a truck or bus).
native bool:Nav_DriveTo(npcid, Float:x, Float:y, Float:z, Float:speed = 1.0, Float:carZ = 1.0, flags = 0);

// Chases a player (or another NPC) with an NPC's car until Nav_Stop. Far off, it
// follows the roads while the target is on them, and the car navmesh once the target
// leaves them; close and with nothing in the way, it drives straight at where the
// target is going, and it stops beside a target that stops. It re-plans as the
// target moves, without pausing. The NPC must be the driver. Ends with
// NAV_END_LOST when the target leaves or changes world. What the NPCs do in the
// chase (shoot, ram, give up) is the script's.
native bool:Nav_Chase(npcid, targetid, Float:speed = 1.4, Float:carZ = 1.0);

// Stops a Nav_WalkTo, Nav_DriveTo or Nav_Chase where the NPC is (OnNavMoveEnd:
// NAV_END_STOPPED).
native bool:Nav_Stop(npcid);

// Whether a Nav_WalkTo, Nav_DriveTo or Nav_Chase is under way (or searching).
native bool:Nav_IsMoving(npcid);

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
