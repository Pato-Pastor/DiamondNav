// A real open.mp server running DiamondNav on the made-up yard (tests/fake_gta.py):
// one NPC walks across the yard round the wall, another drives along the street
// and up the one-way road, and two searches come back through OnNavPathResult.
// tests/server/run.py bakes the yard, compiles this, boots the server and reads
// the "[dnav-test]" lines.

#include <open.mp>
#include <DiamondNav>

#define YX (-2000.0)
#define YY (2000.0)
#define YZ (10.0)
#define TIMEOUT_MS (90000)

new gWalker = INVALID_NPC_ID, gDriver = INVALID_NPC_ID, gCar = INVALID_VEHICLE_ID;
new gPathRequest, gRouteRequest;
new bool:gWalked, bool:gDrove, bool:gPathOk, bool:gRouteOk, bool:gThroughWall;
new gFailures, gDone;

Fail(const what[])
{
    printf("[dnav-test] FAIL: %s", what);
    gFailures++;
}

Finish()
{
    if (!gWalked) Fail("the walker did not arrive");
    if (!gDrove) Fail("the driver did not arrive");
    if (!gPathOk) Fail("no walking path came back");
    if (!gRouteOk) Fail("no driving route came back");
    if (gThroughWall) Fail("the walker went through the wall");
    print(gFailures ? ("[dnav-test] FAILED") : ("[dnav-test] PASS"));
    SendRconCommand("exit");
}

main() {}

public OnGameModeInit()
{
    printf("[dnav-test] %d navmesh tiles, %d road nodes", Nav_GetTileCount(), Nav_GetRoadNodeCount());
    if (!Nav_GetTileCount()) Fail("no navmesh loaded");
    if (!Nav_GetRoadNodeCount()) Fail("no road graph loaded");

    gPathRequest = Nav_RequestPath(YX - 25.0, YY, YZ, YX + 25.0, YY, YZ);
    gRouteRequest = Nav_RequestRoute(YX - 100.0, YY - 40.0, YZ, YX + 100.0, YY + 60.0, YZ);

    new Float:x, Float:y, Float:z;
    if (!Nav_GetNearestRoadPoint(YX + 3.0, YY - 38.0, YZ, x, y, z)) Fail("no road point near the street");

    gWalker = NPC_Create("dnav_walker");
    gDriver = NPC_Create("dnav_driver");
    NPC_Spawn(gWalker);
    NPC_Spawn(gDriver);
    gCar = CreateVehicle(560, YX - 100.0, YY - 42.5, YZ + 1.0, 270.0, 0, 0, -1);
    SetTimer("StartMoving", 500, false);
    SetTimer("Watch", 200, true);
    SetTimer("TimedOut", TIMEOUT_MS, false);
    return 1;
}

forward StartMoving();
public StartMoving()
{
    NPC_SetPos(gWalker, YX - 25.0, YY, YZ + 1.0);
    if (!Nav_WalkTo(gWalker, YX + 25.0, YY, YZ + 1.0, NPC_MOVE_TYPE_JOG)) Fail("Nav_WalkTo refused");
    NPC_PutInVehicle(gDriver, gCar, 0);
    if (!Nav_DriveTo(gDriver, YX + 100.0, YY + 60.0, YZ + 1.0, 1.5)) Fail("Nav_DriveTo refused");
    if (Nav_DriveTo(gWalker, YX, YY, YZ)) Fail("Nav_DriveTo took an NPC that is not driving");
    // That refusal ended nothing: the walker's own walk goes on.
}

forward Watch();
public Watch()
{
    new Float:x, Float:y, Float:z;
    NPC_GetPos(gWalker, x, y, z);
    // The wall: 0.5 m thick along y at x = YX, 20 m long.
    if (floatabs(x - YX) < 0.25 && floatabs(y - YY) < 10.0) gThroughWall = true;
}

forward TimedOut();
public TimedOut()
{
    if (gDone < 2) Fail("timed out");
    Finish();
}

public OnNavPathResult(requestid, status, points)
{
    new Float:x, Float:y, Float:z;
    if (requestid == gPathRequest) {
        printf("[dnav-test] path: status %d, %d points", status, points);
        gPathOk = status == NAV_OK && points > 2 && Nav_GetPathPoint(requestid, points - 1, x, y, z)
            && floatabs(x - (YX + 25.0)) < 1.0;
    } else if (requestid == gRouteRequest) {
        printf("[dnav-test] route: status %d, %d points", status, points);
        gRouteOk = status == NAV_OK && points > 2 && Nav_GetPathPoint(requestid, 0, x, y, z)
            && y < YY - 40.0 && Nav_GetPathPointSpeed(requestid, points - 1) < 1.0;
    }
    return 1;
}

public OnNavMoveEnd(npcid, reason)
{
    new Float:x, Float:y, Float:z;
    if (npcid == gWalker) {
        NPC_GetPos(npcid, x, y, z);
        printf("[dnav-test] walker ended: reason %d at %.1f %.1f %.1f", reason, x, y, z);
        gWalked = reason == NAV_END_ARRIVED && VectorSize(x - (YX + 25.0), y - YY, 0.0) < 2.0;
        gDone++;
    } else if (npcid == gDriver) {
        GetVehiclePos(gCar, x, y, z);
        printf("[dnav-test] driver ended: reason %d, car at %.1f %.1f %.1f", reason, x, y, z);
        gDrove = reason == NAV_END_ARRIVED && VectorSize(x - (YX + 100.0), y - (YY + 60.0), 0.0) < 4.0;
        gDone++;
    }
    if (gDone == 2) Finish();
    return 1;
}
