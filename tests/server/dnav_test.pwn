// A real open.mp server running DiamondNav on the made-up yard (tests/fake_gta.py):
// one NPC walks across the yard round the wall, another drives along the street
// and up the one-way road, a third chases a fourth across the yard round the wall
// on the car navmesh, a fifth turns round by backing up to chase a sixth behind it,
// a truck pulls away as slowly as its handling says, and two searches come back
// through OnNavPathResult. The driver keeps to each road's speed limit: 36 km/h on
// the street (a main road: two lanes each way), 72 km/h up the one-way road.
// tests/server/run.py bakes the yard, compiles this, boots the server and reads
// the "[dnav-test]" lines.

#include <open.mp>
#include <DiamondNav>

#define YX (-2000.0)
#define YY (2000.0)
#define YZ (10.0)
#define TIMEOUT_MS (90000)

new gWalker = INVALID_NPC_ID, gDriver = INVALID_NPC_ID, gCar = INVALID_VEHICLE_ID;
new gChaser = INVALID_NPC_ID, gQuarry = INVALID_NPC_ID, gChaseCar = INVALID_VEHICLE_ID;
new gTurner = INVALID_NPC_ID, gTurnQuarry = INVALID_NPC_ID, gTurnCar = INVALID_VEHICLE_ID;
new Float:gTurnLast[3], bool:gTurnReversed, bool:gTurnCaught, bool:gTurnLeftYard, bool:gTurnStopped;
new gTruck = INVALID_NPC_ID, gTruckCar = INVALID_VEHICLE_ID, gTruckStart, bool:gTruckChecked, bool:gTruckSlow, bool:gTrucked;
new Float:gDriveLast[2], gDriveLastTick, Float:gMainMax, Float:gNorthMax, Float:gCornerMin = 999.0;
new bool:gInfoMain, bool:gInfoStreet, bool:gClassesOk;
new gPathRequest, gRouteRequest;
new bool:gWalked, bool:gDrove, bool:gPathOk, bool:gRouteOk, bool:gThroughWall;
new bool:gCaught, bool:gChaseThroughWall, bool:gChaseStopped;
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
    if (!gCaught) Fail("the chaser did not catch up");
    if (gChaseThroughWall) Fail("the chaser drove through the wall");
    if (!gChaseStopped) Fail("Nav_Stop did not end the chase");
    // open.mp's own NPC component drives forward only: the turner goes round instead.
    #if defined PATCHED_NPCS
        if (!gTurnReversed) Fail("the turner never backed up");
    #endif
    if (!gTurnCaught) Fail("the turner did not reach the NPC behind it");
    if (gTurnLeftYard) Fail("the turner drove off the yard");
    if (!gTurnStopped) Fail("Nav_Stop did not end the turner's chase");
    printf("[dnav-test] driver: up to %.1f m/s on the main road, %.1f round the corner, up to %.1f up the street",
        gMainMax, gCornerMin, gNorthMax);
    if (gMainMax > 10.8) Fail("the driver broke the main road's 36 km/h limit");
    if (gMainMax < 8.0) Fail("the driver never got up to the main road's limit");
    if (gNorthMax < 12.0) Fail("the driver did not speed up on the street (72 km/h)");
    if (gCornerMin >= gNorthMax) Fail("the driver did not slow for the corner");
    if (!gInfoMain) Fail("Nav_GetDriveInfo never said main road at 36 km/h");
    if (!gInfoStreet) Fail("Nav_GetDriveInfo never said street up the one-way road");
    if (!gClassesOk) Fail("the route's points do not carry main road, then street");
    if (!gTruckSlow) Fail("the truck pulled away faster than its handling allows");
    if (!gTrucked) Fail("the truck did not arrive");
    print(gFailures ? ("[dnav-test] FAILED") : ("[dnav-test] PASS"));
    SendRconCommand("exit");
}

main() {}

public OnGameModeInit()
{
    printf("[dnav-test] %d navmesh tiles, %d car tiles, %d road nodes", Nav_GetTileCount(),
        Nav_GetTileCount(NAV_MESH_CAR), Nav_GetRoadNodeCount());
    if (!Nav_GetTileCount()) Fail("no navmesh loaded");
    if (!Nav_GetTileCount(NAV_MESH_CAR)) Fail("no car navmesh loaded");
    if (!Nav_GetRoadNodeCount()) Fail("no road graph loaded");

    // vehicles.dnv, from the yard's made-up handling.cfg: a Sultan (its DFF in gta3.img),
    // a Linerunner, a Sandking, a bike.
    new Float:top, Float:accel, Float:brake, Float:grip, Float:radius;
    if (Nav_GetVehicleModelCount() != 4) Fail("vehicles.dnv did not give 4 models");
    Nav_GetVehicleHandling(560, top, accel, brake, grip, radius);
    if (floatabs(top - 168.8) > 1.0 || floatabs(accel - 11.52) > 0.05) Fail("the Sultan's handling is not vehicles.dnv's");
    if (floatabs(Nav_GetVehicleInfo(560, NAV_VINFO_WHEELBASE) - 2.6) > 0.01
        || floatabs(Nav_GetVehicleInfo(560, NAV_VINFO_LENGTH) - 4.6) > 0.01) Fail("the Sultan's size is not its DFF's");
    new Float:z100 = Nav_GetVehicleInfo(560, NAV_VINFO_ZERO_100);
    if (z100 < 2.5 || z100 > 3.5) Fail("the Sultan's 0-100 km/h is not about 2.9 s");
    if (Nav_GetVehicleAccel(560, 120.0) >= Nav_GetVehicleAccel(560, 20.0)) Fail("the Sultan pulls as hard at 120 km/h as at 20");
    if (Nav_GetVehicleAccel(560, 20.0, true) >= Nav_GetVehicleAccel(560, 20.0)) Fail("the Sultan pulls as hard on dirt");
    if (Nav_GetVehicleInfo(403, NAV_VINFO_YAW) >= Nav_GetVehicleInfo(560, NAV_VINFO_YAW))
        Fail("the Linerunner swings round as fast as the Sultan");
    if (Nav_GetVehicleInfo(495, NAV_VINFO_GRIP_OFFROAD) < Nav_GetVehicleInfo(495, NAV_VINFO_GRIP) * 0.95)
        Fail("the Sandking's off-road ability is lost");
    // How high each sits: the Sultan from its DFF's wheels and springs (0.76 m), the
    // Linerunner (no DFF in the yard) from open.mp's model table, a good deal higher.
    if (floatabs(Nav_GetVehicleInfo(560, NAV_VINFO_RIDE) - 0.76) > 0.01) Fail("the Sultan's ride height is not its DFF's");
    if (Nav_GetVehicleInfo(403, NAV_VINFO_RIDE) < Nav_GetVehicleInfo(560, NAV_VINFO_RIDE) + 0.6)
        Fail("the Linerunner does not sit higher than the Sultan");
    Nav_GetVehicleHandling(403, top, accel, brake, grip, radius);
    if (floatabs(top - 109.5) > 1.0) Fail("the Linerunner's handling is not vehicles.dnv's");
    if (!Nav_SetRoadSpeed(NAV_CLASS_MAIN, 36.0) || !Nav_SetRoadSpeed(NAV_CLASS_STREET, 72.0)) Fail("Nav_SetRoadSpeed refused");
    if (floatabs(Nav_GetRoadSpeed(NAV_CLASS_MAIN) - 36.0) > 0.01) Fail("Nav_GetRoadSpeed is not what was set");
    if (Nav_SetRoadSpeed(99, 50.0) || Nav_SetDriveStyle(0, 99)) Fail("a bad class or style was taken");

    // The wall stands between x = YX - 20 and YX + 20 at y = YY; 15 m north is open.
    if (Nav_IsLineClear(YX - 20.0, YY, YZ, YX + 20.0, YY, YZ, NAV_MESH_CAR)) Fail("a car's line through the wall is clear");
    if (!Nav_IsLineClear(YX - 20.0, YY + 15.0, YZ, YX + 10.0, YY + 15.0, YZ, NAV_MESH_CAR))
        Fail("a car's line across open ground is blocked");
    if (Nav_IsLineClear(YX - 20.0, YY, YZ, YX + 20.0, YY, YZ)) Fail("a ped's line through the wall is clear");

    gPathRequest = Nav_RequestPath(YX - 25.0, YY, YZ, YX + 25.0, YY, YZ);
    gRouteRequest = Nav_RequestRoute(YX - 100.0, YY - 40.0, YZ, YX + 100.0, YY + 60.0, YZ);

    new Float:x, Float:y, Float:z;
    if (!Nav_GetNearestRoadPoint(YX + 3.0, YY - 38.0, YZ, x, y, z)) Fail("no road point near the street");

    gWalker = NPC_Create("dnav_walker");
    gDriver = NPC_Create("dnav_driver");
    NPC_Spawn(gWalker);
    NPC_Spawn(gDriver);
    gCar = CreateVehicle(560, YX - 100.0, YY - 42.5, YZ + 1.0, 270.0, 0, 0, -1);
    gChaser = NPC_Create("dnav_chaser");
    gQuarry = NPC_Create("dnav_quarry");
    NPC_Spawn(gChaser);
    NPC_Spawn(gQuarry);
    gChaseCar = CreateVehicle(560, YX - 20.0, YY + 2.0, YZ + 1.0, 270.0, 0, 0, -1);
    // The turner faces east in the open north-west corner; its quarry is 16 m behind it.
    gTurner = NPC_Create("dnav_turner");
    gTurnQuarry = NPC_Create("dnav_turnq");
    NPC_Spawn(gTurner);
    NPC_Spawn(gTurnQuarry);
    gTurnCar = CreateVehicle(560, YX - 8.0, YY + 22.0, YZ + 1.0, 270.0, 0, 0, -1);
    gTurnLast[0] = YX - 8.0, gTurnLast[1] = YY + 22.0;
    // The truck: along the main road in pursuit (no limits), so only its handling holds it back.
    gTruck = NPC_Create("dnav_truck");
    NPC_Spawn(gTruck);
    gTruckCar = CreateVehicle(403, YX - 60.0, YY - 42.5, YZ + 1.5, 270.0, 0, 0, -1);
    gDriveLast[0] = YX - 100.0, gDriveLast[1] = YY - 42.5, gDriveLastTick = GetTickCount();
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
    if (!Nav_DriveTo(gDriver, YX + 100.0, YY + 60.0, YZ + 1.0)) Fail("Nav_DriveTo refused");
    gDriveLast[0] = YX - 100.0, gDriveLast[1] = YY - 42.5, gDriveLastTick = GetTickCount();
    NPC_PutInVehicle(gTruck, gTruckCar, 0);
    if (!Nav_SetDriveStyle(gTruck, NAV_STYLE_PURSUIT) || Nav_GetDriveStyle(gTruck) != NAV_STYLE_PURSUIT)
        Fail("Nav_SetDriveStyle did not take");
    if (!Nav_DriveTo(gTruck, YX + 60.0, YY - 42.5, YZ + 1.5)) Fail("Nav_DriveTo refused the truck");   // at its own height
    Nav_SetDriveStyle(gWalker, NAV_STYLE_FAST);
    if (!Nav_SetDriveStyle(gWalker, NAV_STYLE_DEFAULT) || Nav_GetDriveStyle(gWalker) != NAV_STYLE_DEFAULT)
        Fail("NAV_STYLE_DEFAULT did not clear a style");
    gTruckStart = GetTickCount();
    if (Nav_DriveTo(gWalker, YX, YY, YZ)) Fail("Nav_DriveTo took an NPC that is not driving");
    // That refusal ended nothing: the walker's own walk goes on.

    // The quarry waits east of the wall; the chaser starts west of it.
    NPC_SetPos(gQuarry, YX + 20.0, YY - 2.0, YZ + 1.0);
    NPC_PutInVehicle(gChaser, gChaseCar, 0);
    if (!Nav_Chase(gChaser, gQuarry, 1.2)) Fail("Nav_Chase refused");

    NPC_SetPos(gTurnQuarry, YX - 24.0, YY + 22.0, YZ + 1.0);
    NPC_PutInVehicle(gTurner, gTurnCar, 0);
    if (!Nav_Chase(gTurner, gTurnQuarry, 1.0)) Fail("Nav_Chase refused the turner");
}

forward Watch();
public Watch()
{
    new Float:x, Float:y, Float:z;
    WatchDriver();
    if (!gTruckChecked && GetTickCount() - gTruckStart >= 3000) {
        // How far its own acceleration (gears, engine, air, tyres) takes it in 3 s at
        // full throttle, as pursuit drives; a Sultan's would take it a good deal further.
        gTruckChecked = true;
        GetVehiclePos(gTruckCar, x, y, z);
        new Float:gone = floatabs(x - (YX - 60.0)), Float:v, Float:d, Float:sv, Float:sd;
        for (new i = 0; i < 60; i++) {
            v += Nav_GetVehicleAccel(403, v * 3.6) * 0.05, d += v * 0.05;
            sv += Nav_GetVehicleAccel(560, sv * 3.6) * 0.05, sd += sv * 0.05;
        }
        printf("[dnav-test] the truck went %.1f m in its first 3 s (its handling: %.1f m, a Sultan's: %.1f m)", gone, d, sd);
        gTruckSlow = gone > d * 0.4 && gone < d * 1.15 && gone < sd * 0.8;
    }
    NPC_GetPos(gWalker, x, y, z);
    // The wall: 0.5 m thick along y at x = YX, 20 m long.
    if (floatabs(x - YX) < 0.25 && floatabs(y - YY) < 10.0) gThroughWall = true;

    if (gTurnCar != INVALID_VEHICLE_ID && !gTurnCaught) {
        new Float:a;
        GetVehiclePos(gTurnCar, x, y, z);
        GetVehicleZAngle(gTurnCar, a);
        // Moving against the way it faces: backing up.
        new Float:dx = x - gTurnLast[0], Float:dy = y - gTurnLast[1];
        if (dx * -floatsin(a, degrees) + dy * floatcos(a, degrees) < -0.2) gTurnReversed = true;
        gTurnLast[0] = x, gTurnLast[1] = y;
        if (floatabs(x - YX) > 29.0 || floatabs(y - YY) > 29.0) gTurnLeftYard = true;
        if (VectorSize(x - (YX - 24.0), y - (YY + 22.0), 0.0) < 7.0) {
            printf("[dnav-test] turner reached its quarry: car at %.1f %.1f %.1f, backed up: %d", x, y, z, gTurnReversed);
            gTurnCaught = true;
            Nav_Stop(gTurner);
        }
    }

    if (gChaseCar == INVALID_VEHICLE_ID || gCaught) return;
    GetVehiclePos(gChaseCar, x, y, z);
    if (floatabs(x - YX) < 0.5 && floatabs(y - YY) < 10.0) gChaseThroughWall = true;
    if (VectorSize(x - (YX + 20.0), y - (YY - 2.0), 0.0) < 9.0)
    {
        printf("[dnav-test] chaser caught up: car at %.1f %.1f %.1f", x, y, z);
        gCaught = true;
        Nav_Stop(gChaser);
    }
}

// The driver's speed from where its car is (not what DiamondNav says), and where.
WatchDriver()
{
    if (gDrove) return;
    new Float:x, Float:y, Float:z, now = GetTickCount();
    GetVehiclePos(gCar, x, y, z);
    // Over 0.6 s or more: NPCs move in steps (each NPC update), so a shorter window
    // catches three steps one time and five the next.
    new Float:dt = float(now - gDriveLastTick) / 1000.0;
    if (dt < 0.6) return;
    new Float:speed = VectorSize(x - gDriveLast[0], y - gDriveLast[1], 0.0) / dt;
    gDriveLast[0] = x, gDriveLast[1] = y, gDriveLastTick = now;

    new Float:kmh, Float:want, cls, Float:limit, bool:back;
    new bool:info = Nav_GetDriveInfo(gDriver, kmh, want, cls, limit, back);
    if (y < YY - 35.0 && x > YX - 90.0 && x < YX + 80.0) {
        if (speed > gMainMax) gMainMax = speed;
        if (info && cls == NAV_CLASS_MAIN && floatabs(limit - 36.0) < 0.5) gInfoMain = true;
    } else if (x > YX + 95.0 && y > YY - 20.0 && y < YY + 45.0) {
        if (speed > gNorthMax) gNorthMax = speed;
        if (info && cls == NAV_CLASS_STREET) gInfoStreet = true;
    }
    if (VectorSize(x - (YX + 100.0), y - (YY - 40.0), 0.0) < 8.0 && speed < gCornerMin) gCornerMin = speed;
}

forward TimedOut();
public TimedOut()
{
    if (gDone < 5) Fail("timed out");
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
        gClassesOk = points > 2 && Nav_GetPathPointClass(requestid, 0) == NAV_CLASS_MAIN
            && Nav_GetPathPointClass(requestid, points - 1) == NAV_CLASS_STREET && Nav_GetPathPointClass(requestid, points) == -1;
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
    } else if (npcid == gChaser) {
        printf("[dnav-test] chase ended: reason %d", reason);
        gChaseStopped = gCaught && reason == NAV_END_STOPPED;
        gDone++;
    } else if (npcid == gTruck) {
        GetVehiclePos(gTruckCar, x, y, z);
        printf("[dnav-test] truck ended: reason %d, car at %.1f %.1f %.1f", reason, x, y, z);
        gTrucked = reason == NAV_END_ARRIVED && VectorSize(x - (YX + 60.0), y - (YY - 42.5), 0.0) < 4.0;
        // On the road at its own ride height: not sunk into it as at a car's.
        if (floatabs(z - (YZ + Nav_GetVehicleInfo(403, NAV_VINFO_RIDE))) > 0.25)
            Fail("the truck does not sit on the road at its ride height");
        gDone++;
    } else if (npcid == gTurner) {
        printf("[dnav-test] turner's chase ended: reason %d", reason);
        gTurnStopped = gTurnCaught && reason == NAV_END_STOPPED;
        gDone++;
    }
    if (gDone == 5) Finish();
    return 1;
}
