// Chases on roads alone, as a server with no car navmesh has them (tests/server/run.py
// writes only roads.dnr and vehicles.dnv for this one):
// - a bridge 15 m over a road, joined to it only by a ramp far to the east: a chaser
//   on the bridge goes off its side after an NPC standing on the road below, and
//   lands, instead of driving round by the ramp;
// - a long road: a chaser keeps up with an NPC driving away from it, instead of
//   braking to a stop once it gets close;
// - a second long road: two chasers after one standing NPC, started one behind the
//   other, come up either side of it instead of stopping in each other;
// - a third: a driver swerves round a car parked in its lane instead of through it;
// - a fourth: another goes round a truck parked across the road, by the truck's own
//   box (long and turned across the way), not a circle round its middle.

#include <open.mp>
#include <DiamondNav>

#define YX (-2000.0)
#define YY (2000.0)
#define YZ (10.0)
#define BRIDGE_Z (YZ + 15.0)
#define FLEE_Y (YY - 200.0)
#define PAIR_Y (YY - 400.0)
#define PARK_Y (YY - 600.0)
#define TRUCK_Y (YY - 800.0)
#define TIMEOUT_MS (60000)

new gDropper = INVALID_NPC_ID, gDropQuarry = INVALID_NPC_ID, gDropCar = INVALID_VEHICLE_ID;
new gFollower = INVALID_NPC_ID, gRunner = INVALID_NPC_ID, gFollowCar = INVALID_VEHICLE_ID, gRunCar = INVALID_VEHICLE_ID;
new gPairA = INVALID_NPC_ID, gPairB = INVALID_NPC_ID, gPairQuarry = INVALID_NPC_ID;
new gPairCarA = INVALID_VEHICLE_ID, gPairCarB = INVALID_VEHICLE_ID;
new bool:gWentEdge, bool:gWentAirborne, bool:gLanded, bool:gWentRound, bool:gDropCaught, Float:gDropLowest = 999.0;
new bool:gAlongside, bool:gCaughtUp, bool:gFollowStopped, Float:gFollowSlowest = 999.0, gCaughtAt, bool:gFollowDone;
new bool:gPairDone, Float:gPairGap;
new gSwerver = INVALID_NPC_ID, gSwerveCar = INVALID_VEHICLE_ID, gParked = INVALID_VEHICLE_ID;
new bool:gSwerved, Float:gSwerveNearest = 999.0;
new gRounder = INVALID_NPC_ID, gRoundCar = INVALID_VEHICLE_ID, gTruck = INVALID_VEHICLE_ID;
new bool:gRounded, bool:gHitTruck, Float:gTruckHalf[2];
new gFailures, bool:gFinished;

Fail(const what[])
{
    printf("[dnav-test] FAIL: %s", what);
    gFailures++;
}

Finish()
{
    if (gFinished) return;
    gFinished = true;
    printf("[dnav-test] drop: edge %d, airborne %d, landed %d, lowest car z %.1f, went round %d, caught %d",
        gWentEdge, gWentAirborne, gLanded, gDropLowest, gWentRound, gDropCaught);
    if (!gWentEdge) Fail("the chaser on the bridge never ran at its edge");
    if (!gWentAirborne) Fail("the chaser never went off the bridge");
    if (gWentRound) Fail("the chaser drove round by the ramp");
    if (!gLanded) Fail("the chaser did not land on the road below");
    if (!gDropCaught) Fail("the chaser did not reach the NPC below the bridge");
    printf("[dnav-test] follow: caught up %d, slowest while beside %.1f km/h, stopped %d", gCaughtUp, gFollowSlowest,
        gFollowStopped);
    if (!gCaughtUp) Fail("the chaser never caught up with the NPC driving away");
    if (gFollowStopped || gFollowSlowest < 30.0) Fail("the chaser slowed right down beside a moving target");
    if (!gAlongside) Fail("the chaser never pulled up alongside the moving target");
    printf("[dnav-test] pair: settled %d, %.1f m apart", gPairDone, gPairGap);
    if (!gPairDone) Fail("the two chasers did not both stop beside their NPC");
    else if (gPairGap < 3.0) Fail("the two chasers stopped in each other");
    printf("[dnav-test] swerve: arrived %d, nearest to the parked car %.1f m", gSwerved, gSwerveNearest);
    if (!gSwerved) Fail("the driver did not get past the parked car");
    if (gSwerveNearest < 2.4) Fail("the driver drove into the parked car");
    printf("[dnav-test] truck: arrived %d, touched it %d (its box %.1f x %.1f m)", gRounded, gHitTruck,
        gTruckHalf[0] * 2.0, gTruckHalf[1] * 2.0);
    if (!gRounded) Fail("the driver did not get round the truck");
    if (gHitTruck) Fail("the driver drove into the truck parked across the road");
    print(gFailures ? ("[dnav-test] FAILED") : ("[dnav-test] PASS"));
    SendRconCommand("exit");
}

main() {}

public OnGameModeInit()
{
    printf("[dnav-test] chase roads: %d navmesh tiles, %d car tiles, %d road nodes", Nav_GetTileCount(),
        Nav_GetTileCount(NAV_MESH_CAR), Nav_GetRoadNodeCount());
    if (Nav_GetTileCount(NAV_MESH_CAR)) Fail("a car navmesh is loaded: this test is for roads alone");
    if (!Nav_GetRoadNodeCount()) Fail("no road graph loaded");
    new Float:ride = Nav_GetVehicleInfo(560, NAV_VINFO_RIDE);

    gDropper = NPC_Create("dnav_dropper");
    gDropQuarry = NPC_Create("dnav_dropq");
    NPC_Spawn(gDropper);
    NPC_Spawn(gDropQuarry);
    gDropCar = CreateVehicle(560, YX - 40.0, YY, BRIDGE_Z + ride, 270.0, 0, 0, -1);

    gFollower = NPC_Create("dnav_follower");
    gRunner = NPC_Create("dnav_runner");
    NPC_Spawn(gFollower);
    NPC_Spawn(gRunner);
    gFollowCar = CreateVehicle(560, YX - 280.0, FLEE_Y, YZ + ride, 270.0, 0, 0, -1);
    gRunCar = CreateVehicle(560, YX - 230.0, FLEE_Y, YZ + ride, 270.0, 0, 0, -1);

    gPairA = NPC_Create("dnav_paira");
    gPairB = NPC_Create("dnav_pairb");
    gPairQuarry = NPC_Create("dnav_pairq");
    NPC_Spawn(gPairA);
    NPC_Spawn(gPairB);
    NPC_Spawn(gPairQuarry);
    gPairCarA = CreateVehicle(560, YX - 50.0, PAIR_Y, YZ + ride, 270.0, 0, 0, -1);
    gPairCarB = CreateVehicle(560, YX - 62.0, PAIR_Y, YZ + ride, 270.0, 0, 0, -1);

    // Driving east keeps right: the lane is 2.5 m south of the centre line.
    gSwerver = NPC_Create("dnav_swerver");
    NPC_Spawn(gSwerver);
    gSwerveCar = CreateVehicle(560, YX - 120.0, PARK_Y - 2.5, YZ + ride, 270.0, 0, 0, -1);
    gParked = CreateVehicle(560, YX, PARK_Y - 2.5, YZ + ride, 270.0, 0, 0, -1);

    // A Linerunner across the road, facing north, in the eastbound lane.
    gRounder = NPC_Create("dnav_rounder");
    NPC_Spawn(gRounder);
    gRoundCar = CreateVehicle(560, YX - 120.0, TRUCK_Y - 2.5, YZ + ride, 270.0, 0, 0, -1);
    gTruck = CreateVehicle(403, YX, TRUCK_Y - 2.5, YZ + 1.5, 0.0, 0, 0, -1);
    gTruckHalf[0] = Nav_GetVehicleInfo(403, NAV_VINFO_WIDTH) * 0.5;
    gTruckHalf[1] = Nav_GetVehicleInfo(403, NAV_VINFO_LENGTH) * 0.5;

    SetTimer("StartMoving", 500, false);
    SetTimer("Watch", 200, true);
    SetTimer("TimedOut", TIMEOUT_MS, false);
    return 1;
}

forward StartMoving();
public StartMoving()
{
    new Float:ride = Nav_GetVehicleInfo(560, NAV_VINFO_RIDE);
    // Below the bridge, 30 m north of it and 40 m east of the chaser.
    NPC_SetPos(gDropQuarry, YX, YY + 30.0, YZ + 1.0);
    NPC_PutInVehicle(gDropper, gDropCar, 0);
    if (!Nav_Chase(gDropper, gDropQuarry)) Fail("Nav_Chase refused the dropper");

    // 50 m ahead, driving east at 15 m/s.
    NPC_PutInVehicle(gRunner, gRunCar, 0);
    NPC_Move(gRunner, YX + 700.0, FLEE_Y, YZ + ride, NPC_MOVE_TYPE_DRIVE, 1.5);
    NPC_PutInVehicle(gFollower, gFollowCar, 0);
    if (!Nav_Chase(gFollower, gRunner)) Fail("Nav_Chase refused the follower");

    NPC_SetPos(gPairQuarry, YX, PAIR_Y, YZ + 1.0);
    NPC_PutInVehicle(gPairA, gPairCarA, 0);
    NPC_PutInVehicle(gPairB, gPairCarB, 0);
    if (!Nav_Chase(gPairA, gPairQuarry) || !Nav_Chase(gPairB, gPairQuarry)) Fail("Nav_Chase refused the pair");

    NPC_PutInVehicle(gSwerver, gSwerveCar, 0);
    if (!Nav_DriveTo(gSwerver, YX + 120.0, PARK_Y, YZ + 1.0)) Fail("Nav_DriveTo refused the swerver");

    NPC_PutInVehicle(gRounder, gRoundCar, 0);
    if (!Nav_DriveTo(gRounder, YX + 120.0, TRUCK_Y, YZ + 1.0)) Fail("Nav_DriveTo refused the rounder");
}

public OnNavMoveEnd(npcid, reason)
{
    if (npcid == gSwerver) {
        new Float:x, Float:y, Float:z;
        GetVehiclePos(gSwerveCar, x, y, z);
        printf("[dnav-test] swerver ended: reason %d, car at %.1f %.1f", reason, x, y);
        gSwerved = reason == NAV_END_ARRIVED && x > YX + 100.0;
    } else if (npcid == gRounder) {
        new Float:x, Float:y, Float:z;
        GetVehiclePos(gRoundCar, x, y, z);
        printf("[dnav-test] rounder ended: reason %d, car at %.1f %.1f", reason, x, y);
        gRounded = reason == NAV_END_ARRIVED && x > YX + 100.0;
    }
    return 1;
}

forward Watch();
public Watch()
{
    new Float:x, Float:y, Float:z, Float:d, mode;
    if (!gDropCaught) {
        GetVehiclePos(gDropCar, x, y, z);
        if (z < gDropLowest) gDropLowest = z;
        if (x > YX + 120.0) gWentRound = true;
        if (Nav_GetChaseInfo(gDropper, d, mode)) {
            if (mode == NAV_CHASE_EDGE) gWentEdge = true;
            if (mode == NAV_CHASE_AIRBORNE) gWentAirborne = true;
        }
        if (z < YZ + 3.0 && y > YY + 15.0) gLanded = true;
        if (gLanded && floatabs(z - (YZ + Nav_GetVehicleInfo(560, NAV_VINFO_RIDE))) < 0.3
        && VectorSize(x - YX, y - (YY + 30.0), 0.0) < 10.0) {
            printf("[dnav-test] dropper reached its quarry: car at %.1f %.1f %.1f", x, y, z);
            gDropCaught = true;
            Nav_Stop(gDropper);
        }
    }

    if (!gFollowDone) {
        new Float:rx, Float:ry, Float:rz;
        GetVehiclePos(gFollowCar, x, y, z);
        GetVehiclePos(gRunCar, rx, ry, rz);
        new Float:gap = VectorSize(x - rx, y - ry, 0.0);
        if (!gCaughtUp && gap < 15.0) {
            gCaughtUp = true;
            gCaughtAt = GetTickCount();
            printf("[dnav-test] follower caught up at %.1f %.1f", x, y);
        }
        if (gCaughtUp) {
            // Alongside it on its left (north, as it drives east), not in its back.
            if (GetTickCount() - gCaughtAt > 3000 && y > ry + 1.0) gAlongside = true;
            new Float:kmh, Float:want, cls, Float:limit, bool:back;
            if (Nav_GetDriveInfo(gFollower, kmh, want, cls, limit, back) && kmh < gFollowSlowest) gFollowSlowest = kmh;
            if (Nav_GetChaseInfo(gFollower, d, mode) && mode == NAV_CHASE_BESIDE) gFollowStopped = true;
            // Five seconds alongside it is enough; then both stop.
            if (GetTickCount() - gCaughtAt > 5000) {
                gFollowDone = true;
                Nav_Stop(gFollower);
                NPC_StopMove(gRunner);
            }
        }
    }

    if (!gPairDone) {
        new ma, mb, Float:da, Float:db;
        if (Nav_GetChaseInfo(gPairA, da, ma) && Nav_GetChaseInfo(gPairB, db, mb) && ma == NAV_CHASE_BESIDE
            && mb == NAV_CHASE_BESIDE) {
            new Float:bx, Float:by, Float:bz, Float:kmh, Float:want, cls, Float:limit, bool:back, Float:kmh2;
            Nav_GetDriveInfo(gPairA, kmh, want, cls, limit, back);
            Nav_GetDriveInfo(gPairB, kmh2, want, cls, limit, back);
            if (kmh < 1.0 && kmh2 < 1.0) {
                GetVehiclePos(gPairCarA, x, y, z);
                GetVehiclePos(gPairCarB, bx, by, bz);
                gPairGap = VectorSize(x - bx, y - by, 0.0);
                printf("[dnav-test] pair stopped at %.1f %.1f and %.1f %.1f", x, y, bx, by);
                gPairDone = true;
                Nav_Stop(gPairA);
                Nav_Stop(gPairB);
            }
        }
    }

    if (gSwerveCar != INVALID_VEHICLE_ID && !gSwerved) {
        new Float:px, Float:py, Float:pz;
        GetVehiclePos(gSwerveCar, x, y, z);
        GetVehiclePos(gParked, px, py, pz);
        d = VectorSize(x - px, y - py, 0.0);
        if (d < gSwerveNearest) gSwerveNearest = d;
    }

    if (gRoundCar != INVALID_VEHICLE_ID && !gRounded) {
        // The truck faces north: its length runs along y. The Sultan, heading east
        // give or take its swerve, is about 4.6 m along x and 2 m across.
        new Float:tx, Float:ty, Float:tz;
        GetVehiclePos(gRoundCar, x, y, z);
        GetVehiclePos(gTruck, tx, ty, tz);
        if (floatabs(x - tx) < gTruckHalf[0] + 2.1 && floatabs(y - ty) < gTruckHalf[1] + 0.9) {
            if (!gHitTruck) printf("[dnav-test] rounder touched the truck: car at %.1f %.1f", x, y);
            gHitTruck = true;
        }
    }

    if (gDropCaught && gFollowDone && gPairDone && gSwerved && gRounded) Finish();
}

forward TimedOut();
public TimedOut()
{
    Fail("timed out");
    Finish();
}
