// Chases and drives with the world's collision (tests/server/run.py writes world.col,
// roads.dnr and vehicles.dnv, no navmesh):
// - a wall 4 m tall between a chaser and the NPC it chases: it drives round by the
//   road, never through the wall;
// - a bridge 15 m over a road with a railing along its edge but for one gap, 8 m along
//   from where the straight line to the NPC below meets it: the chaser goes off
//   through the gap, not through the railing, and lands on the ground below;
// - a concrete block in a road's lane: a driver swerves round it.

#include <open.mp>
#include <DiamondNav>

#define YX (-2000.0)
#define YY (2000.0)
#define YZ (10.0)
#define WALL_Y (YY + 600.0)
#define DECK_Y (YY + 900.0)
#define DECK_Z (25.0)
#define GAP_X (YX - 24.0)
#define BLOCK_Y (YY + 1200.0)
#define TIMEOUT_MS (60000)

new gWaller = INVALID_NPC_ID, gWallQuarry = INVALID_NPC_ID, gWallCar = INVALID_VEHICLE_ID;
new gJumper = INVALID_NPC_ID, gJumpQuarry = INVALID_NPC_ID, gJumpCar = INVALID_VEHICLE_ID;
new gDodger = INVALID_NPC_ID, gDodgeCar = INVALID_VEHICLE_ID;
new bool:gWallDone, bool:gThroughWall, Float:gWallNearest = 999.0;
new bool:gJumpDone, bool:gThroughRail, bool:gWentAirborne, Float:gCrossX = 999.0, Float:gJumpLowest = 999.0, bool:gUnderGround;
new bool:gDodged, bool:gHitBlock, Float:gJumpLastY = 2900.0;
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
    printf("[dnav-test] wall: reached %d, through it %d, nearest %.1f m", gWallDone, gThroughWall, gWallNearest);
    if (!gWallDone) Fail("the chaser never got round the wall to its NPC");
    if (gThroughWall) Fail("the chaser drove through the wall");
    printf("[dnav-test] railing: reached %d, airborne %d, crossed the edge at x %.1f (gap at %.1f), through the railing %d, lowest z %.1f",
        gJumpDone, gWentAirborne, gCrossX, GAP_X, gThroughRail, gJumpLowest);
    if (!gWentAirborne) Fail("the chaser never went off the bridge");
    if (gThroughRail) Fail("the chaser went through the railing");
    if (gUnderGround) Fail("the chaser fell through the ground");
    if (!gJumpDone) Fail("the chaser never reached the NPC below the bridge");
    printf("[dnav-test] block: arrived %d, hit it %d", gDodged, gHitBlock);
    if (!gDodged) Fail("the driver never got past the block");
    if (gHitBlock) Fail("the driver drove into the block");
    print(gFailures ? ("[dnav-test] FAILED") : ("[dnav-test] PASS"));
    SendRconCommand("exit");
}

main() {}

public OnGameModeInit()
{
    printf("[dnav-test] walls: %d car tiles, %d road nodes", Nav_GetTileCount(NAV_MESH_CAR), Nav_GetRoadNodeCount());
    new Float:ride = Nav_GetVehicleInfo(560, NAV_VINFO_RIDE);

    gWaller = NPC_Create("dnav_waller");
    gWallQuarry = NPC_Create("dnav_wallq");
    NPC_Spawn(gWaller);
    NPC_Spawn(gWallQuarry);
    gWallCar = CreateVehicle(560, YX - 60.0, WALL_Y, YZ + ride, 270.0, 0, 0, -1);

    gJumper = NPC_Create("dnav_jumper");
    gJumpQuarry = NPC_Create("dnav_jumpq");
    NPC_Spawn(gJumper);
    NPC_Spawn(gJumpQuarry);
    gJumpCar = CreateVehicle(560, YX - 40.0, DECK_Y, DECK_Z + ride, 270.0, 0, 0, -1);

    gDodger = NPC_Create("dnav_dodger");
    NPC_Spawn(gDodger);
    gDodgeCar = CreateVehicle(560, YX - 120.0, BLOCK_Y - 2.5, YZ + ride, 270.0, 0, 0, -1);

    SetTimer("StartMoving", 500, false);
    SetTimer("Watch", 100, true);
    SetTimer("TimedOut", TIMEOUT_MS, false);
    return 1;
}

forward StartMoving();
public StartMoving()
{
    NPC_SetPos(gWallQuarry, YX + 40.0, WALL_Y, YZ + 1.0);
    NPC_PutInVehicle(gWaller, gWallCar, 0);
    if (!Nav_Chase(gWaller, gWallQuarry)) Fail("Nav_Chase refused the waller");

    NPC_SetPos(gJumpQuarry, YX, DECK_Y + 30.0, YZ + 1.0);
    NPC_PutInVehicle(gJumper, gJumpCar, 0);
    if (!Nav_Chase(gJumper, gJumpQuarry)) Fail("Nav_Chase refused the jumper");

    NPC_PutInVehicle(gDodger, gDodgeCar, 0);
    if (!Nav_DriveTo(gDodger, YX + 120.0, BLOCK_Y, YZ + 1.0)) Fail("Nav_DriveTo refused the dodger");
}

public OnNavMoveEnd(npcid, reason)
{
    if (npcid == gDodger) {
        new Float:x, Float:y, Float:z;
        GetVehiclePos(gDodgeCar, x, y, z);
        printf("[dnav-test] dodger ended: reason %d, car at %.1f %.1f", reason, x, y);
        gDodged = reason == NAV_END_ARRIVED && x > YX + 100.0;
    }
    return 1;
}

forward Watch();
public Watch()
{
    new Float:x, Float:y, Float:z, Float:d, mode;
    new Float:ride = Nav_GetVehicleInfo(560, NAV_VINFO_RIDE);
    if (!gWallDone) {
        GetVehiclePos(gWallCar, x, y, z);
        // The wall: x within 0.5 of YX, y within 15 of WALL_Y; the car's middle within
        // its half length of it means it is in it.
        if (floatabs(y - WALL_Y) < 15.0 + 0.5 && floatabs(x - YX) < 0.5 + 1.0) {
            if (!gThroughWall) printf("[dnav-test] waller in the wall: car at %.1f %.1f", x, y);
            gThroughWall = true;
        }
        d = VectorSize(x - (YX + 40.0), y - WALL_Y, 0.0);
        if (d < gWallNearest) gWallNearest = d;
        if (d < 10.0) {
            printf("[dnav-test] waller reached its quarry: car at %.1f %.1f", x, y);
            gWallDone = true;
            Nav_Stop(gWaller);
        }
    }

    if (!gJumpDone) {
        GetVehiclePos(gJumpCar, x, y, z);
        if (z < gJumpLowest) gJumpLowest = z;
        if (z < YZ + ride - 0.5) gUnderGround = true;
        if (Nav_GetChaseInfo(gJumper, d, mode) && mode == NAV_CHASE_AIRBORNE) gWentAirborne = true;
        // Crossing the bridge's north edge (y DECK_Y + 6) while still near the deck's
        // height: where along it.
        if (gJumpLastY < DECK_Y + 6.0 && y >= DECK_Y + 6.0 && z > DECK_Z - 2.0) {
            gCrossX = x;
            printf("[dnav-test] jumper crossed the edge at %.1f %.1f %.1f", x, y, z);
            if (floatabs(x - GAP_X) > 2.0) gThroughRail = true;
        }
        gJumpLastY = y;
        if (VectorSize(x - YX, y - (DECK_Y + 30.0), 0.0) < 10.0 && z < YZ + 3.0) {
            printf("[dnav-test] jumper reached its quarry: car at %.1f %.1f %.1f", x, y, z);
            gJumpDone = true;
            Nav_Stop(gJumper);
        }
    }

    if (gDodgeCar != INVALID_VEHICLE_ID && !gDodged) {
        // The block: x YX +- 1, y BLOCK_Y - 3.5 .. - 1.5. The Sultan heading east, give or
        // take its swerve, reaches about 2.3 m ahead of its middle and 1 m to the side.
        GetVehiclePos(gDodgeCar, x, y, z);
        if (floatabs(x - YX) < 1.0 + 2.1 && floatabs(y - (BLOCK_Y - 2.5)) < 1.0 + 0.9) {
            if (!gHitBlock) printf("[dnav-test] dodger in the block: car at %.1f %.1f", x, y);
            gHitBlock = true;
        }
    }

    if (gWallDone && gJumpDone && gDodged) Finish();
}

forward TimedOut();
public TimedOut()
{
    Fail("timed out");
    Finish();
}
