// /taxi: an NPC taxi drives to you along the roads, in its lane, and stops at your
// side. /taxi again while it is coming sends it to where you are now.
// Needs a road graph (navmesh/roads.dnr).

#include <open.mp>
#include <DiamondNav>

new gDriver = INVALID_NPC_ID, gTaxi = INVALID_VEHICLE_ID, gCaller = INVALID_PLAYER_ID;

main() {}

public OnGameModeInit()
{
    if (!Nav_GetRoadNodeCount()) print("taxi: no roads.dnr in navmesh/, the taxi can't drive");
    gDriver = NPC_Create("Taxi_Driver");
    NPC_Spawn(gDriver);
    return 1;
}

public OnPlayerCommandText(playerid, cmdtext[])
{
    if (strcmp(cmdtext, "/taxi", true)) return 0;
    new Float:x, Float:y, Float:z, Float:rx, Float:ry, Float:rz;
    GetPlayerPos(playerid, x, y, z);
    if (gTaxi == INVALID_VEHICLE_ID) {
        // Start on a road some way off: the nearest road point to a spot 150 m away.
        if (!Nav_GetNearestRoadPoint(x + 150.0, y, z, rx, ry, rz, 200.0)) {
            SendClientMessage(playerid, -1, "No roads near you.");
            return 1;
        }
        gTaxi = CreateVehicle(420, rx, ry, rz + 1.0, 0.0, 6, 6, -1);
        NPC_PutInVehicle(gDriver, gTaxi, 0);
    }
    gCaller = playerid;
    // A taxi driver in a hurry: a little over the limits.
    Nav_SetDriveStyle(gDriver, NAV_STYLE_FAST);
    if (Nav_DriveTo(gDriver, x, y, z)) SendClientMessage(playerid, -1, "Your taxi is on its way.");
    return 1;
}

public OnNavMoveEnd(npcid, reason)
{
    if (npcid != gDriver || gCaller == INVALID_PLAYER_ID) return 1;
    switch (reason) {
        case NAV_END_ARRIVED: SendClientMessage(gCaller, -1, "Your taxi is here.");
        case NAV_END_NO_PATH: SendClientMessage(gCaller, -1, "The taxi can't find a way to you.");
    }
    return 1;
}
