// Ten NPCs wander the city on foot: each walks to a random walkable point, waits
// a moment, and picks another. Needs a navmesh (navmesh/world.nav).

#include <open.mp>
#include <DiamondNav>

#define WANDERERS (10)

new gWanderer[WANDERERS] = { INVALID_NPC_ID, ... };

main() {}

public OnGameModeInit()
{
    if (!Nav_GetTileCount()) {
        print("wander: no navmesh in navmesh/, nobody will walk");
        return 1;
    }
    new name[24];
    for (new i = 0; i < WANDERERS; i++) {
        format(name, sizeof name, "Wanderer_%d", i);
        gWanderer[i] = NPC_Create(name);
        NPC_Spawn(gWanderer[i]);
    }
    return 1;
}

public OnNPCSpawn(npcid)
{
    new Float:x, Float:y, Float:z;
    if (Nav_GetRandomPoint(x, y, z)) NPC_SetPos(npcid, x, y, z + 1.0);
    WalkSomewhere(npcid);
    return 1;
}

WalkSomewhere(npcid)
{
    new Float:x, Float:y, Float:z;
    if (Nav_GetRandomPoint(x, y, z)) Nav_WalkTo(npcid, x, y, z + 1.0, NPC_MOVE_TYPE_WALK);
}

forward NextWalk(npcid);
public NextWalk(npcid)
{
    if (NPC_IsValid(npcid)) WalkSomewhere(npcid);
}

public OnNavMoveEnd(npcid, reason)
{
    // Arrived, or no way there: either way, a short rest and somewhere new.
    if (reason == NAV_END_ARRIVED || reason == NAV_END_NO_PATH) SetTimerEx("NextWalk", 3000 + random(5000), false, "i", npcid);
    return 1;
}
