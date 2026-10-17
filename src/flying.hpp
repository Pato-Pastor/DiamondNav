// How NPC helicopters fly: a flight model with no path search. A helicopter goes
// straight for where it is going, high enough to clear everything under and ahead of
// it (the world's collision says how high that is), slows to stop there, hovers, and
// lands where it is told to. It leans into what it does: nose down to pick up speed,
// nose up to slow, banked into a turn.
//
// The component flies with this (component.cpp: Nav_FlyTo, and Nav_Chase for an NPC
// in a helicopter); nothing here touches open.mp, so it is tested on its own against
// a made-up world (tests/test_flying.cpp).

#pragma once

#include <functional>

#include "collision.hpp"

namespace dnav {

// Whether a model is a helicopter a pilot flies (not the RC ones).
bool isHelicopter(int model);

// How a helicopter flies, in metres, seconds and degrees.
struct HeliSpec {
    float top = 45.0f;          // m/s, flat out
    float accel = 5.0f;         // m/s^2 picking up speed
    float brake = 6.0f;         // m/s^2 slowing down
    float climb = 9.0f;         // m/s up, at most
    float sink = 6.0f;          // m/s down, at most
    float yawRate = 70.0f;      // degrees a second it turns on the spot, at most
    float ride = 1.2f;          // m its centre sits over the ground on its skids
    float tilt = 14.0f;         // degrees nose down at top speed
    float bank = 22.0f;         // degrees it banks into a turn, at most

    static HeliSpec of(int model);
};

// What a flight needs of the world: the highest solid surface at (x, y) (a roof, a
// tree, a hill; very low with nothing there), and the first face between two points
// (t: how far along, 0 to 1).
struct FlyWorld {
    std::function<float(float x, float y)> top;
    std::function<bool(const Vec3& a, const Vec3& b, float& t)> ray;
};

// Where a flight is going.
struct FlyGoal {
    Vec3 at {};                 // the helicopter's centre there
    Vec3 vel {};                // ...moving like this once there (over a moving target)
    float clear = 20.0f;        // m: it keeps this far above everything under and ahead of it
    float speed = 0.0f;         // m/s it flies at most (0: its top speed)
    bool land = false;          // down onto the ground (or roof) at `at`, instead of hovering
    bool face = false;          // turned to faceHeading (at a target) rather than the way it flies
    float faceHeading = 0.0f;   // GTA's, degrees
    float arrive = 3.0f;        // m: there once this near, slow and at its height
};

// A helicopter in the air.
struct Flight {
    Vec3 pos {};                // its centre
    Vec3 vel {};                // m/s
    float heading = 0.0f;       // GTA's, degrees (0 north, turning left)
    float yawRate = 0.0f;       // degrees a second
    float pitch = 0.0f;         // degrees, nose up positive (its rotation)
    float roll = 0.0f;          // degrees, banked left positive
    float floor = -1e9f;        // the lowest it may fly here: the tops under and ahead, and the clearance
    float planIn = 0.0f;        // s until the floor is looked at again
    float want = 0.0f;          // m/s: the speed it is going for (Nav_GetDriveInfo)
    bool down = false;          // on the ground (landed)
};

// One step of dt seconds toward g. Returns true once it is there: hovering at g.at
// (slow, at its height), or down on the ground with g.land.
bool fly(Flight& f, const FlyGoal& g, const HeliSpec& s, const FlyWorld& w, float dt);

// GTA's heading for a direction (degrees: 0 north, turning left).
float headingOf(float dx, float dy);

// A goal going round a circle: `angle` (degrees, from east, turning left) moves on by
// `rate` degrees a second, and the goal is the point there, `height` over the centre,
// moving with the centre plus its own way round. With face, it looks at the centre (a
// helicopter circling over someone), else the way it flies (one searching).
FlyGoal orbit(const Vec3& centre, const Vec3& centreVel, float radius, float height,
    float rate, float& angle, bool face, float dt);

// A goal following something on the move: `behind` metres back along its way and
// `height` over it, moving as it moves, looking at it (a helicopter after a car keeps
// over the street it has just driven down, clear of the buildings either side).
FlyGoal trail(const Vec3& at, const Vec3& vel, float behind, float height);

// The angle on a circle round centre nearest to p (to start an orbit where it is).
float angleFrom(const Vec3& centre, const Vec3& p);

} // namespace dnav
