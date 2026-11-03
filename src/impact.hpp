// How NPC cars take a hit: a 2D rigid-body impulse between two boxes seen from above,
// and the slide that follows. Nothing here touches open.mp; tests/test_impact.cpp
// tests it on its own. component.cpp does the detecting and moves the car.

#pragma once

namespace dnav {

// A vehicle's footprint: its middle, forward axis (unit) and half width / length.
struct Obb {
    float cx, cy;
    float fx, fy;
    float hw, hl;
};

// Where two boxes meet. n points from a toward b; depth is how far they overlap.
struct Contact {
    float nx = 0.0f, ny = 0.0f;
    float depth = 0.0f;
    float px = 0.0f, py = 0.0f;     // the point of contact, on a's surface
};

// A body seen from above. yaw is rad/s, counter-clockwise positive (GTA's heading
// grows the same way).
struct Body {
    float x = 0.0f, y = 0.0f;
    float vx = 0.0f, vy = 0.0f;
    float yaw = 0.0f;
    float mass = 1400.0f;
    float inertia = 2900.0f;
};

// A guess at a vehicle's mass from its size (m): handling.cfg's mass is not in vehicles.dnv.
float massOf(float length, float width, float height);
// A box's moment of inertia about its middle.
float inertiaOf(float mass, float length, float width);

// Whether the boxes overlap, and where (least penetration axis; the point is the middle of
// the deepest face of the box poking in, kept inside the other box).
bool contact(const Obb& a, const Obb& b, Contact& c);

// A hits B at c: changes a's velocity and spin by the impulse (b is as it was, but its mass
// and its lever arm count, so a heavy truck is not thrown by a light car). e is how bouncy;
// closings slower than minClosing (m/s) do nothing. Returns the impulse (N s), 0 for none.
float collide(Body& a, const Body& b, const Contact& c, float e, float minClosing);

// One step of sliding on the ground: friction (mu) slows it, its spin dies down.
void slide(Body& a, float dt, float mu);

// One step of a car knocked about on its wheels (heading: GTA's, degrees): it rolls along
// its own axis with little resistance (muRoll) but its tyres grip hard across it (muSide), so
// it goes where it points instead of skating sideways, and a spin dies faster the faster it
// goes.
void roll(Body& a, float headingDeg, float dt, float muRoll, float muSide);

} // namespace dnav