// How a car takes a hit, without a server. Run by ctest (or build/dnav_impact_tests).

#include "impact.hpp"

#include <cmath>
#include <cstdio>

using namespace dnav;

static int failures = 0;

#define CHECK(cond)                                                                 \
    do {                                                                            \
        if (!(cond)) {                                                              \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
            failures++;                                                             \
        }                                                                           \
    } while (0)

static Obb car(float x, float y) { return Obb { x, y, 0.0f, 1.0f, 1.0f, 2.3f }; }    // facing north
static Body body(float x, float y, float vy, float mass = 1400.0f)
{
    Body b;
    b.x = x;
    b.y = y;
    b.vy = vy;
    b.mass = mass;
    b.inertia = inertiaOf(mass, 4.6f, 2.0f);
    return b;
}

int main()
{
    Contact c;
    // Nothing touching.
    CHECK(!contact(car(0, 0), car(0, -6), c));

    // A tap square in the back: pushed on, no spin to speak of, speed in proportion.
    CHECK(contact(car(0, 0), car(0, -4.4f), c));
    CHECK(std::fabs(c.ny + 1.0f) < 0.01f && std::fabs(c.px) < 0.01f);
    Body npc = body(0, 0, 0);
    float j = collide(npc, body(0, -4.4f, 8.0f), c, 0.2f, 1.0f);
    CHECK(j > 0.0f && npc.vy > 3.0f && std::fabs(npc.vx) < 0.01f && std::fabs(npc.yaw) < 0.01f);
    Body gentle = body(0, 0, 0);
    collide(gentle, body(0, -4.4f, 2.0f), c, 0.2f, 1.0f);
    CHECK(gentle.vy < npc.vy * 0.4f);

    // The same hit on the rear corner: it spins (counter-clockwise: right side pushed forward).
    CHECK(contact(car(0, 0), car(1.5f, -4.4f), c));
    Body off = body(0, 0, 0);
    collide(off, body(1.5f, -4.4f, 8.0f), c, 0.2f, 1.0f);
    CHECK(off.yaw > 0.5f && off.vy > 0.0f);

    // Drifting apart or too slow to matter: nothing.
    Body apart = body(0, 0, 0);
    CHECK(collide(apart, body(0, -4.4f, -3.0f), c, 0.2f, 1.0f) == 0.0f);
    CHECK(collide(apart, body(0, -4.4f, 0.5f), c, 0.2f, 1.0f) == 0.0f);

    // A truck is hardly moved by a car; a car by a truck is.
    CHECK(contact(car(0, 0), car(0, -4.4f), c));
    Body truck = body(0, 0, 0, 10000.0f);
    collide(truck, body(0, -4.4f, 10.0f), c, 0.2f, 1.0f);
    Body light = body(0, 0, 0);
    collide(light, body(0, -4.4f, 10.0f, 10000.0f), c, 0.2f, 1.0f);
    CHECK(truck.vy < 1.5f && light.vy > 8.0f);

    // Sliding stops, and the spin dies.
    Body s = body(0, 0, 10.0f);
    s.yaw = 2.0f;
    for (int i = 0; i < 300; i++) slide(s, 0.02f, 0.7f);
    CHECK(std::fabs(s.vy) < 0.01f && std::fabs(s.yaw) < 0.01f);
    CHECK(massOf(4.6f, 2.0f, 1.4f) > 1200.0f && massOf(4.6f, 2.0f, 1.4f) < 1600.0f);

    if (failures) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("impact: all checks passed\n");
    return 0;
}
