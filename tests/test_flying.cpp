// How helicopters fly, without a server: a flight against a made-up world (flat ground
// at z 0 with a 120 m tower on it, and a 20 m building to land on). Run by ctest (or
// build/dnav_flying_tests directly).

#include "flying.hpp"

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

// The tower: x 190..210, y -10..10, 120 m. The building: x -60..-40, y 90..110, 20 m.
static float top(float x, float y)
{
    if (x >= 190 && x <= 210 && y >= -10 && y <= 10) return 120.0f;
    if (x >= -60 && x <= -40 && y >= 90 && y <= 110) return 20.0f;
    return 0.0f;
}

static bool inside(const Vec3& p)
{
    return p.z < top(p.x, p.y);
}

// A ray meets the tower or the building where it goes inside one (in steps of 0.5 m).
static bool ray(const Vec3& a, const Vec3& b, float& t)
{
    const float len = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
    const int n = std::max(1, static_cast<int>(len / 0.5f));
    for (int i = 1; i <= n; i++) {
        const float k = static_cast<float>(i) / n;
        const Vec3 p { a.x + (b.x - a.x) * k, a.y + (b.y - a.y) * k, a.z + (b.z - a.z) * k };
        if (p.z > 0.05f && inside(p)) {
            t = k;
            return true;
        }
    }
    return false;
}

static FlyWorld world()
{
    FlyWorld w;
    w.top = top;
    w.ray = ray;
    return w;
}

static float dist2d(const Vec3& a, const Vec3& b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// From the ground, over the tower, to a hover on the far side.
static void overTheTower()
{
    const HeliSpec s = HeliSpec::of(497);
    Flight f;
    f.pos = { 0, 0, s.ride };
    f.heading = 270.0f;                 // facing east, the way it goes
    FlyGoal g;
    g.at = { 400, 0, 30 };
    const FlyWorld w = world();
    bool there = false, wentIn = false;
    float highest = 0.0f, fastest = 0.0f, overTower = 0.0f;
    for (int i = 0; i < 20 * 120 && !there; i++) {
        there = fly(f, g, s, w, 0.05f);
        wentIn = wentIn || inside(f.pos);
        highest = std::max(highest, f.pos.z);
        fastest = std::max(fastest, std::sqrt(f.vel.x * f.vel.x + f.vel.y * f.vel.y));
        if (f.pos.x >= 190 && f.pos.x <= 210) overTower = std::max(overTower, f.pos.z);
        CHECK(f.pitch > -40.0f && f.pitch < 40.0f && std::fabs(f.roll) <= s.bank + 0.01f);
    }
    CHECK(there);
    CHECK(!wentIn);
    CHECK(overTower > 120.0f);
    CHECK(fastest > 30.0f && fastest <= s.top + 0.01f);
    CHECK(dist2d(f.pos, g.at) <= g.arrive);
    CHECK(std::fabs(f.pos.z - 30.0f) < 2.0f);
    CHECK(f.heading > 240.0f && f.heading < 300.0f);
    // And it holds there, hovering.
    for (int i = 0; i < 100; i++) fly(f, g, s, w, 0.05f);
    CHECK(dist2d(f.pos, g.at) < 3.0f && std::fabs(f.pos.z - 30.0f) < 2.0f);
}

// Onto the building's roof, and down to the ground beside it.
static void landings()
{
    const HeliSpec s = HeliSpec::of(487);
    Flight f;
    f.pos = { 100, -100, 40 };
    FlyGoal g;
    g.at = { -50, 100, 21 };
    g.land = true;
    const FlyWorld w = world();
    bool down = false, wentIn = false;
    for (int i = 0; i < 20 * 120 && !down; i++) {
        down = fly(f, g, s, w, 0.05f);
        wentIn = wentIn || inside(f.pos);
    }
    CHECK(down && f.down);
    CHECK(!wentIn);
    CHECK(dist2d(f.pos, g.at) < 3.0f);
    CHECK(std::fabs(f.pos.z - (20.0f + s.ride)) < 0.01f);
    // Stays down.
    CHECK(fly(f, g, s, w, 0.05f) && f.down);

    // Off again, to the ground east of the building.
    g.at = { 0, 100, 1 };
    down = false;
    for (int i = 0; i < 20 * 60 && !down; i++) {
        down = fly(f, g, s, w, 0.05f);
        wentIn = wentIn || inside(f.pos);
    }
    CHECK(down);
    CHECK(!wentIn);
    CHECK(std::fabs(f.pos.z - s.ride) < 0.01f);
}

// Circling over someone walking: it keeps up, at its height, looking at them.
static void overhead()
{
    const HeliSpec s = HeliSpec::of(497);
    Flight f;
    f.pos = { -300, -300, 60 };
    const FlyWorld w = world();
    Vec3 who { 0, -200, 1 };
    const Vec3 walk { 3, 0, 0 };
    float angle = angleFrom(who, f.pos);
    int near = 0;
    for (int i = 0; i < 20 * 120; i++) {
        who.x += walk.x * 0.05f;
        const FlyGoal g = orbit(who, walk, 22.0f, 28.0f, 9.0f, angle, true, 0.05f);
        fly(f, g, s, w, 0.05f);
        CHECK(!inside(f.pos));
        if (i > 20 * 60) {
            const float d = dist2d(f.pos, who);
            const float look = headingOf(who.x - f.pos.x, who.y - f.pos.y);
            float off = std::fabs(look - f.heading);
            if (off > 180.0f) off = 360.0f - off;
            if (d > 14.0f && d < 32.0f && std::fabs(f.pos.z - who.z - 28.0f) < 4.0f && off < 25.0f) near++;
        }
    }
    CHECK(near > 20 * 55);
}

// A wall of towers 150 m tall with one gap: under a ceiling it goes round through the
// gap, never over; without one it goes over.
static float towers(float x, float y)
{
    if (x >= 100 && x <= 130 && (y < -20 || y > 20) && y > -400 && y < 400) return 150.0f;
    return 0.0f;
}

static bool inTowers(const Vec3& p)
{
    return p.z < towers(p.x, p.y);
}

static void roundTheTowers()
{
    const HeliSpec s = HeliSpec::of(497);
    FlyWorld w;
    w.top = towers;
    w.ray = [](const Vec3& a, const Vec3& b, float& t) {
        const float len = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
        const int n = std::max(1, static_cast<int>(len / 0.5f));
        for (int i = 1; i <= n; i++) {
            const float k = static_cast<float>(i) / n;
            if (inTowers({ a.x + (b.x - a.x) * k, a.y + (b.y - a.y) * k, a.z + (b.z - a.z) * k })) {
                t = k;
                return true;
            }
        }
        return false;
    };
    for (int capped = 0; capped < 2; capped++) {
        Flight f;
        f.pos = { 0, 150, 30 };
        FlyGoal g;
        g.at = { 250, 150, 30 };
        if (capped) g.ceiling = 60.0f;
        bool there = false, wentIn = false;
        float highest = 0.0f, gapY = 1e9f;
        for (int i = 0; i < 20 * 120 && !there; i++) {
            there = fly(f, g, s, w, 0.05f);
            wentIn = wentIn || inTowers(f.pos);
            highest = std::max(highest, f.pos.z);
            if (f.pos.x >= 100 && f.pos.x <= 130) gapY = std::min(gapY, std::fabs(f.pos.y));
        }
        CHECK(there);
        CHECK(!wentIn);
        if (capped) {
            CHECK(highest < 60.0f);
            CHECK(gapY < 20.0f);
        } else {
            CHECK(highest > 150.0f);
        }
    }
}

static void basics()
{
    CHECK(isHelicopter(497) && isHelicopter(487) && isHelicopter(425));
    CHECK(!isHelicopter(465) && !isHelicopter(501) && !isHelicopter(596) && !isHelicopter(520));
    CHECK(std::fabs(headingOf(0, 1)) < 0.01f);
    CHECK(std::fabs(headingOf(-1, 0) - 90.0f) < 0.01f);
    CHECK(std::fabs(headingOf(1, 0) - 270.0f) < 0.01f);
    CHECK(HeliSpec::of(425).top > HeliSpec::of(548).top);
}

int main()
{
    basics();
    overTheTower();
    landings();
    overhead();
    roundTheTowers();
    if (failures) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("flying: all checks passed\n");
    return 0;
}
