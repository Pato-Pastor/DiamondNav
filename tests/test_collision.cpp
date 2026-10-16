// The world's collision, without a server: rays against a small made-up world (a
// floor, a wall across it, a ramp and a bridge deck over the floor). Run by ctest (or
// build/dnav_collision_tests directly).

#include "collision.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

using namespace dnav;

static int failures = 0;

#define CHECK(cond)                                                                 \
    do {                                                                            \
        if (!(cond)) {                                                              \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
            failures++;                                                             \
        }                                                                           \
    } while (0)

static bool about(float a, float b, float tolerance)
{
    return std::fabs(a - b) <= tolerance;
}

static void quad(std::vector<float>& out, Vec3 a, Vec3 b, Vec3 c, Vec3 d)
{
    for (const Vec3* t : { &a, &b, &c, &a, &c, &d }) out.insert(out.end(), { t->x, t->y, t->z });
}

// Floor z 0 over x, y in -100..100; a wall at y 20 (x -10..10, 3 m tall); a ramp
// rising 2 m over y 40..60 at x 30..40; a bridge deck at z 10 over x -50..-40.
static std::vector<float> world()
{
    std::vector<float> t;
    quad(t, { -100, -100, 0 }, { 100, -100, 0 }, { 100, 100, 0 }, { -100, 100, 0 });
    quad(t, { -10, 20, 0 }, { 10, 20, 0 }, { 10, 20, 3 }, { -10, 20, 3 });
    quad(t, { 30, 40, 0 }, { 40, 40, 0 }, { 40, 60, 2 }, { 30, 60, 2 });
    quad(t, { -50, -100, 10 }, { -40, -100, 10 }, { -40, 100, 10 }, { -50, 100, 10 });
    return t;
}

static void rays()
{
    Collision c;
    c.build(world());
    CHECK(c.loaded());
    CHECK(c.triangleCount() == 8);
    RayHit hit;
    // Straight at the wall, at bumper height: hits it 15 m on, facing back.
    CHECK(c.wall({ 0, 0, 0.8f }, { 0, 40, 0.8f }, hit));
    CHECK(about(hit.at.y, 20.0f, 0.01f));
    CHECK(about(hit.t, 0.5f, 0.001f));
    CHECK(hit.normal.y < -0.99f);
    // From the other side too (faces are two-sided).
    CHECK(c.wall({ 0, 40, 0.8f }, { 0, 0, 0.8f }, hit) && hit.normal.y > 0.99f);
    // Past its end, or over it: clear.
    CHECK(!c.wall({ 15, 0, 0.8f }, { 15, 40, 0.8f }, hit));
    CHECK(!c.wall({ 0, 0, 3.5f }, { 0, 40, 3.5f }, hit));
    // Up the ramp at bumper height: the ramp is ground, not a wall.
    CHECK(!c.wall({ 35, 30, 0.8f }, { 35, 70, 0.8f }, hit));
    CHECK(c.ray({ 35, 30, 0.8f }, { 35, 70, 0.8f }, hit));     // but a ray does meet it
    // A long ray across many columns finds the nearest face.
    CHECK(c.wall({ -90, 20 - 90, 1 }, { 90, 20 + 90, 1 }, hit));
    CHECK(about(hit.at.y, 20.0f, 0.05f) && about(hit.at.x, 0.0f, 0.05f));
}

static void grounds()
{
    Collision c;
    c.build(world());
    float z = 0.0f;
    CHECK(c.ground(0, 0, 1, 2, 10, z) && about(z, 0.0f, 0.001f));
    CHECK(c.ground(35, 50, 2, 2, 10, z) && about(z, 1.0f, 0.01f));
    // Under the bridge, from the road below: the road, not the deck.
    CHECK(c.ground(-45, 0, 1, 2, 10, z) && about(z, 0.0f, 0.001f));
    // On the deck: the deck.
    CHECK(c.ground(-45, 0, 11, 2, 20, z) && about(z, 10.0f, 0.001f));
    // Off the world's edge: nothing.
    CHECK(!c.ground(500, 500, 1, 2, 10, z));
}

static void file()
{
    const std::vector<float> t = world();
    {
        std::ofstream out("collision_test.tris", std::ios::binary);
        const uint32_t n = static_cast<uint32_t>(t.size() / 9);
        out.write("DNTR", 4);
        out.write(reinterpret_cast<const char*>(&n), 4);
        out.write(reinterpret_cast<const char*>(t.data()), static_cast<std::streamsize>(t.size() * 4));
    }
    Collision c;
    std::string error;
    CHECK(c.load("collision_test.tris", error));
    CHECK(c.triangleCount() == 8);
    {
        std::ofstream out("collision_bad.tris", std::ios::binary);
        out.write("NOPE", 4);
    }
    Collision bad;
    CHECK(!bad.load("collision_bad.tris", error) && !error.empty());
    CHECK(!bad.loaded());
}

int main()
{
    rays();
    grounds();
    file();
    if (failures) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("collision: all checks passed\n");
    return 0;
}
