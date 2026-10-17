// How NPC helicopters fly (flying.hpp).

#include "flying.hpp"

#include <algorithm>
#include <cmath>

namespace dnav {

namespace {

constexpr float PI = 3.14159265f;
constexpr float DEG = PI / 180.0f;
constexpr float NOTHING = -1e9f;        // a top with nothing there
constexpr float PLAN_EVERY = 0.25f;     // s between looks at the floor
constexpr float SAMPLE = 10.0f;         // m between tops looked at along a line
constexpr float SIDE = 6.0f;            // m either side of its way it looks too
constexpr float LAND_NEAR = 25.0f;      // m out, a landing starts down its glide slope
constexpr float GLIDE = 0.8f;           // m down per m in, on that slope
constexpr float TOUCH = 2.5f;           // m/s down, at most, the last metres to the ground
constexpr float VACCEL = 6.0f;          // m/s^2 it changes its climb or sink by
constexpr float WALL_KEEP = 12.0f;      // m short of a face ahead it stops
constexpr float CLIMB_SHARE = 0.8f;     // of its best climb, what it plans to climb over what is ahead
constexpr float DIVE_ABOVE = 40.0f;     // m: further above where it wants to be, it comes down faster...
constexpr float DIVE = 1.6f;            // ...this much faster

float clampf(float v, float lo, float hi)
{
    return std::max(lo, std::min(hi, v));
}

float wrap(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

float topAt(const FlyWorld& w, float x, float y)
{
    return w.top ? w.top(x, y) : NOTHING;
}

// How high it must be here to clear everything along a line from (x, y) on (ux, uy)
// for len metres (and either side of it, with sides), climbing `slope` metres per
// metre on the way: a tower far ahead does not lift it yet, one close by does.
float highest(const FlyWorld& w, float x, float y, float ux, float uy, float len, float slope, bool sides)
{
    float best = NOTHING;
    for (float s = 0.0f; s <= len; s += SAMPLE) {
        const float px = x + ux * s, py = y + uy * s;
        float top = topAt(w, px, py);
        if (sides) {
            top = std::max(top, topAt(w, px + uy * SIDE, py - ux * SIDE));
            top = std::max(top, topAt(w, px - uy * SIDE, py + ux * SIDE));
        }
        best = std::max(best, top - s * slope);
    }
    return best;
}

// Where a landing at g settles: the surface there, or g.at's height with none near it.
float landGround(const FlyGoal& g, const FlyWorld& w)
{
    const float t = topAt(w, g.at.x, g.at.y);
    return t > g.at.z - 10.0f ? t : g.at.z - 1.0f;
}

} // namespace

bool isHelicopter(int model)
{
    switch (model) {
    case 417: case 425: case 447: case 469: case 487: case 488: case 497: case 548: case 563:
        return true;
    default:
        return false;
    }
}

HeliSpec HeliSpec::of(int model)
{
    HeliSpec s;
    switch (model) {
    case 487: case 488: case 497:               // Maverick, News Chopper, Police Maverick
        s.top = 50.0f; s.accel = 6.0f; s.brake = 8.0f;
        break;
    case 469: case 447: case 563:               // Sparrow, Seasparrow, Raindance
        s.top = 42.0f;
        break;
    case 417: case 548:                         // Leviathan, Cargobob
        s.top = 38.0f; s.accel = 4.0f; s.brake = 5.0f; s.yawRate = 45.0f; s.ride = 2.0f;
        break;
    case 425:                                   // Hunter
        s.top = 60.0f; s.accel = 6.5f; s.brake = 7.5f; s.yawRate = 85.0f; s.ride = 1.8f;
        break;
    default:
        break;
    }
    return s;
}

float headingOf(float dx, float dy)
{
    float h = std::atan2(-dx, dy) / DEG;
    if (h < 0.0f) h += 360.0f;
    return h;
}

float angleFrom(const Vec3& centre, const Vec3& p)
{
    const float dx = p.x - centre.x, dy = p.y - centre.y;
    if (dx * dx + dy * dy < 0.01f) return 0.0f;
    return std::atan2(dy, dx) / DEG;
}

FlyGoal orbit(const Vec3& centre, const Vec3& centreVel, float radius, float height,
    float rate, float& angle, bool face, float dt)
{
    angle = std::fmod(angle + rate * dt, 360.0f);
    const float c = std::cos(angle * DEG), s = std::sin(angle * DEG);
    const float round = rate * DEG * radius;    // m/s along the circle
    FlyGoal g;
    g.at = { centre.x + c * radius, centre.y + s * radius, centre.z + height };
    g.vel = { centreVel.x - s * round, centreVel.y + c * round, 0.0f };
    g.face = face;
    g.faceHeading = headingOf(-c, -s);
    return g;
}

FlyGoal trail(const Vec3& at, const Vec3& vel, float behind, float height)
{
    const float v = std::sqrt(vel.x * vel.x + vel.y * vel.y);
    FlyGoal g;
    const float bx = v > 0.1f ? vel.x / v * behind : 0.0f, by = v > 0.1f ? vel.y / v * behind : 0.0f;
    g.at = { at.x - bx, at.y - by, at.z + height };
    g.vel = { vel.x, vel.y, 0.0f };
    g.face = true;
    g.faceHeading = headingOf(bx, by);
    return g;
}

bool fly(Flight& f, const FlyGoal& g, const HeliSpec& s, const FlyWorld& w, float dt)
{
    if (dt <= 0.0f) return false;
    const float dx = g.at.x - f.pos.x, dy = g.at.y - f.pos.y;
    const float d = std::sqrt(dx * dx + dy * dy);
    const float ux = d > 0.01f ? dx / d : 0.0f, uy = d > 0.01f ? dy / d : 0.0f;
    const float vh = std::sqrt(f.vel.x * f.vel.x + f.vel.y * f.vel.y);
    const float cap = g.speed > 0.0f ? std::min(g.speed, s.top) : s.top;
    const bool landing = g.land && d < LAND_NEAR;
    const float ground = g.land ? landGround(g, w) : NOTHING;

    // Look at what is under and ahead of it, now and then: the floor it keeps over.
    f.planIn -= dt;
    if (f.planIn <= 0.0f) {
        f.planIn = PLAN_EVERY;
        const float look = clampf(vh * 3.0f + 30.0f, 30.0f, 150.0f);
        const float slope = CLIMB_SHARE * s.climb / std::max(vh, 8.0f);
        float top = topAt(w, f.pos.x, f.pos.y);
        if (vh > 1.0f) top = std::max(top, highest(w, f.pos.x, f.pos.y, f.vel.x / vh, f.vel.y / vh, look, slope, true));
        if (d > 1.0f) top = std::max(top, highest(w, f.pos.x, f.pos.y, ux, uy, std::min(look, d), slope, false));
        f.floor = top + g.clear;
    }

    // How high it wants to be: the goal's height, over the floor; coming in to land,
    // down a slope to the ground, kept only off what is right under it.
    float zWant = std::max(g.at.z, f.floor);
    if (landing) {
        float under = topAt(w, f.pos.x, f.pos.y);
        if (vh > 1.0f) under = std::max(under, topAt(w, f.pos.x + f.vel.x / vh * 6.0f, f.pos.y + f.vel.y / vh * 6.0f));
        const float keep = under + s.ride + (d > 3.0f ? 2.0f : 0.0f);
        zWant = std::max(ground + s.ride + d * GLIDE, keep);
        zWant = std::min(zWant, std::max(g.at.z, f.floor));
        zWant = std::max(zWant, ground + s.ride);
        if (d < 3.0f) zWant = ground + s.ride - 1.0f;  // aimed under the ground, so it sets down
    }

    if (f.down) {
        if (g.land && d < 3.0f) {
            f.vel = {};
            f.yawRate = 0.0f;
            f.pitch = 0.0f;
            f.roll = 0.0f;
            f.want = 0.0f;
            return true;
        }
        f.down = false;     // somewhere else to be: off it goes
    }

    // Up and down.
    const float zErr = zWant - f.pos.z;
    float vzWant = clampf(zErr * 1.2f, zErr < -DIVE_ABOVE ? -s.sink * DIVE : -s.sink, s.climb);
    if (landing && d < 4.0f && zErr > -6.0f) vzWant = std::max(vzWant, -TOUCH);
    const float vzStep = VACCEL * dt;
    f.vel.z += clampf(vzWant - f.vel.z, -vzStep, vzStep);

    // Across: for the goal, slowing in time to stop there, moving with it once there.
    float speed = std::min(cap, std::sqrt(2.0f * s.brake * 0.8f * d));
    if (landing && d < 4.0f) speed = std::min(speed, d * 0.8f);
    float wx = ux * speed + g.vel.x, wy = uy * speed + g.vel.y;
    // Far under the floor (off the ground, or a tower close ahead), it climbs first.
    const float below = f.floor - f.pos.z;
    if (!landing && below > 10.0f) {
        const float k = clampf(1.0f - (below - 10.0f) / 40.0f, 0.3f, 1.0f);
        wx *= k;
        wy *= k;
    }
    float ws = std::sqrt(wx * wx + wy * wy);
    if (ws > s.top) {
        wx *= s.top / ws;
        wy *= s.top / ws;
        ws = s.top;
    }
    // Anything solid on its way: stop short of it, and climb over it.
    if (w.ray && ws > 0.5f) {
        const float reach = std::max(25.0f, vh * vh / (2.0f * s.brake) + 20.0f);
        const Vec3 to { f.pos.x + wx / ws * reach, f.pos.y + wy / ws * reach, f.pos.z };
        float t = 1.0f;
        if (w.ray(f.pos, to, t)) {
            const float hit = t * reach;
            const float hx = f.pos.x + wx / ws * hit, hy = f.pos.y + wy / ws * hit;
            f.floor = std::max(f.floor, topAt(w, hx + wx / ws * 2.0f, hy + wy / ws * 2.0f) + g.clear);
            const float ok = std::sqrt(2.0f * s.brake * std::max(0.0f, hit - WALL_KEEP));
            if (ws > ok) {
                wx *= ok / ws;
                wy *= ok / ws;
                ws = ok;
            }
        }
    }
    f.want = ws;
    float ax = (wx - f.vel.x) / dt, ay = (wy - f.vel.y) / dt;
    const bool slowing = ax * f.vel.x + ay * f.vel.y < 0.0f;
    const float amax = slowing ? s.brake : s.accel;
    const float am = std::sqrt(ax * ax + ay * ay);
    if (am > amax) {
        ax *= amax / am;
        ay *= amax / am;
    }
    f.vel.x += ax * dt;
    f.vel.y += ay * dt;

    // Which way it points: at what it looks at, else the way it goes.
    float headWant = f.heading;
    if (g.face) headWant = g.faceHeading;
    else if (ws > 4.0f) headWant = headingOf(wx, wy);
    const float yawWant = clampf(wrap(headWant - f.heading) * 2.0f, -s.yawRate, s.yawRate);
    const float yawStep = 120.0f * dt;
    f.yawRate += clampf(yawWant - f.yawRate, -yawStep, yawStep);
    f.heading = std::fmod(f.heading + f.yawRate * dt + 360.0f, 360.0f);

    // How it leans: nose down for speed and to pick it up, up to slow; banked into a
    // turn and toward where it is pushed sideways.
    const float hr = f.heading * DEG;
    const float fx = -std::sin(hr), fy = std::cos(hr);  // forward
    const float lx = -fy, ly = fx;                      // left
    const float vFwd = f.vel.x * fx + f.vel.y * fy;
    const float aFwd = ax * fx + ay * fy;
    const float aLeft = ax * lx + ay * ly;
    const float pitchWant = clampf(-s.tilt * vFwd / s.top - aFwd * 1.5f, -s.tilt - 10.0f, s.tilt + 10.0f);
    const float speedNow = std::sqrt(f.vel.x * f.vel.x + f.vel.y * f.vel.y);
    const float turn = std::atan2(speedNow * f.yawRate * DEG, 9.81f) / DEG;
    const float rollWant = clampf(turn + aLeft * 1.5f, -s.bank, s.bank);
    const float ease = std::min(1.0f, dt * 3.0f);
    f.pitch += (pitchWant - f.pitch) * ease;
    f.roll += (rollWant - f.roll) * ease;

    // Move, never into the ground.
    f.pos.x += f.vel.x * dt;
    f.pos.y += f.vel.y * dt;
    f.pos.z += f.vel.z * dt;
    float floorNow = topAt(w, f.pos.x, f.pos.y);
    if (g.land) {
        const float ex = g.at.x - f.pos.x, ey = g.at.y - f.pos.y;
        if (ex * ex + ey * ey < 16.0f) floorNow = std::max(floorNow, ground);
    }
    floorNow += s.ride;
    if (f.pos.z <= floorNow) {
        f.pos.z = floorNow;
        if (f.vel.z < 0.0f) f.vel.z = 0.0f;
        const float ex = g.at.x - f.pos.x, ey = g.at.y - f.pos.y;
        if (g.land && ex * ex + ey * ey < 9.0f) {
            f.down = true;
            f.vel = {};
            f.yawRate = 0.0f;
            f.pitch = 0.0f;
            f.roll = 0.0f;
            return true;
        }
    }

    if (g.land) return false;
    const float rvx = f.vel.x - g.vel.x, rvy = f.vel.y - g.vel.y;
    return d <= g.arrive && std::fabs(f.pos.z - zWant) < 2.0f && rvx * rvx + rvy * rvy < 4.0f;
}

} // namespace dnav
