// How NPC helicopters fly (flying.hpp).

#include "flying.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace dnav {

namespace {

constexpr float PI = 3.14159265f;
constexpr float DEG = PI / 180.0f;
constexpr float NOTHING = -1e9f;        // a top with nothing there
constexpr float PLAN_EVERY = 0.25f;     // s between looks at the floor
constexpr float SAMPLE = 4.0f;          // m between tops looked at along a line
constexpr float SIDE = 6.0f;            // m either side of its way it looks too...
constexpr float SIDE_ROUND = 3.0f;      // ...going round towers (its way keeps a cell off them)
constexpr float LAND_NEAR = 25.0f;      // m out, a landing starts down its glide slope
constexpr float GLIDE = 0.8f;           // m down per m in, on that slope
constexpr float TOUCH = 2.5f;           // m/s down, at most, the last metres to the ground
constexpr float VACCEL = 6.0f;          // m/s^2 it changes its climb or sink by
constexpr float WALL_KEEP = 12.0f;      // m short of a face ahead it stops
constexpr float KEEP_OFF = 10.0f;       // m: it keeps its rotor this far off anything beside it...
constexpr float KEEP_PUSH = 8.0f;       // ...pushed away at up to this many m/s
constexpr int KEEP_RAYS = 12;
constexpr float ROUTE_EVERY = 0.5f;     // s between plans of its way round
constexpr float CELL = 6.0f;            // m: the grid it plans on
constexpr float ROUTE_PAD = 90.0f;      // m round the box from it to the goal that it may fly through, at least...
constexpr float ROUTE_PAD_SHARE = 0.6f; // ...or this share of the way there
constexpr float ROUTE_REACH = 500.0f;   // m: a goal further than this is planned to a point this far on
constexpr float ROUTE_LONGEST = 3.0f;   // a way round longer than this many times straight: over instead
constexpr int ROUTE_CELLS = 60000;      // at most this many cells in a plan
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
// for len metres (and `sides` metres either side of it), climbing `slope` metres per
// metre on the way: a tower far ahead does not lift it yet, one close by does.
float highest(const FlyWorld& w, float x, float y, float ux, float uy, float len, float slope, float sides)
{
    float best = NOTHING;
    for (float s = 0.0f; s <= len; s += SAMPLE) {
        const float px = x + ux * s, py = y + uy * s;
        float top = topAt(w, px, py);
        if (sides > 0.0f) {
            top = std::max(top, topAt(w, px + uy * sides, py - ux * sides));
            top = std::max(top, topAt(w, px - uy * sides, py + ux * sides));
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

// The highest top in a grid cell (its middle), looked at once per flight.
float cellTop(Flight& f, const FlyWorld& w, int cx, int cy)
{
    const long long key = (static_cast<long long>(cx) << 32) ^ static_cast<unsigned int>(cy);
    auto it = f.tops.find(key);
    if (it != f.tops.end()) return it->second;
    if (f.tops.size() > 300000) f.tops.clear();
    // Its middle and four points round it: a thin mast or a building's corner in a
    // cell is not missed.
    const float mx = (cx + 0.5f) * CELL, my = (cy + 0.5f) * CELL, q = CELL * 0.3f;
    float t = topAt(w, mx, my);
    t = std::max(t, topAt(w, mx - q, my - q));
    t = std::max(t, topAt(w, mx + q, my - q));
    t = std::max(t, topAt(w, mx - q, my + q));
    t = std::max(t, topAt(w, mx + q, my + q));
    f.tops[key] = t;
    return t;
}

// Plans its way to `to` round every cell whose top (and the clearance over it) would
// take it above g.ceiling, on a grid of CELL metres, the cells next to one too: an
// 8-way A* in a box round it and the goal, then the furthest corner of that way it
// can fly to straight. With no way round, or one far longer than straight, it goes
// straight (over).
void route(Flight& f, const FlyGoal& g, const FlyWorld& w, const Vec3& to)
{
    f.aim = to;
    f.aiming = false;
    const float sx = to.x - f.pos.x, sy = to.y - f.pos.y;
    const float straight = std::sqrt(sx * sx + sy * sy);
    f.aimLeft = straight;
    if (straight < CELL * 2.0f) return;
    float tx = to.x, ty = to.y;
    if (straight > ROUTE_REACH) {
        tx = f.pos.x + sx / straight * ROUTE_REACH;
        ty = f.pos.y + sy / straight * ROUTE_REACH;
    }
    const float pad = std::max(ROUTE_PAD, ROUTE_PAD_SHARE * std::min(straight, ROUTE_REACH));
    const int x0 = static_cast<int>(std::floor((std::min(f.pos.x, tx) - pad) / CELL));
    const int y0 = static_cast<int>(std::floor((std::min(f.pos.y, ty) - pad) / CELL));
    const int x1 = static_cast<int>(std::floor((std::max(f.pos.x, tx) + pad) / CELL));
    const int y1 = static_cast<int>(std::floor((std::max(f.pos.y, ty) + pad) / CELL));
    const int gw = x1 - x0 + 1, gh = y1 - y0 + 1;
    if (gw * gh > ROUTE_CELLS) return;
    const float limit = g.ceiling - g.clear;
    std::vector<unsigned char> high(static_cast<size_t>(gw * gh), 0), blocked(high.size(), 0);
    for (int y = 0; y < gh; y++)
        for (int x = 0; x < gw; x++)
            high[y * gw + x] = cellTop(f, w, x0 + x, y0 + y) > limit;
    for (int y = 0; y < gh; y++)
        for (int x = 0; x < gw; x++) {
            if (!high[y * gw + x]) continue;
            for (int ny = std::max(0, y - 1); ny <= std::min(gh - 1, y + 1); ny++)
                for (int nx = std::max(0, x - 1); nx <= std::min(gw - 1, x + 1); nx++)
                    blocked[ny * gw + nx] = 1;
        }
    auto cellOf = [&](float px, float py, int& cx, int& cy) {
        cx = static_cast<int>(std::floor(px / CELL)) - x0;
        cy = static_cast<int>(std::floor(py / CELL)) - y0;
        return cx >= 0 && cy >= 0 && cx < gw && cy < gh;
    };
    auto isBlocked = [&](float px, float py) {
        int cx, cy;
        return cellOf(px, py, cx, cy) && blocked[cy * gw + cx];
    };
    // Straight there already clear: no corners.
    auto clear = [&](float ax, float ay, float bx, float by) {
        const float len = std::sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
        const int n = std::max(1, static_cast<int>(len / 2.0f));
        for (int i = 1; i < n; i++) {
            const float k = static_cast<float>(i) / n;
            if (isBlocked(ax + (bx - ax) * k, ay + (by - ay) * k)) return false;
        }
        return true;
    };
    if (clear(f.pos.x, f.pos.y, tx, ty)) return;
    int sxc, syc, gxc, gyc;
    cellOf(f.pos.x, f.pos.y, sxc, syc);
    cellOf(tx, ty, gxc, gyc);
    // A goal over a tower: the nearest open cell to it.
    if (blocked[gyc * gw + gxc]) {
        int best = -1;
        float bestD = 1e9f;
        for (int r = 1; r <= 6 && best < 0; r++)
            for (int y = std::max(0, gyc - r); y <= std::min(gh - 1, gyc + r); y++)
                for (int x = std::max(0, gxc - r); x <= std::min(gw - 1, gxc + r); x++) {
                    if (blocked[y * gw + x]) continue;
                    const float dd = static_cast<float>((x - gxc) * (x - gxc) + (y - gyc) * (y - gyc));
                    if (dd < bestD) { bestD = dd; best = y * gw + x; }
                }
        if (best < 0) return;
        gxc = best % gw;
        gyc = best / gw;
    }
    const int start = syc * gw + sxc, goal = gyc * gw + gxc;
    std::vector<float> cost(high.size(), 1e30f);
    std::vector<int> from(high.size(), -1);
    using Node = std::pair<float, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    auto guess = [&](int c) {
        const float ex = static_cast<float>(c % gw - gxc), ey = static_cast<float>(c / gw - gyc);
        return std::sqrt(ex * ex + ey * ey);
    };
    cost[start] = 0.0f;
    open.push({ guess(start), start });
    while (!open.empty()) {
        const auto [est, c] = open.top();
        open.pop();
        if (c == goal) break;
        if (est - guess(c) > cost[c] + 1e-3f) continue;
        const int cx = c % gw, cy = c / gw;
        for (int ny = cy - 1; ny <= cy + 1; ny++)
            for (int nx = cx - 1; nx <= cx + 1; nx++) {
                if ((nx == cx && ny == cy) || nx < 0 || ny < 0 || nx >= gw || ny >= gh) continue;
                const int n = ny * gw + nx;
                if (blocked[n] && n != goal) continue;
                const float step = (nx != cx && ny != cy) ? 1.4142f : 1.0f;
                if (cost[c] + step < cost[n]) {
                    cost[n] = cost[c] + step;
                    from[n] = c;
                    open.push({ cost[n] + guess(n), n });
                }
            }
    }
    if (from[goal] < 0 || cost[goal] * CELL > ROUTE_LONGEST * straight + 60.0f) return;
    std::vector<int> way;
    for (int c = goal; c != start && c >= 0; c = from[c]) way.push_back(c);
    std::reverse(way.begin(), way.end());
    // The furthest point along it it can fly to straight.
    float ax = tx, ay = ty;
    for (int i = static_cast<int>(way.size()) - 1; i >= 0; i--) {
        const float px = (x0 + way[i] % gw + 0.5f) * CELL, py = (y0 + way[i] / gw + 0.5f) * CELL;
        if (clear(f.pos.x, f.pos.y, px, py) || i == 0) {
            ax = px;
            ay = py;
            break;
        }
    }
    f.aim = { ax, ay, to.z };
    f.aiming = true;
    f.aimLeft = cost[goal] * CELL + std::sqrt((tx - to.x) * (tx - to.x) + (ty - to.y) * (ty - to.y));
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

    // Its way round anything above the ceiling: where it heads for now, and how far it
    // has to go that way.
    float hx = ux, hy = uy, left = d, ahead = 1e9f;
    if (g.ceiling > NOTHING / 2.0f && w.top && !landing) {
        f.routeIn -= dt;
        if (f.routeIn <= 0.0f) {
            f.routeIn = ROUTE_EVERY;
            route(f, g, w, g.at);
        }
        if (f.aiming) {
            const float ax = f.aim.x - f.pos.x, ay = f.aim.y - f.pos.y, ad = std::sqrt(ax * ax + ay * ay);
            if (ad > 1.0f) {
                hx = ax / ad;
                hy = ay / ad;
                left = std::max(d, f.aimLeft);
                ahead = ad + CELL;      // beyond the corner, it turns
            } else {
                f.routeIn = 0.0f;   // at the corner: on to the next
            }
        }
    } else {
        f.aiming = false;
    }

    // Look at what is under and ahead of it, now and then: the floor it keeps over.
    f.planIn -= dt;
    if (f.planIn <= 0.0f) {
        f.planIn = PLAN_EVERY;
        const float look = clampf(vh * 3.0f + 30.0f, 30.0f, 150.0f);
        const float slope = CLIMB_SHARE * s.climb / std::max(vh, 8.0f);
        float top = topAt(w, f.pos.x, f.pos.y);
        const float side = f.aiming ? SIDE_ROUND : SIDE;
        if (vh > 1.0f) top = std::max(top, highest(w, f.pos.x, f.pos.y, f.vel.x / vh, f.vel.y / vh, std::min(look, ahead), slope, side));
        if (d > 1.0f) top = std::max(top, highest(w, f.pos.x, f.pos.y, hx, hy, std::min(std::min(look, left), ahead), slope, 0.0f));
        f.floor = top + g.clear;
        // Anything solid beside it, at its height: pushed away from it.
        f.push = {};
        if (w.ray) {
            for (int k = 0; k < KEEP_RAYS; k++) {
                const float a = 2.0f * PI * k / KEEP_RAYS, cx = std::cos(a), cy = std::sin(a);
                float t = 1.0f;
                if (w.ray(f.pos, { f.pos.x + cx * KEEP_OFF, f.pos.y + cy * KEEP_OFF, f.pos.z }, t)) {
                    f.push.x -= cx * (1.0f - t);
                    f.push.y -= cy * (1.0f - t);
                }
            }
            const float pl = std::sqrt(f.push.x * f.push.x + f.push.y * f.push.y);
            if (pl > 1.0f) {
                f.push.x /= pl;
                f.push.y /= pl;
            }
        }
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
    float speed = std::min(cap, std::sqrt(2.0f * s.brake * 0.8f * left));
    if (landing && d < 4.0f) speed = std::min(speed, d * 0.8f);
    float wx = hx * speed + g.vel.x, wy = hy * speed + g.vel.y;
    // Far under the floor (off the ground, or a tower close ahead), it climbs first.
    const float below = f.floor - f.pos.z;
    if (!landing && below > 10.0f) {
        const float k = clampf(1.0f - (below - 10.0f) / 40.0f, 0.3f, 1.0f);
        wx *= k;
        wy *= k;
    }
    // Off what is beside it: no speed toward it, and a push away.
    const float pl = std::sqrt(f.push.x * f.push.x + f.push.y * f.push.y);
    if (pl > 0.01f) {
        const float px = f.push.x / pl, py = f.push.y / pl;
        const float toward = -(wx * px + wy * py);
        if (toward > 0.0f) {
            wx += px * toward;
            wy += py * toward;
        }
        wx += f.push.x * KEEP_PUSH;
        wy += f.push.y * KEEP_PUSH;
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
