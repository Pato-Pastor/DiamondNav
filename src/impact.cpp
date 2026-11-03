#include "impact.hpp"

#include <algorithm>
#include <cmath>

namespace dnav {

namespace {

constexpr float DENSITY = 110.0f;       // kg per m^3 of its box: a Sultan (4.6 x 2 x 1.4) comes to about 1400
constexpr float GRAVITY = 9.81f;
constexpr float YAW_DAMP = 2.5f;        // 1/s
constexpr float TIE = 0.15f;            // m: vertices this near the deepest one share the contact

float dot(float ax, float ay, float bx, float by) { return ax * bx + ay * by; }

void corners(const Obb& b, float out[4][2])
{
    const float rx = b.fy, ry = -b.fx;
    int k = 0;
    for (float sl : { 1.0f, -1.0f })
        for (float sw : { 1.0f, -1.0f }) {
            out[k][0] = b.cx + b.fx * b.hl * sl + rx * b.hw * sw;
            out[k][1] = b.cy + b.fy * b.hl * sl + ry * b.hw * sw;
            k++;
        }
}

} // namespace

float massOf(float length, float width, float height)
{
    return std::clamp(length * width * height * DENSITY, 150.0f, 25000.0f);
}

float inertiaOf(float mass, float length, float width)
{
    return mass * (length * length + width * width) / 12.0f;
}

bool contact(const Obb& a, const Obb& b, Contact& c)
{
    const float ar[2] = { a.fy, -a.fx }, br[2] = { b.fy, -b.fx };
    const float axes[4][2] = { { a.fx, a.fy }, { ar[0], ar[1] }, { b.fx, b.fy }, { br[0], br[1] } };
    const float dx = b.cx - a.cx, dy = b.cy - a.cy;
    float best = 1e30f, nx = 0.0f, ny = 0.0f;
    bool onA = true;
    for (int k = 0; k < 4; k++) {
        const float ux = axes[k][0], uy = axes[k][1];
        const float ra = a.hl * std::fabs(dot(a.fx, a.fy, ux, uy)) + a.hw * std::fabs(dot(ar[0], ar[1], ux, uy));
        const float rb = b.hl * std::fabs(dot(b.fx, b.fy, ux, uy)) + b.hw * std::fabs(dot(br[0], br[1], ux, uy));
        const float dist = dot(dx, dy, ux, uy);
        const float overlap = ra + rb - std::fabs(dist);
        if (overlap <= 0.0f) return false;
        if (overlap < best) {
            best = overlap;
            nx = dist >= 0.0f ? ux : -ux;
            ny = dist >= 0.0f ? uy : -uy;
            onA = k < 2;
        }
    }
    // The vertices of the box poking in that go deepest (the middle of them, for a face
    // against a face), then kept inside the box they poke into.
    float v[4][2];
    corners(onA ? b : a, v);
    float lo = 1e30f;
    const float sign = onA ? 1.0f : -1.0f;      // deepest along +n into a, or along -n into b
    for (auto& p : v) lo = std::min(lo, sign * dot(p[0], p[1], nx, ny));
    float sx = 0.0f, sy = 0.0f;
    int n = 0;
    for (auto& p : v)
        if (sign * dot(p[0], p[1], nx, ny) <= lo + TIE) {
            sx += p[0];
            sy += p[1];
            n++;
        }
    sx /= n;
    sy /= n;
    const Obb& ref = onA ? a : b;
    const float rx = ref.fy, ry = -ref.fx;
    const float along = std::clamp(dot(sx - ref.cx, sy - ref.cy, ref.fx, ref.fy), -ref.hl, ref.hl);
    const float across = std::clamp(dot(sx - ref.cx, sy - ref.cy, rx, ry), -ref.hw, ref.hw);
    c.nx = nx;
    c.ny = ny;
    c.depth = best;
    c.px = ref.cx + ref.fx * along + rx * across;
    c.py = ref.cy + ref.fy * along + ry * across;
    return true;
}

float collide(Body& a, const Body& b, const Contact& c, float e, float minClosing)
{
    const float rax = c.px - a.x, ray = c.py - a.y, rbx = c.px - b.x, rby = c.py - b.y;
    const float vax = a.vx - a.yaw * ray, vay = a.vy + a.yaw * rax;
    const float vbx = b.vx - b.yaw * rby, vby = b.vy + b.yaw * rbx;
    const float vn = (vbx - vax) * c.nx + (vby - vay) * c.ny;       // < 0: closing
    if (vn > -minClosing) return 0.0f;
    const float ran = rax * c.ny - ray * c.nx, rbn = rbx * c.ny - rby * c.nx;
    const float k = 1.0f / a.mass + 1.0f / b.mass + ran * ran / a.inertia + rbn * rbn / b.inertia;
    const float j = -(1.0f + e) * vn / k;
    a.vx -= c.nx * j / a.mass;
    a.vy -= c.ny * j / a.mass;
    a.yaw -= ran * j / a.inertia;
    return j;
}

void slide(Body& a, float dt, float mu)
{
    const float s = std::sqrt(a.vx * a.vx + a.vy * a.vy);
    if (s > 1e-4f) {
        const float ns = std::max(0.0f, s - mu * GRAVITY * dt);
        a.vx *= ns / s;
        a.vy *= ns / s;
    }
    a.yaw *= std::max(0.0f, 1.0f - YAW_DAMP * dt);
}

void roll(Body& a, float headingDeg, float dt, float muRoll, float muSide)
{
    const float h = headingDeg * 0.017453292519943295f;
    const float fx = -std::sin(h), fy = std::cos(h);        // forward; right is (fy, -fx)
    float vf = a.vx * fx + a.vy * fy;
    float vs = a.vx * fy - a.vy * fx;
    auto towardZero = [](float v, float d) { return v > 0.0f ? std::max(0.0f, v - d) : std::min(0.0f, v + d); };
    vf = towardZero(vf, muRoll * GRAVITY * dt);
    vs = towardZero(vs, muSide * GRAVITY * dt);
    a.vx = vf * fx + vs * fy;
    a.vy = vf * fy - vs * fx;
    const float speed = std::sqrt(a.vx * a.vx + a.vy * a.vy);
    a.yaw *= std::max(0.0f, 1.0f - (YAW_DAMP + 0.25f * speed) * dt);
}

} // namespace dnav