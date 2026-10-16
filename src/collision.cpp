#include "collision.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <unordered_map>

namespace dnav {

namespace {

constexpr float MIN_CELL = 8.0f;
constexpr size_t MAX_CELLS = 16u << 20;     // the grid grows coarser past this many columns

Vec3 sub(const Vec3& a, const Vec3& b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
Vec3 cross(const Vec3& a, const Vec3& b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

struct Key {
    uint32_t x, y, z;
    bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
};
struct KeyHash {
    size_t operator()(const Key& k) const { return (k.x * 73856093u) ^ (k.y * 19349663u) ^ (k.z * 83492791u); }
};

uint32_t bits(float f)
{
    uint32_t u;
    std::memcpy(&u, &f, 4);
    return u;
}

} // namespace

bool Collision::load(const std::string& path, std::string& error)
{
    std::ifstream in(path, std::ios::binary);
    char magic[4];
    uint32_t count = 0;
    if (!in || !in.read(magic, 4) || std::memcmp(magic, "DNTR", 4) || !in.read(reinterpret_cast<char*>(&count), 4)) {
        error = path + ": not a .tris file (tools/dnav.py world writes them)";
        return false;
    }
    std::vector<float> raw(static_cast<size_t>(count) * 9);
    if (!in.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(raw.size() * sizeof(float)))) {
        error = path + ": cut short";
        return false;
    }
    build(raw);
    return true;
}

void Collision::build(const std::vector<float>& raw)
{
    verts_.clear();
    tris_.clear();
    // The same corner shared by several faces is stored once.
    std::unordered_map<Key, uint32_t, KeyHash> seen;
    seen.reserve(raw.size() / 6);
    tris_.reserve(raw.size() / 3);
    for (size_t f = 0; f + 9 <= raw.size(); f += 9) {
        uint32_t idx[3];
        for (int k = 0; k < 3; k++) {
            const float* p = &raw[f + k * 3];
            const Key key { bits(p[0]), bits(p[1]), bits(p[2]) };
            auto it = seen.find(key);
            if (it == seen.end()) {
                it = seen.emplace(key, static_cast<uint32_t>(verts_.size())).first;
                verts_.push_back({ p[0], p[1], p[2] });
            }
            idx[k] = it->second;
        }
        if (idx[0] == idx[1] || idx[1] == idx[2] || idx[0] == idx[2]) continue;
        tris_.insert(tris_.end(), idx, idx + 3);
    }
    start_.clear();
    faces_.clear();
    w_ = h_ = 0;
    if (tris_.empty()) return;

    float x1 = verts_[0].x, y1 = verts_[0].y;
    x0_ = x1;
    y0_ = y1;
    for (const Vec3& v : verts_) {
        x0_ = std::min(x0_, v.x);
        y0_ = std::min(y0_, v.y);
        x1 = std::max(x1, v.x);
        y1 = std::max(y1, v.y);
    }
    cell_ = MIN_CELL;
    for (;;) {
        w_ = static_cast<int>((x1 - x0_) / cell_) + 1;
        h_ = static_cast<int>((y1 - y0_) / cell_) + 1;
        if (static_cast<size_t>(w_) * static_cast<size_t>(h_) <= MAX_CELLS) break;
        cell_ *= 2.0f;
    }
    // Two passes: count each column's faces, then fill them in.
    auto span = [&](size_t t, int& cx0, int& cy0, int& cx1, int& cy1) {
        const Vec3 &a = verts_[tris_[t]], &b = verts_[tris_[t + 1]], &c = verts_[tris_[t + 2]];
        cx0 = static_cast<int>((std::min({ a.x, b.x, c.x }) - x0_) / cell_);
        cy0 = static_cast<int>((std::min({ a.y, b.y, c.y }) - y0_) / cell_);
        cx1 = std::min(w_ - 1, static_cast<int>((std::max({ a.x, b.x, c.x }) - x0_) / cell_));
        cy1 = std::min(h_ - 1, static_cast<int>((std::max({ a.y, b.y, c.y }) - y0_) / cell_));
    };
    start_.assign(static_cast<size_t>(w_) * h_ + 1, 0);
    for (size_t t = 0; t < tris_.size(); t += 3) {
        int cx0, cy0, cx1, cy1;
        span(t, cx0, cy0, cx1, cy1);
        for (int y = cy0; y <= cy1; y++)
            for (int x = cx0; x <= cx1; x++) start_[static_cast<size_t>(y) * w_ + x + 1]++;
    }
    for (size_t i = 1; i < start_.size(); i++) start_[i] += start_[i - 1];
    faces_.resize(start_.back());
    std::vector<uint32_t> fill(start_.begin(), start_.end() - 1);
    for (size_t t = 0; t < tris_.size(); t += 3) {
        int cx0, cy0, cx1, cy1;
        span(t, cx0, cy0, cx1, cy1);
        for (int y = cy0; y <= cy1; y++)
            for (int x = cx0; x <= cx1; x++) faces_[fill[static_cast<size_t>(y) * w_ + x]++] = static_cast<uint32_t>(t / 3);
    }
}

bool Collision::ray(const Vec3& a, const Vec3& b, RayHit& hit) const { return cast(a, b, hit, 0); }
bool Collision::wall(const Vec3& a, const Vec3& b, RayHit& hit) const { return cast(a, b, hit, 1); }

bool Collision::ground(float x, float y, float z, float up, float down, float& out) const
{
    RayHit hit;
    if (!cast({ x, y, z + up }, { x, y, z - down }, hit, 2)) return false;
    out = hit.at.z;
    return true;
}

// want: 0 any face, 1 walls only, 2 ground only.
bool Collision::cast(const Vec3& a, const Vec3& b, RayHit& hit, int want) const
{
    if (tris_.empty()) return false;
    const Vec3 d = sub(b, a);
    float best = 2.0f;
    Vec3 bestN {};
    auto test = [&](uint32_t f) {
        const Vec3 &p0 = verts_[tris_[f * 3]], &p1 = verts_[tris_[f * 3 + 1]], &p2 = verts_[tris_[f * 3 + 2]];
        // Moller-Trumbore, both sides.
        const Vec3 e1 = sub(p1, p0), e2 = sub(p2, p0);
        const Vec3 pv = cross(d, e2);
        const float det = dot(e1, pv);
        if (std::fabs(det) < 1e-9f) return;
        const float inv = 1.0f / det;
        const Vec3 tv = sub(a, p0);
        const float u = dot(tv, pv) * inv;
        if (u < 0.0f || u > 1.0f) return;
        const Vec3 qv = cross(tv, e1);
        const float v = dot(d, qv) * inv;
        if (v < 0.0f || u + v > 1.0f) return;
        const float t = dot(e2, qv) * inv;
        if (t < 0.0f || t > 1.0f || t >= best) return;
        Vec3 n = cross(e1, e2);
        const float len = std::sqrt(dot(n, n));
        if (len < 1e-12f) return;
        n = { n.x / len, n.y / len, n.z / len };
        if (dot(n, d) > 0.0f) n = { -n.x, -n.y, -n.z };   // toward the ray's start
        if (want == 1 && std::fabs(n.z) >= GROUND_NZ) return;
        if (want == 2 && n.z < GROUND_NZ) return;
        best = t;
        bestN = n;
    };
    auto column = [&](int cx, int cy) {
        if (cx < 0 || cy < 0 || cx >= w_ || cy >= h_) return;
        const size_t i = static_cast<size_t>(cy) * w_ + cx;
        for (uint32_t k = start_[i]; k < start_[i + 1]; k++) test(faces_[k]);
    };

    // Walk the columns the ray crosses from above, in order, until a hit lies
    // within the columns already walked.
    const float fx = (a.x - x0_) / cell_, fy = (a.y - y0_) / cell_;
    int cx = static_cast<int>(std::floor(fx)), cy = static_cast<int>(std::floor(fy));
    const float dx = d.x / cell_, dy = d.y / cell_;
    const int sx = dx > 0.0f ? 1 : -1, sy = dy > 0.0f ? 1 : -1;
    const float inf = 1e30f;
    const float tdx = std::fabs(dx) > 1e-9f ? std::fabs(1.0f / dx) : inf;
    const float tdy = std::fabs(dy) > 1e-9f ? std::fabs(1.0f / dy) : inf;
    float tx = std::fabs(dx) > 1e-9f ? ((dx > 0.0f ? (cx + 1 - fx) : (fx - cx)) * tdx) : inf;
    float ty = std::fabs(dy) > 1e-9f ? ((dy > 0.0f ? (cy + 1 - fy) : (fy - cy)) * tdy) : inf;
    for (int guard = 0; guard < 1 << 20; guard++) {
        column(cx, cy);
        const float exit = std::min(tx, ty);
        if (best <= exit || exit >= 1.0f) break;
        if (tx < ty) {
            cx += sx;
            tx += tdx;
        } else {
            cy += sy;
            ty += tdy;
        }
    }
    if (best > 1.0f) return false;
    hit.t = best;
    hit.at = { a.x + d.x * best, a.y + d.y * best, a.z + d.z * best };
    hit.normal = bestN;
    return true;
}

} // namespace dnav
