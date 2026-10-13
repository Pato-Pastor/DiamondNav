#include "driving.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace dnav {

namespace {

constexpr float KMH = 1.0f / 3.6f;
constexpr float FILLET_MAX = 8.0f;      // a sharp corner is rounded over at most this much of each leg
constexpr float MIN_SPEED = 2.0f;       // no point is planned slower than this (m/s)
constexpr float STRAIGHT = 1e9f;

// limit, grip, brake, accel, junction
constexpr StyleUse STYLES[] = {
    { 0.85f, 0.45f, 0.45f, 0.55f, 0.60f },  // Cautious
    { 1.00f, 0.60f, 0.60f, 0.75f, 0.75f },  // Normal
    { 1.25f, 0.80f, 0.80f, 1.00f, 0.90f },  // Fast
    { 0.00f, 0.90f, 0.90f, 1.00f, 1.00f },  // Pursuit
};

} // namespace

const StyleUse& styleUse(DriveStyle s)
{
    const int i = std::clamp(static_cast<int>(s), 0, static_cast<int>(DriveStyle::Count) - 1);
    return STYLES[i];
}

const char* styleName(DriveStyle s)
{
    switch (s) {
    case DriveStyle::Cautious: return "cautious";
    case DriveStyle::Normal: return "normal";
    case DriveStyle::Fast: return "fast";
    case DriveStyle::Pursuit: return "pursuit";
    default: return "?";
    }
}

const char* className(RoadClass c)
{
    switch (c) {
    case RoadClass::Highway: return "highway";
    case RoadClass::Main: return "main road";
    case RoadClass::Street: return "street";
    case RoadClass::Minor: return "minor road";
    case RoadClass::Service: return "service road";
    case RoadClass::OffRoad: return "off-road";
    default: return "?";
    }
}

Driving::Driving()
{
    setLimit(RoadClass::Highway, 130.0f * KMH);
    setLimit(RoadClass::Main, 80.0f * KMH);
    setLimit(RoadClass::Street, 60.0f * KMH);
    setLimit(RoadClass::Minor, 45.0f * KMH);
    setLimit(RoadClass::Service, 30.0f * KMH);
    setLimit(RoadClass::OffRoad, 60.0f * KMH);
}

bool Driving::loadVehicles(const std::string& path, std::string& error)
{
    std::ifstream in(path);
    if (!in) {
        error = path + ": cannot open";
        return false;
    }
    std::string line;
    if (!std::getline(in, line)) {
        error = path + ": empty";
        return false;
    }
    std::istringstream head(line);
    std::string magic;
    int version = 0;
    head >> magic >> version;
    if (magic != "DNVH") {
        error = path + ": not a DiamondNav vehicles file";
        return false;
    }
    if (version != 1) {
        error = path + ": vehicles file version " + std::to_string(version)
            + ", this build reads 1 (rebuild it with tools/dnav.py vehicles)";
        return false;
    }
    std::unordered_map<int, Handling> table;
    int no = 1;
    while (std::getline(in, line)) {
        no++;
        const size_t hash = line.find('#');
        if (hash != std::string::npos) line.resize(hash);
        std::istringstream t(line);
        int model;
        Handling h;
        if (!(t >> model)) continue;            // blank or a comment
        if (!(t >> h.top >> h.accel >> h.brake >> h.grip >> h.radius) || h.top <= 0.0f || h.accel <= 0.0f
            || h.brake <= 0.0f || h.grip <= 0.0f || h.radius <= 0.0f) {
            error = path + ":" + std::to_string(no) + ": takes MODEL TOP ACCEL BRAKE GRIP RADIUS, all above 0";
            return false;
        }
        table[model] = h;
    }
    vehicles_ = std::move(table);
    return true;
}

Handling Driving::handling(int model) const
{
    auto o = overrides_.find(model);
    if (o != overrides_.end()) return o->second;
    auto v = vehicles_.find(model);
    if (v != vehicles_.end()) return v->second;
    return Handling {};
}

float Driving::cornerRadius(const std::vector<Point>& p, size_t i)
{
    if (i == 0 || i + 1 >= p.size()) return STRAIGHT;
    const float ax = p[i].x - p[i - 1].x, ay = p[i].y - p[i - 1].y;
    const float bx = p[i + 1].x - p[i].x, by = p[i + 1].y - p[i].y;
    const float la = std::hypot(ax, ay), lb = std::hypot(bx, by);
    if (la < 1e-3f || lb < 1e-3f) return STRAIGHT;
    const float turn = std::acos(std::clamp((ax * bx + ay * by) / (la * lb), -1.0f, 1.0f));
    if (turn < 0.01f) return STRAIGHT;
    // The circle that meets both legs `d` from the corner: on a rounded road the
    // legs are the curve's own short steps and this is its curvature; on a sharp
    // corner it is how a car rounds it.
    const float d = std::min({ la, lb, 2.0f * FILLET_MAX }) * 0.5f;
    return d / std::tan(turn * 0.5f);
}

float Driving::roadSpeed(RoadClass c, const Handling& car, DriveStyle style, float cap) const
{
    const StyleUse& use = styleUse(style);
    float v = car.top;
    if (cap > 0.0f) v = std::min(v, cap);
    if (use.limit > 0.0f) v = std::min(v, limit(c) * use.limit);
    return v;
}

std::vector<float> Driving::plan(const std::vector<Point>& points, const std::vector<RoadClass>& classes,
    const std::vector<uint8_t>& junctions, const Handling& car, DriveStyle style, float cap) const
{
    const StyleUse& use = styleUse(style);
    std::vector<float> out(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        const RoadClass c = i < classes.size() ? classes[i] : RoadClass::Street;
        float v = roadSpeed(c, car, style, cap);
        const float r = std::max(cornerRadius(points, i), car.radius);
        v = std::min(v, std::sqrt(car.grip * use.grip * r));
        // Junctions slow a driver who keeps to the limits; a highway's are merges.
        if (use.limit > 0.0f && c != RoadClass::Highway && i < junctions.size() && junctions[i])
            v = std::min(v, limit(c) * use.junction);
        out[i] = std::max(v, MIN_SPEED);
    }
    return out;
}

} // namespace dnav
