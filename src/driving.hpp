// How NPC cars drive: each model's handling (navmesh/vehicles.dnv, tools/dnav.py
// vehicles), each road class's speed limit, the driving styles, and the speed plan
// that turns a path into the fastest speed allowed at each of its points.
//
// The component steers with these (component.cpp, drive()); nothing here touches
// open.mp, so the plan can be tested on its own.

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "pathfinder.hpp"

namespace dnav {

// How a model drives, in metres and seconds.
struct Handling {
    float top = 45.0f;          // top speed, m/s
    float accel = 3.5f;         // pulling away, m/s^2 (it fades towards the top speed)
    float brake = 7.0f;         // the hardest it brakes, m/s^2
    float grip = 7.5f;          // the most sideways acceleration it holds in a corner, m/s^2
    float radius = 6.0f;        // its tightest turning circle, m
};

enum class DriveStyle : int {
    Cautious = 0,               // well under the limits, gentle corners, early braking
    Normal = 1,                 // the limits, comfortable corners
    Fast = 2,                   // a quarter over the limits, brisk corners, late braking
    Pursuit = 3,                // no limits: as fast as the car and the corners allow
    Count = 4,
};

// What a style takes of the limits and of the car.
struct StyleUse {
    float limit;                // share of each road's speed limit (0: ignores limits)
    float grip;                 // share of the car's grip it uses in corners
    float brake;                // share of the car's braking it plans with
    float accel;                // share of the car's acceleration it uses
    float junction;             // share of the limit it crosses a junction at
};

const StyleUse& styleUse(DriveStyle s);
const char* styleName(DriveStyle s);
const char* className(RoadClass c);

class Driving {
public:
    Driving();

    // navmesh/vehicles.dnv; false (with `error`) if it is there but unreadable.
    bool loadVehicles(const std::string& path, std::string& error);
    size_t vehicleCount() const { return vehicles_.size(); }

    // A model's handling: set by the script, from vehicles.dnv, or the default.
    Handling handling(int model) const;
    void setHandling(int model, const Handling& h) { overrides_[model] = h; }

    // Speed limits by road class, m/s.
    float limit(RoadClass c) const { return limits_[static_cast<size_t>(c)]; }
    void setLimit(RoadClass c, float ms) { limits_[static_cast<size_t>(c)] = ms; }

    // The fastest this car may go on a road of class `c` in this style (m/s): the
    // style's share of the limit, the car's top speed and `cap` (0: none).
    float roadSpeed(RoadClass c, const Handling& car, DriveStyle style, float cap) const;

    // The fastest a car may go at each point of `points` (m/s): the road's limit for
    // the style, its corners for the car's grip, junctions, the car's top speed and
    // `cap` (0: none). Braking ahead of each point is the driver's job.
    std::vector<float> plan(const std::vector<Point>& points, const std::vector<RoadClass>& classes,
        const std::vector<uint8_t>& junctions, const Handling& car, DriveStyle style, float cap) const;

    // How tight a path turns at point i (m): the circle a car follows round it.
    static float cornerRadius(const std::vector<Point>& points, size_t i);

private:
    std::array<float, static_cast<size_t>(RoadClass::Count)> limits_;
    std::unordered_map<int, Handling> vehicles_, overrides_;
};

} // namespace dnav
