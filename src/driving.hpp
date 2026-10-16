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

constexpr int ACCEL_POINTS = 16;        // vehicles.dnv's acceleration samples, 0 to top speed

// How a model drives, in metres and seconds (tools/dnavlib/vehicles.py works these
// out from the game's handling.cfg, surface.dat and each model's DFF).
struct Handling {
    float top = 45.0f;          // top speed on the flat, m/s
    float brake = 7.0f;         // full brakes on a road, m/s^2
    float brakeOff = 5.0f;      // ... on dirt
    float coast = 2.0f;         // rolling with no throttle, m/s^2 (before the air)
    float drag = 0.0f;          // the air's deceleration per (m/s)^2
    float grip = 7.5f;          // the most sideways acceleration it holds in a corner on a road, m/s^2
    float gripOff = 5.3f;       // ... on dirt
    float radius = 6.0f;        // the tightest circle its centre turns, m
    float yaw = 4.0f;           // how fast it can start (or stop) turning, rad/s^2
    float reverse = 12.0f;      // top speed backing up, m/s
    float length = 4.5f, width = 1.9f, height = 1.5f, wheelbase = 2.6f;    // m
    float ride = 0.0f;          // its centre over the road, m (0: not known, see rideHeight)
    // Full-throttle acceleration (m/s^2) at top * i / (ACCEL_POINTS - 1): the gears,
    // the engine, the air and how much of it the tyres put down, on a road and on dirt.
    std::array<float, ACCEL_POINTS> accel {}, accelOff {};

    Handling() { *this = simple(45.0f, 3.5f, 7.0f, 7.5f, 6.0f); }

    // A car described by five numbers (vehicles.dnv version 1, Nav_SetVehicleHandling):
    // its acceleration fades towards its top speed, dirt takes a third of its grip.
    static Handling simple(float top, float accel, float brake, float grip, float radius);

    // Full-throttle acceleration at speed v (m/s).
    float accelAt(float v, bool offRoad) const;
    // Seconds from one speed to another at full throttle on a road (0 when it never gets there).
    float accelTime(float from, float to) const;

private:
    struct Raw {};
    explicit Handling(Raw) {}
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
    // Off the road it corners on its grip on dirt.
    std::vector<float> plan(const std::vector<Point>& points, const std::vector<RoadClass>& classes,
        const std::vector<uint8_t>& junctions, const Handling& car, DriveStyle style, float cap) const;

    // How tight a path turns at point i (m): the circle a car follows round it.
    static float cornerRadius(const std::vector<Point>& points, size_t i);

private:
    std::array<float, static_cast<size_t>(RoadClass::Count)> limits_;
    std::unordered_map<int, Handling> vehicles_, overrides_;
};

} // namespace dnav
