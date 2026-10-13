// How cars drive, without a server: road classes from the road graph's flags, the
// corner radius of a path, the speed plan for each style, and vehicles.dnv. Run by
// ctest (or build/dnav_driving_tests directly).

#include "driving.hpp"
#include "roads.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
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

static bool near(float a, float b, float tolerance)
{
    return std::fabs(a - b) <= tolerance;
}

static constexpr float KMH = 1.0f / 3.6f;

// A graph of separate two-node roads, one per case, written the way dnav.py roads does.
struct Case {
    uint16_t a, b;          // the two nodes' flags
    uint8_t lanes, back;
};

static void classes()
{
    const uint16_t low = ROAD_DETAIL | (3 << 4), busy = ROAD_DETAIL | (1 << 4);
    const std::vector<Case> cases = {
        { ROAD_HIGHWAY, ROAD_HIGHWAY, 2, 0 },               // 0 highway
        { ROAD_HIGHWAY, 0, 1, 1 },                          // 1 a slip road off it: a street
        { 0, 0, 2, 2 },                                     // 2 two lanes each way: main
        { 0, 0, 2, 0 },                                     // 3 one-way, two lanes: main
        { 0, 0, 1, 1 },                                     // 4 street
        { low, low, 1, 1 },                                 // 5 GTA's low traffic: minor
        { low, busy, 1, 1 },                                // 6 low at one end only: street
        { 0, ROAD_EMERGENCY_ONLY, 1, 1 },                   // 7 service
        { ROAD_MINOR, 0, 2, 2 },                            // 8 marked minor, even with lanes
        { ROAD_MAIN, ROAD_MAIN, 1, 1 },                     // 9 marked main
        { low, low, 2, 2 },                                 // 10 lanes beat a low traffic level
    };
    const RoadClass want[] = { RoadClass::Highway, RoadClass::Street, RoadClass::Main, RoadClass::Main,
        RoadClass::Street, RoadClass::Minor, RoadClass::Street, RoadClass::Service, RoadClass::Minor,
        RoadClass::Main, RoadClass::Main };

    std::vector<RoadNode> nodes;
    std::vector<RoadEdge> edges;
    for (size_t i = 0; i < cases.size(); i++) {
        const float y = static_cast<float>(i) * 100.0f;
        const uint32_t a = static_cast<uint32_t>(nodes.size());
        nodes.push_back(RoadNode { 0.0f, y, 10.0f, static_cast<uint32_t>(edges.size()), 1, cases[i].a });
        edges.push_back(RoadEdge { a + 1, 0.0f, cases[i].lanes, cases[i].back, 0 });
        nodes.push_back(RoadNode { 50.0f, y, 10.0f, static_cast<uint32_t>(edges.size()), 1, cases[i].b });
        edges.push_back(RoadEdge { a, 0.0f, cases[i].back, cases[i].lanes, 0 });
    }
    {
        std::ofstream out("driving_classes.dnr", std::ios::binary);
        const uint32_t head[4] = { ROADS_MAGIC, ROADS_VERSION, static_cast<uint32_t>(nodes.size()),
            static_cast<uint32_t>(edges.size()) };
        out.write(reinterpret_cast<const char*>(head), sizeof head);
        out.write(reinterpret_cast<const char*>(nodes.data()), nodes.size() * sizeof(RoadNode));
        out.write(reinterpret_cast<const char*>(edges.data()), edges.size() * sizeof(RoadEdge));
    }
    RoadGraph g;
    std::string error;
    CHECK(g.load("driving_classes.dnr", error));
    for (size_t i = 0; i < cases.size(); i++) {
        const uint32_t a = static_cast<uint32_t>(i * 2);
        const RoadClass got = g.classOf(a, edges[nodes[a].firstEdge]);
        if (got != want[i]) std::fprintf(stderr, "case %d: class %d, want %d\n", static_cast<int>(i),
            static_cast<int>(got), static_cast<int>(want[i]));
        CHECK(got == want[i]);
        // A route along it carries the class on every point.
        RouteOptions opt;
        opt.emergency = true;
        std::vector<RoutePoint> pts;
        const float y = static_cast<float>(i) * 100.0f;
        if (cases[i].lanes) {
            CHECK(g.route(0, y, 10, 50, y, 10, opt, pts) == RouteStatus::Ok);
            for (const auto& p : pts) CHECK(p.cls == want[i]);
        }
    }
}

// A route through a junction marks it, and a turn at the junction is rounded.
static void junctions()
{
    //  0 (0,0) - 1 (100,0) - 2 (200,0), and 3 (100,100) off node 1.
    std::vector<RoadNode> nodes = {
        { 0, 0, 10, 0, 1, 0 }, { 100, 0, 10, 1, 3, 0 }, { 200, 0, 10, 4, 1, 0 }, { 100, 100, 10, 5, 1, 0 },
    };
    std::vector<RoadEdge> edges = {
        { 1, 0, 1, 1, 0 },
        { 0, 0, 1, 1, 0 }, { 2, 0, 1, 1, 0 }, { 3, 0, 1, 1, 0 },
        { 1, 0, 1, 1, 0 },
        { 1, 0, 1, 1, 0 },
    };
    {
        std::ofstream out("driving_junction.dnr", std::ios::binary);
        const uint32_t head[4] = { ROADS_MAGIC, ROADS_VERSION, 4, 6 };
        out.write(reinterpret_cast<const char*>(head), sizeof head);
        out.write(reinterpret_cast<const char*>(nodes.data()), nodes.size() * sizeof(RoadNode));
        out.write(reinterpret_cast<const char*>(edges.data()), edges.size() * sizeof(RoadEdge));
    }
    RoadGraph g;
    std::string error;
    CHECK(g.load("driving_junction.dnr", error));
    std::vector<RoutePoint> pts;
    for (const float tx : { 200.0f, 100.0f }) {
        const float ty = tx > 150.0f ? 0.0f : 100.0f;
        CHECK(g.route(0, 0, 10, tx, ty, 10, {}, pts) == RouteStatus::Ok);
        int marked = 0;
        for (const auto& p : pts)
            if (p.junction) {
                marked++;
                CHECK(std::fabs(p.x - 100.0f) < 12.0f && std::fabs(p.y) < 12.0f);
            }
        CHECK(marked == 1);
    }
}

static void corners()
{
    // A circle of radius 20 m, a point every 10 degrees.
    std::vector<Point> arc;
    for (int k = 0; k <= 9; k++) {
        const float a = static_cast<float>(k) * 10.0f * 3.14159265f / 180.0f;
        arc.push_back(Point { 20.0f * std::cos(a), 20.0f * std::sin(a), 0.0f });
    }
    for (size_t i = 1; i + 1 < arc.size(); i++) CHECK(near(Driving::cornerRadius(arc, i), 20.0f, 0.5f));
    // A sharp 90 degree corner between long legs: rounded over 8 m of each.
    const std::vector<Point> sharp = { { 0, 0, 0 }, { 100, 0, 0 }, { 100, 100, 0 } };
    CHECK(near(Driving::cornerRadius(sharp, 1), 8.0f, 0.01f));
    // Short legs round it tighter: 2 m legs, 1 m of each.
    const std::vector<Point> tight = { { 0, 0, 0 }, { 2, 0, 0 }, { 2, 2, 0 } };
    CHECK(near(Driving::cornerRadius(tight, 1), 1.0f, 0.01f));
    // Straight on, and the two ends: no corner.
    const std::vector<Point> line = { { 0, 0, 0 }, { 50, 0, 0 }, { 100, 0, 0 } };
    CHECK(Driving::cornerRadius(line, 1) > 1e6f);
    CHECK(Driving::cornerRadius(line, 0) > 1e6f);
    CHECK(Driving::cornerRadius(line, 2) > 1e6f);
}

static void plans()
{
    Driving d;
    Handling sports { 60.0f, 6.0f, 10.0f, 9.0f, 5.0f };
    Handling truck { 30.0f, 1.5f, 4.0f, 5.0f, 9.0f };
    const std::vector<Point> line = { { 0, 0, 0 }, { 500, 0, 0 }, { 1000, 0, 0 } };
    const std::vector<RoadClass> highway(3, RoadClass::Highway), street(3, RoadClass::Street);
    const std::vector<uint8_t> none(3, 0);

    // The limit for the style, and never over the car's top speed.
    CHECK(near(d.plan(line, highway, none, sports, DriveStyle::Normal, 0.0f)[1], 130.0f * KMH, 0.01f));
    CHECK(near(d.plan(line, highway, none, sports, DriveStyle::Fast, 0.0f)[1], 130.0f * KMH * 1.25f, 0.01f));
    CHECK(near(d.plan(line, highway, none, Handling { 40.0f, 6.0f, 10.0f, 9.0f, 5.0f }, DriveStyle::Fast, 0.0f)[1],
        40.0f, 0.01f));     // 162 km/h is past this car's top speed
    CHECK(near(d.plan(line, highway, none, sports, DriveStyle::Cautious, 0.0f)[1], 130.0f * KMH * 0.85f, 0.01f));
    CHECK(near(d.plan(line, highway, none, truck, DriveStyle::Normal, 0.0f)[1], 30.0f, 0.01f));
    CHECK(near(d.plan(line, street, none, sports, DriveStyle::Fast, 0.0f)[1], 60.0f * KMH * 1.25f, 0.01f));
    // Pursuit ignores limits; a cap still holds.
    CHECK(near(d.plan(line, street, none, sports, DriveStyle::Pursuit, 0.0f)[1], 60.0f, 0.01f));
    CHECK(near(d.plan(line, street, none, sports, DriveStyle::Pursuit, 20.0f)[1], 20.0f, 0.01f));
    // A changed limit.
    d.setLimit(RoadClass::Street, 36.0f * KMH);
    CHECK(near(d.plan(line, street, none, sports, DriveStyle::Normal, 0.0f)[1], 10.0f, 0.01f));
    d.setLimit(RoadClass::Street, 60.0f * KMH);

    // A sharp corner: as fast as the style's share of the car's grip holds on 8 m.
    const std::vector<Point> sharp = { { 0, 0, 0 }, { 100, 0, 0 }, { 100, 100, 0 } };
    const auto normal = d.plan(sharp, street, none, sports, DriveStyle::Normal, 0.0f);
    CHECK(near(normal[1], std::sqrt(9.0f * 0.6f * 8.0f), 0.01f));
    const auto pursuit = d.plan(sharp, street, none, sports, DriveStyle::Pursuit, 0.0f);
    CHECK(near(pursuit[1], std::sqrt(9.0f * 0.9f * 8.0f), 0.01f));
    CHECK(pursuit[1] > normal[1]);
    // The truck turns no tighter than its own circle (9 m), on less grip.
    const auto heavy = d.plan(sharp, street, none, truck, DriveStyle::Normal, 0.0f);
    CHECK(near(heavy[1], std::sqrt(5.0f * 0.6f * 9.0f), 0.01f));

    // A junction: a share of the limit, except in pursuit.
    const std::vector<uint8_t> junction = { 0, 1, 0 };
    CHECK(near(d.plan(line, street, junction, sports, DriveStyle::Normal, 0.0f)[1], 60.0f * KMH * 0.75f, 0.01f));
    CHECK(near(d.plan(line, street, junction, sports, DriveStyle::Pursuit, 0.0f)[1], 60.0f, 0.01f));
    CHECK(near(d.plan(line, highway, junction, sports, DriveStyle::Normal, 0.0f)[1], 130.0f * KMH, 0.01f));
}

static void vehicles()
{
    {
        std::ofstream out("driving_vehicles.dnv");
        out << "DNVH 1\n# model top accel brake grip radius\n"
               "560 50.0 5.0 9.0 8.5 5.2  # sultan\n\n"
               "403 33.0 1.6 4.0 5.0 9.1  # linerun\n";
    }
    Driving d;
    std::string error;
    CHECK(d.loadVehicles("driving_vehicles.dnv", error));
    CHECK(d.vehicleCount() == 2);
    CHECK(near(d.handling(560).top, 50.0f, 0.001f) && near(d.handling(560).radius, 5.2f, 0.001f));
    CHECK(near(d.handling(403).accel, 1.6f, 0.001f));
    // An unknown model drives as the default car; a script's handling wins.
    CHECK(near(d.handling(411).top, Handling {}.top, 0.001f));
    d.setHandling(560, Handling { 20.0f, 2.0f, 3.0f, 4.0f, 6.0f });
    CHECK(near(d.handling(560).top, 20.0f, 0.001f));

    { std::ofstream("driving_bad1.dnv") << "DNVH 2\n560 1 1 1 1 1\n"; }
    { std::ofstream("driving_bad2.dnv") << "DNVH 1\n560 50 5 9\n"; }
    { std::ofstream("driving_bad3.dnv") << "something else\n"; }
    { std::ofstream("driving_bad4.dnv") << "DNVH 1\n560 50 5 9 -1 5\n"; }
    for (const char* bad : { "driving_bad1.dnv", "driving_bad2.dnv", "driving_bad3.dnv", "driving_bad4.dnv",
             "driving_missing.dnv" }) {
        Driving e;
        error.clear();
        CHECK(!e.loadVehicles(bad, error));
        CHECK(!error.empty());
    }
}

int main()
{
    classes();
    junctions();
    corners();
    plans();
    vehicles();
    if (failures) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("driving: all checks passed\n");
    return 0;
}
