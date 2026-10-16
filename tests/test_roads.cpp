// The road graph on made-up roads: loading, lanes, one-way streets, corners and
// the emergency and boat filters. Run by ctest (or build/dnav_tests directly).

#include "roads.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
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

struct Link {
    uint32_t a, b;
    uint8_t ab, ba;     // lanes a->b and b->a
    float median;
};

// Writes a roads.dnr the way tools/dnav.py roads does: edges grouped by node.
static std::string write(const std::string& name, const std::vector<RoadNode>& in, const std::vector<Link>& links)
{
    std::vector<std::vector<RoadEdge>> per(in.size());
    for (const Link& l : links) {
        per[l.a].push_back(RoadEdge { l.b, l.median, l.ab, l.ba, 0 });
        per[l.b].push_back(RoadEdge { l.a, l.median, l.ba, l.ab, 0 });
    }
    std::vector<RoadNode> nodes = in;
    std::vector<RoadEdge> edges;
    for (size_t i = 0; i < nodes.size(); i++) {
        nodes[i].firstEdge = static_cast<uint32_t>(edges.size());
        nodes[i].edgeCount = static_cast<uint16_t>(per[i].size());
        edges.insert(edges.end(), per[i].begin(), per[i].end());
    }
    const std::string path = name + ".dnr";
    std::ofstream out(path, std::ios::binary);
    const uint32_t head[4] = { ROADS_MAGIC, ROADS_VERSION, static_cast<uint32_t>(nodes.size()),
        static_cast<uint32_t>(edges.size()) };
    out.write(reinterpret_cast<const char*>(head), sizeof head);
    out.write(reinterpret_cast<const char*>(nodes.data()), nodes.size() * sizeof(RoadNode));
    out.write(reinterpret_cast<const char*>(edges.data()), edges.size() * sizeof(RoadEdge));
    return path;
}

static RoadNode node(float x, float y, uint16_t flags = 0)
{
    return RoadNode { x, y, 10.0f, 0, 0, flags };
}

int main()
{
    // An L of two-way road: west to east along y = 0, then north at x = 200.
    //   0 (0,0) - 1 (100,0) - 2 (200,0) - 3 (200,100) - 4 (200,200)
    // plus a one-way street 5 -> 6 parallel to the first leg, and an
    // emergency-only spur 2 - 7 and a water route 8 - 9.
    std::vector<RoadNode> nodes = {
        node(0, 0), node(100, 0), node(200, 0), node(200, 100), node(200, 200),
        node(0, 300), node(200, 300),
        node(300, 0, ROAD_EMERGENCY_ONLY),
        node(0, -500, ROAD_BOATS), node(200, -500, ROAD_BOATS),
    };
    std::vector<Link> links = {
        { 0, 1, 1, 1, 0.0f }, { 1, 2, 1, 1, 0.0f }, { 2, 3, 1, 1, 0.0f }, { 3, 4, 1, 1, 0.0f },
        { 5, 6, 2, 0, 0.0f },
        { 2, 7, 1, 1, 0.0f },
        { 8, 9, 1, 1, 0.0f },
    };
    RoadGraph g;
    std::string error;
    CHECK(g.load(write("roads_test", nodes, links), error));
    CHECK(g.nodeCount() == 10);
    CHECK(g.edgeCount() == 14);

    std::vector<RoutePoint> pts;
    RouteOptions car;

    // Along the L: drives on the right (south side going east: y < 0), rounds the
    // corner, ends near (200,200) on the east side going north (x > 200).
    CHECK(g.route(0, 0, 10, 200, 200, 10, car, pts) == RouteStatus::Ok);
    CHECK(pts.size() > 5);
    if (pts.size() > 5) {
        CHECK(std::fabs(pts.front().y + RoadGraph::LANE_WIDTH * 0.5f) < 0.01f);
        CHECK(pts.back().x > 200.0f);
        CHECK(std::fabs(pts.back().y - 200.0f) < 0.01f);
        CHECK(pts.back().speed < 1.0f);
        // The corner is rounded (a point off both lane lines) and slowed.
        bool rounded = false, slowed = false;
        for (const auto& p : pts) {
            if (p.x > 150 && p.x < 199 && p.y > 1 && p.y < 50) rounded = true;
            if (p.speed < 0.5f && p.speed >= 0.3f && p.y < 50) slowed = true;
        }
        CHECK(rounded);
        CHECK(slowed);
    }

    // The way back drives on the other side.
    CHECK(g.route(200, 200, 10, 0, 0, 10, car, pts) == RouteStatus::Ok);
    if (!pts.empty()) CHECK(pts.front().x < 200.0f);

    // One-way: 5 -> 6 is fine, 6 -> 5 has no road.
    CHECK(g.route(0, 300, 10, 200, 300, 10, car, pts) == RouteStatus::Ok);
    if (!pts.empty()) {
        // Two lanes either side of the centre line: the inner one is just left of it.
        CHECK(std::fabs(pts.front().y - 300.0f - RoadGraph::LANE_WIDTH * 0.5f) < 0.01f);
    }
    CHECK(g.route(200, 300, 10, 0, 300, 10, car, pts) == RouteStatus::NoStart);

    // Not connected to the L.
    CHECK(g.route(0, 0, 10, 200, 300, 10, car, pts) == RouteStatus::NoPath);

    // Emergency-only: ignored by ordinary cars, used by emergency ones.
    CHECK(g.route(0, 0, 10, 255, 0, 10, car, pts) == RouteStatus::Ok);
    if (!pts.empty()) CHECK(pts.back().x <= 200.5f);
    RouteOptions ambulance;
    ambulance.emergency = true;
    CHECK(g.route(0, 0, 10, 255, 0, 10, ambulance, pts) == RouteStatus::Ok);
    if (!pts.empty()) CHECK(std::fabs(pts.back().x - 255.0f) < 0.5f);

    // Starting and ending mid-road: the route starts and ends level with them.
    CHECK(g.route(50, 0, 10, 200, 150, 10, car, pts) == RouteStatus::Ok);
    if (pts.size() > 2) {
        CHECK(std::fabs(pts.front().x - 50.0f) < 0.5f);
        CHECK(std::fabs(pts.back().y - 150.0f) < 0.5f);
    }
    // Both on one stretch.
    CHECK(g.route(20, 0, 10, 80, 0, 10, car, pts) == RouteStatus::Ok);
    if (pts.size() >= 2) {
        CHECK(std::fabs(pts.front().x - 20.0f) < 0.5f);
        CHECK(std::fabs(pts.back().x - 80.0f) < 0.5f);
    }

    // Boats only use water routes, cars never do.
    RouteOptions boat;
    boat.boats = true;
    CHECK(g.route(0, -500, 10, 200, -500, 10, boat, pts) == RouteStatus::Ok);
    CHECK(g.route(0, -500, 10, 200, -500, 10, car, pts) == RouteStatus::NoStart);
    CHECK(g.route(0, 0, 10, 200, 0, 10, boat, pts) == RouteStatus::NoStart);

    // Far from any road.
    CHECK(g.route(1000, 1000, 10, 0, 0, 10, car, pts) == RouteStatus::NoStart);
    CHECK(g.route(0, 0, 10, 1000, 1000, 10, car, pts) == RouteStatus::NoEnd);
    CHECK(g.nearest(5, 5, 10, 20, car) == 0);

    // The road surface's height beside a road, for a car off its route: the nearest
    // stretch within reach, on its own level, its height running along the slope.
    float h = 0.0f;
    CHECK(g.surfaceAt(50, 8, 10.5f, 25, h) && std::fabs(h - 10.0f) < 0.001f);
    CHECK(!g.surfaceAt(50, 60, 10.0f, 25, h));             // too far from any road
    CHECK(!g.surfaceAt(50, 8, 30.0f, 25, h));              // a level up: not this road
    RoadGraph slope;
    std::vector<RoadNode> hill = { node(0, 0), node(100, 0) };
    hill[1].z = 20.0f;
    CHECK(slope.load(write("roads_slope", hill, { { 0, 1, 1, 1, 0.0f } }), error));
    CHECK(slope.surfaceAt(25, 6, 12.0f, 25, h) && std::fabs(h - 12.5f) < 0.001f);

    // The deck: only over the road's paved width (a lane each way: 5 m and a 1.5 m
    // shoulder either side of the centre line), on its own level.
    CHECK(slope.deckAt(25, 6, 12.0f, h) && std::fabs(h - 12.5f) < 0.001f);
    CHECK(slope.deckAt(25, -6, 12.0f, h));
    CHECK(!slope.deckAt(25, 8, 12.0f, h));                // past the side: the drop
    CHECK(!slope.deckAt(25, 0, 30.0f, h));                // a level up

    // A long straight with its nodes 300 m apart: a route still starts and ends along
    // it, though neither end is within reach of a node.
    RoadGraph straight;
    CHECK(straight.load(write("roads_long", { node(0, 0), node(300, 0) }, { { 0, 1, 1, 1, 0.0f } }), error));
    CHECK(straight.route(100, 0, 10, 220, 0, 10, car, pts) == RouteStatus::Ok);
    CHECK(pts.size() >= 2 && std::fabs(pts.front().x - 100.0f) < 1.0f && std::fabs(pts.back().x - 220.0f) < 1.0f);
    CHECK(straight.route(100, 200, 10, 220, 0, 10, car, pts) == RouteStatus::NoStart);   // too far off it

    // Bad files.
    RoadGraph bad;
    { std::ofstream("roads_junk.dnr") << "not a road graph"; }
    CHECK(!bad.load("roads_junk.dnr", error));
    CHECK(!bad.load("roads_missing.dnr", error));
    CHECK(!bad.loaded());

    if (failures) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("roads: all checks passed\n");
    return 0;
}
