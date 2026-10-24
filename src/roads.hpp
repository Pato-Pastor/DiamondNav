// The road graph: where NPC cars can drive (navmesh/roads.dnr, tools/dnav.py roads).
//
// Nodes are points on the road's centre line; each directed edge says how many
// lanes run that way. A route is an A* search over the edges that have lanes in
// the direction of travel, turned into points a car can follow: in its lane (the
// right-hand side), with corners rounded off and a speed factor for each point.
//
// The graph is read-only once loaded, so routes can be searched from many threads.

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace dnav {

constexpr uint32_t ROADS_MAGIC = 0x44524E44;    // "DNRD"
constexpr uint32_t ROADS_VERSION = 1;

enum RoadNodeFlags : uint16_t {
    ROAD_HIGHWAY = 1,
    ROAD_EMERGENCY_ONLY = 2,    // GTA: only emergency vehicles drive here
    ROAD_BOATS = 4,             // a water route
    ROAD_DETAIL = 8,            // the traffic level below is GTA's (older files leave it 0)
    ROAD_TRAFFIC = 0x30,        // GTA's traffic level, 0 (full) .. 3 (low), in bits 4-5
    ROAD_MINOR = 0x40,          // a custom road marked minor
    ROAD_MAIN = 0x80,           // a custom road marked main
};

// What sort of road a stretch is, which sets its speed limit (Nav_SetRoadSpeed).
enum class RoadClass : uint8_t {
    Highway = 0,                // GTA's highway flag: freeways
    Main = 1,                   // two or more lanes each way (or one-way)
    Street = 2,                 // one lane each way
    Minor = 3,                  // GTA's low-traffic roads: back streets, country lanes
    Service = 4,                // emergency-only: airport aprons, yards, car parks
    OffRoad = 5,                // the car navmesh, off the roads
    Count = 6,
};

struct RoadNode {
    float x, y, z;              // GTA axes, the road surface
    uint32_t firstEdge;
    uint16_t edgeCount;
    uint16_t flags;
};

struct RoadEdge {
    uint32_t to;
    float median;               // metres between the two directions' inner lanes
    uint8_t lanes;              // lanes from this node towards `to`
    uint8_t lanesBack;          // lanes from `to` towards this node
    uint16_t flags;
};

struct RoutePoint {
    float x, y, z;              // the road surface in the car's lane
    float speed;                // 0..1 of cruising speed (corners, the last stretch)
    RoadClass cls = RoadClass::Street;
    bool junction = false;      // where three or more roads meet
};

struct LanePoint {
    float x, y, z;
    float heading;              // GTA's, degrees
};

struct RouteOptions {
    bool emergency = false;     // may use emergency-only roads
    bool boats = false;         // a boat: water routes only
    // A pursuit: any road either way (against a one-way street's traffic costs a
    // little more), and, with a heading, a start ahead of the car before one behind
    // it (turning round costs about a U-turn's distance).
    bool anyWay = false;
    bool hasHeading = false;
    float heading = 0.0f;       // GTA's, degrees (0 north, turning left)
    float speed = 0.0f;         // m/s with the heading: what turning round costs (turnRoundCost)
    // The car navmesh (OffRoad): may drive off a ledge where Pathfinder::addLedges
    // found a drop to more of the mesh below (a chase).
    bool ledges = false;
    // Nodes not to start from (a road the car has no way onto from where it is).
    const std::vector<uint32_t>* notFrom = nullptr;
};

// What turning round costs a car going at `speed` (m/s), in metres of driving: braking
// to a stop, turning, and getting back up to speed, against carrying on at that speed.
// At a crawl, about a U-turn's length; at 30 m/s, about 300 m.
inline float turnRoundCost(float speed)
{
    constexpr float TURN_TIME = 3.5f, BRAKE = 7.0f, ACCEL = 3.5f, LEAST = 35.0f;
    const float c = TURN_TIME * speed + speed * speed * (0.5f / BRAKE + 0.5f / ACCEL);
    return c > LEAST ? c : LEAST;
}

enum class RouteStatus : int {
    Ok = 0,
    NoStart = -1,               // no road near the start
    NoEnd = -2,                 // no road near the end
    NoPath = -3,                // the roads do not connect
    NotLoaded = -4,             // no road graph
};

class RoadGraph {
public:
    // Lane width and the corner rounding, in metres.
    static constexpr float LANE_WIDTH = 5.0f;
    static constexpr float MAX_CORNER = 12.0f;
    static constexpr float SEARCH_REACH = 60.0f;
    static constexpr float EDGE_REACH = 400.0f;    // ...or a stretch of road starting this near
    static constexpr float AGAINST_COST = 1.3f;    // anyWay: a stretch against the traffic costs this much more
    static constexpr float START_BACK = -0.17f;    // a first stretch heading back more than 100 degrees turns the car round

    bool load(const std::string& path, std::string& error);
    bool loaded() const { return !nodes_.empty(); }
    size_t nodeCount() const { return nodes_.size(); }
    size_t edgeCount() const { return edges_.size(); }

    // The nearest node within reach that a car may leave from (or -1).
    int nearest(float x, float y, float z, float reach, const RouteOptions& opt) const;

    // The road surface's height at (x, y): from the nearest stretch of road within
    // reach (in the plan) that runs within a level of z. False with none.
    bool surfaceAt(float x, float y, float z, float reach, float& out) const;

    // The road's own deck under (x, y): a stretch of road within a level of z whose
    // paved width (its lanes either side of the median, and a shoulder) covers the
    // point. False off the edge of every road, as past a bridge's side.
    // `wider` widens every road by that much either side.
    bool deckAt(float x, float y, float z, float& out, float wider = 0.0f) const;

    // The nearest lane of a road stretch within reach (on z's level) and the heading it
    // runs; of two about as near, the one the given heading follows.
    bool laneAt(float x, float y, float z, float heading, float reach, const RouteOptions& opt, LanePoint& out) const;

    // `startNode`, when given, gets the node the route starts from.
    RouteStatus route(float fx, float fy, float fz, float tx, float ty, float tz, const RouteOptions& opt,
        std::vector<RoutePoint>& out, uint32_t* startNode = nullptr) const;

    const std::vector<RoadNode>& nodes() const { return nodes_; }
    const std::vector<RoadEdge>& edges() const { return edges_; }
    // The class of the road from node `from` along `e`.
    RoadClass classOf(uint32_t from, const RoadEdge& e) const;

private:
    bool usable(const RoadNode& n, const RouteOptions& opt) const;
    bool hasExit(uint32_t node, const RouteOptions& opt) const;
    static bool drivable(const RoadEdge& e, const RouteOptions& opt) { return e.lanes || (opt.anyWay && e.lanesBack); }
    bool hasEntry(uint32_t node, const RouteOptions& opt) const;
    const RoadEdge* edge(uint32_t from, uint32_t to) const;
    float laneOffset(const RoadEdge& e) const;

    std::vector<RoadNode> nodes_;
    std::vector<RoadEdge> edges_;
    std::unordered_map<int64_t, std::vector<uint32_t>> cells_;
};

} // namespace dnav