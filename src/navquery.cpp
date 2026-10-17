// navquery: check a baked folder from the command line, without a server.
//
//   navquery <folder> info
//   navquery <folder> path  x y z  x y z [--car] [--ledges]
//                                                 a walking (or driving) path on the navmesh; --ledges
//                                                 lets a car drive off ledges (needs world.col)
//   navquery <folder> route x y z  x y z [--emergency] [--boats] [--heading h --speed m/s]
//                                                 a driving route on the road graph (as a chase plans
//                                                 it with a heading: any way, from the road ahead)
//   navquery <folder> near  x y z [reach] [--car] the nearest walkable (or drivable) point
//   navquery <folder> road  x y                   the nearest road node in the plan, at any height, and its class
//
// Prints one point per line ("x y z", plus the speed factor for a route) and exits
// 0 when the search found the whole way, 2 for a partial path, 1 otherwise.

#include "collision.hpp"
#include "pathfinder.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace dnav;

namespace {

const char* statusName(PathStatus s)
{
    switch (s) {
    case PathStatus::Ok: return "ok";
    case PathStatus::Partial: return "partial";
    case PathStatus::NoStart: return "nothing near the start";
    case PathStatus::NoEnd: return "nothing near the end";
    case PathStatus::NoPath: return "not connected";
    case PathStatus::NotLoaded: return "not loaded";
    }
    return "?";
}

int usage()
{
    std::fprintf(stderr,
        "usage: navquery <folder> info\n"
        "       navquery <folder> path  x y z  x y z [--car] [--ledges]\n"
        "       navquery <folder> route x y z  x y z [--emergency] [--boats] [--heading h --speed m/s]\n"
        "       navquery <folder> near  x y z [reach] [--car]\n"
        "       navquery <folder> road  x y\n");
    return 1;
}

bool points(int argc, char** argv, int at, int count, float* out)
{
    if (argc < at + count) return false;
    for (int i = 0; i < count; i++) {
        char* end = nullptr;
        out[i] = std::strtof(argv[at + i], &end);
        if (!end || *end) return false;
    }
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 3) return usage();
    const std::string cmd = argv[2];
    Pathfinder finder;
    std::vector<std::string> log;
    finder.load(argv[1], log);
    for (const auto& line : log) std::fprintf(stderr, "%s\n", line.c_str());

    if (cmd == "info") {
        std::printf("tiles %d\npolygons %d\nroad nodes %zu\nroad links %zu\n", finder.tileCount(), finder.polyCount(),
            finder.roads().nodeCount(), finder.roads().edgeCount());
        return finder.loaded() ? 0 : 1;
    }
    if (cmd == "near") {
        float p[4] = { 0, 0, 0, 5.0f };
        if (!points(argc, argv, 3, 3, p)) return usage();
        if (argc > 6) points(argc, argv, 6, 1, p + 3);
        const bool car = argc > 7 && !std::strcmp(argv[7], "--car");
        Point hit;
        if (!finder.nearest(Point { p[0], p[1], p[2] }, p[3], hit, car ? MeshKind::Car : MeshKind::Ped)) {
            std::fprintf(stderr, "nothing walkable within %.1f m\n", p[3]);
            return 1;
        }
        std::printf("%.2f %.2f %.2f\n", hit.x, hit.y, hit.z);
        return 0;
    }
    if (cmd == "road") {
        float p[2];
        if (!points(argc, argv, 3, 2, p)) return usage();
        const RoadNode* best = nullptr;
        float bestD = 0.0f;
        for (const auto& n : finder.roads().nodes()) {
            const float d = (n.x - p[0]) * (n.x - p[0]) + (n.y - p[1]) * (n.y - p[1]);
            if (!best || d < bestD) { best = &n; bestD = d; }
        }
        if (!best) return 1;
        const auto& roads = finder.roads();
        const uint32_t id = (uint32_t)(best - roads.nodes().data());
        static const char* names[] = { "highway", "main", "street", "minor", "service", "offroad" };
        std::printf("%.2f %.2f %.2f %s\n", best->x, best->y, best->z,
            best->edgeCount ? names[(int)roads.classOf(id, roads.edges()[best->firstEdge])] : "dead end");
        return 0;
    }
    if (cmd != "path" && cmd != "route") return usage();
    float v[6];
    if (!points(argc, argv, 3, 6, v)) return usage();
    PathRequest r;
    r.from = Point { v[0], v[1], v[2] };
    r.to = Point { v[3], v[4], v[5] };
    PathResult res;
    if (cmd == "route") {
        r.kind = PathKind::Drive;
        for (int i = 9; i < argc; i++) {
            if (!std::strcmp(argv[i], "--emergency")) r.road.emergency = true;
            else if (!std::strcmp(argv[i], "--boats")) r.road.boats = true;
            else if (!std::strcmp(argv[i], "--heading") && i + 1 < argc) {
                r.road.hasHeading = true;
                r.road.anyWay = true;
                r.road.heading = std::strtof(argv[++i], nullptr);
            } else if (!std::strcmp(argv[i], "--speed") && i + 1 < argc) r.road.speed = std::strtof(argv[++i], nullptr);
            else return usage();
        }
        // With a heading, as a chase plans: the way onto the first road checked against
        // the world's collision when the folder has it.
        Collision world;
        std::string error;
        if (r.road.hasHeading && world.load(std::string(argv[1]) + "/world.col", error)) {
            float secs = 0.0f;
            finder.addLedges(world, secs);
        }
        res = finder.findRouteNow(r);
    } else {
        bool car = false, ledges = false;
        for (int i = 9; i < argc; i++) {
            if (!std::strcmp(argv[i], "--car")) car = true;
            else if (!std::strcmp(argv[i], "--ledges")) car = ledges = true;
            else return usage();
        }
        if (ledges) {
            // The drops off ledges need the world's collision: the folder's world.col.
            Collision world;
            std::string error;
            if (!world.load(std::string(argv[1]) + "/world.col", error)) {
                std::fprintf(stderr, "%s\n", error.c_str());
                return 1;
            }
            float secs = 0.0f;
            const int n = finder.addLedges(world, secs);
            std::fprintf(stderr, "%d ledges (%.1f s)\n", n, secs);
        }
        res = finder.findNow(r.from, r.to, car ? MeshKind::Car : MeshKind::Ped, ledges);
    }
    std::fprintf(stderr, "%s: %zu points\n", statusName(res.status), res.points.size());
    for (size_t i = 0; i < res.points.size(); i++) {
        const Point& p = res.points[i];
        if (i < res.speeds.size()) std::printf("%.2f %.2f %.2f %.2f\n", p.x, p.y, p.z, res.speeds[i]);
        else std::printf("%.2f %.2f %.2f\n", p.x, p.y, p.z);
    }
    return res.status == PathStatus::Ok ? 0 : res.status == PathStatus::Partial ? 2 : 1;
}
