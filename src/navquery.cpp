// navquery: check a baked folder from the command line, without a server.
//
//   navquery <folder> info
//   navquery <folder> path  x y z  x y z          a walking path on the navmesh
//   navquery <folder> route x y z  x y z [--emergency] [--boats]
//                                                 a driving route on the road graph
//   navquery <folder> near  x y z [reach]         the nearest walkable point
//
// Prints one point per line ("x y z", plus the speed factor for a route) and exits
// 0 when the search found the whole way, 2 for a partial path, 1 otherwise.

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
        "       navquery <folder> path  x y z  x y z\n"
        "       navquery <folder> route x y z  x y z [--emergency] [--boats]\n"
        "       navquery <folder> near  x y z [reach]\n");
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
        Point hit;
        if (!finder.nearest(Point { p[0], p[1], p[2] }, p[3], hit)) {
            std::fprintf(stderr, "nothing walkable within %.1f m\n", p[3]);
            return 1;
        }
        std::printf("%.2f %.2f %.2f\n", hit.x, hit.y, hit.z);
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
            else return usage();
        }
        res = finder.findRoute(r);
    } else {
        res = finder.findNow(r.from, r.to);
    }
    std::fprintf(stderr, "%s: %zu points\n", statusName(res.status), res.points.size());
    for (size_t i = 0; i < res.points.size(); i++) {
        const Point& p = res.points[i];
        if (i < res.speeds.size()) std::printf("%.2f %.2f %.2f %.2f\n", p.x, p.y, p.z, res.speeds[i]);
        else std::printf("%.2f %.2f %.2f\n", p.x, p.y, p.z);
    }
    return res.status == PathStatus::Ok ? 0 : res.status == PathStatus::Partial ? 2 : 1;
}
