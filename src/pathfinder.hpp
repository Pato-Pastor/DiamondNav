// The navmesh and the threads that search it.
//
// One dtNavMesh holds every loaded tile; it is only read once loaded, which
// Detour allows from many threads. Each worker owns its own dtNavMeshQuery (a
// query is not thread-safe). Requests go in on the game thread, results come back
// on it through takeResults(): no Pawn code ever runs off the game thread.

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "roads.hpp"

class dtNavMesh;
class dtNavMeshQuery;

namespace dnav {

enum class PathStatus : int {
    Ok = 0,             // the whole way
    Partial = 1,        // as near as the mesh gets to the end
    NoStart = -1,       // nothing walkable near the start
    NoEnd = -2,         // nothing walkable near the end
    NoPath = -3,        // the two are not connected
    NotLoaded = -4,     // no navmesh
};

struct Point {
    float x, y, z;      // GTA axes, the ground under the ped's feet
};

enum class PathKind : int {
    Walk = 0,           // the navmesh
    Drive = 1,          // the road graph
};

struct PathRequest {
    int id = 0;
    void* owner = nullptr;          // the AMX that asked
    PathKind kind = PathKind::Walk;
    RouteOptions road;
    Point from {}, to {};
};

struct PathResult {
    int id = 0;
    void* owner = nullptr;
    PathKind kind = PathKind::Walk;
    PathStatus status = PathStatus::NoPath;
    std::vector<Point> points;
    std::vector<float> speeds;      // Drive: 0..1 of cruising speed at each point
};

class Pathfinder {
public:
    Pathfinder();
    ~Pathfinder();

    // Loads every *.nav in a folder into one mesh. Returns the tile count; errors
    // go to `log` lines.
    int load(const std::string& folder, std::vector<std::string>& log);
    int loadFiles(const std::vector<std::string>& files, std::vector<std::string>& log);
    bool loaded() const { return mesh_ != nullptr || roads_.loaded(); }
    bool meshLoaded() const { return mesh_ != nullptr; }
    const RoadGraph& roads() const { return roads_; }
    int tileCount() const { return tiles_; }
    int polyCount() const { return polys_; }

    void start(int threads);
    void stop();

    // 0 when the queue is full.
    int request(void* owner, const Point& from, const Point& to, PathKind kind = PathKind::Walk,
        const RouteOptions& road = {});
    void takeResults(std::vector<PathResult>& out);
    void forgetOwner(void* owner);
    size_t pending() const;

    // Game-thread queries (their own dtNavMeshQuery).
    bool nearest(const Point& p, float reach, Point& out) const;
    bool random(Point& out) const;

    // The search itself; any thread with its own query.
    PathResult find(dtNavMeshQuery& q, const PathRequest& r) const;
    PathResult findRoute(const PathRequest& r) const;
    // The same on the calling thread (navbake's route check).
    PathResult findNow(const Point& from, const Point& to) const;

private:
    void work();

    RoadGraph roads_;
    dtNavMesh* mesh_ = nullptr;
    dtNavMeshQuery* mainQuery_ = nullptr;
    std::vector<std::vector<unsigned char>> data_;
    int tiles_ = 0, polys_ = 0;

    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::deque<PathRequest> queue_;
    std::vector<PathResult> done_;
    std::vector<std::thread> threads_;
    bool stopping_ = false;
    int nextId_ = 1;
    size_t inFlight_ = 0;
};

} // namespace dnav
