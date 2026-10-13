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

#include "navfile.hpp"
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
    Walk = 0,           // the walking navmesh
    Drive = 1,          // the road graph
    OffRoad = 2,        // the car navmesh: wherever a car fits
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
    std::vector<float> speeds;      // Drive, OffRoad: 0..1 of cruising speed at each point
    std::vector<RoadClass> classes; // Drive, OffRoad: the road each point is on
    std::vector<uint8_t> junctions; // Drive: 1 where the point is in a junction
};

class Pathfinder {
public:
    Pathfinder();
    ~Pathfinder();

    // Loads every *.nav in a folder, each into the mesh of its kind (walking or
    // car), and roads.dnr. Returns the tile count; errors go to `log` lines.
    int load(const std::string& folder, std::vector<std::string>& log);
    int loadFiles(const std::vector<std::string>& files, std::vector<std::string>& log);
    bool loaded() const { return meshLoaded(MeshKind::Ped) || meshLoaded(MeshKind::Car) || roads_.loaded(); }
    bool meshLoaded(MeshKind k = MeshKind::Ped) const { return mesh(k).mesh != nullptr; }
    const RoadGraph& roads() const { return roads_; }
    int tileCount(MeshKind k = MeshKind::Ped) const { return mesh(k).tiles; }
    int polyCount(MeshKind k = MeshKind::Ped) const { return mesh(k).polys; }

    void start(int threads);
    void stop();

    // 0 when the queue is full.
    int request(void* owner, const Point& from, const Point& to, PathKind kind = PathKind::Walk,
        const RouteOptions& road = {});
    void takeResults(std::vector<PathResult>& out);
    void forgetOwner(void* owner);
    size_t pending() const;

    // Game-thread queries (their own dtNavMeshQuery).
    bool nearest(const Point& p, float reach, Point& out, MeshKind k = MeshKind::Ped) const;
    bool random(Point& out, MeshKind k = MeshKind::Ped) const;
    // Whether the mesh goes straight from `from` to `to` with no wall or drop in
    // between (both on the ground). False when either end is off the mesh.
    bool lineClear(const Point& from, const Point& to, MeshKind k = MeshKind::Ped) const;
    // The mesh's ground height under (x, y), nearest to z within `reach` metres up or
    // down. False off the mesh.
    bool groundAt(float x, float y, float z, float reach, float& out, MeshKind k = MeshKind::Ped) const;

    // The search itself; any thread with its own query on the right mesh.
    PathResult find(dtNavMeshQuery& q, const PathRequest& r) const;
    PathResult findRoute(const PathRequest& r) const;
    // The same on the calling thread (navbake's route check).
    PathResult findNow(const Point& from, const Point& to, MeshKind k = MeshKind::Ped) const;

private:
    struct Mesh {
        dtNavMesh* mesh = nullptr;
        dtNavMeshQuery* query = nullptr;    // the game thread's
        int tiles = 0, polys = 0;
    };
    const Mesh& mesh(MeshKind k) const { return meshes_[static_cast<int>(k)]; }
    int addMesh(MeshKind k, std::vector<std::pair<std::string, NavFile>>& files, std::vector<std::string>& log);
    static void offRoadSpeeds(PathResult& res);
    void work();

    RoadGraph roads_;
    Mesh meshes_[MESH_KINDS];
    std::deque<std::vector<unsigned char>> data_;   // tile data the meshes point into

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
