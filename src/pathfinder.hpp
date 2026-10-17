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

class Collision;

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
    std::vector<uint8_t> ledges;    // OffRoad: 1 where a drop off a ledge starts (its run-up, then off)
    size_t joined = 0;              // Drive: the points before the first road one, the way onto it across the car mesh
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

    // With the world's collision: one-way drops into the car mesh, off every ledge a
    // car could drive off to more of the mesh below (a storm drain's edge, a bridge's
    // side with no railing). Only searches that ask for them (RouteOptions::ledges)
    // use them. Before start(). Returns how many; `seconds` how long it took.
    int addLedges(const Collision& world, float& seconds);
    int ledgeCount() const { return ledges_; }

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
    PathResult findMesh(dtNavMeshQuery& q, const PathRequest& r) const;
    // With the car mesh's query, a route whose first point the car can't drive straight
    // to (a road below the wall it is up against) starts with the way there across it.
    PathResult findRoute(const PathRequest& r, dtNavMeshQuery* car = nullptr) const;
    // The same on the calling thread (navbake's route check, navquery).
    PathResult findNow(const Point& from, const Point& to, MeshKind k = MeshKind::Ped, bool ledges = false) const;
    // A road route on the calling thread, joined onto across the car mesh as a chase's is.
    PathResult findRouteNow(const PathRequest& r) const { return findRoute(r, mesh(MeshKind::Car).query); }

private:
    struct Mesh {
        dtNavMesh* mesh = nullptr;
        dtNavMeshQuery* query = nullptr;    // the game thread's
        int tiles = 0, polys = 0;
    };
    const Mesh& mesh(MeshKind k) const { return meshes_[static_cast<int>(k)]; }
    int addMesh(MeshKind k, std::vector<std::pair<std::string, NavFile>>& files, std::vector<std::string>& log);
    static void offRoadSpeeds(PathResult& res);
    static bool lineClearOn(const dtNavMeshQuery& q, const float* reach, const Point& from, const Point& to);
    void work();

    bool joinRoute(dtNavMeshQuery& car, const PathRequest& r, PathResult& res) const;
    // The car's way along `pts` (ground points) meets no wall in the world's collision
    // at bumper height, but for the legs starting where `drops` is set (true without
    // the collision).
    bool clearOfWalls(const std::vector<Point>& pts, const std::vector<uint8_t>& drops) const;

    RoadGraph roads_;
    int ledges_ = 0;
    std::atomic<const Collision*> world_ { nullptr };   // addLedges': for checking ways onto roads
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
