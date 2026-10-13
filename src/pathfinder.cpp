#include "pathfinder.hpp"
#include "navfile.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <map>

#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

namespace dnav {

namespace {

constexpr int MAX_NODES = 8192;         // search nodes per query: a few km of corridor
constexpr int MAX_POLYS = 2048;         // polygons in one corridor
constexpr int MAX_CORNERS = 512;
constexpr size_t MAX_QUEUE = 8192;
constexpr float SAMPLE_STEP = 2.0f;     // a height check every 2 m along each leg
constexpr float KEEP_DZ = 0.15f;        // a sample stays if the ground bends more than this
const float START_REACH[3] = { 1.5f, 2.5f, 1.5f };
const float CAR_REACH[3] = { 4.0f, 3.0f, 4.0f };    // a car's centre can be well inside its mesh's edge
constexpr float CORNER_SLOW = 100.0f;   // OffRoad: a turn this sharp (degrees) brings the car to its slowest
constexpr float CORNER_MIN = 0.35f;     // the slowest, as a share of cruising speed

const float* reachOf(MeshKind k)
{
    return k == MeshKind::Car ? CAR_REACH : START_REACH;
}

MeshKind meshOf(PathKind k)
{
    return k == PathKind::OffRoad ? MeshKind::Car : MeshKind::Ped;
}

float frand()
{
    return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
}

} // namespace

Pathfinder::Pathfinder() = default;

Pathfinder::~Pathfinder()
{
    stop();
    for (Mesh& m : meshes_) {
        if (m.query) dtFreeNavMeshQuery(m.query);
        if (m.mesh) dtFreeNavMesh(m.mesh);
    }
}

int Pathfinder::load(const std::string& folder, std::vector<std::string>& log)
{
    namespace fs = std::filesystem;
    std::error_code ec;
    std::vector<std::string> files;
    for (const auto& e : fs::directory_iterator(folder, ec))
        if (e.is_regular_file() && e.path().extension() == ".nav") files.push_back(e.path().string());
    std::sort(files.begin(), files.end());
    // The road graph, if there is one (tools/dnav.py roads).
    const auto roadsPath = (fs::path(folder) / "roads.dnr").string();
    if (fs::exists(roadsPath, ec)) {
        std::string error;
        if (roads_.load(roadsPath, error))
            log.push_back("loaded roads.dnr: " + std::to_string(roads_.nodeCount()) + " road nodes, "
                + std::to_string(roads_.edgeCount()) + " links");
        else
            log.push_back("ERROR: " + error);
    }
    if (files.empty()) {
        log.push_back("no navmesh: " + folder + "/ has no .nav files");
        return 0;
    }
    return loadFiles(files, log);
}

int Pathfinder::loadFiles(const std::vector<std::string>& files, std::vector<std::string>& log)
{
    namespace fs = std::filesystem;
    std::vector<std::pair<std::string, NavFile>> loaded[MESH_KINDS];
    for (const auto& path : files) {
        NavFile f;
        std::string error;
        if (!readNavFile(path, f, error)) {
            log.push_back("ERROR: " + error);
            continue;
        }
        loaded[static_cast<int>(f.kind)].emplace_back(fs::path(path).filename().string(), std::move(f));
    }
    int total = 0;
    for (int k = 0; k < MESH_KINDS; k++) {
        auto& kind = loaded[k];
        // A world mesh (world.nav, world_car.nav: the whole world baked at once by
        // dnav.py world, custom maps already in it) replaces the per-map files of its
        // kind, which would only overlap it.
        const auto world = std::find_if(kind.begin(), kind.end(),
            [](const auto& f) { return f.first.rfind("world", 0) == 0; });
        if (world != kind.end() && kind.size() > 1) {
            log.push_back(world->first + " found: the " + std::to_string(kind.size() - 1) + " other "
                + kindName(static_cast<MeshKind>(k)) + " navmesh files are skipped");
            auto keep = std::move(*world);
            kind.clear();
            kind.push_back(std::move(keep));
        }
        total += addMesh(static_cast<MeshKind>(k), kind, log);
    }
    return total;
}

int Pathfinder::addMesh(MeshKind k, std::vector<std::pair<std::string, NavFile>>& files, std::vector<std::string>& log)
{
    size_t total = 0;
    for (const auto& f : files) total += f.second.tiles.size();
    if (!total) return 0;
    Mesh& m = meshes_[static_cast<int>(k)];
    m.mesh = dtAllocNavMesh();
    dtNavMeshParams params = worldParams(k, static_cast<int>(total));
    if (dtStatusFailed(m.mesh->init(&params))) {
        log.push_back("ERROR: could not make a " + std::string(kindName(k)) + " navmesh for " + std::to_string(total)
            + " tiles");
        dtFreeNavMesh(m.mesh);
        m.mesh = nullptr;
        return 0;
    }
    std::map<std::pair<int, int>, std::string> owner;
    for (auto& [name, file] : files) {
        for (Tile& t : file.tiles) {
            auto key = std::make_pair(t.x, t.y);
            auto it = owner.find(key);
            if (it != owner.end()) {
                log.push_back("ERROR: " + name + " and " + it->second + " both bake tile " + std::to_string(t.x)
                    + "," + std::to_string(t.y) + "; " + name + "'s is skipped");
                continue;
            }
            // The mesh points into data_ (no DT_TILE_FREE_DATA): we keep it alive.
            data_.push_back(std::move(t.data));
            auto& d = data_.back();
            if (dtStatusFailed(m.mesh->addTile(d.data(), static_cast<int>(d.size()), 0, 0, nullptr))) {
                log.push_back("ERROR: " + name + ": tile " + std::to_string(t.x) + "," + std::to_string(t.y)
                    + " does not load");
                data_.pop_back();
                continue;
            }
            owner[key] = name;
            m.tiles++;
            m.polys += reinterpret_cast<const dtMeshHeader*>(d.data())->polyCount;
        }
        log.push_back("loaded " + name + " (" + kindName(k) + ")");
    }
    m.query = dtAllocNavMeshQuery();
    m.query->init(m.mesh, MAX_NODES);
    return m.tiles;
}

void Pathfinder::start(int threads)
{
    stopping_ = false;
    for (int i = 0; i < threads; i++) threads_.emplace_back(&Pathfinder::work, this);
}

void Pathfinder::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    wake_.notify_all();
    for (auto& t : threads_) t.join();
    threads_.clear();
}

int Pathfinder::request(void* owner, const Point& from, const Point& to, PathKind kind, const RouteOptions& road)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() >= MAX_QUEUE) return 0;
    PathRequest r;
    r.id = nextId_++;
    if (nextId_ <= 0) nextId_ = 1;
    r.owner = owner;
    r.kind = kind;
    r.road = road;
    r.from = from;
    r.to = to;
    queue_.push_back(r);
    inFlight_++;
    // Nothing to search or no threads: answer it straight away, on the next tick.
    const bool have = kind == PathKind::Drive ? roads_.loaded() : meshLoaded(meshOf(kind));
    if (!have || threads_.empty()) {
        queue_.pop_back();
        PathResult res;
        res.id = r.id;
        res.owner = owner;
        res.kind = kind;
        res.status = PathStatus::NotLoaded;
        done_.push_back(std::move(res));
        return r.id;
    }
    wake_.notify_one();
    return r.id;
}

void Pathfinder::takeResults(std::vector<PathResult>& out)
{
    std::lock_guard<std::mutex> lock(mutex_);
    out.swap(done_);
    inFlight_ -= out.size();
}

void Pathfinder::forgetOwner(void* owner)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& r : queue_)
        if (r.owner == owner) r.owner = nullptr;
    for (auto& r : done_)
        if (r.owner == owner) r.owner = nullptr;
}

size_t Pathfinder::pending() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return inFlight_;
}

void Pathfinder::work()
{
    dtNavMeshQuery* q[MESH_KINDS] = {};
    for (int k = 0; k < MESH_KINDS; k++) {
        if (!meshes_[k].mesh) continue;
        q[k] = dtAllocNavMeshQuery();
        q[k]->init(meshes_[k].mesh, MAX_NODES);
    }
    for (;;) {
        PathRequest r;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            wake_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
            if (stopping_) break;
            r = queue_.front();
            queue_.pop_front();
        }
        dtNavMeshQuery* mq = q[static_cast<int>(meshOf(r.kind))];
        PathResult res = r.kind == PathKind::Drive || !mq ? findRoute(r) : find(*mq, r);
        std::lock_guard<std::mutex> lock(mutex_);
        done_.push_back(std::move(res));
    }
    for (auto* qq : q)
        if (qq) dtFreeNavMeshQuery(qq);
}

PathResult Pathfinder::findRoute(const PathRequest& r) const
{
    PathResult res;
    res.id = r.id;
    res.owner = r.owner;
    res.kind = r.kind;
    std::vector<RoutePoint> pts;
    res.status = static_cast<PathStatus>(roads_.route(r.from.x, r.from.y, r.from.z, r.to.x, r.to.y, r.to.z, r.road, pts));
    for (const auto& p : pts) {
        res.points.push_back(Point { p.x, p.y, p.z });
        res.speeds.push_back(p.speed);
    }
    return res;
}

PathResult Pathfinder::find(dtNavMeshQuery& q, const PathRequest& r) const
{
    PathResult res;
    res.id = r.id;
    res.owner = r.owner;
    res.kind = r.kind;
    const float* reach = reachOf(meshOf(r.kind));
    dtQueryFilter filter;
    float s[3], e[3], ns[3], ne[3];
    toRecast(&r.from.x, s);
    toRecast(&r.to.x, e);
    dtPolyRef sref = 0, eref = 0;
    q.findNearestPoly(s, reach, &filter, &sref, ns);
    if (!sref) {
        res.status = PathStatus::NoStart;
        return res;
    }
    q.findNearestPoly(e, reach, &filter, &eref, ne);
    if (!eref) {
        res.status = PathStatus::NoEnd;
        return res;
    }

    dtPolyRef polys[MAX_POLYS];
    int npolys = 0;
    q.findPath(sref, eref, ns, ne, &filter, polys, &npolys, MAX_POLYS);
    if (npolys == 0) {
        res.status = PathStatus::NoPath;
        return res;
    }
    res.status = PathStatus::Ok;
    if (polys[npolys - 1] != eref) {
        // As near as it gets: the end moves to the closest point of the last polygon.
        q.closestPointOnPoly(polys[npolys - 1], e, ne, nullptr);
        res.status = PathStatus::Partial;
    }

    float corners[MAX_CORNERS * 3];
    int ncorners = 0;
    q.findStraightPath(ns, ne, polys, npolys, corners, nullptr, nullptr, &ncorners, MAX_CORNERS);
    if (ncorners == 0) {
        res.status = PathStatus::NoPath;
        return res;
    }

    // An NPC moves in straight lines, its height too, so a leg that crosses a level
    // change (the foot of an escalator) needs points where the ground bends. Walk
    // each leg in steps, find the corridor polygon under each step and read its
    // height there; keep a step only where the ground leaves the straight line.
    std::vector<float> pts;
    std::vector<bool> corner { true };
    pts.insert(pts.end(), corners, corners + 3);
    int at = 0;     // corridor polygon reached so far
    for (int c = 1; c < ncorners; c++) {
        const float* a = &corners[(c - 1) * 3];
        const float* b = &corners[c * 3];
        const float len = std::sqrt((b[0] - a[0]) * (b[0] - a[0]) + (b[2] - a[2]) * (b[2] - a[2]));
        const int steps = static_cast<int>(len / SAMPLE_STEP);
        for (int k = 1; k < steps; k++) {
            float p[3];
            dtVlerp(p, a, b, static_cast<float>(k) / steps);
            for (int i = at; i < npolys; i++) {
                float h;
                if (dtStatusSucceed(q.getPolyHeight(polys[i], p, &h))) {
                    p[1] = h;
                    at = i;
                    break;
                }
            }
            pts.insert(pts.end(), p, p + 3);
            corner.push_back(false);
        }
        pts.insert(pts.end(), b, b + 3);
        corner.push_back(true);
    }

    // Keep every corner, and a sample only where the ground leaves the line from
    // the last kept point to the next one by more than KEEP_DZ.
    const int n = static_cast<int>(pts.size() / 3);
    std::vector<int> keep { 0 };
    for (int i = 1; i < n; i++) {
        if (corner[i] || i == n - 1) {
            keep.push_back(i);
            continue;
        }
        const float* p = &pts[keep.back() * 3];
        const float* m = &pts[i * 3];
        const float* nx = &pts[(i + 1) * 3];
        const float total = std::hypot(nx[0] - p[0], nx[2] - p[2]);
        const float t = total > 1e-4f ? std::hypot(m[0] - p[0], m[2] - p[2]) / total : 0.0f;
        if (std::fabs(m[1] - (p[1] + (nx[1] - p[1]) * t)) > KEEP_DZ) keep.push_back(i);
    }
    for (int i : keep) {
        Point g;
        toGta(&pts[i * 3], &g.x);
        res.points.push_back(g);
    }
    if (r.kind == PathKind::OffRoad) offRoadSpeeds(res);
    return res;
}

// A car slows for each turn by how sharp it is, and for the end.
void Pathfinder::offRoadSpeeds(PathResult& res)
{
    const auto& p = res.points;
    res.speeds.assign(p.size(), 1.0f);
    for (size_t i = 1; i + 1 < p.size(); i++) {
        const float ax = p[i].x - p[i - 1].x, ay = p[i].y - p[i - 1].y;
        const float bx = p[i + 1].x - p[i].x, by = p[i + 1].y - p[i].y;
        const float la = std::hypot(ax, ay), lb = std::hypot(bx, by);
        if (la < 1e-3f || lb < 1e-3f) continue;
        const float c = std::clamp((ax * bx + ay * by) / (la * lb), -1.0f, 1.0f);
        const float turn = std::acos(c) * 57.29578f;
        res.speeds[i] = std::clamp(1.0f - turn / CORNER_SLOW * (1.0f - CORNER_MIN), CORNER_MIN, 1.0f);
    }
    if (!res.speeds.empty()) res.speeds.back() = CORNER_MIN;
}

PathResult Pathfinder::findNow(const Point& from, const Point& to, MeshKind k) const
{
    PathRequest r;
    r.kind = k == MeshKind::Car ? PathKind::OffRoad : PathKind::Walk;
    r.from = from;
    r.to = to;
    if (!mesh(k).query) {
        PathResult res;
        res.status = PathStatus::NotLoaded;
        return res;
    }
    return find(*mesh(k).query, r);
}

bool Pathfinder::nearest(const Point& p, float reach, Point& out, MeshKind k) const
{
    dtNavMeshQuery* q = mesh(k).query;
    if (!q) return false;
    dtQueryFilter filter;
    float rc[3], ext[3] = { reach, reach, reach }, hit[3];
    toRecast(&p.x, rc);
    dtPolyRef ref = 0;
    q->findNearestPoly(rc, ext, &filter, &ref, hit);
    if (!ref) return false;
    toGta(hit, &out.x);
    return true;
}

bool Pathfinder::random(Point& out, MeshKind k) const
{
    dtNavMeshQuery* q = mesh(k).query;
    if (!q) return false;
    dtQueryFilter filter;
    dtPolyRef ref = 0;
    float rc[3];
    if (dtStatusFailed(q->findRandomPoint(&filter, frand, &ref, rc)) || !ref) return false;
    toGta(rc, &out.x);
    return true;
}

bool Pathfinder::groundAt(float x, float y, float z, float reach, float& out, MeshKind k) const
{
    dtNavMeshQuery* q = mesh(k).query;
    if (!q) return false;
    dtQueryFilter filter;
    const Point p { x, y, z };
    float rc[3], ext[3] = { 0.5f, reach, 0.5f }, hit[3];
    toRecast(&p.x, rc);
    dtPolyRef ref = 0;
    q->findNearestPoly(rc, ext, &filter, &ref, hit);
    if (!ref) return false;
    float h = 0.0f;
    if (dtStatusFailed(q->getPolyHeight(ref, rc, &h))) h = hit[1];
    out = h;
    return true;
}

bool Pathfinder::lineClear(const Point& from, const Point& to, MeshKind k) const
{
    dtNavMeshQuery* q = mesh(k).query;
    if (!q) return false;
    dtQueryFilter filter;
    float s[3], e[3], ns[3], ne[3];
    toRecast(&from.x, s);
    toRecast(&to.x, e);
    dtPolyRef sref = 0, eref = 0;
    q->findNearestPoly(s, reachOf(k), &filter, &sref, ns);
    q->findNearestPoly(e, reachOf(k), &filter, &eref, ne);
    if (!sref || !eref) return false;
    // The ray runs along the mesh's surface; t stays FLT_MAX when it reaches the
    // end without leaving the mesh.
    float t = 0.0f, normal[3];
    dtPolyRef path[256];
    int npath = 0;
    if (dtStatusFailed(q->raycast(sref, ns, ne, &filter, &t, normal, path, &npath, 256))) return false;
    if (t < 1.0f || npath == 0) return false;
    // It reached the end's x and z; the end's polygon must be on the same floor.
    return path[npath - 1] == eref;
}

} // namespace dnav
