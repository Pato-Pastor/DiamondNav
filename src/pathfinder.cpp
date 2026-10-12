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

float frand()
{
    return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
}

} // namespace

Pathfinder::Pathfinder() = default;

Pathfinder::~Pathfinder()
{
    stop();
    if (mainQuery_) dtFreeNavMeshQuery(mainQuery_);
    if (mesh_) dtFreeNavMesh(mesh_);
}

int Pathfinder::load(const std::string& folder, std::vector<std::string>& log)
{
    namespace fs = std::filesystem;
    std::error_code ec;
    std::vector<std::string> files;
    for (const auto& e : fs::directory_iterator(folder, ec))
        if (e.is_regular_file() && e.path().extension() == ".nav") files.push_back(e.path().string());
    std::sort(files.begin(), files.end());
    // world.nav is the whole world baked at once (tools/dnav_world.py), with any
    // custom maps already in it, so per-map files would only overlap it.
    const auto world = std::find_if(files.begin(), files.end(),
        [](const std::string& f) { return fs::path(f).filename() == "world.nav"; });
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
    if (world != files.end() && files.size() > 1) {
        log.push_back("world.nav found: the " + std::to_string(files.size() - 1) + " per-map navmesh files are skipped");
        files = { *world };
    }
    if (files.empty()) {
        log.push_back("no navmesh: " + folder + "/ has no .nav files");
        return 0;
    }
    return loadFiles(files, log);
}

int Pathfinder::loadFiles(const std::vector<std::string>& files, std::vector<std::string>& log)
{

    std::vector<std::pair<std::string, NavFile>> loaded;
    size_t total = 0;
    for (const auto& path : files) {
        NavFile f;
        std::string error;
        if (!readNavFile(path, f, error)) {
            log.push_back("ERROR: " + error);
            continue;
        }
        total += f.tiles.size();
        loaded.emplace_back(std::filesystem::path(path).filename().string(), std::move(f));
    }
    if (!total) return 0;

    mesh_ = dtAllocNavMesh();
    dtNavMeshParams params = worldParams(static_cast<int>(total));
    if (dtStatusFailed(mesh_->init(&params))) {
        log.push_back("ERROR: could not make a navmesh for " + std::to_string(total) + " tiles");
        dtFreeNavMesh(mesh_);
        mesh_ = nullptr;
        return 0;
    }
    std::map<std::pair<int, int>, std::string> owner;
    for (auto& [name, file] : loaded) {
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
            if (dtStatusFailed(mesh_->addTile(d.data(), static_cast<int>(d.size()), 0, 0, nullptr))) {
                log.push_back("ERROR: " + name + ": tile " + std::to_string(t.x) + "," + std::to_string(t.y)
                    + " does not load");
                data_.pop_back();
                continue;
            }
            owner[key] = name;
            tiles_++;
            polys_ += reinterpret_cast<const dtMeshHeader*>(d.data())->polyCount;
        }
        log.push_back("loaded " + name);
    }
    mainQuery_ = dtAllocNavMeshQuery();
    mainQuery_->init(mesh_, MAX_NODES);
    return tiles_;
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
    const bool have = kind == PathKind::Drive ? roads_.loaded() : mesh_ != nullptr;
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
    dtNavMeshQuery* q = nullptr;
    if (mesh_) {
        q = dtAllocNavMeshQuery();
        q->init(mesh_, MAX_NODES);
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
        PathResult res = r.kind == PathKind::Drive || !q ? findRoute(r) : find(*q, r);
        std::lock_guard<std::mutex> lock(mutex_);
        done_.push_back(std::move(res));
    }
    if (q) dtFreeNavMeshQuery(q);
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
    dtQueryFilter filter;
    float s[3], e[3], ns[3], ne[3];
    toRecast(&r.from.x, s);
    toRecast(&r.to.x, e);
    dtPolyRef sref = 0, eref = 0;
    q.findNearestPoly(s, START_REACH, &filter, &sref, ns);
    if (!sref) {
        res.status = PathStatus::NoStart;
        return res;
    }
    q.findNearestPoly(e, START_REACH, &filter, &eref, ne);
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
    return res;
}

PathResult Pathfinder::findNow(const Point& from, const Point& to) const
{
    PathRequest r;
    r.from = from;
    r.to = to;
    if (!mainQuery_) {
        PathResult res;
        res.status = PathStatus::NotLoaded;
        return res;
    }
    return find(*mainQuery_, r);
}

bool Pathfinder::nearest(const Point& p, float reach, Point& out) const
{
    if (!mainQuery_) return false;
    dtQueryFilter filter;
    float rc[3], ext[3] = { reach, reach, reach }, hit[3];
    toRecast(&p.x, rc);
    dtPolyRef ref = 0;
    mainQuery_->findNearestPoly(rc, ext, &filter, &ref, hit);
    if (!ref) return false;
    toGta(hit, &out.x);
    return true;
}

bool Pathfinder::random(Point& out) const
{
    if (!mainQuery_) return false;
    dtQueryFilter filter;
    dtPolyRef ref = 0;
    float rc[3];
    if (dtStatusFailed(mainQuery_->findRandomPoint(&filter, frand, &ref, rc)) || !ref) return false;
    toGta(rc, &out.x);
    return true;
}

} // namespace dnav
