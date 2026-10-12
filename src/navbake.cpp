// navbake: collision -> navmesh tiles (navmesh/<map>.nav, navmesh/world.nav).
//
//   navbake <geometry> <out.nav> [--routes routes.txt [--paths paths.txt]]
//           [--seeds seeds.txt] [--water water.txt] [--dump polys.obj] [--threads N]
//
// geometry is world triangles in GTA axes: an OBJ, or a .tris file
// (tools/dnav_world.py: "DNTR", a count, then float32 x y z for each corner).
// Every face is walkable from either side, as the game's collision is in practice.
//
// --routes checks the result: each line is "name<TAB>x y z<TAB>x y z", and every
// route must find a complete path, or navbake exits 1.
// --seeds is points known to be walked or driven ("x y z" lines: the game's path
// nodes). With routes or seeds, only the polygons connected to one are kept.
// --water is "x0 y0 x1 y1 level" lines: nothing deeper than a ped's knees under
// them is walkable.
// --paths writes the points of each route's path, --dump the baked polygons (GTA
// axes), for previews (tools/dnav_preview.py).
//
// The bake is Recast's tiled pipeline (Sample_TileMesh in RecastDemo, cut down),
// tiles in parallel, each seeing only the triangles that touch it.

#include "navfile.hpp"
#include "pathfinder.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <DetourNavMeshQuery.h>
#include <Recast.h>

using namespace dnav;

namespace {

struct Geometry {
    std::vector<float> verts;   // Recast axes
    std::vector<int> tris;      // each face twice, once each way
    float bmin[3], bmax[3];
    // per tile (key tileKey), the faces (indices into tris / 3) that touch it
    std::vector<std::vector<int>> buckets;
    int tx0 = 0, ty0 = 0, tx1 = -1, ty1 = -1;
};

struct Water {
    float x0, y0, x1, y1, level;    // GTA axes
};

bool loadObj(const std::string& path, Geometry& g)
{
    std::ifstream in(path);
    if (!in) return false;
    std::string line;
    while (std::getline(in, line)) {
        if (line.size() < 2) continue;
        std::istringstream s(line.substr(2));
        if (line[0] == 'v' && line[1] == ' ') {
            float gta[3], rc[3];
            s >> gta[0] >> gta[1] >> gta[2];
            toRecast(gta, rc);
            g.verts.insert(g.verts.end(), rc, rc + 3);
        } else if (line[0] == 'f' && line[1] == ' ') {
            int a, b, c;
            s >> a >> b >> c;
            // both ways: the face holds whichever side it is walked on
            g.tris.insert(g.tris.end(), { a - 1, b - 1, c - 1, a - 1, c - 1, b - 1 });
        }
    }
    if (g.verts.empty() || g.tris.empty()) return false;
    rcCalcBounds(g.verts.data(), static_cast<int>(g.verts.size() / 3), g.bmin, g.bmax);
    return true;
}

bool loadTris(const std::string& path, Geometry& g)
{
    std::ifstream in(path, std::ios::binary);
    char magic[4];
    uint32_t count = 0;
    if (!in.read(magic, 4) || std::memcmp(magic, "DNTR", 4) || !in.read(reinterpret_cast<char*>(&count), 4))
        return false;
    std::vector<float> raw(static_cast<size_t>(count) * 9);
    if (!in.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(raw.size() * sizeof(float))))
        return false;
    g.verts.resize(raw.size());
    for (size_t i = 0; i < raw.size(); i += 3) toRecast(&raw[i], &g.verts[i]);
    g.tris.reserve(static_cast<size_t>(count) * 6);
    for (int f = 0; f < static_cast<int>(count); f++) {
        const int a = f * 3, b = a + 1, c = a + 2;
        g.tris.insert(g.tris.end(), { a, b, c, a, c, b });
    }
    if (g.tris.empty()) return false;
    rcCalcBounds(g.verts.data(), static_cast<int>(g.verts.size() / 3), g.bmin, g.bmax);
    return true;
}

// Which tiles each face touches (its bounds grown by the tile border), so a tile's
// bake only rasterises its own faces: RecastDemo's chunky mesh, as a grid.
void bucketFaces(Geometry& g, float border)
{
    const float ts = Grid::TILE_SIZE;
    auto tileOf = [&](float v, float origin) { return static_cast<int>(std::floor((v - origin) / ts)); };
    g.tx0 = tileOf(g.bmin[0], Grid::ORIGIN_X);
    g.tx1 = tileOf(g.bmax[0], Grid::ORIGIN_X);
    g.ty0 = tileOf(g.bmin[2], Grid::ORIGIN_Z);
    g.ty1 = tileOf(g.bmax[2], Grid::ORIGIN_Z);
    const int w = g.tx1 - g.tx0 + 1, h = g.ty1 - g.ty0 + 1;
    g.buckets.assign(static_cast<size_t>(w) * h, {});
    const int nfaces = static_cast<int>(g.tris.size() / 3);
    for (int f = 0; f < nfaces; f++) {
        float lo[2] = { 1e30f, 1e30f }, hi[2] = { -1e30f, -1e30f };
        for (int k = 0; k < 3; k++) {
            const float* v = &g.verts[g.tris[f * 3 + k] * 3];
            lo[0] = std::min(lo[0], v[0]);
            hi[0] = std::max(hi[0], v[0]);
            lo[1] = std::min(lo[1], v[2]);
            hi[1] = std::max(hi[1], v[2]);
        }
        const int x0 = std::max(g.tx0, tileOf(lo[0] - border, Grid::ORIGIN_X));
        const int x1 = std::min(g.tx1, tileOf(hi[0] + border, Grid::ORIGIN_X));
        const int y0 = std::max(g.ty0, tileOf(lo[1] - border, Grid::ORIGIN_Z));
        const int y1 = std::min(g.ty1, tileOf(hi[1] + border, Grid::ORIGIN_Z));
        for (int ty = y0; ty <= y1; ty++)
            for (int tx = x0; tx <= x1; tx++)
                g.buckets[static_cast<size_t>(ty - g.ty0) * w + (tx - g.tx0)].push_back(f);
    }
}

std::vector<Water> loadWater(const std::string& path)
{
    std::vector<Water> out;
    std::ifstream in(path);
    Water w;
    while (in >> w.x0 >> w.y0 >> w.x1 >> w.y1 >> w.level) out.push_back(w);
    return out;
}

std::vector<std::array<float, 3>> loadPoints(const std::string& path)
{
    std::vector<std::array<float, 3>> out;
    std::ifstream in(path);
    std::array<float, 3> p;
    while (in >> p[0] >> p[1] >> p[2]) out.push_back(p);
    return out;
}

struct Context : rcContext {
    Context() : rcContext(false) {}
    void doLog(const rcLogCategory cat, const char* msg, const int) override
    {
        if (cat != RC_LOG_PROGRESS) std::fprintf(stderr, "[navbake] recast: %s\n", msg);
    }
};

rcConfig baseConfig()
{
    rcConfig c {};
    c.cs = Grid::CELL;
    c.ch = Grid::CELL_HEIGHT;
    c.walkableSlopeAngle = Agent::MAX_SLOPE;
    c.walkableHeight = static_cast<int>(std::ceil(Agent::HEIGHT / c.ch));
    c.walkableClimb = static_cast<int>(std::floor(Agent::CLIMB / c.ch));
    c.walkableRadius = static_cast<int>(std::ceil(Agent::RADIUS / c.cs));
    c.maxEdgeLen = static_cast<int>(12.0f / c.cs);
    c.maxSimplificationError = 1.3f;
    c.minRegionArea = 8 * 8;
    c.mergeRegionArea = 20 * 20;
    c.maxVertsPerPoly = 6;
    c.tileSize = Grid::TILE_CELLS;
    c.borderSize = c.walkableRadius + 3;
    c.width = c.tileSize + c.borderSize * 2;
    c.height = c.tileSize + c.borderSize * 2;
    c.detailSampleDist = c.cs * 6.0f;
    c.detailSampleMaxError = c.ch * 1.0f;
    return c;
}

// One tile's Detour data, or empty when nothing walkable is in it.
std::vector<unsigned char> bakeTile(Context& ctx, const Geometry& g, const std::vector<Water>& water, int tx, int ty)
{
    rcConfig cfg = baseConfig();
    const float ts = Grid::TILE_SIZE;
    cfg.bmin[0] = Grid::ORIGIN_X + tx * ts;
    cfg.bmin[1] = g.bmin[1];
    cfg.bmin[2] = Grid::ORIGIN_Z + ty * ts;
    cfg.bmax[0] = cfg.bmin[0] + ts;
    cfg.bmax[1] = g.bmax[1];
    cfg.bmax[2] = cfg.bmin[2] + ts;
    cfg.bmin[0] -= cfg.borderSize * cfg.cs;
    cfg.bmin[2] -= cfg.borderSize * cfg.cs;
    cfg.bmax[0] += cfg.borderSize * cfg.cs;
    cfg.bmax[2] += cfg.borderSize * cfg.cs;

    std::vector<unsigned char> out;
    const std::vector<int>& faces = g.buckets[static_cast<size_t>(ty - g.ty0) * (g.tx1 - g.tx0 + 1) + (tx - g.tx0)];
    if (faces.empty()) return out;
    std::vector<int> tris;
    tris.reserve(faces.size() * 3);
    for (int f : faces) tris.insert(tris.end(), &g.tris[f * 3], &g.tris[f * 3] + 3);
    const int nverts = static_cast<int>(g.verts.size() / 3);
    const int ntris = static_cast<int>(faces.size());

    rcHeightfield* solid = rcAllocHeightfield();
    if (!rcCreateHeightfield(&ctx, *solid, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch)) {
        rcFreeHeightField(solid);
        return out;
    }
    std::vector<unsigned char> areas(ntris, 0);
    rcMarkWalkableTriangles(&ctx, cfg.walkableSlopeAngle, g.verts.data(), nverts, tris.data(), ntris, areas.data());
    rcRasterizeTriangles(&ctx, g.verts.data(), nverts, tris.data(), areas.data(), ntris, *solid, cfg.walkableClimb);
    rcFilterLowHangingWalkableObstacles(&ctx, cfg.walkableClimb, *solid);
    rcFilterLedgeSpans(&ctx, cfg.walkableHeight, cfg.walkableClimb, *solid);
    rcFilterWalkableLowHeightSpans(&ctx, cfg.walkableHeight, *solid);

    rcCompactHeightfield* chf = rcAllocCompactHeightfield();
    bool ok = rcBuildCompactHeightfield(&ctx, cfg.walkableHeight, cfg.walkableClimb, *solid, *chf);
    rcFreeHeightField(solid);
    ok = ok && chf->spanCount > 0;
    ok = ok && rcErodeWalkableArea(&ctx, cfg.walkableRadius, *chf);
    // Under water deeper than a ped's knees is not walkable (the sea bed, rivers).
    for (const Water& w : water) {
        const float lo[3] = { w.x0, -1000.0f, -w.y1 }, hi[3] = { w.x1, w.level - 0.6f, -w.y0 };
        if (hi[0] < cfg.bmin[0] || lo[0] > cfg.bmax[0] || hi[2] < cfg.bmin[2] || lo[2] > cfg.bmax[2]) continue;
        if (ok) rcMarkBoxArea(&ctx, lo, hi, RC_NULL_AREA, *chf);
    }
    ok = ok && rcBuildDistanceField(&ctx, *chf);
    ok = ok && rcBuildRegions(&ctx, *chf, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea);
    if (!ok) {
        rcFreeCompactHeightfield(chf);
        return out;
    }

    rcContourSet* cset = rcAllocContourSet();
    rcPolyMesh* pmesh = rcAllocPolyMesh();
    rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
    ok = rcBuildContours(&ctx, *chf, cfg.maxSimplificationError, cfg.maxEdgeLen, *cset) && cset->nconts > 0;
    ok = ok && rcBuildPolyMesh(&ctx, *cset, cfg.maxVertsPerPoly, *pmesh);
    ok = ok && rcBuildPolyMeshDetail(&ctx, *pmesh, *chf, cfg.detailSampleDist, cfg.detailSampleMaxError, *dmesh);
    rcFreeCompactHeightfield(chf);
    rcFreeContourSet(cset);

    if (ok && pmesh->npolys > 0) {
        for (int i = 0; i < pmesh->npolys; i++) {
            if (pmesh->areas[i] == RC_WALKABLE_AREA) pmesh->areas[i] = 0;
            pmesh->flags[i] = 1;                // walk
        }
        dtNavMeshCreateParams p {};
        p.verts = pmesh->verts;
        p.vertCount = pmesh->nverts;
        p.polys = pmesh->polys;
        p.polyAreas = pmesh->areas;
        p.polyFlags = pmesh->flags;
        p.polyCount = pmesh->npolys;
        p.nvp = pmesh->nvp;
        p.detailMeshes = dmesh->meshes;
        p.detailVerts = dmesh->verts;
        p.detailVertsCount = dmesh->nverts;
        p.detailTris = dmesh->tris;
        p.detailTriCount = dmesh->ntris;
        p.walkableHeight = Agent::HEIGHT;
        p.walkableRadius = Agent::RADIUS;
        p.walkableClimb = Agent::CLIMB;
        p.tileX = tx;
        p.tileY = ty;
        p.tileLayer = 0;
        rcVcopy(p.bmin, pmesh->bmin);
        rcVcopy(p.bmax, pmesh->bmax);
        p.cs = cfg.cs;
        p.ch = cfg.ch;
        p.buildBvTree = true;
        unsigned char* data = nullptr;
        int size = 0;
        if (dtCreateNavMeshData(&p, &data, &size)) {
            out.assign(data, data + size);
            dtFree(data);
        }
    }
    rcFreePolyMesh(pmesh);
    rcFreePolyMeshDetail(dmesh);
    return out;
}

struct Route {
    std::string name;
    float from[3], to[3];       // GTA axes
};

std::vector<Route> loadRoutes(const std::string& path)
{
    std::vector<Route> out;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        size_t t1 = line.find('\t'), t2 = line.find('\t', t1 + 1);
        if (t1 == std::string::npos || t2 == std::string::npos) continue;
        Route r;
        r.name = line.substr(0, t1);
        std::istringstream a(line.substr(t1 + 1, t2 - t1 - 1)), b(line.substr(t2 + 1));
        a >> r.from[0] >> r.from[1] >> r.from[2];
        b >> r.to[0] >> r.to[1] >> r.to[2];
        out.push_back(r);
    }
    return out;
}

// Every route must be walkable end to end, by the same search the server runs.
// Writes each path to `paths` (for previews) when given. Returns the failures.
int checkRoutes(const std::string& navPath, const std::vector<Route>& routes, const std::string& pathsOut)
{
    Pathfinder finder;
    std::vector<std::string> log;
    if (!finder.loadFiles({ navPath }, log)) {
        for (const auto& l : log) std::fprintf(stderr, "[navbake] %s\n", l.c_str());
        return static_cast<int>(routes.size());
    }
    std::ofstream paths;
    if (!pathsOut.empty()) paths.open(pathsOut);
    int bad = 0;
    for (const Route& r : routes) {
        PathResult res = finder.findNow(Point { r.from[0], r.from[1], r.from[2] }, Point { r.to[0], r.to[1], r.to[2] });
        float length = 0.0f;
        for (size_t i = 1; i < res.points.size(); i++) {
            const Point &a = res.points[i - 1], &b = res.points[i];
            length += std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
        }
        if (res.status != PathStatus::Ok) {
            bad++;
            const char* why = res.status == PathStatus::NoStart ? "start is off the mesh"
                : res.status == PathStatus::NoEnd               ? "end is off the mesh"
                : res.status == PathStatus::Partial             ? "the end cannot be reached"
                                                                : "no path";
            std::printf("[navbake] route FAILED: %s: %s\n", r.name.c_str(), why);
        } else {
            std::printf("[navbake] route ok: %s (%.1f m, %zu points)\n", r.name.c_str(), length, res.points.size());
        }
        if (paths) {
            paths << r.name << '\t' << static_cast<int>(res.status);
            for (const Point& p : res.points) paths << '\t' << p.x << ' ' << p.y << ' ' << p.z;
            paths << '\n';
        }
    }
    return bad;
}

// Roofs, the tops of counters and planters: walkable, but nobody gets there on
// foot. Keeps only the polygons connected to a route's end or a seed (flag 0 elsewhere,
// which every search filters out), so a nearest or random point never lands on
// a roof. Returns the polygons kept.
int keepReachable(dtNavMesh* mesh, const std::vector<Route>& routes, const std::vector<std::array<float, 3>>& seeds,
    int& seedsOnMesh)
{
    dtNavMeshQuery* q = dtAllocNavMeshQuery();
    q->init(mesh, 64);
    dtQueryFilter filter;
    const float ext[3] = { 1.5f, 2.5f, 1.5f };
    std::vector<dtPolyRef> open;
    std::unordered_set<dtPolyRef> seen;
    const dtNavMesh* cmesh = mesh;
    auto seed = [&](const float* g) {
        float rc[3], hit[3];
        toRecast(g, rc);
        dtPolyRef ref = 0;
        q->findNearestPoly(rc, ext, &filter, &ref, hit);
        if (ref && seen.insert(ref).second) open.push_back(ref);
        return ref != 0;
    };
    for (const Route& r : routes) {
        seed(r.from);
        seed(r.to);
    }
    seedsOnMesh = 0;
    for (const auto& p : seeds) seedsOnMesh += seed(p.data());
    dtFreeNavMeshQuery(q);
    while (!open.empty()) {
        dtPolyRef ref = open.back();
        open.pop_back();
        const dtMeshTile* tile = nullptr;
        const dtPoly* poly = nullptr;
        cmesh->getTileAndPolyByRefUnsafe(ref, &tile, &poly);
        for (unsigned int l = poly->firstLink; l != DT_NULL_LINK; l = tile->links[l].next) {
            const dtPolyRef next = tile->links[l].ref;
            if (next && seen.insert(next).second) open.push_back(next);
        }
    }
    int kept = 0;
    for (int i = 0; i < mesh->getMaxTiles(); i++) {
        const dtMeshTile* tile = cmesh->getTile(i);
        if (!tile || !tile->header) continue;
        const dtPolyRef base = mesh->getPolyRefBase(tile);
        for (int p = 0; p < tile->header->polyCount; p++) {
            const dtPolyRef ref = base | static_cast<dtPolyRef>(p);
            const bool keep = seen.count(ref) != 0;
            mesh->setPolyFlags(ref, keep ? 1 : 0);
            kept += keep;
        }
    }
    return kept;
}

void dumpPolys(const dtNavMesh* mesh, const std::string& path)
{
    std::ofstream out(path);
    out << "# navbake polygons, GTA axes\n";
    int base = 1;
    for (int i = 0; i < mesh->getMaxTiles(); i++) {
        const dtMeshTile* tile = mesh->getTile(i);
        if (!tile || !tile->header) continue;
        for (int v = 0; v < tile->header->vertCount; v++) {
            float g[3];
            toGta(&tile->verts[v * 3], g);
            out << "v " << g[0] << ' ' << g[1] << ' ' << g[2] << '\n';
        }
        for (int p = 0; p < tile->header->polyCount; p++) {
            const dtPoly& poly = tile->polys[p];
            if (!poly.flags) continue;
            out << 'f';
            for (int k = 0; k < poly.vertCount; k++) out << ' ' << base + poly.verts[k];
            out << '\n';
        }
        base += tile->header->vertCount;
    }
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::fprintf(stderr, "usage: navbake <geometry.obj|.tris> <out.nav> [--routes routes.txt [--paths paths.txt]]\n"
                             "               [--seeds seeds.txt] [--water water.txt] [--dump polys.obj] [--threads N]\n");
        return 2;
    }
    std::string geomPath = argv[1], outPath = argv[2], routesPath, dumpPath, pathsPath, seedsPath, waterPath;
    int threads = static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
    for (int i = 3; i + 1 < argc; i += 2) {
        if (!std::strcmp(argv[i], "--routes")) routesPath = argv[i + 1];
        else if (!std::strcmp(argv[i], "--dump")) dumpPath = argv[i + 1];
        else if (!std::strcmp(argv[i], "--paths")) pathsPath = argv[i + 1];
        else if (!std::strcmp(argv[i], "--seeds")) seedsPath = argv[i + 1];
        else if (!std::strcmp(argv[i], "--water")) waterPath = argv[i + 1];
        else if (!std::strcmp(argv[i], "--threads")) threads = std::max(1, std::atoi(argv[i + 1]));
    }

    Geometry g;
    const bool tris = geomPath.size() > 5 && geomPath.compare(geomPath.size() - 5, 5, ".tris") == 0;
    if (!(tris ? loadTris(geomPath, g) : loadObj(geomPath, g))) {
        std::fprintf(stderr, "[navbake] %s: no geometry\n", geomPath.c_str());
        return 1;
    }
    const rcConfig cfg = baseConfig();
    bucketFaces(g, (cfg.borderSize + 1) * cfg.cs);
    const std::vector<Water> water = waterPath.empty() ? std::vector<Water> {} : loadWater(waterPath);
    const auto seeds = seedsPath.empty() ? std::vector<std::array<float, 3>> {} : loadPoints(seedsPath);

    // Tiles in parallel; each thread takes the next tile until none are left.
    std::vector<std::pair<int, int>> todo;
    for (int ty = g.ty0; ty <= g.ty1; ty++)
        for (int tx = g.tx0; tx <= g.tx1; tx++)
            if (!g.buckets[static_cast<size_t>(ty - g.ty0) * (g.tx1 - g.tx0 + 1) + (tx - g.tx0)].empty())
                todo.emplace_back(tx, ty);
    std::vector<std::vector<unsigned char>> baked(todo.size());
    std::atomic<size_t> next { 0 }, done { 0 };
    auto worker = [&] {
        Context ctx;
        for (size_t i; (i = next++) < todo.size();) {
            baked[i] = bakeTile(ctx, g, water, todo[i].first, todo[i].second);
            const size_t d = ++done;
            if (todo.size() >= 200 && d % (todo.size() / 20) == 0)
                std::printf("[navbake] %zu%% of %zu tiles\n", d * 100 / todo.size(), todo.size()), std::fflush(stdout);
        }
    };
    std::vector<std::thread> pool;
    for (int i = 0; i < std::min<int>(threads, static_cast<int>(todo.size())); i++) pool.emplace_back(worker);
    for (auto& t : pool) t.join();

    NavFile file;
    for (size_t i = 0; i < todo.size(); i++) {
        if (baked[i].empty()) continue;
        Tile t;
        t.x = todo[i].first;
        t.y = todo[i].second;
        t.data = std::move(baked[i]);
        file.tiles.push_back(std::move(t));
    }
    std::printf("[navbake] %s: %zu triangles, %d of %zu tiles walkable (%d threads)\n", geomPath.c_str(),
        g.tris.size() / 6, static_cast<int>(file.tiles.size()), todo.size(), threads);
    if (file.tiles.empty()) {
        std::fprintf(stderr, "[navbake] nothing walkable\n");
        return 1;
    }

    // Load what was baked the way the component will, to check it.
    dtNavMesh* mesh = dtAllocNavMesh();
    dtNavMeshParams params = worldParams(static_cast<int>(file.tiles.size()));
    if (dtStatusFailed(mesh->init(&params))) {
        std::fprintf(stderr, "[navbake] dtNavMesh init failed\n");
        return 1;
    }
    int polys = 0;
    for (Tile& t : file.tiles) {
        if (dtStatusFailed(mesh->addTile(t.data.data(), static_cast<int>(t.data.size()), 0, 0, nullptr))) {
            std::fprintf(stderr, "[navbake] tile %d,%d does not load\n", t.x, t.y);
            return 1;
        }
        polys += reinterpret_cast<const dtMeshHeader*>(t.data.data())->polyCount;
    }
    std::vector<Route> routes;
    if (!routesPath.empty()) routes = loadRoutes(routesPath);
    if (!routes.empty() || !seeds.empty()) {
        // The flags live in the tile data the mesh points into, so the file gets them.
        int seedsOnMesh = 0;
        const int kept = keepReachable(mesh, routes, seeds, seedsOnMesh);
        std::printf("[navbake] %d polygons, %d of them connected to a route or path node (the rest are roofs and tops)\n",
            polys, kept);
        if (!seeds.empty())
            std::printf("[navbake] path nodes on the mesh: %d of %zu (%.1f%%)\n", seedsOnMesh, seeds.size(),
                100.0 * seedsOnMesh / seeds.size());
    } else {
        std::printf("[navbake] %d polygons\n", polys);
    }

    if (!dumpPath.empty()) dumpPolys(mesh, dumpPath);
    dtFreeNavMesh(mesh);    // the tiles were added without DT_TILE_FREE_DATA: file owns them

    std::string error;
    if (!writeNavFile(outPath, file, error)) {
        std::fprintf(stderr, "[navbake] %s\n", error.c_str());
        return 1;
    }
    std::printf("[navbake] wrote %s\n", outPath.c_str());
    // Check the file as written, loaded the way the component loads it.
    const int bad = routes.empty() ? 0 : checkRoutes(outPath, routes, pathsPath);
    return bad ? 1 : 0;
}
