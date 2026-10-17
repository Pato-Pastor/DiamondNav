#include "pathfinder.hpp"
#include "collision.hpp"
#include "navfile.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <filesystem>
#include <map>
#include <unordered_map>

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

// Polygon flags: 1 every polygon (navbake), LEDGE the drops addLedges adds, which
// only a search that asks for them crosses.
constexpr unsigned short FLAG_LEDGE = 2;
constexpr unsigned char AREA_LEDGE = 1;
constexpr float MOMENTUM_FROM = 6.0f;   // m/s: slower than this, the way from where it is
constexpr float MOMENTUM_AHEAD = 1.5f;  // s: the ways from this far on along its heading, and twice it...
constexpr float MOMENTUM_MIN = 10.0f;   // m: ...no nearer...
constexpr float MOMENTUM_MAX = 60.0f;   // ...nor further
constexpr float MOMENTUM_LEG = 8.0f;    // m: a way turns the car round when a leg starting this near...
constexpr float MOMENTUM_STUB = 0.5f;   // (and longer than this)
constexpr float MOMENTUM_BACK = -0.17f; // ...heads back more than 100 degrees from its heading
constexpr float ROAD_WEIGH = 60.0f;     // m: a chase further than this across the car mesh is weighed against the roads
constexpr float OFF_DECK = 1.5f;        // a metre off every road's deck counts this many (dirt, kerbs, verges)
constexpr float DECK_WIDER = 2.0f;      // m: a road's deck, widened by this either side
constexpr float DECK_STEP = 4.0f;       // m: how often a leg is sampled for it
constexpr float STEEP = 0.12f;          // a grade steeper than this...
constexpr float STEEP_COST = 5.0f;      // ...costs this much more per unit of grade past it
constexpr float JOIN_NEAR = 6.0f;       // m: the way onto a route's first point gets this near it
constexpr int JOIN_TRIES = 4;           // roads tried to start from when there is no way onto one
constexpr float JOIN_BUMPER = 0.6f;     // m over the ground: the walls the way onto a road may not meet
constexpr float LEDGE_COST = 3.0f;      // a drop costs as much as driving three times its length
// Ledges: where the car mesh stops at an edge with more of it below.
constexpr float LEDGE_SPACING = 4.0f;   // a drop every this much along an edge of the mesh
constexpr float LEDGE_INSET = 0.3f;     // it starts this far in from the mesh's edge
constexpr float LEDGE_STEP = 0.5f;      // the real edge is looked for this often...
constexpr float LEDGE_REACH = 5.0f;     // ...this far past the mesh's (the mesh keeps a car's radius back)
constexpr float LEDGE_LEVEL = 0.6f;     // the ground up to the edge climbs no more than this a step...
constexpr float LEDGE_FALL = 1.5f;      // ...and the edge is where it is this much lower than at the start
constexpr float LEDGE_DOWN = 12.0f;     // past it, the mesh below is looked for this far out (down a bank)
constexpr float LEDGE_MIN = 2.0f;       // a drop at least this far...
constexpr float LEDGE_MAX = 15.0f;      // ...and at most this far
constexpr float LEDGE_LAND = 1.5f;      // the mesh below this near where it gets down
constexpr float LEDGE_LOW = 0.4f;       // m over the ground: a railing, a kerb's wall
constexpr float LEDGE_HIGH = 1.1f;      // ...and a rail with a gap under it
constexpr float LEDGE_SIDE = 1.6f;      // a car's half width and some, either side of the way off
constexpr float LEDGE_RUNUP = 10.0f;    // it starts this far back from the edge, square on to it, where there is room
constexpr float LEDGE_APART = 2.5f;     // two drops this near on one tile are one

dtQueryFilter plainFilter()
{
    dtQueryFilter f;
    f.setExcludeFlags(FLAG_LEDGE);
    return f;
}

dtQueryFilter ledgeFilter()
{
    dtQueryFilter f;
    f.setAreaCost(AREA_LEDGE, LEDGE_COST);
    return f;
}

struct Ledge {
    float start[3], end[3];     // Recast axes
};

// Detour's own: which neighbour tile a point outside a tile's box is in (0xff inside).
unsigned char sideOf(const float* pt, const float* bmin, const float* bmax)
{
    unsigned char out = 0;
    out |= (pt[0] >= bmax[0]) ? 1 : 0;
    out |= (pt[2] >= bmax[2]) ? 2 : 0;
    out |= (pt[0] < bmin[0]) ? 4 : 0;
    out |= (pt[2] < bmin[2]) ? 8 : 0;
    switch (out) {
    case 1: return 0;
    case 1 | 2: return 1;
    case 2: return 2;
    case 4 | 2: return 3;
    case 4: return 4;
    case 4 | 8: return 5;
    case 8: return 6;
    case 1 | 8: return 7;
    }
    return 0xff;
}

int align4(int x)
{
    return (x + 3) & ~3;
}

// A tile's data (dtCreateNavMeshData's) again with one-way off-mesh connections
// added, laid out as Detour lays them out: the same polygons, detail and tree, the
// connections' two points after the vertices, a polygon each after the polygons,
// and room for their links. False when the data is not as expected.
bool withLedges(const std::vector<unsigned char>& in, const std::vector<Ledge>& ledges, std::vector<unsigned char>& out)
{
    if (in.size() < sizeof(dtMeshHeader)) return false;
    dtMeshHeader h;
    std::memcpy(&h, in.data(), sizeof h);
    if (h.magic != DT_NAVMESH_MAGIC || h.version != DT_NAVMESH_VERSION || h.offMeshConCount != 0
        || h.offMeshBase != h.polyCount)
        return false;
    const int n = static_cast<int>(ledges.size());
    const int headerSize = align4(sizeof(dtMeshHeader));
    const int vertsSize = align4(sizeof(float) * 3 * h.vertCount);
    const int polysSize = align4(sizeof(dtPoly) * h.polyCount);
    const int linksSize = align4(sizeof(dtLink) * h.maxLinkCount);
    const int detailSize = align4(sizeof(dtPolyDetail) * h.detailMeshCount)
        + align4(sizeof(float) * 3 * h.detailVertCount) + align4(4 * h.detailTriCount);
    const int bvSize = align4(sizeof(dtBVNode) * h.bvNodeCount);
    if (headerSize + vertsSize + polysSize + linksSize + detailSize + bvSize != static_cast<int>(in.size())) return false;
    if (h.vertCount + 2 * n >= 0xffff) return false;

    dtMeshHeader o = h;
    o.vertCount = h.vertCount + 2 * n;
    o.polyCount = h.polyCount + n;
    o.maxLinkCount = h.maxLinkCount + 4 * n;   // its start (both ways) and its end, with room to spare
    o.offMeshConCount = n;
    o.offMeshBase = h.polyCount;
    const int vertsOut = align4(sizeof(float) * 3 * o.vertCount);
    const int polysOut = align4(sizeof(dtPoly) * o.polyCount);
    const int linksOut = align4(sizeof(dtLink) * o.maxLinkCount);
    const int consOut = align4(sizeof(dtOffMeshConnection) * n);
    out.assign(headerSize + vertsOut + polysOut + linksOut + detailSize + bvSize + consOut, 0);

    const unsigned char* src = in.data() + headerSize;
    unsigned char* dst = out.data();
    std::memcpy(dst, &o, sizeof o);
    dst += headerSize;
    float* verts = reinterpret_cast<float*>(dst);
    std::memcpy(verts, src, sizeof(float) * 3 * h.vertCount);
    for (int i = 0; i < n; i++) {
        std::memcpy(&verts[(h.vertCount + i * 2) * 3], ledges[i].start, sizeof(float) * 3);
        std::memcpy(&verts[(h.vertCount + i * 2 + 1) * 3], ledges[i].end, sizeof(float) * 3);
    }
    src += vertsSize;
    dst += vertsOut;
    dtPoly* polys = reinterpret_cast<dtPoly*>(dst);
    std::memcpy(polys, src, sizeof(dtPoly) * h.polyCount);
    for (int i = 0; i < n; i++) {
        dtPoly& p = polys[h.polyCount + i];
        p.vertCount = 2;
        p.verts[0] = static_cast<unsigned short>(h.vertCount + i * 2);
        p.verts[1] = static_cast<unsigned short>(h.vertCount + i * 2 + 1);
        p.flags = FLAG_LEDGE;
        p.setArea(AREA_LEDGE);
        p.setType(DT_POLYTYPE_OFFMESH_CONNECTION);
    }
    src += polysSize + linksSize;   // links are rebuilt when the tile is added
    dst += polysOut + linksOut;
    std::memcpy(dst, src, detailSize + bvSize);
    dst += detailSize + bvSize;
    dtOffMeshConnection* cons = reinterpret_cast<dtOffMeshConnection*>(dst);
    for (int i = 0; i < n; i++) {
        dtOffMeshConnection& c = cons[i];
        std::memcpy(c.pos, ledges[i].start, sizeof(float) * 3);
        std::memcpy(c.pos + 3, ledges[i].end, sizeof(float) * 3);
        c.rad = 1.0f;
        c.poly = static_cast<unsigned short>(h.polyCount + i);
        c.flags = 0;                // one way: down
        c.side = sideOf(ledges[i].end, h.bmin, h.bmax);
        c.userId = static_cast<unsigned int>(i);
    }
    return true;
}

// The drops off one tile's ledges: along every edge of its polygons with nothing
// beyond it, out square to the edge, where the ground the mesh is on runs level
// (no wall, no railing in the way) to an edge it falls away from, and comes down
// LEDGE_MIN..LEDGE_MAX lower beside more of the car mesh.
void findLedges(const dtNavMeshQuery& q, const dtMeshTile& tile, const Collision& world, std::vector<Ledge>& out)
{
    const dtQueryFilter filter = plainFilter();
    RayHit hit;
    for (int pi = 0; pi < tile.header->polyCount; pi++) {
        const dtPoly& poly = tile.polys[pi];
        if (poly.getType() != DT_POLYTYPE_GROUND || !(poly.flags & 1)) continue;
        // Its middle, to tell the edge's outside from its inside.
        float cx = 0.0f, cy = 0.0f;
        for (int v = 0; v < poly.vertCount; v++) {
            float g[3];
            toGta(&tile.verts[poly.verts[v] * 3], g);
            cx += g[0];
            cy += g[1];
        }
        cx /= poly.vertCount;
        cy /= poly.vertCount;
        for (int e = 0; e < poly.vertCount; e++) {
            if (poly.neis[e] != 0) continue;   // another polygon, or the next tile
            float a[3], b[3];
            toGta(&tile.verts[poly.verts[e] * 3], a);
            toGta(&tile.verts[poly.verts[(e + 1) % poly.vertCount] * 3], b);
            const float ex = b[0] - a[0], ey = b[1] - a[1], len = std::hypot(ex, ey);
            if (len < 1.0f) continue;
            float nx = ey / len, ny = -ex / len;
            if (nx * ((a[0] + b[0]) * 0.5f - cx) + ny * ((a[1] + b[1]) * 0.5f - cy) < 0.0f) {
                nx = -nx;
                ny = -ny;
            }
            const int count = std::max(1, static_cast<int>(len / LEDGE_SPACING));
            for (int k = 0; k < count; k++) {
                const float t = (k + 0.5f) / count;
                const float sx = a[0] + ex * t - nx * LEDGE_INSET, sy = a[1] + ey * t - ny * LEDGE_INSET;
                float level;
                if (!world.ground(sx, sy, a[2] + (b[2] - a[2]) * t, 1.0f, 1.5f, level)) continue;
                // Out to where the ground falls away, never up a wall.
                float lip = -1.0f, top = level;
                for (float d = LEDGE_STEP; d <= LEDGE_REACH; d += LEDGE_STEP) {
                    float gz;
                    // (Falling away: below where it started by more than LEDGE_FALL, so
                    // the top of a bank counts as much as a wall's.)
                    const bool on = world.ground(sx + nx * d, sy + ny * d, top, LEDGE_LEVEL + 1.0f,
                        top - level + LEDGE_FALL, gz);
                    if (on && gz > top + LEDGE_LEVEL) break;        // a step up, a wall's top: no edge
                    if (!on) {
                        lip = d;
                        break;
                    }
                    top = gz;
                }
                if (lip < 0.0f) continue;
                // Past it, straight down or down a bank, out to where the mesh is below.
                float lx = 0.0f, ly = 0.0f, landed[3], landedRc[3];
                bool land = false;
                {
                    float prev = top;
                    for (float d = lip + 1.0f; d <= lip + LEDGE_DOWN; d += 1.0f) {
                        const float x = sx + nx * d, y = sy + ny * d;
                        float gz;
                        if (!world.ground(x, y, prev, LEDGE_LEVEL, LEDGE_MAX, gz) || top - gz > LEDGE_MAX) break;
                        prev = gz;
                        if (top - gz < LEDGE_MIN) continue;
                        float rc[3];
                        const float p[3] = { x, y, gz };
                        toRecast(p, rc);
                        const float ext[3] = { LEDGE_LAND, 1.0f, LEDGE_LAND };
                        dtPolyRef ref = 0;
                        q.findNearestPoly(rc, ext, &filter, &ref, landedRc);
                        if (!ref) continue;
                        toGta(landedRc, landed);
                        if (std::hypot(landed[0] - x, landed[1] - y) > LEDGE_LAND || std::fabs(landed[2] - gz) > 1.0f) continue;
                        lx = x;
                        ly = y;
                        land = true;
                        break;
                    }
                }
                if (!land) continue;
                // Nothing in the way of a car's width going off it, low or high, from a
                // run-up square on to the edge (where the mesh has room for one) and
                // just past the edge, where it leaves it flying level.
                const float px = sx + nx * (lip + 0.5f), py = sy + ny * (lip + 0.5f);
                const float fly = std::min(std::hypot(lx - px, ly - py), 3.0f);
                if (world.wall({ px, py, top + LEDGE_LOW }, { px + nx * fly, py + ny * fly, top + LEDGE_LOW }, hit)) continue;
                auto clear = [&](float fx, float fy, float fz) {
                    for (float side : { -LEDGE_SIDE, 0.0f, LEDGE_SIDE }) {
                        const float ox = -ny * side, oy = nx * side;
                        for (float up : { LEDGE_LOW, LEDGE_HIGH })
                            if (world.wall({ fx + ox, fy + oy, fz + up }, { px + ox, py + oy, top + up }, hit)) return false;
                    }
                    return true;
                };
                float bx = sx, by = sy, bz = level;
                bool ok = false;
                for (float back : { LEDGE_RUNUP, LEDGE_RUNUP * 0.6f, LEDGE_RUNUP * 0.3f, 0.0f }) {
                    const float rx = sx - nx * back, ry = sy - ny * back;
                    float rz = level;
                    if (back > 0.0f) {
                        if (!world.ground(rx, ry, level, LEDGE_LEVEL, LEDGE_LEVEL, rz)) continue;
                        float rc[3], onRc[3];
                        const float p[3] = { rx, ry, rz };
                        toRecast(p, rc);
                        const float ext[3] = { 0.5f, 1.0f, 0.5f };
                        dtPolyRef ref = 0;
                        q.findNearestPoly(rc, ext, &filter, &ref, onRc);
                        if (!ref || std::hypot(onRc[0] - rc[0], onRc[2] - rc[2]) > 0.3f) continue;
                    }
                    if (!clear(rx, ry, rz)) continue;
                    bx = rx;
                    by = ry;
                    bz = rz;
                    ok = true;
                    break;
                }
                if (!ok) continue;
                Ledge l;
                const float start[3] = { bx, by, bz };
                toRecast(start, l.start);
                std::memcpy(l.end, landedRc, sizeof l.end);
                bool near = false;
                for (const Ledge& o : out)
                    if (dtVdist(o.start, l.start) < LEDGE_APART) {
                        near = true;
                        break;
                    }
                if (!near) out.push_back(l);
            }
        }
    }
}

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

int Pathfinder::addLedges(const Collision& world, float& seconds)
{
    const auto began = std::chrono::steady_clock::now();
    seconds = 0.0f;
    Mesh& m = meshes_[static_cast<int>(MeshKind::Car)];
    if (world.loaded()) world_ = &world;
    if (!m.mesh || !world.loaded() || ledges_) return 0;
    const dtNavMesh* cmesh = m.mesh;
    std::vector<const dtMeshTile*> tiles;
    for (int i = 0; i < m.mesh->getMaxTiles(); i++) {
        const dtMeshTile* t = cmesh->getTile(i);
        if (t && t->header) tiles.push_back(t);
    }
    // Found on every core, each with its own query; the mesh is only read.
    std::vector<std::vector<Ledge>> found(tiles.size());
    std::atomic<size_t> next { 0 };
    auto work = [&]() {
        dtNavMeshQuery* q = dtAllocNavMeshQuery();
        q->init(m.mesh, 256);
        for (size_t i; (i = next++) < tiles.size();) findLedges(*q, *tiles[i], world, found[i]);
        dtFreeNavMeshQuery(q);
    };
    const int cores = std::clamp(static_cast<int>(std::thread::hardware_concurrency()), 1, 16);
    std::vector<std::thread> pool;
    for (int i = 1; i < cores; i++) pool.emplace_back(work);
    work();
    for (auto& t : pool) t.join();

    // Each tile with drops goes back in with them.
    std::unordered_map<const unsigned char*, std::vector<unsigned char>*> owner;
    for (auto& d : data_) owner[d.data()] = &d;
    for (size_t i = 0; i < tiles.size(); i++) {
        if (found[i].empty()) continue;
        auto it = owner.find(tiles[i]->data);
        if (it == owner.end()) continue;
        std::vector<unsigned char>& data = *it->second;
        std::vector<unsigned char> with;
        if (!withLedges(data, found[i], with)) continue;
        const dtTileRef ref = m.mesh->getTileRef(tiles[i]);
        if (dtStatusFailed(m.mesh->removeTile(ref, nullptr, nullptr))) continue;
        if (dtStatusSucceed(m.mesh->addTile(with.data(), static_cast<int>(with.size()), 0, ref, nullptr))) {
            data.swap(with);
            ledges_ += static_cast<int>(found[i].size());
        } else {
            m.mesh->addTile(data.data(), static_cast<int>(data.size()), 0, ref, nullptr);
        }
    }
    seconds = std::chrono::duration<float>(std::chrono::steady_clock::now() - began).count();
    return ledges_;
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
        PathResult res = r.kind == PathKind::Drive || !mq ? findRoute(r, q[static_cast<int>(MeshKind::Car)]) : find(*mq, r);
        std::lock_guard<std::mutex> lock(mutex_);
        done_.push_back(std::move(res));
    }
    for (auto* qq : q)
        if (qq) dtFreeNavMeshQuery(qq);
}

PathResult Pathfinder::findRoute(const PathRequest& r, dtNavMeshQuery* car) const
{
    // The road graph knows nothing of walls: the road it starts on can be one the car
    // has no straight way onto (below the terrace it is on, up a ramp past its kerb). A
    // car that drove at it would sit nose to the wall: it goes round to it across the
    // car mesh, and where even that meets a wall, the route starts from another road.
    std::vector<uint32_t> notFrom;
    PathResult first;
    for (int attempt = 0; attempt < JOIN_TRIES; attempt++) {
        PathResult res;
        res.id = r.id;
        res.owner = r.owner;
        res.kind = r.kind;
        RouteOptions opt = r.road;
        opt.notFrom = &notFrom;
        std::vector<RoutePoint> pts;
        uint32_t start = 0;
        res.status = static_cast<PathStatus>(roads_.route(r.from.x, r.from.y, r.from.z, r.to.x, r.to.y, r.to.z, opt, pts, &start));
        for (const auto& p : pts) {
            res.points.push_back(Point { p.x, p.y, p.z });
            res.speeds.push_back(p.speed);
            res.classes.push_back(p.cls);
            res.junctions.push_back(p.junction ? 1 : 0);
        }
        if (res.status != PathStatus::Ok || res.points.empty() || !car || r.road.boats)
            return attempt == 0 ? res : first;
        if (joinRoute(*car, r, res)) {
            // (The way onto the road: from the car to the road's first point.)
            std::vector<Point> way { r.from };
            std::vector<uint8_t> drops { 0 };
            for (size_t i = 0; i <= res.joined && i < res.points.size(); i++) {
                way.push_back(res.points[i]);
                drops.push_back(i < res.ledges.size() ? res.ledges[i] : 0);
            }
            if (clearOfWalls(way, drops)) return res;
        }
        if (attempt == 0) first = std::move(res);
        notFrom.push_back(start);
    }
    return first;
}

// Prepends the way from r.from onto res's first point across the car mesh, when the
// car can't drive straight to it. False when the car is off the mesh or the mesh has
// no way there (res is then as it was).
bool Pathfinder::joinRoute(dtNavMeshQuery& car, const PathRequest& r, PathResult& res) const
{
    const float* reach = reachOf(MeshKind::Car);
    const Point& first = res.points.front();
    float s[3], ns[3];
    dtPolyRef ref = 0;
    toRecast(&r.from.x, s);
    const dtQueryFilter filter = plainFilter();
    car.findNearestPoly(s, reach, &filter, &ref, ns);
    if (!ref) return false;
    if (lineClearOn(car, reach, r.from, first)) return true;
    PathRequest on = r;
    on.kind = PathKind::OffRoad;
    on.to = first;
    PathResult way = findMesh(car, on);
    if (way.status != PathStatus::Ok || way.points.size() < 2
        || std::hypot(way.points.back().x - first.x, way.points.back().y - first.y) > JOIN_NEAR)
        return false;
    way.points.pop_back();
    way.speeds.pop_back();
    way.classes.pop_back();
    way.junctions.pop_back();
    way.ledges.pop_back();
    res.ledges.assign(res.points.size(), 0);
    res.points.insert(res.points.begin(), way.points.begin(), way.points.end());
    res.speeds.insert(res.speeds.begin(), way.speeds.begin(), way.speeds.end());
    res.classes.insert(res.classes.begin(), way.classes.begin(), way.classes.end());
    res.junctions.insert(res.junctions.begin(), way.junctions.begin(), way.junctions.end());
    res.ledges.insert(res.ledges.begin(), way.ledges.begin(), way.ledges.end());
    res.joined = way.points.size();
    return true;
}

bool Pathfinder::clearOfWalls(const std::vector<Point>& pts, const std::vector<uint8_t>& drops) const
{
    const Collision* world = world_;
    if (!world) return true;
    RayHit hit;
    for (size_t i = 1; i < pts.size(); i++) {
        if (i - 1 < drops.size() && drops[i - 1]) continue;     // a drop off a ledge is a fall, not a drive
        const Vec3 a { pts[i - 1].x, pts[i - 1].y, pts[i - 1].z + JOIN_BUMPER };
        const Vec3 b { pts[i].x, pts[i].y, pts[i].z + JOIN_BUMPER };
        if (world->wall(a, b, hit)) return false;
    }
    return true;
}

// Across the car mesh with a heading and a speed (a chase): the mesh knows neither,
// so the way from where the car is, which may start by turning it round, is weighed
// against the way from a little further on along its heading, which keeps its
// momentum: a turn round costs what stopping, turning and getting back up to speed
// costs at that speed (turnRoundCost). Further off, the mesh's shortest way (often
// straight across a hillside or a verge) is weighed against the roads' way too: a
// metre off the road or up a steep grade costs more than one on it.
PathResult Pathfinder::find(dtNavMeshQuery& q, const PathRequest& r) const
{
    PathResult here = findMesh(q, r);
    if (r.kind != PathKind::OffRoad || !r.road.hasHeading) return here;
    const float h = r.road.heading * 0.01745329f, fx = -std::sin(h), fy = std::cos(h);
    const bool moving = r.road.speed >= MOMENTUM_FROM;
    const float turn = moving ? turnRoundCost(r.road.speed) : 0.0f;
    const bool roads = roads_.loaded();
    float deck = 0.0f;
    auto onDeck = [&](float x, float y, float z) { return roads_.deckAt(x, y, z, deck, DECK_WIDER); };
    auto cost = [&](const PathResult& p) {
        if (p.status != PathStatus::Ok && p.status != PathStatus::Partial) return std::numeric_limits<float>::infinity();
        float len = 0.0f, turns = 0.0f, total = 0.0f;
        for (size_t i = 1; i < p.points.size(); i++) {
            const Point& a = p.points[i - 1];
            const Point& b = p.points[i];
            const float dx = b.x - a.x, dy = b.y - a.y;
            const float l = std::hypot(dx, dy);
            // (A leg of its first few metres heading back the way the car came: a turn
            // round. Its first leg alone missed a way that doubles back round a post
            // and then runs sideways.)
            if (moving && len < MOMENTUM_LEG && l > MOMENTUM_STUB && (dx * fx + dy * fy) / l < MOMENTUM_BACK)
                turns = turn;
            len += l;
            if (l < 1e-3f) continue;
            float w = 1.0f;
            if (roads && (i >= p.classes.size() || p.classes[i] == RoadClass::OffRoad)) {
                const int n = std::max(1, static_cast<int>(l / DECK_STEP));
                int off = 0;
                for (int k = 0; k < n; k++) {
                    const float t = (k + 0.5f) / n;
                    if (!onDeck(a.x + dx * t, a.y + dy * t, a.z + (b.z - a.z) * t)) off++;
                }
                w += (OFF_DECK - 1.0f) * off / n;
            }
            if (i - 1 >= p.ledges.size() || !p.ledges[i - 1])
                w += STEEP_COST * std::max(0.0f, std::fabs(b.z - a.z) / l - STEEP);
            total += l * w;
        }
        // (Short of the end, the rest as the crow flies, off the road.)
        if (!p.points.empty()) total += std::hypot(r.to.x - p.points.back().x, r.to.y - p.points.back().y) * OFF_DECK;
        return total + turns;
    };
    float best = cost(here);
    PathResult pick = std::move(here);
    if (moving && best >= turn * 0.5f) {    // (else nowhere near worth weighing)
        for (float secs : { MOMENTUM_AHEAD, MOMENTUM_AHEAD * 2.0f }) {
            const float d = std::clamp(r.road.speed * secs, MOMENTUM_MIN, MOMENTUM_MAX);
            Point ahead { r.from.x + fx * d, r.from.y + fy * d, r.from.z };
            if (!lineClearOn(q, reachOf(MeshKind::Car), r.from, ahead)) continue;
            PathRequest on = r;
            on.from = ahead;
            PathResult there = findMesh(q, on);
            if (there.status != PathStatus::Ok && there.status != PathStatus::Partial) continue;
            // From the car, on to there, then that way.
            there.points.insert(there.points.begin(), r.from);
            there.speeds.insert(there.speeds.begin(), 1.0f);
            there.classes.insert(there.classes.begin(), RoadClass::OffRoad);
            there.junctions.insert(there.junctions.begin(), 0);
            there.ledges.insert(there.ledges.begin(), 0);
            if (there.speeds.size() > 1) there.speeds[1] = 1.0f;
            const float c = cost(there);
            if (c < best) {
                best = c;
                pick = std::move(there);
            }
        }
    }
    if (roads && std::hypot(r.to.x - r.from.x, r.to.y - r.from.y) > ROAD_WEIGH) {
        PathRequest drive = r;
        drive.kind = PathKind::Drive;
        PathResult road = findRoute(drive, &q);
        if (road.status == PathStatus::Ok && road.points.size() >= 2 && cost(road) < best) return road;
    }
    return pick;
}

PathResult Pathfinder::findMesh(dtNavMeshQuery& q, const PathRequest& r) const
{
    PathResult res;
    res.id = r.id;
    res.owner = r.owner;
    res.kind = r.kind;
    const float* reach = reachOf(meshOf(r.kind));
    const dtQueryFilter filter = r.kind == PathKind::OffRoad && r.road.ledges ? ledgeFilter() : plainFilter();
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
    unsigned char cornerFlags[MAX_CORNERS];
    q.findStraightPath(ns, ne, polys, npolys, corners, cornerFlags, nullptr, &ncorners, MAX_CORNERS);
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
    std::vector<uint8_t> ledgeAt { static_cast<uint8_t>(cornerFlags[0] & DT_STRAIGHTPATH_OFFMESH_CONNECTION ? 1 : 0) };
    pts.insert(pts.end(), corners, corners + 3);
    int at = 0;     // corridor polygon reached so far
    for (int c = 1; c < ncorners; c++) {
        const float* a = &corners[(c - 1) * 3];
        const float* b = &corners[c * 3];
        const float len = std::sqrt((b[0] - a[0]) * (b[0] - a[0]) + (b[2] - a[2]) * (b[2] - a[2]));
        // (A drop off a ledge is a straight line: no ground under it to follow.)
        const bool ledge = cornerFlags[c - 1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION;
        const int steps = ledge ? 0 : static_cast<int>(len / SAMPLE_STEP);
        for (int k = 1; k < steps; k++) {
            float p[3];
            dtVlerp(p, a, b, static_cast<float>(k) / steps);
            for (int i = at; i < npolys; i++) {
                // (A drop off a ledge has a height anywhere along it.)
                const dtMeshTile* tile = nullptr;
                const dtPoly* poly = nullptr;
                q.getAttachedNavMesh()->getTileAndPolyByRefUnsafe(polys[i], &tile, &poly);
                if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) continue;
                float h;
                if (dtStatusSucceed(q.getPolyHeight(polys[i], p, &h))) {
                    p[1] = h;
                    at = i;
                    break;
                }
            }
            pts.insert(pts.end(), p, p + 3);
            corner.push_back(false);
            ledgeAt.push_back(0);
        }
        pts.insert(pts.end(), b, b + 3);
        corner.push_back(true);
        ledgeAt.push_back(cornerFlags[c] & DT_STRAIGHTPATH_OFFMESH_CONNECTION ? 1 : 0);
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
    if (r.kind == PathKind::OffRoad) {
        offRoadSpeeds(res);
        for (int i : keep) res.ledges.push_back(ledgeAt[i]);
    }
    return res;
}

// A car slows for each turn by how sharp it is, and for the end.
void Pathfinder::offRoadSpeeds(PathResult& res)
{
    const auto& p = res.points;
    res.speeds.assign(p.size(), 1.0f);
    res.classes.assign(p.size(), RoadClass::OffRoad);
    res.junctions.assign(p.size(), 0);
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

PathResult Pathfinder::findNow(const Point& from, const Point& to, MeshKind k, bool ledges) const
{
    PathRequest r;
    r.kind = k == MeshKind::Car ? PathKind::OffRoad : PathKind::Walk;
    r.road.ledges = ledges;
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
    const dtQueryFilter filter = plainFilter();
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
    const dtQueryFilter filter = plainFilter();
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
    const dtQueryFilter filter = plainFilter();
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
    return q && lineClearOn(*q, reachOf(k), from, to);
}

bool Pathfinder::lineClearOn(const dtNavMeshQuery& q, const float* reach, const Point& from, const Point& to)
{
    const dtQueryFilter filter = plainFilter();
    float s[3], e[3], ns[3], ne[3];
    toRecast(&from.x, s);
    toRecast(&to.x, e);
    dtPolyRef sref = 0, eref = 0;
    q.findNearestPoly(s, reach, &filter, &sref, ns);
    q.findNearestPoly(e, reach, &filter, &eref, ne);
    if (!sref || !eref) return false;
    // The ray runs along the mesh's surface; t stays FLT_MAX when it reaches the
    // end without leaving the mesh. It aims at the end itself, not the nearest point
    // on the mesh, so an end just off the mesh's edge is not clear.
    float t = 0.0f, normal[3];
    dtPolyRef path[256];
    int npath = 0;
    if (dtStatusFailed(q.raycast(sref, ns, e, &filter, &t, normal, path, &npath, 256))) return false;
    if (t < 1.0f || npath == 0) return false;
    // It reached the end's x and z; the end's polygon must be on the same floor.
    return path[npath - 1] == eref;
}

} // namespace dnav
