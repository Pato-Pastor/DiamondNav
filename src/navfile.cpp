#include "navfile.hpp"

#include <cmath>
#include <cstdio>
#include <memory>

#include <DetourCommon.h>

namespace dnav {

namespace {

struct FileCloser {
    void operator()(FILE* f) const { if (f) std::fclose(f); }
};
using FilePtr = std::unique_ptr<FILE, FileCloser>;

// Bake settings are part of the header: a file baked with other settings than
// the component expects is refused, not misread.
struct Header {
    uint32_t magic;
    uint32_t version;
    uint32_t polyRefBits;
    float cell, cellHeight, tileSize;
    float origin[3];
    float agentHeight, agentRadius, agentClimb;
    uint32_t tileCount;
};

Header expectedHeader()
{
    Header h {};
    h.magic = FILE_MAGIC;
    h.version = FILE_VERSION;
    h.polyRefBits = sizeof(dtPolyRef) * 8;
    h.cell = Grid::CELL;
    h.cellHeight = Grid::CELL_HEIGHT;
    h.tileSize = Grid::TILE_SIZE;
    h.origin[0] = Grid::ORIGIN_X;
    h.origin[1] = Grid::ORIGIN_Y;
    h.origin[2] = Grid::ORIGIN_Z;
    h.agentHeight = Agent::HEIGHT;
    h.agentRadius = Agent::RADIUS;
    h.agentClimb = Agent::CLIMB;
    return h;
}

} // namespace

bool writeNavFile(const std::string& path, const NavFile& file, std::string& error)
{
    FilePtr f(std::fopen(path.c_str(), "wb"));
    if (!f) {
        error = "cannot write " + path;
        return false;
    }
    Header h = expectedHeader();
    h.tileCount = static_cast<uint32_t>(file.tiles.size());
    bool ok = std::fwrite(&h, sizeof h, 1, f.get()) == 1;
    for (const Tile& t : file.tiles) {
        int32_t meta[3] = { t.x, t.y, static_cast<int32_t>(t.data.size()) };
        ok = ok && std::fwrite(meta, sizeof meta, 1, f.get()) == 1;
        ok = ok && std::fwrite(t.data.data(), 1, t.data.size(), f.get()) == t.data.size();
    }
    if (!ok)
        error = "short write to " + path;
    return ok;
}

bool readNavFile(const std::string& path, NavFile& file, std::string& error)
{
    FilePtr f(std::fopen(path.c_str(), "rb"));
    if (!f) {
        error = "cannot open " + path;
        return false;
    }
    Header h {};
    if (std::fread(&h, sizeof h, 1, f.get()) != 1 || h.magic != FILE_MAGIC) {
        error = path + " is not a navmesh file";
        return false;
    }
    Header want = expectedHeader();
    if (h.version != want.version || h.polyRefBits != want.polyRefBits || h.cell != want.cell
        || h.cellHeight != want.cellHeight || h.tileSize != want.tileSize || h.origin[0] != want.origin[0]
        || h.origin[1] != want.origin[1] || h.origin[2] != want.origin[2] || h.agentHeight != want.agentHeight
        || h.agentRadius != want.agentRadius || h.agentClimb != want.agentClimb) {
        error = path + " was baked with other settings: bake it again with this navbake";
        return false;
    }
    file.polyRefBits = h.polyRefBits;
    file.tiles.clear();
    file.tiles.reserve(h.tileCount);
    for (uint32_t i = 0; i < h.tileCount; i++) {
        int32_t meta[3];
        if (std::fread(meta, sizeof meta, 1, f.get()) != 1 || meta[2] <= 0 || meta[2] > (64 << 20)) {
            error = path + " is cut short";
            return false;
        }
        Tile t;
        t.x = meta[0];
        t.y = meta[1];
        t.data.resize(static_cast<size_t>(meta[2]));
        if (std::fread(t.data.data(), 1, t.data.size(), f.get()) != t.data.size()) {
            error = path + " is cut short";
            return false;
        }
        file.tiles.push_back(std::move(t));
    }
    return true;
}

dtNavMeshParams worldParams(int maxTiles)
{
    dtNavMeshParams p {};
    p.orig[0] = Grid::ORIGIN_X;
    p.orig[1] = Grid::ORIGIN_Y;
    p.orig[2] = Grid::ORIGIN_Z;
    p.tileWidth = Grid::TILE_SIZE;
    p.tileHeight = Grid::TILE_SIZE;
    p.maxTiles = maxTiles < 1 ? 1 : maxTiles;
    // Polygons per tile: the most a 25.6 m tile of busy city could need.
    p.maxPolys = 1 << 14;
    return p;
}

} // namespace dnav
