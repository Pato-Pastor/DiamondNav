// The .nav file and the settings the baker and the component share.
//
// One world-wide tile grid: every map bakes the tiles its geometry touches, on the
// same grid, so the component can put the tiles of every navmesh/*.nav file into
// one dtNavMesh. Two files must not bake the same tile (the loader refuses the
// second). Coordinates: GTA is x, y, z with z up; Recast is y up. The swap below is
// a rotation (not a mirror), so triangle winding means the same in both.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <DetourNavMesh.h>

namespace dnav {

constexpr uint32_t FILE_MAGIC = 0x564E4E44;     // "DNNV"
constexpr uint32_t FILE_VERSION = 1;

// A ped in San Andreas: 1.8 m tall, about 0.35 m round, steps up 0.45 m.
struct Agent {
    static constexpr float HEIGHT = 1.8f;
    static constexpr float RADIUS = 0.35f;
    static constexpr float CLIMB = 0.45f;
    static constexpr float MAX_SLOPE = 50.0f;   // degrees: escalator ramps are ~30
};

// The grid. 0.2 m cells, 25.6 m tiles; the origin is the map's south-west corner
// (in Recast's axes), so tile (0, 0) starts at GTA x -3000, y +3000.
struct Grid {
    static constexpr float CELL = 0.2f;
    static constexpr float CELL_HEIGHT = 0.1f;
    static constexpr int TILE_CELLS = 128;
    static constexpr float TILE_SIZE = CELL * TILE_CELLS;
    static constexpr float ORIGIN_X = -3000.0f;
    static constexpr float ORIGIN_Y = -200.0f;  // Recast y = GTA z
    static constexpr float ORIGIN_Z = -3000.0f; // Recast z = -GTA y
};

inline void toRecast(const float* gta, float* rc)
{
    rc[0] = gta[0];
    rc[1] = gta[2];
    rc[2] = -gta[1];
}

inline void toGta(const float* rc, float* gta)
{
    gta[0] = rc[0];
    gta[1] = -rc[2];
    gta[2] = rc[1];
}

struct Tile {
    int x = 0, y = 0;
    std::vector<unsigned char> data;            // dtCreateNavMeshData's output
};

struct NavFile {
    uint32_t polyRefBits = 0;                   // sizeof(dtPolyRef) * 8 when baked
    std::vector<Tile> tiles;
};

bool writeNavFile(const std::string& path, const NavFile& file, std::string& error);
bool readNavFile(const std::string& path, NavFile& file, std::string& error);

// dtNavMesh params for the world grid, room for maxTiles tiles.
dtNavMeshParams worldParams(int maxTiles);

} // namespace dnav
