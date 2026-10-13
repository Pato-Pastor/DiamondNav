// The .nav file and the settings the baker and the component share.
//
// One world-wide tile grid per kind of mesh: every map bakes the tiles its geometry touches, on the
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
constexpr uint32_t FILE_VERSION = 2;            // 2: the header says which kind of mesh

// Who the navmesh is for. A walking mesh and a car mesh live side by side, each in
// its own dtNavMesh.
enum class MeshKind : uint32_t {
    Ped = 0,
    Car = 1,
};
constexpr int MESH_KINDS = 2;

// The bake settings of each kind. The grid's origin is shared: the map's south-west
// corner (in Recast's axes), so tile (0, 0) starts at GTA x -3000, y +3000.
struct Profile {
    float cell, cellHeight;     // metres
    int tileCells;
    float height, radius, climb, maxSlope;
    float tileSize() const { return cell * tileCells; }
};

// A ped in San Andreas: 1.8 m tall, about 0.35 m round, steps up 0.45 m; escalator
// ramps are about 30 degrees. 0.2 m cells, 25.6 m tiles.
constexpr Profile PED_PROFILE { 0.2f, 0.1f, 128, 1.8f, 0.35f, 0.45f, 50.0f };
// A car: a Sultan is 4.5 by 1.8 m, so a 1.4 m radius keeps it off walls when it
// turns; kerbs and low walls up to 0.6 m are crossed, slopes up to 35 degrees. 0.5 m
// cells, 64 m tiles: a car needs less detail than a ped.
constexpr Profile CAR_PROFILE { 0.5f, 0.2f, 128, 2.0f, 1.4f, 0.6f, 35.0f };

inline const Profile& profile(MeshKind kind)
{
    return kind == MeshKind::Car ? CAR_PROFILE : PED_PROFILE;
}

inline const char* kindName(MeshKind kind)
{
    return kind == MeshKind::Car ? "car" : "walking";
}

struct Grid {
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
    MeshKind kind = MeshKind::Ped;
    uint32_t polyRefBits = 0;                   // sizeof(dtPolyRef) * 8 when baked
    std::vector<Tile> tiles;
};

bool writeNavFile(const std::string& path, const NavFile& file, std::string& error);
bool readNavFile(const std::string& path, NavFile& file, std::string& error);

// dtNavMesh params for a kind's world grid, room for maxTiles tiles.
dtNavMeshParams worldParams(MeshKind kind, int maxTiles);

} // namespace dnav
