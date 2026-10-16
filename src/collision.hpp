// The world's collision, for rays: what a car would hit.
//
// The meshes say where a ped or a car may go; this says what is solid. It is the
// same triangles navbake bakes from (a .tris file from tools/dnav.py world, GTA's map
// with the custom maps in it), loaded as they are and sorted into a grid of columns
// from above, so a ray only tests the triangles in the columns it crosses. Read-only
// once loaded.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace dnav {

struct Vec3 {
    float x, y, z;
};

struct RayHit {
    float t = 1.0f;             // how far along the ray (0 at its start, 1 at its end)
    Vec3 at {};                 // where
    Vec3 normal {};             // the face's normal, pointing back at the ray's start
};

class Collision {
public:
    // A .tris file ("DNTR", u32 count, count x 9 float32: GTA axes).
    bool load(const std::string& path, std::string& error);
    // The same from triangles already in memory (tests).
    void build(const std::vector<float>& tris);

    bool loaded() const { return !tris_.empty(); }
    size_t triangleCount() const { return tris_.size() / 3; }

    // The first face between a and b (either side of it), or false.
    bool ray(const Vec3& a, const Vec3& b, RayHit& hit) const;
    // The first face between a and b steeper than a car drives up (a wall, a kerb's
    // face, a railing, a post), or false. Ground and ceilings are passed through.
    bool wall(const Vec3& a, const Vec3& b, RayHit& hit) const;
    // The ground under (x, y): the first face, from z + up down to z - down, that a
    // car could stand on.
    bool ground(float x, float y, float z, float up, float down, float& out) const;

    // A face whose normal points up at least this much is ground a car drives on, up
    // to about 63 degrees (a storm drain's banks, a steep ramp); steeper is a wall.
    static constexpr float GROUND_NZ = 0.45f;

private:
    bool cast(const Vec3& a, const Vec3& b, RayHit& hit, int want) const;

    std::vector<Vec3> verts_;
    std::vector<uint32_t> tris_;        // three vertex indices each
    // The grid: columns of CELL metres from (x0, y0), w x h of them; the faces whose
    // box crosses column i are faces_[start_[i] .. start_[i + 1]).
    float x0_ = 0.0f, y0_ = 0.0f, cell_ = 8.0f;
    int w_ = 0, h_ = 0;
    std::vector<uint32_t> start_;
    std::vector<uint32_t> faces_;
};

} // namespace dnav
