#include "roads.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <queue>

namespace dnav {

namespace {

constexpr float CELL = 64.0f;   // nearest-node lookup grid

int64_t cellKey(int cx, int cy)
{
    return (static_cast<int64_t>(cx) << 32) ^ static_cast<uint32_t>(cy);
}

int cellOf(float v)
{
    return static_cast<int>(std::floor(v / CELL));
}

struct V2 {
    float x, y;
};

float len2d(float x, float y)
{
    return std::sqrt(x * x + y * y);
}

} // namespace

RoadClass RoadGraph::classOf(uint32_t from, const RoadEdge& e) const
{
    // Both ends must agree for the faster classes: a slip road off a highway is not one.
    const uint16_t a = nodes_[from].flags, b = nodes_[e.to].flags;
    const uint16_t both = a & b, either = a | b;
    if (either & ROAD_EMERGENCY_ONLY) return RoadClass::Service;
    if (both & ROAD_HIGHWAY) return RoadClass::Highway;
    if (either & ROAD_MINOR) return RoadClass::Minor;
    if ((both & ROAD_MAIN) || e.lanes >= 2 || e.lanesBack >= 2) return RoadClass::Main;
    // GTA's low traffic level marks back streets and country lanes.
    if ((both & ROAD_DETAIL) && ((a & ROAD_TRAFFIC) >> 4) == 3 && ((b & ROAD_TRAFFIC) >> 4) == 3) return RoadClass::Minor;
    return RoadClass::Street;
}

bool RoadGraph::load(const std::string& path, std::string& error)
{
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = path + ": cannot open";
        return false;
    }
    uint32_t head[4] {};
    in.read(reinterpret_cast<char*>(head), sizeof head);
    if (!in || head[0] != ROADS_MAGIC) {
        error = path + ": not a DiamondNav road graph";
        return false;
    }
    if (head[1] != ROADS_VERSION) {
        error = path + ": road graph version " + std::to_string(head[1]) + ", this build reads "
            + std::to_string(ROADS_VERSION) + " (rebuild it with tools/dnav.py roads)";
        return false;
    }
    std::vector<RoadNode> nodes(head[2]);
    std::vector<RoadEdge> edges(head[3]);
    static_assert(sizeof(RoadNode) == 20 && sizeof(RoadEdge) == 12, "road file layout");
    in.read(reinterpret_cast<char*>(nodes.data()), nodes.size() * sizeof(RoadNode));
    in.read(reinterpret_cast<char*>(edges.data()), edges.size() * sizeof(RoadEdge));
    if (!in) {
        error = path + ": truncated";
        return false;
    }
    for (const auto& n : nodes) {
        if (static_cast<size_t>(n.firstEdge) + n.edgeCount > edges.size()) {
            error = path + ": an edge index is out of range";
            return false;
        }
    }
    for (const auto& e : edges) {
        if (e.to >= nodes.size()) {
            error = path + ": an edge points past the last node";
            return false;
        }
    }
    nodes_ = std::move(nodes);
    edges_ = std::move(edges);
    cells_.clear();
    for (uint32_t i = 0; i < nodes_.size(); i++)
        cells_[cellKey(cellOf(nodes_[i].x), cellOf(nodes_[i].y))].push_back(i);
    return true;
}

bool RoadGraph::usable(const RoadNode& n, const RouteOptions& opt) const
{
    if (((n.flags & ROAD_BOATS) != 0) != opt.boats) return false;
    if ((n.flags & ROAD_EMERGENCY_ONLY) && !opt.emergency) return false;
    return true;
}

bool RoadGraph::hasExit(uint32_t node, const RouteOptions& opt) const
{
    const RoadNode& n = nodes_[node];
    for (uint32_t i = n.firstEdge; i < n.firstEdge + n.edgeCount; i++)
        if (drivable(edges_[i], opt) && usable(nodes_[edges_[i].to], opt)) return true;
    return false;
}

bool RoadGraph::hasEntry(uint32_t node, const RouteOptions& opt) const
{
    const RoadNode& n = nodes_[node];
    for (uint32_t i = n.firstEdge; i < n.firstEdge + n.edgeCount; i++)
        if ((edges_[i].lanesBack || (opt.anyWay && edges_[i].lanes)) && usable(nodes_[edges_[i].to], opt)) return true;
    return false;
}

const RoadEdge* RoadGraph::edge(uint32_t from, uint32_t to) const
{
    const RoadNode& n = nodes_[from];
    for (uint32_t i = n.firstEdge; i < n.firstEdge + n.edgeCount; i++)
        if (edges_[i].to == to) return &edges_[i];
    return nullptr;
}

float RoadGraph::laneOffset(const RoadEdge& e) const
{
    // The inner lane: next to the median on a two-way road, the leftmost lane of a
    // one-way road (whose lanes sit either side of the centre line).
    if (e.lanesBack) return e.median * 0.5f + LANE_WIDTH * 0.5f;
    return LANE_WIDTH * (0.5f - e.lanes * 0.5f);
}

int RoadGraph::nearest(float x, float y, float z, float reach, const RouteOptions& opt) const
{
    int best = -1;
    float bestD = reach * reach;
    const int r = static_cast<int>(std::ceil(reach / CELL));
    const int cx = cellOf(x), cy = cellOf(y);
    for (int dx = -r; dx <= r; dx++) {
        for (int dy = -r; dy <= r; dy++) {
            auto it = cells_.find(cellKey(cx + dx, cy + dy));
            if (it == cells_.end()) continue;
            for (uint32_t i : it->second) {
                const RoadNode& n = nodes_[i];
                if (!usable(n, opt)) continue;
                // Height counts double: a flyover is not the road under it.
                const float d = (n.x - x) * (n.x - x) + (n.y - y) * (n.y - y) + 4.0f * (n.z - z) * (n.z - z);
                if (d < bestD) {
                    bestD = d;
                    best = static_cast<int>(i);
                }
            }
        }
    }
    return best;
}

bool RoadGraph::surfaceAt(float x, float y, float z, float reach, float& out) const
{
    constexpr float LEVEL = 4.0f;   // a road further up or down than this is another level
    bool found = false;
    float bestD = reach * reach;
    const int r = static_cast<int>(std::ceil(reach / CELL));
    const int cx = cellOf(x), cy = cellOf(y);
    for (int dx = -r; dx <= r; dx++) {
        for (int dy = -r; dy <= r; dy++) {
            auto it = cells_.find(cellKey(cx + dx, cy + dy));
            if (it == cells_.end()) continue;
            for (uint32_t i : it->second) {
                const RoadNode& a = nodes_[i];
                for (uint32_t k = 0; k < a.edgeCount; k++) {
                    const RoadNode& b = nodes_[edges_[a.firstEdge + k].to];
                    const float ex = b.x - a.x, ey = b.y - a.y;
                    const float len2 = ex * ex + ey * ey;
                    const float t = len2 > 1e-6f ? std::clamp(((x - a.x) * ex + (y - a.y) * ey) / len2, 0.0f, 1.0f) : 0.0f;
                    const float h = a.z + (b.z - a.z) * t;
                    if (std::fabs(h - z) > LEVEL) continue;
                    const float px = a.x + ex * t - x, py = a.y + ey * t - y;
                    const float d = px * px + py * py;
                    if (d < bestD) {
                        bestD = d;
                        out = h;
                        found = true;
                    }
                }
            }
        }
    }
    return found;
}

bool RoadGraph::deckAt(float x, float y, float z, float& out, float wider) const
{
    constexpr float LEVEL = 4.0f;       // as surfaceAt
    constexpr float SHOULDER = 1.5f;    // paved past the outer lane
    constexpr float WIDEST = 40.0f;     // no road is wider than twice this
    bool found = false;
    float bestD = 1e30f;
    const int r = static_cast<int>(std::ceil((WIDEST + wider) / CELL));
    const int cx = cellOf(x), cy = cellOf(y);
    for (int dx = -r; dx <= r; dx++) {
        for (int dy = -r; dy <= r; dy++) {
            auto it = cells_.find(cellKey(cx + dx, cy + dy));
            if (it == cells_.end()) continue;
            for (uint32_t i : it->second) {
                const RoadNode& a = nodes_[i];
                for (uint32_t k = 0; k < a.edgeCount; k++) {
                    const RoadEdge& e = edges_[a.firstEdge + k];
                    const RoadNode& b = nodes_[e.to];
                    const float ex = b.x - a.x, ey = b.y - a.y;
                    const float len2 = ex * ex + ey * ey;
                    const float t = len2 > 1e-6f ? std::clamp(((x - a.x) * ex + (y - a.y) * ey) / len2, 0.0f, 1.0f) : 0.0f;
                    const float h = a.z + (b.z - a.z) * t;
                    if (std::fabs(h - z) > LEVEL) continue;
                    const float half = (e.lanesBack ? e.median * 0.5f + std::max(e.lanes, e.lanesBack) * LANE_WIDTH
                                                    : e.lanes * LANE_WIDTH * 0.5f) + SHOULDER + wider;
                    const float px = a.x + ex * t - x, py = a.y + ey * t - y;
                    const float d = px * px + py * py;
                    if (d <= half * half && d < bestD) {
                        bestD = d;
                        out = h;
                        found = true;
                    }
                }
            }
        }
    }
    return found;
}

RouteStatus RoadGraph::route(float fx, float fy, float fz, float tx, float ty, float tz, const RouteOptions& opt,
    std::vector<RoutePoint>& out) const
{
    out.clear();
    if (nodes_.empty()) return RouteStatus::NotLoaded;

    // Start where a car can leave from, end where one can arrive.
    auto pick = [&](float x, float y, float z, bool start) -> int {
        int best = -1;
        float bestD = SEARCH_REACH * SEARCH_REACH;
        const int r = static_cast<int>(std::ceil(SEARCH_REACH / CELL));
        const int cx = cellOf(x), cy = cellOf(y);
        for (int dx = -r; dx <= r; dx++)
            for (int dy = -r; dy <= r; dy++) {
                auto it = cells_.find(cellKey(cx + dx, cy + dy));
                if (it == cells_.end()) continue;
                for (uint32_t i : it->second) {
                    const RoadNode& n = nodes_[i];
                    if (!usable(n, opt) || !(start ? hasExit(i, opt) : hasEntry(i, opt))) continue;
                    const float d = (n.x - x) * (n.x - x) + (n.y - y) * (n.y - y) + 4.0f * (n.z - z) * (n.z - z);
                    if (d < bestD) {
                        bestD = d;
                        best = static_cast<int>(i);
                    }
                }
            }
        if (best >= 0) return best;
        // No node that near, but a long stretch of road may pass close by (a straight
        // with its nodes far apart): the nearest node at either end of a stretch that
        // runs within reach.
        const int r2 = static_cast<int>(std::ceil(EDGE_REACH / CELL));
        bestD = EDGE_REACH * EDGE_REACH;
        for (int dx = -r2; dx <= r2; dx++)
            for (int dy = -r2; dy <= r2; dy++) {
                auto it = cells_.find(cellKey(cx + dx, cy + dy));
                if (it == cells_.end()) continue;
                for (uint32_t i : it->second) {
                    const RoadNode& a = nodes_[i];
                    if (!usable(a, opt) || !(start ? hasExit(i, opt) : hasEntry(i, opt))) continue;
                    const float d = (a.x - x) * (a.x - x) + (a.y - y) * (a.y - y) + 4.0f * (a.z - z) * (a.z - z);
                    if (d >= bestD) continue;
                    for (uint32_t k = a.firstEdge; k < a.firstEdge + a.edgeCount; k++) {
                        // A stretch that may be driven from this node on, with the point
                        // along it (not past its far end: a one-way street's end is not on it).
                        if (!drivable(edges_[k], opt) || !usable(nodes_[edges_[k].to], opt)) continue;
                        const RoadNode& b = nodes_[edges_[k].to];
                        const float ex = b.x - a.x, ey = b.y - a.y, len2 = ex * ex + ey * ey;
                        if (len2 < 1e-6f) continue;
                        const float t = ((x - a.x) * ex + (y - a.y) * ey) / len2;
                        if (t < 0.0f || t > 0.99f) continue;
                        const float px = a.x + ex * t - x, py = a.y + ey * t - y, pz = a.z + (b.z - a.z) * t - z;
                        if (px * px + py * py + 4.0f * pz * pz < SEARCH_REACH * SEARCH_REACH) {
                            bestD = d;
                            best = static_cast<int>(i);
                            break;
                        }
                    }
                }
            }
        return best;
    };
    // With a heading, every node near the start is a way in, each costing the way to
    // it plus a turn round if it is behind the car: a car going one way takes the
    // road ahead of it, not the nearest node at its back.
    std::vector<std::pair<uint32_t, float>> starts;
    if (opt.hasHeading) {
        const float h = opt.heading * 0.01745329f;
        const float fwdX = -std::sin(h), fwdY = std::cos(h);
        const int r = static_cast<int>(std::ceil(SEARCH_REACH / CELL));
        const int cx = cellOf(fx), cy = cellOf(fy);
        for (int dx = -r; dx <= r; dx++)
            for (int dy = -r; dy <= r; dy++) {
                auto it = cells_.find(cellKey(cx + dx, cy + dy));
                if (it == cells_.end()) continue;
                for (uint32_t i : it->second) {
                    const RoadNode& n = nodes_[i];
                    if (!usable(n, opt) || !hasExit(i, opt) || std::fabs(n.z - fz) > 6.0f) continue;
                    const float ox = n.x - fx, oy = n.y - fy, d = len2d(ox, oy);
                    if (d > SEARCH_REACH) continue;
                    const bool behind = d > 6.0f && ox * fwdX + oy * fwdY < 0.0f;
                    starts.push_back({ i, d + (behind ? TURN_ROUND : 0.0f) });
                }
            }
    }
    if (starts.empty()) {
        const int s = pick(fx, fy, fz, true);
        if (s < 0) return RouteStatus::NoStart;
        starts.push_back({ static_cast<uint32_t>(s), 0.0f });
    }
    const int g = pick(tx, ty, tz, false);
    if (g < 0) return RouteStatus::NoEnd;

    // A* on distance.
    const size_t n = nodes_.size();
    std::vector<float> cost(n, std::numeric_limits<float>::infinity());
    std::vector<int> prev(n, -1);
    std::vector<bool> closed(n, false);
    using Item = std::pair<float, uint32_t>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    const RoadNode& goal = nodes_[g];
    auto h = [&](uint32_t i) {
        const RoadNode& a = nodes_[i];
        return std::sqrt((a.x - goal.x) * (a.x - goal.x) + (a.y - goal.y) * (a.y - goal.y) + (a.z - goal.z) * (a.z - goal.z));
    };
    for (const auto& [s, c] : starts) {
        if (c >= cost[s]) continue;
        cost[s] = c;
        open.push({ c + h(s), s });
    }
    while (!open.empty()) {
        const uint32_t cur = open.top().second;
        open.pop();
        if (closed[cur]) continue;
        closed[cur] = true;
        if (cur == static_cast<uint32_t>(g)) break;
        const RoadNode& a = nodes_[cur];
        for (uint32_t i = a.firstEdge; i < a.firstEdge + a.edgeCount; i++) {
            const RoadEdge& e = edges_[i];
            if (!drivable(e, opt) || closed[e.to] || !usable(nodes_[e.to], opt)) continue;
            const RoadNode& b = nodes_[e.to];
            const float c = cost[cur] + (e.lanes ? 1.0f : AGAINST_COST)
                * std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
            if (c < cost[e.to]) {
                cost[e.to] = c;
                prev[e.to] = static_cast<int>(cur);
                open.push({ c + h(e.to), e.to });
            }
        }
    }
    if (!std::isfinite(cost[g])) return RouteStatus::NoPath;

    std::vector<uint32_t> chain;
    for (int i = g; i >= 0; i = prev[i]) chain.push_back(static_cast<uint32_t>(i));
    std::reverse(chain.begin(), chain.end());

    // The end node can fall short of the destination (the destination is between it
    // and the next node on): go on to that next node, and the last leg is cut level
    // with the destination below.
    {
        const RoadNode& e = nodes_[g];
        int next = -1;
        float bestOff = SEARCH_REACH;
        for (uint32_t i = e.firstEdge; i < e.firstEdge + e.edgeCount; i++) {
            const RoadEdge& ed = edges_[i];
            if (!drivable(ed, opt) || !usable(nodes_[ed.to], opt)) continue;
            if (chain.size() >= 2 && ed.to == chain[chain.size() - 2]) continue;
            const RoadNode& b = nodes_[ed.to];
            const float dx = b.x - e.x, dy = b.y - e.y, len = len2d(dx, dy);
            if (len < 0.01f) continue;
            const float t = ((tx - e.x) * dx + (ty - e.y) * dy) / len;
            const float off = std::fabs((tx - e.x) * dy - (ty - e.y) * dx) / len;
            if (t > 1.0f && t < len && off < bestOff) {
                bestOff = off;
                next = static_cast<int>(ed.to);
            }
        }
        if (next >= 0) chain.push_back(static_cast<uint32_t>(next));
    }

    if (chain.size() == 1) {
        const RoadNode& a = nodes_[chain[0]];
        out.push_back({ a.x, a.y, a.z, 0.5f, RoadClass::Street });
        return RouteStatus::Ok;
    }

    // Each leg in its lane: shifted to the right of the direction of travel.
    struct Leg {
        float ax, ay, az, bx, by, bz, dx, dy, len;
        RoadClass cls;
        bool junction;          // its end node is a junction
    };
    std::vector<Leg> legs;
    for (size_t i = 0; i + 1 < chain.size(); i++) {
        const RoadNode& a = nodes_[chain[i]];
        const RoadNode& b = nodes_[chain[i + 1]];
        const RoadEdge* e = edge(chain[i], chain[i + 1]);
        float dx = b.x - a.x, dy = b.y - a.y;
        const float len = len2d(dx, dy);
        if (len < 0.01f) continue;
        dx /= len;
        dy /= len;
        const float off = e ? laneOffset(*e) : 0.0f;
        const float rx = dy * off, ry = -dx * off;      // the right-hand side
        legs.push_back({ a.x + rx, a.y + ry, a.z, b.x + rx, b.y + ry, b.z, dx, dy, len,
            e ? classOf(chain[i], *e) : RoadClass::Street, b.edgeCount >= 3 });
    }
    if (legs.empty()) {
        const RoadNode& a = nodes_[chain.back()];
        out.push_back({ a.x, a.y, a.z, 0.5f, RoadClass::Street });
        return RouteStatus::Ok;
    }

    // Join the road where the car is and leave it where it is going: drop the legs
    // already behind the start (or past the end), and cut the first and last legs
    // at the start's and end's place along them.
    auto along = [](const Leg& l, float px, float py) { return (px - l.ax) * l.dx + (py - l.ay) * l.dy; };
    auto cutStart = [](Leg& l, float t) {
        const float f = t / l.len;
        l.ax += (l.bx - l.ax) * f;
        l.ay += (l.by - l.ay) * f;
        l.az += (l.bz - l.az) * f;
        l.len -= t;
    };
    while (legs.size() > 1 && along(legs.front(), fx, fy) >= legs.front().len) legs.erase(legs.begin());
    while (legs.size() > 1 && along(legs.back(), tx, ty) <= 0.0f) legs.pop_back();
    if (legs.size() == 1) {
        Leg& l = legs.front();
        const float t0 = std::clamp(along(l, fx, fy), 0.0f, l.len), t1 = std::clamp(along(l, tx, ty), 0.0f, l.len);
        if (t1 - t0 > 1.0f) {
            Leg cut = l;
            cut.bx = l.ax + l.dx * t1;
            cut.by = l.ay + l.dy * t1;
            cut.bz = l.az + (l.bz - l.az) * (t1 / l.len);
            cut.len = t1;
            cutStart(cut, t0);
            l = cut;
        }
    } else {
        Leg& first = legs.front();
        const float t0 = std::clamp(along(first, fx, fy), 0.0f, first.len - 1.0f);
        if (t0 > 0.0f) cutStart(first, t0);
        Leg& last = legs.back();
        const float t1 = std::clamp(along(last, tx, ty), 1.0f, last.len);
        const float f = t1 / last.len;
        last.bx = last.ax + (last.bx - last.ax) * f;
        last.by = last.ay + (last.by - last.ay) * f;
        last.bz = last.az + (last.bz - last.az) * f;
        last.len = t1;
    }

    out.push_back({ legs[0].ax, legs[0].ay, legs[0].az, 1.0f, legs[0].cls });
    for (size_t i = 0; i + 1 < legs.size(); i++) {
        const Leg& p = legs[i];
        const Leg& q = legs[i + 1];
        const float turn = std::acos(std::clamp(p.dx * q.dx + p.dy * q.dy, -1.0f, 1.0f)) * 57.29578f;
        // Round a corner at the slower road's class.
        const RoadClass cls = std::max(p.cls, q.cls);
        if (turn < 4.0f) {
            out.push_back({ p.bx, p.by, p.bz, 1.0f, p.cls, p.junction });
            continue;
        }
        // Round the corner: a quadratic curve from R before the node on this leg to
        // R after it on the next, through where the two lane lines cross.
        const float r = std::min({ MAX_CORNER, p.len * 0.45f, q.len * 0.45f });
        const float t0 = std::max(0.0f, 1.0f - r / p.len);
        const V2 p0 { p.ax + (p.bx - p.ax) * t0, p.ay + (p.by - p.ay) * t0 };
        const float z0 = p.az + (p.bz - p.az) * t0;
        const float t2 = std::min(1.0f, r / q.len);
        const V2 p2 { q.ax + (q.bx - q.ax) * t2, q.ay + (q.by - q.ay) * t2 };
        const float z2 = q.az + (q.bz - q.az) * t2;
        V2 p1 { (p.bx + q.ax) * 0.5f, (p.by + q.ay) * 0.5f };
        const float den = p.dx * q.dy - p.dy * q.dx;
        if (std::fabs(den) > 1e-3f) {
            const float u = ((q.ax - p.ax) * q.dy - (q.ay - p.ay) * q.dx) / den;
            const V2 hit { p.ax + p.dx * u, p.ay + p.dy * u };
            if (len2d(hit.x - p.bx, hit.y - p.by) < 2.0f * r + LANE_WIDTH) p1 = hit;
        }
        const float speed = std::clamp(1.0f - turn / 110.0f, 0.3f, 1.0f);
        constexpr int STEPS = 4;
        for (int k = 0; k <= STEPS; k++) {
            const float t = static_cast<float>(k) / STEPS, u = 1.0f - t;
            out.push_back({ u * u * p0.x + 2 * u * t * p1.x + t * t * p2.x, u * u * p0.y + 2 * u * t * p1.y + t * t * p2.y,
                u * z0 + t * z2, speed, k == 0 ? p.cls : k == STEPS ? q.cls : cls, p.junction && k == STEPS / 2 });
        }
    }
    const Leg& last = legs.back();
    out.push_back({ last.bx, last.by, last.bz, 0.5f, last.cls });
    // Ease off over the last stretch.
    if (out.size() >= 2) out[out.size() - 2].speed = std::min(out[out.size() - 2].speed, 0.7f);
    return RouteStatus::Ok;
}

} // namespace dnav
