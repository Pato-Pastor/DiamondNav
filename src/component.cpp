// DiamondNav: the open.mp component (README.md).
//
// Loads navmesh/*.nav (where peds walk, and where cars fit off the road) and
// navmesh/roads.dnr (where cars drive) at start, searches paths on worker threads,
// and either hands the points to the script that asked (Nav_RequestPath /
// Nav_RequestRoute -> OnNavPathResult) or moves an NPC along them itself
// (Nav_WalkTo / Nav_DriveTo / Nav_Chase -> OnNavMoveEnd). Every callback runs on
// the game thread.

#include <sdk.hpp>

#include <Server/Components/NPCs/npcs.hpp>
#include <Server/Components/Pawn/pawn.hpp>
#include <Server/Components/Pawn/Impl/pawn_natives.hpp>
#include <Server/Components/Pawn/Impl/pawn_impl.hpp>
#include <Server/Components/Vehicles/vehicles.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <thread>
#include <unordered_map>

#include "driving.hpp"
#include "pathfinder.hpp"

#ifndef DNAV_VERSION_MAJOR
    #define DNAV_VERSION_MAJOR 1
    #define DNAV_VERSION_MINOR 0
    #define DNAV_VERSION_PATCH 0
#endif

namespace {

constexpr char NAV_FOLDER[] = "navmesh";
constexpr size_t MAX_RESULTS_PER_TICK = 256;    // the rest wait for the next tick
constexpr float FOOT_Z = 1.0f;                  // a ped's position stands this far over the ground
constexpr float WALK_STOP = 0.4f;               // how near a walking point counts as reached
constexpr float DRIVE_STOP = 1.5f;
constexpr float DRIVE_MIN_SPEED = 0.25f;        // NPC_MOVE_TYPE_DRIVE speed floor in corners

// Driving: the component steers the car itself, like a driver, instead of sliding it
// from point to point. Each time it looks a little way down the path, turns toward
// that point no faster than the car could, speeds up or brakes as the car and the
// road allow (driving.hpp: handling, speed limits, styles, the speed plan), and hands
// NPC_Move a target just ahead, on the ground.
// NPC_Move restarts the NPC's clock, so a move sent between two NPC updates loses the
// time since the last one: drive() runs right after each NPC update instead (the
// NPC's position changed), or after DRIVE_IDLE when it stands still.
constexpr auto DRIVE_IDLE = std::chrono::milliseconds(120);
constexpr float SPEED_UNIT = 10.0f;     // m/s per NPC_MOVE_TYPE_DRIVE speed 1.0
constexpr float TURN_MIN_SPEED = 3.0f;  // never slower than this while it still has to turn
constexpr float LOOK_MIN = 4.0f;        // how far down the path it aims: 4 m at a crawl...
constexpr float LOOK_TIME = 0.6f;       // ...plus 0.6 s of travel
constexpr float LOOK_MAX = 24.0f;
constexpr float PASSED_CORNER = 3.0f;   // still this near the point it passed, it keeps that point's speed
constexpr float SCAN_EXTRA = 20.0f;     // looks for slower points this far past its braking distance
constexpr float AHEAD_TIME = 0.5f;      // NPC_Move's target: this far ahead of the car
constexpr float AHEAD_MIN = 3.0f;
constexpr float ARRIVE = 2.5f;          // a drive ends this near its last point
constexpr float GROUND_REACH = 3.0f;    // how far up or down the car mesh's ground is looked for
constexpr float REVERSE_SPEED = 4.0f;   // m/s backing up (or less: a third of the car's top speed)
constexpr float REVERSE_BEHIND = 100.0f; // backs up when the point it aims at is further round than this...
constexpr float REVERSE_NEAR = 2.5f;    // ...and nearer than this many turning radii (a U-turn is two across)
constexpr float REVERSE_INSIDE = 60.0f; // or this far round and inside its turning circle
constexpr float REVERSE_DONE = 40.0f;   // drives forward again once it points this near the aim
constexpr float REVERSE_MAX = 12.0f;    // backs up at most this far at a time
constexpr float REVERSE_CHECK = 4.0f;   // the car mesh must be clear this far behind it
constexpr float CIRCLING_TURN = 60.0f;  // turning this hard...
constexpr float CIRCLING_SECS = 5.0f;   // ...this long means it is going round in circles: back up

// Nav_Chase.
constexpr auto CHASE_THINK = std::chrono::milliseconds(250);   // how often a chaser looks at its target
constexpr auto CHASE_REPLAN_AFTER = std::chrono::seconds(6);   // a plan this old is redone anyway
constexpr auto CHASE_NO_OFFROAD = std::chrono::seconds(4);     // after the car mesh found no way
constexpr float CHASE_CLOSE = 6.0f;         // this near, the car stops beside its target
constexpr float CHASE_DIRECT = 60.0f;       // within this, a clear straight line is driven straight
constexpr float CHASE_BLIND = 25.0f;        // ...or, with no car mesh to check the line, within this
constexpr float CHASE_DIRECT_DZ = 4.0f;     // no straight line up or down a level
constexpr float CHASE_REPLAN = 10.0f;       // re-plan once the target is this far from the plan's end
constexpr float CHASE_REPLAN_SHARE = 0.2f;  // ...or a fifth of the way to it, if more
constexpr float CHASE_ON_ROAD = 15.0f;      // a target this near a road node is on the road
constexpr float CHASE_LEAD = 25.0f;         // aims where the target will be in 0.5 s (velocity is ~50 m/s per unit)
constexpr float CHASE_OFFROAD_RANGE = 200.0f; // with a car mesh, nearer than this the chase goes straight across it
constexpr float CHASE_GROUND_STEP = 4.0f;   // a straight run samples the ground this often

// OnNavMoveEnd's reasons (DiamondNav.inc).
enum MoveEnd : int {
    END_ARRIVED = 0,
    END_NO_PATH = 1,        // no way there (or no navmesh / road graph)
    END_STOPPED = 2,        // Nav_Stop, or a new Nav_WalkTo / Nav_DriveTo
    END_INTERRUPTED = 3,    // something else moved it, it left the car, died or was destroyed
    END_LOST = 4,           // Nav_Chase: the target left, or is in another world
};

// Nav_DriveTo's flags beyond the road ones (DiamondNav.inc).
constexpr int DRIVE_OFFROAD = 4;

bool driving(dnav::PathKind k)
{
    return k != dnav::PathKind::Walk;
}

struct Mover {
    int npc = -1;
    void* owner = nullptr;              // the script told when it ends
    int request = 0;                    // the search it waits on (0: moving)
    dnav::PathKind kind = dnav::PathKind::Walk;
    NPCMoveType moveType = NPCMoveType_Walk;
    float speed = NPC_MOVE_SPEED_AUTO;
    float carZ = 1.0f;                  // Drive: the car's centre over the road
    std::vector<Vector3> points;
    std::vector<float> speeds;
    // Driving: each point's road class, whether it is in a junction, the fastest the
    // car may go there (m/s, the speed plan) and the distance along the path to it.
    std::vector<dnav::RoadClass> classes;
    std::vector<uint8_t> junctions;
    std::vector<float> limits, cum;
    dnav::Handling car;                 // the car's handling, read when the path is set
    dnav::DriveStyle style = dnav::DriveStyle::Normal;
    float wantSpeed = 0.0f;             // the speed it is going for (Nav_GetDriveInfo)
    size_t next = 0;
    Vector3 target {};

    // Driving (drive()): the path segment the car is on, its speed and heading.
    size_t seg = 0;
    float v = 0.0f;                     // m/s
    float heading = 0.0f;               // degrees, GTA's
    bool rolling = false;               // speed and heading are the car's own
    bool ground = false;                // read the height off the car mesh as it goes
    TimePoint lastDrive {};
    Vector3 lastPos {};                 // the NPC's position at the last drive()
    bool reversing = false;             // backing up (a three-point turn, or out of a corner)
    float backed = 0.0f;                // metres backed up this time
    float circling = 0.0f;              // seconds spent turning hard
    int wrongWay = 0;                   // drive()s in a row that a back-up went forward

    // Nav_Chase: the car follows a player until Nav_Stop. It re-plans as the target
    // moves and keeps driving the old plan until the new one comes back.
    bool chase = false;
    int quarry = -1;                    // the player chased
    int plan = 0;                       // a re-plan being searched
    Vector3 planGoal {};                // where the path being driven ends up
    Vector3 askedGoal {};               // where the re-plan being searched ends up
    TimePoint nextThink {}, planned {}, noOffRoadUntil {};
    bool direct = false;                // driving straight at the target
    bool idle = false;                  // stopped beside it
};

class DiamondNav final : public IComponent, public PawnEventHandler, public CoreEventHandler, public NPCEventHandler {
public:
    PROVIDE_UID(0xc7e6abc6cb842c2c);

    static DiamondNav* instance;

    dnav::Pathfinder finder;
    dnav::Driving drivingModel;         // handling, speed limits, the speed plan
    std::unordered_map<int, dnav::DriveStyle> styles;   // Nav_SetDriveStyle, by NPC
    bool canReverse_ = true;            // the NPC component backs cars up (our patched build)
    // The result being handed to a script right now (Nav_GetPathPoint reads it).
    const dnav::PathResult* current = nullptr;

    ~DiamondNav()
    {
        if (pawn_) pawn_->getEventDispatcher().removeEventHandler(this);
        if (npcs_) npcs_->getEventDispatcher().removeEventHandler(this);
        if (core_) core_->getEventDispatcher().removeEventHandler(this);
        finder.stop();
        instance = nullptr;
    }

    StringView componentName() const override { return "DiamondNav"; }
    SemanticVersion componentVersion() const override
    {
        return SemanticVersion(DNAV_VERSION_MAJOR, DNAV_VERSION_MINOR, DNAV_VERSION_PATCH, 0);
    }

    void onLoad(ICore* c) override
    {
        core_ = c;
        instance = this;
        setAmxLookups(core_);
        std::vector<std::string> log;
        finder.load(NAV_FOLDER, log);
        for (const auto& line : log) {
            if (line.rfind("ERROR: ", 0) == 0) core_->logLn(LogLevel::Error, "[nav] %s", line.c_str() + 7);
            else core_->printLn("[nav] %s", line.c_str());
        }
        loadVehicles();
        const unsigned hw = std::thread::hardware_concurrency();
        const int threads = static_cast<int>(std::clamp(hw > 1 ? hw - 1 : 1u, 1u, 4u));
        if (finder.loaded()) finder.start(threads);
        core_->printLn("[nav] DiamondNav %d.%d.%d: %d walking tiles, %d car tiles, %d road nodes, %d search threads",
            DNAV_VERSION_MAJOR, DNAV_VERSION_MINOR, DNAV_VERSION_PATCH, finder.tileCount(dnav::MeshKind::Ped),
            finder.tileCount(dnav::MeshKind::Car), static_cast<int>(finder.roads().nodeCount()),
            finder.loaded() ? threads : 0);
        core_->getEventDispatcher().addEventHandler(this);
    }

    void loadVehicles()
    {
        const std::string path = std::string(NAV_FOLDER) + "/vehicles.dnv";
        if (!std::ifstream(path)) {
            core_->printLn("[nav] no vehicles.dnv: every car drives alike (tools/dnav.py vehicles makes it from GTA)");
            return;
        }
        std::string error;
        if (drivingModel.loadVehicles(path, error)) {
            core_->printLn("[nav] loaded vehicles.dnv: %d vehicle models", static_cast<int>(drivingModel.vehicleCount()));
        } else {
            core_->logLn(LogLevel::Error, "[nav] %s", error.c_str());
        }
    }

    // The style npc drives in: its own (Nav_SetDriveStyle), else pursuit for a chase
    // and normal for anything else.
    dnav::DriveStyle styleOf(int npcid, bool chase) const
    {
        auto it = styles.find(npcid);
        if (it != styles.end()) return it->second;
        return chase ? dnav::DriveStyle::Pursuit : dnav::DriveStyle::Normal;
    }

    void onInit(IComponentList* components) override
    {
        pawn_ = components->queryComponent<IPawnComponent>();
        if (pawn_) {
            setAmxFunctions(pawn_->getAmxFunctions());
            setAmxLookups(components);
            pawn_->getEventDispatcher().addEventHandler(this);
        }
        npcs_ = components->queryComponent<INPCComponent>();
        if (npcs_) npcs_->getEventDispatcher().addEventHandler(this);
        else core_->printLn("[nav] no NPC component: Nav_WalkTo / Nav_DriveTo are off, path searches still work");
    }

    void onAmxLoad(IPawnScript& script) override { pawn_natives::AmxLoad(script.GetAMX()); }

    void onAmxUnload(IPawnScript& script) override
    {
        void* amx = script.GetAMX();
        finder.forgetOwner(amx);
        for (auto& r : results_)
            if (r.owner == amx) r.owner = nullptr;
        for (auto& [id, m] : movers_)
            if (m.owner == amx) m.owner = nullptr;
    }

    void onTick(Microseconds, TimePoint now) override
    {
        // Chasers look at their targets (ids first: thinking can end a chase).
        thinking_.clear();
        for (const auto& [id, m] : movers_)
            if (m.chase && m.nextThink <= now) thinking_.push_back(id);
        for (int id : thinking_) {
            auto it = movers_.find(id);
            if (it != movers_.end()) think(id, it->second, now);
        }
        // Drivers steer (ids first: arriving ends a drive).
        thinking_.clear();
        for (const auto& [id, m] : movers_) {
            if (!driving(m.kind) || m.request || (!m.chase && m.points.empty())) continue;
            INPC* npc = npcs_ ? npcs_->get(id) : nullptr;
            if (!npc || npc->getPosition() != m.lastPos || now - m.lastDrive >= DRIVE_IDLE) thinking_.push_back(id);
        }
        for (int id : thinking_) {
            auto it = movers_.find(id);
            if (it != movers_.end()) drive(id, it->second, now);
        }

        fresh_.clear();
        finder.takeResults(fresh_);
        for (auto& r : fresh_) results_.push_back(std::move(r));
        // Oldest first, at most MAX_RESULTS_PER_TICK; the rest wait for the next tick.
        size_t handed = 0;
        for (; handed < results_.size() && handed < MAX_RESULTS_PER_TICK; handed++) {
            const auto& r = results_[handed];
            auto mv = byRequest_.find(r.id);
            if (mv != byRequest_.end()) {
                const int npc = mv->second;
                byRequest_.erase(mv);
                auto m = movers_.find(npc);
                if (m != movers_.end() && m->second.chase) planDone(npc, m->second, r, now);
                else startMoving(npc, r);
                continue;
            }
            if (!r.owner || !pawn_) continue;           // its script has gone
            IPawnScript* script = pawn_->getScript(static_cast<AMX*>(r.owner));
            if (!script) continue;
            current = &r;
            script->Call("OnNavPathResult", DefaultReturnValue_False, r.id, static_cast<int>(r.status),
                static_cast<int>(r.points.size()));
            current = nullptr;
        }
        results_.erase(results_.begin(), results_.begin() + handed);
    }

    // ── NPC movement ─────────────────────────────────────────────────────────

    // Starts a search for npc's walk or drive; true if it was queued.
    bool goTo(AMX* amx, int npcid, dnav::PathKind kind, const Vector3& to, NPCMoveType moveType, float speed,
        float carZ, const dnav::RouteOptions& road)
    {
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        if (!npc) return false;
        Vector3 from = npc->getPosition();
        if (driving(kind)) {
            IVehicle* car = npc->getVehicle();
            if (!car || npc->getVehicleSeat() != 0) return false;   // only a driver drives
            from = car->getPosition() - Vector3(0.0f, 0.0f, carZ);
        } else {
            from.z -= FOOT_Z;
        }
        end(npcid, END_STOPPED, true);
        const Vector3 goal = driving(kind) ? to - Vector3(0.0f, 0.0f, carZ) : to - Vector3(0.0f, 0.0f, FOOT_Z);
        const int id = finder.request(amx, dnav::Point { from.x, from.y, from.z }, dnav::Point { goal.x, goal.y, goal.z },
            kind, road);
        if (!id) return false;
        Mover m;
        m.npc = npcid;
        m.owner = amx;
        m.request = id;
        m.kind = kind;
        m.moveType = driving(kind) ? NPCMoveType_Drive : moveType;
        m.speed = speed;
        m.carZ = carZ;
        m.style = styleOf(npcid, false);
        movers_[npcid] = std::move(m);
        byRequest_[id] = npcid;
        return true;
    }

    // Ends npc's walk or drive, if it has one, and tells its script why.
    void end(int npcid, MoveEnd why, bool stopNpc)
    {
        auto it = movers_.find(npcid);
        if (it == movers_.end()) return;
        Mover m = std::move(it->second);
        movers_.erase(it);
        if (m.request) byRequest_.erase(m.request);
        if (m.plan) byRequest_.erase(m.plan);
        if (stopNpc && !m.request && npcs_) {
            if (INPC* npc = npcs_->get(npcid)) npc->stopMove();
        }
        if (m.owner && pawn_) {
            if (IPawnScript* script = pawn_->getScript(static_cast<AMX*>(m.owner)))
                script->Call("OnNavMoveEnd", DefaultReturnValue_False, npcid, static_cast<int>(why));
        }
    }

    bool isMoving(int npcid) const { return movers_.count(npcid) != 0; }

    // Nav_SetDriveStyle: from now on, and for the drive it is on. -1 goes back to the
    // default (normal, or pursuit in a chase).
    void setStyle(int npcid, int style)
    {
        if (style < 0) styles.erase(npcid);
        else styles[npcid] = static_cast<dnav::DriveStyle>(style);
        auto it = movers_.find(npcid);
        if (it == movers_.end() || !driving(it->second.kind)) return;
        Mover& m = it->second;
        m.style = styleOf(npcid, m.chase);
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        if (npc && npc->getVehicle() && !m.points.empty()) planSpeeds(m, *npc->getVehicle());
    }

    // A rescan of every driver's plan: speed limits or a model's handling changed.
    void replanAll()
    {
        for (auto& [id, m] : movers_) {
            if (!driving(m.kind) || m.points.empty()) continue;
            INPC* npc = npcs_ ? npcs_->get(id) : nullptr;
            if (npc && npc->getVehicle()) planSpeeds(m, *npc->getVehicle());
        }
    }

    // Nav_GetDriveInfo: how a driver is doing right now.
    bool driveInfo(int npcid, float& speed, float& want, dnav::RoadClass& cls, float& limit, bool& reversing) const
    {
        auto it = movers_.find(npcid);
        if (it == movers_.end() || !driving(it->second.kind)) return false;
        const Mover& m = it->second;
        speed = m.v;
        want = m.wantSpeed;
        cls = m.seg + 1 < m.classes.size() ? m.classes[m.seg + 1] : m.classes.empty() ? dnav::RoadClass::Street : m.classes.back();
        limit = drivingModel.limit(cls);
        reversing = m.reversing;
        return true;
    }

    void startMoving(int npcid, const dnav::PathResult& r)
    {
        auto it = movers_.find(npcid);
        if (it == movers_.end() || it->second.request != r.id) return;
        Mover& m = it->second;
        m.request = 0;
        if (r.status < dnav::PathStatus::Ok || r.points.empty()) {
            end(npcid, END_NO_PATH, false);
            return;
        }
        if (driving(m.kind)) {
            setDrivePath(npcid, m, r);
            return;
        }
        for (const auto& p : r.points) m.points.push_back(Vector3(p.x, p.y, p.z + FOOT_Z));
        m.next = 0;
        step(npcid, m);
    }

    // A driver's new path: from where the car is now along the search's points. The
    // car keeps its speed and heading.
    void setDrivePath(int npcid, Mover& m, const dnav::PathResult& r)
    {
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        IVehicle* car = npc ? npc->getVehicle() : nullptr;
        if (!car) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        m.points.clear();
        m.speeds.clear();
        m.classes.clear();
        m.junctions.clear();
        const dnav::RoadClass first = r.classes.empty() ? dnav::RoadClass::Street : r.classes.front();
        m.points.push_back(car->getPosition());
        m.speeds.push_back(1.0f);
        m.classes.push_back(first);
        m.junctions.push_back(0);
        for (size_t i = 0; i < r.points.size(); i++) {
            const auto& p = r.points[i];
            m.points.push_back(Vector3(p.x, p.y, p.z + m.carZ));
            m.speeds.push_back(i < r.speeds.size() ? r.speeds[i] : 1.0f);
            m.classes.push_back(i < r.classes.size() ? r.classes[i] : first);
            m.junctions.push_back(i < r.junctions.size() ? r.junctions[i] : 0);
        }
        m.seg = 0;
        m.ground = r.kind == dnav::PathKind::OffRoad && finder.meshLoaded(dnav::MeshKind::Car);
        planSpeeds(m, *car);
    }

    // The speed plan for m's path in this car (driving.hpp), and the distance along it.
    void planSpeeds(Mover& m, IVehicle& car)
    {
        m.car = drivingModel.handling(car.getModel());
        std::vector<dnav::Point> pts;
        pts.reserve(m.points.size());
        for (const auto& p : m.points) pts.push_back(dnav::Point { p.x, p.y, p.z });
        m.classes.resize(m.points.size(), m.classes.empty() ? dnav::RoadClass::Street : m.classes.back());
        m.junctions.resize(m.points.size(), 0);
        m.limits = drivingModel.plan(pts, m.classes, m.junctions, m.car, m.style, m.speed > 0.0f ? m.speed * SPEED_UNIT : 0.0f);
        m.cum.assign(m.points.size(), 0.0f);
        for (size_t i = 1; i < m.points.size(); i++)
            m.cum[i] = m.cum[i - 1] + glm::distance(Vector2(m.points[i - 1]), Vector2(m.points[i]));
    }

    static float wrap180(float a)
    {
        while (a > 180.0f) a -= 360.0f;
        while (a < -180.0f) a += 360.0f;
        return a;
    }

    // The point `d` metres along the path from segment `seg` at fraction `t`.
    static Vector3 along(const std::vector<Vector3>& pts, size_t seg, float t, float d)
    {
        Vector3 at = pts[seg] + (pts[seg + 1] - pts[seg]) * t;
        for (size_t i = seg; i + 1 < pts.size(); i++) {
            const Vector3 to = pts[i + 1];
            const float len = glm::distance(Vector2(at), Vector2(to));
            if (d <= len) return len > 1e-4f ? at + (to - at) * (d / len) : to;
            d -= len;
            at = to;
        }
        return pts.back();
    }

    // One step of driving (DRIVE_TICK): steer toward a point down the path, no faster
    // than the car turns; ease the speed toward what the path allows; move.
    void drive(int npcid, Mover& m, TimePoint now)
    {
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        IVehicle* car = npc ? npc->getVehicle() : nullptr;
        if (!npc || !car || npc->getVehicleSeat() != 0) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        float dt = std::chrono::duration<float>(now - m.lastDrive).count();
        m.lastDrive = now;
        const Vector3 pos = npc->getPosition();
        const Vector3 moved = pos - m.lastPos;
        m.lastPos = pos;
        if (!m.rolling) {
            m.heading = car->getRotation().ToEuler().z;
            m.v = 0.0f;
            m.rolling = true;
            dt = 0.05f;
        }
        dt = std::clamp(dt, 0.0f, 0.25f);

        float target = 0.0f, error = 0.0f, remaining = 0.0f;
        Vector3 ahead2;
        const size_t n = m.points.size();
        float aheadZ = pos.z;
        if (n >= 2) {
            // Where along the path the car is: the nearest point of the next few segments.
            size_t best = m.seg;
            float bestD = 1e30f, bestT = 0.0f;
            for (size_t i = m.seg; i + 1 < n && i < m.seg + 8; i++) {
                const Vector2 a(m.points[i]), b(m.points[i + 1]), p(pos);
                const Vector2 ab = b - a;
                const float len2 = glm::dot(ab, ab);
                const float t = len2 > 1e-6f ? std::clamp(glm::dot(p - a, ab) / len2, 0.0f, 1.0f) : 1.0f;
                const float d = glm::distance(p, a + ab * t);
                if (d < bestD) {
                    bestD = d;
                    best = i;
                    bestT = t;
                }
            }
            m.seg = best;
            const Vector3 at = m.points[best] + (m.points[best + 1] - m.points[best]) * bestT;
            const float here = m.cum[best] + (m.cum[best + 1] - m.cum[best]) * bestT;   // along the path
            remaining = m.cum[n - 1] - here;

            if (!m.chase && remaining < ARRIVE) {
                npc->stopMove();
                end(npcid, END_ARRIVED, false);
                return;
            }

            float look = std::clamp(LOOK_MIN + m.v * LOOK_TIME, LOOK_MIN, LOOK_MAX);
            Vector3 aim = along(m.points, best, bestT, look);
            // Off the road, never aim across a corner the car would clip: shorten the
            // look until the car mesh says the straight line to it is clear.
            if (m.ground) {
                const dnav::Point from { pos.x, pos.y, pos.z - m.carZ };
                while (look > 2.0f
                    && !finder.lineClear(from, dnav::Point { aim.x, aim.y, aim.z - m.carZ }, dnav::MeshKind::Car)) {
                    look *= 0.5f;
                    aim = along(m.points, best, bestT, look);
                }
            }
            const float want = glm::degrees(std::atan2(-(aim.x - pos.x), aim.y - pos.y));
            error = wrap180(want - m.heading);

            // How fast: no faster than the speed plan allows here, nor than lets it brake
            // down to each slower point ahead (a corner, a junction, a lower limit) by the
            // time it gets there, with the share of its brakes its style plans with.
            const dnav::StyleUse& use = dnav::styleUse(m.style);
            const float brakePlan = m.car.brake * use.brake;
            // The road it is on (a point carries the class of the stretch leading to it).
            target = drivingModel.roadSpeed(m.classes[best + 1], m.car, m.style, m.speed > 0.0f ? m.speed * SPEED_UNIT : 0.0f);
            if (glm::distance(Vector2(at), Vector2(m.points[best])) < PASSED_CORNER) target = std::min(target, m.limits[best]);
            const float scan = m.v * m.v / (2.0f * brakePlan) + SCAN_EXTRA;
            for (size_t i = best + 1; i < n && m.cum[i] - here <= scan; i++)
                target = std::min(target, std::sqrt(m.limits[i] * m.limits[i] + 2.0f * brakePlan * (m.cum[i] - here)));
            // Turning toward the aim follows an arc through it: no faster than the car's
            // grip holds on that arc, but fast enough to get round.
            const float abs = std::fabs(error);
            if (abs > 3.0f) {
                const float arc = look / (2.0f * std::sin(glm::radians(std::min(abs, 90.0f))));
                target = std::min(target, std::sqrt(m.car.grip * use.grip * std::max(arc, m.car.radius)));
            }
            if (abs > 30.0f) target = std::max(target, TURN_MIN_SPEED);
            // Brake to stop at the end.
            target = std::min(target, std::sqrt(2.0f * brakePlan * std::max(0.0f, remaining - ARRIVE * 0.5f)));
            ahead2 = Vector3(aim);

            // Backing up. A car can't turn on the spot: a point behind it and near, or
            // inside its turning circle, is reached by backing up with the wheels turned
            // so the nose swings round to it (a three-point turn), not by a wide loop.
            const float abs2 = std::fabs(error);
            const Vector2 to = Vector2(aim) - Vector2(pos);
            const float d = glm::length(to);
            const float hr = glm::radians(m.heading);
            const Vector2 fwd(-std::sin(hr), std::cos(hr));
            const Vector2 left(-fwd.y, fwd.x);
            const float turnR = m.car.radius;
            const Vector2 centre = Vector2(pos) + left * (glm::dot(to, left) >= 0.0f ? turnR : -turnR);
            const bool inside = glm::distance(Vector2(aim), centre) < turnR * 0.95f;
            // open.mp's own NPC component only drives forward (our patched build backs up):
            // a car told to back up that moves forward instead means it never will.
            if (m.reversing && m.v > 1.0f) {
                if (glm::dot(Vector2(moved), fwd) > 0.02f) {
                    if (++m.wrongWay >= 3 && canReverse_) {
                        canReverse_ = false;
                        core_->printLn("[nav] this NPC component can't drive backwards, so cars turn round going forward (the patched NPCs build in npcs/ can)");
                    }
                } else {
                    m.wrongWay = 0;
                }
            }
            bool back = m.reversing;
            if (!m.reversing) {
                if (abs2 > CIRCLING_TURN && m.v > 1.0f) m.circling += dt;
                else if (abs2 < CIRCLING_TURN * 0.5f) m.circling = 0.0f;
                back = (abs2 > REVERSE_BEHIND && d < REVERSE_NEAR * turnR) || (abs2 > REVERSE_INSIDE && inside)
                    || m.circling > CIRCLING_SECS;
            } else if (abs2 < REVERSE_DONE || m.backed > REVERSE_MAX) {
                back = false;
            }
            if (!canReverse_) back = false;
            // Never back into a wall the car mesh knows about.
            if (back) {
                float gz;
                const dnav::Point from { pos.x, pos.y, pos.z - m.carZ };
                const Vector2 rear = Vector2(pos) - fwd * REVERSE_CHECK;
                if (finder.groundAt(from.x, from.y, from.z, GROUND_REACH, gz, dnav::MeshKind::Car)
                    && !finder.lineClear(from, dnav::Point { rear.x, rear.y, from.z }, dnav::MeshKind::Car))
                    back = false;
            }
            if (back != m.reversing) {
                // Stop before changing direction.
                target = 0.0f;
                if (m.v < 0.5f) {
                    m.reversing = back;
                    m.backed = 0.0f;
                    m.circling = 0.0f;
                }
            } else if (m.reversing) {
                target = std::min(std::min(REVERSE_SPEED, m.car.top / 3.0f), std::max(target, TURN_MIN_SPEED));
            }
        }

        // Pulling away fades towards the car's top speed, as in GTA; braking is as hard
        // as the car can when it must (the plan itself only counts on its style's share).
        m.wantSpeed = target;
        if (target > m.v) {
            const float r = m.v / m.car.top;
            const float pull = m.car.accel * dnav::styleUse(m.style).accel * std::max(0.2f, 1.0f - r * r);
            m.v = std::min(target, m.v + pull * dt);
        } else {
            m.v = std::max(target, m.v - m.car.brake * dt);
        }
        m.v = std::max(0.0f, m.v);
        if (m.v < 0.3f && target < 0.3f) {
            m.v = 0.0f;
            if (npc->isMoving()) npc->stopMove();
            return;
        }
        // It turns no tighter than its turning circle, nor, at speed, than its grip holds.
        // Backing up, the nose still swings toward the aim: the wheels turn the other way.
        const float turning = std::max(m.car.radius, m.v * m.v / (m.car.grip * 0.95f));
        const float maxYaw = glm::degrees(std::max(m.v, TURN_MIN_SPEED * 0.5f) / turning);
        m.heading = wrap180(m.heading + std::clamp(error, -maxYaw * dt, maxYaw * dt));
        if (m.reversing) m.backed += m.v * dt;

        // NPC_Move's target: straight ahead on the new heading, at the path's height (or
        // the car mesh's ground under it).
        const float reach = std::max(AHEAD_MIN, m.v * AHEAD_TIME);
        const float h = glm::radians(m.heading);
        Vector3 ahead(pos.x - std::sin(h) * reach, pos.y + std::cos(h) * reach, pos.z);
        if (n >= 2) {
            const Vector3 onPath = along(m.points, m.seg, 0.0f,
                glm::distance(Vector2(m.points[m.seg]), Vector2(pos)) + reach);
            aheadZ = onPath.z;
        }
        float gz;
        if (m.ground && finder.groundAt(ahead.x, ahead.y, aheadZ - m.carZ, GROUND_REACH, gz, dnav::MeshKind::Car))
            aheadZ = gz + m.carZ;
        ahead.z = aheadZ;
        float moveSpeed = std::max(m.v, 0.5f) / SPEED_UNIT;
        if (m.reversing) {
            // NPC_Move faces its target and a negative speed moves away from it, so
            // backing toward a point behind is a move to its mirror image ahead.
            Vector3 behind(pos.x + std::sin(h) * reach, pos.y - std::cos(h) * reach, pos.z);
            if (finder.groundAt(behind.x, behind.y, pos.z - m.carZ, GROUND_REACH, gz, dnav::MeshKind::Car))
                behind.z = gz + m.carZ;
            ahead = pos * 2.0f - behind;
            moveSpeed = -moveSpeed;
        }
        if (!npc->move(ahead, NPCMoveType_Drive, moveSpeed, 0.05f)) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        // The front wheels show the turn (GTA: left is negative); backing up, they
        // turn the other way, and the car holds the brake/reverse key.
        uint16_t ud, lr, keys;
        npc->getKeys(ud, lr, keys);
        int steer = std::clamp(static_cast<int>(-error * 3.0f), -128, 128);
        if (m.reversing) {
            steer = -steer;
            keys = (keys & ~static_cast<uint16_t>(Key::SPRINT)) | static_cast<uint16_t>(Key::JUMP);
        }
        npc->setKeys(ud, static_cast<uint16_t>(static_cast<int16_t>(steer)), keys);
    }

    // Sends npc to its next point, or ends the move there.
    void step(int npcid, Mover& m)
    {
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        if (!npc) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        if (driving(m.kind) && (!npc->getVehicle() || npc->getVehicleSeat() != 0)) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        if (m.next >= m.points.size()) {
            // A chase never arrives: the next think plans again or drives straight on.
            if (m.chase) {
                m.points.clear();
                m.nextThink = TimePoint {};
                return;
            }
            end(npcid, END_ARRIVED, false);
            return;
        }
        m.target = m.points[m.next];
        float speed = m.speed;
        float stop = WALK_STOP;
        if (driving(m.kind)) {
            const float factor = m.next < m.speeds.size() ? m.speeds[m.next] : 1.0f;
            speed = std::max(DRIVE_MIN_SPEED, m.speed * factor);
            stop = DRIVE_STOP;
        }
        if (!npc->move(m.target, m.moveType, speed, stop)) end(npcid, END_INTERRUPTED, false);
    }

    void onNPCFinishMove(INPC& npc) override
    {
        const int id = npc.getID();
        auto it = movers_.find(id);
        if (it == movers_.end() || it->second.request) return;
        Mover& m = it->second;
        // A driver is steered by drive(), which gives it its next move.
        if (driving(m.kind)) return;
        // Finished a move of ours, or one some other script started?
        const Vector3 at = driving(m.kind) && npc.getVehicle() ? npc.getVehicle()->getPosition() : npc.getPosition();
        const float slack = driving(m.kind) ? DRIVE_STOP + 3.0f : WALK_STOP + 1.5f;
        if (!m.chase && glm::distance(at, m.target) > slack) {
            end(id, END_INTERRUPTED, false);
            return;
        }
        m.next++;
        step(id, m);
    }

    // ── Nav_Chase ────────────────────────────────────────────────────────────

    bool chase(AMX* amx, int npcid, int quarry, float speed, float carZ)
    {
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        if (!npc || !npc->getVehicle() || npc->getVehicleSeat() != 0) return false;
        IPlayer* target = core_->getPlayers().get(quarry);
        if (!target || target->getID() == npc->getID()) return false;
        end(npcid, END_STOPPED, true);
        Mover m;
        m.npc = npcid;
        m.owner = amx;
        m.kind = dnav::PathKind::Drive;
        m.moveType = NPCMoveType_Drive;
        m.speed = speed;
        m.carZ = carZ;
        m.chase = true;
        m.quarry = quarry;
        m.style = styleOf(npcid, true);
        movers_[npcid] = std::move(m);
        return true;
    }

    // Where a player is and how fast they go (in their car, if they are in one).
    static void whereIs(IPlayer& p, Vector3& pos, Vector3& vel)
    {
        IPlayerVehicleData* data = queryExtension<IPlayerVehicleData>(p);
        IVehicle* car = data ? data->getVehicle() : nullptr;
        if (car) {
            pos = car->getPosition();
            vel = car->getVelocity();
        } else {
            pos = p.getPosition();
            vel = p.getVelocity();
        }
    }

    // A chaser decides what to do next: stop beside its target, drive straight at
    // it, or follow a path to it (a road route, or the car mesh off the roads).
    void think(int npcid, Mover& m, TimePoint now)
    {
        m.nextThink = now + CHASE_THINK;
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        IVehicle* car = npc ? npc->getVehicle() : nullptr;
        if (!npc || !car || npc->getVehicleSeat() != 0) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        IPlayer* target = core_->getPlayers().get(m.quarry);
        if (!target || target->getVirtualWorld() != npc->getVirtualWorld()) {
            end(npcid, END_LOST, true);
            return;
        }
        Vector3 tpos, tvel;
        whereIs(*target, tpos, tvel);
        const Vector3 cpos = car->getPosition();
        const float dist = glm::distance(Vector2(tpos.x, tpos.y), Vector2(cpos.x, cpos.y));
        // Both on the ground, as the meshes and roads have them.
        const dnav::Point carGround { cpos.x, cpos.y, cpos.z - m.carZ };
        const dnav::Point targetGround { tpos.x, tpos.y, tpos.z - FOOT_Z };

        if (dist < CHASE_CLOSE) {
            // Beside it: drive() brakes to a stop with no path left.
            dropPlan(m);
            m.points.clear();
            m.idle = true;
            m.direct = false;
            return;
        }

        const bool carMesh = finder.meshLoaded(dnav::MeshKind::Car);
        const bool roads = finder.roads().loaded();
        bool straight = dist < CHASE_DIRECT && std::fabs(tpos.z - cpos.z) < CHASE_DIRECT_DZ
            && (carMesh ? finder.lineClear(carGround, targetGround, dnav::MeshKind::Car) : dist < CHASE_BLIND);
        // With nothing to plan on, straight at it is all there is.
        if (!carMesh && !roads) straight = true;
        if (straight) {
            dropPlan(m);
            // Lead the target, unless that point is round a corner.
            Vector3 aim = tpos + tvel * CHASE_LEAD;
            if (carMesh
                && !finder.lineClear(carGround, dnav::Point { aim.x, aim.y, aim.z - FOOT_Z }, dnav::MeshKind::Car))
                aim = tpos;
            aim.z = tpos.z - FOOT_Z + m.carZ;
            // A straight run, with the ground under it every few metres.
            m.points.clear();
            m.speeds.clear();
            m.points.push_back(cpos);
            m.speeds.push_back(1.0f);
            const float len = glm::distance(Vector2(aim), Vector2(cpos));
            const int steps = static_cast<int>(len / CHASE_GROUND_STEP);
            for (int i = 1; i <= steps; i++) {
                Vector3 p = cpos + (aim - cpos) * (static_cast<float>(i) / (steps + 1));
                float gz;
                if (carMesh && finder.groundAt(p.x, p.y, p.z - m.carZ, GROUND_REACH, gz, dnav::MeshKind::Car))
                    p.z = gz + m.carZ;
                m.points.push_back(p);
                m.speeds.push_back(1.0f);
            }
            m.points.push_back(aim);
            m.speeds.push_back(1.0f);
            m.seg = 0;
            m.ground = carMesh;
            m.kind = carMesh ? dnav::PathKind::OffRoad : dnav::PathKind::Drive;
            m.classes.assign(m.points.size(), carMesh ? dnav::RoadClass::OffRoad : dnav::RoadClass::Street);
            m.junctions.assign(m.points.size(), 0);
            planSpeeds(m, *car);
            m.direct = true;
            m.idle = false;
            return;
        }

        // Follow a path; plan one when there is none or the target has moved off it.
        const float moved = glm::distance(tpos, m.planGoal);
        const bool stale = m.direct || m.idle || m.points.empty()
            || moved > std::max(CHASE_REPLAN, dist * CHASE_REPLAN_SHARE) || now - m.planned > CHASE_REPLAN_AFTER;
        if (!stale || m.plan) return;
        // Near, straight across the car mesh (on or off the road); far, the roads while
        // the target is on one.
        const bool onRoad = roads && finder.roads().nearest(targetGround.x, targetGround.y, targetGround.z,
            CHASE_ON_ROAD, {}) >= 0;
        const bool offRoad = carMesh && now >= m.noOffRoadUntil && (!roads || !onRoad || dist < CHASE_OFFROAD_RANGE);
        const dnav::PathKind kind = offRoad ? dnav::PathKind::OffRoad : dnav::PathKind::Drive;
        const int id = finder.request(m.owner, carGround, targetGround, kind, {});
        if (!id) return;
        m.plan = id;
        m.askedGoal = tpos;
        byRequest_[id] = npcid;
    }

    void dropPlan(Mover& m)
    {
        if (m.plan) byRequest_.erase(m.plan);
        m.plan = 0;
    }

    void planDone(int npcid, Mover& m, const dnav::PathResult& r, TimePoint now)
    {
        if (m.plan != r.id) return;
        m.plan = 0;
        if (r.status < dnav::PathStatus::Ok || r.points.empty()) {
            // The car mesh found no way: roads for a while. The roads found none:
            // the next think tries again (or drives straight once close).
            if (r.kind == dnav::PathKind::OffRoad) {
                m.noOffRoadUntil = now + CHASE_NO_OFFROAD;
                core_->printLn("[nav] chase: npc %d found no way across the car navmesh (status %d)", npcid,
                    static_cast<int>(r.status));
            }
            m.nextThink = TimePoint {};
            return;
        }
        m.kind = r.kind;
        setDrivePath(npcid, m, r);
        m.planGoal = m.askedGoal;
        m.planned = now;
        m.direct = false;
        m.idle = false;
    }

    void onNPCDeath(INPC& npc, IPlayer*, int) override { end(npc.getID(), END_INTERRUPTED, false); }

    void onNPCDestroy(INPC& npc) override
    {
        end(npc.getID(), END_INTERRUPTED, false);
        styles.erase(npc.getID());
    }

    void onFree(IComponent* component) override
    {
        if (component == pawn_) {
            pawn_ = nullptr;
            setAmxFunctions();
            setAmxLookups();
        }
        if (component == npcs_) {
            npcs_ = nullptr;
            movers_.clear();
            byRequest_.clear();
        }
    }

    void reset() override
    {
        movers_.clear();
        byRequest_.clear();
    }

    void free() override { delete this; }

private:
    ICore* core_ = nullptr;
    IPawnComponent* pawn_ = nullptr;
    INPCComponent* npcs_ = nullptr;
    std::vector<dnav::PathResult> results_;     // waiting to be handed over
    std::vector<dnav::PathResult> fresh_;
    std::vector<int> thinking_;
    std::unordered_map<int, Mover> movers_;     // npc id -> its walk or drive
    std::unordered_map<int, int> byRequest_;    // request id -> npc id
};

DiamondNav* DiamondNav::instance = nullptr;

dnav::RouteOptions roadOptions(int flags)
{
    dnav::RouteOptions o;
    o.emergency = (flags & 1) != 0;
    o.boats = (flags & 2) != 0;
    return o;
}

} // namespace

COMPONENT_ENTRY_POINT()
{
    return new DiamondNav();
}

// ── Information ──────────────────────────────────────────────────────────────

dnav::MeshKind meshKind(int kind)
{
    return kind == 1 ? dnav::MeshKind::Car : dnav::MeshKind::Ped;
}

SCRIPT_API(Nav_GetTileCount, int(int kind))
{
    return DiamondNav::instance ? DiamondNav::instance->finder.tileCount(meshKind(kind)) : 0;
}

SCRIPT_API(Nav_GetRoadNodeCount, int())
{
    return DiamondNav::instance ? static_cast<int>(DiamondNav::instance->finder.roads().nodeCount()) : 0;
}

SCRIPT_API(Nav_GetPendingCount, int())
{
    return DiamondNav::instance ? static_cast<int>(DiamondNav::instance->finder.pending()) : 0;
}

// ── Searches the script reads itself ─────────────────────────────────────────

SCRIPT_API(Nav_RequestPath, int(float fromX, float fromY, float fromZ, float toX, float toY, float toZ))
{
    if (!DiamondNav::instance) return 0;
    return DiamondNav::instance->finder.request(GetAMX(), dnav::Point { fromX, fromY, fromZ },
        dnav::Point { toX, toY, toZ });
}

SCRIPT_API(Nav_RequestRoute, int(float fromX, float fromY, float fromZ, float toX, float toY, float toZ, int flags))
{
    if (!DiamondNav::instance) return 0;
    return DiamondNav::instance->finder.request(GetAMX(), dnav::Point { fromX, fromY, fromZ },
        dnav::Point { toX, toY, toZ }, dnav::PathKind::Drive, roadOptions(flags));
}

SCRIPT_API(Nav_GetPathPoint, bool(int requestid, int index, float& x, float& y, float& z))
{
    const dnav::PathResult* r = DiamondNav::instance ? DiamondNav::instance->current : nullptr;
    if (!r || r->id != requestid || index < 0 || index >= static_cast<int>(r->points.size())) return false;
    x = r->points[index].x;
    y = r->points[index].y;
    z = r->points[index].z;
    return true;
}

SCRIPT_API(Nav_GetPathPointSpeed, float(int requestid, int index))
{
    const dnav::PathResult* r = DiamondNav::instance ? DiamondNav::instance->current : nullptr;
    if (!r || r->id != requestid || index < 0 || index >= static_cast<int>(r->speeds.size())) return 1.0f;
    return r->speeds[index];
}

SCRIPT_API(Nav_GetNearestPoint, bool(float x, float y, float z, float& outX, float& outY, float& outZ, float reach, int kind))
{
    dnav::Point out {};
    if (!DiamondNav::instance
        || !DiamondNav::instance->finder.nearest(dnav::Point { x, y, z }, reach, out, meshKind(kind)))
        return false;
    outX = out.x;
    outY = out.y;
    outZ = out.z;
    return true;
}

SCRIPT_API(Nav_GetRandomPoint, bool(float& x, float& y, float& z, int kind))
{
    dnav::Point out {};
    if (!DiamondNav::instance || !DiamondNav::instance->finder.random(out, meshKind(kind))) return false;
    x = out.x;
    y = out.y;
    z = out.z;
    return true;
}

SCRIPT_API(Nav_IsLineClear, bool(float x1, float y1, float z1, float x2, float y2, float z2, int kind))
{
    return DiamondNav::instance
        && DiamondNav::instance->finder.lineClear(dnav::Point { x1, y1, z1 }, dnav::Point { x2, y2, z2 }, meshKind(kind));
}

SCRIPT_API(Nav_GetNearestRoadPoint, bool(float x, float y, float z, float& outX, float& outY, float& outZ, float reach, int flags))
{
    if (!DiamondNav::instance) return false;
    const auto& roads = DiamondNav::instance->finder.roads();
    const int i = roads.nearest(x, y, z, reach, roadOptions(flags));
    if (i < 0) return false;
    const auto& n = roads.nodes()[i];
    outX = n.x;
    outY = n.y;
    outZ = n.z;
    return true;
}

// ── NPCs moved by the component ──────────────────────────────────────────────

SCRIPT_API(Nav_WalkTo, bool(int npcid, float x, float y, float z, int moveType, float speed))
{
    if (!DiamondNav::instance) return false;
    NPCMoveType type = static_cast<NPCMoveType>(moveType);
    if (type != NPCMoveType_Walk && type != NPCMoveType_Jog && type != NPCMoveType_Sprint) type = NPCMoveType_Walk;
    return DiamondNav::instance->goTo(GetAMX(), npcid, dnav::PathKind::Walk, Vector3(x, y, z), type, speed, 0.0f, {});
}

SCRIPT_API(Nav_DriveTo, bool(int npcid, float x, float y, float z, float speed, float carZ, int flags))
{
    if (!DiamondNav::instance) return false;
    const dnav::PathKind kind = (flags & DRIVE_OFFROAD) ? dnav::PathKind::OffRoad : dnav::PathKind::Drive;
    return DiamondNav::instance->goTo(GetAMX(), npcid, kind, Vector3(x, y, z), NPCMoveType_Drive, speed, carZ,
        roadOptions(flags));
}

SCRIPT_API(Nav_Chase, bool(int npcid, int targetid, float speed, float carZ))
{
    return DiamondNav::instance && DiamondNav::instance->chase(GetAMX(), npcid, targetid, speed, carZ);
}

SCRIPT_API(Nav_Stop, bool(int npcid))
{
    if (!DiamondNav::instance || !DiamondNav::instance->isMoving(npcid)) return false;
    DiamondNav::instance->end(npcid, END_STOPPED, true);
    return true;
}

SCRIPT_API(Nav_IsMoving, bool(int npcid))
{
    return DiamondNav::instance && DiamondNav::instance->isMoving(npcid);
}

// ── How cars drive ───────────────────────────────────────────────────────────

namespace {

constexpr float KMH = 3.6f;

bool validClass(int c)
{
    return c >= 0 && c < static_cast<int>(dnav::RoadClass::Count);
}

} // namespace

SCRIPT_API(Nav_SetDriveStyle, bool(int npcid, int style))
{
    if (!DiamondNav::instance || style < -1 || style >= static_cast<int>(dnav::DriveStyle::Count)) return false;
    DiamondNav::instance->setStyle(npcid, style);
    return true;
}

SCRIPT_API(Nav_GetDriveStyle, int(int npcid))
{
    if (!DiamondNav::instance) return -1;
    auto it = DiamondNav::instance->styles.find(npcid);
    return it == DiamondNav::instance->styles.end() ? -1 : static_cast<int>(it->second);
}

SCRIPT_API(Nav_SetRoadSpeed, bool(int roadClass, float kmh))
{
    if (!DiamondNav::instance || !validClass(roadClass) || !(kmh > 0.0f)) return false;
    DiamondNav::instance->drivingModel.setLimit(static_cast<dnav::RoadClass>(roadClass), kmh / KMH);
    DiamondNav::instance->replanAll();
    return true;
}

SCRIPT_API(Nav_GetRoadSpeed, float(int roadClass))
{
    if (!DiamondNav::instance || !validClass(roadClass)) return 0.0f;
    return DiamondNav::instance->drivingModel.limit(static_cast<dnav::RoadClass>(roadClass)) * KMH;
}

SCRIPT_API(Nav_SetVehicleHandling, bool(int model, float topKmh, float accel, float brake, float grip, float radius))
{
    if (!DiamondNav::instance || !(topKmh > 0.0f && accel > 0.0f && brake > 0.0f && grip > 0.0f && radius > 0.0f))
        return false;
    DiamondNav::instance->drivingModel.setHandling(model, dnav::Handling { topKmh / KMH, accel, brake, grip, radius });
    DiamondNav::instance->replanAll();
    return true;
}

SCRIPT_API(Nav_GetVehicleHandling, bool(int model, float& topKmh, float& accel, float& brake, float& grip, float& radius))
{
    const dnav::Handling h = DiamondNav::instance ? DiamondNav::instance->drivingModel.handling(model) : dnav::Handling {};
    topKmh = h.top * KMH;
    accel = h.accel;
    brake = h.brake;
    grip = h.grip;
    radius = h.radius;
    return DiamondNav::instance != nullptr;
}

SCRIPT_API(Nav_GetVehicleModelCount, int())
{
    return DiamondNav::instance ? static_cast<int>(DiamondNav::instance->drivingModel.vehicleCount()) : 0;
}

SCRIPT_API(Nav_GetPathPointClass, int(int requestid, int index))
{
    const dnav::PathResult* r = DiamondNav::instance ? DiamondNav::instance->current : nullptr;
    if (!r || r->id != requestid || index < 0 || index >= static_cast<int>(r->classes.size())) return -1;
    return static_cast<int>(r->classes[index]);
}

SCRIPT_API(Nav_GetDriveInfo, bool(int npcid, float& speedKmh, float& wantKmh, int& roadClass, float& limitKmh, bool& reversing))
{
    float speed = 0.0f, want = 0.0f, limit = 0.0f;
    dnav::RoadClass cls = dnav::RoadClass::Street;
    bool back = false;
    if (!DiamondNav::instance || !DiamondNav::instance->driveInfo(npcid, speed, want, cls, limit, back)) return false;
    speedKmh = speed * KMH;
    wantKmh = want * KMH;
    roadClass = static_cast<int>(cls);
    limitKmh = limit * KMH;
    reversing = back;
    return true;
}
