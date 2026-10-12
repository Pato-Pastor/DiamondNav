// DiamondNav: the open.mp component (README.md).
//
// Loads navmesh/*.nav (where peds walk) and navmesh/roads.dnr (where cars drive)
// at start, searches paths on worker threads, and either hands the points to the
// script that asked (Nav_RequestPath / Nav_RequestRoute -> OnNavPathResult) or moves
// an NPC along them itself (Nav_WalkTo / Nav_DriveTo -> OnNavMoveEnd). Every
// callback runs on the game thread.

#include <sdk.hpp>

#include <Server/Components/NPCs/npcs.hpp>
#include <Server/Components/Pawn/pawn.hpp>
#include <Server/Components/Pawn/Impl/pawn_natives.hpp>
#include <Server/Components/Pawn/Impl/pawn_impl.hpp>
#include <Server/Components/Vehicles/vehicles.hpp>

#include <algorithm>
#include <thread>
#include <unordered_map>

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

// OnNavMoveEnd's reasons (DiamondNav.inc).
enum MoveEnd : int {
    END_ARRIVED = 0,
    END_NO_PATH = 1,        // no way there (or no navmesh / road graph)
    END_STOPPED = 2,        // Nav_Stop, or a new Nav_WalkTo / Nav_DriveTo
    END_INTERRUPTED = 3,    // something else moved it, it left the car, died or was destroyed
};

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
    size_t next = 0;
    Vector3 target {};
};

class DiamondNav final : public IComponent, public PawnEventHandler, public CoreEventHandler, public NPCEventHandler {
public:
    PROVIDE_UID(0xc7e6abc6cb842c2c);

    static DiamondNav* instance;

    dnav::Pathfinder finder;
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
        const unsigned hw = std::thread::hardware_concurrency();
        const int threads = static_cast<int>(std::clamp(hw > 1 ? hw - 1 : 1u, 1u, 4u));
        if (finder.loaded()) finder.start(threads);
        core_->printLn("[nav] DiamondNav %d.%d.%d: %d navmesh tiles, %d road nodes, %d search threads",
            DNAV_VERSION_MAJOR, DNAV_VERSION_MINOR, DNAV_VERSION_PATCH, finder.tileCount(),
            static_cast<int>(finder.roads().nodeCount()), finder.loaded() ? threads : 0);
        core_->getEventDispatcher().addEventHandler(this);
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

    void onTick(Microseconds, TimePoint) override
    {
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
                startMoving(npc, r);
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
        if (kind == dnav::PathKind::Drive) {
            IVehicle* car = npc->getVehicle();
            if (!car || npc->getVehicleSeat() != 0) return false;   // only a driver drives
            from = car->getPosition() - Vector3(0.0f, 0.0f, carZ);
        } else {
            from.z -= FOOT_Z;
        }
        end(npcid, END_STOPPED, true);
        const Vector3 goal = kind == dnav::PathKind::Drive ? to - Vector3(0.0f, 0.0f, carZ) : to - Vector3(0.0f, 0.0f, FOOT_Z);
        const int id = finder.request(amx, dnav::Point { from.x, from.y, from.z }, dnav::Point { goal.x, goal.y, goal.z },
            kind, road);
        if (!id) return false;
        Mover m;
        m.npc = npcid;
        m.owner = amx;
        m.request = id;
        m.kind = kind;
        m.moveType = kind == dnav::PathKind::Drive ? NPCMoveType_Drive : moveType;
        m.speed = speed;
        m.carZ = carZ;
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
        if (stopNpc && !m.request && npcs_) {
            if (INPC* npc = npcs_->get(npcid)) npc->stopMove();
        }
        if (m.owner && pawn_) {
            if (IPawnScript* script = pawn_->getScript(static_cast<AMX*>(m.owner)))
                script->Call("OnNavMoveEnd", DefaultReturnValue_False, npcid, static_cast<int>(why));
        }
    }

    bool isMoving(int npcid) const { return movers_.count(npcid) != 0; }

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
        const float lift = m.kind == dnav::PathKind::Drive ? m.carZ : FOOT_Z;
        for (const auto& p : r.points) m.points.push_back(Vector3(p.x, p.y, p.z + lift));
        m.speeds = r.speeds;
        m.next = 0;
        step(npcid, m);
    }

    // Sends npc to its next point, or ends the move there.
    void step(int npcid, Mover& m)
    {
        INPC* npc = npcs_ ? npcs_->get(npcid) : nullptr;
        if (!npc) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        if (m.kind == dnav::PathKind::Drive && (!npc->getVehicle() || npc->getVehicleSeat() != 0)) {
            end(npcid, END_INTERRUPTED, false);
            return;
        }
        if (m.next >= m.points.size()) {
            end(npcid, END_ARRIVED, false);
            return;
        }
        m.target = m.points[m.next];
        float speed = m.speed;
        float stop = WALK_STOP;
        if (m.kind == dnav::PathKind::Drive) {
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
        // Finished a move of ours, or one some other script started?
        const Vector3 at = m.kind == dnav::PathKind::Drive && npc.getVehicle() ? npc.getVehicle()->getPosition()
                                                                              : npc.getPosition();
        const float slack = m.kind == dnav::PathKind::Drive ? DRIVE_STOP + 3.0f : WALK_STOP + 1.5f;
        if (glm::distance(at, m.target) > slack) {
            end(id, END_INTERRUPTED, false);
            return;
        }
        m.next++;
        step(id, m);
    }

    void onNPCDeath(INPC& npc, IPlayer*, int) override { end(npc.getID(), END_INTERRUPTED, false); }

    void onNPCDestroy(INPC& npc) override { end(npc.getID(), END_INTERRUPTED, false); }

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

SCRIPT_API(Nav_GetTileCount, int())
{
    return DiamondNav::instance ? DiamondNav::instance->finder.tileCount() : 0;
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

SCRIPT_API(Nav_GetNearestPoint, bool(float x, float y, float z, float& outX, float& outY, float& outZ, float reach))
{
    dnav::Point out {};
    if (!DiamondNav::instance || !DiamondNav::instance->finder.nearest(dnav::Point { x, y, z }, reach, out))
        return false;
    outX = out.x;
    outY = out.y;
    outZ = out.z;
    return true;
}

SCRIPT_API(Nav_GetRandomPoint, bool(float& x, float& y, float& z))
{
    dnav::Point out {};
    if (!DiamondNav::instance || !DiamondNav::instance->finder.random(out)) return false;
    x = out.x;
    y = out.y;
    z = out.z;
    return true;
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
    return DiamondNav::instance->goTo(GetAMX(), npcid, dnav::PathKind::Drive, Vector3(x, y, z), NPCMoveType_Drive,
        speed, carZ, roadOptions(flags));
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
