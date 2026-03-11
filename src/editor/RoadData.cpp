#include "RoadData.h"

#include <fstream>
#include <rpc.h>
#include <stdexcept>

#pragma comment(lib, "Rpcrt4.lib")

using namespace DirectX;

namespace
{
constexpr int kCurveRoadTypeDefault = 0;
constexpr float kCurveDefaultTargetSpeed = 30.0f;
constexpr float kCurveDefaultFriction = 0.15f;

std::string GenerateUuidString()
{
    UUID uuid = {};
    const RPC_STATUS createStatus = UuidCreate(&uuid);
    if (createStatus != RPC_S_OK && createStatus != RPC_S_UUID_LOCAL_ONLY)
        throw std::runtime_error("UuidCreate failed");

    RPC_CSTR str = nullptr;
    if (UuidToStringA(&uuid, &str) != RPC_S_OK || str == nullptr)
        throw std::runtime_error("UuidToStringA failed");

    std::string result(reinterpret_cast<const char*>(str));
    RpcStringFreeA(&str);
    return result;
}

void EnsureRoadId(Road& road)
{
    if (road.id.empty())
        road.id = GenerateUuidString();
}

void EnsureIntersectionId(Intersection& intersection)
{
    if (intersection.id.empty())
        intersection.id = GenerateUuidString();
}

void EnsureGroupId(RoadGroup& group)
{
    if (group.id.empty())
        group.id = GenerateUuidString();
}
}

// ---------------------------------------------------------------------------
// RoadNetwork
// ---------------------------------------------------------------------------

int RoadNetwork::AddRoad(const std::string& name)
{
    EnsureDefaultGroup();

    Road r;
    r.id = GenerateUuidString();
    r.name = name;
    r.groupId = groups.front().id;
    roads.push_back(std::move(r));
    return static_cast<int>(roads.size()) - 1;
}

int RoadNetwork::AddIntersection(XMFLOAT3 pos, const std::string& name)
{
    EnsureDefaultGroup();

    Intersection i;
    i.id   = GenerateUuidString();
    i.name = name + " " + std::to_string(intersections.size());
    i.groupId = groups.front().id;
    i.type = "intersection";
    i.pos  = pos;
    intersections.push_back(std::move(i));
    return static_cast<int>(intersections.size()) - 1;
}

int RoadNetwork::AddGroup(const std::string& name)
{
    RoadGroup group;
    group.id = GenerateUuidString();
    group.name = name;
    groups.push_back(std::move(group));
    return static_cast<int>(groups.size()) - 1;
}

void RoadNetwork::RemoveRoad(int index)
{
    if (index >= 0 && index < static_cast<int>(roads.size()))
        roads.erase(roads.begin() + index);
}

void RoadNetwork::RemoveIntersection(int index)
{
    if (index >= 0 && index < static_cast<int>(intersections.size()))
        intersections.erase(intersections.begin() + index);
}

void RoadNetwork::RemoveGroup(int index)
{
    if (index < 0 || index >= static_cast<int>(groups.size()))
        return;

    EnsureDefaultGroup();
    const std::string removedId = groups[index].id;
    const std::string fallbackId =
        (groups.size() > 1)
        ? groups[(index == 0) ? 1 : 0].id
        : groups[index].id;

    if (groups.size() == 1)
        return;

    for (Road& road : roads)
    {
        if (road.groupId == removedId)
            road.groupId = fallbackId;
    }

    for (Intersection& intersection : intersections)
    {
        if (intersection.groupId == removedId)
            intersection.groupId = fallbackId;
    }

    groups.erase(groups.begin() + index);
}

int RoadNetwork::FindGroupIndexById(const std::string& id) const
{
    for (int i = 0; i < static_cast<int>(groups.size()); ++i)
    {
        if (groups[i].id == id)
            return i;
    }
    return -1;
}

const RoadGroup* RoadNetwork::FindGroupById(const std::string& id) const
{
    const int index = FindGroupIndexById(id);
    return (index >= 0) ? &groups[index] : nullptr;
}

RoadGroup* RoadNetwork::FindGroupById(const std::string& id)
{
    const int index = FindGroupIndexById(id);
    return (index >= 0) ? &groups[index] : nullptr;
}

const RoadGroup* RoadNetwork::GetDefaultGroup() const
{
    return groups.empty() ? nullptr : &groups.front();
}

RoadGroup* RoadNetwork::GetDefaultGroup()
{
    return groups.empty() ? nullptr : &groups.front();
}

void RoadNetwork::EnsureDefaultGroup()
{
    if (!groups.empty())
        return;

    RoadGroup group;
    group.id = GenerateUuidString();
    group.name = "Default";
    groups.push_back(std::move(group));
}

// ---------------------------------------------------------------------------
// JSON helpers
// ---------------------------------------------------------------------------

static nlohmann::json PointToJson(const RoadPoint& p)
{
    return {
        { "x", p.pos.x }, { "y", p.pos.y }, { "z", p.pos.z },
        { "width", p.width }
    };
}

static RoadPoint PointFromJson(const nlohmann::json& j)
{
    RoadPoint p;
    p.pos   = { j.value("x", 0.0f), j.value("y", 0.0f), j.value("z", 0.0f) };
    p.width = j.value("width", 3.0f);
    return p;
}

static nlohmann::json CurvePointToJson(const RoadPoint& p)
{
    return {
        { "pos", { p.pos.x, p.pos.y, p.pos.z } },
        { "useCurvatureRadius", 0 },
        { "curvatureRadius", 0.0f }
    };
}

static RoadPoint CurvePointFromJson(const nlohmann::json& j)
{
    RoadPoint p;
    if (j.contains("pos") && j["pos"].is_array() && j["pos"].size() >= 3)
    {
        p.pos =
        {
            j["pos"][0].get<float>(),
            j["pos"][1].get<float>(),
            j["pos"][2].get<float>()
        };
    }
    else
    {
        p.pos = { 0.0f, 0.0f, 0.0f };
    }
    p.width = 3.0f;
    return p;
}

static nlohmann::json RoadToJson(const Road& r)
{
    nlohmann::json legacyPoints = nlohmann::json::array();
    nlohmann::json pts = nlohmann::json::array();
    for (const auto& p : r.points)
    {
        legacyPoints.push_back(PointToJson(p));
        pts.push_back(CurvePointToJson(p));
    }

    return {
        { "id", r.id },
        { "name", r.name },
        { "groupId", r.groupId },
        { "closed", r.closed },
        { "points", legacyPoints },
        { "startIntersectionId", r.startIntersectionId },
        { "endIntersectionId", r.endIntersectionId },
        { "roadType", kCurveRoadTypeDefault },
        { "defaultTargetSpeed", kCurveDefaultTargetSpeed },
        { "defaultFriction", kCurveDefaultFriction },
        { "active", 1 },
        { "point", pts },
        { "verticalCurve", nlohmann::json::array() },
        { "bankAngle", nlohmann::json::array() },
        { "laneSection", nlohmann::json::array() }
    };
}

static Road RoadFromJson(const nlohmann::json& j)
{
    Road r;
    r.id = j.value("id", std::string());
    r.name = j.value("name", std::string("Road"));
    r.groupId = j.value("groupId", std::string());
    r.closed = j.value("closed", false);
    r.startIntersectionId = j.value("startIntersectionId", std::string());
    r.endIntersectionId = j.value("endIntersectionId", std::string());
    if (j.contains("points"))
    {
        for (const auto& p : j["points"])
            r.points.push_back(PointFromJson(p));
    }
    else if (j.contains("point"))
    {
        for (const auto& p : j["point"])
            r.points.push_back(CurvePointFromJson(p));
    }
    EnsureRoadId(r);
    return r;
}

static nlohmann::json IntersectionToJson(const Intersection& i)
{
    return {
        { "id",     i.id     },
        { "name",   i.name   },
        { "groupId", i.groupId },
        { "type",   i.type   },
        { "x",      i.pos.x  },
        { "y",      i.pos.y  },
        { "z",      i.pos.z  },
        { "radius", i.radius }
    };
}

static Intersection IntersectionFromJson(const nlohmann::json& j)
{
    Intersection i;
    i.id     = j.value("id", std::string());
    i.name   = j.value("name", std::string("Intersection"));
    i.groupId = j.value("groupId", std::string());
    i.type   = j.value("type", std::string("intersection"));
    i.pos    = { j.value("x", 0.0f), j.value("y", 0.0f), j.value("z", 0.0f) };
    i.radius = j.value("radius", 4.0f);
    EnsureIntersectionId(i);
    return i;
}

static nlohmann::json GroupToJson(const RoadGroup& group)
{
    return {
        { "id", group.id },
        { "name", group.name },
        { "visible", group.visible },
        { "locked", group.locked }
    };
}

static RoadGroup GroupFromJson(const nlohmann::json& j)
{
    RoadGroup group;
    group.id = j.value("id", std::string());
    group.name = j.value("name", std::string("Group"));
    group.visible = j.value("visible", true);
    group.locked = j.value("locked", false);
    EnsureGroupId(group);
    return group;
}

// ---------------------------------------------------------------------------

bool RoadNetwork::SaveToFile(const char* path) const
{
    try
    {
        nlohmann::json root;
        root["version"] = 3;
        root["groups"] = nlohmann::json::array();
        root["roads"] = nlohmann::json::array();
        root["intersections"] = nlohmann::json::array();
        for (const auto& group : groups)
            root["groups"].push_back(GroupToJson(group));
        for (const auto& r : roads)
            root["roads"].push_back(RoadToJson(r));
        for (const auto& i : intersections)
            root["intersections"].push_back(IntersectionToJson(i));

        std::ofstream ofs(path);
        if (!ofs)
            return false;
        ofs << root.dump(2);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool RoadNetwork::LoadFromFile(const char* path)
{
    try
    {
        std::ifstream ifs(path);
        if (!ifs)
            return false;

        nlohmann::json root;
        ifs >> root;

        roads.clear();
        intersections.clear();
        groups.clear();

        if (root.is_array())
        {
            EnsureDefaultGroup();
            for (const auto& r : root)
                roads.push_back(RoadFromJson(r));
            const std::string defaultGroupId = groups.front().id;
            for (Road& road : roads)
                road.groupId = defaultGroupId;
            return true;
        }

        if (root.contains("groups"))
        {
            for (const auto& group : root["groups"])
                groups.push_back(GroupFromJson(group));
        }
        EnsureDefaultGroup();
        if (root.contains("roads"))
        {
            for (const auto& r : root["roads"])
                roads.push_back(RoadFromJson(r));
        }
        if (root.contains("intersections"))
        {
            for (const auto& i : root["intersections"])
                intersections.push_back(IntersectionFromJson(i));
        }

        const std::string defaultGroupId = groups.front().id;
        for (Road& road : roads)
        {
            if (FindGroupIndexById(road.groupId) < 0)
                road.groupId = defaultGroupId;
        }
        for (Intersection& intersection : intersections)
        {
            if (FindGroupIndexById(intersection.groupId) < 0)
                intersection.groupId = defaultGroupId;
        }
        return true;
    }
    catch (...)
    {
        return false;
    }
}
