#include "RoadData.h"

#include <fstream>
#include <cmath>
#include <limits>
#include <rpc.h>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#pragma comment(lib, "Rpcrt4.lib")

using namespace DirectX;

namespace
{
constexpr int kCurveRoadTypeDefault = 0;
constexpr float kCurveDefaultTargetSpeed = 40.0f;
constexpr float kCurveDefaultFriction = 0.15f;

float JsonToFloat(const nlohmann::json& value, float defaultValue)
{
    if (value.is_number_float())
        return value.get<float>();
    if (value.is_number_integer())
        return static_cast<float>(value.get<int>());
    if (value.is_number_unsigned())
        return static_cast<float>(value.get<unsigned int>());
    return defaultValue;
}

std::string LegacyIntersectionTypeToString(const nlohmann::json& value)
{
    if (value.is_number_integer())
    {
        return value.get<int>() == 1 ? "roundabout" : "intersection";
    }
    if (value.is_string())
    {
        const std::string type = value.get<std::string>();
        return type.empty() ? "intersection" : type;
    }
    return "intersection";
}

nlohmann::json PointJsonToArray(const XMFLOAT3& pos)
{
    return nlohmann::json::array({ pos.x, pos.y, pos.z });
}

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

static RoadPoint PointFromJson(const nlohmann::json& j)
{
    RoadPoint p;
    if (j.contains("pos") && j["pos"].is_array() && j["pos"].size() >= 3)
    {
        p.pos =
        {
            JsonToFloat(j["pos"][0], 0.0f),
            JsonToFloat(j["pos"][1], 0.0f),
            JsonToFloat(j["pos"][2], 0.0f)
        };
    }
    else
    {
        p.pos = { j.value("x", 0.0f), j.value("y", 0.0f), j.value("z", 0.0f) };
    }
    p.width = j.value("width", 3.0f);
    return p;
}

static nlohmann::json CurvePointToJson(const RoadPoint& p)
{
    nlohmann::json pointJson =
        (p.curveData.is_object() ? p.curveData : nlohmann::json::object());
    pointJson["pos"] = PointJsonToArray(p.pos);
    if (!pointJson.contains("useCurvatureRadius"))
        pointJson["useCurvatureRadius"] = 0;
    if (!pointJson.contains("curvatureRadius"))
        pointJson["curvatureRadius"] = 0.0f;
    return pointJson;
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
    if (j.is_object())
        p.curveData = j;
    return p;
}

static VerticalCurvePoint VerticalCurvePointFromJson(const nlohmann::json& j)
{
    VerticalCurvePoint point;
    if (!j.is_object())
        return point;

    point.uCoord = JsonToFloat(j.value("u_coord", nlohmann::json()), 0.0f);
    point.vcl = JsonToFloat(j.value("vcl", nlohmann::json()), 50.0f);
    point.offset = JsonToFloat(j.value("offset", nlohmann::json()), 0.0f);
    return point;
}

static nlohmann::json VerticalCurvePointToJson(const VerticalCurvePoint& point)
{
    return nlohmann::json::object(
        {
            { "u_coord", point.uCoord },
            { "vcl", point.vcl },
            { "offset", point.offset }
        });
}

static BankAnglePoint BankAnglePointFromJson(const nlohmann::json& j)
{
    BankAnglePoint point;
    if (!j.is_object())
        return point;

    point.uCoord = JsonToFloat(j.value("u_coord", nlohmann::json()), 0.0f);
    point.targetSpeed = JsonToFloat(j.value("targetSpeed", nlohmann::json()), 40.0f);
    point.overrideBank = j.value("overrideBank", 0) != 0;
    point.bankAngle = JsonToFloat(j.value("bankAngle", nlohmann::json()), 0.0f);
    return point;
}

static nlohmann::json BankAnglePointToJson(const BankAnglePoint& point)
{
    return nlohmann::json::object(
        {
            { "u_coord", point.uCoord },
            { "targetSpeed", point.targetSpeed },
            { "overrideBank", point.overrideBank ? 1 : 0 },
            { "bankAngle", point.bankAngle }
        });
}

static LaneSectionPoint LaneSectionPointFromJson(const nlohmann::json& j)
{
    LaneSectionPoint point;
    if (!j.is_object())
        return point;

    point.uCoord = JsonToFloat(j.value("u_coord", nlohmann::json()), 0.0f);
    point.useLaneLeft2 = j.value("useLaneLeft2", 0) != 0;
    point.widthLaneLeft2 = JsonToFloat(j.value("widthLaneLeft2", nlohmann::json()), 3.0f);
    point.useLaneLeft1 = j.value("useLaneLeft1", 1) != 0;
    point.widthLaneLeft1 = JsonToFloat(j.value("widthLaneLeft1", nlohmann::json()), 3.0f);
    point.useLaneCenter = j.value("useLaneCenter", 1) != 0;
    point.widthLaneCenter = JsonToFloat(j.value("widthLaneCenter", nlohmann::json()), 0.0f);
    point.useLaneRight1 = j.value("useLaneRight1", 1) != 0;
    point.widthLaneRight1 = JsonToFloat(j.value("widthLaneRight1", nlohmann::json()), 3.0f);
    point.useLaneRight2 = j.value("useLaneRight2", 0) != 0;
    point.widthLaneRight2 = JsonToFloat(j.value("widthLaneRight2", nlohmann::json()), 3.0f);
    point.offsetCenter = JsonToFloat(j.value("offsetCenter", nlohmann::json()), 0.0f);
    return point;
}

static nlohmann::json LaneSectionPointToJson(const LaneSectionPoint& point)
{
    return nlohmann::json::object(
        {
            { "u_coord", point.uCoord },
            { "useLaneLeft2", point.useLaneLeft2 ? 1 : 0 },
            { "widthLaneLeft2", point.widthLaneLeft2 },
            { "useLaneLeft1", point.useLaneLeft1 ? 1 : 0 },
            { "widthLaneLeft1", point.widthLaneLeft1 },
            { "useLaneCenter", point.useLaneCenter ? 1 : 0 },
            { "widthLaneCenter", point.widthLaneCenter },
            { "useLaneRight1", point.useLaneRight1 ? 1 : 0 },
            { "widthLaneRight1", point.widthLaneRight1 },
            { "useLaneRight2", point.useLaneRight2 ? 1 : 0 },
            { "widthLaneRight2", point.widthLaneRight2 },
            { "offsetCenter", point.offsetCenter }
        });
}

static nlohmann::json RoadToJson(const Road& r)
{
    nlohmann::json roadJson =
        (r.legacyData.is_object() ? r.legacyData : nlohmann::json::object());
    nlohmann::json pts = nlohmann::json::array();
    nlohmann::json verticalCurveJson = nlohmann::json::array();
    nlohmann::json bankAngleJson = nlohmann::json::array();
    nlohmann::json laneSectionsJson = nlohmann::json::array();
    for (const auto& p : r.points)
        pts.push_back(CurvePointToJson(p));
    for (const VerticalCurvePoint& point : r.verticalCurve)
        verticalCurveJson.push_back(VerticalCurvePointToJson(point));
    for (const BankAnglePoint& point : r.bankAngle)
        bankAngleJson.push_back(BankAnglePointToJson(point));
    for (const LaneSectionPoint& point : r.laneSections)
        laneSectionsJson.push_back(LaneSectionPointToJson(point));

    roadJson["id"] = r.id;
    roadJson["name"] = r.name;
    roadJson["groupId"] = r.groupId;
    roadJson["closed"] = r.closed;
    roadJson.erase("points");
    roadJson["startIntersectionId"] = r.startIntersectionId;
    roadJson["endIntersectionId"] = r.endIntersectionId;
    roadJson["defaultTargetSpeed"] = r.defaultTargetSpeed;
    roadJson["defaultFriction"] = r.defaultFriction;
    if (!roadJson.contains("roadType"))
        roadJson["roadType"] = kCurveRoadTypeDefault;
    if (!roadJson.contains("active"))
        roadJson["active"] = 1;
    roadJson["defaultWidthLaneLeft1"] = r.defaultWidthLaneLeft1;
    roadJson["defaultWidthLaneCenter"] = r.defaultWidthLaneCenter;
    roadJson["defaultWidthLaneRight1"] = r.defaultWidthLaneRight1;
    roadJson["point"] = pts;
    roadJson["verticalCurve"] = verticalCurveJson;
    roadJson["bankAngle"] = bankAngleJson;
    roadJson["laneSections"] = laneSectionsJson;
    roadJson.erase("laneSection");
    roadJson.erase("laneWidth");
    roadJson.erase("laneLeft");
    roadJson.erase("laneRight");
    return roadJson;
}

static Road RoadFromJson(const nlohmann::json& j)
{
    Road r;
    if (j.is_object())
        r.legacyData = j;
    r.id = j.value("id", std::string());
    r.name = j.value("name", std::string("Road"));
    r.groupId = j.value("groupId", std::string());
    r.closed = j.value("closed", false);
    r.startIntersectionId = j.value("startIntersectionId", std::string());
    r.endIntersectionId = j.value("endIntersectionId", std::string());
    const float legacyLaneWidth = JsonToFloat(j.value("laneWidth", nlohmann::json()), 4.0f);
    r.defaultWidthLaneLeft1 = JsonToFloat(
        j.value("defaultWidthLaneLeft1", nlohmann::json()),
        legacyLaneWidth);
    r.defaultWidthLaneRight1 = JsonToFloat(
        j.value("defaultWidthLaneRight1", nlohmann::json()),
        legacyLaneWidth);
    r.defaultWidthLaneCenter = JsonToFloat(
        j.value("defaultWidthLaneCenter", nlohmann::json()),
        0.0f);
    r.defaultTargetSpeed = JsonToFloat(
        j.value("defaultTargetSpeed", nlohmann::json()),
        kCurveDefaultTargetSpeed);
    r.defaultFriction = JsonToFloat(
        j.value("defaultFriction", nlohmann::json()),
        kCurveDefaultFriction);
    if (j.contains("point"))
    {
        for (const auto& p : j["point"])
            r.points.push_back(CurvePointFromJson(p));
    }
    else if (j.contains("points"))
    {
        for (const auto& p : j["points"])
            r.points.push_back(PointFromJson(p));
    }
    if (j.contains("verticalCurve") && j["verticalCurve"].is_array())
    {
        for (const auto& point : j["verticalCurve"])
            r.verticalCurve.push_back(VerticalCurvePointFromJson(point));
    }
    if (j.contains("bankAngle") && j["bankAngle"].is_array())
    {
        for (const auto& point : j["bankAngle"])
            r.bankAngle.push_back(BankAnglePointFromJson(point));
    }
    if (j.contains("laneSections") && j["laneSections"].is_array())
    {
        for (const auto& point : j["laneSections"])
            r.laneSections.push_back(LaneSectionPointFromJson(point));
    }
    else if (j.contains("laneSection") && j["laneSection"].is_array())
    {
        for (const auto& point : j["laneSection"])
            r.laneSections.push_back(LaneSectionPointFromJson(point));
    }
    EnsureRoadId(r);
    return r;
}

static nlohmann::json IntersectionToJson(const Intersection& i)
{
    nlohmann::json intersectionJson =
        (i.legacyData.is_object() ? i.legacyData : nlohmann::json::object());
    intersectionJson["id"] = i.id;
    intersectionJson["name"] = i.name;
    intersectionJson["groupId"] = i.groupId;
    if (!intersectionJson.contains("type") || intersectionJson["type"].is_string())
        intersectionJson["type"] = i.type;
    intersectionJson["pos"] = PointJsonToArray(i.pos);
    intersectionJson.erase("radius");
    intersectionJson["entryDist"] = i.entryDist;
    return intersectionJson;
}

static Intersection IntersectionFromJson(const nlohmann::json& j)
{
    Intersection i;
    if (j.is_object())
        i.legacyData = j;
    i.id     = j.value("id", std::string());
    i.name   = j.value("name", std::string("Intersection"));
    i.groupId = j.value("groupId", std::string());
    if (j.contains("type"))
        i.type = LegacyIntersectionTypeToString(j["type"]);
    if (j.contains("pos") && j["pos"].is_array() && j["pos"].size() >= 3)
    {
        i.pos =
        {
            j["pos"][0].get<float>(),
            j["pos"][1].get<float>(),
            j["pos"][2].get<float>()
        };
    }
    else
    {
        i.pos = { j.value("x", 0.0f), j.value("y", 0.0f), j.value("z", 0.0f) };
    }
    i.entryDist = j.contains("entryDist")
        ? JsonToFloat(j["entryDist"], 8.0f)
        : (j.contains("radius")
            ? JsonToFloat(j["radius"], 8.0f)
            : JsonToFloat(j.value("outerRadius", nlohmann::json()), 8.0f));
    EnsureIntersectionId(i);
    return i;
}

static void MergeLegacyIntersectionData(
    const nlohmann::json& legacyIntersections,
    std::vector<Road>& roads,
    std::vector<Intersection>& intersections,
    const std::string& defaultGroupId)
{
    if (!legacyIntersections.is_array())
        return;

    std::unordered_map<std::string, std::vector<std::pair<int, bool>>> roadEndpoints;
    roadEndpoints.reserve(roads.size());
    for (int roadIndex = 0; roadIndex < static_cast<int>(roads.size()); ++roadIndex)
    {
        const Road& road = roads[roadIndex];
        if (!road.id.empty() && !road.points.empty())
        {
            roadEndpoints[road.id].push_back({ roadIndex, true });
            if (road.points.size() > 1)
                roadEndpoints[road.id].push_back({ roadIndex, false });
        }
    }

    for (const auto& item : legacyIntersections)
    {
        Intersection intersection = IntersectionFromJson(item);
        if (intersection.groupId.empty())
            intersection.groupId = defaultGroupId;

        bool hasExplicitPosition =
            item.contains("pos") ||
            item.contains("x") ||
            item.contains("y") ||
            item.contains("z");

        struct EndpointCandidate
        {
            int roadIndex = -1;
            bool isStart = true;
            XMFLOAT3 pos = { 0.0f, 0.0f, 0.0f };
        };

        std::vector<EndpointCandidate> candidates;
        if (item.contains("connectedIds") && item["connectedIds"].is_array())
        {
            for (const auto& connected : item["connectedIds"])
            {
                if (!connected.is_object())
                    continue;
                const std::string roadId = connected.value("prim_id", std::string());
                const auto endpointIt = roadEndpoints.find(roadId);
                if (endpointIt == roadEndpoints.end() || endpointIt->second.empty())
                    continue;

                for (auto& endpoint : endpointIt->second)
                {
                    Road& road = roads[endpoint.first];
                    const XMFLOAT3& endpointPos = endpoint.second
                        ? road.points.front().pos
                        : road.points.back().pos;
                    candidates.push_back({ endpoint.first, endpoint.second, endpointPos });
                }
            }
        }

        XMFLOAT3 centroid = intersection.pos;
        if (!hasExplicitPosition && !candidates.empty())
        {
            float bestPairDistance = (std::numeric_limits<float>::max)();
            bool foundPair = false;
            for (size_t a = 0; a < candidates.size(); ++a)
            {
                for (size_t b = a + 1; b < candidates.size(); ++b)
                {
                    if (candidates[a].roadIndex == candidates[b].roadIndex)
                        continue;
                    const float dx = candidates[a].pos.x - candidates[b].pos.x;
                    const float dy = candidates[a].pos.y - candidates[b].pos.y;
                    const float dz = candidates[a].pos.z - candidates[b].pos.z;
                    const float distance = dx * dx + dy * dy + dz * dz;
                    if (distance < bestPairDistance)
                    {
                        bestPairDistance = distance;
                        centroid =
                        {
                            (candidates[a].pos.x + candidates[b].pos.x) * 0.5f,
                            (candidates[a].pos.y + candidates[b].pos.y) * 0.5f,
                            (candidates[a].pos.z + candidates[b].pos.z) * 0.5f
                        };
                        foundPair = true;
                    }
                }
            }

            if (!foundPair)
            {
                centroid = { 0.0f, 0.0f, 0.0f };
                for (const EndpointCandidate& candidate : candidates)
                {
                    centroid.x += candidate.pos.x;
                    centroid.y += candidate.pos.y;
                    centroid.z += candidate.pos.z;
                }
                const float invCount = 1.0f / static_cast<float>(candidates.size());
                centroid = { centroid.x * invCount, centroid.y * invCount, centroid.z * invCount };
            }
        }

        XMFLOAT3 assignedSum = { 0.0f, 0.0f, 0.0f };
        int assignedCount = 0;
        if (item.contains("connectedIds") && item["connectedIds"].is_array())
        {
            for (const auto& connected : item["connectedIds"])
            {
                if (!connected.is_object())
                    continue;
                const std::string roadId = connected.value("prim_id", std::string());
                const auto endpointIt = roadEndpoints.find(roadId);
                if (endpointIt == roadEndpoints.end() || endpointIt->second.empty())
                    continue;

                float bestDistance = (std::numeric_limits<float>::max)();
                std::pair<int, bool> bestEndpoint = endpointIt->second.front();
                for (const auto& endpoint : endpointIt->second)
                {
                    const Road& road = roads[endpoint.first];
                    const XMFLOAT3& endpointPos = endpoint.second
                        ? road.points.front().pos
                        : road.points.back().pos;
                    const float dx = endpointPos.x - centroid.x;
                    const float dy = endpointPos.y - centroid.y;
                    const float dz = endpointPos.z - centroid.z;
                    const float distance = dx * dx + dy * dy + dz * dz;
                    if (distance < bestDistance)
                    {
                        bestDistance = distance;
                        bestEndpoint = endpoint;
                    }
                }

                Road& road = roads[bestEndpoint.first];
                const bool assignStart = bestEndpoint.second;
                const bool slotAvailable = assignStart
                    ? (road.startIntersectionId.empty() || road.startIntersectionId == intersection.id)
                    : (road.endIntersectionId.empty() || road.endIntersectionId == intersection.id);
                if (!slotAvailable && road.points.size() > 1)
                {
                    const bool fallbackIsStart = !assignStart;
                    const bool fallbackAvailable = fallbackIsStart
                        ? (road.startIntersectionId.empty() || road.startIntersectionId == intersection.id)
                        : (road.endIntersectionId.empty() || road.endIntersectionId == intersection.id);
                    if (fallbackAvailable)
                    {
                        if (fallbackIsStart)
                            road.startIntersectionId = intersection.id;
                        else
                            road.endIntersectionId = intersection.id;

                        const XMFLOAT3& endpointPos = fallbackIsStart
                            ? road.points.front().pos
                            : road.points.back().pos;
                        assignedSum.x += endpointPos.x;
                        assignedSum.y += endpointPos.y;
                        assignedSum.z += endpointPos.z;
                        ++assignedCount;
                        continue;
                    }
                }

                if (assignStart)
                    road.startIntersectionId = intersection.id;
                else
                    road.endIntersectionId = intersection.id;

                const XMFLOAT3& endpointPos = assignStart
                    ? road.points.front().pos
                    : road.points.back().pos;
                assignedSum.x += endpointPos.x;
                assignedSum.y += endpointPos.y;
                assignedSum.z += endpointPos.z;
                ++assignedCount;
            }
        }

        if ((hasExplicitPosition || !candidates.empty()) && assignedCount > 0)
        {
            const float invCount = 1.0f / static_cast<float>(assignedCount);
            intersection.pos =
            {
                assignedSum.x * invCount,
                assignedSum.y * invCount,
                assignedSum.z * invCount
            };
        }

        intersections.push_back(std::move(intersection));
    }
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

        if (root.contains("roadsLegacy") && root["roadsLegacy"].is_array())
            root["roads"] = root["roadsLegacy"];

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
        else if (root.contains("intersectionsLegacy"))
        {
            MergeLegacyIntersectionData(
                root["intersectionsLegacy"],
                roads,
                intersections,
                groups.front().id);
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
