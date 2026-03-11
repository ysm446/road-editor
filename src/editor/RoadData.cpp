#include "RoadData.h"

#include <fstream>
#include <rpc.h>
#include <stdexcept>

#pragma comment(lib, "Rpcrt4.lib")

using namespace DirectX;

namespace
{
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
}

// ---------------------------------------------------------------------------
// RoadNetwork
// ---------------------------------------------------------------------------

int RoadNetwork::AddRoad(const std::string& name)
{
    Road r;
    r.id = GenerateUuidString();
    r.name = name;
    roads.push_back(std::move(r));
    return static_cast<int>(roads.size()) - 1;
}

int RoadNetwork::AddIntersection(XMFLOAT3 pos, const std::string& name)
{
    Intersection i;
    i.id   = GenerateUuidString();
    i.name = name + " " + std::to_string(intersections.size());
    i.type = "intersection";
    i.pos  = pos;
    intersections.push_back(std::move(i));
    return static_cast<int>(intersections.size()) - 1;
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

static nlohmann::json RoadToJson(const Road& r)
{
    nlohmann::json pts = nlohmann::json::array();
    for (const auto& p : r.points)
        pts.push_back(PointToJson(p));

    return {
        { "id",     r.id     },
        { "name",   r.name   },
        { "closed", r.closed },
        { "points", pts      },
        { "startIntersectionId", r.startIntersectionId },
        { "endIntersectionId",   r.endIntersectionId   }
    };
}

static Road RoadFromJson(const nlohmann::json& j)
{
    Road r;
    r.id     = j.value("id", std::string());
    r.name   = j.value("name",   "Road");
    r.closed = j.value("closed", false);
    r.startIntersectionId = j.value("startIntersectionId", std::string());
    r.endIntersectionId   = j.value("endIntersectionId", std::string());
    if (j.contains("points"))
    {
        for (const auto& p : j["points"])
            r.points.push_back(PointFromJson(p));
    }
    EnsureRoadId(r);
    return r;
}

static nlohmann::json IntersectionToJson(const Intersection& i)
{
    return {
        { "id",     i.id     },
        { "name",   i.name   },
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
    i.type   = j.value("type", std::string("intersection"));
    i.pos    = { j.value("x", 0.0f), j.value("y", 0.0f), j.value("z", 0.0f) };
    i.radius = j.value("radius", 4.0f);
    EnsureIntersectionId(i);
    return i;
}

// ---------------------------------------------------------------------------

bool RoadNetwork::SaveToFile(const char* path) const
{
    try
    {
        nlohmann::json root;
        root["version"] = 1;
        root["roads"]   = nlohmann::json::array();
        root["intersections"] = nlohmann::json::array();
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
        return true;
    }
    catch (...)
    {
        return false;
    }
}
