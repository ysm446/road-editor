#include "RoadData.h"

#include <fstream>
#include <stdexcept>

using namespace DirectX;

// ---------------------------------------------------------------------------
// RoadNetwork
// ---------------------------------------------------------------------------

int RoadNetwork::AddRoad(const std::string& name)
{
    Road r;
    r.name = name;
    roads.push_back(std::move(r));
    return static_cast<int>(roads.size()) - 1;
}

void RoadNetwork::RemoveRoad(int index)
{
    if (index >= 0 && index < static_cast<int>(roads.size()))
        roads.erase(roads.begin() + index);
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
        { "name",   r.name   },
        { "closed", r.closed },
        { "points", pts      }
    };
}

static Road RoadFromJson(const nlohmann::json& j)
{
    Road r;
    r.name   = j.value("name",   "Road");
    r.closed = j.value("closed", false);
    if (j.contains("points"))
    {
        for (const auto& p : j["points"])
            r.points.push_back(PointFromJson(p));
    }
    return r;
}

// ---------------------------------------------------------------------------

bool RoadNetwork::SaveToFile(const char* path) const
{
    try
    {
        nlohmann::json root;
        root["version"] = 1;
        root["roads"]   = nlohmann::json::array();
        for (const auto& r : roads)
            root["roads"].push_back(RoadToJson(r));

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
        if (root.contains("roads"))
        {
            for (const auto& r : root["roads"])
                roads.push_back(RoadFromJson(r));
        }
        return true;
    }
    catch (...)
    {
        return false;
    }
}
