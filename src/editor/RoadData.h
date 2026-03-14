#pragma once

#include <vector>
#include <string>
#include <DirectXMath.h>
#include <nlohmann/json.hpp>

// One control point on a road
struct RoadPoint
{
    DirectX::XMFLOAT3 pos;  // World-space position (y = terrain height)
    float             width; // Road half-width at this point
    nlohmann::json    curveData = nlohmann::json::object();
};

struct VerticalCurvePoint
{
    float uCoord = 0.0f;
    float vcl = 50.0f;
    float offset = 0.0f;
};

struct BankAnglePoint
{
    float uCoord = 0.0f;
    float targetSpeed = 40.0f;
    bool overrideBank = false;
    float bankAngle = 0.0f;
};

// A single polyline road
struct Road
{
    std::string            id;
    std::string            name;
    std::string            groupId;
    std::vector<RoadPoint> points;
    bool                   closed = false;  // Loop road?
    std::string            startIntersectionId;
    std::string            endIntersectionId;
    float                  laneWidth = 3.0f;
    int                    laneLeft = 1;
    int                    laneRight = 1;
    float                  defaultFriction = 0.15f;
    float                  defaultTargetSpeed = 40.0f;
    std::vector<VerticalCurvePoint> verticalCurve;
    std::vector<BankAnglePoint> bankAngle;
    nlohmann::json         legacyData = nlohmann::json::object();

    // Convenience
    bool IsValid() const { return points.size() >= 2; }
};

struct Intersection
{
    std::string         id;
    std::string         name;
    std::string         groupId;
    std::string         type = "intersection";
    DirectX::XMFLOAT3   pos = { 0.0f, 0.0f, 0.0f };
    float               entryDist = 8.0f;
    nlohmann::json      legacyData = nlohmann::json::object();
};

struct RoadGroup
{
    std::string id;
    std::string name;
    bool        visible = true;
    bool        locked = false;
};

// Container for all roads in the scene
class RoadNetwork
{
public:
    std::vector<Road> roads;
    std::vector<Intersection> intersections;
    std::vector<RoadGroup> groups;

    // Add a new empty road, return its index
    int  AddRoad(const std::string& name = "Road");
    int  AddIntersection(DirectX::XMFLOAT3 pos,
                         const std::string& name = "Intersection");
    int  AddGroup(const std::string& name = "Group");

    // Remove road at index
    void RemoveRoad(int index);
    void RemoveIntersection(int index);
    void RemoveGroup(int index);
    int  FindGroupIndexById(const std::string& id) const;
    const RoadGroup* FindGroupById(const std::string& id) const;
    RoadGroup* FindGroupById(const std::string& id);
    const RoadGroup* GetDefaultGroup() const;
    RoadGroup* GetDefaultGroup();
    void EnsureDefaultGroup();

    // Serialization
    bool SaveToFile(const char* path) const;
    bool LoadFromFile(const char* path);
};
