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
};

// A single polyline road
struct Road
{
    std::string            id;
    std::string            name;
    std::vector<RoadPoint> points;
    bool                   closed = false;  // Loop road?
    std::string            startIntersectionId;
    std::string            endIntersectionId;

    // Convenience
    bool IsValid() const { return points.size() >= 2; }
};

struct Intersection
{
    std::string         id;
    std::string         name;
    std::string         type = "intersection";
    DirectX::XMFLOAT3   pos = { 0.0f, 0.0f, 0.0f };
    float               radius = 4.0f;
};

// Container for all roads in the scene
class RoadNetwork
{
public:
    std::vector<Road> roads;
    std::vector<Intersection> intersections;

    // Add a new empty road, return its index
    int  AddRoad(const std::string& name = "Road");
    int  AddIntersection(DirectX::XMFLOAT3 pos,
                         const std::string& name = "Intersection");

    // Remove road at index
    void RemoveRoad(int index);
    void RemoveIntersection(int index);

    // Serialization
    bool SaveToFile(const char* path) const;
    bool LoadFromFile(const char* path);
};
