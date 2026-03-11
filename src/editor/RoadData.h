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
    std::string            name;
    std::vector<RoadPoint> points;
    bool                   closed = false;  // Loop road?

    // Convenience
    bool IsValid() const { return points.size() >= 2; }
};

// Container for all roads in the scene
class RoadNetwork
{
public:
    std::vector<Road> roads;

    // Add a new empty road, return its index
    int  AddRoad(const std::string& name = "Road");

    // Remove road at index
    void RemoveRoad(int index);

    // Serialization
    bool SaveToFile(const char* path) const;
    bool LoadFromFile(const char* path);
};
