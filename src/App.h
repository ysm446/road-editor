#pragma once

#include <Windows.h>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include <DirectXMath.h>

#include "renderer/D3D11Context.h"
#include "renderer/Buffer.h"
#include "renderer/DebugDraw.h"
#include "scene/Camera.h"
#include "scene/Grid.h"
#include "scene/Terrain.h"
#include "ui/ImGuiLayer.h"
#include "editor/RoadData.h"
#include "editor/PolylineEditor.h"

// Per-frame constant buffer data (must be 16-byte aligned).
// Matches cbuffer PerFrame in common.hlsli.
struct PerFrameData
{
    DirectX::XMFLOAT4X4 viewProj;       // 64 bytes
    DirectX::XMFLOAT4X4 invViewProj;    // 64 bytes
    DirectX::XMFLOAT3   cameraPos;      // 12 bytes
    float               time;           // 4  bytes
    float               gridBaseScale;  // 4 bytes
    float               gridFadeDistance; // 4 bytes
    DirectX::XMFLOAT2   padding = { 0.0f, 0.0f }; // 8 bytes
};                                      // = 160 bytes (multiple of 16)

class App
{
public:
    bool Initialize(HINSTANCE hInstance, int nCmdShow);
    int  Run();

    static LRESULT CALLBACK StaticWndProc(HWND hwnd, UINT msg,
                                           WPARAM wParam, LPARAM lParam);
private:
    LRESULT WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);
    bool    CreateAppWindow(HINSTANCE hInstance, int nCmdShow);
    bool    SaveProject(const char* path);
    bool    LoadProject(const char* path);
    bool    SaveProjectAs();
    bool    SaveRoads(const char* path);
    bool    SaveRoadsAs();
    bool    LoadRoadsFromPath(const char* path);
    bool    OpenRoads();
    void    UpdateWindowTitle() const;
    void    AddRecentProjectPath(const char* path);
    void    NewProject();
    void    ApplyTerrainSettings();
    void    LoadViewSettings();
    void    SaveViewSettings() const;
    void    SetStatusMessage(const std::string& message);
    void    ResetPathfindingState();
    bool    SyncPathfindingEndpointsFromSelectedRoad();
    void    RebuildContourCache();
    void    UpdatePathfindingInput(bool wantMouseByImGui);
    void    DrawPathfindingPreview();
    void    DrawPathfindingOverlay(DirectX::XMMATRIX viewProj, int vpW, int vpH);
    void    DrawContourPreview();
    void    DrawPathfindingPanel();
    bool    ComputePathfindingPreview();
    bool    ApplyPathfindingPreviewAsRoad();
    void    UpdateSunInput(bool wantMouse);
    DirectX::XMFLOAT3 ComputeSunDirection() const;
    DirectX::XMFLOAT4X4 ComputeLightViewProjMatrix() const;
    void    Render();

    struct PathfindingState
    {
        bool enabled = false;
        bool hasStart = false;
        bool hasEnd = false;
        bool strictMaxGrade = true;
        float maxGradePercent = 10.0f;
        float gridStep = 5.0f;
        float slopePenalty = 40.0f;
        float turnPenalty = 8.0f;
        bool draggingStart = false;
        bool draggingEnd = false;
        DirectX::XMFLOAT3 startPos = { 0.0f, 0.0f, 0.0f };
        DirectX::XMFLOAT3 endPos = { 0.0f, 0.0f, 0.0f };
        std::vector<DirectX::XMFLOAT3> previewPath;
    };

    struct ContourSegment
    {
        DirectX::XMFLOAT3 a = { 0.0f, 0.0f, 0.0f };
        DirectX::XMFLOAT3 b = { 0.0f, 0.0f, 0.0f };
    };

    HWND m_hwnd    = nullptr;
    bool m_running = true;
    float m_time   = 0.0f;

    std::unique_ptr<D3D11Context> m_d3d;
    std::unique_ptr<ImGuiLayer>   m_imgui;
    std::unique_ptr<Camera>       m_camera;
    std::unique_ptr<Grid>         m_grid;
    std::unique_ptr<Terrain>      m_terrain;

    ConstantBuffer<PerFrameData> m_perFrameCB;

    char m_terrainPath[260] = "data/heightmap.png";
    char m_terrainTexturePath[260] = "";
    char m_projectPath[260] = "";

    // Load mesh divisions (cell counts, 0 = native image resolution - 1)
    int m_loadResW = 1024;
    int m_loadResH = 1024;
    float m_loadWidthM  = 1024.0f;
    float m_loadDepthM  = 1024.0f;
    float m_loadHeightM = 1024.0f;
    float m_loadOffsetX = 0.0f;
    float m_loadOffsetY = 0.0f;
    float m_loadOffsetZ = 0.0f;

    // Mouse-terrain intersection (updated each frame)
    bool              m_cursorHitValid = false;
    DirectX::XMFLOAT3 m_cursorHitPos   = {};
    std::string       m_statusMessage  = "Ready";
    bool              m_prevFocusKey   = false;
    bool              m_showGrid = true;
    bool              m_showRoadNames = false;
    bool              m_showIntersectionNames = true;
    bool              m_showRoadPreviewMetrics = false;
    bool              m_showRoadGradeGradient = false;
    bool              m_showFps = true;
    float             m_roadGradeRedThresholdPercent = 12.0f;
    bool              m_prevPathPickLButton = false;
    bool              m_sunDragActive = false;
    POINT             m_lastSunMouse = { 0, 0 };
    float             m_sunAzimuth = 0.98f;
    float             m_sunElevation = 0.78f;
    bool              m_showContours = false;
    bool              m_showBackgroundSettings = false;
    bool              m_showEditorDisplaySettings = false;
    float             m_contourInterval = 5.0f;
    float             m_gridBaseScale = 1.0f;
    float             m_gridFadeDistance = 1200.0f;
    float             m_roadLineThickness = 2.0f;
    float             m_selectedRoadLineThickness = 3.0f;
    float             m_roadVertexScreenRadius = 3.0f;
    float             m_intersectionScreenGizmoRadius = 10.0f;
    DirectX::XMFLOAT3 m_roadVertexColor = { 160.0f / 255.0f, 160.0f / 255.0f, 160.0f / 255.0f };
    DirectX::XMFLOAT3 m_selectedRoadColor = { 1.0f, 1.0f, 1.0f };
    DirectX::XMFLOAT3 m_intersectionCircleColor = { 80.0f / 255.0f, 240.0f / 255.0f, 1.0f };
    DirectX::XMFLOAT3 m_contourColor = { 0.18f, 0.18f, 0.18f };
    DirectX::XMFLOAT3 m_backgroundColor = { 0.12f, 0.12f, 0.14f };
    std::vector<std::string> m_recentProjectPaths;
    EditorMode        m_prevEditorMode = EditorMode::Navigate;
    PathfindingState  m_pathfinding;
    std::vector<ContourSegment> m_contourSegments;

    DebugDraw      m_debugDraw;
    RoadNetwork    m_roadNetwork;
    PolylineEditor m_editor;
};
