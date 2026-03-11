#pragma once

#include <Windows.h>
#include <DirectXMath.h>
#include <string>

#include "EditorState.h"
#include "RoadData.h"
#include "scene/Terrain.h"
#include "renderer/DebugDraw.h"

// Manages interactive polyline road editing.
// Owned by App, updated every frame after camera input.
class PolylineEditor
{
public:
    enum class GizmoAxis
    {
        None,
        Center,
        X,
        Y,
        Z
    };

    // Setup
    void SetTerrain(Terrain* terrain) { m_terrain = terrain; }
    void SetNetwork(RoadNetwork* network) { m_network = network; }

    // Current mode
    EditorMode GetMode() const { return m_mode; }

    // Selection
    int  GetActiveRoadIndex()  const { return m_activeRoad;  }
    int  GetActivePointIndex() const { return m_activePoint; }

    // --- Per-frame update ---
    // Call after camera input, before rendering.
    // viewportW/H : client area size
    // mousePos    : cursor in client pixels
    // Handles left-click (add/select), Delete (remove), Enter/Esc (confirm/cancel)
    void Update(int viewportW, int viewportH,
                DirectX::XMFLOAT2 mousePos,
                DirectX::XMMATRIX invViewProj,
                bool wantMouseByImGui);

    // Queue debug lines representing the current network (3D road edges).
    void DrawNetwork(DebugDraw& dd,
                     DirectX::XMMATRIX viewProj,
                     int vpW, int vpH) const;

    // Draw 2D point circles on the viewport using ImGui foreground draw list.
    // Call inside an ImGui frame, after BeginFrame().
    void DrawOverlay(DirectX::XMMATRIX viewProj, int vpW, int vpH) const;

    // UI panels (called inside ImGui frame)
    void DrawUI(ID3D11Device* device);

    // Mode switching (also called from toolbar in App)
    void SetMode(EditorMode mode);
    void StartNewRoad();
    void ConfirmRoad();    // Enter pressed
    void CancelRoad();     // Esc pressed

    // Save / load
    bool Save(const char* path) const { return m_network->SaveToFile(path); }
    bool Load(const char* path) { return m_network->LoadFromFile(path); }
    const char* GetFilePath() const { return m_filePath; }
    void SetFilePath(const char* path) { strncpy_s(m_filePath, sizeof(m_filePath), path, _TRUNCATE); }
    bool ConsumeStatusMessage(std::string& outMessage);

private:
    // Unproject a screen pixel to a world-space ray
    void ScreenToRay(int vpW, int vpH, DirectX::XMFLOAT2 px,
                     DirectX::XMMATRIX invVP,
                     DirectX::XMFLOAT3& outOrigin,
                     DirectX::XMFLOAT3& outDir) const;

    // Find nearest road point within screenspace threshold
    // Returns road index and point index, or {-1,-1}
    void FindNearestPoint(int vpW, int vpH,
                          DirectX::XMFLOAT2 px,
                          DirectX::XMMATRIX viewProj,
                          int& outRoad, int& outPt) const;
    GizmoAxis PickGizmoAxis(int vpW, int vpH,
                            DirectX::XMFLOAT2 px,
                            DirectX::XMMATRIX viewProj) const;
    float ComputeGizmoAxisLength(DirectX::XMFLOAT3 pivot,
                                 GizmoAxis axis,
                                 DirectX::XMMATRIX viewProj,
                                 int vpW, int vpH) const;

    Terrain*     m_terrain = nullptr;
    RoadNetwork* m_network = nullptr;

    EditorMode m_mode       = EditorMode::Navigate;

    // Index of road being drawn / edited
    int m_activeRoad  = -1;
    int m_activePoint = -1;   // selected point in PointEdit mode

    // Drag state
    bool              m_dragging          = false;
    DirectX::XMFLOAT3 m_dragOffset        = { 0,0,0 };
    GizmoAxis         m_activeGizmoAxis   = GizmoAxis::None;
    DirectX::XMFLOAT3 m_axisDragStartPos  = { 0,0,0 };
    DirectX::XMFLOAT2 m_axisDragStartMouse = { 0,0 };
    DirectX::XMFLOAT3 m_planeDragStartHit = { 0,0,0 };
    DirectX::XMFLOAT3 m_planeDragNormal   = { 0,0,1 };

    // Cursor preview position (hover on terrain)
    bool              m_hasCursorPos = false;
    DirectX::XMFLOAT3 m_cursorPos    = { 0,0,0 };

    // Track left-button state for click detection
    bool m_prevLButton = false;

    // Default width for new points
    float m_defaultWidth = 3.0f;

    // Save/load path buffer
    char m_filePath[260] = "data/roads.json";
    std::string m_statusMessage;
};
