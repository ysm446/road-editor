#pragma once

#include <Windows.h>
#include <DirectXMath.h>
#include <vector>
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
    struct PointRef
    {
        int roadIndex = -1;
        int pointIndex = -1;
    };

    enum class GizmoAxis
    {
        None,
        Center,
        X,
        Y,
        Z,
        RotateY,
        ScaleXZ
    };

    // Setup
    void SetTerrain(Terrain* terrain) { m_terrain = terrain; }
    void SetNetwork(RoadNetwork* network) { m_network = network; }

    // Current mode
    EditorMode GetMode() const { return m_mode; }

    // Selection
    int  GetActiveRoadIndex()  const { return m_activeRoad;  }
    int  GetActivePointIndex() const { return m_activePoint; }
    int  GetActiveIntersectionIndex() const { return m_activeIntersection; }

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
    void ResetState();

    // Save / load
    bool Save(const char* path) const { return m_network->SaveToFile(path); }
    bool Load(const char* path);
    const char* GetFilePath() const { return m_filePath; }
    void SetFilePath(const char* path) { strncpy_s(m_filePath, sizeof(m_filePath), path, _TRUNCATE); }
    bool ConsumeStatusMessage(std::string& outMessage);
    bool GetFocusTarget(DirectX::XMFLOAT3& outTarget) const;
    bool GetPrimaryRoadForPathfinding(int& outRoadIndex) const;
    void SetShowRoadNames(bool show) { m_showRoadNames = show; }
    void SetShowIntersectionNames(bool show) { m_showIntersectionNames = show; }
    void SetShowRoadPreviewMetrics(bool show) { m_showRoadPreviewMetrics = show; }
    void SetShowRoadGradeGradient(bool show) { m_showRoadGradeGradient = show; }
    void SetRoadGradeRedThresholdPercent(float value) { m_roadGradeRedThresholdPercent = value; }
    void RecordUndoState() { PushUndoState(); }

private:
    // Unproject a screen pixel to a world-space ray
    void ScreenToRay(int vpW, int vpH, DirectX::XMFLOAT2 px,
                     DirectX::XMMATRIX invVP,
                     DirectX::XMFLOAT3& outOrigin,
                     DirectX::XMFLOAT3& outDir) const;
    void SanitizeSelection();

    // Find nearest road point within screenspace threshold
    // Returns road index and point index, or {-1,-1}
    void FindNearestPoint(int vpW, int vpH,
                          DirectX::XMFLOAT2 px,
                          DirectX::XMMATRIX viewProj,
                          int& outRoad, int& outPt) const;
    int FindNearestPointOnRoad(int roadIndex,
                               int vpW, int vpH,
                               DirectX::XMFLOAT2 px,
                               DirectX::XMMATRIX viewProj) const;
    int FindNearestSegmentOnRoad(int roadIndex,
                                 int vpW, int vpH,
                                 DirectX::XMFLOAT2 px,
                                 DirectX::XMMATRIX viewProj) const;
    int FindNearestRoad(int vpW, int vpH,
                        DirectX::XMFLOAT2 px,
                        DirectX::XMMATRIX viewProj) const;
    int FindNearestIntersection(int vpW, int vpH,
                                DirectX::XMFLOAT2 px,
                                DirectX::XMMATRIX viewProj) const;
    int FindGroupIndexById(const std::string& id) const;
    bool IsRoadVisible(const Road& road) const;
    bool IsIntersectionVisible(const Intersection& intersection) const;
    void SetActiveGroupById(const std::string& id);
    int FindIntersectionIndexById(const std::string& id) const;
    bool IsSelectedRoadEndpoint() const;
    bool GetSelectedRoadConnectionId(std::string& outId) const;
    void SetSelectedRoadConnectionId(const std::string& intersectionId);
    void ClearSelectedRoadConnection();
    bool SplitSelectedRoadAtPoint();
    bool MergeSelectedRoads();
    int FindSnapIntersectionForSelectedEndpoint(int vpW, int vpH,
                                                DirectX::XMMATRIX viewProj) const;
    void SnapSelectedEndpointToIntersection(int intersectionIndex);
    void SyncRoadConnectionsForIntersection(int intersectionIndex);
    bool GetActiveGizmoPivot(DirectX::XMFLOAT3& outPivot) const;
    bool IsRoadSelected(int roadIndex) const;
    bool IsPointSelected(int roadIndex, int pointIndex) const;
    bool GetPrimarySelectedPoint(PointRef& outPoint) const;
    bool IsIntersectionSelected(int intersectionIndex) const;
    void CollectSelectedRoadIndices(std::vector<int>& outRoadIndices) const;
    void CollectSelectedIntersectionIndices(std::vector<int>& outIntersectionIndices) const;
    bool SelectAllPointsOnSelectedRoads();
    bool DisconnectSelectedRoadEndpoints();
    bool CopySelectedRoads();
    bool PasteCopiedRoadsAtCursor();
    void ClearRoadSelection();
    void SelectSingleRoad(int roadIndex);
    void ToggleRoadSelection(int roadIndex);
    void ClearPointSelection();
    void ClearIntersectionSelection();
    void SelectSinglePoint(int roadIndex, int pointIndex);
    void TogglePointSelection(int roadIndex, int pointIndex);
    void SelectSingleIntersection(int intersectionIndex);
    void ToggleIntersectionSelection(int intersectionIndex);
    void ApplyMarqueeSelection(int vpW, int vpH, DirectX::XMMATRIX viewProj, bool addToSelection, bool removeFromSelection);
    GizmoAxis PickGizmoAxis(int vpW, int vpH,
                            DirectX::XMFLOAT2 px,
                            DirectX::XMMATRIX viewProj) const;
    float ComputeGizmoAxisLength(DirectX::XMFLOAT3 pivot,
                                 GizmoAxis axis,
                                 DirectX::XMMATRIX viewProj,
                                 int vpW, int vpH) const;
    float ComputeRotationGizmoRadius(DirectX::XMFLOAT3 pivot,
                                     DirectX::XMMATRIX viewProj,
                                     int vpW, int vpH) const;
    float ComputeScaleGizmoRadius(DirectX::XMFLOAT3 pivot,
                                  DirectX::XMMATRIX viewProj,
                                  int vpW, int vpH) const;
    void PushUndoState();
    void Undo();
    void Redo();
    void ClearHistory();

    struct EditorSnapshot
    {
        RoadNetwork network;
        EditorMode  mode = EditorMode::Navigate;
        int activeRoad = -1;
        int activePoint = -1;
        std::vector<int> selectedRoads;
        std::vector<PointRef> selectedPoints;
        std::vector<int> selectedIntersections;
        int activeGroup = -1;
        int activeIntersection = -1;
        int hoverSnapIntersection = -1;
        float defaultWidth = 3.0f;
        bool snapToTerrain = true;
        bool rotateYMode = false;
        bool scaleXZMode = false;
    };
    struct RoadClipboard
    {
        std::vector<Road> roads;
        std::vector<Intersection> intersections;
        DirectX::XMFLOAT3 anchor = { 0.0f, 0.0f, 0.0f };
    };
    EditorSnapshot CaptureSnapshot() const;
    void RestoreSnapshot(const EditorSnapshot& snapshot);

    Terrain*     m_terrain = nullptr;
    RoadNetwork* m_network = nullptr;

    EditorMode m_mode       = EditorMode::Navigate;

    // Index of road being drawn / edited
    int m_activeRoad  = -1;
    int m_activePoint = -1;   // selected point in PointEdit mode
    std::vector<int> m_selectedRoads;
    std::vector<PointRef> m_selectedPoints;
    std::vector<int> m_selectedIntersections;
    int m_activeGroup = -1;
    int m_activeIntersection = -1;
    int m_hoverSnapIntersection = -1;

    // Drag state
    bool              m_dragging          = false;
    DirectX::XMFLOAT3 m_dragOffset        = { 0,0,0 };
    GizmoAxis         m_activeGizmoAxis   = GizmoAxis::None;
    DirectX::XMFLOAT3 m_axisDragStartPos  = { 0,0,0 };
    DirectX::XMFLOAT2 m_axisDragStartMouse = { 0,0 };
    DirectX::XMFLOAT3 m_planeDragStartHit = { 0,0,0 };
    DirectX::XMFLOAT3 m_planeDragNormal   = { 0,0,1 };
    float m_rotateDragStartAngle = 0.0f;
    float m_scaleDragStartDistance = 1.0f;
    std::vector<DirectX::XMFLOAT3> m_pointDragStartPositions;
    std::vector<DirectX::XMFLOAT3> m_intersectionDragStartPositions;
    bool m_marqueeSelecting = false;
    DirectX::XMFLOAT2 m_marqueeStart = { 0,0 };
    DirectX::XMFLOAT2 m_marqueeEnd = { 0,0 };
    bool m_marqueeAdditive = false;
    bool m_marqueeSubtractive = false;

    // Cursor preview position (hover on terrain)
    bool              m_hasCursorPos = false;
    DirectX::XMFLOAT3 m_cursorPos    = { 0,0,0 };

    // Track left-button state for click detection
    bool m_prevLButton = false;
    bool m_prevWKey    = false;
    bool m_prevEKey    = false;
    bool m_prevRKey    = false;
    bool m_prevVKey    = false;

    // Default width for new points
    float m_defaultWidth = 3.0f;
    bool  m_snapToTerrain = true;

    // Save/load path buffer
    char m_filePath[260] = "data/roads.json";
    std::string m_statusMessage;
    bool m_showRoadNames = false;
    bool m_showIntersectionNames = true;
    bool m_showRoadPreviewMetrics = false;
    bool m_showRoadGradeGradient = false;
    bool m_rotateYMode = false;
    bool m_scaleXZMode = false;
    float m_roadGradeRedThresholdPercent = 12.0f;
    std::vector<EditorSnapshot> m_undoStack;
    std::vector<EditorSnapshot> m_redoStack;
    bool m_prevUndoShortcut = false;
    bool m_prevRedoShortcut = false;
    bool m_prevCopyShortcut = false;
    bool m_prevPasteShortcut = false;
    RoadClipboard m_roadClipboard;
};
