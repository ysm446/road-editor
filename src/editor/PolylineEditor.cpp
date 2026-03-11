#include "PolylineEditor.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>

using namespace DirectX;

bool PolylineEditor::ConsumeStatusMessage(std::string& outMessage)
{
    if (m_statusMessage.empty())
        return false;

    outMessage = m_statusMessage;
    m_statusMessage.clear();
    return true;
}

void PolylineEditor::SanitizeSelection()
{
    if (m_activeRoad < 0 || m_activeRoad >= static_cast<int>(m_network->roads.size()))
    {
        m_activeRoad = -1;
        m_activePoint = -1;
    }
    else
    {
        const int pointCount = static_cast<int>(m_network->roads[m_activeRoad].points.size());
        if (m_activePoint < 0 || m_activePoint >= pointCount)
            m_activePoint = -1;
    }

    if (m_activeIntersection < 0 ||
        m_activeIntersection >= static_cast<int>(m_network->intersections.size()))
    {
        m_activeIntersection = -1;
    }

    if (m_hoverSnapIntersection < 0 ||
        m_hoverSnapIntersection >= static_cast<int>(m_network->intersections.size()))
    {
        m_hoverSnapIntersection = -1;
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static float Dist2D(XMFLOAT2 a, XMFLOAT2 b)
{
    float dx = a.x - b.x, dy = a.y - b.y;
    return sqrtf(dx * dx + dy * dy);
}

static float DistPointToSegment2D(XMFLOAT2 p, XMFLOAT2 a, XMFLOAT2 b)
{
    float abx = b.x - a.x;
    float aby = b.y - a.y;
    float ab2 = abx * abx + aby * aby;
    if (ab2 <= 1e-6f)
        return Dist2D(p, a);

    float apx = p.x - a.x;
    float apy = p.y - a.y;
    float t = (apx * abx + apy * aby) / ab2;
    t = std::clamp(t, 0.0f, 1.0f);
    XMFLOAT2 q = { a.x + abx * t, a.y + aby * t };
    return Dist2D(p, q);
}

static XMFLOAT3 AxisDirection(PolylineEditor::GizmoAxis axis)
{
    switch (axis)
    {
    case PolylineEditor::GizmoAxis::Center: return { 0.0f, 0.0f, 0.0f };
    case PolylineEditor::GizmoAxis::X: return { 1.0f, 0.0f, 0.0f };
    case PolylineEditor::GizmoAxis::Y: return { 0.0f, 1.0f, 0.0f };
    case PolylineEditor::GizmoAxis::Z: return { 0.0f, 0.0f, 1.0f };
    default:                           return { 0.0f, 0.0f, 0.0f };
    }
}

static bool IntersectRayPlane(XMFLOAT3 rayOrigin, XMFLOAT3 rayDir,
                              XMFLOAT3 planePoint, XMFLOAT3 planeNormal,
                              XMFLOAT3& outHit)
{
    const XMVECTOR ro = XMLoadFloat3(&rayOrigin);
    const XMVECTOR rd = XMVector3Normalize(XMLoadFloat3(&rayDir));
    const XMVECTOR pp = XMLoadFloat3(&planePoint);
    const XMVECTOR pn = XMVector3Normalize(XMLoadFloat3(&planeNormal));

    const float denom = XMVectorGetX(XMVector3Dot(rd, pn));
    if (fabsf(denom) < 1e-5f)
        return false;

    const float t = XMVectorGetX(XMVector3Dot(XMVectorSubtract(pp, ro), pn)) / denom;
    if (t < 0.0f)
        return false;

    XMStoreFloat3(&outHit, XMVectorAdd(ro, XMVectorScale(rd, t)));
    return true;
}

// Project a world point to screen pixels (returns {-1,-1} if behind camera)
static XMFLOAT2 WorldToScreen(XMFLOAT3 world, XMMATRIX viewProj,
                               int vpW, int vpH)
{
    XMVECTOR pw  = XMVectorSet(world.x, world.y, world.z, 1.0f);
    XMVECTOR ph  = XMVector4Transform(pw, viewProj);

    float w = XMVectorGetW(ph);
    if (w <= 0.0f)
        return { -1.0f, -1.0f };

    float ndcX =  XMVectorGetX(ph) / w;
    float ndcY =  XMVectorGetY(ph) / w;

    return {
        (ndcX * 0.5f + 0.5f) * vpW,
        (1.0f - (ndcY * 0.5f + 0.5f)) * vpH
    };
}

// ---------------------------------------------------------------------------
// ScreenToRay
// ---------------------------------------------------------------------------
void PolylineEditor::ScreenToRay(int vpW, int vpH, XMFLOAT2 px,
                                  XMMATRIX invVP,
                                  XMFLOAT3& outOrigin,
                                  XMFLOAT3& outDir) const
{
    // NDC [-1,1]
    float ndcX =  (px.x / vpW) * 2.0f - 1.0f;
    float ndcY = -((px.y / vpH) * 2.0f - 1.0f);

    XMVECTOR nearH = XMVector4Transform(
        XMVectorSet(ndcX, ndcY, 0.0f, 1.0f), invVP);
    XMVECTOR farH  = XMVector4Transform(
        XMVectorSet(ndcX, ndcY, 1.0f, 1.0f), invVP);

    XMVECTOR nearW = XMVectorScale(nearH, 1.0f / XMVectorGetW(nearH));
    XMVECTOR farW  = XMVectorScale(farH,  1.0f / XMVectorGetW(farH));

    XMVECTOR dir   = XMVector3Normalize(XMVectorSubtract(farW, nearW));

    XMStoreFloat3(&outOrigin, nearW);
    XMStoreFloat3(&outDir,    dir);
}

// ---------------------------------------------------------------------------
// FindNearestPoint
// ---------------------------------------------------------------------------
void PolylineEditor::FindNearestPoint(int vpW, int vpH, XMFLOAT2 px,
                                       XMMATRIX viewProj,
                                       int& outRoad, int& outPt) const
{
    outRoad = outPt = -1;
    const float threshold = 12.0f; // pixels
    float bestDist = threshold;

    for (int r = 0; r < static_cast<int>(m_network->roads.size()); ++r)
    {
        const Road& road = m_network->roads[r];
        for (int p = 0; p < static_cast<int>(road.points.size()); ++p)
        {
            XMFLOAT2 sp = WorldToScreen(road.points[p].pos, viewProj, vpW, vpH);
            if (sp.x < 0)
                continue;
            float d = Dist2D(px, sp);
            if (d < bestDist)
            {
                bestDist = d;
                outRoad  = r;
                outPt    = p;
            }
        }
    }
}

int PolylineEditor::FindNearestPointOnRoad(
    int roadIndex, int vpW, int vpH, XMFLOAT2 px, XMMATRIX viewProj) const
{
    if (roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return -1;

    const Road& road = m_network->roads[roadIndex];
    const float threshold = 12.0f;
    float bestDist = threshold;
    int bestPt = -1;

    for (int p = 0; p < static_cast<int>(road.points.size()); ++p)
    {
        const XMFLOAT2 sp = WorldToScreen(road.points[p].pos, viewProj, vpW, vpH);
        if (sp.x < 0.0f)
            continue;

        const float d = Dist2D(px, sp);
        if (d < bestDist)
        {
            bestDist = d;
            bestPt = p;
        }
    }

    return bestPt;
}

int PolylineEditor::FindNearestRoad(
    int vpW, int vpH, XMFLOAT2 px, XMMATRIX viewProj) const
{
    const float threshold = 10.0f;
    float bestDist = threshold;
    int bestRoad = -1;

    for (int r = 0; r < static_cast<int>(m_network->roads.size()); ++r)
    {
        const Road& road = m_network->roads[r];
        for (int p = 0; p + 1 < static_cast<int>(road.points.size()); ++p)
        {
            const XMFLOAT2 a = WorldToScreen(road.points[p].pos, viewProj, vpW, vpH);
            const XMFLOAT2 b = WorldToScreen(road.points[p + 1].pos, viewProj, vpW, vpH);
            if (a.x < 0.0f || b.x < 0.0f)
                continue;

            const float d = DistPointToSegment2D(px, a, b);
            if (d < bestDist)
            {
                bestDist = d;
                bestRoad = r;
            }
        }
    }

    return bestRoad;
}

int PolylineEditor::FindNearestIntersection(
    int vpW, int vpH, XMFLOAT2 px, XMMATRIX viewProj) const
{
    const float threshold = 14.0f;
    float bestDist = threshold;
    int best = -1;

    for (int i = 0; i < static_cast<int>(m_network->intersections.size()); ++i)
    {
        XMFLOAT2 sp = WorldToScreen(m_network->intersections[i].pos, viewProj, vpW, vpH);
        if (sp.x < 0.0f)
            continue;

        const float d = Dist2D(px, sp);
        if (d < bestDist)
        {
            bestDist = d;
            best = i;
        }
    }

    return best;
}

int PolylineEditor::FindIntersectionIndexById(const std::string& id) const
{
    if (id.empty())
        return -1;

    for (int i = 0; i < static_cast<int>(m_network->intersections.size()); ++i)
    {
        if (m_network->intersections[i].id == id)
            return i;
    }
    return -1;
}

bool PolylineEditor::IsSelectedRoadEndpoint() const
{
    if (m_activeRoad < 0 || m_activePoint < 0 ||
        m_activeRoad >= static_cast<int>(m_network->roads.size()))
        return false;

    const Road& road = m_network->roads[m_activeRoad];
    const int last = static_cast<int>(road.points.size()) - 1;
    return m_activePoint == 0 || m_activePoint == last;
}

bool PolylineEditor::GetSelectedRoadConnectionId(std::string& outId) const
{
    if (!IsSelectedRoadEndpoint())
        return false;

    const Road& road = m_network->roads[m_activeRoad];
    outId = (m_activePoint == 0) ? road.startIntersectionId : road.endIntersectionId;
    return !outId.empty();
}

void PolylineEditor::SetSelectedRoadConnectionId(const std::string& intersectionId)
{
    if (!IsSelectedRoadEndpoint())
        return;

    Road& road = m_network->roads[m_activeRoad];
    if (m_activePoint == 0)
        road.startIntersectionId = intersectionId;
    else
        road.endIntersectionId = intersectionId;
}

void PolylineEditor::ClearSelectedRoadConnection()
{
    SetSelectedRoadConnectionId("");
}

int PolylineEditor::FindSnapIntersectionForSelectedEndpoint(
    int vpW, int vpH, XMMATRIX viewProj) const
{
    if (!IsSelectedRoadEndpoint())
        return -1;

    const Road& road = m_network->roads[m_activeRoad];
    const XMFLOAT3 pointPos = road.points[m_activePoint].pos;
    const XMFLOAT2 pointScreen = WorldToScreen(pointPos, viewProj, vpW, vpH);
    if (pointScreen.x < 0.0f)
        return -1;

    const float threshold = 16.0f;
    float bestDist = threshold;
    int best = -1;

    for (int i = 0; i < static_cast<int>(m_network->intersections.size()); ++i)
    {
        const XMFLOAT2 isecScreen = WorldToScreen(
            m_network->intersections[i].pos, viewProj, vpW, vpH);
        if (isecScreen.x < 0.0f)
            continue;

        const float d = Dist2D(pointScreen, isecScreen);
        if (d < bestDist)
        {
            bestDist = d;
            best = i;
        }
    }

    return best;
}

void PolylineEditor::SnapSelectedEndpointToIntersection(int intersectionIndex)
{
    if (!IsSelectedRoadEndpoint() ||
        intersectionIndex < 0 ||
        intersectionIndex >= static_cast<int>(m_network->intersections.size()))
        return;

    Road& road = m_network->roads[m_activeRoad];
    road.points[m_activePoint].pos = m_network->intersections[intersectionIndex].pos;
    SetSelectedRoadConnectionId(m_network->intersections[intersectionIndex].id);
}

void PolylineEditor::SyncRoadConnectionsForIntersection(int intersectionIndex)
{
    if (intersectionIndex < 0 ||
        intersectionIndex >= static_cast<int>(m_network->intersections.size()))
        return;

    const Intersection& isec = m_network->intersections[intersectionIndex];
    for (Road& road : m_network->roads)
    {
        if (!road.startIntersectionId.empty() &&
            road.startIntersectionId == isec.id &&
            !road.points.empty())
        {
            road.points.front().pos = isec.pos;
        }
        if (!road.endIntersectionId.empty() &&
            road.endIntersectionId == isec.id &&
            !road.points.empty())
        {
            road.points.back().pos = isec.pos;
        }
    }
}

bool PolylineEditor::GetActiveGizmoPivot(XMFLOAT3& outPivot) const
{
    if (m_mode == EditorMode::PointEdit &&
        m_activeRoad >= 0 &&
        m_activePoint >= 0 &&
        m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        const Road& road = m_network->roads[m_activeRoad];
        if (m_activePoint < static_cast<int>(road.points.size()))
        {
            outPivot = road.points[m_activePoint].pos;
            return true;
        }
    }

    if (m_mode == EditorMode::IntersectionEdit &&
        m_activeIntersection >= 0 &&
        m_activeIntersection < static_cast<int>(m_network->intersections.size()))
    {
        outPivot = m_network->intersections[m_activeIntersection].pos;
        return true;
    }

    return false;
}

PolylineEditor::GizmoAxis PolylineEditor::PickGizmoAxis(
    int vpW, int vpH, XMFLOAT2 px, XMMATRIX viewProj) const
{
    XMFLOAT3 pivotPos;
    if (!GetActiveGizmoPivot(pivotPos))
        return GizmoAxis::None;
    const float threshold = 10.0f;

    XMFLOAT2 pivotScreen = WorldToScreen(pivotPos, viewProj, vpW, vpH);
    if (pivotScreen.x < 0.0f)
        return GizmoAxis::None;

    if (Dist2D(px, pivotScreen) <= threshold)
        return GizmoAxis::Center;

    struct Candidate
    {
        GizmoAxis axis;
        float dist;
    };

    Candidate best = { GizmoAxis::None, threshold };

    for (GizmoAxis axis : { GizmoAxis::X, GizmoAxis::Y, GizmoAxis::Z })
    {
        XMFLOAT3 dir = AxisDirection(axis);
        const float axisLen = ComputeGizmoAxisLength(pivotPos, axis, viewProj, vpW, vpH);
        XMFLOAT3 end =
        {
            pivotPos.x + dir.x * axisLen,
            pivotPos.y + dir.y * axisLen,
            pivotPos.z + dir.z * axisLen
        };

        XMFLOAT2 endScreen = WorldToScreen(end, viewProj, vpW, vpH);
        if (endScreen.x < 0.0f)
            continue;

        float d = DistPointToSegment2D(px,
            pivotScreen,
            endScreen);
        if (d < best.dist)
            best = { axis, d };
    }

    return best.axis;
}

float PolylineEditor::ComputeGizmoAxisLength(
    XMFLOAT3 pivot, GizmoAxis axis, XMMATRIX viewProj, int vpW, int vpH) const
{
    const float targetPixels = 72.0f;
    const XMFLOAT3 dir = AxisDirection(axis);
    const XMFLOAT2 pivotScreen = WorldToScreen(pivot, viewProj, vpW, vpH);
    if (pivotScreen.x < 0.0f)
        return 2.0f;

    const XMFLOAT3 unitEnd =
    {
        pivot.x + dir.x,
        pivot.y + dir.y,
        pivot.z + dir.z
    };
    const XMFLOAT2 unitScreen = WorldToScreen(unitEnd, viewProj, vpW, vpH);
    if (unitScreen.x < 0.0f)
        return 2.0f;

    const float pixelsPerUnit = Dist2D(pivotScreen, unitScreen);
    if (pixelsPerUnit <= 1e-3f)
        return 2.0f;

    return std::clamp(targetPixels / pixelsPerUnit, 0.5f, 1000.0f);
}

// ---------------------------------------------------------------------------
// SetMode
// ---------------------------------------------------------------------------
void PolylineEditor::SetMode(EditorMode mode)
{
    // Cancel any in-progress draw
    if (m_mode == EditorMode::PolylineDraw && mode != EditorMode::PolylineDraw)
        CancelRoad();

    m_mode = mode;
    if (mode != EditorMode::PointEdit)
    {
        m_activePoint = -1;
        m_dragging    = false;
        m_activeGizmoAxis = GizmoAxis::None;
    }
    if (mode != EditorMode::IntersectionEdit)
        m_activeIntersection = -1;
}

void PolylineEditor::StartNewRoad()
{
    m_activeRoad  = m_network->AddRoad("Road " +
        std::to_string(m_network->roads.size()));
    m_activePoint = -1;
    m_mode        = EditorMode::PolylineDraw;
}

void PolylineEditor::ConfirmRoad()
{
    if (m_mode == EditorMode::PolylineDraw)
    {
        // Remove incomplete road if fewer than 2 points
        if (m_activeRoad >= 0 &&
            m_activeRoad < static_cast<int>(m_network->roads.size()))
        {
            if (!m_network->roads[m_activeRoad].IsValid())
                m_network->RemoveRoad(m_activeRoad);
        }
        m_activeRoad  = -1;
        m_activePoint = -1;
        m_mode        = EditorMode::Navigate;
    }
}

void PolylineEditor::CancelRoad()
{
    if (m_mode == EditorMode::PolylineDraw)
    {
        if (m_activeRoad >= 0 &&
            m_activeRoad < static_cast<int>(m_network->roads.size()))
        {
            m_network->RemoveRoad(m_activeRoad);
        }
        m_activeRoad  = -1;
        m_activePoint = -1;
        m_mode        = EditorMode::Navigate;
    }
}

// ---------------------------------------------------------------------------
// Update
// ---------------------------------------------------------------------------
void PolylineEditor::Update(int vpW, int vpH,
                             XMFLOAT2 mousePos,
                             XMMATRIX invViewProj,
                             bool wantMouse)
{
    SanitizeSelection();

    bool alt = (GetAsyncKeyState(VK_MENU) & 0x8000) != 0;
    bool lDown = (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;
    bool lClick = lDown && !m_prevLButton;  // rising edge
    bool wDown = (GetAsyncKeyState('W') & 0x8000) != 0;
    bool wPress = wDown && !m_prevWKey;

    if (wantMouse || alt)
    {
        m_hasCursorPos = false;
        m_hoverSnapIntersection = -1;
        m_prevLButton  = lDown;
        m_prevWKey     = wDown;
        return;
    }
    m_prevLButton = lDown;
    m_prevWKey = wDown;

    // Compute ray
    XMFLOAT3 rayOrig, rayDir;
    ScreenToRay(vpW, vpH, mousePos, invViewProj, rayOrig, rayDir);

    // Terrain intersection for cursor preview and placement
    XMFLOAT3 hitPos = {};
    bool hasHit = m_terrain && m_terrain->IsReady() &&
                  m_terrain->Raycast(rayOrig, rayDir, hitPos);
    m_hasCursorPos = hasHit;
    m_cursorPos    = hitPos;

    if (wPress)
    {
        if (m_mode == EditorMode::PointEdit || m_mode == EditorMode::IntersectionEdit)
        {
            m_mode = EditorMode::Navigate;
            m_dragging = false;
            m_activeGizmoAxis = GizmoAxis::None;
            m_hoverSnapIntersection = -1;
        }
        else if (m_activeIntersection >= 0)
        {
            SetMode(EditorMode::IntersectionEdit);
        }
        else if (m_activeRoad >= 0 && m_activePoint >= 0)
        {
            SetMode(EditorMode::PointEdit);
        }
    }

    if (m_mode == EditorMode::Navigate)
    {
        const XMMATRIX viewProj = XMMatrixInverse(nullptr, invViewProj);
        if (lClick)
        {
            if (m_activeRoad >= 0)
            {
                const int selPoint = FindNearestPointOnRoad(
                    m_activeRoad, vpW, vpH, mousePos, viewProj);
                if (selPoint >= 0)
                {
                    m_activePoint = selPoint;
                    m_activeIntersection = -1;
                    return;
                }
            }

            const int selIntersection = FindNearestIntersection(vpW, vpH, mousePos, viewProj);
            if (selIntersection >= 0)
            {
                m_activeIntersection = selIntersection;
                m_activeRoad = -1;
                m_activePoint = -1;
                return;
            }

            const int selRoad = FindNearestRoad(vpW, vpH, mousePos, viewProj);
            if (selRoad >= 0)
            {
                m_activeRoad = selRoad;
                m_activePoint = -1;
                m_activeIntersection = -1;
                return;
            }

            m_activeRoad = -1;
            m_activePoint = -1;
            m_activeIntersection = -1;
        }
        return;
    }

    // --- PolylineDraw mode ---
    if (m_mode == EditorMode::PolylineDraw)
    {
        if (lClick && hasHit &&
            m_activeRoad >= 0 &&
            m_activeRoad < static_cast<int>(m_network->roads.size()))
        {
            RoadPoint rp;
            rp.pos   = hitPos;
            rp.width = m_defaultWidth;
            m_network->roads[m_activeRoad].points.push_back(rp);
        }

        // Enter -> confirm, Esc -> cancel
        if (GetAsyncKeyState(VK_RETURN) & 0x8000)
            ConfirmRoad();
        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
            CancelRoad();

        return;
    }

    if (m_mode == EditorMode::IntersectionDraw)
    {
        if (lClick && hasHit)
        {
            m_activeIntersection = m_network->AddIntersection(hitPos);
            m_statusMessage = "Intersection placed";
        }

        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
            SetMode(EditorMode::Navigate);

        return;
    }

    // --- PointEdit mode ---
    if (m_mode == EditorMode::PointEdit)
    {
        // Rebuild viewProj from invViewProj (inverse)
        XMMATRIX viewProj = XMMatrixInverse(nullptr, invViewProj);

        if (m_dragging && lDown)
        {
            if (m_activeGizmoAxis != GizmoAxis::None &&
                m_activeRoad  >= 0 &&
                m_activePoint >= 0 &&
                m_activeRoad  < static_cast<int>(m_network->roads.size()))
            {
                Road& road = m_network->roads[m_activeRoad];
                if (m_activePoint < static_cast<int>(road.points.size()))
                {
                    if (m_snapToTerrain &&
                        hasHit &&
                        m_activeGizmoAxis == GizmoAxis::Center)
                    {
                        RoadPoint& point = road.points[m_activePoint];
                        point.pos = hitPos;
                    }
                    else if (m_activeGizmoAxis == GizmoAxis::Center)
                    {
                        XMFLOAT3 planeHit;
                        if (IntersectRayPlane(rayOrig, rayDir, m_axisDragStartPos,
                                              m_planeDragNormal, planeHit))
                        {
                            RoadPoint& point = road.points[m_activePoint];
                            point.pos =
                            {
                                m_axisDragStartPos.x + (planeHit.x - m_planeDragStartHit.x),
                                m_axisDragStartPos.y + (planeHit.y - m_planeDragStartHit.y),
                                m_axisDragStartPos.z + (planeHit.z - m_planeDragStartHit.z)
                            };
                        }
                    }
                    else
                    {
                        XMFLOAT3 axisDir = AxisDirection(m_activeGizmoAxis);
                        const float axisLen = ComputeGizmoAxisLength(
                            m_axisDragStartPos, m_activeGizmoAxis, viewProj, vpW, vpH);
                        const XMFLOAT2 startScreen =
                            WorldToScreen(m_axisDragStartPos, viewProj, vpW, vpH);
                        const XMFLOAT2 endScreen = WorldToScreen(
                            {
                                m_axisDragStartPos.x + axisDir.x * axisLen,
                                m_axisDragStartPos.y + axisDir.y * axisLen,
                                m_axisDragStartPos.z + axisDir.z * axisLen
                            },
                            viewProj, vpW, vpH);

                        const float axisScreenLen = Dist2D(startScreen, endScreen);
                        if (axisScreenLen > 1e-3f)
                        {
                            const XMFLOAT2 axisScreenDir =
                            {
                                (endScreen.x - startScreen.x) / axisScreenLen,
                                (endScreen.y - startScreen.y) / axisScreenLen
                            };
                            const XMFLOAT2 mouseDelta =
                            {
                                mousePos.x - m_axisDragStartMouse.x,
                                mousePos.y - m_axisDragStartMouse.y
                            };
                            const float deltaPixels =
                                mouseDelta.x * axisScreenDir.x +
                                mouseDelta.y * axisScreenDir.y;
                            const float deltaWorld = (deltaPixels / axisScreenLen) * axisLen;
                            RoadPoint& point = road.points[m_activePoint];
                            point.pos =
                            {
                                m_axisDragStartPos.x + axisDir.x * deltaWorld,
                                m_axisDragStartPos.y + axisDir.y * deltaWorld,
                                m_axisDragStartPos.z + axisDir.z * deltaWorld
                            };
                        }
                    }

                    if (m_snapToTerrain &&
                        m_terrain && m_terrain->IsReady() &&
                        m_activeGizmoAxis != GizmoAxis::Y)
                    {
                        road.points[m_activePoint].pos.y =
                            m_terrain->GetHeightAt(
                                road.points[m_activePoint].pos.x,
                                road.points[m_activePoint].pos.z);
                    }

                    if (IsSelectedRoadEndpoint())
                        m_hoverSnapIntersection = FindSnapIntersectionForSelectedEndpoint(vpW, vpH, viewProj);
                    else
                        m_hoverSnapIntersection = -1;
                }
            }
        }
        else if (m_dragging && !lDown)
        {
            if (IsSelectedRoadEndpoint())
            {
                if (m_hoverSnapIntersection >= 0)
                {
                    SnapSelectedEndpointToIntersection(m_hoverSnapIntersection);
                    m_statusMessage = "Road endpoint connected";
                }
                else
                {
                    ClearSelectedRoadConnection();
                }
            }
            m_dragging = false;
            m_activeGizmoAxis = GizmoAxis::None;
            m_hoverSnapIntersection = -1;
        }
        else if (lClick)
        {
            GizmoAxis gizmoAxis = PickGizmoAxis(vpW, vpH, mousePos, viewProj);
            if (gizmoAxis != GizmoAxis::None &&
                m_activeRoad >= 0 &&
                m_activePoint >= 0 &&
                m_activeRoad < static_cast<int>(m_network->roads.size()))
            {
                Road& road = m_network->roads[m_activeRoad];
                if (m_activePoint < static_cast<int>(road.points.size()))
                {
                    m_activeGizmoAxis  = gizmoAxis;
                    m_axisDragStartPos = road.points[m_activePoint].pos;
                    m_axisDragStartMouse = mousePos;
                    m_planeDragNormal = rayDir;
                    if (!IntersectRayPlane(rayOrig, rayDir, m_axisDragStartPos,
                                           m_planeDragNormal, m_planeDragStartHit))
                    {
                        m_planeDragStartHit = m_axisDragStartPos;
                    }
                    m_dragging         = true;
                }
            }
            else
            {
                // Try to select a point
                int selRoad, selPt;
                FindNearestPoint(vpW, vpH, mousePos, viewProj, selRoad, selPt);
                if (selRoad >= 0)
                {
                    m_activeRoad       = selRoad;
                    m_activePoint      = selPt;
                    m_dragging         = false;
                    m_activeGizmoAxis  = GizmoAxis::None;
                    m_hoverSnapIntersection = -1;
                }
                else
                {
                    m_activeRoad       = -1;
                    m_activePoint      = -1;
                    m_dragging         = false;
                    m_activeGizmoAxis  = GizmoAxis::None;
                    m_hoverSnapIntersection = -1;
                    m_mode             = EditorMode::Navigate;
                }
            }
        }

        // Delete selected point
        if ((GetAsyncKeyState(VK_DELETE) & 0x8000) &&
            m_activeRoad  >= 0 &&
            m_activePoint >= 0 &&
            m_activeRoad  < static_cast<int>(m_network->roads.size()))
        {
            Road& road = m_network->roads[m_activeRoad];
            if (m_activePoint < static_cast<int>(road.points.size()))
            {
                if (m_activePoint == 0)
                    road.startIntersectionId.clear();
                if (m_activePoint == static_cast<int>(road.points.size()) - 1)
                    road.endIntersectionId.clear();
                road.points.erase(road.points.begin() + m_activePoint);
                m_activePoint = -1;
                m_activeGizmoAxis = GizmoAxis::None;
                m_hoverSnapIntersection = -1;
            }
        }
    }

    if (m_mode == EditorMode::IntersectionEdit)
    {
        XMMATRIX viewProj = XMMatrixInverse(nullptr, invViewProj);

        if (m_dragging && lDown)
        {
            if (m_activeGizmoAxis != GizmoAxis::None &&
                m_activeIntersection >= 0 &&
                m_activeIntersection < static_cast<int>(m_network->intersections.size()))
            {
                Intersection& isec = m_network->intersections[m_activeIntersection];
                XMFLOAT3 newPos = m_axisDragStartPos;

                if (m_snapToTerrain &&
                    hasHit &&
                    m_activeGizmoAxis == GizmoAxis::Center)
                {
                    newPos = hitPos;
                }
                else if (m_activeGizmoAxis == GizmoAxis::Center)
                {
                    XMFLOAT3 planeHit;
                    if (IntersectRayPlane(rayOrig, rayDir, m_axisDragStartPos,
                                          m_planeDragNormal, planeHit))
                    {
                        newPos =
                        {
                            m_axisDragStartPos.x + (planeHit.x - m_planeDragStartHit.x),
                            m_axisDragStartPos.y + (planeHit.y - m_planeDragStartHit.y),
                            m_axisDragStartPos.z + (planeHit.z - m_planeDragStartHit.z)
                        };
                    }
                }
                else
                {
                    XMFLOAT3 axisDir = AxisDirection(m_activeGizmoAxis);
                    const float axisLen = ComputeGizmoAxisLength(
                        m_axisDragStartPos, m_activeGizmoAxis, viewProj, vpW, vpH);
                    const XMFLOAT2 startScreen =
                        WorldToScreen(m_axisDragStartPos, viewProj, vpW, vpH);
                    const XMFLOAT2 endScreen = WorldToScreen(
                        {
                            m_axisDragStartPos.x + axisDir.x * axisLen,
                            m_axisDragStartPos.y + axisDir.y * axisLen,
                            m_axisDragStartPos.z + axisDir.z * axisLen
                        },
                        viewProj, vpW, vpH);
                    const float axisScreenLen = Dist2D(startScreen, endScreen);
                    if (axisScreenLen > 1e-3f)
                    {
                        const XMFLOAT2 axisScreenDir =
                        {
                            (endScreen.x - startScreen.x) / axisScreenLen,
                            (endScreen.y - startScreen.y) / axisScreenLen
                        };
                        const XMFLOAT2 mouseDelta =
                        {
                            mousePos.x - m_axisDragStartMouse.x,
                            mousePos.y - m_axisDragStartMouse.y
                        };
                        const float deltaPixels =
                            mouseDelta.x * axisScreenDir.x +
                            mouseDelta.y * axisScreenDir.y;
                        const float deltaWorld = (deltaPixels / axisScreenLen) * axisLen;
                        newPos =
                        {
                            m_axisDragStartPos.x + axisDir.x * deltaWorld,
                            m_axisDragStartPos.y + axisDir.y * deltaWorld,
                            m_axisDragStartPos.z + axisDir.z * deltaWorld
                        };
                    }
                }

                if (m_snapToTerrain &&
                    m_terrain && m_terrain->IsReady() &&
                    m_activeGizmoAxis != GizmoAxis::Y)
                    newPos.y = m_terrain->GetHeightAt(newPos.x, newPos.z);
                isec.pos = newPos;
                SyncRoadConnectionsForIntersection(m_activeIntersection);
            }
        }
        else if (m_dragging && !lDown)
        {
            m_dragging = false;
            m_activeGizmoAxis = GizmoAxis::None;
        }
        else if (lClick)
        {
            GizmoAxis gizmoAxis = PickGizmoAxis(vpW, vpH, mousePos, viewProj);
            if (gizmoAxis != GizmoAxis::None &&
                m_activeIntersection >= 0 &&
                m_activeIntersection < static_cast<int>(m_network->intersections.size()))
            {
                m_activeGizmoAxis = gizmoAxis;
                m_axisDragStartPos = m_network->intersections[m_activeIntersection].pos;
                m_axisDragStartMouse = mousePos;
                m_planeDragNormal = rayDir;
                if (!IntersectRayPlane(rayOrig, rayDir, m_axisDragStartPos,
                                       m_planeDragNormal, m_planeDragStartHit))
                {
                    m_planeDragStartHit = m_axisDragStartPos;
                }
                m_dragging = true;
            }
            else
            {
                m_activeIntersection = FindNearestIntersection(vpW, vpH, mousePos, viewProj);
                m_activeGizmoAxis = GizmoAxis::None;
                m_dragging = false;
                if (m_activeIntersection < 0)
                    m_mode = EditorMode::Navigate;
            }
        }

        if ((GetAsyncKeyState(VK_DELETE) & 0x8000) &&
            m_activeIntersection >= 0 &&
            m_activeIntersection < static_cast<int>(m_network->intersections.size()))
        {
            const std::string removedId = m_network->intersections[m_activeIntersection].id;
            for (Road& road : m_network->roads)
            {
                if (road.startIntersectionId == removedId)
                    road.startIntersectionId.clear();
                if (road.endIntersectionId == removedId)
                    road.endIntersectionId.clear();
            }
            m_network->RemoveIntersection(m_activeIntersection);
            m_activeIntersection = -1;
            m_statusMessage = "Intersection deleted";
        }
    }
}

// ---------------------------------------------------------------------------
// DrawNetwork / DrawOverlay
// ---------------------------------------------------------------------------

void PolylineEditor::DrawNetwork(DebugDraw& dd, XMMATRIX viewProj, int vpW, int vpH) const
{
    const_cast<PolylineEditor*>(this)->SanitizeSelection();

    static const XMFLOAT4 colorRoad     = { 1.0f, 0.8f, 0.1f, 1.0f };
    static const XMFLOAT4 colorSelected = { 1.0f, 0.3f, 0.3f, 1.0f };
    static const XMFLOAT4 colorCursor   = { 0.2f, 1.0f, 0.4f, 0.4f };
    static const XMFLOAT4 colorAxisX    = { 1.0f, 0.2f, 0.2f, 1.0f };
    static const XMFLOAT4 colorAxisY    = { 0.2f, 1.0f, 0.2f, 1.0f };
    static const XMFLOAT4 colorAxisZ    = { 0.2f, 0.6f, 1.0f, 1.0f };
    static const XMFLOAT4 colorPivot    = { 1.0f, 1.0f, 1.0f, 0.9f };
    static const XMFLOAT4 colorIntersection = { 0.3f, 0.95f, 1.0f, 1.0f };
    static const XMFLOAT4 colorIntersectionSelected = { 1.0f, 0.4f, 0.2f, 1.0f };
    static const XMFLOAT4 colorConnection = { 0.9f, 0.9f, 0.3f, 0.9f };
    static const XMFLOAT4 colorSnapCandidate = { 1.0f, 1.0f, 0.2f, 1.0f };

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        XMFLOAT4 col = (ri == m_activeRoad) ? colorSelected : colorRoad;

        for (int pi = 0; pi + 1 < static_cast<int>(road.points.size()); ++pi)
            dd.AddLine(road.points[pi].pos, road.points[pi + 1].pos, col);

        if (road.closed && road.points.size() >= 2)
            dd.AddLine(road.points.back().pos, road.points.front().pos, col);

        const int startIsec = FindIntersectionIndexById(road.startIntersectionId);
        const int endIsec   = FindIntersectionIndexById(road.endIntersectionId);
        if (startIsec >= 0 && !road.points.empty())
            dd.AddLine(road.points.front().pos, m_network->intersections[startIsec].pos, colorConnection);
        if (endIsec >= 0 && !road.points.empty())
            dd.AddLine(road.points.back().pos, m_network->intersections[endIsec].pos, colorConnection);
    }

    for (int ii = 0; ii < static_cast<int>(m_network->intersections.size()); ++ii)
    {
        const Intersection& isec = m_network->intersections[ii];
        const XMFLOAT4 col = (ii == m_activeIntersection)
            ? colorIntersectionSelected
            : colorIntersection;
        const float r = (std::max)(1.5f, isec.radius * 0.35f);
        dd.AddLine({ isec.pos.x - r, isec.pos.y, isec.pos.z },
                   { isec.pos.x + r, isec.pos.y, isec.pos.z }, col);
        dd.AddLine({ isec.pos.x, isec.pos.y, isec.pos.z - r },
                   { isec.pos.x, isec.pos.y, isec.pos.z + r }, col);
        dd.AddLine({ isec.pos.x - r * 0.7f, isec.pos.y, isec.pos.z - r * 0.7f },
                   { isec.pos.x + r * 0.7f, isec.pos.y, isec.pos.z + r * 0.7f }, col);
        dd.AddLine({ isec.pos.x - r * 0.7f, isec.pos.y, isec.pos.z + r * 0.7f },
                   { isec.pos.x + r * 0.7f, isec.pos.y, isec.pos.z - r * 0.7f }, col);

        if (ii == m_hoverSnapIntersection)
        {
            const float rr = r * 1.5f;
            dd.AddLine({ isec.pos.x - rr, isec.pos.y, isec.pos.z }, { isec.pos.x + rr, isec.pos.y, isec.pos.z }, colorSnapCandidate);
            dd.AddLine({ isec.pos.x, isec.pos.y, isec.pos.z - rr }, { isec.pos.x, isec.pos.y, isec.pos.z + rr }, colorSnapCandidate);
        }
    }

    // Preview segment from cursor to last placed point
    if (m_hasCursorPos &&
        m_mode == EditorMode::PolylineDraw &&
        m_activeRoad >= 0 &&
        m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        const Road& road = m_network->roads[m_activeRoad];
        if (!road.points.empty())
            dd.AddLine(road.points.back().pos, m_cursorPos, colorCursor);
    }

    if (m_mode == EditorMode::PointEdit &&
        m_activeRoad >= 0 &&
        m_activePoint >= 0 &&
        m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        const Road& road = m_network->roads[m_activeRoad];
        if (m_activePoint < static_cast<int>(road.points.size()))
        {
            const RoadPoint& point = road.points[m_activePoint];
            const float axisLenX = ComputeGizmoAxisLength(point.pos, GizmoAxis::X, viewProj, vpW, vpH);
            const float axisLenY = ComputeGizmoAxisLength(point.pos, GizmoAxis::Y, viewProj, vpW, vpH);
            const float axisLenZ = ComputeGizmoAxisLength(point.pos, GizmoAxis::Z, viewProj, vpW, vpH);
            const XMFLOAT3 p = point.pos;
            dd.AddLine({ p.x - 0.15f, p.y, p.z }, { p.x + 0.15f, p.y, p.z }, colorPivot);
            dd.AddLine({ p.x, p.y - 0.15f, p.z }, { p.x, p.y + 0.15f, p.z }, colorPivot);
            dd.AddLine({ p.x, p.y, p.z - 0.15f }, { p.x, p.y, p.z + 0.15f }, colorPivot);
            dd.AddLine(p, { p.x + axisLenX, p.y, p.z }, colorAxisX);
            dd.AddLine(p, { p.x, p.y + axisLenY, p.z }, colorAxisY);
            dd.AddLine(p, { p.x, p.y, p.z + axisLenZ }, colorAxisZ);
        }
    }

    if (m_mode == EditorMode::IntersectionEdit &&
        m_activeIntersection >= 0 &&
        m_activeIntersection < static_cast<int>(m_network->intersections.size()))
    {
        const Intersection& isec = m_network->intersections[m_activeIntersection];
        const float axisLenX = ComputeGizmoAxisLength(isec.pos, GizmoAxis::X, viewProj, vpW, vpH);
        const float axisLenY = ComputeGizmoAxisLength(isec.pos, GizmoAxis::Y, viewProj, vpW, vpH);
        const float axisLenZ = ComputeGizmoAxisLength(isec.pos, GizmoAxis::Z, viewProj, vpW, vpH);
        const XMFLOAT3 p = isec.pos;
        dd.AddLine({ p.x - 0.15f, p.y, p.z }, { p.x + 0.15f, p.y, p.z }, colorPivot);
        dd.AddLine({ p.x, p.y - 0.15f, p.z }, { p.x, p.y + 0.15f, p.z }, colorPivot);
        dd.AddLine({ p.x, p.y, p.z - 0.15f }, { p.x, p.y, p.z + 0.15f }, colorPivot);
        dd.AddLine(p, { p.x + axisLenX, p.y, p.z }, colorAxisX);
        dd.AddLine(p, { p.x, p.y + axisLenY, p.z }, colorAxisY);
        dd.AddLine(p, { p.x, p.y, p.z + axisLenZ }, colorAxisZ);
    }
}

// Project a world-space point to screen pixels. Returns false if behind camera.
static bool WorldToScreen(XMFLOAT3 world, XMMATRIX viewProj,
                           int vpW, int vpH, ImVec2& out)
{
    XMVECTOR h = XMVector4Transform(
        XMVectorSet(world.x, world.y, world.z, 1.0f), viewProj);
    float w = XMVectorGetW(h);
    if (w <= 0.0f) return false;
    out.x = ( XMVectorGetX(h) / w * 0.5f + 0.5f) * vpW;
    out.y = (-XMVectorGetY(h) / w * 0.5f + 0.5f) * vpH;
    return true;
}

void PolylineEditor::DrawOverlay(XMMATRIX viewProj, int vpW, int vpH) const
{
    const_cast<PolylineEditor*>(this)->SanitizeSelection();

    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const float kRadius    = 4.0f;
    const ImU32 colPoint    = IM_COL32(255, 255, 255, 220);
    const ImU32 colSelected = IM_COL32(255,  80,  80, 255);
    const ImU32 colCursor   = IM_COL32( 60, 255, 110, 220);
    const ImU32 colAxisX    = IM_COL32(255,  80,  80, 255);
    const ImU32 colAxisY    = IM_COL32( 80, 255, 120, 255);
    const ImU32 colAxisZ    = IM_COL32( 80, 150, 255, 255);
    const ImU32 colIsec     = IM_COL32( 80, 240, 255, 255);
    const ImU32 colIsecSel  = IM_COL32(255, 120, 80, 255);
    const ImU32 colSnap     = IM_COL32(255, 240, 80, 255);

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        for (int pi = 0; pi < static_cast<int>(road.points.size()); ++pi)
        {
            ImVec2 sp;
            if (!WorldToScreen(road.points[pi].pos, viewProj, vpW, vpH, sp))
                continue;
            bool isActive = (ri == m_activeRoad && pi == m_activePoint);
            ImU32 col = isActive ? colSelected : colPoint;
            float r   = isActive ? kRadius + 2.0f : kRadius;
            dl->AddCircleFilled(sp, r, col, 20);
        }
    }

    for (int ii = 0; ii < static_cast<int>(m_network->intersections.size()); ++ii)
    {
        const Intersection& isec = m_network->intersections[ii];
        ImVec2 sp;
        if (!WorldToScreen(isec.pos, viewProj, vpW, vpH, sp))
            continue;
        const bool selected = (ii == m_activeIntersection);
        const ImU32 col = selected ? colIsecSel : colIsec;
        dl->AddCircle(sp, 10.0f, col, 24, 2.0f);
        dl->AddLine(ImVec2(sp.x - 8.0f, sp.y), ImVec2(sp.x + 8.0f, sp.y), col, 2.0f);
        dl->AddLine(ImVec2(sp.x, sp.y - 8.0f), ImVec2(sp.x, sp.y + 8.0f), col, 2.0f);
        dl->AddText(ImVec2(sp.x + 12.0f, sp.y - 8.0f), col, isec.name.c_str());
    }

    if (m_hoverSnapIntersection >= 0 &&
        m_hoverSnapIntersection < static_cast<int>(m_network->intersections.size()))
    {
        ImVec2 sp;
        if (WorldToScreen(m_network->intersections[m_hoverSnapIntersection].pos,
                          viewProj, vpW, vpH, sp))
        {
            dl->AddCircle(sp, 22.0f, colSnap, 32, 3.0f);
            dl->AddCircle(sp, 30.0f, IM_COL32(255, 240, 80, 120), 32, 2.0f);
            dl->AddText(ImVec2(sp.x + 18.0f, sp.y - 24.0f), colSnap, "SNAP");
        }
    }

    if (m_mode == EditorMode::PointEdit &&
        m_activeRoad >= 0 &&
        m_activePoint >= 0 &&
        m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        const Road& road = m_network->roads[m_activeRoad];
        if (m_activePoint < static_cast<int>(road.points.size()))
        {
            const RoadPoint& point = road.points[m_activePoint];
            const XMFLOAT3 p = point.pos;
            ImVec2 pivot;
            if (WorldToScreen(p, viewProj, vpW, vpH, pivot))
            {
                dl->AddCircle(pivot, 8.0f, IM_COL32(255,255,255,220), 24, 2.0f);
                dl->AddCircleFilled(pivot, 3.0f, IM_COL32(255,255,255,220), 16);

                struct AxisOverlay
                {
                    XMFLOAT3 end;
                    ImU32 color;
                    const char* label;
                };
                AxisOverlay axes[] =
                {
                    { { p.x + ComputeGizmoAxisLength(p, GizmoAxis::X, viewProj, vpW, vpH), p.y, p.z }, colAxisX, "X" },
                    { { p.x, p.y + ComputeGizmoAxisLength(p, GizmoAxis::Y, viewProj, vpW, vpH), p.z }, colAxisY, "Y" },
                    { { p.x, p.y, p.z + ComputeGizmoAxisLength(p, GizmoAxis::Z, viewProj, vpW, vpH) }, colAxisZ, "Z" },
                };

                for (const AxisOverlay& axis : axes)
                {
                    ImVec2 end;
                    if (!WorldToScreen(axis.end, viewProj, vpW, vpH, end))
                        continue;
                    dl->AddLine(pivot, end, axis.color, 2.0f);
                    dl->AddText(ImVec2(end.x + 4.0f, end.y - 8.0f), axis.color, axis.label);
                }
            }
        }
    }

    if (m_mode == EditorMode::IntersectionEdit &&
        m_activeIntersection >= 0 &&
        m_activeIntersection < static_cast<int>(m_network->intersections.size()))
    {
        const Intersection& isec = m_network->intersections[m_activeIntersection];
        const XMFLOAT3 p = isec.pos;
        ImVec2 pivot;
        if (WorldToScreen(p, viewProj, vpW, vpH, pivot))
        {
            dl->AddCircle(pivot, 8.0f, IM_COL32(255,255,255,220), 24, 2.0f);
            dl->AddCircleFilled(pivot, 3.0f, IM_COL32(255,255,255,220), 16);

            struct AxisOverlay
            {
                XMFLOAT3 end;
                ImU32 color;
                const char* label;
            };
            AxisOverlay axes[] =
            {
                { { p.x + ComputeGizmoAxisLength(p, GizmoAxis::X, viewProj, vpW, vpH), p.y, p.z }, colAxisX, "X" },
                { { p.x, p.y + ComputeGizmoAxisLength(p, GizmoAxis::Y, viewProj, vpW, vpH), p.z }, colAxisY, "Y" },
                { { p.x, p.y, p.z + ComputeGizmoAxisLength(p, GizmoAxis::Z, viewProj, vpW, vpH) }, colAxisZ, "Z" },
            };

            for (const AxisOverlay& axis : axes)
            {
                ImVec2 end;
                if (!WorldToScreen(axis.end, viewProj, vpW, vpH, end))
                    continue;
                dl->AddLine(pivot, end, axis.color, 2.0f);
                dl->AddText(ImVec2(end.x + 4.0f, end.y - 8.0f), axis.color, axis.label);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// DrawUI
// ---------------------------------------------------------------------------
void PolylineEditor::DrawUI(ID3D11Device* /*device*/)
{
    SanitizeSelection();

    ImGui::SetNextWindowPos(ImVec2(10, 360), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(320, 420), ImGuiCond_FirstUseEver);
    ImGui::Begin("Road Editor");

    // Mode toolbar
    {
        bool navActive  = (m_mode == EditorMode::Navigate);
        bool drawActive = (m_mode == EditorMode::PolylineDraw);
        bool editActive = (m_mode == EditorMode::PointEdit);
        bool isecDrawActive = (m_mode == EditorMode::IntersectionDraw);
        bool isecEditActive = (m_mode == EditorMode::IntersectionEdit);

        if (navActive)  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button("Navigate")) SetMode(EditorMode::Navigate);
        if (navActive)  ImGui::PopStyleColor();

        ImGui::SameLine();

        if (drawActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button("Draw Road")) StartNewRoad();
        if (drawActive) ImGui::PopStyleColor();

        ImGui::SameLine();

        if (editActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button("Edit Points")) SetMode(EditorMode::PointEdit);
        if (editActive) ImGui::PopStyleColor();

        ImGui::NewLine();

        if (isecDrawActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button("Draw Isec")) SetMode(EditorMode::IntersectionDraw);
        if (isecDrawActive) ImGui::PopStyleColor();

        ImGui::SameLine();

        if (isecEditActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button("Edit Isec")) SetMode(EditorMode::IntersectionEdit);
        if (isecEditActive) ImGui::PopStyleColor();
    }

    ImGui::Separator();

    // Mode hint
    switch (m_mode)
    {
    case EditorMode::Navigate:
        ImGui::TextDisabled("Camera navigation only");
        ImGui::TextDisabled("Click road: select road");
        ImGui::TextDisabled("Selected road points can be picked directly");
        ImGui::TextDisabled("Press W: show gizmo for selected point/intersection");
        break;
    case EditorMode::PolylineDraw:
        ImGui::TextColored(ImVec4(0.2f,1,0.4f,1), "Drawing: %s",
            (m_activeRoad >= 0 &&
             m_activeRoad < static_cast<int>(m_network->roads.size()))
            ? m_network->roads[m_activeRoad].name.c_str() : "");
        ImGui::TextDisabled("Left click: add point");
        ImGui::TextDisabled("Enter: confirm  Esc: cancel");
        ImGui::SliderFloat("Width", &m_defaultWidth, 0.5f, 20.0f);
        break;
    case EditorMode::PointEdit:
        ImGui::TextColored(ImVec4(1,0.8f,0.2f,1), "Edit Points");
        ImGui::TextDisabled("Click point: select");
        ImGui::TextDisabled("Drag center: pan in screen plane");
        ImGui::TextDisabled("Click X/Y/Z axis: move on that axis");
        ImGui::TextDisabled("Endpoint near intersection: snap/connect");
        ImGui::TextDisabled("Delete: remove point");
        break;
    case EditorMode::IntersectionDraw:
        ImGui::TextColored(ImVec4(0.2f,0.9f,1.0f,1), "Draw Intersections");
        ImGui::TextDisabled("Left click: place intersection");
        ImGui::TextDisabled("Esc: back to navigate");
        break;
    case EditorMode::IntersectionEdit:
        ImGui::TextColored(ImVec4(0.2f,0.9f,1.0f,1), "Edit Intersections");
        ImGui::TextDisabled("Click: select intersection");
        ImGui::TextDisabled("Delete: remove intersection");
        break;
    }

    ImGui::Separator();
    if (ImGui::Checkbox("Snap To Terrain", &m_snapToTerrain) &&
        m_snapToTerrain && m_terrain && m_terrain->IsReady())
    {
        if (m_activeRoad >= 0 &&
            m_activePoint >= 0 &&
            m_activeRoad < static_cast<int>(m_network->roads.size()))
        {
            Road& road = m_network->roads[m_activeRoad];
            if (m_activePoint < static_cast<int>(road.points.size()))
                road.points[m_activePoint].pos.y =
                    m_terrain->GetHeightAt(road.points[m_activePoint].pos.x,
                                           road.points[m_activePoint].pos.z);
        }

        if (m_activeIntersection >= 0 &&
            m_activeIntersection < static_cast<int>(m_network->intersections.size()))
        {
            Intersection& isec = m_network->intersections[m_activeIntersection];
            isec.pos.y = m_terrain->GetHeightAt(isec.pos.x, isec.pos.z);
            SyncRoadConnectionsForIntersection(m_activeIntersection);
        }
    }
    ImGui::TextDisabled(m_snapToTerrain
        ? "Center/XZ gizmo moves snap to terrain"
        : "Points and intersections move freely in 3D");
    ImGui::Separator();

    // Road list
    ImGui::Text("Roads (%d)", static_cast<int>(m_network->roads.size()));
    ImGui::BeginChild("RoadList", ImVec2(0, 110), true);
    for (int i = 0; i < static_cast<int>(m_network->roads.size()); ++i)
    {
        Road& road = m_network->roads[i];
        bool selected = (i == m_activeRoad);
        if (ImGui::Selectable(road.name.c_str(), selected))
        {
            m_activeRoad  = i;
            m_activePoint = -1;
        }
        if (selected && ImGui::BeginPopupContextItem())
        {
            if (ImGui::MenuItem("Delete Road"))
            {
                m_network->RemoveRoad(i);
                if (m_activeRoad >= static_cast<int>(m_network->roads.size()))
                    m_activeRoad = -1;
            }
            ImGui::EndPopup();
        }
    }
    ImGui::EndChild();

    ImGui::Text("Intersections (%d)", static_cast<int>(m_network->intersections.size()));
    ImGui::BeginChild("IntersectionList", ImVec2(0, 90), true);
    for (int i = 0; i < static_cast<int>(m_network->intersections.size()); ++i)
    {
        Intersection& isec = m_network->intersections[i];
        const bool selected = (i == m_activeIntersection);
        if (ImGui::Selectable(isec.name.c_str(), selected))
            m_activeIntersection = i;
        if (selected && ImGui::BeginPopupContextItem())
        {
            if (ImGui::MenuItem("Delete Intersection"))
            {
                m_network->RemoveIntersection(i);
                if (m_activeIntersection >= static_cast<int>(m_network->intersections.size()))
                    m_activeIntersection = -1;
            }
            ImGui::EndPopup();
        }
    }
    ImGui::EndChild();

    // Selected point info
    if (m_activeRoad  >= 0 &&
        m_activePoint >= 0 &&
        m_activeRoad  < static_cast<int>(m_network->roads.size()))
    {
        Road& road = m_network->roads[m_activeRoad];
        if (m_activePoint < static_cast<int>(road.points.size()))
        {
            RoadPoint& rp = road.points[m_activePoint];
            ImGui::Text("Point %d", m_activePoint);
            if (ImGui::InputFloat3("Pos", &rp.pos.x) &&
                m_snapToTerrain && m_terrain && m_terrain->IsReady())
            {
                rp.pos.y = m_terrain->GetHeightAt(rp.pos.x, rp.pos.z);
            }
            ImGui::SliderFloat("Width##pt", &rp.width, 0.5f, 20.0f);
            if (IsSelectedRoadEndpoint())
            {
                std::string connectionId;
                if (GetSelectedRoadConnectionId(connectionId))
                    ImGui::Text("Connected: %s", connectionId.c_str());
                else
                    ImGui::TextDisabled("Connected: none");

                if (ImGui::Button("Disconnect Endpoint"))
                {
                    ClearSelectedRoadConnection();
                    m_statusMessage = "Road endpoint disconnected";
                }
            }
        }
    }

    if (m_activeIntersection >= 0 &&
        m_activeIntersection < static_cast<int>(m_network->intersections.size()))
    {
        Intersection& isec = m_network->intersections[m_activeIntersection];
        char nameBuf[128] = {};
        strncpy_s(nameBuf, sizeof(nameBuf), isec.name.c_str(), _TRUNCATE);
        static const char* kIntersectionTypes[] = { "intersection", "roundabout" };
        int typeIndex = (isec.type == "roundabout") ? 1 : 0;
        ImGui::Text("Intersection");
        if (ImGui::InputText("Name##isec", nameBuf, sizeof(nameBuf)))
            isec.name = nameBuf;
        if (ImGui::Combo("Type##isec", &typeIndex, kIntersectionTypes, IM_ARRAYSIZE(kIntersectionTypes)))
            isec.type = kIntersectionTypes[typeIndex];
        if (ImGui::InputFloat3("Pos##isec", &isec.pos.x) &&
            m_snapToTerrain && m_terrain && m_terrain->IsReady())
        {
            isec.pos.y = m_terrain->GetHeightAt(isec.pos.x, isec.pos.z);
            SyncRoadConnectionsForIntersection(m_activeIntersection);
        }
        ImGui::SliderFloat("Radius##isec", &isec.radius, 1.0f, 20.0f);
    }

    ImGui::Separator();

    // Save / load
    ImGui::InputText("Path", m_filePath, sizeof(m_filePath));
    if (ImGui::Button("Save"))
    {
        if (!Save(m_filePath))
        {
            m_statusMessage = std::string("Road save failed: ") + m_filePath;
            ImGui::OpenPopup("SaveError");
        }
        else
        {
            m_statusMessage = std::string("Roads saved: ") + m_filePath;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Load"))
    {
        if (!Load(m_filePath))
        {
            m_statusMessage = std::string("Road load failed: ") + m_filePath;
            ImGui::OpenPopup("LoadError");
        }
        else
        {
            m_statusMessage = std::string("Roads loaded: ") + m_filePath;
        }
    }
    if (ImGui::BeginPopup("SaveError"))
    {
        ImGui::Text("Save failed: %s", m_filePath);
        if (ImGui::Button("OK")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    if (ImGui::BeginPopup("LoadError"))
    {
        ImGui::Text("Load failed: %s", m_filePath);
        if (ImGui::Button("OK")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    ImGui::End();
}
