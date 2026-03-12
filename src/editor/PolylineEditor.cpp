#include "PolylineEditor.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>

using namespace DirectX;

namespace
{
constexpr size_t kMaxUndoStates = 128;
constexpr int kPreviewCurveSubdivisions = 12;

float Distance3(XMFLOAT3 a, XMFLOAT3 b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

XMFLOAT3 Lerp3(XMFLOAT3 a, XMFLOAT3 b, float t)
{
    return
    {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    };
}

XMFLOAT4 Lerp4(XMFLOAT4 a, XMFLOAT4 b, float t)
{
    return
    {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
        a.w + (b.w - a.w) * t
    };
}

float DistanceXZ3(XMFLOAT3 a, XMFLOAT3 b)
{
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return sqrtf(dx * dx + dz * dz);
}

XMFLOAT4 PreviewGradeColor(float gradePercent, float redThresholdPercent)
{
    const XMFLOAT4 colorLow = { 0.45f, 0.95f, 0.95f, 0.9f };
    const XMFLOAT4 colorMid = { 1.0f, 0.88f, 0.20f, 0.95f };
    const XMFLOAT4 colorHigh = { 1.0f, 0.24f, 0.18f, 1.0f };
    const float safeThreshold = (std::max)(0.1f, redThresholdPercent);
    const float t = std::clamp(gradePercent / safeThreshold, 0.0f, 1.0f);
    if (t < 0.5f)
        return Lerp4(colorLow, colorMid, t * 2.0f);
    return Lerp4(colorMid, colorHigh, (t - 0.5f) * 2.0f);
}

XMFLOAT3 QuadraticBezier(XMFLOAT3 p0, XMFLOAT3 p1, XMFLOAT3 p2, float t)
{
    const float u = 1.0f - t;
    const float uu = u * u;
    const float tt = t * t;
    return
    {
        uu * p0.x + 2.0f * u * t * p1.x + tt * p2.x,
        uu * p0.y + 2.0f * u * t * p1.y + tt * p2.y,
        uu * p0.z + 2.0f * u * t * p1.z + tt * p2.z
    };
}

void AppendQuadraticBezierSamples(
    std::vector<XMFLOAT3>& samples,
    XMFLOAT3 p0, XMFLOAT3 p1, XMFLOAT3 p2,
    int subdivisions)
{
    for (int step = 1; step <= subdivisions; ++step)
    {
        const float t = static_cast<float>(step) / static_cast<float>(subdivisions);
        samples.push_back(QuadraticBezier(p0, p1, p2, t));
    }
}

std::vector<XMFLOAT3> BuildRoadPreviewCurve(const Road& road)
{
    std::vector<XMFLOAT3> samples;
    const int pointCount = static_cast<int>(road.points.size());
    if (pointCount < 2)
        return samples;

    if (pointCount == 2)
    {
        samples.push_back(road.points[0].pos);
        samples.push_back(road.points[1].pos);
        return samples;
    }

    const bool closed = road.closed && pointCount >= 3;
    const int edgeCount = closed ? pointCount : pointCount - 1;
    std::vector<XMFLOAT3> edgeMidpoints(edgeCount);

    for (int edgeIndex = 0; edgeIndex < edgeCount; ++edgeIndex)
    {
        const int startIndex = edgeIndex;
        const int endIndex = (edgeIndex + 1) % pointCount;
        float t = 0.5f;

        if (closed || (edgeIndex > 0 && edgeIndex < edgeCount - 1))
        {
            const int prevEdgeIndex = closed ? (edgeIndex - 1 + edgeCount) % edgeCount : edgeIndex - 1;
            const int nextEdgeIndex = closed ? (edgeIndex + 1) % edgeCount : edgeIndex + 1;
            const int prevStartIndex = prevEdgeIndex;
            const int prevEndIndex = (prevEdgeIndex + 1) % pointCount;
            const int nextStartIndex = nextEdgeIndex;
            const int nextEndIndex = (nextEdgeIndex + 1) % pointCount;
            const float prevLen = Distance3(
                road.points[prevStartIndex].pos,
                road.points[prevEndIndex].pos);
            const float nextLen = Distance3(
                road.points[nextStartIndex].pos,
                road.points[nextEndIndex].pos);
            const float sum = prevLen + nextLen;
            if (sum > 1e-5f)
                t = prevLen / sum;
        }

        edgeMidpoints[edgeIndex] = Lerp3(
            road.points[startIndex].pos,
            road.points[endIndex].pos,
            t);
    }

    if (closed)
    {
        samples.push_back(edgeMidpoints.back());
        for (int pointIndex = 0; pointIndex < pointCount; ++pointIndex)
        {
            const XMFLOAT3 p0 = edgeMidpoints[(pointIndex - 1 + edgeCount) % edgeCount];
            const XMFLOAT3 p1 = road.points[pointIndex].pos;
            const XMFLOAT3 p2 = edgeMidpoints[pointIndex];
            AppendQuadraticBezierSamples(samples, p0, p1, p2, kPreviewCurveSubdivisions);
        }
        samples.push_back(samples.front());
        return samples;
    }

    samples.push_back(road.points.front().pos);
    samples.push_back(edgeMidpoints.front());
    for (int pointIndex = 1; pointIndex < pointCount - 1; ++pointIndex)
    {
        const XMFLOAT3 p0 = edgeMidpoints[pointIndex - 1];
        const XMFLOAT3 p1 = road.points[pointIndex].pos;
        const XMFLOAT3 p2 = edgeMidpoints[pointIndex];
        AppendQuadraticBezierSamples(samples, p0, p1, p2, kPreviewCurveSubdivisions);
    }
    samples.push_back(road.points.back().pos);
    return samples;
}

float ComputePolylineLength(const std::vector<XMFLOAT3>& points)
{
    float length = 0.0f;
    for (size_t i = 1; i < points.size(); ++i)
        length += Distance3(points[i - 1], points[i]);
    return length;
}

float ComputeAverageAbsoluteGradePercent(const std::vector<XMFLOAT3>& points)
{
    float totalHorizontalDistance = 0.0f;
    float totalAbsoluteRise = 0.0f;
    for (size_t i = 1; i < points.size(); ++i)
    {
        const float dx = points[i].x - points[i - 1].x;
        const float dz = points[i].z - points[i - 1].z;
        const float horizontalDistance = sqrtf(dx * dx + dz * dz);
        if (horizontalDistance <= 1e-4f)
            continue;

        totalHorizontalDistance += horizontalDistance;
        totalAbsoluteRise += fabsf(points[i].y - points[i - 1].y);
    }

    if (totalHorizontalDistance <= 1e-4f)
        return 0.0f;

    return (totalAbsoluteRise / totalHorizontalDistance) * 100.0f;
}

float ComputeMaxAbsoluteGradePercent(const std::vector<XMFLOAT3>& points)
{
    float maxGradePercent = 0.0f;
    for (size_t i = 1; i < points.size(); ++i)
    {
        const float dx = points[i].x - points[i - 1].x;
        const float dz = points[i].z - points[i - 1].z;
        const float horizontalDistance = sqrtf(dx * dx + dz * dz);
        if (horizontalDistance <= 1e-4f)
            continue;

        const float gradePercent =
            (fabsf(points[i].y - points[i - 1].y) / horizontalDistance) * 100.0f;
        maxGradePercent = (std::max)(maxGradePercent, gradePercent);
    }

    return maxGradePercent;
}
}

bool PolylineEditor::ConsumeStatusMessage(std::string& outMessage)
{
    if (m_statusMessage.empty())
        return false;

    outMessage = m_statusMessage;
    m_statusMessage.clear();
    return true;
}

bool PolylineEditor::Load(const char* path)
{
    if (!m_network->LoadFromFile(path))
        return false;

    ClearHistory();
    SanitizeSelection();
    return true;
}

bool PolylineEditor::GetFocusTarget(XMFLOAT3& outTarget) const
{
    if (!m_network)
        return false;

    if (m_activeRoad >= 0 &&
        m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        const Road& road = m_network->roads[m_activeRoad];
        if (m_activePoint >= 0 &&
            m_activePoint < static_cast<int>(road.points.size()))
        {
            outTarget = road.points[m_activePoint].pos;
            return true;
        }

        if (!road.points.empty())
        {
            XMFLOAT3 center = { 0.0f, 0.0f, 0.0f };
            for (const RoadPoint& point : road.points)
            {
                center.x += point.pos.x;
                center.y += point.pos.y;
                center.z += point.pos.z;
            }

            const float invCount = 1.0f / static_cast<float>(road.points.size());
            outTarget = { center.x * invCount, center.y * invCount, center.z * invCount };
            return true;
        }
    }

    if (m_activeIntersection >= 0 &&
        m_activeIntersection < static_cast<int>(m_network->intersections.size()))
    {
        outTarget = m_network->intersections[m_activeIntersection].pos;
        return true;
    }

    return false;
}

bool PolylineEditor::GetPrimaryRoadForPathfinding(int& outRoadIndex) const
{
    if (!m_network)
        return false;

    if (m_selectedPoints.size() == 1)
    {
        const PointRef& pointRef = m_selectedPoints.front();
        if (pointRef.roadIndex >= 0 &&
            pointRef.roadIndex < static_cast<int>(m_network->roads.size()))
        {
            outRoadIndex = pointRef.roadIndex;
            return true;
        }
    }

    if (m_selectedRoads.size() == 1)
    {
        const int roadIndex = m_selectedRoads.front();
        if (roadIndex >= 0 && roadIndex < static_cast<int>(m_network->roads.size()))
        {
            outRoadIndex = roadIndex;
            return true;
        }
    }

    if (m_activeRoad >= 0 && m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        outRoadIndex = m_activeRoad;
        return true;
    }

    return false;
}

void PolylineEditor::CollectSelectedRoadIndices(std::vector<int>& outRoadIndices) const
{
    outRoadIndices.clear();
    if (!m_network)
        return;

    outRoadIndices = m_selectedRoads;
    for (const PointRef& pointRef : m_selectedPoints)
    {
        if (pointRef.roadIndex >= 0 &&
            pointRef.roadIndex < static_cast<int>(m_network->roads.size()))
        {
            outRoadIndices.push_back(pointRef.roadIndex);
        }
    }

    if (outRoadIndices.empty() &&
        m_activeRoad >= 0 &&
        m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        outRoadIndices.push_back(m_activeRoad);
    }

    std::sort(outRoadIndices.begin(), outRoadIndices.end());
    outRoadIndices.erase(
        std::unique(outRoadIndices.begin(), outRoadIndices.end()),
        outRoadIndices.end());
}

bool PolylineEditor::SelectAllPointsOnSelectedRoads()
{
    if (!m_network)
        return false;

    std::vector<int> roadIndices;
    CollectSelectedRoadIndices(roadIndices);
    if (roadIndices.empty())
        return false;

    std::vector<PointRef> pointRefs;
    for (int roadIndex : roadIndices)
    {
        if (roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
            continue;

        const Road& road = m_network->roads[roadIndex];
        for (int pointIndex = 0; pointIndex < static_cast<int>(road.points.size()); ++pointIndex)
            pointRefs.push_back({ roadIndex, pointIndex });
    }

    if (pointRefs.empty())
        return false;

    m_selectedPoints = std::move(pointRefs);
    m_activeRoad = m_selectedPoints.front().roadIndex;
    m_activePoint = m_selectedPoints.front().pointIndex;
    return true;
}

bool PolylineEditor::DisconnectSelectedRoadEndpoints()
{
    if (!m_network)
        return false;

    bool disconnectedAny = false;
    for (const PointRef& pointRef : m_selectedPoints)
    {
        if (pointRef.roadIndex < 0 ||
            pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
            continue;

        Road& road = m_network->roads[pointRef.roadIndex];
        if (road.points.empty() ||
            pointRef.pointIndex < 0 ||
            pointRef.pointIndex >= static_cast<int>(road.points.size()))
            continue;

        if (pointRef.pointIndex == 0 && !road.startIntersectionId.empty())
        {
            const int intersectionIndex = FindIntersectionIndexById(road.startIntersectionId);
            if (intersectionIndex < 0 || !IsIntersectionSelected(intersectionIndex))
            {
                road.startIntersectionId.clear();
                disconnectedAny = true;
            }
        }

        const int lastPointIndex = static_cast<int>(road.points.size()) - 1;
        if (pointRef.pointIndex == lastPointIndex && !road.endIntersectionId.empty())
        {
            const int intersectionIndex = FindIntersectionIndexById(road.endIntersectionId);
            if (intersectionIndex < 0 || !IsIntersectionSelected(intersectionIndex))
            {
                road.endIntersectionId.clear();
                disconnectedAny = true;
            }
        }
    }

    return disconnectedAny;
}

bool PolylineEditor::CopySelectedRoads()
{
    if (!m_network)
        return false;

    std::vector<int> roadIndices;
    CollectSelectedRoadIndices(roadIndices);
    if (roadIndices.empty())
    {
        m_statusMessage = "Select a road before copying";
        return false;
    }

    RoadClipboard clipboard;
    XMFLOAT3 anchor = { 0.0f, 0.0f, 0.0f };
    int pointCount = 0;
    for (int roadIndex : roadIndices)
    {
        if (roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
            continue;

        const Road& road = m_network->roads[roadIndex];
        clipboard.roads.push_back(road);
        for (const RoadPoint& point : road.points)
        {
            anchor.x += point.pos.x;
            anchor.y += point.pos.y;
            anchor.z += point.pos.z;
            ++pointCount;
        }
    }

    if (clipboard.roads.empty() || pointCount == 0)
    {
        m_statusMessage = "Selected road has no points to copy";
        return false;
    }

    const float invPointCount = 1.0f / static_cast<float>(pointCount);
    clipboard.anchor =
    {
        anchor.x * invPointCount,
        anchor.y * invPointCount,
        anchor.z * invPointCount
    };
    m_roadClipboard = std::move(clipboard);
    m_statusMessage =
        m_roadClipboard.roads.size() > 1 ? "Roads copied" : "Road copied";
    return true;
}

bool PolylineEditor::PasteCopiedRoadsAtCursor()
{
    if (!m_network)
        return false;
    if (m_roadClipboard.roads.empty())
    {
        m_statusMessage = "Copy a road before pasting";
        return false;
    }
    if (!m_hasCursorPos)
    {
        m_statusMessage = "Move the cursor over the ground before pasting";
        return false;
    }

    m_network->EnsureDefaultGroup();
    PushUndoState();

    const XMFLOAT3 targetAnchor = m_cursorPos;
    const XMFLOAT3 delta =
    {
        targetAnchor.x - m_roadClipboard.anchor.x,
        targetAnchor.y - m_roadClipboard.anchor.y,
        targetAnchor.z - m_roadClipboard.anchor.z
    };

    std::vector<int> pastedRoads;
    pastedRoads.reserve(m_roadClipboard.roads.size());

    for (const Road& copiedRoad : m_roadClipboard.roads)
    {
        const std::string pastedName = copiedRoad.name.empty()
            ? std::string("Road Copy")
            : copiedRoad.name + " Copy";
        const int newRoadIndex = m_network->AddRoad(pastedName);
        if (newRoadIndex < 0 || newRoadIndex >= static_cast<int>(m_network->roads.size()))
            continue;

        Road& newRoad = m_network->roads[newRoadIndex];
        newRoad.name = pastedName;
        newRoad.groupId =
            FindGroupIndexById(copiedRoad.groupId) >= 0
            ? copiedRoad.groupId
            : m_network->roads[newRoadIndex].groupId;
        newRoad.closed = copiedRoad.closed;
        newRoad.laneWidth = copiedRoad.laneWidth;
        newRoad.laneLeft = copiedRoad.laneLeft;
        newRoad.laneRight = copiedRoad.laneRight;
        newRoad.startIntersectionId.clear();
        newRoad.endIntersectionId.clear();
        newRoad.points.clear();
        newRoad.points.reserve(copiedRoad.points.size());

        for (const RoadPoint& copiedPoint : copiedRoad.points)
        {
            RoadPoint newPoint = copiedPoint;
            newPoint.pos.x += delta.x;
            newPoint.pos.z += delta.z;
            if (m_terrain && m_terrain->IsReady())
                newPoint.pos.y = m_terrain->GetHeightAt(newPoint.pos.x, newPoint.pos.z);
            else
                newPoint.pos.y += delta.y;
            newRoad.points.push_back(newPoint);
        }

        pastedRoads.push_back(newRoadIndex);
    }

    if (pastedRoads.empty())
    {
        m_statusMessage = "Paste failed";
        return false;
    }

    m_selectedRoads = pastedRoads;
    m_activeRoad = pastedRoads.front();
    m_activePoint = -1;
    m_selectedPoints.clear();
    m_selectedIntersections.clear();
    m_activeIntersection = -1;
    m_dragging = false;
    m_activeGizmoAxis = GizmoAxis::None;
    m_hoverSnapIntersection = -1;
    SetActiveGroupById(m_network->roads[m_activeRoad].groupId);
    m_statusMessage = pastedRoads.size() > 1 ? "Roads pasted" : "Road pasted";
    return true;
}

PolylineEditor::EditorSnapshot PolylineEditor::CaptureSnapshot() const
{
    EditorSnapshot snapshot;
    snapshot.network = *m_network;
    snapshot.mode = m_mode;
    snapshot.activeRoad = m_activeRoad;
    snapshot.activePoint = m_activePoint;
    snapshot.selectedRoads = m_selectedRoads;
    snapshot.selectedPoints = m_selectedPoints;
    snapshot.selectedIntersections = m_selectedIntersections;
    snapshot.activeGroup = m_activeGroup;
    snapshot.activeIntersection = m_activeIntersection;
    snapshot.hoverSnapIntersection = m_hoverSnapIntersection;
    snapshot.defaultWidth = m_defaultWidth;
    snapshot.snapToTerrain = m_snapToTerrain;
    snapshot.rotateYMode = m_rotateYMode;
    snapshot.scaleXZMode = m_scaleXZMode;
    return snapshot;
}

void PolylineEditor::RestoreSnapshot(const EditorSnapshot& snapshot)
{
    *m_network = snapshot.network;
    m_mode = snapshot.mode;
    m_activeRoad = snapshot.activeRoad;
    m_activePoint = snapshot.activePoint;
    m_selectedRoads = snapshot.selectedRoads;
    m_selectedPoints = snapshot.selectedPoints;
    m_selectedIntersections = snapshot.selectedIntersections;
    m_activeGroup = snapshot.activeGroup;
    m_activeIntersection = snapshot.activeIntersection;
    m_hoverSnapIntersection = snapshot.hoverSnapIntersection;
    m_defaultWidth = snapshot.defaultWidth;
    m_snapToTerrain = snapshot.snapToTerrain;
    m_rotateYMode = snapshot.rotateYMode;
    m_scaleXZMode = snapshot.scaleXZMode;

    m_dragging = false;
    m_activeGizmoAxis = GizmoAxis::None;
    m_pointDragStartPositions.clear();
    m_intersectionDragStartPositions.clear();
    m_marqueeSelecting = false;
    SanitizeSelection();
}

void PolylineEditor::PushUndoState()
{
    if (!m_network)
        return;

    if (m_undoStack.size() >= kMaxUndoStates)
        m_undoStack.erase(m_undoStack.begin());

    m_undoStack.push_back(CaptureSnapshot());
    m_redoStack.clear();
}

void PolylineEditor::Undo()
{
    if (m_undoStack.empty() || !m_network)
        return;

    m_redoStack.push_back(CaptureSnapshot());
    RestoreSnapshot(m_undoStack.back());
    m_undoStack.pop_back();
    m_statusMessage = "Undo";
}

void PolylineEditor::Redo()
{
    if (m_redoStack.empty() || !m_network)
        return;

    m_undoStack.push_back(CaptureSnapshot());
    RestoreSnapshot(m_redoStack.back());
    m_redoStack.pop_back();
    m_statusMessage = "Redo";
}

void PolylineEditor::ClearHistory()
{
    m_undoStack.clear();
    m_redoStack.clear();
}

void PolylineEditor::ResetState()
{
    m_mode = EditorMode::Navigate;
    m_activeRoad = -1;
    m_activePoint = -1;
    m_selectedRoads.clear();
    m_selectedPoints.clear();
    m_selectedIntersections.clear();
    m_activeGroup = -1;
    m_activeIntersection = -1;
    m_hoverSnapIntersection = -1;
    m_dragging = false;
    m_activeGizmoAxis = GizmoAxis::None;
    m_dragOffset = { 0, 0, 0 };
    m_axisDragStartPos = { 0, 0, 0 };
    m_axisDragStartMouse = { 0, 0 };
    m_planeDragStartHit = { 0, 0, 0 };
    m_planeDragNormal = { 0, 0, 1 };
    m_pointDragStartPositions.clear();
    m_intersectionDragStartPositions.clear();
    m_marqueeSelecting = false;
    m_marqueeStart = { 0, 0 };
    m_marqueeEnd = { 0, 0 };
    m_marqueeAdditive = false;
    m_marqueeSubtractive = false;
    m_hasCursorPos = false;
    m_cursorPos = { 0, 0, 0 };
    m_prevLButton = false;
    m_prevWKey = false;
    m_prevEKey = false;
    m_prevRKey = false;
    m_prevVKey = false;
    m_prevUndoShortcut = false;
    m_prevRedoShortcut = false;
    m_prevCopyShortcut = false;
    m_prevPasteShortcut = false;
    m_rotateYMode = false;
    m_scaleXZMode = false;
    m_defaultWidth = 3.0f;
    m_snapToTerrain = true;
    m_statusMessage.clear();
    ClearHistory();
}

void PolylineEditor::SanitizeSelection()
{
    if (m_activeGroup < 0 || m_activeGroup >= static_cast<int>(m_network->groups.size()))
        m_activeGroup = m_network->groups.empty() ? -1 : 0;

    m_selectedRoads.erase(
        std::remove_if(
            m_selectedRoads.begin(),
            m_selectedRoads.end(),
            [this](int roadIndex)
            {
                return roadIndex < 0 ||
                    roadIndex >= static_cast<int>(m_network->roads.size());
            }),
        m_selectedRoads.end());

    m_selectedPoints.erase(
        std::remove_if(
            m_selectedPoints.begin(),
            m_selectedPoints.end(),
            [this](const PointRef& pointRef)
            {
                if (pointRef.roadIndex < 0 ||
                    pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                    return true;

                const int pointCount = static_cast<int>(
                    m_network->roads[pointRef.roadIndex].points.size());
                return pointRef.pointIndex < 0 || pointRef.pointIndex >= pointCount;
            }),
        m_selectedPoints.end());

    PointRef primaryPoint;
    if (GetPrimarySelectedPoint(primaryPoint))
    {
        m_activeRoad = primaryPoint.roadIndex;
        m_activePoint = primaryPoint.pointIndex;
    }
    else
    {
        m_activePoint = -1;
        if (m_activeRoad < 0 || m_activeRoad >= static_cast<int>(m_network->roads.size()) ||
            !IsRoadSelected(m_activeRoad))
        {
            m_activeRoad = m_selectedRoads.empty() ? -1 : m_selectedRoads.front();
        }
    }

    if (m_activeIntersection < 0 ||
        m_activeIntersection >= static_cast<int>(m_network->intersections.size()))
    {
        m_activeIntersection = -1;
    }
    m_selectedIntersections.erase(
        std::remove_if(
            m_selectedIntersections.begin(),
            m_selectedIntersections.end(),
            [this](int intersectionIndex)
            {
                return intersectionIndex < 0 ||
                    intersectionIndex >= static_cast<int>(m_network->intersections.size());
            }),
        m_selectedIntersections.end());
    if (m_activeIntersection >= 0 && !IsIntersectionSelected(m_activeIntersection))
        m_activeIntersection = m_selectedIntersections.empty() ? -1 : m_selectedIntersections.front();

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

static float NormalizeAngleDelta(float angle)
{
    while (angle > XM_PI)
        angle -= XM_2PI;
    while (angle < -XM_PI)
        angle += XM_2PI;
    return angle;
}

static XMFLOAT3 RotateAroundY(XMFLOAT3 point, XMFLOAT3 pivot, float radians)
{
    const float dx = point.x - pivot.x;
    const float dz = point.z - pivot.z;
    const float c = cosf(radians);
    const float s = sinf(radians);
    return
    {
        pivot.x + dx * c - dz * s,
        point.y,
        pivot.z + dx * s + dz * c
    };
}

static XMFLOAT3 ScaleAroundXZ(XMFLOAT3 point, XMFLOAT3 pivot, float scale)
{
    return
    {
        pivot.x + (point.x - pivot.x) * scale,
        point.y,
        pivot.z + (point.z - pivot.z) * scale
    };
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
        if (!IsRoadVisible(road))
            continue;
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

int PolylineEditor::FindNearestSegmentOnRoad(
    int roadIndex, int vpW, int vpH, XMFLOAT2 px, XMMATRIX viewProj) const
{
    if (roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return -1;

    const Road& road = m_network->roads[roadIndex];
    const float threshold = 10.0f;
    float bestDist = threshold;
    int bestSegment = -1;

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
            bestSegment = p;
        }
    }

    return bestSegment;
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
        if (!IsRoadVisible(road))
            continue;
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
        if (!IsIntersectionVisible(m_network->intersections[i]))
            continue;
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

int PolylineEditor::FindGroupIndexById(const std::string& id) const
{
    return m_network ? m_network->FindGroupIndexById(id) : -1;
}

bool PolylineEditor::IsRoadVisible(const Road& road) const
{
    const RoadGroup* group = m_network->FindGroupById(road.groupId);
    return group == nullptr || group->visible;
}

bool PolylineEditor::IsIntersectionVisible(const Intersection& intersection) const
{
    const RoadGroup* group = m_network->FindGroupById(intersection.groupId);
    return group == nullptr || group->visible;
}

void PolylineEditor::SetActiveGroupById(const std::string& id)
{
    m_activeGroup = FindGroupIndexById(id);
    if (m_activeGroup < 0 && !m_network->groups.empty())
        m_activeGroup = 0;
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
    if (m_selectedPoints.size() != 1)
        return false;

    PointRef selectedPoint;
    if (!GetPrimarySelectedPoint(selectedPoint))
        return false;

    const Road& road = m_network->roads[selectedPoint.roadIndex];
    const int last = static_cast<int>(road.points.size()) - 1;
    return selectedPoint.pointIndex == 0 || selectedPoint.pointIndex == last;
}

bool PolylineEditor::GetSelectedRoadConnectionId(std::string& outId) const
{
    if (!IsSelectedRoadEndpoint())
        return false;

    PointRef selectedPoint;
    if (!GetPrimarySelectedPoint(selectedPoint))
        return false;

    const Road& road = m_network->roads[selectedPoint.roadIndex];
    outId = (selectedPoint.pointIndex == 0) ? road.startIntersectionId : road.endIntersectionId;
    return !outId.empty();
}

void PolylineEditor::SetSelectedRoadConnectionId(const std::string& intersectionId)
{
    if (!IsSelectedRoadEndpoint())
        return;

    PointRef selectedPoint;
    if (!GetPrimarySelectedPoint(selectedPoint))
        return;

    Road& road = m_network->roads[selectedPoint.roadIndex];
    if (selectedPoint.pointIndex == 0)
        road.startIntersectionId = intersectionId;
    else
        road.endIntersectionId = intersectionId;
}

void PolylineEditor::ClearSelectedRoadConnection()
{
    SetSelectedRoadConnectionId("");
}

bool PolylineEditor::SplitSelectedRoadAtPoint()
{
    if (!m_network || m_selectedPoints.size() != 1)
        return false;

    PointRef selectedPoint;
    if (!GetPrimarySelectedPoint(selectedPoint))
        return false;
    if (selectedPoint.roadIndex < 0 ||
        selectedPoint.roadIndex >= static_cast<int>(m_network->roads.size()))
        return false;

    const int sourceRoadIndex = selectedPoint.roadIndex;
    const Road& sourceRoad = m_network->roads[sourceRoadIndex];
    const int splitIndex = selectedPoint.pointIndex;
    if (splitIndex <= 0 || splitIndex >= static_cast<int>(sourceRoad.points.size()) - 1)
    {
        m_statusMessage = "Select a middle point to split the road";
        return false;
    }

    PushUndoState();

    const XMFLOAT3 splitPos = sourceRoad.points[splitIndex].pos;
    const std::string originalStartIntersectionId = sourceRoad.startIntersectionId;
    const std::string originalEndIntersectionId = sourceRoad.endIntersectionId;
    const std::string originalName = sourceRoad.name;
    const std::string originalGroupId = sourceRoad.groupId;
    const bool originalClosed = sourceRoad.closed;
    const float originalLaneWidth = sourceRoad.laneWidth;
    const int originalLaneLeft = sourceRoad.laneLeft;
    const int originalLaneRight = sourceRoad.laneRight;
    const std::vector<RoadPoint> originalPoints = sourceRoad.points;

    const int newIntersectionIndex = m_network->AddIntersection(splitPos, "Intersection");
    if (newIntersectionIndex < 0 ||
        newIntersectionIndex >= static_cast<int>(m_network->intersections.size()))
        return false;

    Intersection& splitIntersection = m_network->intersections[newIntersectionIndex];
    splitIntersection.groupId = originalGroupId;
    splitIntersection.pos = splitPos;

    const std::string splitIntersectionId = splitIntersection.id;

    const int newRoadIndex = m_network->AddRoad(originalName + " B");
    if (newRoadIndex < 0 || newRoadIndex >= static_cast<int>(m_network->roads.size()))
        return false;

    Road& newRoad = m_network->roads[newRoadIndex];
    newRoad.groupId = originalGroupId;
    newRoad.closed = false;
    newRoad.startIntersectionId = splitIntersectionId;
    newRoad.endIntersectionId = originalEndIntersectionId;
    newRoad.laneWidth = originalLaneWidth;
    newRoad.laneLeft = originalLaneLeft;
    newRoad.laneRight = originalLaneRight;
    newRoad.points.assign(originalPoints.begin() + splitIndex, originalPoints.end());
    newRoad.points.front().pos = splitPos;

    Road& updatedSourceRoad = m_network->roads[sourceRoadIndex];
    updatedSourceRoad.groupId = originalGroupId;
    updatedSourceRoad.name = originalName + " A";
    updatedSourceRoad.startIntersectionId = originalStartIntersectionId;
    updatedSourceRoad.closed = false;
    updatedSourceRoad.endIntersectionId = splitIntersectionId;
    updatedSourceRoad.laneWidth = originalLaneWidth;
    updatedSourceRoad.laneLeft = originalLaneLeft;
    updatedSourceRoad.laneRight = originalLaneRight;
    updatedSourceRoad.points.assign(originalPoints.begin(), originalPoints.begin() + splitIndex + 1);
    updatedSourceRoad.points.back().pos = splitPos;

    m_activeRoad = sourceRoadIndex;
    ClearPointSelection();
    SelectSingleIntersection(newIntersectionIndex);
    m_activeGizmoAxis = GizmoAxis::None;
    m_hoverSnapIntersection = -1;
    m_dragging = false;
    m_statusMessage = originalClosed
        ? "Closed road split into two roads"
        : "Road split with new intersection";
    return true;
}

bool PolylineEditor::MergeSelectedRoads()
{
    constexpr float kMergeEndpointDistance = 5.0f;

    if (!m_network || m_selectedRoads.size() != 2)
        return false;

    const int roadAIndex = m_selectedRoads[0];
    const int roadBIndex = m_selectedRoads[1];
    if (roadAIndex < 0 || roadAIndex >= static_cast<int>(m_network->roads.size()) ||
        roadBIndex < 0 || roadBIndex >= static_cast<int>(m_network->roads.size()) ||
        roadAIndex == roadBIndex)
        return false;

    const Road& roadA = m_network->roads[roadAIndex];
    const Road& roadB = m_network->roads[roadBIndex];
    if (roadA.points.size() < 2 || roadB.points.size() < 2)
    {
        m_statusMessage = "Both roads need at least two points";
        return false;
    }

    auto endpointsMatch = [kMergeEndpointDistance](const std::string& lhsId,
                                                   const std::string& rhsId,
                                                   XMFLOAT3 lhsPos,
                                                   XMFLOAT3 rhsPos)
    {
        if (!lhsId.empty() && !rhsId.empty() && lhsId == rhsId)
            return true;
        return Distance3(lhsPos, rhsPos) <= kMergeEndpointDistance;
    };

    bool reverseA = false;
    bool reverseB = false;
    bool foundConnection = false;
    XMFLOAT3 mergePoint = {};
    if (endpointsMatch(roadA.endIntersectionId, roadB.startIntersectionId, roadA.points.back().pos, roadB.points.front().pos))
    {
        reverseA = false;
        reverseB = false;
        foundConnection = true;
        mergePoint =
        {
            0.5f * (roadA.points.back().pos.x + roadB.points.front().pos.x),
            0.5f * (roadA.points.back().pos.y + roadB.points.front().pos.y),
            0.5f * (roadA.points.back().pos.z + roadB.points.front().pos.z)
        };
    }
    else if (endpointsMatch(roadA.endIntersectionId, roadB.endIntersectionId, roadA.points.back().pos, roadB.points.back().pos))
    {
        reverseA = false;
        reverseB = true;
        foundConnection = true;
        mergePoint =
        {
            0.5f * (roadA.points.back().pos.x + roadB.points.back().pos.x),
            0.5f * (roadA.points.back().pos.y + roadB.points.back().pos.y),
            0.5f * (roadA.points.back().pos.z + roadB.points.back().pos.z)
        };
    }
    else if (endpointsMatch(roadA.startIntersectionId, roadB.startIntersectionId, roadA.points.front().pos, roadB.points.front().pos))
    {
        reverseA = true;
        reverseB = false;
        foundConnection = true;
        mergePoint =
        {
            0.5f * (roadA.points.front().pos.x + roadB.points.front().pos.x),
            0.5f * (roadA.points.front().pos.y + roadB.points.front().pos.y),
            0.5f * (roadA.points.front().pos.z + roadB.points.front().pos.z)
        };
    }
    else if (endpointsMatch(roadA.startIntersectionId, roadB.endIntersectionId, roadA.points.front().pos, roadB.points.back().pos))
    {
        reverseA = true;
        reverseB = true;
        foundConnection = true;
        mergePoint =
        {
            0.5f * (roadA.points.front().pos.x + roadB.points.back().pos.x),
            0.5f * (roadA.points.front().pos.y + roadB.points.back().pos.y),
            0.5f * (roadA.points.front().pos.z + roadB.points.back().pos.z)
        };
    }

    if (!foundConnection)
    {
        m_statusMessage = "Selected roads need endpoints within 5m to merge";
        return false;
    }

    PushUndoState();

    std::vector<RoadPoint> pointsA = roadA.points;
    std::vector<RoadPoint> pointsB = roadB.points;
    if (reverseA)
        std::reverse(pointsA.begin(), pointsA.end());
    if (reverseB)
        std::reverse(pointsB.begin(), pointsB.end());
    pointsA.back().pos = mergePoint;
    pointsB.front().pos = mergePoint;

    Road mergedRoad = roadA;
    mergedRoad.points = pointsA;
    mergedRoad.points.insert(mergedRoad.points.end(), pointsB.begin() + 1, pointsB.end());
    mergedRoad.startIntersectionId = reverseA ? roadA.endIntersectionId : roadA.startIntersectionId;
    mergedRoad.endIntersectionId = reverseB ? roadB.startIntersectionId : roadB.endIntersectionId;
    mergedRoad.closed = false;

    const int keepIndex = (std::min)(roadAIndex, roadBIndex);
    const int removeIndex = (std::max)(roadAIndex, roadBIndex);
    m_network->roads[keepIndex] = mergedRoad;
    m_network->RemoveRoad(removeIndex);

    SelectSingleRoad(keepIndex);
    ClearPointSelection();
    ClearIntersectionSelection();
    m_activeIntersection = -1;
    m_statusMessage = "Roads merged";
    return true;
}

int PolylineEditor::FindSnapIntersectionForSelectedEndpoint(
    int vpW, int vpH, XMMATRIX viewProj) const
{
    if (!IsSelectedRoadEndpoint())
        return -1;

    PointRef selectedPoint;
    if (!GetPrimarySelectedPoint(selectedPoint))
        return -1;

    const Road& road = m_network->roads[selectedPoint.roadIndex];
    const XMFLOAT3 pointPos = road.points[selectedPoint.pointIndex].pos;
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

    PointRef selectedPoint;
    if (!GetPrimarySelectedPoint(selectedPoint))
        return;

    Road& road = m_network->roads[selectedPoint.roadIndex];
    road.points[selectedPoint.pointIndex].pos = m_network->intersections[intersectionIndex].pos;
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
        (!m_selectedPoints.empty() || !m_selectedIntersections.empty()))
    {
        XMFLOAT3 center = { 0.0f, 0.0f, 0.0f };
        int validCount = 0;
        for (const PointRef& pointRef : m_selectedPoints)
        {
            if (pointRef.roadIndex < 0 ||
                pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                continue;

            const Road& road = m_network->roads[pointRef.roadIndex];
            if (pointRef.pointIndex < 0 || pointRef.pointIndex >= static_cast<int>(road.points.size()))
                continue;

            const XMFLOAT3& pos = road.points[pointRef.pointIndex].pos;
            center.x += pos.x;
            center.y += pos.y;
            center.z += pos.z;
            ++validCount;
        }
        for (int intersectionIndex : m_selectedIntersections)
        {
            if (intersectionIndex < 0 ||
                intersectionIndex >= static_cast<int>(m_network->intersections.size()))
                continue;
            const XMFLOAT3& pos = m_network->intersections[intersectionIndex].pos;
            center.x += pos.x;
            center.y += pos.y;
            center.z += pos.z;
            ++validCount;
        }
        if (validCount > 0)
        {
            const float invCount = 1.0f / static_cast<float>(validCount);
            outPivot = { center.x * invCount, center.y * invCount, center.z * invCount };
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

bool PolylineEditor::IsRoadSelected(int roadIndex) const
{
    return std::find(
        m_selectedRoads.begin(),
        m_selectedRoads.end(),
        roadIndex) != m_selectedRoads.end();
}

bool PolylineEditor::IsPointSelected(int roadIndex, int pointIndex) const
{
    return std::find_if(
        m_selectedPoints.begin(),
        m_selectedPoints.end(),
        [roadIndex, pointIndex](const PointRef& pointRef)
        {
            return pointRef.roadIndex == roadIndex && pointRef.pointIndex == pointIndex;
        }) != m_selectedPoints.end();
}

bool PolylineEditor::GetPrimarySelectedPoint(PointRef& outPoint) const
{
    if (m_selectedPoints.empty())
        return false;

    outPoint = m_selectedPoints.front();
    return true;
}

bool PolylineEditor::IsIntersectionSelected(int intersectionIndex) const
{
    return std::find(
        m_selectedIntersections.begin(),
        m_selectedIntersections.end(),
        intersectionIndex) != m_selectedIntersections.end();
}

void PolylineEditor::ClearRoadSelection()
{
    m_selectedRoads.clear();
    m_activeRoad = -1;
}

void PolylineEditor::SelectSingleRoad(int roadIndex)
{
    m_selectedRoads.clear();
    if (roadIndex >= 0)
        m_selectedRoads.push_back(roadIndex);
    m_activeRoad = roadIndex;
    m_activePoint = -1;
}

void PolylineEditor::ToggleRoadSelection(int roadIndex)
{
    if (roadIndex < 0)
        return;

    std::vector<int>::iterator it = std::find(
        m_selectedRoads.begin(),
        m_selectedRoads.end(),
        roadIndex);
    if (it != m_selectedRoads.end())
    {
        m_selectedRoads.erase(it);
        m_activeRoad = m_selectedRoads.empty() ? -1 : m_selectedRoads.front();
        return;
    }

    m_selectedRoads.push_back(roadIndex);
    m_activeRoad = roadIndex;
    m_activePoint = -1;
}

void PolylineEditor::ClearPointSelection()
{
    m_selectedPoints.clear();
    m_activePoint = -1;
}

void PolylineEditor::ClearIntersectionSelection()
{
    m_selectedIntersections.clear();
    m_activeIntersection = -1;
}

void PolylineEditor::SelectSinglePoint(int roadIndex, int pointIndex)
{
    m_selectedPoints.clear();
    if (roadIndex >= 0 && pointIndex >= 0)
        m_selectedPoints.push_back({ roadIndex, pointIndex });
    m_activeRoad = roadIndex;
    m_activePoint = pointIndex;
}

void PolylineEditor::TogglePointSelection(int roadIndex, int pointIndex)
{
    if (roadIndex < 0 || pointIndex < 0)
        return;

    std::vector<PointRef>::iterator it = std::find_if(
        m_selectedPoints.begin(),
        m_selectedPoints.end(),
        [roadIndex, pointIndex](const PointRef& pointRef)
        {
            return pointRef.roadIndex == roadIndex && pointRef.pointIndex == pointIndex;
        });
    if (it != m_selectedPoints.end())
    {
        m_selectedPoints.erase(it);
        PointRef primaryPoint;
        if (GetPrimarySelectedPoint(primaryPoint))
        {
            m_activeRoad = primaryPoint.roadIndex;
            m_activePoint = primaryPoint.pointIndex;
        }
        else
        {
            m_activePoint = -1;
        }
        return;
    }

    m_selectedPoints.push_back({ roadIndex, pointIndex });
    m_activeRoad = roadIndex;
    m_activePoint = pointIndex;
}

void PolylineEditor::SelectSingleIntersection(int intersectionIndex)
{
    m_selectedIntersections.clear();
    if (intersectionIndex >= 0)
        m_selectedIntersections.push_back(intersectionIndex);
    m_activeIntersection = intersectionIndex;
}

void PolylineEditor::ToggleIntersectionSelection(int intersectionIndex)
{
    if (intersectionIndex < 0)
        return;

    std::vector<int>::iterator it = std::find(
        m_selectedIntersections.begin(),
        m_selectedIntersections.end(),
        intersectionIndex);
    if (it != m_selectedIntersections.end())
    {
        m_selectedIntersections.erase(it);
        m_activeIntersection = m_selectedIntersections.empty() ? -1 : m_selectedIntersections.front();
        return;
    }

    m_selectedIntersections.push_back(intersectionIndex);
    m_activeIntersection = intersectionIndex;
}

void PolylineEditor::ApplyMarqueeSelection(
    int vpW, int vpH, XMMATRIX viewProj, bool addToSelection, bool removeFromSelection)
{
    const float minX = (std::min)(m_marqueeStart.x, m_marqueeEnd.x);
    const float maxX = (std::max)(m_marqueeStart.x, m_marqueeEnd.x);
    const float minY = (std::min)(m_marqueeStart.y, m_marqueeEnd.y);
    const float maxY = (std::max)(m_marqueeStart.y, m_marqueeEnd.y);

    if (!addToSelection && !removeFromSelection)
        m_selectedPoints.clear();

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadVisible(road))
            continue;

        for (int pi = 0; pi < static_cast<int>(road.points.size()); ++pi)
        {
            const XMFLOAT2 pointScreen = WorldToScreen(road.points[pi].pos, viewProj, vpW, vpH);
            if (pointScreen.x < 0.0f)
                continue;
            if (pointScreen.x < minX || pointScreen.x > maxX ||
                pointScreen.y < minY || pointScreen.y > maxY)
                continue;

            if (removeFromSelection)
            {
                auto it = std::remove_if(
                    m_selectedPoints.begin(),
                    m_selectedPoints.end(),
                    [ri, pi](const PointRef& pointRef)
                    {
                        return pointRef.roadIndex == ri && pointRef.pointIndex == pi;
                    });
                m_selectedPoints.erase(it, m_selectedPoints.end());
            }
            else if (!IsPointSelected(ri, pi))
            {
                m_selectedPoints.push_back({ ri, pi });
            }
        }
    }

    for (int ii = 0; ii < static_cast<int>(m_network->intersections.size()); ++ii)
    {
        const Intersection& isec = m_network->intersections[ii];
        if (!IsIntersectionVisible(isec))
            continue;

        const XMFLOAT2 pointScreen = WorldToScreen(isec.pos, viewProj, vpW, vpH);
        if (pointScreen.x < 0.0f)
            continue;
        if (pointScreen.x < minX || pointScreen.x > maxX ||
            pointScreen.y < minY || pointScreen.y > maxY)
            continue;

        if (removeFromSelection)
        {
            auto it = std::remove(
                m_selectedIntersections.begin(),
                m_selectedIntersections.end(),
                ii);
            m_selectedIntersections.erase(it, m_selectedIntersections.end());
        }
        else if (!IsIntersectionSelected(ii))
        {
            m_selectedIntersections.push_back(ii);
        }
    }

    PointRef primaryPoint;
    if (GetPrimarySelectedPoint(primaryPoint))
    {
        m_activeRoad = primaryPoint.roadIndex;
        m_activePoint = primaryPoint.pointIndex;
    }
    else
    {
        m_activeRoad = -1;
        m_activePoint = -1;
    }
    if (!m_selectedIntersections.empty())
        m_activeIntersection = m_selectedIntersections.front();
    else if (m_selectedPoints.empty())
        m_activeIntersection = -1;
}

PolylineEditor::GizmoAxis PolylineEditor::PickGizmoAxis(
    int vpW, int vpH, XMFLOAT2 px, XMMATRIX viewProj) const
{
    XMFLOAT3 pivotPos;
    if (!GetActiveGizmoPivot(pivotPos))
        return GizmoAxis::None;

    XMFLOAT2 pivotScreen = WorldToScreen(pivotPos, viewProj, vpW, vpH);
    if (pivotScreen.x < 0.0f)
        return GizmoAxis::None;

    if (m_scaleXZMode)
    {
        const float worldRadius = ComputeScaleGizmoRadius(pivotPos, viewProj, vpW, vpH);
        constexpr int kSegments = 4;
        constexpr float kThreshold = 10.0f;
        float bestDist = kThreshold;
        XMFLOAT3 corners[kSegments + 1] =
        {
            { pivotPos.x + worldRadius, pivotPos.y, pivotPos.z + worldRadius },
            { pivotPos.x - worldRadius, pivotPos.y, pivotPos.z + worldRadius },
            { pivotPos.x - worldRadius, pivotPos.y, pivotPos.z - worldRadius },
            { pivotPos.x + worldRadius, pivotPos.y, pivotPos.z - worldRadius },
            { pivotPos.x + worldRadius, pivotPos.y, pivotPos.z + worldRadius }
        };
        for (int i = 0; i < kSegments; ++i)
        {
            const XMFLOAT2 s0 = WorldToScreen(corners[i], viewProj, vpW, vpH);
            const XMFLOAT2 s1 = WorldToScreen(corners[i + 1], viewProj, vpW, vpH);
            if (s0.x < 0.0f || s1.x < 0.0f)
                continue;
            bestDist = (std::min)(bestDist, DistPointToSegment2D(px, s0, s1));
        }
        return bestDist < kThreshold ? GizmoAxis::ScaleXZ : GizmoAxis::None;
    }

    if (m_rotateYMode)
    {
        const float worldRadius = ComputeRotationGizmoRadius(pivotPos, viewProj, vpW, vpH);
        constexpr int kSegments = 48;
        constexpr float kThreshold = 10.0f;
        float bestDist = kThreshold;
        for (int segmentIndex = 0; segmentIndex < kSegments; ++segmentIndex)
        {
            const float t0 = XM_2PI * static_cast<float>(segmentIndex) / static_cast<float>(kSegments);
            const float t1 = XM_2PI * static_cast<float>(segmentIndex + 1) / static_cast<float>(kSegments);
            const XMFLOAT3 p0 =
            {
                pivotPos.x + cosf(t0) * worldRadius,
                pivotPos.y,
                pivotPos.z + sinf(t0) * worldRadius
            };
            const XMFLOAT3 p1 =
            {
                pivotPos.x + cosf(t1) * worldRadius,
                pivotPos.y,
                pivotPos.z + sinf(t1) * worldRadius
            };
            const XMFLOAT2 s0 = WorldToScreen(p0, viewProj, vpW, vpH);
            const XMFLOAT2 s1 = WorldToScreen(p1, viewProj, vpW, vpH);
            if (s0.x < 0.0f || s1.x < 0.0f)
                continue;

            bestDist = (std::min)(bestDist, DistPointToSegment2D(px, s0, s1));
        }
        return bestDist < kThreshold ? GizmoAxis::RotateY : GizmoAxis::None;
    }

    const float threshold = 10.0f;

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

float PolylineEditor::ComputeRotationGizmoRadius(
    XMFLOAT3 pivot, XMMATRIX viewProj, int vpW, int vpH) const
{
    const float radiusX = ComputeGizmoAxisLength(pivot, GizmoAxis::X, viewProj, vpW, vpH);
    const float radiusZ = ComputeGizmoAxisLength(pivot, GizmoAxis::Z, viewProj, vpW, vpH);
    return (std::max)(0.5f, 0.8f * (radiusX + radiusZ) * 0.5f);
}

float PolylineEditor::ComputeScaleGizmoRadius(
    XMFLOAT3 pivot, XMMATRIX viewProj, int vpW, int vpH) const
{
    const float radiusX = ComputeGizmoAxisLength(pivot, GizmoAxis::X, viewProj, vpW, vpH);
    const float radiusZ = ComputeGizmoAxisLength(pivot, GizmoAxis::Z, viewProj, vpW, vpH);
    return (std::max)(0.5f, 0.55f * (radiusX + radiusZ) * 0.5f);
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
    if (mode == EditorMode::PointEdit)
    {
        if (m_selectedPoints.empty() && m_selectedIntersections.empty())
            SelectAllPointsOnSelectedRoads();
    }
    else
    {
        m_rotateYMode = false;
        m_scaleXZMode = false;
        ClearPointSelection();
        m_dragging    = false;
        m_activeGizmoAxis = GizmoAxis::None;
        m_pointDragStartPositions.clear();
        m_intersectionDragStartPositions.clear();
    }
    if (mode != EditorMode::IntersectionEdit)
        m_activeIntersection = -1;
}

void PolylineEditor::StartNewRoad()
{
    m_network->EnsureDefaultGroup();
    PushUndoState();
    m_activeRoad  = m_network->AddRoad("Road " +
        std::to_string(m_network->roads.size()));
    if (m_activeGroup >= 0 &&
        m_activeGroup < static_cast<int>(m_network->groups.size()) &&
        m_activeRoad >= 0 &&
        m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        m_network->roads[m_activeRoad].groupId = m_network->groups[m_activeGroup].id;
    }
    m_activePoint = -1;
    SelectSingleRoad(m_activeRoad);
    m_selectedPoints.clear();
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
            {
                PushUndoState();
                m_network->RemoveRoad(m_activeRoad);
            }
        }
        m_activeRoad  = -1;
        ClearRoadSelection();
        ClearPointSelection();
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
            PushUndoState();
            m_network->RemoveRoad(m_activeRoad);
        }
        m_activeRoad  = -1;
        ClearRoadSelection();
        ClearPointSelection();
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
    bool eDown = (GetAsyncKeyState('E') & 0x8000) != 0;
    bool ePress = eDown && !m_prevEKey;
    bool rDown = (GetAsyncKeyState('R') & 0x8000) != 0;
    bool rPress = rDown && !m_prevRKey;
    bool vDown = (GetAsyncKeyState('V') & 0x8000) != 0;
    bool vPress = vDown && !m_prevVKey;

    if (wantMouse || alt)
    {
        m_hasCursorPos = false;
        m_hoverSnapIntersection = -1;
        m_prevLButton  = lDown;
        m_prevWKey     = wDown;
        m_prevEKey     = eDown;
        m_prevRKey     = rDown;
        m_prevVKey     = vDown;
        return;
    }
    m_prevLButton = lDown;
    m_prevWKey = wDown;
    m_prevEKey = eDown;
    m_prevRKey = rDown;
    m_prevVKey = vDown;

    // Compute ray
    XMFLOAT3 rayOrig, rayDir;
    ScreenToRay(vpW, vpH, mousePos, invViewProj, rayOrig, rayDir);

    // Terrain intersection for cursor preview and placement
    XMFLOAT3 hitPos = {};
    bool hasHit = false;
    if (m_terrain && m_terrain->IsReady())
    {
        hasHit = m_terrain->Raycast(rayOrig, rayDir, hitPos);
    }
    else
    {
        hasHit = IntersectRayPlane(
            rayOrig, rayDir,
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f },
            hitPos);
    }
    m_hasCursorPos = hasHit;
    m_cursorPos    = hitPos;

    const bool ctrlDown = (GetAsyncKeyState(VK_CONTROL) & 0x8000) != 0;
    const bool zDown = (GetAsyncKeyState('Z') & 0x8000) != 0;
    const bool yDown = (GetAsyncKeyState('Y') & 0x8000) != 0;
    const bool cDown = (GetAsyncKeyState('C') & 0x8000) != 0;
    const bool pasteVDown = (GetAsyncKeyState('V') & 0x8000) != 0;
    const bool undoShortcut = ctrlDown && zDown;
    const bool redoShortcut = ctrlDown && yDown;
    const bool copyShortcut = ctrlDown && cDown;
    const bool pasteShortcut = ctrlDown && pasteVDown;
    if (!ImGui::GetIO().WantTextInput)
    {
        if (undoShortcut && !m_prevUndoShortcut)
        {
            Undo();
            m_prevUndoShortcut = undoShortcut;
            m_prevRedoShortcut = redoShortcut;
            return;
        }
        if (redoShortcut && !m_prevRedoShortcut)
        {
            Redo();
            m_prevUndoShortcut = undoShortcut;
            m_prevRedoShortcut = redoShortcut;
            return;
        }
        if (copyShortcut && !m_prevCopyShortcut)
        {
            CopySelectedRoads();
            m_prevCopyShortcut = copyShortcut;
            m_prevPasteShortcut = pasteShortcut;
            return;
        }
        if (pasteShortcut && !m_prevPasteShortcut)
        {
            PasteCopiedRoadsAtCursor();
            m_prevCopyShortcut = copyShortcut;
            m_prevPasteShortcut = pasteShortcut;
            return;
        }
    }
    m_prevUndoShortcut = undoShortcut;
    m_prevRedoShortcut = redoShortcut;
    m_prevCopyShortcut = copyShortcut;
    m_prevPasteShortcut = pasteShortcut;

    if (wPress && m_mode != EditorMode::Pathfinding)
    {
        m_rotateYMode = false;
        m_scaleXZMode = false;
        if (!m_selectedRoads.empty() ||
            !m_selectedPoints.empty() ||
            !m_selectedIntersections.empty() ||
            m_mode == EditorMode::PointEdit)
        {
            SetMode(EditorMode::PointEdit);
        }
        else if (m_activeIntersection >= 0)
        {
            SetMode(EditorMode::IntersectionEdit);
        }
    }

    if (ePress && m_mode != EditorMode::Pathfinding)
    {
        m_rotateYMode = true;
        m_scaleXZMode = false;
        if (!m_selectedRoads.empty() ||
            !m_selectedPoints.empty() ||
            !m_selectedIntersections.empty() ||
            m_mode == EditorMode::PointEdit)
        {
            SetMode(EditorMode::PointEdit);
        }
    }

    if (rPress && m_mode != EditorMode::Pathfinding)
    {
        m_rotateYMode = false;
        m_scaleXZMode = true;
        if (!m_selectedRoads.empty() ||
            !m_selectedPoints.empty() ||
            !m_selectedIntersections.empty() ||
            m_mode == EditorMode::PointEdit)
        {
            SetMode(EditorMode::PointEdit);
        }
    }

    if (m_mode == EditorMode::Pathfinding)
    {
        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
            SetMode(EditorMode::Navigate);
        return;
    }

    if (m_marqueeSelecting)
    {
        m_marqueeEnd = mousePos;
        if (!lDown)
        {
            const XMMATRIX viewProj = XMMatrixInverse(nullptr, invViewProj);
            const float width = fabsf(m_marqueeEnd.x - m_marqueeStart.x);
            const float height = fabsf(m_marqueeEnd.y - m_marqueeStart.y);
            if (width >= 4.0f || height >= 4.0f)
            {
                ApplyMarqueeSelection(vpW, vpH, viewProj, m_marqueeAdditive, m_marqueeSubtractive);
                if (m_mode == EditorMode::Navigate && !m_selectedPoints.empty())
                    SetMode(EditorMode::PointEdit);
            }
            else if (!m_marqueeAdditive && !m_marqueeSubtractive)
            {
                m_activeRoad = -1;
                ClearPointSelection();
                ClearIntersectionSelection();
            }
            if (m_mode == EditorMode::PointEdit &&
                m_selectedPoints.empty() &&
                m_selectedIntersections.empty())
            {
                m_mode = EditorMode::Navigate;
            }
            m_marqueeSelecting = false;
        }
        return;
    }

    if (m_mode == EditorMode::Navigate)
    {
        const XMMATRIX viewProj = XMMatrixInverse(nullptr, invViewProj);
        if ((GetAsyncKeyState(VK_DELETE) & 0x8000) &&
            m_activeIntersection >= 0 &&
            m_activeIntersection < static_cast<int>(m_network->intersections.size()))
        {
            PushUndoState();
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
            m_hoverSnapIntersection = -1;
            m_statusMessage = "Intersection deleted";
            return;
        }

        if ((GetAsyncKeyState(VK_DELETE) & 0x8000) &&
            !m_selectedRoads.empty() &&
            m_activePoint < 0 &&
            m_selectedPoints.empty())
        {
            PushUndoState();
            std::vector<int> roadsToDelete = m_selectedRoads;
            std::sort(roadsToDelete.begin(), roadsToDelete.end(), std::greater<int>());
            roadsToDelete.erase(std::unique(roadsToDelete.begin(), roadsToDelete.end()), roadsToDelete.end());
            for (int roadIndex : roadsToDelete)
            {
                if (roadIndex >= 0 && roadIndex < static_cast<int>(m_network->roads.size()))
                    m_network->RemoveRoad(roadIndex);
            }
            ClearRoadSelection();
            ClearPointSelection();
            m_hoverSnapIntersection = -1;
            m_statusMessage = roadsToDelete.size() > 1 ? "Roads deleted" : "Road deleted";
            return;
        }

        if (lClick)
        {
            if (m_activeRoad >= 0)
            {
                const int selPoint = FindNearestPointOnRoad(
                    m_activeRoad, vpW, vpH, mousePos, viewProj);
                if (selPoint >= 0)
                {
                    if (ctrlDown)
                        TogglePointSelection(m_activeRoad, selPoint);
                    else
                    {
                        SelectSinglePoint(m_activeRoad, selPoint);
                        ClearIntersectionSelection();
                    }
                    SetMode(EditorMode::PointEdit);
                    return;
                }
            }

            const int selIntersection = FindNearestIntersection(vpW, vpH, mousePos, viewProj);
            if (selIntersection >= 0)
            {
                if (ctrlDown)
                    ToggleIntersectionSelection(selIntersection);
                else
                {
                    SelectSingleIntersection(selIntersection);
                    ClearPointSelection();
                }
                SetActiveGroupById(m_network->intersections[selIntersection].groupId);
                ClearRoadSelection();
                return;
            }

            const int selRoad = FindNearestRoad(vpW, vpH, mousePos, viewProj);
            if (selRoad >= 0)
            {
                if (ctrlDown)
                    ToggleRoadSelection(selRoad);
                else
                    SelectSingleRoad(selRoad);
                SetActiveGroupById(m_network->roads[selRoad].groupId);
                ClearPointSelection();
                ClearIntersectionSelection();
                return;
            }

            ClearRoadSelection();
            m_dragging = false;
            m_activeGizmoAxis = GizmoAxis::None;
            m_hoverSnapIntersection = -1;
            m_marqueeSelecting = true;
            m_marqueeStart = mousePos;
            m_marqueeEnd = mousePos;
            m_marqueeAdditive = ctrlDown && (GetAsyncKeyState(VK_SHIFT) & 0x8000) == 0;
            m_marqueeSubtractive = ctrlDown && (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;
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
            PushUndoState();
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
            m_network->EnsureDefaultGroup();
            PushUndoState();
            m_activeIntersection = m_network->AddIntersection(hitPos);
            if (m_activeGroup >= 0 &&
                m_activeGroup < static_cast<int>(m_network->groups.size()) &&
                m_activeIntersection >= 0 &&
                m_activeIntersection < static_cast<int>(m_network->intersections.size()))
            {
                m_network->intersections[m_activeIntersection].groupId =
                    m_network->groups[m_activeGroup].id;
            }
            ClearPointSelection();
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

        if (vPress && !ImGui::GetIO().WantTextInput)
        {
            if (SplitSelectedRoadAtPoint())
                return;
        }

        if (m_dragging && lDown)
        {
            if (m_activeGizmoAxis != GizmoAxis::None)
            {
                if ((!m_selectedPoints.empty() || !m_selectedIntersections.empty()) &&
                    m_pointDragStartPositions.size() == m_selectedPoints.size() &&
                    m_intersectionDragStartPositions.size() == m_selectedIntersections.size())
                {
                    XMFLOAT3 pivotStart = m_axisDragStartPos;
                    XMFLOAT3 delta = { 0.0f, 0.0f, 0.0f };
                    float rotationDelta = 0.0f;
                    float scaleFactor = 1.0f;
                    if (m_activeGizmoAxis == GizmoAxis::RotateY)
                    {
                        const XMFLOAT2 pivotScreen =
                            WorldToScreen(m_axisDragStartPos, viewProj, vpW, vpH);
                        if (pivotScreen.x >= 0.0f)
                        {
                            const float currentAngle =
                                atan2f(mousePos.y - pivotScreen.y, mousePos.x - pivotScreen.x);
                            rotationDelta = NormalizeAngleDelta(currentAngle - m_rotateDragStartAngle);
                        }
                    }
                    else if (m_activeGizmoAxis == GizmoAxis::ScaleXZ)
                    {
                        const XMFLOAT2 pivotScreen =
                            WorldToScreen(m_axisDragStartPos, viewProj, vpW, vpH);
                        if (pivotScreen.x >= 0.0f)
                        {
                            const float currentDistance = Dist2D(mousePos, pivotScreen);
                            scaleFactor = currentDistance / (std::max)(m_scaleDragStartDistance, 1.0f);
                            scaleFactor = std::clamp(scaleFactor, 0.1f, 20.0f);
                        }
                    }
                    else if (m_snapToTerrain &&
                             hasHit &&
                             m_activeGizmoAxis == GizmoAxis::Center)
                    {
                        delta =
                        {
                            hitPos.x - pivotStart.x,
                            hitPos.y - pivotStart.y,
                            hitPos.z - pivotStart.z
                        };
                    }
                    else if (m_activeGizmoAxis == GizmoAxis::Center)
                    {
                        XMFLOAT3 planeHit;
                        if (IntersectRayPlane(rayOrig, rayDir, m_axisDragStartPos,
                                              m_planeDragNormal, planeHit))
                        {
                            delta =
                            {
                                planeHit.x - m_planeDragStartHit.x,
                                planeHit.y - m_planeDragStartHit.y,
                                planeHit.z - m_planeDragStartHit.z
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
                            delta = { axisDir.x * deltaWorld, axisDir.y * deltaWorld, axisDir.z * deltaWorld };
                        }
                    }

                    for (size_t selectionIndex = 0; selectionIndex < m_selectedPoints.size(); ++selectionIndex)
                    {
                        const PointRef& pointRef = m_selectedPoints[selectionIndex];
                        if (pointRef.roadIndex < 0 ||
                            pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                            continue;
                        Road& road = m_network->roads[pointRef.roadIndex];
                        const int pointIndex = pointRef.pointIndex;
                        if (pointIndex < 0 || pointIndex >= static_cast<int>(road.points.size()))
                            continue;

                        XMFLOAT3 newPos = m_pointDragStartPositions[selectionIndex];
                        if (m_activeGizmoAxis == GizmoAxis::RotateY)
                        {
                            newPos = RotateAroundY(newPos, pivotStart, rotationDelta);
                            if (m_snapToTerrain &&
                                m_terrain && m_terrain->IsReady())
                            {
                                newPos.y = m_terrain->GetHeightAt(newPos.x, newPos.z);
                            }
                        }
                        else if (m_activeGizmoAxis == GizmoAxis::ScaleXZ)
                        {
                            newPos = ScaleAroundXZ(newPos, pivotStart, scaleFactor);
                            if (m_snapToTerrain &&
                                m_terrain && m_terrain->IsReady())
                            {
                                newPos.y = m_terrain->GetHeightAt(newPos.x, newPos.z);
                            }
                        }
                        else
                        {
                            newPos =
                            {
                                m_pointDragStartPositions[selectionIndex].x + delta.x,
                                m_pointDragStartPositions[selectionIndex].y + delta.y,
                                m_pointDragStartPositions[selectionIndex].z + delta.z
                            };

                            if (m_snapToTerrain &&
                                m_terrain && m_terrain->IsReady() &&
                                m_activeGizmoAxis != GizmoAxis::Y)
                            {
                                newPos.y = m_terrain->GetHeightAt(newPos.x, newPos.z);
                            }
                        }

                        road.points[pointIndex].pos = newPos;
                    }

                    for (size_t selectionIndex = 0; selectionIndex < m_selectedIntersections.size(); ++selectionIndex)
                    {
                        const int intersectionIndex = m_selectedIntersections[selectionIndex];
                        if (intersectionIndex < 0 ||
                            intersectionIndex >= static_cast<int>(m_network->intersections.size()) ||
                            selectionIndex >= m_intersectionDragStartPositions.size())
                            continue;

                        XMFLOAT3 newPos = m_intersectionDragStartPositions[selectionIndex];
                        if (m_activeGizmoAxis == GizmoAxis::RotateY)
                        {
                            newPos = RotateAroundY(newPos, pivotStart, rotationDelta);
                            if (m_snapToTerrain &&
                                m_terrain && m_terrain->IsReady())
                            {
                                newPos.y = m_terrain->GetHeightAt(newPos.x, newPos.z);
                            }
                        }
                        else if (m_activeGizmoAxis == GizmoAxis::ScaleXZ)
                        {
                            newPos = ScaleAroundXZ(newPos, pivotStart, scaleFactor);
                            if (m_snapToTerrain &&
                                m_terrain && m_terrain->IsReady())
                            {
                                newPos.y = m_terrain->GetHeightAt(newPos.x, newPos.z);
                            }
                        }
                        else
                        {
                            newPos =
                            {
                                m_intersectionDragStartPositions[selectionIndex].x + delta.x,
                                m_intersectionDragStartPositions[selectionIndex].y + delta.y,
                                m_intersectionDragStartPositions[selectionIndex].z + delta.z
                            };

                            if (m_snapToTerrain &&
                                m_terrain && m_terrain->IsReady() &&
                                m_activeGizmoAxis != GizmoAxis::Y)
                            {
                                newPos.y = m_terrain->GetHeightAt(newPos.x, newPos.z);
                            }
                        }

                        m_network->intersections[intersectionIndex].pos = newPos;
                        SyncRoadConnectionsForIntersection(intersectionIndex);
                    }

                    if (m_selectedPoints.size() == 1 && IsSelectedRoadEndpoint())
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
                    PushUndoState();
                    SnapSelectedEndpointToIntersection(m_hoverSnapIntersection);
                    m_statusMessage = "Road endpoint connected";
                }
                else
                {
                    PushUndoState();
                    ClearSelectedRoadConnection();
                }
            }
            m_dragging = false;
            m_activeGizmoAxis = GizmoAxis::None;
            m_hoverSnapIntersection = -1;
            m_pointDragStartPositions.clear();
            m_intersectionDragStartPositions.clear();
        }
        else if (lClick)
        {
            GizmoAxis gizmoAxis = PickGizmoAxis(vpW, vpH, mousePos, viewProj);
            if (gizmoAxis != GizmoAxis::None)
            {
                if (!m_selectedPoints.empty() || !m_selectedIntersections.empty())
                {
                    m_activeGizmoAxis  = gizmoAxis;
                    if (!GetActiveGizmoPivot(m_axisDragStartPos))
                        return;
                    m_axisDragStartMouse = mousePos;
                    const XMFLOAT2 pivotScreen =
                        WorldToScreen(m_axisDragStartPos, viewProj, vpW, vpH);
                    m_rotateDragStartAngle =
                        atan2f(mousePos.y - pivotScreen.y, mousePos.x - pivotScreen.x);
                    m_scaleDragStartDistance = (std::max)(1.0f, Dist2D(mousePos, pivotScreen));
                    m_planeDragNormal = rayDir;
                    if (!IntersectRayPlane(rayOrig, rayDir, m_axisDragStartPos,
                                           m_planeDragNormal, m_planeDragStartHit))
                    {
                        m_planeDragStartHit = m_axisDragStartPos;
                    }
                    m_pointDragStartPositions.clear();
                    for (const PointRef& pointRef : m_selectedPoints)
                    {
                        if (pointRef.roadIndex < 0 ||
                            pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                            continue;
                        const Road& road = m_network->roads[pointRef.roadIndex];
                        if (pointRef.pointIndex >= 0 &&
                            pointRef.pointIndex < static_cast<int>(road.points.size()))
                        {
                            m_pointDragStartPositions.push_back(road.points[pointRef.pointIndex].pos);
                        }
                    }
                    m_intersectionDragStartPositions.clear();
                    for (int intersectionIndex : m_selectedIntersections)
                    {
                        if (intersectionIndex >= 0 &&
                            intersectionIndex < static_cast<int>(m_network->intersections.size()))
                        {
                            m_intersectionDragStartPositions.push_back(
                                m_network->intersections[intersectionIndex].pos);
                        }
                    }
                    PushUndoState();
                    DisconnectSelectedRoadEndpoints();
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
                    SetActiveGroupById(m_network->roads[selRoad].groupId);
                    if (ctrlDown)
                        TogglePointSelection(selRoad, selPt);
                    else
                    {
                        SelectSinglePoint(selRoad, selPt);
                        ClearIntersectionSelection();
                    }
                    m_activeIntersection = -1;
                    m_dragging         = false;
                    m_activeGizmoAxis  = GizmoAxis::None;
                    m_hoverSnapIntersection = -1;
                }
                else
                {
                    const int selIntersection = FindNearestIntersection(vpW, vpH, mousePos, viewProj);
                    if (selIntersection >= 0)
                    {
                        SetActiveGroupById(m_network->intersections[selIntersection].groupId);
                        if (ctrlDown)
                            ToggleIntersectionSelection(selIntersection);
                        else
                        {
                            SelectSingleIntersection(selIntersection);
                            ClearPointSelection();
                        }
                        m_dragging = false;
                        m_activeGizmoAxis = GizmoAxis::None;
                        m_hoverSnapIntersection = -1;
                    }
                    else if (m_activeRoad >= 0 &&
                             m_activeRoad < static_cast<int>(m_network->roads.size()) &&
                             hasHit)
                    {
                        Road& road = m_network->roads[m_activeRoad];
                        const int segmentIndex =
                            FindNearestSegmentOnRoad(m_activeRoad, vpW, vpH, mousePos, viewProj);
                        if (segmentIndex >= 0)
                        {
                            PushUndoState();
                            RoadPoint newPoint;
                            newPoint.pos = hitPos;
                            if (segmentIndex >= 0 &&
                                segmentIndex + 1 < static_cast<int>(road.points.size()))
                            {
                                newPoint.width =
                                    0.5f * (road.points[segmentIndex].width +
                                            road.points[segmentIndex + 1].width);
                            }
                            else
                            {
                                newPoint.width = m_defaultWidth;
                            }

                            road.points.insert(road.points.begin() + segmentIndex + 1, newPoint);
                            SelectSinglePoint(m_activeRoad, segmentIndex + 1);
                            ClearIntersectionSelection();
                            m_dragging = false;
                            m_activeGizmoAxis = GizmoAxis::None;
                            m_hoverSnapIntersection = -1;
                            m_statusMessage = "Point inserted";
                        }
                        else
                        {
                            m_dragging         = false;
                            m_activeGizmoAxis  = GizmoAxis::None;
                            m_hoverSnapIntersection = -1;
                            m_marqueeSelecting = true;
                            m_marqueeStart = mousePos;
                            m_marqueeEnd = mousePos;
                            m_marqueeAdditive = ctrlDown && (GetAsyncKeyState(VK_SHIFT) & 0x8000) == 0;
                            m_marqueeSubtractive = ctrlDown && (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;
                        }
                    }
                    else
                    {
                        m_dragging         = false;
                        m_activeGizmoAxis  = GizmoAxis::None;
                        m_hoverSnapIntersection = -1;
                        m_marqueeSelecting = true;
                        m_marqueeStart = mousePos;
                        m_marqueeEnd = mousePos;
                        m_marqueeAdditive = ctrlDown && (GetAsyncKeyState(VK_SHIFT) & 0x8000) == 0;
                        m_marqueeSubtractive = ctrlDown && (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;
                    }
                }
            }
        }

        // Delete selected point
        if ((GetAsyncKeyState(VK_DELETE) & 0x8000) &&
            !m_selectedPoints.empty())
        {
            PushUndoState();
            std::vector<PointRef> pointsToDelete = m_selectedPoints;
            std::sort(
                pointsToDelete.begin(),
                pointsToDelete.end(),
                [](const PointRef& lhs, const PointRef& rhs)
                {
                    if (lhs.roadIndex != rhs.roadIndex)
                        return lhs.roadIndex > rhs.roadIndex;
                    return lhs.pointIndex > rhs.pointIndex;
                });
            pointsToDelete.erase(
                std::unique(
                    pointsToDelete.begin(),
                    pointsToDelete.end(),
                    [](const PointRef& lhs, const PointRef& rhs)
                    {
                        return lhs.roadIndex == rhs.roadIndex && lhs.pointIndex == rhs.pointIndex;
                    }),
                pointsToDelete.end());
            bool removedAny = false;
            for (auto it = pointsToDelete.rbegin(); it != pointsToDelete.rend(); ++it)
            {
                if (it->roadIndex < 0 || it->roadIndex >= static_cast<int>(m_network->roads.size()))
                    continue;
                Road& road = m_network->roads[it->roadIndex];
                const int pointIndex = it->pointIndex;
                if (pointIndex < 0 || pointIndex >= static_cast<int>(road.points.size()))
                    continue;
                if (pointIndex == 0)
                    road.startIntersectionId.clear();
                if (pointIndex == static_cast<int>(road.points.size()) - 1)
                    road.endIntersectionId.clear();
                road.points.erase(road.points.begin() + pointIndex);
                removedAny = true;
            }
            if (removedAny)
                m_statusMessage = pointsToDelete.size() > 1 ? "Points deleted" : "Point deleted";
            ClearPointSelection();
            SanitizeSelection();
            m_activeGizmoAxis = GizmoAxis::None;
            m_hoverSnapIntersection = -1;
            m_pointDragStartPositions.clear();
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
                PushUndoState();
                m_dragging = true;
            }
            else
            {
                m_activeIntersection = FindNearestIntersection(vpW, vpH, mousePos, viewProj);
                if (m_activeIntersection >= 0)
                    SetActiveGroupById(m_network->intersections[m_activeIntersection].groupId);
                ClearPointSelection();
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
            PushUndoState();
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

    static const XMFLOAT4 colorRoad     = { 0.72f, 0.72f, 0.75f, 1.0f };
    static const XMFLOAT4 colorSelected = { 1.0f, 1.0f, 1.0f, 1.0f };
    static const XMFLOAT4 colorCursor   = { 0.2f, 1.0f, 0.4f, 0.4f };
    static const XMFLOAT4 colorAxisX    = { 1.0f, 0.2f, 0.2f, 1.0f };
    static const XMFLOAT4 colorAxisY    = { 0.2f, 1.0f, 0.2f, 1.0f };
    static const XMFLOAT4 colorAxisZ    = { 0.2f, 0.6f, 1.0f, 1.0f };
    static const XMFLOAT4 colorRotateY  = { 1.0f, 0.85f, 0.2f, 1.0f };
    static const XMFLOAT4 colorScaleXZ  = { 1.0f, 0.45f, 0.9f, 1.0f };
    static const XMFLOAT4 colorPivot    = { 1.0f, 1.0f, 1.0f, 0.9f };
    static const XMFLOAT4 colorIntersection = { 0.3f, 0.95f, 1.0f, 1.0f };
    static const XMFLOAT4 colorIntersectionSelected = { 1.0f, 0.4f, 0.2f, 1.0f };
    static const XMFLOAT4 colorConnection = { 0.9f, 0.9f, 0.3f, 0.9f };
    static const XMFLOAT4 colorSnapCandidate = { 1.0f, 1.0f, 0.2f, 1.0f };
    static const XMFLOAT4 colorPreview = { 0.45f, 0.95f, 0.95f, 0.9f };

    auto drawRoadLines = [&](int roadIndex, XMFLOAT4 color)
    {
        const Road& road = m_network->roads[roadIndex];
        for (int pi = 0; pi + 1 < static_cast<int>(road.points.size()); ++pi)
            dd.AddLine(road.points[pi].pos, road.points[pi + 1].pos, color);

        if (road.closed && road.points.size() >= 2)
            dd.AddLine(road.points.back().pos, road.points.front().pos, color);

        const int startIsec = FindIntersectionIndexById(road.startIntersectionId);
        const int endIsec   = FindIntersectionIndexById(road.endIntersectionId);
        if (startIsec >= 0 && !road.points.empty() &&
            IsIntersectionVisible(m_network->intersections[startIsec]))
            dd.AddLine(road.points.front().pos, m_network->intersections[startIsec].pos, colorConnection);
        if (endIsec >= 0 && !road.points.empty() &&
            IsIntersectionVisible(m_network->intersections[endIsec]))
            dd.AddLine(road.points.back().pos, m_network->intersections[endIsec].pos, colorConnection);
    };

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadVisible(road) || IsRoadSelected(ri) || ri == m_activeRoad)
            continue;
        drawRoadLines(ri, colorRoad);
    }

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadVisible(road) || (!IsRoadSelected(ri) && ri != m_activeRoad))
            continue;
        drawRoadLines(ri, colorSelected);
    }

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadVisible(road))
            continue;

        const std::vector<XMFLOAT3> previewCurve = BuildRoadPreviewCurve(road);
        for (int sampleIndex = 0; sampleIndex + 1 < static_cast<int>(previewCurve.size()); ++sampleIndex)
        {
            XMFLOAT4 segmentColor = colorPreview;
            if (m_showRoadGradeGradient)
            {
                const float horizontalDistance = DistanceXZ3(previewCurve[sampleIndex], previewCurve[sampleIndex + 1]);
                if (horizontalDistance > 1e-4f)
                {
                    const float dy = previewCurve[sampleIndex + 1].y - previewCurve[sampleIndex].y;
                    const float gradePercent = fabsf(dy) / horizontalDistance * 100.0f;
                    segmentColor = PreviewGradeColor(gradePercent, m_roadGradeRedThresholdPercent);
                }
            }
            dd.AddLine(previewCurve[sampleIndex], previewCurve[sampleIndex + 1], segmentColor);
        }
    }

    for (int ii = 0; ii < static_cast<int>(m_network->intersections.size()); ++ii)
    {
        const Intersection& isec = m_network->intersections[ii];
        if (!IsIntersectionVisible(isec))
            continue;
        const XMFLOAT4 col = (ii == m_activeIntersection)
            || IsIntersectionSelected(ii)
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

    if (m_mode == EditorMode::PointEdit)
    {
        XMFLOAT3 pivot;
        if (GetActiveGizmoPivot(pivot))
        {
            const XMFLOAT3 p = pivot;
            dd.AddLine({ p.x - 0.15f, p.y, p.z }, { p.x + 0.15f, p.y, p.z }, colorPivot);
            dd.AddLine({ p.x, p.y - 0.15f, p.z }, { p.x, p.y + 0.15f, p.z }, colorPivot);
            dd.AddLine({ p.x, p.y, p.z - 0.15f }, { p.x, p.y, p.z + 0.15f }, colorPivot);
            if (m_rotateYMode)
            {
                constexpr int kSegments = 48;
                const float radius = ComputeRotationGizmoRadius(pivot, viewProj, vpW, vpH);
                XMFLOAT3 prev =
                {
                    p.x + radius,
                    p.y,
                    p.z
                };
                for (int segmentIndex = 1; segmentIndex <= kSegments; ++segmentIndex)
                {
                    const float t = XM_2PI * static_cast<float>(segmentIndex) / static_cast<float>(kSegments);
                    const XMFLOAT3 next =
                    {
                        p.x + cosf(t) * radius,
                        p.y,
                        p.z + sinf(t) * radius
                    };
                    dd.AddLine(prev, next, colorRotateY);
                    prev = next;
                }
            }
            else if (m_scaleXZMode)
            {
                const float radius = ComputeScaleGizmoRadius(pivot, viewProj, vpW, vpH);
                XMFLOAT3 corners[5] =
                {
                    { p.x + radius, p.y, p.z + radius },
                    { p.x - radius, p.y, p.z + radius },
                    { p.x - radius, p.y, p.z - radius },
                    { p.x + radius, p.y, p.z - radius },
                    { p.x + radius, p.y, p.z + radius }
                };
                for (int i = 0; i < 4; ++i)
                    dd.AddLine(corners[i], corners[i + 1], colorScaleXZ);
            }
            else
            {
                const float axisLenX = ComputeGizmoAxisLength(pivot, GizmoAxis::X, viewProj, vpW, vpH);
                const float axisLenY = ComputeGizmoAxisLength(pivot, GizmoAxis::Y, viewProj, vpW, vpH);
                const float axisLenZ = ComputeGizmoAxisLength(pivot, GizmoAxis::Z, viewProj, vpW, vpH);
                dd.AddLine(p, { p.x + axisLenX, p.y, p.z }, colorAxisX);
                dd.AddLine(p, { p.x, p.y + axisLenY, p.z }, colorAxisY);
                dd.AddLine(p, { p.x, p.y, p.z + axisLenZ }, colorAxisZ);
            }
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

    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const float kRadius    = 3.0f;
    const float kSelectedRoadThickness = 2.0f;
    const ImU32 colPoint    = IM_COL32(160, 160, 160, 220);
    const ImU32 colRoadSel  = IM_COL32(255, 255, 255, 255);
    const ImU32 colPointSel = IM_COL32(255, 140,  60, 255);
    const ImU32 colCursor   = IM_COL32( 60, 255, 110, 220);
    const ImU32 colAxisX    = IM_COL32(255,  80,  80, 255);
    const ImU32 colAxisY    = IM_COL32( 80, 255, 120, 255);
    const ImU32 colAxisZ    = IM_COL32( 80, 150, 255, 255);
    const ImU32 colScaleXZ  = IM_COL32(255, 110, 220, 255);
    const ImU32 colIsec     = IM_COL32( 80, 240, 255, 255);
    const ImU32 colIsecSel  = IM_COL32(255, 120, 80, 255);
    const ImU32 colSnap     = IM_COL32(255, 240, 80, 255);

    auto drawSelectedRoadOverlay = [&](const Road& road)
    {
        for (int pi = 0; pi + 1 < static_cast<int>(road.points.size()); ++pi)
        {
            ImVec2 a;
            ImVec2 b;
            if (!WorldToScreen(road.points[pi].pos, viewProj, vpW, vpH, a) ||
                !WorldToScreen(road.points[pi + 1].pos, viewProj, vpW, vpH, b))
                continue;
            dl->AddLine(a, b, colRoadSel, kSelectedRoadThickness);
        }

        if (road.closed && road.points.size() >= 2)
        {
            ImVec2 a;
            ImVec2 b;
            if (WorldToScreen(road.points.back().pos, viewProj, vpW, vpH, a) &&
                WorldToScreen(road.points.front().pos, viewProj, vpW, vpH, b))
            {
                dl->AddLine(a, b, colRoadSel, kSelectedRoadThickness);
            }
        }
    };

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadVisible(road) || (!IsRoadSelected(ri) && ri != m_activeRoad))
            continue;
        drawSelectedRoadOverlay(road);
    }

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadVisible(road))
            continue;
        for (int pi = 0; pi < static_cast<int>(road.points.size()); ++pi)
        {
            ImVec2 sp;
            if (!WorldToScreen(road.points[pi].pos, viewProj, vpW, vpH, sp))
                continue;
            const bool pointSelected = IsPointSelected(ri, pi);
            const bool roadSelected = IsRoadSelected(ri) || ri == m_activeRoad;
            ImU32 col = colPoint;
            if (roadSelected)
                col = colRoadSel;
            if (pointSelected)
                col = colPointSel;
            const float radius = pointSelected ? (kRadius + 2.0f) : kRadius;
            dl->AddCircleFilled(sp, radius, col, 20);
        }

        if ((m_showRoadNames || m_showRoadPreviewMetrics) && !road.points.empty())
        {
            XMFLOAT3 center = { 0.0f, 0.0f, 0.0f };
            for (const RoadPoint& point : road.points)
            {
                center.x += point.pos.x;
                center.y += point.pos.y;
                center.z += point.pos.z;
            }
            const float invCount = 1.0f / static_cast<float>(road.points.size());
            center.x *= invCount;
            center.y *= invCount;
            center.z *= invCount;

            ImVec2 labelPos;
            if (WorldToScreen(center, viewProj, vpW, vpH, labelPos))
            {
                const ImVec2 textPos(labelPos.x + 10.0f, labelPos.y - 8.0f);
                if (m_showRoadNames && !road.name.empty())
                    dl->AddText(textPos, IM_COL32(255, 215, 90, 255), road.name.c_str());

                if (m_showRoadPreviewMetrics)
                {
                    const std::vector<XMFLOAT3> previewCurve = BuildRoadPreviewCurve(road);
                    const float previewLength = ComputePolylineLength(previewCurve);
                    const float averageGradePercent =
                        ComputeAverageAbsoluteGradePercent(previewCurve);
                    const float maxGradePercent =
                        ComputeMaxAbsoluteGradePercent(previewCurve);
                    char metricsBuf[128] = {};
                    sprintf_s(
                        metricsBuf,
                        "%.0fm / %.1f%%(%.1f%%)",
                        std::round(previewLength),
                        averageGradePercent,
                        maxGradePercent);
                    const float metricsOffsetY = (m_showRoadNames && !road.name.empty()) ? 10.0f : 0.0f;
                    dl->AddText(
                        ImVec2(textPos.x, textPos.y + metricsOffsetY),
                        IM_COL32(180, 245, 255, 255),
                        metricsBuf);
                }
            }
        }
    }

    if (m_marqueeSelecting)
    {
        const float minX = (std::min)(m_marqueeStart.x, m_marqueeEnd.x);
        const float maxX = (std::max)(m_marqueeStart.x, m_marqueeEnd.x);
        const float minY = (std::min)(m_marqueeStart.y, m_marqueeEnd.y);
        const float maxY = (std::max)(m_marqueeStart.y, m_marqueeEnd.y);
        dl->AddRect(
            ImVec2(minX, minY),
            ImVec2(maxX, maxY),
            IM_COL32(255, 220, 80, 220),
            0.0f,
            0,
            1.5f);
        dl->AddRectFilled(
            ImVec2(minX, minY),
            ImVec2(maxX, maxY),
            IM_COL32(255, 220, 80, 35));
    }

    for (int ii = 0; ii < static_cast<int>(m_network->intersections.size()); ++ii)
    {
        const Intersection& isec = m_network->intersections[ii];
        if (!IsIntersectionVisible(isec))
            continue;
        ImVec2 sp;
        if (!WorldToScreen(isec.pos, viewProj, vpW, vpH, sp))
            continue;
        const bool selected = (ii == m_activeIntersection) || IsIntersectionSelected(ii);
        const ImU32 col = selected ? colIsecSel : colIsec;
        dl->AddCircle(sp, 10.0f, col, 24, 2.0f);
        dl->AddLine(ImVec2(sp.x - 8.0f, sp.y), ImVec2(sp.x + 8.0f, sp.y), col, 2.0f);
        dl->AddLine(ImVec2(sp.x, sp.y - 8.0f), ImVec2(sp.x, sp.y + 8.0f), col, 2.0f);
        if (m_showIntersectionNames)
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

    if (m_mode == EditorMode::PointEdit)
    {
        XMFLOAT3 p;
        if (GetActiveGizmoPivot(p))
        {
            ImVec2 pivot;
            if (WorldToScreen(p, viewProj, vpW, vpH, pivot))
            {
                dl->AddCircle(pivot, 8.0f, IM_COL32(255,255,255,220), 24, 2.0f);
                dl->AddCircleFilled(pivot, 3.0f, IM_COL32(255,255,255,220), 16);
                if (m_rotateYMode)
                {
                    constexpr int kSegments = 64;
                    std::vector<ImVec2> ringPoints;
                    ringPoints.reserve(kSegments + 1);
                    const float radius = ComputeRotationGizmoRadius(p, viewProj, vpW, vpH);
                    for (int segmentIndex = 0; segmentIndex <= kSegments; ++segmentIndex)
                    {
                        const float t = XM_2PI * static_cast<float>(segmentIndex) / static_cast<float>(kSegments);
                        ImVec2 ringPoint;
                        if (!WorldToScreen(
                                { p.x + cosf(t) * radius, p.y, p.z + sinf(t) * radius },
                                viewProj,
                                vpW,
                                vpH,
                                ringPoint))
                            continue;
                        ringPoints.push_back(ringPoint);
                    }
                    if (ringPoints.size() >= 2)
                        dl->AddPolyline(ringPoints.data(), static_cast<int>(ringPoints.size()), IM_COL32(255, 215, 60, 240), ImDrawFlags_None, 2.0f);
                    dl->AddText(ImVec2(pivot.x + 10.0f, pivot.y - 22.0f), IM_COL32(255, 215, 60, 240), "RY");
                }
                else if (m_scaleXZMode)
                {
                    const float radius = ComputeScaleGizmoRadius(p, viewProj, vpW, vpH);
                    ImVec2 corners[4];
                    bool valid = true;
                    const XMFLOAT3 cornersWorld[4] =
                    {
                        { p.x + radius, p.y, p.z + radius },
                        { p.x - radius, p.y, p.z + radius },
                        { p.x - radius, p.y, p.z - radius },
                        { p.x + radius, p.y, p.z - radius }
                    };
                    for (int i = 0; i < 4; ++i)
                        valid = valid && WorldToScreen(cornersWorld[i], viewProj, vpW, vpH, corners[i]);
                    if (valid)
                        dl->AddPolyline(corners, 4, colScaleXZ, ImDrawFlags_Closed, 2.0f);
                    dl->AddText(ImVec2(pivot.x + 10.0f, pivot.y - 22.0f), colScaleXZ, "SXZ");
                }
                else
                {
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
    m_network->EnsureDefaultGroup();
    SanitizeSelection();

    ImGui::SetNextWindowPos(ImVec2(10, 360), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(320, 420), ImGuiCond_FirstUseEver);
    ImGui::Begin(u8"\u9053\u8DEF\u30A8\u30C7\u30A3\u30BF");

    // Mode toolbar
    {
        bool navActive  = (m_mode == EditorMode::Navigate);
        bool drawActive = (m_mode == EditorMode::PolylineDraw);
        bool editActive = (m_mode == EditorMode::PointEdit);
        bool isecDrawActive = (m_mode == EditorMode::IntersectionDraw);
        bool isecEditActive = (m_mode == EditorMode::IntersectionEdit);
        bool pathActive = (m_mode == EditorMode::Pathfinding);

        if (navActive)  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button(u8"\u30AA\u30D6\u30B8\u30A7\u30AF\u30C8")) SetMode(EditorMode::Navigate);
        if (navActive)  ImGui::PopStyleColor();

        ImGui::SameLine();

        if (drawActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button(u8"\u9053\u8DEF\u4F5C\u6210")) StartNewRoad();
        if (drawActive) ImGui::PopStyleColor();

        ImGui::SameLine();

        if (editActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button(u8"\u30DD\u30A4\u30F3\u30C8\u7DE8\u96C6")) SetMode(EditorMode::PointEdit);
        if (editActive) ImGui::PopStyleColor();

        ImGui::NewLine();

        if (isecDrawActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button(u8"\u4EA4\u5DEE\u70B9\u4F5C\u6210")) SetMode(EditorMode::IntersectionDraw);
        if (isecDrawActive) ImGui::PopStyleColor();

        ImGui::SameLine();

        if (isecEditActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button(u8"\u4EA4\u5DEE\u70B9\u7DE8\u96C6")) SetMode(EditorMode::IntersectionEdit);
        if (isecEditActive) ImGui::PopStyleColor();

        ImGui::SameLine();

        if (pathActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button(u8"\u7D4C\u8DEF\u63A2\u7D22"))
        {
            if (pathActive)
                SetMode(EditorMode::Navigate);
            else
                SetMode(EditorMode::Pathfinding);
        }
        if (pathActive) ImGui::PopStyleColor();
    }

    ImGui::Separator();

    // Mode hint
    switch (m_mode)
    {
    case EditorMode::Navigate:
        ImGui::TextDisabled(u8"\u30AA\u30D6\u30B8\u30A7\u30AF\u30C8\u9078\u629E\u30E2\u30FC\u30C9");
        ImGui::TextDisabled(u8"\u9053\u8DEF\u3092\u30AF\u30EA\u30C3\u30AF: \u9053\u8DEF\u9078\u629E");
        ImGui::TextDisabled(u8"Ctrl+\u30AF\u30EA\u30C3\u30AF: \u9053\u8DEF\u3092\u8907\u6570\u9078\u629E");
        ImGui::TextDisabled(u8"\u9078\u629E\u4E2D\u9053\u8DEF\u306E\u30DD\u30A4\u30F3\u30C8\u306F\u76F4\u63A5\u9078\u629E\u53EF\u80FD");
        ImGui::TextDisabled(u8"\u7A7A\u304D\u9818\u57DF\u3092\u30C9\u30E9\u30C3\u30B0: \u77E9\u5F62\u9078\u629E");
        ImGui::TextDisabled(u8"W \u30AD\u30FC: \u79FB\u52D5\u30AE\u30BA\u30E2");
        ImGui::TextDisabled(u8"E \u30AD\u30FC: Y\u56DE\u8EE2\u30AE\u30BA\u30E2");
        ImGui::TextDisabled(u8"R \u30AD\u30FC: XZ\u62E1\u5927\u30AE\u30BA\u30E2");
        break;
    case EditorMode::PolylineDraw:
        ImGui::TextColored(ImVec4(0.2f,1,0.4f,1), u8"\u4F5C\u6210\u4E2D: %s",
            (m_activeRoad >= 0 &&
             m_activeRoad < static_cast<int>(m_network->roads.size()))
            ? m_network->roads[m_activeRoad].name.c_str() : "");
        ImGui::TextDisabled(u8"\u5DE6\u30AF\u30EA\u30C3\u30AF: \u30DD\u30A4\u30F3\u30C8\u8FFD\u52A0");
        ImGui::TextDisabled(u8"Enter: \u78BA\u5B9A  Esc: \u30AD\u30E3\u30F3\u30BB\u30EB");
        ImGui::SliderFloat(u8"\u5E45", &m_defaultWidth, 0.5f, 20.0f);
        break;
    case EditorMode::PointEdit:
        ImGui::TextColored(ImVec4(1,0.8f,0.2f,1), u8"\u30DD\u30A4\u30F3\u30C8\u7DE8\u96C6");
        ImGui::TextDisabled(u8"\u30DD\u30A4\u30F3\u30C8\u3092\u30AF\u30EA\u30C3\u30AF: \u9078\u629E");
        ImGui::TextDisabled(u8"Ctrl+\u30AF\u30EA\u30C3\u30AF: \u8FFD\u52A0/\u89E3\u9664");
        ImGui::TextDisabled(u8"\u7A7A\u304D\u9818\u57DF\u3092\u30C9\u30E9\u30C3\u30B0: \u77E9\u5F62\u9078\u629E");
        ImGui::TextDisabled(u8"Ctrl+Shift \u77E9\u5F62: \u9078\u629E\u89E3\u9664");
        ImGui::TextDisabled(u8"\u4E2D\u592E\u30C9\u30E9\u30C3\u30B0: \u30B9\u30AF\u30EA\u30FC\u30F3\u5E73\u9762\u3067\u79FB\u52D5");
        ImGui::TextDisabled(u8"X/Y/Z \u8EF8\u30AF\u30EA\u30C3\u30AF: \u8EF8\u65B9\u5411\u306B\u79FB\u52D5");
        ImGui::TextDisabled(u8"E: Y\u56DE\u8EE2\u30EA\u30F3\u30B0");
        ImGui::TextDisabled(u8"R: XZ\u62E1\u5927\u30EA\u30F3\u30B0");
        ImGui::TextDisabled(u8"\u7AEF\u70B9\u304C\u4EA4\u5DEE\u70B9\u4ED8\u8FD1: \u30B9\u30CA\u30C3\u30D7/\u63A5\u7D9A");
        ImGui::TextDisabled(u8"Delete: \u30DD\u30A4\u30F3\u30C8\u524A\u9664");
        break;
    case EditorMode::IntersectionDraw:
        ImGui::TextColored(ImVec4(0.2f,0.9f,1.0f,1), u8"\u4EA4\u5DEE\u70B9\u4F5C\u6210");
        ImGui::TextDisabled(u8"\u5DE6\u30AF\u30EA\u30C3\u30AF: \u4EA4\u5DEE\u70B9\u914D\u7F6E");
        ImGui::TextDisabled(u8"Esc: \u30AA\u30D6\u30B8\u30A7\u30AF\u30C8\u30E2\u30FC\u30C9\u3078\u623B\u308B");
        break;
    case EditorMode::IntersectionEdit:
        ImGui::TextColored(ImVec4(0.2f,0.9f,1.0f,1), u8"\u4EA4\u5DEE\u70B9\u7DE8\u96C6");
        ImGui::TextDisabled(u8"\u30AF\u30EA\u30C3\u30AF: \u4EA4\u5DEE\u70B9\u9078\u629E");
        ImGui::TextDisabled(u8"Delete: \u4EA4\u5DEE\u70B9\u524A\u9664");
        break;
    case EditorMode::Pathfinding:
        ImGui::TextColored(ImVec4(1.0f,0.7f,0.2f,1), u8"\u7D4C\u8DEF\u63A2\u7D22");
        ImGui::TextDisabled(u8"\u5730\u5F62\u4E0A\u306E\u59CB\u70B9\u3068\u7D42\u70B9\u3092\u8ABF\u6574");
        ImGui::TextDisabled(u8"\u30B0\u30EA\u30C3\u30C9\u9593\u9694\u3068\u6700\u5927\u52FE\u914D\u3092\u8ABF\u6574");
        ImGui::TextDisabled(u8"\u30D7\u30EC\u30D3\u30E5\u30FC\u3092\u78BA\u8A8D\u3057\u3066\u9053\u8DEF\u3078\u9069\u7528");
        break;
    }

    ImGui::Separator();
    bool snapToTerrain = m_snapToTerrain;
    if (ImGui::Checkbox(u8"\u5730\u5F62\u306B\u30B9\u30CA\u30C3\u30D7", &snapToTerrain))
    {
        PushUndoState();
        m_snapToTerrain = snapToTerrain;
        if (m_snapToTerrain && m_terrain && m_terrain->IsReady())
        {
            if (!m_selectedPoints.empty())
            {
                for (const PointRef& pointRef : m_selectedPoints)
                {
                    if (pointRef.roadIndex < 0 ||
                        pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                        continue;
                    Road& road = m_network->roads[pointRef.roadIndex];
                    const int pointIndex = pointRef.pointIndex;
                    if (pointIndex < 0 || pointIndex >= static_cast<int>(road.points.size()))
                        continue;
                    road.points[pointIndex].pos.y =
                        m_terrain->GetHeightAt(road.points[pointIndex].pos.x,
                                               road.points[pointIndex].pos.z);
                }
            }

            for (int intersectionIndex : m_selectedIntersections)
            {
                if (intersectionIndex < 0 ||
                    intersectionIndex >= static_cast<int>(m_network->intersections.size()))
                    continue;
                Intersection& isec = m_network->intersections[intersectionIndex];
                isec.pos.y = m_terrain->GetHeightAt(isec.pos.x, isec.pos.z);
                SyncRoadConnectionsForIntersection(intersectionIndex);
            }
        }
    }
    ImGui::TextDisabled(m_snapToTerrain
        ? u8"\u4E2D\u592E/XZ \u30AE\u30BA\u30E2\u79FB\u52D5\u306F\u5730\u5F62\u3078\u30B9\u30CA\u30C3\u30D7\u3057\u307E\u3059"
        : u8"\u30DD\u30A4\u30F3\u30C8\u3068\u4EA4\u5DEE\u70B9\u306F 3D \u7A7A\u9593\u5185\u3092\u81EA\u7531\u306B\u79FB\u52D5\u3057\u307E\u3059");
    ImGui::Separator();

    ImGui::Text(u8"\u30B0\u30EB\u30FC\u30D7 (%d)", static_cast<int>(m_network->groups.size()));
    if (ImGui::Button(u8"\u30B0\u30EB\u30FC\u30D7\u8FFD\u52A0"))
    {
        PushUndoState();
        const int newIndex = m_network->AddGroup(
            "Group " + std::to_string(m_network->groups.size()));
        m_activeGroup = newIndex;
        m_statusMessage = "Group created";
    }

    ImGui::BeginChild("GroupTree", ImVec2(0, 210), true);
    int groupToDelete = -1;
    for (int gi = 0; gi < static_cast<int>(m_network->groups.size()); ++gi)
    {
        RoadGroup& group = m_network->groups[gi];
        ImGui::PushID(group.id.c_str());

        ImGuiTreeNodeFlags groupFlags =
            ImGuiTreeNodeFlags_DefaultOpen |
            ImGuiTreeNodeFlags_OpenOnArrow |
            ((gi == m_activeGroup) ? ImGuiTreeNodeFlags_Selected : 0);
        const bool groupOpen = ImGui::TreeNodeEx("Group", groupFlags, "%s", group.name.c_str());
        if (ImGui::IsItemClicked())
            m_activeGroup = gi;

        ImGui::SameLine();
        bool groupVisible = group.visible;
        if (ImGui::Checkbox("##visible", &groupVisible))
        {
            PushUndoState();
            group.visible = groupVisible;
        }
        ImGui::SameLine();
        bool groupLocked = group.locked;
        if (ImGui::Checkbox("##locked", &groupLocked))
        {
            PushUndoState();
            group.locked = groupLocked;
        }

        if (ImGui::BeginPopupContextItem("GroupContext"))
        {
            if (ImGui::MenuItem(u8"\u30B0\u30EB\u30FC\u30D7\u524A\u9664", nullptr, false,
                                static_cast<int>(m_network->groups.size()) > 1))
            {
                groupToDelete = gi;
            }
            ImGui::EndPopup();
        }

        if (groupOpen)
        {
            if (ImGui::TreeNodeEx("Roads", ImGuiTreeNodeFlags_DefaultOpen,
                                  u8"\u9053\u8DEF (%d)",
                                  static_cast<int>(std::count_if(
                                      m_network->roads.begin(),
                                      m_network->roads.end(),
                                      [&group](const Road& road)
                                      {
                                          return road.groupId == group.id;
                                      }))))
            {
                for (int i = 0; i < static_cast<int>(m_network->roads.size()); ++i)
                {
                    Road& road = m_network->roads[i];
                    if (road.groupId != group.id)
                        continue;

                    const bool selected = IsRoadSelected(i);
                    if (ImGui::Selectable(road.name.c_str(), selected))
                    {
                        if (ImGui::GetIO().KeyCtrl)
                            ToggleRoadSelection(i);
                        else
                            SelectSingleRoad(i);
                        ClearPointSelection();
                        ClearIntersectionSelection();
                        m_activeIntersection = -1;
                        m_activeGroup = gi;
                    }
                    if (selected && ImGui::BeginPopupContextItem())
                    {
                        if (ImGui::MenuItem(u8"\u9053\u8DEF\u524A\u9664"))
                        {
                            PushUndoState();
                            m_network->RemoveRoad(i);
                            SanitizeSelection();
                        }
                        ImGui::EndPopup();
                    }
                }
                ImGui::TreePop();
            }

            if (ImGui::TreeNodeEx("Intersections", ImGuiTreeNodeFlags_DefaultOpen,
                                  u8"\u4EA4\u5DEE\u70B9 (%d)",
                                  static_cast<int>(std::count_if(
                                      m_network->intersections.begin(),
                                      m_network->intersections.end(),
                                      [&group](const Intersection& intersection)
                                      {
                                          return intersection.groupId == group.id;
                                      }))))
            {
                for (int i = 0; i < static_cast<int>(m_network->intersections.size()); ++i)
                {
                    Intersection& isec = m_network->intersections[i];
                    if (isec.groupId != group.id)
                        continue;

                    const bool selected = (i == m_activeIntersection);
            if (ImGui::Selectable(isec.name.c_str(), selected))
            {
                SelectSingleIntersection(i);
                ClearRoadSelection();
                ClearPointSelection();
                m_activeGroup = gi;
            }
                    if (selected && ImGui::BeginPopupContextItem())
                    {
                        if (ImGui::MenuItem(u8"\u4EA4\u5DEE\u70B9\u524A\u9664"))
                        {
                            PushUndoState();
                            m_network->RemoveIntersection(i);
                            if (m_activeIntersection >= static_cast<int>(m_network->intersections.size()))
                                m_activeIntersection = -1;
                        }
                        ImGui::EndPopup();
                    }
                }
                ImGui::TreePop();
            }
            ImGui::TreePop();
        }

        ImGui::PopID();
    }
    ImGui::EndChild();
    if (groupToDelete >= 0)
    {
        PushUndoState();
        m_network->RemoveGroup(groupToDelete);
        SanitizeSelection();
        m_statusMessage = "Group deleted";
    }

    if (m_activeGroup >= 0 &&
        m_activeGroup < static_cast<int>(m_network->groups.size()))
    {
        RoadGroup& group = m_network->groups[m_activeGroup];
        char groupNameBuf[128] = {};
        strncpy_s(groupNameBuf, sizeof(groupNameBuf), group.name.c_str(), _TRUNCATE);
        ImGui::Text(u8"\u30A2\u30AF\u30C6\u30A3\u30D6\u30B0\u30EB\u30FC\u30D7");
        if (ImGui::InputText(u8"\u540D\u524D##group", groupNameBuf, sizeof(groupNameBuf)))
        {
            PushUndoState();
            group.name = groupNameBuf;
        }
        bool groupVisible = group.visible;
        if (ImGui::Checkbox(u8"\u8868\u793A##group", &groupVisible))
        {
            PushUndoState();
            group.visible = groupVisible;
        }
        ImGui::SameLine();
        bool groupLocked = group.locked;
        if (ImGui::Checkbox(u8"\u30ED\u30C3\u30AF##group", &groupLocked))
        {
            PushUndoState();
            group.locked = groupLocked;
        }
    }

    if (m_activeRoad >= 0 &&
        m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        Road& road = m_network->roads[m_activeRoad];
        ImGui::Separator();
        ImGui::Text(u8"\u9053\u8DEF");
        if (m_selectedRoads.size() > 1)
            ImGui::TextDisabled(u8"\u9078\u629E\u4E2D\u306E\u9053\u8DEF: %d", static_cast<int>(m_selectedRoads.size()));

        if (m_selectedRoads.size() == 2)
        {
            if (ImGui::Button(u8"\u9078\u629E\u4E2D\u306E\u9053\u8DEF\u3092\u30DE\u30FC\u30B8"))
                MergeSelectedRoads();
        }

        char roadNameBuf[128] = {};
        strncpy_s(roadNameBuf, sizeof(roadNameBuf), road.name.c_str(), _TRUNCATE);
        if (ImGui::InputText(u8"\u540D\u524D##road", roadNameBuf, sizeof(roadNameBuf)))
        {
            PushUndoState();
            road.name = roadNameBuf;
        }

        float laneWidth = road.laneWidth;
        if (ImGui::InputFloat(u8"\u8ECA\u7DDA\u5E45", &laneWidth, 0.1f, 1.0f, "%.2f"))
        {
            PushUndoState();
            road.laneWidth = (std::max)(0.1f, laneWidth);
        }

        int laneLeft = road.laneLeft;
        if (ImGui::InputInt(u8"\u5DE6\u8ECA\u7DDA\u6570", &laneLeft))
        {
            PushUndoState();
            road.laneLeft = (std::max)(0, laneLeft);
        }

        int laneRight = road.laneRight;
        if (ImGui::InputInt(u8"\u53F3\u8ECA\u7DDA\u6570", &laneRight))
        {
            PushUndoState();
            road.laneRight = (std::max)(0, laneRight);
        }

        const RoadGroup* currentGroup = m_network->FindGroupById(road.groupId);
        const char* preview = currentGroup ? currentGroup->name.c_str() : "<none>";
        if (ImGui::BeginCombo(u8"\u30B0\u30EB\u30FC\u30D7##road", preview))
        {
            for (int gi = 0; gi < static_cast<int>(m_network->groups.size()); ++gi)
            {
                const RoadGroup& group = m_network->groups[gi];
                const bool selected = (road.groupId == group.id);
                if (ImGui::Selectable(group.name.c_str(), selected))
                {
                    PushUndoState();
                    road.groupId = group.id;
                    m_activeGroup = gi;
                }
                if (selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    }

    // Selected point info
    PointRef selectedPoint;
    if (GetPrimarySelectedPoint(selectedPoint) &&
        selectedPoint.roadIndex >= 0 &&
        selectedPoint.roadIndex < static_cast<int>(m_network->roads.size()))
    {
        Road& road = m_network->roads[selectedPoint.roadIndex];
        if (selectedPoint.pointIndex < static_cast<int>(road.points.size()))
        {
            RoadPoint& rp = road.points[selectedPoint.pointIndex];
            ImGui::Text(u8"\u30DD\u30A4\u30F3\u30C8 %d:%d", selectedPoint.roadIndex, selectedPoint.pointIndex);
            float pointPos[3] = { rp.pos.x, rp.pos.y, rp.pos.z };
            if (ImGui::InputFloat3(u8"\u4F4D\u7F6E", pointPos))
            {
                PushUndoState();
                rp.pos = { pointPos[0], pointPos[1], pointPos[2] };
                if (m_snapToTerrain && m_terrain && m_terrain->IsReady())
                    rp.pos.y = m_terrain->GetHeightAt(rp.pos.x, rp.pos.z);
            }
            if (IsSelectedRoadEndpoint())
            {
                std::string connectionId;
                if (GetSelectedRoadConnectionId(connectionId))
                    ImGui::Text(u8"\u63A5\u7D9A\u5148: %s", connectionId.c_str());
                else
                    ImGui::TextDisabled(u8"\u63A5\u7D9A\u5148: \u306A\u3057");

                if (ImGui::Button(u8"\u7AEF\u70B9\u306E\u63A5\u7D9A\u3092\u89E3\u9664"))
                {
                    PushUndoState();
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
        ImGui::Text(u8"\u4EA4\u5DEE\u70B9");
        if (ImGui::InputText(u8"\u540D\u524D##isec", nameBuf, sizeof(nameBuf)))
        {
            PushUndoState();
            isec.name = nameBuf;
        }
        const RoadGroup* currentGroup = m_network->FindGroupById(isec.groupId);
        const char* preview = currentGroup ? currentGroup->name.c_str() : "<none>";
        if (ImGui::BeginCombo(u8"\u30B0\u30EB\u30FC\u30D7##isec", preview))
        {
            for (int gi = 0; gi < static_cast<int>(m_network->groups.size()); ++gi)
            {
                const RoadGroup& group = m_network->groups[gi];
                const bool selected = (isec.groupId == group.id);
                if (ImGui::Selectable(group.name.c_str(), selected))
                {
                    PushUndoState();
                    isec.groupId = group.id;
                    m_activeGroup = gi;
                }
                if (selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        if (ImGui::Combo("Type##isec", &typeIndex, kIntersectionTypes, IM_ARRAYSIZE(kIntersectionTypes)))
        {
            PushUndoState();
            isec.type = kIntersectionTypes[typeIndex];
        }
        float isecPos[3] = { isec.pos.x, isec.pos.y, isec.pos.z };
        if (ImGui::InputFloat3("Pos##isec", isecPos))
        {
            PushUndoState();
            isec.pos = { isecPos[0], isecPos[1], isecPos[2] };
            if (m_snapToTerrain && m_terrain && m_terrain->IsReady())
                isec.pos.y = m_terrain->GetHeightAt(isec.pos.x, isec.pos.z);
            SyncRoadConnectionsForIntersection(m_activeIntersection);
        }
        float isecRadius = isec.radius;
        if (ImGui::SliderFloat("Radius##isec", &isecRadius, 1.0f, 20.0f))
        {
            PushUndoState();
            isec.radius = isecRadius;
        }
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
