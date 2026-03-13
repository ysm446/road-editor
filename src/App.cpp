#include "App.h"
#include "resource.h"
#include "imgui.h"
#include "imgui_impl_win32.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <commdlg.h>
#include <DirectXMath.h>
#include <fstream>
#include <filesystem>
#include <limits>
#include <queue>

#include <nlohmann/json.hpp>

// Forward declaration required by imgui_impl_win32
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(
    HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

using namespace DirectX;

namespace
{
constexpr const char* kViewSettingsPath = "data/view_settings.json";
namespace fs = std::filesystem;

struct PathGridConfig
{
    float minX = 0.0f;
    float minZ = 0.0f;
    float maxX = 0.0f;
    float maxZ = 0.0f;
    float step = 5.0f;
    int cols = 0;
    int rows = 0;
};

float SamplePathHeightAt(const Terrain* terrain, float x, float z);

float DistanceXZ(XMFLOAT3 a, XMFLOAT3 b)
{
    const float dx = b.x - a.x;
    const float dz = b.z - a.z;
    return sqrtf(dx * dx + dz * dz);
}

float Distance2D(XMFLOAT2 a, XMFLOAT2 b)
{
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    return sqrtf(dx * dx + dy * dy);
}

bool WorldToScreenPoint(XMFLOAT3 world, XMMATRIX viewProj, int vpW, int vpH, XMFLOAT2& outScreen)
{
    const XMVECTOR clip = XMVector4Transform(XMVectorSet(world.x, world.y, world.z, 1.0f), viewProj);
    const float w = XMVectorGetW(clip);
    if (w <= 1e-5f)
        return false;

    const float invW = 1.0f / w;
    const float ndcX = XMVectorGetX(clip) * invW;
    const float ndcY = XMVectorGetY(clip) * invW;
    const float ndcZ = XMVectorGetZ(clip) * invW;
    if (ndcZ < 0.0f || ndcZ > 1.0f)
        return false;

    outScreen.x = (ndcX * 0.5f + 0.5f) * static_cast<float>(vpW);
    outScreen.y = (-ndcY * 0.5f + 0.5f) * static_cast<float>(vpH);
    return true;
}

float Distance3D(XMFLOAT3 a, XMFLOAT3 b)
{
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float dz = b.z - a.z;
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

float DistancePointToSegment3D(XMFLOAT3 point, XMFLOAT3 segA, XMFLOAT3 segB)
{
    const XMVECTOR p = XMLoadFloat3(&point);
    const XMVECTOR a = XMLoadFloat3(&segA);
    const XMVECTOR b = XMLoadFloat3(&segB);
    const XMVECTOR ab = XMVectorSubtract(b, a);
    const float abLenSq = XMVectorGetX(XMVector3LengthSq(ab));
    if (abLenSq <= 1e-6f)
        return Distance3D(point, segA);

    const float t = std::clamp(
        XMVectorGetX(XMVector3Dot(XMVectorSubtract(p, a), ab)) / abLenSq,
        0.0f,
        1.0f);
    XMFLOAT3 projected;
    XMStoreFloat3(&projected, XMVectorAdd(a, XMVectorScale(ab, t)));
    return Distance3D(point, projected);
}

void SimplifyDouglasPeuckerRecursive(const std::vector<XMFLOAT3>& points,
                                     int startIndex,
                                     int endIndex,
                                     float tolerance,
                                     std::vector<unsigned char>& keepFlags)
{
    if (endIndex <= startIndex + 1)
        return;

    float maxDistance = -1.0f;
    int maxIndex = -1;
    for (int i = startIndex + 1; i < endIndex; ++i)
    {
        const float distance = DistancePointToSegment3D(
            points[i], points[startIndex], points[endIndex]);
        if (distance > maxDistance)
        {
            maxDistance = distance;
            maxIndex = i;
        }
    }

    if (maxIndex >= 0 && maxDistance > tolerance)
    {
        keepFlags[maxIndex] = 1;
        SimplifyDouglasPeuckerRecursive(points, startIndex, maxIndex, tolerance, keepFlags);
        SimplifyDouglasPeuckerRecursive(points, maxIndex, endIndex, tolerance, keepFlags);
    }
}

std::vector<XMFLOAT3> SimplifyDouglasPeucker(const std::vector<XMFLOAT3>& points, float tolerance)
{
    if (points.size() <= 2)
        return points;

    std::vector<unsigned char> keepFlags(points.size(), 0);
    keepFlags.front() = 1;
    keepFlags.back() = 1;
    SimplifyDouglasPeuckerRecursive(
        points,
        0,
        static_cast<int>(points.size()) - 1,
        (std::max)(tolerance, 0.01f),
        keepFlags);

    std::vector<XMFLOAT3> simplified;
    simplified.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (keepFlags[i] != 0)
            simplified.push_back(points[i]);
    }
    return simplified;
}

std::vector<XMFLOAT3> SmoothPathForRoadControls(std::vector<XMFLOAT3> points,
                                                const Terrain* terrain,
                                                float gridStep)
{
    if (points.size() < 3)
        return points;

    std::vector<XMFLOAT3> deduped;
    deduped.reserve(points.size());
    deduped.push_back(points.front());
    for (size_t i = 1; i < points.size(); ++i)
    {
        if (Distance3D(points[i], deduped.back()) > 0.05f)
            deduped.push_back(points[i]);
    }
    points = std::move(deduped);
    if (points.size() < 3)
        return points;

    constexpr int kChaikinIterations = 2;
    for (int iteration = 0; iteration < kChaikinIterations; ++iteration)
    {
        std::vector<XMFLOAT3> refined;
        refined.reserve(points.size() * 2);
        refined.push_back(points.front());

        for (size_t i = 0; i + 1 < points.size(); ++i)
        {
            const XMFLOAT3 q = Lerp3(points[i], points[i + 1], 0.25f);
            const XMFLOAT3 r = Lerp3(points[i], points[i + 1], 0.75f);
            refined.push_back(q);
            refined.push_back(r);
        }

        refined.push_back(points.back());
        points = std::move(refined);
    }

    for (XMFLOAT3& point : points)
        point.y = SamplePathHeightAt(terrain, point.x, point.z);

    std::vector<XMFLOAT3> controls = SimplifyDouglasPeucker(points, gridStep * 0.75f);
    if (controls.size() > 2)
    {
        const float minSpacing = (std::max)(gridStep * 0.6f, 1.0f);
        std::vector<XMFLOAT3> filtered;
        filtered.reserve(controls.size());
        filtered.push_back(controls.front());
        for (size_t i = 1; i + 1 < controls.size(); ++i)
        {
            if (DistanceXZ(controls[i], filtered.back()) >= minSpacing)
                filtered.push_back(controls[i]);
        }
        filtered.push_back(controls.back());
        controls = std::move(filtered);
    }

    if (controls.size() < 2)
        return { points.front(), points.back() };

    controls.front() = points.front();
    controls.back() = points.back();
    return controls;
}

float SamplePathHeightAt(const Terrain* terrain, float x, float z)
{
    if (terrain && terrain->IsReady())
        return terrain->GetHeightAt(x, z);
    return 0.0f;
}

bool BuildPathGridConfig(const Terrain* terrain,
                         XMFLOAT3 startPos,
                         XMFLOAT3 endPos,
                         float requestedStep,
                         PathGridConfig& outConfig)
{
    outConfig.step = (std::max)(requestedStep, 1.0f);

    if (terrain && terrain->IsReady())
    {
        const float halfWidth =
            static_cast<float>(terrain->GetMeshW() - 1) * terrain->horizontalScaleX * 0.5f;
        const float halfDepth =
            static_cast<float>(terrain->GetMeshH() - 1) * terrain->horizontalScaleZ * 0.5f;

        outConfig.minX = terrain->offsetX - halfWidth;
        outConfig.maxX = terrain->offsetX + halfWidth;
        outConfig.minZ = terrain->offsetZ - halfDepth;
        outConfig.maxZ = terrain->offsetZ + halfDepth;
    }
    else
    {
        const float margin = 100.0f;
        outConfig.minX = (std::min)(startPos.x, endPos.x) - margin;
        outConfig.maxX = (std::max)(startPos.x, endPos.x) + margin;
        outConfig.minZ = (std::min)(startPos.z, endPos.z) - margin;
        outConfig.maxZ = (std::max)(startPos.z, endPos.z) + margin;
    }

    const float width = outConfig.maxX - outConfig.minX;
    const float depth = outConfig.maxZ - outConfig.minZ;
    if (width <= 0.0f || depth <= 0.0f)
        return false;

    outConfig.cols = (std::max)(2, static_cast<int>(ceilf(width / outConfig.step)) + 1);
    outConfig.rows = (std::max)(2, static_cast<int>(ceilf(depth / outConfig.step)) + 1);
    return true;
}

XMFLOAT3 GridToWorld(const PathGridConfig& grid, int col, int row, const Terrain* terrain)
{
    XMFLOAT3 pos =
    {
        grid.minX + static_cast<float>(col) * grid.step,
        0.0f,
        grid.minZ + static_cast<float>(row) * grid.step
    };
    pos.y = SamplePathHeightAt(terrain, pos.x, pos.z);
    return pos;
}

bool WorldToGrid(const PathGridConfig& grid, XMFLOAT3 worldPos, int& outCol, int& outRow)
{
    const float colF = (worldPos.x - grid.minX) / grid.step;
    const float rowF = (worldPos.z - grid.minZ) / grid.step;
    outCol = static_cast<int>(std::round(colF));
    outRow = static_cast<int>(std::round(rowF));
    return outCol >= 0 && outCol < grid.cols && outRow >= 0 && outRow < grid.rows;
}

XMFLOAT3 InterpolateContourPoint(XMFLOAT3 a, XMFLOAT3 b, float level)
{
    const float denom = b.y - a.y;
    float t = 0.5f;
    if (fabsf(denom) > 1e-6f)
        t = std::clamp((level - a.y) / denom, 0.0f, 1.0f);

    return
    {
        a.x + (b.x - a.x) * t,
        level,
        a.z + (b.z - a.z) * t
    };
}

bool IntersectRayWithGroundPlane(XMFLOAT3 rayOrigin, XMFLOAT3 rayDir, XMFLOAT3& outHit)
{
    const XMVECTOR ro = XMLoadFloat3(&rayOrigin);
    const XMVECTOR rd = XMVector3Normalize(XMLoadFloat3(&rayDir));
    const XMVECTOR planePoint = XMVectorZero();
    const XMVECTOR planeNormal = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);

    const float denom = XMVectorGetX(XMVector3Dot(rd, planeNormal));
    if (fabsf(denom) < 1e-5f)
        return false;

    const float t = XMVectorGetX(XMVector3Dot(XMVectorSubtract(planePoint, ro), planeNormal)) / denom;
    if (t < 0.0f)
        return false;

    XMStoreFloat3(&outHit, XMVectorAdd(ro, XMVectorScale(rd, t)));
    return true;
}

void DrawViewAxisGizmo(XMMATRIX view)
{
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const ImVec2 center(
        viewport->WorkPos.x + 58.0f,
        viewport->WorkPos.y + viewport->WorkSize.y - 92.0f);
    const float axisLength = 28.0f;

    struct AxisLine
    {
        const char* label;
        ImU32 color;
        XMFLOAT3 dir;
        float depth;
    };

    std::array<AxisLine, 3> axes =
    {{
        { "X", IM_COL32(255, 90, 90, 255), {}, 0.0f },
        { "Y", IM_COL32(90, 255, 120, 255), {}, 0.0f },
        { "Z", IM_COL32(90, 160, 255, 255), {}, 0.0f }
    }};

    const std::array<XMVECTOR, 3> basis =
    {{
        XMVectorSet(1.0f, 0.0f, 0.0f, 0.0f),
        XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f),
        XMVectorSet(0.0f, 0.0f, 1.0f, 0.0f)
    }};

    for (size_t i = 0; i < axes.size(); ++i)
    {
        XMFLOAT3 dir;
        XMStoreFloat3(&dir, XMVector3TransformNormal(basis[i], view));
        axes[i].dir = dir;
        axes[i].depth = dir.z;
    }

    std::sort(axes.begin(), axes.end(),
        [](const AxisLine& a, const AxisLine& b)
        {
            return a.depth < b.depth;
        });

    dl->AddCircleFilled(center, 4.0f, IM_COL32(235, 235, 235, 220), 16);

    for (const AxisLine& axis : axes)
    {
        const ImVec2 end(
            center.x + axis.dir.x * axisLength,
            center.y - axis.dir.y * axisLength);
        const float thickness = (axis.depth >= 0.0f) ? 2.6f : 1.8f;
        dl->AddLine(center, end, axis.color, thickness);
        dl->AddText(ImVec2(end.x + 8.0f, end.y - 8.0f), axis.color, axis.label);
    }
}

XMFLOAT3 DirectionFromAzimuthElevation(float azimuth, float elevation)
{
    const float cosElevation = cosf(elevation);
    return
    {
        cosElevation * sinf(azimuth),
        sinf(elevation),
        cosElevation * cosf(azimuth)
    };
}

fs::path GetProjectDirectory(const char* projectPath)
{
    if (projectPath == nullptr || projectPath[0] == '\0')
        return fs::current_path();

    std::error_code ec;
    fs::path path = fs::absolute(fs::path(projectPath), ec);
    if (ec)
        path = fs::path(projectPath);

    if (path.has_parent_path())
    {
        fs::path parent = path.parent_path();
        const fs::path normalized = fs::weakly_canonical(parent, ec);
        return ec ? parent : normalized;
    }

    const fs::path current = fs::current_path(ec);
    return ec ? fs::path(".") : current;
}

std::string MakePathRelativeToProject(const char* projectPath, const char* targetPath)
{
    if (targetPath == nullptr || targetPath[0] == '\0')
        return std::string();

    const fs::path target(targetPath);
    if (!target.is_absolute())
        return target.generic_string();

    const fs::path projectDir = GetProjectDirectory(projectPath);
    std::error_code ec;
    const fs::path relative = fs::relative(target, projectDir, ec);
    return ec ? target.generic_string() : relative.generic_string();
}

std::string ResolvePathFromProject(const char* projectPath, const std::string& storedPath)
{
    if (storedPath.empty())
        return std::string();

    fs::path path(storedPath);
    if (path.is_absolute())
        return path.string();

    const fs::path projectDir = GetProjectDirectory(projectPath);
    std::error_code ec;
    const fs::path resolved = fs::weakly_canonical(projectDir / path, ec);
    return ec ? (projectDir / path).string() : resolved.string();
}

std::string NormalizePathString(const char* path)
{
    if (path == nullptr || path[0] == '\0')
        return std::string();

    std::error_code ec;
    const fs::path normalized = fs::weakly_canonical(fs::path(path), ec);
    return ec ? fs::path(path).string() : normalized.string();
}
}

// ---------------------------------------------------------------------------
// Open a Windows file-picker and write the result into buf (MAX_PATH chars).
// Returns true if the user picked a file.
// ---------------------------------------------------------------------------
static bool OpenFileDialog(HWND owner, char* buf, DWORD bufSize,
                           const char* filter, const char* title)
{
    // Use a separate temp buffer so an invalid existing path won't block the dialog
    char tmp[MAX_PATH] = {};

    OPENFILENAMEA ofn  = {};
    ofn.lStructSize    = sizeof(ofn);
    ofn.hwndOwner      = owner;
    ofn.lpstrFilter    = filter;
    ofn.lpstrFile      = tmp;
    ofn.nMaxFile       = MAX_PATH;
    ofn.lpstrTitle     = title;
    ofn.Flags          = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;

    if (GetOpenFileNameA(&ofn))
    {
        strncpy_s(buf, bufSize, tmp, _TRUNCATE);
        return true;
    }
    return false;
}

static bool SaveFileDialog(HWND owner, char* buf, DWORD bufSize,
                           const char* filter, const char* title)
{
    char tmp[MAX_PATH] = {};
    strncpy_s(tmp, bufSize, buf, _TRUNCATE);

    OPENFILENAMEA ofn  = {};
    ofn.lStructSize    = sizeof(ofn);
    ofn.hwndOwner      = owner;
    ofn.lpstrFilter    = filter;
    ofn.lpstrFile      = tmp;
    ofn.nMaxFile       = MAX_PATH;
    ofn.lpstrTitle     = title;
    ofn.Flags          = OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR | OFN_OVERWRITEPROMPT;

    if (GetSaveFileNameA(&ofn))
    {
        strncpy_s(buf, bufSize, tmp, _TRUNCATE);
        return true;
    }
    return false;
}

void App::ApplyTerrainSettings()
{
    m_terrain->meshSubdivW = m_loadResW;
    m_terrain->meshSubdivH = m_loadResH;

    const int cellsW = (m_loadResW > 0) ? m_loadResW : (m_terrain->GetRawW() - 1);
    const int cellsH = (m_loadResH > 0) ? m_loadResH : (m_terrain->GetRawH() - 1);
    m_terrain->horizontalScaleX = m_loadWidthM / static_cast<float>((cellsW > 0) ? cellsW : 1);
    m_terrain->horizontalScaleZ = m_loadDepthM / static_cast<float>((cellsH > 0) ? cellsH : 1);
    m_terrain->heightScale      = m_loadHeightM;
    m_terrain->offsetX          = m_loadOffsetX;
    m_terrain->offsetY          = m_loadOffsetY;
    m_terrain->offsetZ          = m_loadOffsetZ;
    m_terrain->Rebuild(m_d3d->GetDevice());
    RebuildContourCache();
}

void App::LoadViewSettings()
{
    m_showRoadNames = false;
    m_showIntersectionNames = true;
    m_showRoadPreviewMetrics = false;
    m_showRoadGradeGradient = false;
    m_showGrid = true;
    m_showCameraWindow = true;
    m_showTerrainWindow = true;
    m_showRoadEditorWindow = true;
    m_showPropertiesWindow = true;
    m_showFps = true;
    m_roadGradeRedThresholdPercent = 12.0f;
    m_showContours = false;
    m_contourInterval = 5.0f;
    m_gridBaseScale = 1.0f;
    m_gridFadeDistance = 1200.0f;
    m_roadLineThickness = 2.0f;
    m_previewCurveThickness = 2.0f;
    m_selectedRoadLineThickness = 3.0f;
    m_roadVertexScreenRadius = 3.0f;
    m_intersectionScreenGizmoRadius = 10.0f;
    m_roadVertexColor = { 160.0f / 255.0f, 160.0f / 255.0f, 160.0f / 255.0f };
    m_selectedRoadColor = { 1.0f, 1.0f, 1.0f };
    m_intersectionCircleColor = { 80.0f / 255.0f, 240.0f / 255.0f, 1.0f };
    m_contourColor = { 0.18f, 0.18f, 0.18f };
    m_backgroundColor = { 0.12f, 0.12f, 0.14f };
    m_recentProjectPaths.clear();

    try
    {
        std::ifstream ifs(kViewSettingsPath);
        if (ifs)
        {
            nlohmann::json root;
            ifs >> root;
            m_showRoadNames = root.value("showRoadNames", false);
            m_showIntersectionNames = root.value("showIntersectionNames", true);
            m_showRoadPreviewMetrics = root.value("showRoadPreviewMetrics", false);
            m_showRoadGradeGradient = root.value("showRoadGradeGradient", false);
            m_showGrid = root.value("showGrid", true);
            m_showCameraWindow = root.value("showCameraWindow", true);
            m_showTerrainWindow = root.value("showTerrainWindow", true);
            m_showRoadEditorWindow = root.value("showRoadEditorWindow", true);
            m_showPropertiesWindow = root.value("showPropertiesWindow", true);
            m_showFps = root.value("showFps", true);
            m_roadGradeRedThresholdPercent = root.value("roadGradeRedThresholdPercent", 12.0f);
            m_showContours = root.value("showContours", false);
            m_contourInterval = root.value("contourInterval", 5.0f);
            m_gridBaseScale = root.value("gridBaseScale", 1.0f);
            m_gridFadeDistance = root.value("gridFadeDistance", 1200.0f);
            m_roadLineThickness = root.value("roadLineThickness", 2.0f);
            m_previewCurveThickness = root.value("previewCurveThickness", 2.0f);
            m_selectedRoadLineThickness = root.value("selectedRoadLineThickness", 3.0f);
            m_roadVertexScreenRadius = root.value("roadVertexScreenRadius", 3.0f);
            m_intersectionScreenGizmoRadius = root.value("intersectionScreenGizmoRadius", 10.0f);
            if (root.contains("roadVertexColor") && root["roadVertexColor"].is_array() && root["roadVertexColor"].size() == 3)
            {
                m_roadVertexColor.x = root["roadVertexColor"][0].get<float>();
                m_roadVertexColor.y = root["roadVertexColor"][1].get<float>();
                m_roadVertexColor.z = root["roadVertexColor"][2].get<float>();
            }
            if (root.contains("selectedRoadColor") && root["selectedRoadColor"].is_array() && root["selectedRoadColor"].size() == 3)
            {
                m_selectedRoadColor.x = root["selectedRoadColor"][0].get<float>();
                m_selectedRoadColor.y = root["selectedRoadColor"][1].get<float>();
                m_selectedRoadColor.z = root["selectedRoadColor"][2].get<float>();
            }
            if (root.contains("intersectionCircleColor") && root["intersectionCircleColor"].is_array() && root["intersectionCircleColor"].size() == 3)
            {
                m_intersectionCircleColor.x = root["intersectionCircleColor"][0].get<float>();
                m_intersectionCircleColor.y = root["intersectionCircleColor"][1].get<float>();
                m_intersectionCircleColor.z = root["intersectionCircleColor"][2].get<float>();
            }
            if (root.contains("contourColor") && root["contourColor"].is_array() && root["contourColor"].size() == 3)
            {
                m_contourColor.x = root["contourColor"][0].get<float>();
                m_contourColor.y = root["contourColor"][1].get<float>();
                m_contourColor.z = root["contourColor"][2].get<float>();
            }
            if (root.contains("backgroundColor") && root["backgroundColor"].is_array() && root["backgroundColor"].size() == 3)
            {
                m_backgroundColor.x = root["backgroundColor"][0].get<float>();
                m_backgroundColor.y = root["backgroundColor"][1].get<float>();
                m_backgroundColor.z = root["backgroundColor"][2].get<float>();
            }
            if (root.contains("recentProjectPaths") && root["recentProjectPaths"].is_array())
            {
                for (const auto& item : root["recentProjectPaths"])
                {
                    if (!item.is_string())
                        continue;
                    const std::string normalized = NormalizePathString(item.get<std::string>().c_str());
                    if (!normalized.empty())
                        m_recentProjectPaths.push_back(normalized);
                    if (m_recentProjectPaths.size() >= 8)
                        break;
                }
            }
        }
    }
    catch (...)
    {
    }

    m_editor.SetShowRoadNames(m_showRoadNames);
    m_editor.SetShowIntersectionNames(m_showIntersectionNames);
    m_editor.SetShowRoadPreviewMetrics(m_showRoadPreviewMetrics);
    m_editor.SetShowRoadGradeGradient(m_showRoadGradeGradient);
    m_editor.SetRoadGradeRedThresholdPercent(m_roadGradeRedThresholdPercent);
    m_editor.SetRoadLineThickness(m_roadLineThickness);
    m_editor.SetPreviewCurveThickness(m_previewCurveThickness);
    m_editor.SetSelectedRoadLineThickness(m_selectedRoadLineThickness);
    m_editor.SetRoadVertexScreenRadius(m_roadVertexScreenRadius);
    m_editor.SetRoadVertexColor(m_roadVertexColor);
    m_editor.SetSelectedRoadColor(m_selectedRoadColor);
    m_editor.SetIntersectionScreenGizmoRadius(m_intersectionScreenGizmoRadius);
    m_editor.SetIntersectionCircleColor(m_intersectionCircleColor);
    RebuildContourCache();
}

void App::SaveViewSettings() const
{
    try
    {
        nlohmann::json root =
        {
            { "showRoadNames", m_showRoadNames },
            { "showIntersectionNames", m_showIntersectionNames },
            { "showRoadPreviewMetrics", m_showRoadPreviewMetrics },
            { "showRoadGradeGradient", m_showRoadGradeGradient },
            { "showGrid", m_showGrid },
            { "showCameraWindow", m_showCameraWindow },
            { "showTerrainWindow", m_showTerrainWindow },
            { "showRoadEditorWindow", m_showRoadEditorWindow },
            { "showPropertiesWindow", m_showPropertiesWindow },
            { "showFps", m_showFps },
            { "roadGradeRedThresholdPercent", m_roadGradeRedThresholdPercent },
            { "showContours", m_showContours },
            { "contourInterval", m_contourInterval },
            { "gridBaseScale", m_gridBaseScale },
            { "gridFadeDistance", m_gridFadeDistance },
            { "roadLineThickness", m_roadLineThickness },
            { "previewCurveThickness", m_previewCurveThickness },
            { "selectedRoadLineThickness", m_selectedRoadLineThickness },
            { "roadVertexScreenRadius", m_roadVertexScreenRadius },
            { "intersectionScreenGizmoRadius", m_intersectionScreenGizmoRadius },
            { "roadVertexColor", { m_roadVertexColor.x, m_roadVertexColor.y, m_roadVertexColor.z } },
            { "selectedRoadColor", { m_selectedRoadColor.x, m_selectedRoadColor.y, m_selectedRoadColor.z } },
            { "intersectionCircleColor", { m_intersectionCircleColor.x, m_intersectionCircleColor.y, m_intersectionCircleColor.z } },
            { "contourColor", { m_contourColor.x, m_contourColor.y, m_contourColor.z } },
            { "backgroundColor", { m_backgroundColor.x, m_backgroundColor.y, m_backgroundColor.z } },
            { "recentProjectPaths", m_recentProjectPaths }
        };

        std::ofstream ofs(kViewSettingsPath);
        if (ofs)
            ofs << root.dump(2);
    }
    catch (...)
    {
    }
}

void App::SetStatusMessage(const std::string& message)
{
    m_statusMessage = message;
}

void App::AddRecentProjectPath(const char* path)
{
    const std::string normalized = NormalizePathString(path);
    if (normalized.empty())
        return;

    m_recentProjectPaths.erase(
        std::remove(m_recentProjectPaths.begin(), m_recentProjectPaths.end(), normalized),
        m_recentProjectPaths.end());
    m_recentProjectPaths.insert(m_recentProjectPaths.begin(), normalized);
    if (m_recentProjectPaths.size() > 8)
        m_recentProjectPaths.resize(8);
}

void App::ResetPathfindingState()
{
    m_pathfinding = PathfindingState();
    m_prevPathPickLButton = false;
}

bool App::SyncPathfindingEndpointsFromSelectedRoad()
{
    int roadIndex = -1;
    if (!m_editor.GetPrimaryRoadForPathfinding(roadIndex) ||
        roadIndex < 0 ||
        roadIndex >= static_cast<int>(m_roadNetwork.roads.size()))
    {
        SetStatusMessage("Select one road before entering pathfinding");
        return false;
    }

    const Road& road = m_roadNetwork.roads[roadIndex];
    if (road.points.size() < 2)
    {
        SetStatusMessage("Selected road needs at least two points");
        return false;
    }

    m_pathfinding.startPos = road.points.front().pos;
    m_pathfinding.endPos = road.points.back().pos;
    m_pathfinding.hasStart = true;
    m_pathfinding.hasEnd = true;
    m_pathfinding.draggingStart = false;
    m_pathfinding.draggingEnd = false;
    m_pathfinding.previewPath.clear();
    if (!ComputePathfindingPreview())
        return false;

    SetStatusMessage("Pathfinding preview synced from selected road");
    return true;
}

void App::RebuildContourCache()
{
    m_contourSegments.clear();

    if (!m_showContours || !m_terrain || !m_terrain->IsReady())
        return;

    const float interval = std::clamp(m_contourInterval, 0.5f, 1000.0f);
    const int cols = m_terrain->GetMeshW();
    const int rows = m_terrain->GetMeshH();
    if (cols < 2 || rows < 2)
        return;

    const float halfWidth = static_cast<float>(cols - 1) * m_terrain->horizontalScaleX * 0.5f;
    const float halfDepth = static_cast<float>(rows - 1) * m_terrain->horizontalScaleZ * 0.5f;
    const float minX = m_terrain->offsetX - halfWidth;
    const float minZ = m_terrain->offsetZ - halfDepth;

    auto sampleVertex = [this, minX, minZ](int col, int row)
    {
        const float x = minX + static_cast<float>(col) * m_terrain->horizontalScaleX;
        const float z = minZ + static_cast<float>(row) * m_terrain->horizontalScaleZ;
        return XMFLOAT3{ x, m_terrain->GetHeightAt(x, z), z };
    };

    auto appendSegmentsForCell = [this](XMFLOAT3 p0, XMFLOAT3 p1, XMFLOAT3 p2, XMFLOAT3 p3, float level)
    {
        std::vector<XMFLOAT3> intersections;
        intersections.reserve(4);

        auto tryEdge = [&intersections, level](XMFLOAT3 a, XMFLOAT3 b)
        {
            const bool aAbove = a.y >= level;
            const bool bAbove = b.y >= level;
            if (aAbove == bAbove)
                return;
            intersections.push_back(InterpolateContourPoint(a, b, level));
        };

        tryEdge(p0, p1);
        tryEdge(p1, p2);
        tryEdge(p2, p3);
        tryEdge(p3, p0);

        if (intersections.size() == 2)
        {
            m_contourSegments.push_back({ intersections[0], intersections[1] });
        }
        else if (intersections.size() == 4)
        {
            m_contourSegments.push_back({ intersections[0], intersections[1] });
            m_contourSegments.push_back({ intersections[2], intersections[3] });
        }
    };

    for (int row = 0; row < rows - 1; ++row)
    {
        for (int col = 0; col < cols - 1; ++col)
        {
            const XMFLOAT3 p0 = sampleVertex(col, row);
            const XMFLOAT3 p1 = sampleVertex(col + 1, row);
            const XMFLOAT3 p2 = sampleVertex(col + 1, row + 1);
            const XMFLOAT3 p3 = sampleVertex(col, row + 1);

            const float cellMin = (std::min)((std::min)(p0.y, p1.y), (std::min)(p2.y, p3.y));
            const float cellMax = (std::max)((std::max)(p0.y, p1.y), (std::max)(p2.y, p3.y));
            if (cellMax - cellMin <= 1e-5f)
                continue;

            const int firstLevel = static_cast<int>(std::ceil(cellMin / interval));
            const int lastLevel = static_cast<int>(std::floor(cellMax / interval));
            for (int levelIndex = firstLevel; levelIndex <= lastLevel; ++levelIndex)
            {
                const float level = static_cast<float>(levelIndex) * interval;
                appendSegmentsForCell(p0, p1, p2, p3, level);
            }
        }
    }
}

void App::NewProject()
{
    strncpy_s(m_terrainPath, sizeof(m_terrainPath), "data/heightmap.png", _TRUNCATE);
    m_projectPath[0] = '\0';

    m_loadResW = 1024;
    m_loadResH = 1024;
    m_loadWidthM = 1024.0f;
    m_loadDepthM = 1024.0f;
    m_loadHeightM = 1024.0f;
    m_loadOffsetX = 0.0f;
    m_loadOffsetY = 0.0f;
    m_loadOffsetZ = 0.0f;

    m_cursorHitValid = false;
    m_cursorHitPos = {};
    ResetPathfindingState();
    m_contourSegments.clear();

    m_terrain->Reset();
    m_terrainTexturePath[0] = '\0';
    m_roadNetwork = RoadNetwork();
    m_editor.SetNetwork(&m_roadNetwork);
    m_editor.SetFilePath("data/roads.json");
    m_editor.ResetState();
    m_camera->SetOrbitState({ 0.0f, 0.0f, 0.0f }, 20.0f, 0.785f, 0.4f);
    m_sunAzimuth = 0.98f;
    m_sunElevation = 0.78f;
    m_terrain->sunDirection = ComputeSunDirection();
    m_terrain->lightingMode = Terrain::LightingModeBasic;
    UpdateWindowTitle();
    SetStatusMessage("New project");
}

bool App::SaveProjectAs()
{
    char projectPath[260] = {};
    strncpy_s(projectPath, sizeof(projectPath), m_projectPath, _TRUNCATE);
    if (!SaveFileDialog(m_hwnd, projectPath, sizeof(projectPath),
                        "Project Files\0*.json\0All Files\0*.*\0",
                        "Save Project"))
    {
        return true;
    }

    strncpy_s(m_projectPath, sizeof(m_projectPath), projectPath, _TRUNCATE);
    const bool ok = SaveProject(m_projectPath);
    if (ok)
    {
        AddRecentProjectPath(m_projectPath);
        SaveViewSettings();
        UpdateWindowTitle();
    }
    return ok;
}

bool App::SaveRoads(const char* path)
{
    if (path == nullptr || path[0] == '\0')
        return SaveRoadsAs();

    m_editor.SetFilePath(path);
    if (!m_editor.Save(path))
    {
        SetStatusMessage(std::string("Road save failed: ") + path);
        return false;
    }

    SetStatusMessage(std::string("Roads saved: ") + path);
    return true;
}

bool App::SaveRoadsAs()
{
    char roadPath[260] = {};
    strncpy_s(roadPath, sizeof(roadPath), m_editor.GetFilePath(), _TRUNCATE);
    if (!SaveFileDialog(m_hwnd, roadPath, sizeof(roadPath),
                        "Road Files\0*.json\0All Files\0*.*\0",
                        "Save Roads"))
    {
        return false;
    }

    return SaveRoads(roadPath);
}

bool App::LoadRoadsFromPath(const char* path)
{
    if (path == nullptr || path[0] == '\0')
        return false;

    m_editor.SetFilePath(path);
    if (!m_editor.Load(path))
    {
        SetStatusMessage(std::string("Road load failed: ") + path);
        return false;
    }

    SetStatusMessage(std::string("Roads loaded: ") + path);
    return true;
}

bool App::OpenRoads()
{
    char roadPath[260] = {};
    strncpy_s(roadPath, sizeof(roadPath), m_editor.GetFilePath(), _TRUNCATE);
    if (!OpenFileDialog(m_hwnd, roadPath, sizeof(roadPath),
                        "Road Files\0*.json\0All Files\0*.*\0",
                        "Open Roads"))
    {
        return false;
    }

    return LoadRoadsFromPath(roadPath);
}

bool App::SaveProject(const char* path)
{
    try
    {
        if (path == nullptr || path[0] == '\0')
            return SaveProjectAs();

        if (m_editor.GetFilePath()[0] != '\0' && !SaveRoads(m_editor.GetFilePath()))
        {
            return false;
        }

        nlohmann::json root;
        const std::string relativeTerrainPath = MakePathRelativeToProject(path, m_terrainPath);
        const std::string relativeTerrainTexturePath =
            MakePathRelativeToProject(path, m_terrainTexturePath);
        const std::string relativeRoadPath =
            MakePathRelativeToProject(path, m_editor.GetFilePath());
        root["version"] = 1;
        root["terrain"] =
        {
            { "path",        relativeTerrainPath      },
            { "texturePath", relativeTerrainTexturePath },
            { "divisionsX",  m_loadResW               },
            { "divisionsY",  m_loadResH               },
            { "widthM",      m_loadWidthM             },
            { "depthM",      m_loadDepthM             },
            { "heightM",     m_loadHeightM            },
            { "offsetX",     m_loadOffsetX            },
            { "offsetY",     m_loadOffsetY            },
            { "offsetZ",     m_loadOffsetZ            },
            { "colorMode",   m_terrain->colorMode     },
            { "lightingMode", m_terrain->lightingMode },
            { "sunAzimuth",  m_sunAzimuth             },
            { "sunElevation", m_sunElevation          },
            { "shadowStrength", m_terrain->shadowStrength },
            { "shadowBias",  m_terrain->shadowBias    },
            { "visible",     m_terrain->visible       },
            { "wireframe",   m_terrain->wireframe     }
        };
        root["roads"] =
        {
            { "path", relativeRoadPath }
        };
        root["camera"] =
        {
            { "targetX",   m_camera->GetTarget().x    },
            { "targetY",   m_camera->GetTarget().y    },
            { "targetZ",   m_camera->GetTarget().z    },
            { "distance",  m_camera->GetDistance()    },
            { "azimuth",   m_camera->GetAzimuth()     },
            { "elevation", m_camera->GetElevation()   }
        };

        std::ofstream ofs(path);
        if (!ofs)
            return false;

        ofs << root.dump(2);
        AddRecentProjectPath(path);
        SaveViewSettings();
        UpdateWindowTitle();
        SetStatusMessage(std::string("Project saved: ") + path);
        return true;
    }
    catch (...)
    {
        SetStatusMessage(std::string("Project save failed: ") + path);
        return false;
    }
}

bool App::LoadProject(const char* path)
{
    try
    {
        ResetPathfindingState();
        std::ifstream ifs(path);
        if (!ifs)
            return false;

        nlohmann::json root;
        ifs >> root;

        if (root.contains("terrain"))
        {
            const auto& t = root["terrain"];
            std::string terrainPath = ResolvePathFromProject(path, t.value("path", std::string()));
            std::string terrainTexturePath = ResolvePathFromProject(path, t.value("texturePath", std::string()));
            strncpy_s(m_terrainPath, sizeof(m_terrainPath), terrainPath.c_str(), _TRUNCATE);
            strncpy_s(m_terrainTexturePath, sizeof(m_terrainTexturePath), terrainTexturePath.c_str(), _TRUNCATE);
            m_loadResW    = t.value("divisionsX", 1024);
            m_loadResH    = t.value("divisionsY", 1024);
            m_loadWidthM  = t.value("widthM", 1024.0f);
            m_loadDepthM  = t.value("depthM", 1024.0f);
            m_loadHeightM = t.value("heightM", 1024.0f);
            m_loadOffsetX = t.value("offsetX", 0.0f);
            m_loadOffsetY = t.value("offsetY", 0.0f);
            m_loadOffsetZ = t.value("offsetZ", 0.0f);
            m_terrain->colorMode = t.value("colorMode", 1);
            m_terrain->lightingMode = t.value("lightingMode", Terrain::LightingModeBasic);
            m_sunAzimuth = t.value("sunAzimuth", 0.98f);
            m_sunElevation = t.value("sunElevation", 0.78f);
            m_terrain->shadowStrength = t.value("shadowStrength", 0.72f);
            m_terrain->shadowBias = t.value("shadowBias", 0.0015f);
            m_terrain->sunDirection = ComputeSunDirection();
            m_terrain->visible   = t.value("visible", true);
            m_terrain->wireframe = t.value("wireframe", false);
            if (m_terrainTexturePath[0] != '\0')
                m_terrain->LoadColorTexture(m_d3d->GetDevice(), m_terrainTexturePath);
            else
                m_terrain->ClearColorTexture();

            if (m_terrainPath[0] != '\0')
            {
                if (!m_terrain->LoadFromFile(m_d3d->GetDevice(), m_terrainPath))
                {
                    SetStatusMessage(std::string("Terrain load failed: ") + m_terrainPath);
                    return false;
                }
                ApplyTerrainSettings();
            }
            else
            {
                m_contourSegments.clear();
            }
        }

        if (root.contains("roads"))
        {
            const auto& r = root["roads"];
            std::string roadPath = ResolvePathFromProject(path, r.value("path", std::string()));
            m_editor.SetFilePath(roadPath.c_str());
            if (roadPath.empty())
            {
                m_roadNetwork.roads.clear();
            }
            else if (!LoadRoadsFromPath(roadPath.c_str()))
            {
                return false;
            }
        }

        strncpy_s(m_projectPath, sizeof(m_projectPath), path, _TRUNCATE);
        AddRecentProjectPath(path);
        SaveViewSettings();
        UpdateWindowTitle();

        if (root.contains("camera"))
        {
            const auto& c = root["camera"];
            m_camera->SetOrbitState(
                {
                    c.value("targetX", 0.0f),
                    c.value("targetY", 0.0f),
                    c.value("targetZ", 0.0f)
                },
                c.value("distance", 1000.0f),
                c.value("azimuth", 0.785f),
                c.value("elevation", 0.4f));
        }

        SetStatusMessage(std::string("Project loaded: ") + path);
        return true;
    }
    catch (...)
    {
        SetStatusMessage(std::string("Project load failed: ") + path);
        return false;
    }
}

void App::UpdateSunInput(bool wantMouse)
{
    const bool lightHotkeyDown = (GetAsyncKeyState('L') & 0x8000) != 0;
    const bool lmbDown = (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;
    const bool altDown = (GetAsyncKeyState(VK_MENU) & 0x8000) != 0;
    const bool sunDragRequested =
        lightHotkeyDown &&
        lmbDown &&
        !altDown &&
        !wantMouse &&
        m_terrain &&
        m_terrain->lightingMode == Terrain::LightingModeSunShadowed;

    if (!sunDragRequested)
    {
        m_sunDragActive = false;
        return;
    }

    POINT cursor = {};
    GetCursorPos(&cursor);
    if (!m_sunDragActive)
    {
        m_sunDragActive = true;
        m_lastSunMouse = cursor;
        SetStatusMessage("Sun direction editing");
        return;
    }

    const float dx = static_cast<float>(cursor.x - m_lastSunMouse.x);
    const float dy = static_cast<float>(cursor.y - m_lastSunMouse.y);
    m_lastSunMouse = cursor;

    const float sensitivity = 0.008f;
    m_sunAzimuth -= dx * sensitivity;
    m_sunElevation = std::clamp(m_sunElevation + dy * sensitivity, 0.05f, 1.52f);
    m_terrain->sunDirection = ComputeSunDirection();
}

XMFLOAT3 App::ComputeSunDirection() const
{
    const XMFLOAT3 dir = DirectionFromAzimuthElevation(m_sunAzimuth, m_sunElevation);
    XMVECTOR sun = XMVector3Normalize(XMLoadFloat3(&dir));
    XMFLOAT3 normalized = {};
    XMStoreFloat3(&normalized, sun);
    return normalized;
}

XMFLOAT4X4 App::ComputeLightViewProjMatrix() const
{
    XMFLOAT4X4 result = {};
    XMStoreFloat4x4(&result, XMMatrixIdentity());

    if (!m_terrain || !m_terrain->IsReady())
        return result;

    const XMFLOAT3 sunDir = ComputeSunDirection();
    const float halfWidth =
        static_cast<float>(m_terrain->GetMeshW() - 1) * m_terrain->horizontalScaleX * 0.5f;
    const float halfDepth =
        static_cast<float>(m_terrain->GetMeshH() - 1) * m_terrain->horizontalScaleZ * 0.5f;
    const XMFLOAT3 boundsMin =
    {
        m_terrain->offsetX - halfWidth,
        0.0f,
        m_terrain->offsetZ - halfDepth
    };
    const XMFLOAT3 boundsMax =
    {
        m_terrain->offsetX + halfWidth,
        m_terrain->heightScale,
        m_terrain->offsetZ + halfDepth
    };
    const XMFLOAT3 center =
    {
        0.5f * (boundsMin.x + boundsMax.x),
        0.5f * (boundsMin.y + boundsMax.y),
        0.5f * (boundsMin.z + boundsMax.z)
    };

    const float radius = 0.5f * sqrtf(
        (boundsMax.x - boundsMin.x) * (boundsMax.x - boundsMin.x) +
        (boundsMax.y - boundsMin.y) * (boundsMax.y - boundsMin.y) +
        (boundsMax.z - boundsMin.z) * (boundsMax.z - boundsMin.z));
    const XMFLOAT3 lightPos =
    {
        center.x + sunDir.x * (radius + 200.0f),
        center.y + sunDir.y * (radius + 200.0f),
        center.z + sunDir.z * (radius + 200.0f)
    };

    XMVECTOR up = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
    if (fabsf(sunDir.y) > 0.97f)
        up = XMVectorSet(0.0f, 0.0f, 1.0f, 0.0f);

    const XMMATRIX lightView = XMMatrixLookAtLH(
        XMLoadFloat3(&lightPos),
        XMLoadFloat3(&center),
        up);

    std::array<XMFLOAT3, 8> corners =
    {{
        { boundsMin.x, boundsMin.y, boundsMin.z },
        { boundsMin.x, boundsMin.y, boundsMax.z },
        { boundsMin.x, boundsMax.y, boundsMin.z },
        { boundsMin.x, boundsMax.y, boundsMax.z },
        { boundsMax.x, boundsMin.y, boundsMin.z },
        { boundsMax.x, boundsMin.y, boundsMax.z },
        { boundsMax.x, boundsMax.y, boundsMin.z },
        { boundsMax.x, boundsMax.y, boundsMax.z }
    }};

    const float kFloatMax = (std::numeric_limits<float>::max)();
    XMFLOAT3 lightMin = { kFloatMax, kFloatMax, kFloatMax };
    XMFLOAT3 lightMax = { -kFloatMax, -kFloatMax, -kFloatMax };
    for (const XMFLOAT3& corner : corners)
    {
        XMFLOAT3 ls = {};
        XMStoreFloat3(&ls, XMVector3TransformCoord(XMLoadFloat3(&corner), lightView));
        lightMin.x = (std::min)(lightMin.x, ls.x);
        lightMin.y = (std::min)(lightMin.y, ls.y);
        lightMin.z = (std::min)(lightMin.z, ls.z);
        lightMax.x = (std::max)(lightMax.x, ls.x);
        lightMax.y = (std::max)(lightMax.y, ls.y);
        lightMax.z = (std::max)(lightMax.z, ls.z);
    }

    const float padding = 20.0f;
    const XMMATRIX lightProj = XMMatrixOrthographicOffCenterLH(
        lightMin.x - padding,
        lightMax.x + padding,
        lightMin.y - padding,
        lightMax.y + padding,
        (std::max)(0.1f, lightMin.z - padding),
        lightMax.z + padding);
    XMStoreFloat4x4(&result, XMMatrixMultiply(lightView, lightProj));
    return result;
}

// --- Window ------------------------------------------------------------------

bool App::CreateAppWindow(HINSTANCE hInstance, int nCmdShow)
{
    HICON largeIcon = static_cast<HICON>(LoadImageW(
        hInstance,
        MAKEINTRESOURCEW(IDI_APP_ICON),
        IMAGE_ICON,
        GetSystemMetrics(SM_CXICON),
        GetSystemMetrics(SM_CYICON),
        0));
    HICON smallIcon = static_cast<HICON>(LoadImageW(
        hInstance,
        MAKEINTRESOURCEW(IDI_APP_ICON),
        IMAGE_ICON,
        GetSystemMetrics(SM_CXSMICON),
        GetSystemMetrics(SM_CYSMICON),
        0));

    WNDCLASSEXW wc    = {};
    wc.cbSize          = sizeof(wc);
    wc.style           = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc     = StaticWndProc;
    wc.hInstance       = hInstance;
    wc.hIcon           = largeIcon;
    wc.hIconSm         = smallIcon;
    wc.hCursor         = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground   = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    wc.lpszClassName   = L"RoadEditorClass";

    if (!RegisterClassExW(&wc))
        return false;

    m_hwnd = CreateWindowExW(
        0, L"RoadEditorClass",
        L"Road Editor",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT, 1600, 900,
        nullptr, nullptr, hInstance,
        this);  // passed to WM_NCCREATE as CREATESTRUCT::lpCreateParams

    if (!m_hwnd)
        return false;

    ShowWindow(m_hwnd, nCmdShow);
    UpdateWindow(m_hwnd);
    UpdateWindowTitle();
    return true;
}

// --- Initialize --------------------------------------------------------------

bool App::Initialize(HINSTANCE hInstance, int nCmdShow)
{
    if (!CreateAppWindow(hInstance, nCmdShow))
        return false;

    RECT rc = {};
    GetClientRect(m_hwnd, &rc);
    uint32_t w = static_cast<uint32_t>(rc.right  - rc.left);
    uint32_t h = static_cast<uint32_t>(rc.bottom - rc.top);

    m_d3d = std::make_unique<D3D11Context>();
    if (!m_d3d->Initialize(m_hwnd, w, h))
        return false;

    m_imgui = std::make_unique<ImGuiLayer>();
    if (!m_imgui->Initialize(m_hwnd, m_d3d->GetDevice(), m_d3d->GetContext()))
        return false;

    m_camera = std::make_unique<Camera>();
    m_camera->Initialize(1000.0f, 0.785f, 0.4f);

    m_grid = std::make_unique<Grid>();
    if (!m_grid->Initialize(m_d3d->GetDevice()))
        return false;

    m_terrain = std::make_unique<Terrain>();
    if (!m_terrain->Initialize(m_d3d->GetDevice()))
        return false;

    if (!m_perFrameCB.Initialize(m_d3d->GetDevice()))
        return false;

    if (!m_debugDraw.Initialize(m_d3d->GetDevice()))
        return false;

    m_editor.SetTerrain(m_terrain.get());
    m_editor.SetNetwork(&m_roadNetwork);
    LoadViewSettings();

    return true;
}

void App::UpdateWindowTitle() const
{
    if (!m_hwnd)
        return;

    std::wstring title = L"Road Editor";
    if (m_projectPath[0] != '\0')
    {
        const std::filesystem::path projectPath(m_projectPath);
        const std::string projectName = projectPath.stem().string();
        if (!projectName.empty())
        {
            title += L" - ";
            title += std::wstring(projectName.begin(), projectName.end());
        }
    }

    SetWindowTextW(m_hwnd, title.c_str());
}

// --- Run (main loop) ---------------------------------------------------------

int App::Run()
{
    auto lastTime = std::chrono::high_resolution_clock::now();

    MSG msg = {};
    while (m_running)
    {
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
            if (msg.message == WM_QUIT)
                m_running = false;
        }

        if (!m_running)
            break;

        auto   now = std::chrono::high_resolution_clock::now();
        float  dt  = std::chrono::duration<float>(now - lastTime).count();
        lastTime    = now;
        m_time     += dt;

        bool wantMouse = ImGui::GetIO().WantCaptureMouse;
        m_camera->HandleInput(wantMouse);
        UpdateSunInput(wantMouse);

        const bool focusKeyDown = (GetAsyncKeyState('F') & 0x8000) != 0;
        const bool focusPressed = focusKeyDown && !m_prevFocusKey;
        m_prevFocusKey = focusKeyDown;

        // Editor update (must happen after camera input, before render)
        {
            RECT rc = {};
            GetClientRect(m_hwnd, &rc);
            int vpW = rc.right  - rc.left;
            int vpH = rc.bottom - rc.top;

            if (IsIconic(m_hwnd) || vpW <= 0 || vpH <= 0)
            {
                m_cursorHitValid = false;
                Render();
                continue;
            }

            POINT cursor = {};
            GetCursorPos(&cursor);
            ScreenToClient(m_hwnd, &cursor);

            XMMATRIX vp = XMMatrixMultiply(m_camera->GetViewMatrix(),
                                            m_camera->GetProjMatrix(
                                                static_cast<float>(vpW) /
                                                static_cast<float>(vpH)));
            XMMATRIX invVP = XMMatrixInverse(nullptr, vp);
            const bool blockSceneMouse = wantMouse || m_sunDragActive;
            m_editor.Update(vpW, vpH,
                            { static_cast<float>(cursor.x),
                              static_cast<float>(cursor.y) },
                            invVP, blockSceneMouse);

            if (focusPressed && !ImGui::GetIO().WantTextInput)
            {
                XMFLOAT3 focusTarget;
                if (m_editor.GetFocusTarget(focusTarget))
                {
                    m_camera->SetOrbitState(
                        focusTarget,
                        m_camera->GetDistance(),
                        m_camera->GetAzimuth(),
                        m_camera->GetElevation());
                    SetStatusMessage("Camera focused");
                }
            }

            // Terrain cursor raycast or XZ ground plane fallback
            m_cursorHitValid = false;
            if (vpW > 0 && vpH > 0)
            {
                float ndcX =  (static_cast<float>(cursor.x) / vpW) * 2.0f - 1.0f;
                float ndcY = -(static_cast<float>(cursor.y) / vpH) * 2.0f + 1.0f;

                XMVECTOR nearH = XMVector4Transform(
                    XMVectorSet(ndcX, ndcY, 0.0f, 1.0f), invVP);
                XMVECTOR farH  = XMVector4Transform(
                    XMVectorSet(ndcX, ndcY, 1.0f, 1.0f), invVP);

                nearH = nearH / XMVectorSplatW(nearH);
                farH  = farH  / XMVectorSplatW(farH);

                XMFLOAT3 rayO, rayD3;
                XMStoreFloat3(&rayO, nearH);
                XMStoreFloat3(&rayD3, XMVector3Normalize(farH - nearH));

            if (m_terrain->IsReady())
                    m_cursorHitValid = m_terrain->Raycast(rayO, rayD3, m_cursorHitPos);
                else
                    m_cursorHitValid = IntersectRayWithGroundPlane(rayO, rayD3, m_cursorHitPos);
            }

            const EditorMode currentMode = m_editor.GetMode();
            if (currentMode != m_prevEditorMode)
            {
                if (currentMode == EditorMode::Pathfinding)
                    SyncPathfindingEndpointsFromSelectedRoad();
                m_prevEditorMode = currentMode;
            }

            UpdatePathfindingInput(blockSceneMouse);
        }

        Render();
    }

    m_debugDraw.Shutdown();
    m_terrain->Shutdown();
    m_grid->Shutdown();
    m_imgui->Shutdown();
    m_d3d->Shutdown();

    return static_cast<int>(msg.wParam);
}

// --- Render ------------------------------------------------------------------

void App::Render()
{
    if (IsIconic(m_hwnd) || m_d3d->GetWidth() <= 0 || m_d3d->GetHeight() <= 0)
        return;

    const XMFLOAT4X4 lightViewProj = ComputeLightViewProjMatrix();
    if (m_terrain->lightingMode == Terrain::LightingModeSunShadowed)
        m_terrain->RenderShadowMap(m_d3d->GetContext(), lightViewProj);

    m_d3d->BeginFrame(m_backgroundColor.x, m_backgroundColor.y, m_backgroundColor.z);

    // Build per-frame constant buffer
    float    aspect = static_cast<float>(m_d3d->GetWidth())
                    / static_cast<float>(m_d3d->GetHeight());
    XMMATRIX view   = m_camera->GetViewMatrix();
    XMMATRIX proj   = m_camera->GetProjMatrix(aspect);
    XMMATRIX vp     = XMMatrixMultiply(view, proj);

    PerFrameData pfd;
    XMStoreFloat4x4(&pfd.viewProj,    vp);
    XMStoreFloat4x4(&pfd.invViewProj, XMMatrixInverse(nullptr, vp));
    pfd.cameraPos = m_camera->GetPosition();
    pfd.time      = m_time;
    pfd.gridBaseScale = m_gridBaseScale;
    pfd.gridFadeDistance = m_gridFadeDistance;
    m_perFrameCB.Update(m_d3d->GetContext(), pfd);

    // 3D scene  (order: terrain -> grid -> debug lines)
    m_terrain->Render(m_d3d->GetContext(), m_perFrameCB.Get(), lightViewProj);
    if (m_showGrid)
        m_grid->Render(m_d3d->GetContext(), m_perFrameCB.Get());

    m_editor.DrawNetwork(m_debugDraw, vp, m_d3d->GetWidth(), m_d3d->GetHeight());
    DrawContourPreview();
    DrawPathfindingPreview();
    m_debugDraw.Flush(m_d3d->GetContext(), m_perFrameCB.Get());

    // ImGui
    m_imgui->BeginFrame();
    DrawViewAxisGizmo(view);

    if (ImGui::GetIO().KeyCtrl &&
        ImGui::IsKeyPressed(ImGuiKey_S, false) &&
        !ImGui::GetIO().WantTextInput)
    {
        if (!SaveProject(m_projectPath))
            ImGui::OpenPopup("ProjectSaveError");
    }

    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu(u8"\u30D5\u30A1\u30A4\u30EB"))
        {
            if (ImGui::MenuItem(u8"\u65B0\u898F\u30D7\u30ED\u30B8\u30A7\u30AF\u30C8"))
                NewProject();
            if (ImGui::MenuItem(u8"\u30D7\u30ED\u30B8\u30A7\u30AF\u30C8\u3092\u958B\u304F..."))
            {
                if (OpenFileDialog(m_hwnd, m_projectPath, sizeof(m_projectPath),
                                   "Project Files\0*.json\0All Files\0*.*\0",
                                   "Open Project"))
                {
                    if (!LoadProject(m_projectPath))
                        ImGui::OpenPopup("ProjectLoadError");
                }
            }
            if (!m_recentProjectPaths.empty())
            {
                ImGui::Separator();
                if (ImGui::BeginMenu(u8"\u6700\u8FD1\u958B\u3044\u305F\u30D7\u30ED\u30B8\u30A7\u30AF\u30C8"))
                {
                    for (size_t i = 0; i < m_recentProjectPaths.size() && i < 8; ++i)
                    {
                        const std::string& recentPath = m_recentProjectPaths[i];
                        const std::filesystem::path labelPath(recentPath);
                        const std::string label =
                            std::to_string(i + 1) + ". " + labelPath.stem().string();
                        if (ImGui::MenuItem(label.c_str()))
                        {
                            strncpy_s(m_projectPath, sizeof(m_projectPath), recentPath.c_str(), _TRUNCATE);
                            if (!LoadProject(m_projectPath))
                                ImGui::OpenPopup("ProjectLoadError");
                        }
                    }
                    ImGui::EndMenu();
                }
            }
            if (ImGui::MenuItem(u8"\u30D7\u30ED\u30B8\u30A7\u30AF\u30C8\u3092\u4FDD\u5B58"))
            {
                if (!SaveProject(m_projectPath))
                    ImGui::OpenPopup("ProjectSaveError");
            }
            if (ImGui::MenuItem(u8"\u540D\u524D\u3092\u4ED8\u3051\u3066\u4FDD\u5B58..."))
            {
                if (!SaveProjectAs())
                    ImGui::OpenPopup("ProjectSaveError");
            }
            ImGui::Separator();
            if (ImGui::MenuItem(u8"\u9053\u8DEF\u3092\u958B\u304F..."))
            {
                if (!OpenRoads())
                    ImGui::OpenPopup("RoadLoadError");
            }
            if (ImGui::MenuItem(u8"\u9053\u8DEF\u3092\u4FDD\u5B58"))
            {
                if (!SaveRoads(m_editor.GetFilePath()))
                    ImGui::OpenPopup("RoadSaveError");
            }
            if (ImGui::MenuItem(u8"\u9053\u8DEF\u3092\u5225\u540D\u3067\u4FDD\u5B58..."))
            {
                if (!SaveRoadsAs())
                    ImGui::OpenPopup("RoadSaveError");
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu(u8"\u7DE8\u96C6"))
        {
            if (ImGui::MenuItem("Undo", "Ctrl+Z"))
                m_editor.PerformUndo();
            if (ImGui::MenuItem("Redo", "Ctrl+Y"))
                m_editor.PerformRedo();
            ImGui::Separator();
            if (ImGui::MenuItem(u8"\u4EA4\u5DEE\u70B9\u3092\u81EA\u52D5\u4F5C\u6210"))
                m_editor.AutoCreateIntersections();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu(u8"\u8868\u793A"))
        {
            if (ImGui::MenuItem(u8"\u9053\u8DEF\u540D", nullptr, m_showRoadNames))
            {
                m_showRoadNames = !m_showRoadNames;
                m_editor.SetShowRoadNames(m_showRoadNames);
                SaveViewSettings();
            }
            if (ImGui::MenuItem(u8"\u4EA4\u5DEE\u70B9\u540D", nullptr, m_showIntersectionNames))
            {
                m_showIntersectionNames = !m_showIntersectionNames;
                m_editor.SetShowIntersectionNames(m_showIntersectionNames);
                SaveViewSettings();
            }
            if (ImGui::MenuItem(u8"\u9053\u8DEF\u30D7\u30EC\u30D3\u30E5\u30FC\u60C5\u5831", nullptr, m_showRoadPreviewMetrics))
            {
                m_showRoadPreviewMetrics = !m_showRoadPreviewMetrics;
                m_editor.SetShowRoadPreviewMetrics(m_showRoadPreviewMetrics);
                SaveViewSettings();
            }
            if (ImGui::MenuItem(u8"\u9053\u8DEF\u52FE\u914D\u30B0\u30E9\u30C7\u30FC\u30B7\u30E7\u30F3", nullptr, m_showRoadGradeGradient))
            {
                m_showRoadGradeGradient = !m_showRoadGradeGradient;
                m_editor.SetShowRoadGradeGradient(m_showRoadGradeGradient);
                SaveViewSettings();
            }
            if (ImGui::MenuItem(u8"\u30B0\u30EA\u30C3\u30C9", nullptr, m_showGrid))
            {
                m_showGrid = !m_showGrid;
                SaveViewSettings();
            }
            if (ImGui::MenuItem("FPS", nullptr, m_showFps))
            {
                m_showFps = !m_showFps;
                SaveViewSettings();
            }
            if (m_showRoadGradeGradient)
            {
                float threshold = m_roadGradeRedThresholdPercent;
                if (ImGui::InputFloat(u8"\u8D64\u95BE\u5024 (%)", &threshold, 0.5f, 1.0f, "%.1f"))
                {
                    m_roadGradeRedThresholdPercent = (std::max)(0.1f, threshold);
                    m_editor.SetRoadGradeRedThresholdPercent(m_roadGradeRedThresholdPercent);
                    SaveViewSettings();
                }
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu(u8"\u30A6\u30A4\u30F3\u30C9\u30A6"))
        {
            if (ImGui::MenuItem(u8"\u5730\u5F62", nullptr, m_showTerrainWindow))
            {
                m_showTerrainWindow = !m_showTerrainWindow;
                SaveViewSettings();
            }
            if (ImGui::MenuItem(u8"\u30AB\u30E1\u30E9", nullptr, m_showCameraWindow))
            {
                m_showCameraWindow = !m_showCameraWindow;
                SaveViewSettings();
            }
            if (ImGui::MenuItem(u8"\u9053\u8DEF\u30A8\u30C7\u30A3\u30BF", nullptr, m_showRoadEditorWindow))
            {
                m_showRoadEditorWindow = !m_showRoadEditorWindow;
                SaveViewSettings();
            }
            if (ImGui::MenuItem(u8"\u30D7\u30ED\u30D1\u30C6\u30A3", nullptr, m_showPropertiesWindow))
            {
                m_showPropertiesWindow = !m_showPropertiesWindow;
                SaveViewSettings();
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu(u8"\u8A2D\u5B9A"))
        {
            if (ImGui::MenuItem(u8"\u80CC\u666F"))
                m_showBackgroundSettings = true;
            if (ImGui::MenuItem(u8"\u30A8\u30C7\u30A3\u30BF\u8868\u793A"))
                m_showEditorDisplaySettings = true;
            ImGui::EndMenu();
        }

        if (m_showFps)
        {
            char fpsLabel[32] = {};
            sprintf_s(fpsLabel, "FPS: %.0f", ImGui::GetIO().Framerate);
            const float fpsWidth = ImGui::CalcTextSize(fpsLabel).x;
            const float targetX = ImGui::GetWindowWidth() - fpsWidth - ImGui::GetStyle().ItemSpacing.x * 2.0f;
            const float cursorX = ImGui::GetCursorPosX();
            if (targetX > cursorX)
                ImGui::SetCursorPosX(targetX);
            else
                ImGui::SameLine();
            ImGui::TextUnformatted(fpsLabel);
        }

        ImGui::EndMainMenuBar();
    }

    if (ImGui::BeginPopup("ProjectSaveError"))
    {
        ImGui::Text(u8"\u30D7\u30ED\u30B8\u30A7\u30AF\u30C8\u306E\u4FDD\u5B58\u306B\u5931\u6557\u3057\u307E\u3057\u305F: %s", m_projectPath);
        if (ImGui::Button("OK")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    if (ImGui::BeginPopup("ProjectLoadError"))
    {
        ImGui::Text(u8"\u30D7\u30ED\u30B8\u30A7\u30AF\u30C8\u306E\u8AAD\u307F\u8FBC\u307F\u306B\u5931\u6557\u3057\u307E\u3057\u305F: %s", m_projectPath);
        if (ImGui::Button("OK")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    if (ImGui::BeginPopup("RoadSaveError"))
    {
        ImGui::Text(u8"\u9053\u8DEF\u306E\u4FDD\u5B58\u306B\u5931\u6557\u3057\u307E\u3057\u305F: %s", m_editor.GetFilePath());
        if (ImGui::Button("OK")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    if (ImGui::BeginPopup("RoadLoadError"))
    {
        ImGui::Text(u8"\u9053\u8DEF\u306E\u8AAD\u307F\u8FBC\u307F\u306B\u5931\u6557\u3057\u307E\u3057\u305F: %s", m_editor.GetFilePath());
        if (ImGui::Button("OK")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    if (m_showBackgroundSettings)
    {
        ImGui::SetNextWindowSize(ImVec2(340, 220), ImGuiCond_FirstUseEver);
        if (ImGui::Begin(u8"\u80CC\u666F\u8A2D\u5B9A", &m_showBackgroundSettings))
        {
            if (ImGui::CollapsingHeader(u8"\u30B0\u30EA\u30C3\u30C9", ImGuiTreeNodeFlags_DefaultOpen))
            {
                float gridBaseScale = m_gridBaseScale;
                if (ImGui::InputFloat(
                        u8"\u30B0\u30EA\u30C3\u30C9\u30B5\u30A4\u30BA (m)",
                        &gridBaseScale,
                        0.1f,
                        1.0f,
                        "%.2f"))
                {
                    m_gridBaseScale = (std::max)(0.01f, gridBaseScale);
                    SaveViewSettings();
                }

                float gridFadeDistance = m_gridFadeDistance;
                if (ImGui::InputFloat(
                        u8"\u30D5\u30A9\u30B0\u8DDD\u96E2 (m)",
                        &gridFadeDistance,
                        10.0f,
                        100.0f,
                        "%.0f"))
                {
                    m_gridFadeDistance = (std::max)(1.0f, gridFadeDistance);
                    SaveViewSettings();
                }
            }

            if (ImGui::CollapsingHeader(u8"\u80CC\u666F\u8272", ImGuiTreeNodeFlags_DefaultOpen))
            {
                float backgroundColor[3] = { m_backgroundColor.x, m_backgroundColor.y, m_backgroundColor.z };
                if (ImGui::ColorEdit3(
                        u8"\u80CC\u666F\u8272",
                        backgroundColor,
                        ImGuiColorEditFlags_Float |
                        ImGuiColorEditFlags_DisplayRGB))
                {
                    m_backgroundColor = { backgroundColor[0], backgroundColor[1], backgroundColor[2] };
                    SaveViewSettings();
                }
            }
        }
        ImGui::End();
    }

    if (m_showEditorDisplaySettings)
    {
        ImGui::SetNextWindowSize(ImVec2(360, 240), ImGuiCond_FirstUseEver);
        if (ImGui::Begin(u8"\u30A8\u30C7\u30A3\u30BF\u8868\u793A", &m_showEditorDisplaySettings))
        {
            if (ImGui::CollapsingHeader(u8"\u9053\u8DEF", ImGuiTreeNodeFlags_DefaultOpen))
            {
                float roadLineThickness = m_roadLineThickness;
                if (ImGui::SliderFloat(
                        u8"\u9053\u8DEF\u306E\u592A\u3055 (px)",
                        &roadLineThickness,
                        1.0f,
                        12.0f,
                        "%.1f"))
                {
                    m_roadLineThickness = roadLineThickness;
                    m_editor.SetRoadLineThickness(m_roadLineThickness);
                    SaveViewSettings();
                }

                float previewCurveThickness = m_previewCurveThickness;
                if (ImGui::SliderFloat(
                        u8"\u30D7\u30EC\u30D3\u30E5\u30FC\u30AB\u30FC\u30D6\u306E\u592A\u3055 (px)",
                        &previewCurveThickness,
                        1.0f,
                        16.0f,
                        "%.1f"))
                {
                    m_previewCurveThickness = previewCurveThickness;
                    m_editor.SetPreviewCurveThickness(m_previewCurveThickness);
                    SaveViewSettings();
                }

                float selectedRoadLineThickness = m_selectedRoadLineThickness;
                if (ImGui::SliderFloat(
                        u8"\u9078\u629E\u6642\u306E\u9053\u8DEF\u306E\u592A\u3055 (px)",
                        &selectedRoadLineThickness,
                        1.0f,
                        16.0f,
                        "%.1f"))
                {
                    m_selectedRoadLineThickness = selectedRoadLineThickness;
                    m_editor.SetSelectedRoadLineThickness(m_selectedRoadLineThickness);
                    SaveViewSettings();
                }

                float roadVertexScreenRadius = m_roadVertexScreenRadius;
                if (ImGui::SliderFloat(
                        u8"\u9802\u70B9\u30B5\u30A4\u30BA (px)",
                        &roadVertexScreenRadius,
                        2.0f,
                        20.0f,
                        "%.0f"))
                {
                    m_roadVertexScreenRadius = roadVertexScreenRadius;
                    m_editor.SetRoadVertexScreenRadius(m_roadVertexScreenRadius);
                    SaveViewSettings();
                }

                float roadVertexColor[3] = { m_roadVertexColor.x, m_roadVertexColor.y, m_roadVertexColor.z };
                if (ImGui::ColorEdit3(
                        u8"\u8272##roadVertexColor",
                        roadVertexColor,
                        ImGuiColorEditFlags_Float |
                        ImGuiColorEditFlags_DisplayRGB))
                {
                    m_roadVertexColor = { roadVertexColor[0], roadVertexColor[1], roadVertexColor[2] };
                    m_editor.SetRoadVertexColor(m_roadVertexColor);
                    SaveViewSettings();
                }

                float selectedRoadColor[3] = { m_selectedRoadColor.x, m_selectedRoadColor.y, m_selectedRoadColor.z };
                if (ImGui::ColorEdit3(
                        u8"\u9078\u629E\u6642\u306E\u8272##selectedRoadColor",
                        selectedRoadColor,
                        ImGuiColorEditFlags_Float |
                        ImGuiColorEditFlags_DisplayRGB))
                {
                    m_selectedRoadColor = { selectedRoadColor[0], selectedRoadColor[1], selectedRoadColor[2] };
                    m_editor.SetSelectedRoadColor(m_selectedRoadColor);
                    SaveViewSettings();
                }
            }

            if (ImGui::CollapsingHeader(u8"\u4EA4\u5DEE\u70B9", ImGuiTreeNodeFlags_DefaultOpen))
            {
                float intersectionScreenGizmoRadius = m_intersectionScreenGizmoRadius;
                if (ImGui::SliderFloat(
                        u8"\u4EA4\u5DEE\u70B9\u30B5\u30FC\u30AF\u30EB\u30B5\u30A4\u30BA (px)",
                        &intersectionScreenGizmoRadius,
                        4.0f,
                        40.0f,
                        "%.0f"))
                {
                    m_intersectionScreenGizmoRadius = intersectionScreenGizmoRadius;
                    m_editor.SetIntersectionScreenGizmoRadius(m_intersectionScreenGizmoRadius);
                    SaveViewSettings();
                }

                float intersectionCircleColor[3] = { m_intersectionCircleColor.x, m_intersectionCircleColor.y, m_intersectionCircleColor.z };
                if (ImGui::ColorEdit3(
                        u8"\u8272##intersectionCircleColor",
                        intersectionCircleColor,
                        ImGuiColorEditFlags_Float |
                        ImGuiColorEditFlags_DisplayRGB))
                {
                    m_intersectionCircleColor = { intersectionCircleColor[0], intersectionCircleColor[1], intersectionCircleColor[2] };
                    m_editor.SetIntersectionCircleColor(m_intersectionCircleColor);
                    SaveViewSettings();
                }
            }
        }
        ImGui::End();
    }

    const bool wasShowingCameraWindow = m_showCameraWindow;
    if (m_showCameraWindow)
    {
        ImGui::SetNextWindowPos(ImVec2(10, 30), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(280, 110), ImGuiCond_FirstUseEver);
        if (ImGui::Begin(u8"\u30AB\u30E1\u30E9", &m_showCameraWindow))
        {
            auto p = m_camera->GetPosition();
            ImGui::Text("Position  %.2f  %.2f  %.2f", p.x, p.y, p.z);
            ImGui::Separator();
            if (m_cursorHitValid)
                ImGui::Text(u8"\u30AB\u30FC\u30BD\u30EB  %.2f  %.2f  %.2f",
                            m_cursorHitPos.x, m_cursorHitPos.y, m_cursorHitPos.z);
            else
                ImGui::TextDisabled(u8"\u30AB\u30FC\u30BD\u30EB  --");
            ImGui::Separator();
            ImGui::TextDisabled(u8"Alt + \u5DE6\u30C9\u30E9\u30C3\u30B0 : \u56DE\u8EE2");
            ImGui::TextDisabled(u8"Alt + \u4E2D\u30C9\u30E9\u30C3\u30B0 : \u30D1\u30F3");
            ImGui::TextDisabled(u8"\u30DE\u30A6\u30B9\u30DB\u30A4\u30FC\u30EB   : \u30BA\u30FC\u30E0");
            ImGui::TextDisabled(u8"L + \u5DE6\u30C9\u30E9\u30C3\u30B0   : \u592A\u967D\u65B9\u5411");
        }
        ImGui::End();
    }
    if (wasShowingCameraWindow != m_showCameraWindow)
        SaveViewSettings();

    const bool wasShowingTerrainWindow = m_showTerrainWindow;
    if (m_showTerrainWindow)
    {
        ImGui::SetNextWindowPos(ImVec2(10, 150), ImGuiCond_FirstUseEver);
        ImGui::Begin(u8"\u5730\u5F62", &m_showTerrainWindow, ImGuiWindowFlags_AlwaysAutoResize);
        {
        ImGui::Checkbox(u8"\u8868\u793A",   &m_terrain->visible);
        ImGui::SameLine();
        ImGui::Checkbox(u8"\u30EF\u30A4\u30E4\u30FC", &m_terrain->wireframe);
        static const char* kTerrainColorModes[] =
        {
            u8"\u30B0\u30EC\u30FC",
            u8"\u5730\u5F62",
            u8"\u52FE\u914D",
            u8"\u30C6\u30AF\u30B9\u30C1\u30E3"
        };
        static const char* kLightingModes[] =
        {
            u8"\u6A19\u6E96",
            u8"\u592A\u967D\u5149 + \u5F71"
        };
        int terrainColorMode = std::clamp(m_terrain->colorMode, 0, 3);
        if (ImGui::Combo(u8"\u8868\u793A\u30E2\u30FC\u30C9", &terrainColorMode, kTerrainColorModes, IM_ARRAYSIZE(kTerrainColorModes)))
            m_terrain->colorMode = terrainColorMode;
        int terrainLightingMode = std::clamp(m_terrain->lightingMode, 0, 1);
        if (ImGui::Combo(u8"\u30E9\u30A4\u30C6\u30A3\u30F3\u30B0", &terrainLightingMode, kLightingModes, IM_ARRAYSIZE(kLightingModes)))
            m_terrain->lightingMode = terrainLightingMode;

        if (m_terrain->lightingMode == Terrain::LightingModeSunShadowed)
        {
            ImGui::SliderAngle(u8"\u592A\u967D\u65B9\u4F4D", &m_sunAzimuth, -180.0f, 180.0f);
            ImGui::SliderAngle(u8"\u592A\u967D\u9AD8\u5EA6", &m_sunElevation, 3.0f, 87.0f);
            ImGui::SliderFloat(u8"\u5F71\u306E\u6FC3\u3055", &m_terrain->shadowStrength, 0.0f, 1.0f);
            ImGui::SliderFloat(u8"\u5F71\u30D0\u30A4\u30A2\u30B9", &m_terrain->shadowBias, 0.0001f, 0.01f, "%.4f", ImGuiSliderFlags_Logarithmic);
            m_terrain->sunDirection = ComputeSunDirection();
            ImGui::TextDisabled(u8"L + \u5DE6\u30C9\u30E9\u30C3\u30B0\u3067\u592A\u967D\u65B9\u5411\u3092\u56DE\u8EE2");
        }

        if (m_terrain->colorMode == 3)
        {
            ImGui::TextDisabled(u8"\u5730\u5F62\u30C6\u30AF\u30B9\u30C1\u30E3");
            ImGui::SetNextItemWidth(260.0f);
            ImGui::InputText("##terraintex", m_terrainTexturePath, sizeof(m_terrainTexturePath));
            bool texturePathCommitted = ImGui::IsItemDeactivatedAfterEdit();
            ImGui::SameLine();
            if (ImGui::Button(u8"\u53C2\u7167..."))
            {
                if (OpenFileDialog(m_hwnd, m_terrainTexturePath, sizeof(m_terrainTexturePath),
                                   "Image Files\0*.png;*.bmp;*.tga;*.jpg;*.jpeg\0All Files\0*.*\0",
                                   "Open Terrain Texture"))
                {
                    texturePathCommitted = true;
                }
            }
            if (texturePathCommitted)
            {
                if (m_terrainTexturePath[0] == '\0')
                {
                    m_terrain->ClearColorTexture();
                }
                else if (!m_terrain->LoadColorTexture(m_d3d->GetDevice(), m_terrainTexturePath))
                {
                    SetStatusMessage(std::string("Terrain texture load failed: ") + m_terrainTexturePath);
                }
                else
                {
                    SetStatusMessage(std::string("Terrain texture loaded: ") + m_terrainTexturePath);
                }
            }
        }

        ImGui::Separator();

        if (m_terrain->IsReady())
        {
            int rawW = m_terrain->GetRawW();
            int rawH = m_terrain->GetRawH();
            int mW   = m_terrain->GetMeshW();
            int mH   = m_terrain->GetMeshH();
            ImGui::Text(u8"\u753B\u50CF: %d x %d px", rawW, rawH);
            ImGui::Text(u8"\u30E1\u30C3\u30B7\u30E5: %d x %d \u9802\u70B9  %d x %d \u30BB\u30EB",
                        mW, mH, mW - 1, mH - 1);
        }
        else
        {
            ImGui::TextDisabled(u8"\u5730\u5F62\u306F\u672A\u30ED\u30FC\u30C9\u3067\u3059");
        }

        ImGui::Separator();

        ImGui::TextDisabled(u8"\u30CF\u30A4\u30C8\u30DE\u30C3\u30D7");
        ImGui::SetNextItemWidth(-60);
        ImGui::InputText("##hmap", m_terrainPath, sizeof(m_terrainPath));
        ImGui::SameLine();
        if (ImGui::Button(u8"\u53C2\u7167"))
        {
            OpenFileDialog(m_hwnd, m_terrainPath, sizeof(m_terrainPath),
                           "Image Files\0*.png;*.bmp;*.tga;*.jpg\0All Files\0*.*\0",
                           "Open Heightmap");
        }

        bool applyToCurrentTerrain = false;

        int loadDivisions[2] = { m_loadResW, m_loadResH };
        ImGui::InputInt2(u8"\u5206\u5272\u6570 X / Z", loadDivisions);
        m_loadResW = loadDivisions[0];
        m_loadResH = loadDivisions[1];
        if (ImGui::IsItemDeactivatedAfterEdit())
            applyToCurrentTerrain = true;
        ImGui::TextDisabled(u8"\u5024 = \u30EF\u30A4\u30E4\u30FC\u30BB\u30EB\u6570\u30010 \u306F\u753B\u50CF\u30B5\u30A4\u30BA - 1");
        if (m_loadResW < 0) m_loadResW = 0;
        if (m_loadResH < 0) m_loadResH = 0;
        if (m_loadResW > 4096) m_loadResW = 4096;
        if (m_loadResH > 4096) m_loadResH = 4096;

        float loadSizeM[3] = { m_loadWidthM, m_loadDepthM, m_loadHeightM };
        ImGui::InputFloat3(u8"\u30B5\u30A4\u30BA X / Z / Y (m)", loadSizeM, "%.0f");
        m_loadWidthM  = loadSizeM[0];
        m_loadDepthM  = loadSizeM[1];
        m_loadHeightM = loadSizeM[2];
        if (ImGui::IsItemDeactivatedAfterEdit())
            applyToCurrentTerrain = true;
        if (m_loadWidthM < 1.0f) m_loadWidthM = 1.0f;
        if (m_loadDepthM < 1.0f) m_loadDepthM = 1.0f;
        if (m_loadHeightM < 1.0f) m_loadHeightM = 1.0f;

        float loadOffsetM[3] = { m_loadOffsetX, m_loadOffsetY, m_loadOffsetZ };
        ImGui::InputFloat3(u8"\u30AA\u30D5\u30BB\u30C3\u30C8 X / Y / Z (m)", loadOffsetM, "%.1f");
        m_loadOffsetX = loadOffsetM[0];
        m_loadOffsetY = loadOffsetM[1];
        m_loadOffsetZ = loadOffsetM[2];
        if (ImGui::IsItemDeactivatedAfterEdit())
            applyToCurrentTerrain = true;

        if (applyToCurrentTerrain && m_terrain->IsReady())
            ApplyTerrainSettings();

        if (ImGui::Button(u8"\u8AAD\u307F\u8FBC\u307F", ImVec2(-1, 0)))
        {
            if (!m_terrain->LoadFromFile(m_d3d->GetDevice(), m_terrainPath))
            {
                SetStatusMessage(std::string("Terrain load failed: ") + m_terrainPath);
                ImGui::OpenPopup("LoadError");
            }
            else
            {
                ApplyTerrainSettings();
                SetStatusMessage(std::string("Terrain loaded: ") + m_terrainPath);
            }
        }
        if (ImGui::Button(u8"\u30CF\u30A4\u30C8\u30D5\u30A3\u30FC\u30EB\u30C9\u3092\u30AF\u30EA\u30A2", ImVec2(-1, 0)))
        {
            m_terrain->Reset();
            m_loadResW = 1024;
            m_loadResH = 1024;
            m_loadWidthM = 1024.0f;
            m_loadDepthM = 1024.0f;
            m_loadHeightM = 1024.0f;
            m_loadOffsetX = 0.0f;
            m_loadOffsetY = 0.0f;
            m_loadOffsetZ = 0.0f;
            m_contourSegments.clear();
            SetStatusMessage("Height field cleared");
        }
        if (ImGui::BeginPopup("LoadError"))
        {
            ImGui::Text(u8"\u8AAD\u307F\u8FBC\u307F\u5931\u6557: %s", m_terrainPath);
            if (ImGui::Button("OK")) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }

        ImGui::Separator();
        bool showContours = m_showContours;
        if (ImGui::Checkbox(u8"\u7B49\u9AD8\u7DDA\u3092\u8868\u793A", &showContours))
        {
            m_showContours = showContours;
            RebuildContourCache();
            SaveViewSettings();
        }

        float contourInterval = m_contourInterval;
        ImGui::InputFloat(u8"\u7B49\u9AD8\u7DDA\u9593\u9694 (m)", &contourInterval, 1.0f, 5.0f, "%.1f");
        if (ImGui::IsItemDeactivatedAfterEdit())
        {
            contourInterval = std::clamp(contourInterval, 0.5f, 1000.0f);
            if (fabsf(contourInterval - m_contourInterval) > 1e-4f)
            {
                m_contourInterval = contourInterval;
                RebuildContourCache();
                SaveViewSettings();
            }
        }

        ImGui::TextUnformatted(u8"\u7B49\u9AD8\u7DDA\u306E\u8272");
        ImGui::SameLine();
        float contourColor[3] = { m_contourColor.x, m_contourColor.y, m_contourColor.z };
        if (ImGui::ColorEdit3(
                "##contourColor",
                contourColor,
                ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel))
        {
            m_contourColor = { contourColor[0], contourColor[1], contourColor[2] };
            SaveViewSettings();
        }

        }
        ImGui::End();
    }
    if (wasShowingTerrainWindow != m_showTerrainWindow)
        SaveViewSettings();

    if (m_editor.GetMode() == EditorMode::Pathfinding)
        DrawPathfindingPanel();

    // 2D overlay: road point circles
    {
        RECT rc = {};
        GetClientRect(m_hwnd, &rc);
        int vpW = rc.right  - rc.left;
        int vpH = rc.bottom - rc.top;
        DrawPathfindingOverlay(vp, vpW, vpH);
        m_editor.DrawOverlay(vp, vpW, vpH);
    }

    // Road editor panel
    const bool wasShowingRoadEditorWindow = m_showRoadEditorWindow;
    const bool wasShowingPropertiesWindow = m_showPropertiesWindow;
    m_editor.DrawUI(m_d3d->GetDevice(), &m_showRoadEditorWindow, &m_showPropertiesWindow);
    if (wasShowingRoadEditorWindow != m_showRoadEditorWindow ||
        wasShowingPropertiesWindow != m_showPropertiesWindow)
    {
        SaveViewSettings();
    }

    std::string editorStatus;
    if (m_editor.ConsumeStatusMessage(editorStatus))
        SetStatusMessage(editorStatus);

    {
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        const ImGuiStyle& style = ImGui::GetStyle();
        const float statusBarHeight =
            ImGui::GetTextLineHeight() + style.WindowPadding.y * 2.0f;
        ImGui::SetNextWindowPos(
            ImVec2(viewport->WorkPos.x,
                   viewport->WorkPos.y + viewport->WorkSize.y - statusBarHeight));
        ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, statusBarHeight));
        ImGui::Begin("StatusBar", nullptr,
                     ImGuiWindowFlags_NoDecoration |
                     ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoNav |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);
        ImGui::TextUnformatted(m_statusMessage.c_str());
        ImGui::End();
    }

    m_imgui->EndFrame();
    m_d3d->EndFrame();
}

// --- WndProc -----------------------------------------------------------------

LRESULT CALLBACK App::StaticWndProc(HWND hwnd, UINT msg,
                                     WPARAM wParam, LPARAM lParam)
{
    // Store the App pointer on the first message
    if (msg == WM_NCCREATE)
    {
        auto* cs = reinterpret_cast<CREATESTRUCTW*>(lParam);
        SetWindowLongPtr(hwnd, GWLP_USERDATA,
                         reinterpret_cast<LONG_PTR>(cs->lpCreateParams));
    }

    auto* app = reinterpret_cast<App*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
    if (app)
        return app->WndProc(hwnd, msg, wParam, lParam);

    return DefWindowProc(hwnd, msg, wParam, lParam);
}

void App::UpdatePathfindingInput(bool wantMouseByImGui)
{
    const bool lDown = (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;
    const bool lClick = lDown && !m_prevPathPickLButton;
    m_prevPathPickLButton = lDown;

    if (m_editor.GetMode() != EditorMode::Pathfinding)
    {
        m_pathfinding.draggingStart = false;
        m_pathfinding.draggingEnd = false;
        return;
    }

    if (wantMouseByImGui || (GetAsyncKeyState(VK_MENU) & 0x8000) != 0)
        return;

    if ((GetAsyncKeyState(VK_ESCAPE) & 0x8000) != 0)
    {
        m_pathfinding.draggingStart = false;
        m_pathfinding.draggingEnd = false;
    }

    RECT rc = {};
    GetClientRect(m_hwnd, &rc);
    const int vpW = rc.right - rc.left;
    const int vpH = rc.bottom - rc.top;

    if (IsIconic(m_hwnd) || vpW <= 0 || vpH <= 0)
        return;

    POINT cursor = {};
    GetCursorPos(&cursor);
    ScreenToClient(m_hwnd, &cursor);
    const XMFLOAT2 mousePos =
    {
        static_cast<float>(cursor.x),
        static_cast<float>(cursor.y)
    };

    const XMMATRIX viewProj = XMMatrixMultiply(
        m_camera->GetViewMatrix(),
        m_camera->GetProjMatrix(static_cast<float>(vpW) / static_cast<float>(vpH)));

    auto updateDraggedHandle = [this]()
    {
        if (!m_cursorHitValid)
            return false;

        bool changed = false;
        if (m_pathfinding.draggingStart)
        {
            m_pathfinding.startPos = m_cursorHitPos;
            m_pathfinding.hasStart = true;
            changed = true;
        }
        if (m_pathfinding.draggingEnd)
        {
            m_pathfinding.endPos = m_cursorHitPos;
            m_pathfinding.hasEnd = true;
            changed = true;
        }
        if (changed && m_pathfinding.hasStart && m_pathfinding.hasEnd)
            ComputePathfindingPreview();
        return changed;
    };

    if ((m_pathfinding.draggingStart || m_pathfinding.draggingEnd) && lDown)
    {
        updateDraggedHandle();
        return;
    }

    if ((m_pathfinding.draggingStart || m_pathfinding.draggingEnd) && !lDown)
    {
        m_pathfinding.draggingStart = false;
        m_pathfinding.draggingEnd = false;
        return;
    }

    if (!lClick)
        return;

    const float handlePickRadius = 12.0f;
    XMFLOAT2 startScreen = {};
    XMFLOAT2 endScreen = {};
    const bool hasStartScreen =
        m_pathfinding.hasStart && WorldToScreenPoint(m_pathfinding.startPos, viewProj, vpW, vpH, startScreen);
    const bool hasEndScreen =
        m_pathfinding.hasEnd && WorldToScreenPoint(m_pathfinding.endPos, viewProj, vpW, vpH, endScreen);

    if (hasStartScreen && Distance2D(mousePos, startScreen) <= handlePickRadius)
    {
        m_pathfinding.draggingStart = true;
        m_pathfinding.draggingEnd = false;
        updateDraggedHandle();
        SetStatusMessage("Dragging pathfinding start");
    }
    else if (hasEndScreen && Distance2D(mousePos, endScreen) <= handlePickRadius)
    {
        m_pathfinding.draggingStart = false;
        m_pathfinding.draggingEnd = true;
        updateDraggedHandle();
        SetStatusMessage("Dragging pathfinding end");
    }
}

void App::DrawPathfindingPreview()
{
    if (m_pathfinding.previewPath.size() >= 2)
    {
        const XMFLOAT4 pathColor = { 1.0f, 0.75f, 0.2f, 1.0f };
        for (size_t i = 1; i < m_pathfinding.previewPath.size(); ++i)
            m_debugDraw.AddLine(m_pathfinding.previewPath[i - 1], m_pathfinding.previewPath[i], pathColor);
    }
}

void App::DrawPathfindingOverlay(XMMATRIX viewProj, int vpW, int vpH)
{
    if (m_editor.GetMode() != EditorMode::Pathfinding)
        return;

    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    auto drawHandle = [&](XMFLOAT3 worldPos, ImU32 fillColor, bool active, const char* label)
    {
        XMFLOAT2 screenPos = {};
        if (!WorldToScreenPoint(worldPos, viewProj, vpW, vpH, screenPos))
            return;

        const float radius = active ? 9.0f : 7.0f;
        const ImVec2 center(screenPos.x, screenPos.y);
        dl->AddCircleFilled(center, radius, fillColor, 20);
        dl->AddCircle(center, radius + 2.0f, IM_COL32(255, 255, 255, 220), 20, 2.0f);
        dl->AddText(ImVec2(center.x + 12.0f, center.y - 9.0f), fillColor, label);
    };

    if (m_pathfinding.hasStart)
    {
        drawHandle(
            m_pathfinding.startPos,
            IM_COL32(60, 220, 90, 255),
            m_pathfinding.draggingStart,
            "Start");
    }

    if (m_pathfinding.hasEnd)
    {
        drawHandle(
            m_pathfinding.endPos,
            IM_COL32(235, 90, 90, 255),
            m_pathfinding.draggingEnd,
            "End");
    }
}

void App::DrawContourPreview()
{
    if (!m_showContours || m_contourSegments.empty())
        return;

    const XMFLOAT4 contourColor = { m_contourColor.x, m_contourColor.y, m_contourColor.z, 0.85f };
    for (const ContourSegment& segment : m_contourSegments)
        m_debugDraw.AddLine(segment.a, segment.b, contourColor);
}

bool App::ComputePathfindingPreview()
{
    m_pathfinding.previewPath.clear();

    if (!m_pathfinding.hasStart || !m_pathfinding.hasEnd)
    {
        SetStatusMessage("Pick start and end first");
        return false;
    }

    PathGridConfig grid;
    if (!BuildPathGridConfig(
            m_terrain.get(),
            m_pathfinding.startPos,
            m_pathfinding.endPos,
            m_pathfinding.gridStep,
            grid))
    {
        SetStatusMessage("Pathfinding grid setup failed");
        return false;
    }

    int startCol = 0;
    int startRow = 0;
    int endCol = 0;
    int endRow = 0;
    if (!WorldToGrid(grid, m_pathfinding.startPos, startCol, startRow) ||
        !WorldToGrid(grid, m_pathfinding.endPos, endCol, endRow))
    {
        SetStatusMessage("Start or end is outside the search area");
        return false;
    }

    const int cellCount = grid.cols * grid.rows;
    if (cellCount <= 0 || cellCount > 1500000)
    {
        SetStatusMessage("Pathfinding grid is too large");
        return false;
    }

    auto toIndex = [&grid](int col, int row)
    {
        return row * grid.cols + col;
    };

    auto heuristic = [&grid](int col, int row, int targetCol, int targetRow)
    {
        const float dx = static_cast<float>(targetCol - col) * grid.step;
        const float dz = static_cast<float>(targetRow - row) * grid.step;
        return sqrtf(dx * dx + dz * dz);
    };

    struct OpenNode
    {
        float score = 0.0f;
        int cellIndex = -1;
        int dirIndex = -1;

        bool operator<(const OpenNode& other) const
        {
            return score > other.score;
        }
    };

    const int startIndex = toIndex(startCol, startRow);
    const int endIndex = toIndex(endCol, endRow);
    const float maxGrade = (std::max)(0.0f, m_pathfinding.maxGradePercent) * 0.01f;
    const float slopePenalty = (std::max)(0.0f, m_pathfinding.slopePenalty);
    const float turnPenalty = (std::max)(0.0f, m_pathfinding.turnPenalty);

    constexpr std::array<int, 8> kNeighborDx = { -1, 0, 1, -1, 1, -1, 0, 1 };
    constexpr std::array<int, 8> kNeighborDz = { -1, -1, -1, 0, 0, 1, 1, 1 };
    constexpr int kDirectionCount = 8;

    auto toStateIndex = [cellCount](int cellIndex, int dirIndex)
    {
        return dirIndex * cellCount + cellIndex;
    };

    auto turnStepCount = [](int fromDir, int toDir)
    {
        const int diff = abs(fromDir - toDir);
        return (std::min)(diff, 8 - diff);
    };

    if (startIndex == endIndex)
    {
        m_pathfinding.previewPath = { m_pathfinding.startPos, m_pathfinding.endPos };
        SetStatusMessage("Path preview computed (2 points)");
        return true;
    }

    const int stateCount = cellCount * kDirectionCount;
    std::vector<float> bestCost(stateCount, (std::numeric_limits<float>::max)());
    std::vector<int> parentState(stateCount, -1);
    std::vector<unsigned char> closed(stateCount, 0);
    std::priority_queue<OpenNode> openSet;

    const XMFLOAT3 startPosWorld = GridToWorld(grid, startCol, startRow, m_terrain.get());
    for (int dirIndex = 0; dirIndex < kDirectionCount; ++dirIndex)
    {
        const int nextCol = startCol + kNeighborDx[dirIndex];
        const int nextRow = startRow + kNeighborDz[dirIndex];
        if (nextCol < 0 || nextCol >= grid.cols || nextRow < 0 || nextRow >= grid.rows)
            continue;

        const int nextCellIndex = toIndex(nextCol, nextRow);
        const XMFLOAT3 nextPos = GridToWorld(grid, nextCol, nextRow, m_terrain.get());
        const float horizontalDistance = DistanceXZ(startPosWorld, nextPos);
        if (horizontalDistance <= 1e-4f)
            continue;

        const float dy = nextPos.y - startPosWorld.y;
        const float grade = fabsf(dy) / horizontalDistance;
        if (m_pathfinding.strictMaxGrade && grade > maxGrade)
            continue;

        float moveCost = horizontalDistance + fabsf(dy);
        if (!m_pathfinding.strictMaxGrade && grade > maxGrade)
            moveCost += (grade - maxGrade) * slopePenalty * horizontalDistance;
        else
            moveCost += grade * 0.1f * horizontalDistance;

        const int stateIndex = toStateIndex(nextCellIndex, dirIndex);
        bestCost[stateIndex] = moveCost;
        openSet.push({ moveCost + heuristic(nextCol, nextRow, endCol, endRow), nextCellIndex, dirIndex });
    }

    int bestEndState = -1;

    while (!openSet.empty())
    {
        const OpenNode current = openSet.top();
        openSet.pop();

        if (current.cellIndex < 0 || current.cellIndex >= cellCount ||
            current.dirIndex < 0 || current.dirIndex >= kDirectionCount)
            continue;

        const int currentStateIndex = toStateIndex(current.cellIndex, current.dirIndex);
        if (closed[currentStateIndex] != 0)
            continue;

        closed[currentStateIndex] = 1;
        if (current.cellIndex == endIndex)
        {
            bestEndState = currentStateIndex;
            break;
        }

        const int row = current.cellIndex / grid.cols;
        const int col = current.cellIndex % grid.cols;
        const XMFLOAT3 currentPos = GridToWorld(grid, col, row, m_terrain.get());

        for (int nextDirIndex = 0; nextDirIndex < kDirectionCount; ++nextDirIndex)
        {
            const int nextCol = col + kNeighborDx[nextDirIndex];
            const int nextRow = row + kNeighborDz[nextDirIndex];
            if (nextCol < 0 || nextCol >= grid.cols || nextRow < 0 || nextRow >= grid.rows)
                continue;

            const int nextCellIndex = toIndex(nextCol, nextRow);
            const int nextStateIndex = toStateIndex(nextCellIndex, nextDirIndex);
            if (closed[nextStateIndex] != 0)
                continue;

            const XMFLOAT3 nextPos = GridToWorld(grid, nextCol, nextRow, m_terrain.get());
            const float horizontalDistance = DistanceXZ(currentPos, nextPos);
            if (horizontalDistance <= 1e-4f)
                continue;

            const float dy = nextPos.y - currentPos.y;
            const float grade = fabsf(dy) / horizontalDistance;
            if (m_pathfinding.strictMaxGrade && grade > maxGrade)
                continue;

            float moveCost = horizontalDistance + fabsf(dy);
            if (!m_pathfinding.strictMaxGrade && grade > maxGrade)
                moveCost += (grade - maxGrade) * slopePenalty * horizontalDistance;
            else
                moveCost += grade * 0.1f * horizontalDistance;

            const int turnSteps = turnStepCount(current.dirIndex, nextDirIndex);
            moveCost += turnPenalty * (static_cast<float>(turnSteps) * 0.5f);

            const float tentativeCost = bestCost[currentStateIndex] + moveCost;
            if (tentativeCost >= bestCost[nextStateIndex])
                continue;

            bestCost[nextStateIndex] = tentativeCost;
            parentState[nextStateIndex] = currentStateIndex;
            openSet.push(
                {
                    tentativeCost + heuristic(nextCol, nextRow, endCol, endRow),
                    nextCellIndex,
                    nextDirIndex
                });
        }
    }

    if (bestEndState < 0)
    {
        SetStatusMessage("No route found for the current grade settings");
        return false;
    }

    std::vector<XMFLOAT3> reversedPath;
    for (int stateIndex = bestEndState; stateIndex >= 0; stateIndex = parentState[stateIndex])
    {
        const int index = stateIndex % cellCount;
        const int row = index / grid.cols;
        const int col = index % grid.cols;
        reversedPath.push_back(GridToWorld(grid, col, row, m_terrain.get()));
        if (parentState[stateIndex] < 0)
            break;
    }

    if (reversedPath.empty())
    {
        SetStatusMessage("Pathfinding produced an empty path");
        return false;
    }

    m_pathfinding.previewPath.assign(reversedPath.rbegin(), reversedPath.rend());
    m_pathfinding.previewPath.insert(m_pathfinding.previewPath.begin(), m_pathfinding.startPos);
    m_pathfinding.previewPath.back() = m_pathfinding.endPos;
    SetStatusMessage(
        "Path preview computed (" + std::to_string(m_pathfinding.previewPath.size()) + " points)");
    return true;
}

bool App::ApplyPathfindingPreviewAsRoad()
{
    if (m_pathfinding.previewPath.size() < 2)
    {
        SetStatusMessage("Compute a path preview first");
        return false;
    }

    const std::vector<XMFLOAT3> controlPoints = SmoothPathForRoadControls(
        m_pathfinding.previewPath,
        m_terrain.get(),
        m_pathfinding.gridStep);
    if (controlPoints.size() < 2)
    {
        SetStatusMessage("Path smoothing failed");
        return false;
    }

    const int roadIndex = m_editor.GetActiveRoadIndex();
    if (roadIndex < 0 || roadIndex >= static_cast<int>(m_roadNetwork.roads.size()))
    {
        SetStatusMessage("Select a road before applying pathfinding");
        return false;
    }

    m_editor.RecordUndoState();
    Road& road = m_roadNetwork.roads[roadIndex];
    const std::string preservedStartIntersectionId = road.startIntersectionId;
    const std::string preservedEndIntersectionId = road.endIntersectionId;
    road.points.clear();
    road.points.reserve(controlPoints.size());
    for (const XMFLOAT3& pos : controlPoints)
        road.points.push_back({ pos, 3.0f });
    road.startIntersectionId = preservedStartIntersectionId;
    road.endIntersectionId = preservedEndIntersectionId;
    if (!road.points.empty())
    {
        if (!road.startIntersectionId.empty())
        {
            for (const Intersection& intersection : m_roadNetwork.intersections)
            {
                if (intersection.id == road.startIntersectionId)
                {
                    road.points.front().pos = intersection.pos;
                    break;
                }
            }
        }
        if (!road.endIntersectionId.empty())
        {
            for (const Intersection& intersection : m_roadNetwork.intersections)
            {
                if (intersection.id == road.endIntersectionId)
                {
                    road.points.back().pos = intersection.pos;
                    break;
                }
            }
        }
    }

    m_editor.SetMode(EditorMode::Navigate);
    ResetPathfindingState();
    SetStatusMessage(
        "Selected road updated from pathfinding (" + std::to_string(road.points.size()) + " points)");
    return true;
}

void App::DrawPathfindingPanel()
{
    ImGui::SetNextWindowPos(ImVec2(340, 30), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(320, 260), ImGuiCond_FirstUseEver);
    bool open = true;
    if (!ImGui::Begin(u8"\u7D4C\u8DEF\u63A2\u7D22", &open))
    {
        ImGui::End();
        if (!open)
        {
            m_editor.SetMode(EditorMode::Navigate);
            ResetPathfindingState();
        }
        return;
    }

    ImGui::TextDisabled(u8"\u5730\u5F62\u30B0\u30EA\u30C3\u30C9\u30D9\u30FC\u30B9\u306E A* \u7D4C\u8DEF\u30D7\u30EC\u30D3\u30E5\u30FC");
    ImGui::TextDisabled(u8"\u30D3\u30E5\u30FC\u30DD\u30FC\u30C8\u3067\u59CB\u70B9\u30FB\u7D42\u70B9\u30CF\u30F3\u30C9\u30EB\u3092\u30C9\u30E9\u30C3\u30B0");
    ImGui::Separator();

    if (ImGui::Button(u8"\u9078\u629E\u4E2D\u306E\u9053\u8DEF\u3092\u4F7F\u7528", ImVec2(-1, 0)))
        SyncPathfindingEndpointsFromSelectedRoad();

    if (m_pathfinding.hasStart)
        ImGui::Text(u8"\u59CB\u70B9  %.1f, %.1f, %.1f", m_pathfinding.startPos.x, m_pathfinding.startPos.y, m_pathfinding.startPos.z);
    else
        ImGui::TextDisabled(u8"\u59CB\u70B9  \u672A\u8A2D\u5B9A");

    if (m_pathfinding.hasEnd)
        ImGui::Text(u8"\u7D42\u70B9  %.1f, %.1f, %.1f", m_pathfinding.endPos.x, m_pathfinding.endPos.y, m_pathfinding.endPos.z);
    else
        ImGui::TextDisabled(u8"\u7D42\u70B9  \u672A\u8A2D\u5B9A");

    ImGui::Separator();

    if (ImGui::SliderFloat(u8"\u6700\u5927\u52FE\u914D (%)", &m_pathfinding.maxGradePercent, 0.0f, 100.0f, "%.1f") &&
        m_pathfinding.hasStart && m_pathfinding.hasEnd)
        ComputePathfindingPreview();

    if (ImGui::SliderFloat(u8"\u30B0\u30EA\u30C3\u30C9\u9593\u9694 (m)", &m_pathfinding.gridStep, 1.0f, 100.0f, "%.1f") &&
        m_pathfinding.hasStart && m_pathfinding.hasEnd)
        ComputePathfindingPreview();

    ImGui::Checkbox(u8"\u6700\u5927\u52FE\u914D\u3092\u53B3\u5B88", &m_pathfinding.strictMaxGrade);
    if (ImGui::IsItemEdited() && m_pathfinding.hasStart && m_pathfinding.hasEnd)
        ComputePathfindingPreview();
    if (!m_pathfinding.strictMaxGrade)
    {
        ImGui::InputFloat(u8"\u52FE\u914D\u30DA\u30CA\u30EB\u30C6\u30A3", &m_pathfinding.slopePenalty, 5.0f, 20.0f, "%.1f");
        const float clampedPenalty = std::clamp(m_pathfinding.slopePenalty, 0.0f, 1000.0f);
        if (fabsf(clampedPenalty - m_pathfinding.slopePenalty) > 1e-4f)
            m_pathfinding.slopePenalty = clampedPenalty;
        if (ImGui::IsItemDeactivatedAfterEdit() && m_pathfinding.hasStart && m_pathfinding.hasEnd)
            ComputePathfindingPreview();
    }

    if (ImGui::SliderFloat("Turn Penalty", &m_pathfinding.turnPenalty, 0.0f, 1000.0f, "%.1f") &&
        m_pathfinding.hasStart && m_pathfinding.hasEnd)
        ComputePathfindingPreview();

    ImGui::Separator();

    if (ImGui::Button(u8"\u9053\u8DEF\u306B\u9069\u7528", ImVec2(-1, 0)))
        ApplyPathfindingPreviewAsRoad();

    if (ImGui::Button(u8"\u30AF\u30EA\u30A2", ImVec2(-1, 0)))
    {
        m_pathfinding.draggingStart = false;
        m_pathfinding.draggingEnd = false;
        m_pathfinding.hasStart = false;
        m_pathfinding.hasEnd = false;
        m_pathfinding.previewPath.clear();
        SetStatusMessage("Pathfinding preview cleared");
    }

    ImGui::Separator();
    ImGui::TextDisabled(u8"\u30CF\u30F3\u30C9\u30EB\u3092\u30AF\u30EA\u30C3\u30AF\u3057\u3066\u5730\u5F62\u4E0A\u3092\u30C9\u30E9\u30C3\u30B0");
    ImGui::TextDisabled(u8"\u30C9\u30E9\u30C3\u30B0\u4E2D\u306F\u30D7\u30EC\u30D3\u30E5\u30FC\u3092\u81EA\u52D5\u518D\u8A08\u7B97");

    ImGui::End();

    if (!open)
    {
        m_editor.SetMode(EditorMode::Navigate);
        ResetPathfindingState();
        SetStatusMessage("Pathfinding closed");
    }
}

LRESULT App::WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (ImGui_ImplWin32_WndProcHandler(hwnd, msg, wParam, lParam))
        return TRUE;

    switch (msg)
    {
    case WM_SIZE:
        if (m_d3d && wParam != SIZE_MINIMIZED)
            m_d3d->Resize(LOWORD(lParam), HIWORD(lParam));
        return 0;

    case WM_SETCURSOR:
        if (LOWORD(lParam) == HTCLIENT && !ImGui::GetIO().WantCaptureMouse)
        {
            bool alt  = (GetAsyncKeyState(VK_MENU)    & 0x8000) != 0;
            bool lmb  = (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;
            bool mmb  = (GetAsyncKeyState(VK_MBUTTON) & 0x8000) != 0;
            if (alt)
            {
                LPCTSTR id = (lmb || mmb) ? IDC_SIZEALL : IDC_HAND;
                SetCursor(LoadCursor(nullptr, id));
                return TRUE;
            }
        }
        break;

    case WM_MOUSEWHEEL:
        if (!ImGui::GetIO().WantCaptureMouse)
        {
            float delta = static_cast<float>(GET_WHEEL_DELTA_WPARAM(wParam))
                        / static_cast<float>(WHEEL_DELTA);
            m_camera->OnScroll(delta);
        }
        return 0;

    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }

    return DefWindowProc(hwnd, msg, wParam, lParam);
}
