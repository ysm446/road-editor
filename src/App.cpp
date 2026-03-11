#include "App.h"
#include "imgui.h"
#include "imgui_impl_win32.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <commdlg.h>
#include <DirectXMath.h>
#include <fstream>
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
    ImDrawList* dl = ImGui::GetForegroundDrawList();
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
    m_terrain->offsetZ          = m_loadOffsetZ;
    m_terrain->Rebuild(m_d3d->GetDevice());
}

void App::LoadViewSettings()
{
    m_showRoadNames = false;
    m_showIntersectionNames = true;

    try
    {
        std::ifstream ifs(kViewSettingsPath);
        if (ifs)
        {
            nlohmann::json root;
            ifs >> root;
            m_showRoadNames = root.value("showRoadNames", false);
            m_showIntersectionNames = root.value("showIntersectionNames", true);
        }
    }
    catch (...)
    {
    }

    m_editor.SetShowRoadNames(m_showRoadNames);
    m_editor.SetShowIntersectionNames(m_showIntersectionNames);
}

void App::SaveViewSettings() const
{
    try
    {
        nlohmann::json root =
        {
            { "showRoadNames", m_showRoadNames },
            { "showIntersectionNames", m_showIntersectionNames }
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

void App::ResetPathfindingState()
{
    m_pathfinding = PathfindingState();
    m_prevPathPickLButton = false;
}

void App::NewProject()
{
    strncpy_s(m_terrainPath, sizeof(m_terrainPath), "data/heightmap.png", _TRUNCATE);
    strncpy_s(m_projectPath, sizeof(m_projectPath), "data/project.json", _TRUNCATE);

    m_loadResW = 0;
    m_loadResH = 0;
    m_loadWidthM = 255.0f;
    m_loadDepthM = 255.0f;
    m_loadHeightM = 100.0f;
    m_loadOffsetX = 0.0f;
    m_loadOffsetZ = 0.0f;

    m_cursorHitValid = false;
    m_cursorHitPos = {};
    ResetPathfindingState();

    m_terrain->Reset();
    m_roadNetwork = RoadNetwork();
    m_editor.SetNetwork(&m_roadNetwork);
    m_editor.SetFilePath("data/roads.json");
    m_editor.ResetState();
    m_camera->SetOrbitState({ 0.0f, 0.0f, 0.0f }, 20.0f, 0.785f, 0.4f);
    SetStatusMessage("New project");
}

bool App::SaveProject(const char* path)
{
    try
    {
        if (m_editor.GetFilePath()[0] != '\0' && !m_editor.Save(m_editor.GetFilePath()))
        {
            SetStatusMessage(std::string("Road save failed: ") + m_editor.GetFilePath());
            return false;
        }

        nlohmann::json root;
        root["version"] = 1;
        root["terrain"] =
        {
            { "path",        m_terrainPath            },
            { "divisionsX",  m_loadResW               },
            { "divisionsY",  m_loadResH               },
            { "widthM",      m_loadWidthM             },
            { "depthM",      m_loadDepthM             },
            { "heightM",     m_loadHeightM            },
            { "offsetX",     m_loadOffsetX            },
            { "offsetZ",     m_loadOffsetZ            },
            { "visible",     m_terrain->visible       },
            { "wireframe",   m_terrain->wireframe     }
        };
        root["roads"] =
        {
            { "path", m_editor.GetFilePath() }
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
            std::string terrainPath = t.value("path", std::string());
            strncpy_s(m_terrainPath, sizeof(m_terrainPath), terrainPath.c_str(), _TRUNCATE);
            m_loadResW    = t.value("divisionsX", 0);
            m_loadResH    = t.value("divisionsY", 0);
            m_loadWidthM  = t.value("widthM", 255.0f);
            m_loadDepthM  = t.value("depthM", 255.0f);
            m_loadHeightM = t.value("heightM", 100.0f);
            m_loadOffsetX = t.value("offsetX", 0.0f);
            m_loadOffsetZ = t.value("offsetZ", 0.0f);
            m_terrain->visible   = t.value("visible", true);
            m_terrain->wireframe = t.value("wireframe", false);

            if (m_terrainPath[0] != '\0')
            {
                if (!m_terrain->LoadFromFile(m_d3d->GetDevice(), m_terrainPath))
                {
                    SetStatusMessage(std::string("Terrain load failed: ") + m_terrainPath);
                    return false;
                }
                ApplyTerrainSettings();
            }
        }

        if (root.contains("roads"))
        {
            const auto& r = root["roads"];
            std::string roadPath = r.value("path", std::string());
            m_editor.SetFilePath(roadPath.c_str());
            if (roadPath.empty())
            {
                m_roadNetwork.roads.clear();
            }
            else if (!m_editor.Load(roadPath.c_str()))
            {
                SetStatusMessage(std::string("Road load failed: ") + roadPath);
                return false;
            }
        }

        if (root.contains("camera"))
        {
            const auto& c = root["camera"];
            m_camera->SetOrbitState(
                {
                    c.value("targetX", 0.0f),
                    c.value("targetY", 0.0f),
                    c.value("targetZ", 0.0f)
                },
                c.value("distance", 20.0f),
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

// --- Window ------------------------------------------------------------------

bool App::CreateAppWindow(HINSTANCE hInstance, int nCmdShow)
{
    WNDCLASSEXW wc    = {};
    wc.cbSize          = sizeof(wc);
    wc.style           = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc     = StaticWndProc;
    wc.hInstance       = hInstance;
    wc.hCursor         = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground   = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    wc.lpszClassName   = L"RoadEditorClass";

    if (!RegisterClassExW(&wc))
        return false;

    m_hwnd = CreateWindowExW(
        0, L"RoadEditorClass",
        L"Road Editor - Phase 3",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT, 1600, 900,
        nullptr, nullptr, hInstance,
        this);  // passed to WM_NCCREATE as CREATESTRUCT::lpCreateParams

    if (!m_hwnd)
        return false;

    ShowWindow(m_hwnd, nCmdShow);
    UpdateWindow(m_hwnd);
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
    m_camera->Initialize(20.0f, 0.785f, 0.4f);

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

        const bool focusKeyDown = (GetAsyncKeyState('F') & 0x8000) != 0;
        const bool focusPressed = focusKeyDown && !m_prevFocusKey;
        m_prevFocusKey = focusKeyDown;

        // Editor update (must happen after camera input, before render)
        {
            RECT rc = {};
            GetClientRect(m_hwnd, &rc);
            int vpW = rc.right  - rc.left;
            int vpH = rc.bottom - rc.top;

            POINT cursor = {};
            GetCursorPos(&cursor);
            ScreenToClient(m_hwnd, &cursor);

            XMMATRIX vp = XMMatrixMultiply(m_camera->GetViewMatrix(),
                                            m_camera->GetProjMatrix(
                                                static_cast<float>(vpW) /
                                                static_cast<float>(vpH)));
            XMMATRIX invVP = XMMatrixInverse(nullptr, vp);
            m_editor.Update(vpW, vpH,
                            { static_cast<float>(cursor.x),
                              static_cast<float>(cursor.y) },
                            invVP, wantMouse);

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

            UpdatePathfindingInput(wantMouse);
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
    m_d3d->BeginFrame();

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
    m_perFrameCB.Update(m_d3d->GetContext(), pfd);

    // 3D scene  (order: terrain -> grid -> debug lines)
    m_terrain->Render(m_d3d->GetContext(), m_perFrameCB.Get());
    m_grid->Render(m_d3d->GetContext(), m_perFrameCB.Get());

    m_editor.DrawNetwork(m_debugDraw, vp, m_d3d->GetWidth(), m_d3d->GetHeight());
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
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("New Project"))
                NewProject();
            if (ImGui::MenuItem("Open Project..."))
            {
                if (OpenFileDialog(m_hwnd, m_projectPath, sizeof(m_projectPath),
                                   "Project Files\0*.json\0All Files\0*.*\0",
                                   "Open Project"))
                {
                    if (!LoadProject(m_projectPath))
                        ImGui::OpenPopup("ProjectLoadError");
                }
            }
            if (ImGui::MenuItem("Save Project"))
            {
                if (!SaveProject(m_projectPath))
                    ImGui::OpenPopup("ProjectSaveError");
            }
            if (ImGui::MenuItem("Save Project As..."))
            {
                if (SaveFileDialog(m_hwnd, m_projectPath, sizeof(m_projectPath),
                                   "Project Files\0*.json\0All Files\0*.*\0",
                                   "Save Project"))
                {
                    if (!SaveProject(m_projectPath))
                        ImGui::OpenPopup("ProjectSaveError");
                }
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View"))
        {
            if (ImGui::MenuItem("Road Names", nullptr, m_showRoadNames))
            {
                m_showRoadNames = !m_showRoadNames;
                m_editor.SetShowRoadNames(m_showRoadNames);
                SaveViewSettings();
            }
            if (ImGui::MenuItem("Intersection Names", nullptr, m_showIntersectionNames))
            {
                m_showIntersectionNames = !m_showIntersectionNames;
                m_editor.SetShowIntersectionNames(m_showIntersectionNames);
                SaveViewSettings();
            }
            ImGui::MenuItem("ImGui Demo", nullptr, false, false);
            ImGui::EndMenu();
        }
        ImGui::Text("  FPS: %.0f", ImGui::GetIO().Framerate);
        ImGui::EndMainMenuBar();
    }

    if (ImGui::BeginPopup("ProjectSaveError"))
    {
        ImGui::Text("Project save failed: %s", m_projectPath);
        if (ImGui::Button("OK")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    if (ImGui::BeginPopup("ProjectLoadError"))
    {
        ImGui::Text("Project load failed: %s", m_projectPath);
        if (ImGui::Button("OK")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    // Camera panel
    ImGui::SetNextWindowPos(ImVec2(10, 30), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(280, 110), ImGuiCond_FirstUseEver);
    ImGui::Begin("Camera");
    {
        auto p = m_camera->GetPosition();
        ImGui::Text("Position  %.2f  %.2f  %.2f", p.x, p.y, p.z);
        ImGui::Separator();
        if (m_cursorHitValid)
            ImGui::Text("Cursor    %.2f  %.2f  %.2f",
                        m_cursorHitPos.x, m_cursorHitPos.y, m_cursorHitPos.z);
        else
            ImGui::TextDisabled("Cursor    --");
        ImGui::Separator();
        ImGui::TextDisabled("Alt + LMB drag : rotate");
        ImGui::TextDisabled("Alt + MMB drag : pan");
        ImGui::TextDisabled("Scroll wheel   : zoom");
    }
    ImGui::End();

    // Terrain panel
    ImGui::SetNextWindowPos(ImVec2(10, 150), ImGuiCond_FirstUseEver);
    ImGui::Begin("Terrain", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        ImGui::Checkbox("Visible",   &m_terrain->visible);
        ImGui::SameLine();
        ImGui::Checkbox("Wireframe", &m_terrain->wireframe);

        ImGui::Separator();

        if (m_terrain->IsReady())
        {
            int rawW = m_terrain->GetRawW();
            int rawH = m_terrain->GetRawH();
            int mW   = m_terrain->GetMeshW();
            int mH   = m_terrain->GetMeshH();
            ImGui::Text("Image: %d x %d px", rawW, rawH);
            ImGui::Text("Mesh: %d x %d verts  %d x %d cells",
                        mW, mH, mW - 1, mH - 1);
        }
        else
        {
            ImGui::TextDisabled("No terrain loaded");
        }

        ImGui::Separator();

        ImGui::TextDisabled("Load Heightmap");
        ImGui::SetNextItemWidth(-60);
        ImGui::InputText("##hmap", m_terrainPath, sizeof(m_terrainPath));
        ImGui::SameLine();
        if (ImGui::Button("Browse"))
        {
            OpenFileDialog(m_hwnd, m_terrainPath, sizeof(m_terrainPath),
                           "Image Files\0*.png;*.bmp;*.tga;*.jpg\0All Files\0*.*\0",
                           "Open Heightmap");
        }

        bool applyToCurrentTerrain = false;

        int loadDivisions[2] = { m_loadResW, m_loadResH };
        ImGui::InputInt2("Divisions X / Z", loadDivisions);
        m_loadResW = loadDivisions[0];
        m_loadResH = loadDivisions[1];
        if (ImGui::IsItemDeactivatedAfterEdit())
            applyToCurrentTerrain = true;
        ImGui::TextDisabled("Value = wireframe cell count, 0 = image size - 1");
        if (m_loadResW < 0) m_loadResW = 0;
        if (m_loadResH < 0) m_loadResH = 0;
        if (m_loadResW > 4096) m_loadResW = 4096;
        if (m_loadResH > 4096) m_loadResH = 4096;

        float loadSizeM[3] = { m_loadWidthM, m_loadDepthM, m_loadHeightM };
        ImGui::InputFloat3("Size X / Z / Y (m)", loadSizeM, "%.0f");
        m_loadWidthM  = loadSizeM[0];
        m_loadDepthM  = loadSizeM[1];
        m_loadHeightM = loadSizeM[2];
        if (ImGui::IsItemDeactivatedAfterEdit())
            applyToCurrentTerrain = true;
        if (m_loadWidthM < 1.0f) m_loadWidthM = 1.0f;
        if (m_loadDepthM < 1.0f) m_loadDepthM = 1.0f;
        if (m_loadHeightM < 1.0f) m_loadHeightM = 1.0f;

        float loadOffsetM[2] = { m_loadOffsetX, m_loadOffsetZ };
        ImGui::InputFloat2("Offset X / Z (m)", loadOffsetM, "%.1f");
        m_loadOffsetX = loadOffsetM[0];
        m_loadOffsetZ = loadOffsetM[1];
        if (ImGui::IsItemDeactivatedAfterEdit())
            applyToCurrentTerrain = true;

        if (applyToCurrentTerrain && m_terrain->IsReady())
            ApplyTerrainSettings();

        if (ImGui::Button("Load", ImVec2(-1, 0)))
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
        if (ImGui::Button("Clear Height Field", ImVec2(-1, 0)))
        {
            m_terrain->Reset();
            m_loadResW = 0;
            m_loadResH = 0;
            m_loadWidthM = 255.0f;
            m_loadDepthM = 255.0f;
            m_loadHeightM = 100.0f;
            m_loadOffsetX = 0.0f;
            m_loadOffsetZ = 0.0f;
            SetStatusMessage("Height field cleared");
        }
        if (ImGui::BeginPopup("LoadError"))
        {
            ImGui::Text("Failed to load: %s", m_terrainPath);
            if (ImGui::Button("OK")) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }

    }
    ImGui::End();

    if (m_editor.GetMode() == EditorMode::Pathfinding)
        DrawPathfindingPanel();

    // 2D overlay: road point circles
    {
        RECT rc = {};
        GetClientRect(m_hwnd, &rc);
        int vpW = rc.right  - rc.left;
        int vpH = rc.bottom - rc.top;
        m_editor.DrawOverlay(vp, vpW, vpH);
    }

    // Road editor panel
    m_editor.DrawUI(m_d3d->GetDevice());

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
        m_pathfinding.pickingStart = false;
        m_pathfinding.pickingEnd = false;
        return;
    }

    if (wantMouseByImGui || (GetAsyncKeyState(VK_MENU) & 0x8000) != 0)
        return;

    if ((GetAsyncKeyState(VK_ESCAPE) & 0x8000) != 0)
    {
        m_pathfinding.pickingStart = false;
        m_pathfinding.pickingEnd = false;
    }

    if (!lClick || !m_cursorHitValid)
        return;

    if (m_pathfinding.pickingStart)
    {
        m_pathfinding.startPos = m_cursorHitPos;
        m_pathfinding.hasStart = true;
        m_pathfinding.pickingStart = false;
        SetStatusMessage("Pathfinding start picked");
    }
    else if (m_pathfinding.pickingEnd)
    {
        m_pathfinding.endPos = m_cursorHitPos;
        m_pathfinding.hasEnd = true;
        m_pathfinding.pickingEnd = false;
        SetStatusMessage("Pathfinding end picked");
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

    auto drawMarker = [this](XMFLOAT3 pos, XMFLOAT4 color)
    {
        const float radius = 1.5f;
        m_debugDraw.AddLine(
            { pos.x - radius, pos.y, pos.z },
            { pos.x + radius, pos.y, pos.z },
            color);
        m_debugDraw.AddLine(
            { pos.x, pos.y, pos.z - radius },
            { pos.x, pos.y, pos.z + radius },
            color);
    };

    if (m_pathfinding.hasStart)
        drawMarker(m_pathfinding.startPos, { 0.2f, 1.0f, 0.3f, 1.0f });
    if (m_pathfinding.hasEnd)
        drawMarker(m_pathfinding.endPos, { 1.0f, 0.3f, 0.3f, 1.0f });
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
        int index = -1;

        bool operator<(const OpenNode& other) const
        {
            return score > other.score;
        }
    };

    const int startIndex = toIndex(startCol, startRow);
    const int endIndex = toIndex(endCol, endRow);
    const float maxGrade = (std::max)(0.0f, m_pathfinding.maxGradePercent) * 0.01f;
    const float slopePenalty = (std::max)(0.0f, m_pathfinding.slopePenalty);

    std::vector<float> bestCost(cellCount, (std::numeric_limits<float>::max)());
    std::vector<int> parent(cellCount, -1);
    std::vector<unsigned char> closed(cellCount, 0);
    std::priority_queue<OpenNode> openSet;

    bestCost[startIndex] = 0.0f;
    openSet.push({ heuristic(startCol, startRow, endCol, endRow), startIndex });

    constexpr std::array<int, 8> kNeighborDx = { -1, 0, 1, -1, 1, -1, 0, 1 };
    constexpr std::array<int, 8> kNeighborDz = { -1, -1, -1, 0, 0, 1, 1, 1 };

    while (!openSet.empty())
    {
        const OpenNode current = openSet.top();
        openSet.pop();

        if (current.index < 0 || current.index >= cellCount || closed[current.index] != 0)
            continue;

        closed[current.index] = 1;
        if (current.index == endIndex)
            break;

        const int row = current.index / grid.cols;
        const int col = current.index % grid.cols;
        const XMFLOAT3 currentPos = GridToWorld(grid, col, row, m_terrain.get());

        for (size_t i = 0; i < kNeighborDx.size(); ++i)
        {
            const int nextCol = col + kNeighborDx[i];
            const int nextRow = row + kNeighborDz[i];
            if (nextCol < 0 || nextCol >= grid.cols || nextRow < 0 || nextRow >= grid.rows)
                continue;

            const int nextIndex = toIndex(nextCol, nextRow);
            if (closed[nextIndex] != 0)
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

            const float tentativeCost = bestCost[current.index] + moveCost;
            if (tentativeCost >= bestCost[nextIndex])
                continue;

            bestCost[nextIndex] = tentativeCost;
            parent[nextIndex] = current.index;
            openSet.push(
                {
                    tentativeCost + heuristic(nextCol, nextRow, endCol, endRow),
                    nextIndex
                });
        }
    }

    if (parent[endIndex] < 0 && endIndex != startIndex)
    {
        SetStatusMessage("No route found for the current grade settings");
        return false;
    }

    std::vector<XMFLOAT3> reversedPath;
    for (int index = endIndex; index >= 0; index = parent[index])
    {
        const int row = index / grid.cols;
        const int col = index % grid.cols;
        reversedPath.push_back(GridToWorld(grid, col, row, m_terrain.get()));
        if (index == startIndex)
            break;
    }

    if (reversedPath.empty())
    {
        SetStatusMessage("Pathfinding produced an empty path");
        return false;
    }

    m_pathfinding.previewPath.assign(reversedPath.rbegin(), reversedPath.rend());
    m_pathfinding.previewPath.front() = m_pathfinding.startPos;
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

    m_editor.RecordUndoState();
    const int roadIndex = m_roadNetwork.AddRoad("Path Road " + std::to_string(m_roadNetwork.roads.size()));
    if (roadIndex < 0 || roadIndex >= static_cast<int>(m_roadNetwork.roads.size()))
    {
        SetStatusMessage("Failed to create road from preview");
        return false;
    }

    Road& road = m_roadNetwork.roads[roadIndex];
    road.points.clear();
    road.points.reserve(controlPoints.size());
    for (const XMFLOAT3& pos : controlPoints)
        road.points.push_back({ pos, 3.0f });

    m_editor.SetMode(EditorMode::Navigate);
    SetStatusMessage(
        "Path applied as smoothed control points (" + std::to_string(road.points.size()) + ")");
    return true;
}

void App::DrawPathfindingPanel()
{
    ImGui::SetNextWindowPos(ImVec2(340, 30), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(320, 260), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Pathfinding"))
    {
        ImGui::End();
        return;
    }

    ImGui::TextDisabled("Terrain/grid based A* route preview");
    ImGui::Separator();

    if (ImGui::Button(m_pathfinding.pickingStart ? "Picking Start..." : "Pick Start", ImVec2(120, 0)))
    {
        m_pathfinding.pickingStart = true;
        m_pathfinding.pickingEnd = false;
    }
    ImGui::SameLine();
    if (m_pathfinding.hasStart)
        ImGui::Text("%.1f, %.1f, %.1f", m_pathfinding.startPos.x, m_pathfinding.startPos.y, m_pathfinding.startPos.z);
    else
        ImGui::TextDisabled("Not set");

    if (ImGui::Button(m_pathfinding.pickingEnd ? "Picking End..." : "Pick End", ImVec2(120, 0)))
    {
        m_pathfinding.pickingStart = false;
        m_pathfinding.pickingEnd = true;
    }
    ImGui::SameLine();
    if (m_pathfinding.hasEnd)
        ImGui::Text("%.1f, %.1f, %.1f", m_pathfinding.endPos.x, m_pathfinding.endPos.y, m_pathfinding.endPos.z);
    else
        ImGui::TextDisabled("Not set");

    ImGui::Separator();

    ImGui::InputFloat("Max Grade (%)", &m_pathfinding.maxGradePercent, 0.5f, 5.0f, "%.1f");
    m_pathfinding.maxGradePercent = std::clamp(m_pathfinding.maxGradePercent, 0.0f, 100.0f);

    ImGui::InputFloat("Grid Step (m)", &m_pathfinding.gridStep, 0.5f, 5.0f, "%.1f");
    m_pathfinding.gridStep = std::clamp(m_pathfinding.gridStep, 1.0f, 100.0f);

    ImGui::Checkbox("Strict Max Grade", &m_pathfinding.strictMaxGrade);
    if (!m_pathfinding.strictMaxGrade)
    {
        ImGui::InputFloat("Slope Penalty", &m_pathfinding.slopePenalty, 5.0f, 20.0f, "%.1f");
        m_pathfinding.slopePenalty = std::clamp(m_pathfinding.slopePenalty, 0.0f, 1000.0f);
    }

    ImGui::Separator();

    if (ImGui::Button("Compute", ImVec2(-1, 0)))
        ComputePathfindingPreview();

    if (ImGui::Button("Apply as Road", ImVec2(-1, 0)))
        ApplyPathfindingPreviewAsRoad();

    if (ImGui::Button("Clear", ImVec2(-1, 0)))
    {
        m_pathfinding.pickingStart = false;
        m_pathfinding.pickingEnd = false;
        m_pathfinding.hasStart = false;
        m_pathfinding.hasEnd = false;
        m_pathfinding.previewPath.clear();
        SetStatusMessage("Pathfinding preview cleared");
    }

    ImGui::Separator();
    ImGui::TextDisabled("Esc: cancel picking");
    ImGui::TextDisabled("Pick start/end, then compute");

    ImGui::End();
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
