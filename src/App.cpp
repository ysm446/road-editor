#include "App.h"
#include "imgui.h"
#include "imgui_impl_win32.h"

#include <chrono>
#include <commdlg.h>
#include <DirectXMath.h>

// Forward declaration required by imgui_impl_win32
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(
    HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

using namespace DirectX;

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
        CW_USEDEFAULT, CW_USEDEFAULT, 1600, 1200,
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

        auto   now = std::chrono::high_resolution_clock::now();
        float  dt  = std::chrono::duration<float>(now - lastTime).count();
        lastTime    = now;
        m_time     += dt;

        bool wantMouse = ImGui::GetIO().WantCaptureMouse;
        m_camera->HandleInput(wantMouse);

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

            // Terrain cursor raycast
            m_cursorHitValid = false;
            if (m_terrain->IsReady() && vpW > 0 && vpH > 0)
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

                m_cursorHitValid = m_terrain->Raycast(rayO, rayD3, m_cursorHitPos);
            }
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

    m_editor.DrawNetwork(m_debugDraw);
    m_debugDraw.Flush(m_d3d->GetContext(), m_perFrameCB.Get());

    // ImGui
    m_imgui->BeginFrame();

    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("ImGui Demo", nullptr, false, false);
            ImGui::EndMenu();
        }
        ImGui::Text("  FPS: %.0f", ImGui::GetIO().Framerate);
        ImGui::EndMainMenuBar();
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
    ImGui::SetNextWindowSize(ImVec2(290, 280), ImGuiCond_Always);
    ImGui::Begin("Terrain");
    {
        ImGui::Checkbox("Visible",   &m_terrain->visible);
        ImGui::SameLine();
        ImGui::Checkbox("Wireframe", &m_terrain->wireframe);

        ImGui::Separator();

        // --- Size / scale in metres ---
        if (m_terrain->IsReady())
        {
            int W = m_terrain->GetRawW();
            int H = m_terrain->GetRawH();
            ImGui::Text("Resolution: %d x %d px", W, H);

            // Derive current size in metres (editing buffers, not committed yet)
            float widthM  = (W - 1) * m_terrain->horizontalScaleX;
            float depthM  = (H - 1) * m_terrain->horizontalScaleZ;
            float heightM = m_terrain->heightScale;

            // Rebuild when editing is confirmed (Enter or focus lost)
            bool changed = false;
            ImGui::InputFloat("Width (m)",  &widthM,  0, 0, "%.0f");
            if (ImGui::IsItemDeactivatedAfterEdit())
            {
                if (widthM  >= 1.0f) m_terrain->horizontalScaleX = widthM  / (W - 1);
                changed = true;
            }
            ImGui::InputFloat("Depth (m)",  &depthM,  0, 0, "%.0f");
            if (ImGui::IsItemDeactivatedAfterEdit())
            {
                if (depthM  >= 1.0f) m_terrain->horizontalScaleZ = depthM  / (H - 1);
                changed = true;
            }
            ImGui::InputFloat("Height (m)", &heightM, 0, 0, "%.0f");
            if (ImGui::IsItemDeactivatedAfterEdit())
            {
                if (heightM >= 1.0f) m_terrain->heightScale = heightM;
                changed = true;
            }
            ImGui::TextDisabled("Enter or click away to apply");
            if (changed)
                m_terrain->Rebuild(m_d3d->GetDevice());
        }
        else
        {
            ImGui::TextDisabled("No terrain loaded");
        }

        ImGui::Separator();

        // --- Load heightmap from file ---
        ImGui::Text("Load Heightmap");
        ImGui::SetNextItemWidth(-60);
        ImGui::InputText("##hmap", m_terrainPath, sizeof(m_terrainPath));
        ImGui::SameLine();
        if (ImGui::Button("Browse"))
        {
            OpenFileDialog(m_hwnd, m_terrainPath, sizeof(m_terrainPath),
                           "Image Files\0*.png;*.bmp;*.tga;*.jpg\0All Files\0*.*\0",
                           "Open Heightmap");
        }
        // Resolution override (0 = native)
        ImGui::SetNextItemWidth(110);
        ImGui::InputInt("W##res", &m_loadResW, 0, 0);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110);
        ImGui::InputInt("H##res", &m_loadResH, 0, 0);
        ImGui::SameLine();
        ImGui::TextDisabled("(0=native)");
        if (m_loadResW < 0) m_loadResW = 0;
        if (m_loadResH < 0) m_loadResH = 0;
        if (m_loadResW > 4096) m_loadResW = 4096;
        if (m_loadResH > 4096) m_loadResH = 4096;

        if (ImGui::Button("Load", ImVec2(-1, 0)))
        {
            if (!m_terrain->LoadFromFile(m_d3d->GetDevice(), m_terrainPath,
                                         m_loadResW, m_loadResH))
                ImGui::OpenPopup("LoadError");
        }
        if (ImGui::BeginPopup("LoadError"))
        {
            ImGui::Text("Failed to load: %s", m_terrainPath);
            if (ImGui::Button("OK")) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }

    }
    ImGui::End();

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
