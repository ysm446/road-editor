#pragma once

#include <Windows.h>
#include <memory>
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
};                                      // = 144 bytes (multiple of 16)

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
    void    Render();

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

    DebugDraw      m_debugDraw;
    RoadNetwork    m_roadNetwork;
    PolylineEditor m_editor;
};
