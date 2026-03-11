#pragma once

#include <Windows.h>
#include <d3d11.h>

class ImGuiLayer
{
public:
    bool Initialize(HWND hwnd, ID3D11Device* device, ID3D11DeviceContext* ctx);
    void BeginFrame();
    void EndFrame();
    void Shutdown();
};
