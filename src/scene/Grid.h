#pragma once

#include <d3d11.h>
#include <wrl/client.h>
#include "renderer/Shader.h"

// Renders an infinite XZ-plane grid using a fullscreen-triangle technique.
// No vertex buffer required — SV_VertexID drives the VS.
class Grid
{
public:
    bool Initialize(ID3D11Device* device);
    void Render(ID3D11DeviceContext* ctx, ID3D11Buffer* perFrameCB);
    void Shutdown();

private:
    Shader                                          m_shader;
    Microsoft::WRL::ComPtr<ID3D11BlendState>        m_blendState;
    Microsoft::WRL::ComPtr<ID3D11DepthStencilState> m_dsState;
    Microsoft::WRL::ComPtr<ID3D11RasterizerState>   m_rsState;
};
