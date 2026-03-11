#include "Grid.h"

bool Grid::Initialize(ID3D11Device* device)
{
    if (!m_shader.LoadVertex(device, L"shaders/grid_vs.hlsl"))
        return false;
    if (!m_shader.LoadPixel(device, L"shaders/grid_ps.hlsl"))
        return false;

    // Alpha blending (src_alpha / inv_src_alpha)
    D3D11_BLEND_DESC bd                            = {};
    bd.RenderTarget[0].BlendEnable                  = TRUE;
    bd.RenderTarget[0].SrcBlend                     = D3D11_BLEND_SRC_ALPHA;
    bd.RenderTarget[0].DestBlend                    = D3D11_BLEND_INV_SRC_ALPHA;
    bd.RenderTarget[0].BlendOp                      = D3D11_BLEND_OP_ADD;
    bd.RenderTarget[0].SrcBlendAlpha                = D3D11_BLEND_ONE;
    bd.RenderTarget[0].DestBlendAlpha               = D3D11_BLEND_ZERO;
    bd.RenderTarget[0].BlendOpAlpha                 = D3D11_BLEND_OP_ADD;
    bd.RenderTarget[0].RenderTargetWriteMask        = D3D11_COLOR_WRITE_ENABLE_ALL;
    device->CreateBlendState(&bd, &m_blendState);

    // Depth: test enabled, write enabled (custom depth output in PS)
    D3D11_DEPTH_STENCIL_DESC dsd = {};
    dsd.DepthEnable    = TRUE;
    dsd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
    dsd.DepthFunc      = D3D11_COMPARISON_LESS;
    device->CreateDepthStencilState(&dsd, &m_dsState);

    // No backface culling (grid is double-sided)
    D3D11_RASTERIZER_DESC rd = {};
    rd.FillMode        = D3D11_FILL_SOLID;
    rd.CullMode        = D3D11_CULL_NONE;
    rd.DepthClipEnable = TRUE;
    device->CreateRasterizerState(&rd, &m_rsState);

    return true;
}

void Grid::Render(ID3D11DeviceContext* ctx, ID3D11Buffer* perFrameCB)
{
    m_shader.Bind(ctx);

    ctx->VSSetConstantBuffers(0, 1, &perFrameCB);
    ctx->PSSetConstantBuffers(0, 1, &perFrameCB);

    float blendFactor[4] = {};
    ctx->OMSetBlendState(m_blendState.Get(), blendFactor, 0xFFFFFFFF);
    ctx->OMSetDepthStencilState(m_dsState.Get(), 0);
    ctx->RSSetState(m_rsState.Get());

    // No vertex buffer - SV_VertexID generates a fullscreen triangle
    ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ctx->IASetInputLayout(nullptr);
    ctx->IASetVertexBuffers(0, 0, nullptr, nullptr, nullptr);
    ctx->Draw(3, 0);

    // Reset blend state so subsequent renderers get the default
    ctx->OMSetBlendState(nullptr, blendFactor, 0xFFFFFFFF);
    ctx->OMSetDepthStencilState(nullptr, 0);
    ctx->RSSetState(nullptr);
}

void Grid::Shutdown()
{
    m_blendState.Reset();
    m_dsState.Reset();
    m_rsState.Reset();
}
