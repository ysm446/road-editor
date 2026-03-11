#include "DebugDraw.h"

using namespace DirectX;
using Microsoft::WRL::ComPtr;

// ---------------------------------------------------------------------------
bool DebugDraw::Initialize(ID3D11Device* device)
{
    m_deviceRef = device;

    if (!m_shader.LoadVertex(device, L"shaders/line_vs.hlsl"))
        return false;
    if (!m_shader.LoadPixel(device, L"shaders/line_ps.hlsl"))
        return false;

    // Input layout
    D3D11_INPUT_ELEMENT_DESC layout[] =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT,    0,  0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
        { "COLOR",    0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
    };
    HRESULT hr = device->CreateInputLayout(
        layout, 2,
        m_shader.GetVSBlob()->GetBufferPointer(),
        m_shader.GetVSBlob()->GetBufferSize(),
        &m_inputLayout);
    if (FAILED(hr))
        return false;

    // Alpha-blend state
    D3D11_BLEND_DESC bd = {};
    bd.RenderTarget[0].BlendEnable           = TRUE;
    bd.RenderTarget[0].SrcBlend              = D3D11_BLEND_SRC_ALPHA;
    bd.RenderTarget[0].DestBlend             = D3D11_BLEND_INV_SRC_ALPHA;
    bd.RenderTarget[0].BlendOp               = D3D11_BLEND_OP_ADD;
    bd.RenderTarget[0].SrcBlendAlpha         = D3D11_BLEND_ONE;
    bd.RenderTarget[0].DestBlendAlpha        = D3D11_BLEND_ZERO;
    bd.RenderTarget[0].BlendOpAlpha          = D3D11_BLEND_OP_ADD;
    bd.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
    device->CreateBlendState(&bd, &m_blendState);

    // Depth: always draw on top of terrain (depth test disabled)
    D3D11_DEPTH_STENCIL_DESC dsd = {};
    dsd.DepthEnable    = FALSE;
    dsd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
    dsd.DepthFunc      = D3D11_COMPARISON_ALWAYS;
    device->CreateDepthStencilState(&dsd, &m_depthState);

    // Initial vertex buffer
    GrowBuffer(device, 1024);

    return true;
}

// ---------------------------------------------------------------------------
void DebugDraw::Shutdown()
{
    m_vb.Reset();
    m_inputLayout.Reset();
    m_blendState.Reset();
    m_depthState.Reset();
}

// ---------------------------------------------------------------------------
void DebugDraw::AddLine(XMFLOAT3 a, XMFLOAT3 b, XMFLOAT4 color)
{
    m_lines.push_back({ a, color });
    m_lines.push_back({ b, color });
}

// ---------------------------------------------------------------------------
void DebugDraw::Flush(ID3D11DeviceContext* ctx, ID3D11Buffer* perFrameCB)
{
    if (m_lines.empty())
        return;

    UINT needed = static_cast<UINT>(m_lines.size());
    if (needed > m_vbCapacity)
        GrowBuffer(m_deviceRef, needed * 2);

    // Upload
    D3D11_MAPPED_SUBRESOURCE ms = {};
    ctx->Map(m_vb.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &ms);
    memcpy(ms.pData, m_lines.data(), m_lines.size() * sizeof(LineVertex));
    ctx->Unmap(m_vb.Get(), 0);

    // States
    float blendFactor[4] = {};
    ctx->OMSetBlendState(m_blendState.Get(), blendFactor, 0xFFFFFFFF);
    ctx->OMSetDepthStencilState(m_depthState.Get(), 0);

    m_shader.Bind(ctx);
    ctx->VSSetConstantBuffers(0, 1, &perFrameCB);

    ctx->IASetInputLayout(m_inputLayout.Get());
    ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);

    UINT stride = sizeof(LineVertex), offset = 0;
    ctx->IASetVertexBuffers(0, 1, m_vb.GetAddressOf(), &stride, &offset);

    ctx->Draw(needed, 0);

    // Restore defaults
    ctx->OMSetBlendState(nullptr, blendFactor, 0xFFFFFFFF);
    ctx->OMSetDepthStencilState(nullptr, 0);

    m_lines.clear();
}

// ---------------------------------------------------------------------------
void DebugDraw::GrowBuffer(ID3D11Device* device, UINT required)
{
    m_vb.Reset();
    m_vbCapacity = required;

    D3D11_BUFFER_DESC bd = {};
    bd.ByteWidth      = required * sizeof(LineVertex);
    bd.Usage          = D3D11_USAGE_DYNAMIC;
    bd.BindFlags      = D3D11_BIND_VERTEX_BUFFER;
    bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    device->CreateBuffer(&bd, nullptr, &m_vb);
}
