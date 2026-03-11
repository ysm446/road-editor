#pragma once

#include <d3d11.h>
#include <DirectXMath.h>
#include <wrl/client.h>
#include <vector>

#include "Shader.h"

// Dynamic line renderer - submit lines each frame, flushed in Flush().
class DebugDraw
{
public:
    bool Initialize(ID3D11Device* device);
    void Shutdown();

    // Queue a 3-D line segment with the given colour
    void AddLine(DirectX::XMFLOAT3 a, DirectX::XMFLOAT3 b,
                 DirectX::XMFLOAT4 color = { 1,1,1,1 });

    // Draw all queued lines and clear the queue
    void Flush(ID3D11DeviceContext* ctx, ID3D11Buffer* perFrameCB);

private:
    struct LineVertex
    {
        DirectX::XMFLOAT3 pos;
        DirectX::XMFLOAT4 color;
    };

    void GrowBuffer(ID3D11Device* device, UINT required);

    Shader   m_shader;

    Microsoft::WRL::ComPtr<ID3D11Buffer>      m_vb;
    Microsoft::WRL::ComPtr<ID3D11InputLayout> m_inputLayout;

    // Rasterizer / blend / depth states
    Microsoft::WRL::ComPtr<ID3D11BlendState>        m_blendState;
    Microsoft::WRL::ComPtr<ID3D11DepthStencilState> m_depthState;

    UINT                    m_vbCapacity = 0;
    std::vector<LineVertex> m_lines;     // accumulated this frame

    ID3D11Device* m_deviceRef = nullptr; // non-owning
};
