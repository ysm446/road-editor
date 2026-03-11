#pragma once

#include <d3d11.h>
#include <dxgi.h>
#include <wrl/client.h>
#include <cstdint>

class D3D11Context
{
public:
    bool Initialize(HWND hwnd, uint32_t width, uint32_t height);
    void Resize(uint32_t width, uint32_t height);
    void BeginFrame(float r = 0.12f, float g = 0.12f, float b = 0.14f);
    void EndFrame();
    void Shutdown();

    ID3D11Device*        GetDevice()  const { return m_device.Get();  }
    ID3D11DeviceContext* GetContext() const { return m_context.Get(); }
    uint32_t             GetWidth()   const { return m_width;         }
    uint32_t             GetHeight()  const { return m_height;        }

private:
    void CreateRenderTarget();
    void ReleaseRenderTarget();

    Microsoft::WRL::ComPtr<ID3D11Device>            m_device;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext>      m_context;
    Microsoft::WRL::ComPtr<IDXGISwapChain>           m_swapChain;
    Microsoft::WRL::ComPtr<ID3D11RenderTargetView>   m_rtv;
    Microsoft::WRL::ComPtr<ID3D11DepthStencilView>   m_dsv;
    Microsoft::WRL::ComPtr<ID3D11Texture2D>          m_depthBuffer;

    uint32_t m_width  = 0;
    uint32_t m_height = 0;
};
