#include "D3D11Context.h"

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")

bool D3D11Context::Initialize(HWND hwnd, uint32_t width, uint32_t height)
{
    m_width  = width;
    m_height = height;

    DXGI_SWAP_CHAIN_DESC scd                        = {};
    scd.BufferCount                                  = 2;
    scd.BufferDesc.Width                             = width;
    scd.BufferDesc.Height                            = height;
    scd.BufferDesc.Format                            = DXGI_FORMAT_R8G8B8A8_UNORM;
    scd.BufferDesc.RefreshRate.Numerator             = 60;
    scd.BufferDesc.RefreshRate.Denominator           = 1;
    scd.BufferUsage                                  = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    scd.OutputWindow                                 = hwnd;
    scd.SampleDesc.Count                             = 1;
    scd.Windowed                                     = TRUE;
    scd.SwapEffect                                   = DXGI_SWAP_EFFECT_DISCARD;

    UINT deviceFlags = 0;
#ifdef _DEBUG
    deviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

    D3D_FEATURE_LEVEL levels[]    = { D3D_FEATURE_LEVEL_11_0 };
    D3D_FEATURE_LEVEL featureLevel = {};

    HRESULT hr = D3D11CreateDeviceAndSwapChain(
        nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
        deviceFlags, levels, 1, D3D11_SDK_VERSION,
        &scd, &m_swapChain, &m_device, &featureLevel, &m_context);

    // Graphics Tools (debug layer) が未インストールの場合はフォールバック
    if (FAILED(hr) && (deviceFlags & D3D11_CREATE_DEVICE_DEBUG))
    {
        deviceFlags &= ~D3D11_CREATE_DEVICE_DEBUG;
        hr = D3D11CreateDeviceAndSwapChain(
            nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
            deviceFlags, levels, 1, D3D11_SDK_VERSION,
            &scd, &m_swapChain, &m_device, &featureLevel, &m_context);
    }

    if (FAILED(hr))
    {
        MessageBoxW(hwnd, L"D3D11CreateDeviceAndSwapChain failed.", L"D3D11 Error", MB_OK | MB_ICONERROR);
        return false;
    }

    CreateRenderTarget();
    return true;
}

void D3D11Context::CreateRenderTarget()
{
    Microsoft::WRL::ComPtr<ID3D11Texture2D> backBuffer;
    m_swapChain->GetBuffer(0, IID_PPV_ARGS(&backBuffer));
    m_device->CreateRenderTargetView(backBuffer.Get(), nullptr, &m_rtv);

    D3D11_TEXTURE2D_DESC dd  = {};
    dd.Width                  = m_width;
    dd.Height                 = m_height;
    dd.MipLevels              = 1;
    dd.ArraySize              = 1;
    dd.Format                 = DXGI_FORMAT_D24_UNORM_S8_UINT;
    dd.SampleDesc.Count       = 1;
    dd.Usage                  = D3D11_USAGE_DEFAULT;
    dd.BindFlags              = D3D11_BIND_DEPTH_STENCIL;

    m_device->CreateTexture2D(&dd, nullptr, &m_depthBuffer);
    m_device->CreateDepthStencilView(m_depthBuffer.Get(), nullptr, &m_dsv);
}

void D3D11Context::ReleaseRenderTarget()
{
    m_rtv.Reset();
    m_dsv.Reset();
    m_depthBuffer.Reset();
}

void D3D11Context::Resize(uint32_t width, uint32_t height)
{
    if (width == 0 || height == 0)
        return;

    m_width  = width;
    m_height = height;

    m_context->OMSetRenderTargets(0, nullptr, nullptr);
    ReleaseRenderTarget();
    m_swapChain->ResizeBuffers(0, width, height, DXGI_FORMAT_UNKNOWN, 0);
    CreateRenderTarget();
}

void D3D11Context::BeginFrame(float r, float g, float b)
{
    D3D11_VIEWPORT vp = {};
    vp.Width    = static_cast<float>(m_width);
    vp.Height   = static_cast<float>(m_height);
    vp.MaxDepth = 1.0f;
    m_context->RSSetViewports(1, &vp);

    float clearColor[4] = { r, g, b, 1.0f };
    m_context->OMSetRenderTargets(1, m_rtv.GetAddressOf(), m_dsv.Get());
    m_context->ClearRenderTargetView(m_rtv.Get(), clearColor);
    m_context->ClearDepthStencilView(m_dsv.Get(), D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);
}

void D3D11Context::EndFrame()
{
    m_swapChain->Present(1, 0);
}

void D3D11Context::Shutdown()
{
    ReleaseRenderTarget();
    m_swapChain.Reset();
    m_context.Reset();
    m_device.Reset();
}
