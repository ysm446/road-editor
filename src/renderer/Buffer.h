#pragma once

#include <d3d11.h>
#include <wrl/client.h>
#include <cstring>

// RAII wrapper for a D3D11 DYNAMIC constant buffer.
// Usage:
//   ConstantBuffer<MyStruct> cb;
//   cb.Initialize(device);
//   cb.Update(ctx, myData);
//   ctx->VSSetConstantBuffers(0, 1, cb.GetAddressOf());
template <typename T>
class ConstantBuffer
{
public:
    bool Initialize(ID3D11Device* device)
    {
        static_assert((sizeof(T) % 16) == 0,
            "ConstantBuffer size must be a multiple of 16 bytes");

        D3D11_BUFFER_DESC bd  = {};
        bd.ByteWidth           = sizeof(T);
        bd.Usage               = D3D11_USAGE_DYNAMIC;
        bd.BindFlags           = D3D11_BIND_CONSTANT_BUFFER;
        bd.CPUAccessFlags      = D3D11_CPU_ACCESS_WRITE;

        return SUCCEEDED(device->CreateBuffer(&bd, nullptr, &m_buffer));
    }

    void Update(ID3D11DeviceContext* ctx, const T& data)
    {
        D3D11_MAPPED_SUBRESOURCE mapped = {};
        ctx->Map(m_buffer.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped);
        std::memcpy(mapped.pData, &data, sizeof(T));
        ctx->Unmap(m_buffer.Get(), 0);
    }

    ID3D11Buffer*        Get()           const { return m_buffer.Get();           }
    ID3D11Buffer* const* GetAddressOf()  const { return m_buffer.GetAddressOf(); }

private:
    Microsoft::WRL::ComPtr<ID3D11Buffer> m_buffer;
};
