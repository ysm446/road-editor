#pragma once

#include <d3d11.h>
#include <d3dcommon.h>
#include <wrl/client.h>
#include <string>

class Shader
{
public:
    bool LoadVertex(ID3D11Device* device, const std::wstring& path,
                    const std::string& entry = "main");
    bool LoadPixel (ID3D11Device* device, const std::wstring& path,
                    const std::string& entry = "main");

    void Bind(ID3D11DeviceContext* ctx) const;

    // Expose VS bytecode for input-layout creation (Phase 2+)
    ID3DBlob* GetVSBlob() const { return m_vsBlob.Get(); }

private:
    Microsoft::WRL::ComPtr<ID3D11VertexShader> m_vs;
    Microsoft::WRL::ComPtr<ID3D11PixelShader>  m_ps;
    Microsoft::WRL::ComPtr<ID3DBlob>           m_vsBlob;
};
