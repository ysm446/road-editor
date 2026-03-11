#include "Shader.h"
#include <d3dcompiler.h>
#include <Windows.h>
#include <string>

#pragma comment(lib, "d3dcompiler.lib")

static bool CompileFromFile(const std::wstring& path, const std::string& entry,
                             const std::string& target, ID3DBlob** ppBlob)
{
    UINT flags = D3DCOMPILE_ENABLE_STRICTNESS;
#ifdef _DEBUG
    flags |= D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;
#endif

    Microsoft::WRL::ComPtr<ID3DBlob> errBlob;
    HRESULT hr = D3DCompileFromFile(
        path.c_str(), nullptr, D3D_COMPILE_STANDARD_FILE_INCLUDE,
        entry.c_str(), target.c_str(), flags, 0, ppBlob, &errBlob);

    if (FAILED(hr))
    {
        std::string msg = "Shader compile failed: " + std::string(path.begin(), path.end()) + "\n";
        if (errBlob)
            msg += static_cast<const char*>(errBlob->GetBufferPointer());
        OutputDebugStringA(msg.c_str());
        MessageBoxA(nullptr, msg.c_str(), "Shader Error", MB_OK | MB_ICONERROR);
        return false;
    }
    return true;
}

bool Shader::LoadVertex(ID3D11Device* device, const std::wstring& path,
                         const std::string& entry)
{
    if (!CompileFromFile(path, entry, "vs_5_0", &m_vsBlob))
        return false;

    return SUCCEEDED(device->CreateVertexShader(
        m_vsBlob->GetBufferPointer(), m_vsBlob->GetBufferSize(), nullptr, &m_vs));
}

bool Shader::LoadPixel(ID3D11Device* device, const std::wstring& path,
                        const std::string& entry)
{
    Microsoft::WRL::ComPtr<ID3DBlob> blob;
    if (!CompileFromFile(path, entry, "ps_5_0", &blob))
        return false;

    return SUCCEEDED(device->CreatePixelShader(
        blob->GetBufferPointer(), blob->GetBufferSize(), nullptr, &m_ps));
}

void Shader::Bind(ID3D11DeviceContext* ctx) const
{
    ctx->VSSetShader(m_vs.Get(), nullptr, 0);
    ctx->PSSetShader(m_ps.Get(), nullptr, 0);
}
