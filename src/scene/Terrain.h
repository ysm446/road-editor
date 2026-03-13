#pragma once

#include <d3d11.h>
#include <wrl/client.h>
#include <DirectXMath.h>
#include <string>
#include <vector>

#include "renderer/Shader.h"
#include "renderer/Buffer.h"

// Constant buffer for terrain shader (slot b1), must be 16 bytes.
struct TerrainCB
{
    DirectX::XMFLOAT3 sunDir;    // 12 bytes  (normalized, pointing toward light)
    float             maxHeight; //  4 bytes
    int               colorMode; //  4 bytes
    int               lightingMode; // 4 bytes
    DirectX::XMFLOAT2 shadowMapTexelSize; // 8 bytes
    float             shadowStrength; // 4 bytes
    float             shadowBias; // 4 bytes
    DirectX::XMFLOAT2 padding; // 8 bytes
};                               // = 48 bytes

struct TerrainShadowCB
{
    DirectX::XMFLOAT4X4 lightViewProj;
};

class Terrain
{
public:
    enum LightingMode
    {
        LightingModeBasic = 0,
        LightingModeSunShadowed = 1
    };

    bool Initialize(ID3D11Device* device);

    // Generate a procedural sinusoidal heightmap.
    void GenerateProcedural(ID3D11Device* device, int width = 256, int height = 256);

    // Load a grayscale PNG/BMP/TGA via stb_image at native resolution.
    // Returns false if the file cannot be opened.
    bool LoadFromFile(ID3D11Device* device, const char* path);
    bool LoadColorTexture(ID3D11Device* device, const char* path);
    void ClearColorTexture();

    // Rebuild vertex/index buffers using current heightScale / horizontalScale.
    void Rebuild(ID3D11Device* device);

    void RenderShadowMap(ID3D11DeviceContext* ctx, const DirectX::XMFLOAT4X4& lightViewProj);
    void Render(ID3D11DeviceContext* ctx,
                ID3D11Buffer* perFrameCB,
                const DirectX::XMFLOAT4X4& lightViewProj);
    void Shutdown();
    void Reset();

    bool IsReady()    const { return m_ready;  }
    int  GetRawW()    const { return m_rawW;   }  // native image resolution
    int  GetRawH()    const { return m_rawH;   }
    int  GetMeshW()   const { return m_meshW;  }  // actual mesh vertex count X
    int  GetMeshH()   const { return m_meshH;  }

    // Sample the height (Y) at any world-space XZ position.
    // Returns 0 if the terrain is not ready or the point is outside bounds.
    float GetHeightAt(float worldX, float worldZ) const;

    // Cast a ray against the terrain using iterative refinement.
    // Returns true and fills hitPos if an intersection is found.
    bool Raycast(DirectX::XMFLOAT3 rayOrigin, DirectX::XMFLOAT3 rayDir,
                 DirectX::XMFLOAT3& hitPos) const;

    // Adjustable parameters (change then call Rebuild).
    // horizontalScaleX/Z: metres per grid cell in X and Z.
    // heightScale: maps normalised height [0,1] to metres.
    // meshSubdivW/H: mesh cell count (0 = use native image resolution - 1).
    float heightScale      = 100.0f;
    float horizontalScaleX =   1.0f;
    float horizontalScaleZ =   1.0f;
    float offsetX          =   0.0f;
    float offsetY          =   0.0f;
    float offsetZ          =   0.0f;
    int   meshSubdivW      =   0;
    int   meshSubdivH      =   0;
    int   colorMode        =   1;
    int   lightingMode     =   LightingModeBasic;
    bool  wireframe        = false;
    bool  visible          = true;
    DirectX::XMFLOAT3 sunDirection = { 0.6f, 1.0f, 0.4f };
    float shadowStrength   = 0.72f;
    float shadowBias       = 0.0015f;
    std::string colorTexturePath;

private:
    struct Vertex
    {
        DirectX::XMFLOAT3 pos;
        DirectX::XMFLOAT3 normal;
        DirectX::XMFLOAT2 uv;
    };

    void BuildMesh(ID3D11Device* device);

    Shader m_shader;
    Microsoft::WRL::ComPtr<ID3D11VertexShader>    m_shadowVS;
    Microsoft::WRL::ComPtr<ID3D11Buffer>          m_vb;
    Microsoft::WRL::ComPtr<ID3D11Buffer>          m_ib;
    Microsoft::WRL::ComPtr<ID3D11InputLayout>     m_inputLayout;
    Microsoft::WRL::ComPtr<ID3D11RasterizerState> m_rsSolid;
    Microsoft::WRL::ComPtr<ID3D11RasterizerState> m_rsWireframe;
    Microsoft::WRL::ComPtr<ID3D11RasterizerState> m_rsShadow;
    Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> m_colorTextureSRV;
    Microsoft::WRL::ComPtr<ID3D11SamplerState>       m_colorTextureSampler;
    Microsoft::WRL::ComPtr<ID3D11Texture2D>          m_shadowMap;
    Microsoft::WRL::ComPtr<ID3D11DepthStencilView>   m_shadowDSV;
    Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> m_shadowSRV;
    Microsoft::WRL::ComPtr<ID3D11SamplerState>       m_shadowSampler;
    ConstantBuffer<TerrainCB>                     m_terrainCB;
    ConstantBuffer<TerrainShadowCB>               m_shadowCB;

    std::vector<float> m_rawHeights; // normalised heights [0, 1] at native resolution
    int      m_rawW       = 0;  // native image width
    int      m_rawH       = 0;  // native image height
    int      m_meshW      = 0;  // actual mesh vertex count X (after subdivision)
    int      m_meshH      = 0;  // actual mesh vertex count Z
    uint32_t m_indexCount = 0;
    bool     m_ready      = false;
};
