#pragma once

#include <d3d11.h>
#include <wrl/client.h>
#include <DirectXMath.h>
#include <vector>

#include "renderer/Shader.h"
#include "renderer/Buffer.h"

// Constant buffer for terrain shader (slot b1), must be 16 bytes.
struct TerrainCB
{
    DirectX::XMFLOAT3 sunDir;    // 12 bytes  (normalized, pointing toward light)
    float             maxHeight; //  4 bytes
};                               // = 16 bytes

class Terrain
{
public:
    bool Initialize(ID3D11Device* device);

    // Generate a procedural sinusoidal heightmap.
    void GenerateProcedural(ID3D11Device* device, int width = 256, int height = 256);

    // Load a grayscale PNG/BMP/TGA via stb_image at native resolution.
    // Returns false if the file cannot be opened.
    bool LoadFromFile(ID3D11Device* device, const char* path);

    // Rebuild vertex/index buffers using current heightScale / horizontalScale.
    void Rebuild(ID3D11Device* device);

    void Render(ID3D11DeviceContext* ctx, ID3D11Buffer* perFrameCB);
    void Shutdown();

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
    int   meshSubdivW      =   0;
    int   meshSubdivH      =   0;
    bool  wireframe        = false;
    bool  visible          = true;

private:
    struct Vertex
    {
        DirectX::XMFLOAT3 pos;
        DirectX::XMFLOAT3 normal;
        DirectX::XMFLOAT2 uv;
    };

    void BuildMesh(ID3D11Device* device);

    Shader m_shader;
    Microsoft::WRL::ComPtr<ID3D11Buffer>          m_vb;
    Microsoft::WRL::ComPtr<ID3D11Buffer>          m_ib;
    Microsoft::WRL::ComPtr<ID3D11InputLayout>     m_inputLayout;
    Microsoft::WRL::ComPtr<ID3D11RasterizerState> m_rsSolid;
    Microsoft::WRL::ComPtr<ID3D11RasterizerState> m_rsWireframe;
    ConstantBuffer<TerrainCB>                     m_terrainCB;

    std::vector<float> m_rawHeights; // normalised heights [0, 1] at native resolution
    int      m_rawW       = 0;  // native image width
    int      m_rawH       = 0;  // native image height
    int      m_meshW      = 0;  // actual mesh vertex count X (after subdivision)
    int      m_meshH      = 0;  // actual mesh vertex count Z
    uint32_t m_indexCount = 0;
    bool     m_ready      = false;
};
