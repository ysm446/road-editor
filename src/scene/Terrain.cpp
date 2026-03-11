#include "Terrain.h"

#include <algorithm>
#include <cmath>

// stb_image - single-header image loader (included once here)
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

using namespace DirectX;

// ---------------------------------------------------------------------------
// Initialize shaders and pipeline states
// ---------------------------------------------------------------------------
bool Terrain::Initialize(ID3D11Device* device)
{
    if (!m_shader.LoadVertex(device, L"shaders/terrain_vs.hlsl"))
        return false;
    if (!m_shader.LoadPixel(device, L"shaders/terrain_ps.hlsl"))
        return false;

    // Rasterizer: solid, back-face cull
    D3D11_RASTERIZER_DESC rd = {};
    rd.FillMode        = D3D11_FILL_SOLID;
    rd.CullMode        = D3D11_CULL_BACK;
    rd.DepthClipEnable = TRUE;
    device->CreateRasterizerState(&rd, &m_rsSolid);

    // Rasterizer: wireframe, no cull
    rd.FillMode = D3D11_FILL_WIREFRAME;
    rd.CullMode = D3D11_CULL_NONE;
    device->CreateRasterizerState(&rd, &m_rsWireframe);

    if (!m_terrainCB.Initialize(device))
        return false;

    return true;
}

// ---------------------------------------------------------------------------
// Procedural heightmap (combination of sine waves)
// ---------------------------------------------------------------------------
void Terrain::GenerateProcedural(ID3D11Device* device, int width, int height)
{
    m_rawW = width;
    m_rawH = height;
    m_rawHeights.resize(width * height);

    for (int row = 0; row < height; ++row)
    {
        for (int col = 0; col < width; ++col)
        {
            float nx = static_cast<float>(col) / (width  - 1); // [0, 1]
            float nz = static_cast<float>(row) / (height - 1); // [0, 1]

            float h  = 0.0f;
            h += sinf(nx * 6.28318f * 2.0f) * cosf(nz * 6.28318f * 1.5f) * 0.50f;
            h += sinf(nx * 6.28318f * 5.0f + 1.2f) * cosf(nz * 6.28318f * 4.0f) * 0.25f;
            h += sinf(nx * 6.28318f * 9.0f - 0.7f) * cosf(nz * 6.28318f * 7.0f) * 0.12f;

            // Remap from [-~0.87, +~0.87] to [0, 1]
            m_rawHeights[row * width + col] = h * 0.55f + 0.5f;
        }
    }

    BuildMesh(device);
}

// ---------------------------------------------------------------------------
// Load from greyscale image file
// ---------------------------------------------------------------------------
bool Terrain::LoadFromFile(ID3D11Device* device, const char* path)
{
    int w, h, ch;
    stbi_uc* data = stbi_load(path, &w, &h, &ch, 1); // force 1-channel
    if (!data)
        return false;

    m_rawW = w;
    m_rawH = h;
    m_rawHeights.resize(w * h);
    for (int i = 0; i < w * h; ++i)
        m_rawHeights[i] = data[i] / 255.0f;
    stbi_image_free(data);

    BuildMesh(device);
    return true;
}

// ---------------------------------------------------------------------------
// Rebuild mesh with current scale parameters
// ---------------------------------------------------------------------------
void Terrain::Rebuild(ID3D11Device* device)
{
    if (!m_rawHeights.empty())
        BuildMesh(device);
}

// ---------------------------------------------------------------------------
// Internal mesh construction
// ---------------------------------------------------------------------------
void Terrain::BuildMesh(ID3D11Device* device)
{
    if (m_rawW < 2 || m_rawH < 2)
        return;

    m_vb.Reset();
    m_ib.Reset();
    m_inputLayout.Reset();
    m_indexCount = 0;
    m_ready      = false;

    // meshSubdivW/H are cell counts. Vertex count is cells + 1.
    const int   cellsW = (meshSubdivW > 0) ? meshSubdivW : (m_rawW - 1);
    const int   cellsH = (meshSubdivH > 0) ? meshSubdivH : (m_rawH - 1);
    const int   W      = cellsW + 1;
    const int   H      = cellsH + 1;
    m_meshW = W;
    m_meshH = H;

    const float sX     = horizontalScaleX;
    const float sZ     = horizontalScaleZ;
    const float vScale = heightScale;

    // Sample m_rawHeights via bilinear interpolation at mesh (col, row)
    auto sampleRaw = [&](float u, float v) -> float
    {
        // u,v in [0,1]
        float fc = u * (m_rawW - 1);
        float fr = v * (m_rawH - 1);
        int c0 = static_cast<int>(fc); if (c0 >= m_rawW - 1) c0 = m_rawW - 2;
        int r0 = static_cast<int>(fr); if (r0 >= m_rawH - 1) r0 = m_rawH - 2;
        int c1 = c0 + 1;
        int r1 = r0 + 1;
        float dc = fc - c0;
        float dr = fr - r0;
        float h00 = m_rawHeights[r0 * m_rawW + c0];
        float h10 = m_rawHeights[r0 * m_rawW + c1];
        float h01 = m_rawHeights[r1 * m_rawW + c0];
        float h11 = m_rawHeights[r1 * m_rawW + c1];
        return (h00 + (h10 - h00) * dc) * (1.0f - dr)
             + (h01 + (h11 - h01) * dc) * dr;
    };

    auto getH = [&](int col, int row) -> float
    {
        if (col < 0) col = 0; if (col >= W) col = W - 1;
        if (row < 0) row = 0; if (row >= H) row = H - 1;
        float u = (W > 1) ? static_cast<float>(col) / (W - 1) : 0.5f;
        float v = (H > 1) ? static_cast<float>(row) / (H - 1) : 0.5f;
        return sampleRaw(u, v) * vScale;
    };

    // Build vertices
    std::vector<Vertex> verts;
    verts.reserve(W * H);

    for (int row = 0; row < H; ++row)
    {
        for (int col = 0; col < W; ++col)
        {
            float y = getH(col, row);
            float x = (col - W * 0.5f) * sX;
            float z = (row - H * 0.5f) * sZ;

            // Finite-difference normal (tangents scaled by cell size)
            float hL = getH(col - 1, row);
            float hR = getH(col + 1, row);
            float hD = getH(col, row - 1);
            float hU = getH(col, row + 1);

            XMVECTOR tx = XMVectorSet(2.0f * sX, hR - hL, 0.0f, 0.0f);
            XMVECTOR tz = XMVectorSet(0.0f, hU - hD, 2.0f * sZ, 0.0f);
            XMVECTOR n  = XMVector3Normalize(XMVector3Cross(tz, tx));

            Vertex v;
            v.pos = { x, y, z };
            XMStoreFloat3(&v.normal, n);
            v.uv = { static_cast<float>(col) / (W - 1),
                     static_cast<float>(row) / (H - 1) };
            verts.push_back(v);
        }
    }

    // Create vertex buffer
    {
        D3D11_BUFFER_DESC bd  = {};
        bd.ByteWidth           = static_cast<UINT>(verts.size() * sizeof(Vertex));
        bd.Usage               = D3D11_USAGE_DEFAULT;
        bd.BindFlags           = D3D11_BIND_VERTEX_BUFFER;
        D3D11_SUBRESOURCE_DATA sd = { verts.data() };
        device->CreateBuffer(&bd, &sd, &m_vb);
    }

    // Build index buffer (CW winding, back-face = bottom)
    std::vector<uint32_t> indices;
    indices.reserve(static_cast<size_t>((W - 1) * (H - 1)) * 6);

    for (int row = 0; row < H - 1; ++row)
    {
        for (int col = 0; col < W - 1; ++col)
        {
            uint32_t v00 =  row      * W +  col;
            uint32_t v10 =  row      * W + (col + 1);
            uint32_t v01 = (row + 1) * W +  col;
            uint32_t v11 = (row + 1) * W + (col + 1);

            indices.push_back(v00); indices.push_back(v01); indices.push_back(v10);
            indices.push_back(v10); indices.push_back(v01); indices.push_back(v11);
        }
    }
    m_indexCount = static_cast<uint32_t>(indices.size());

    {
        D3D11_BUFFER_DESC bd  = {};
        bd.ByteWidth           = m_indexCount * sizeof(uint32_t);
        bd.Usage               = D3D11_USAGE_DEFAULT;
        bd.BindFlags           = D3D11_BIND_INDEX_BUFFER;
        D3D11_SUBRESOURCE_DATA sd = { indices.data() };
        device->CreateBuffer(&bd, &sd, &m_ib);
    }

    // Create input layout from VS bytecode
    D3D11_INPUT_ELEMENT_DESC layout[] =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,
          offsetof(Vertex, pos),    D3D11_INPUT_PER_VERTEX_DATA, 0 },
        { "NORMAL",   0, DXGI_FORMAT_R32G32B32_FLOAT, 0,
          offsetof(Vertex, normal), D3D11_INPUT_PER_VERTEX_DATA, 0 },
        { "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT,    0,
          offsetof(Vertex, uv),     D3D11_INPUT_PER_VERTEX_DATA, 0 },
    };
    device->CreateInputLayout(
        layout, 3,
        m_shader.GetVSBlob()->GetBufferPointer(),
        m_shader.GetVSBlob()->GetBufferSize(),
        &m_inputLayout);

    m_ready = true;
}

// ---------------------------------------------------------------------------
// Height query helpers
// ---------------------------------------------------------------------------
float Terrain::GetHeightAt(float worldX, float worldZ) const
{
    if (!m_ready || m_rawW < 2 || m_rawH < 2)
        return 0.0f;

    const float vScale = heightScale;

    // Convert world XZ -> normalised grid col/row
    float col = worldX / horizontalScaleX + m_rawW * 0.5f;
    float row  = worldZ / horizontalScaleZ + m_rawH * 0.5f;

    int c0 = static_cast<int>(col);
    int r0 = static_cast<int>(row);
    int c1 = c0 + 1;
    int r1 = r0 + 1;

    // Clamp
    c0 = std::clamp(c0, 0, m_rawW - 1);
    c1 = std::clamp(c1, 0, m_rawW - 1);
    r0 = std::clamp(r0, 0, m_rawH - 1);
    r1 = std::clamp(r1, 0, m_rawH - 1);

    float fc = col - static_cast<int>(col);
    float fr = row - static_cast<int>(row);
    fc = std::clamp(fc, 0.0f, 1.0f);
    fr = std::clamp(fr, 0.0f, 1.0f);

    float h00 = m_rawHeights[r0 * m_rawW + c0] * vScale;
    float h10 = m_rawHeights[r0 * m_rawW + c1] * vScale;
    float h01 = m_rawHeights[r1 * m_rawW + c0] * vScale;
    float h11 = m_rawHeights[r1 * m_rawW + c1] * vScale;

    // Bilinear interpolation
    float h0 = h00 + (h10 - h00) * fc;
    float h1 = h01 + (h11 - h01) * fc;
    return h0 + (h1 - h0) * fr;
}

bool Terrain::Raycast(XMFLOAT3 rayOrigin, XMFLOAT3 rayDir,
                      XMFLOAT3& hitPos) const
{
    if (!m_ready)
        return false;

    XMVECTOR orig = XMLoadFloat3(&rayOrigin);
    XMVECTOR dir  = XMVector3Normalize(XMLoadFloat3(&rayDir));

    // March along the ray with a coarse step, then refine
    const float maxDist  = 2000.0f;
    const float coarse   = (horizontalScaleX < horizontalScaleZ
                            ? horizontalScaleX : horizontalScaleZ) * 0.5f;

    float t = 0.0f;
    float prevDiff = 0.0f;

    for (; t < maxDist; t += coarse)
    {
        XMVECTOR p   = XMVectorAdd(orig, XMVectorScale(dir, t));
        XMFLOAT3 pf;  XMStoreFloat3(&pf, p);
        float terrH  = GetHeightAt(pf.x, pf.z);
        float diff   = pf.y - terrH;

        if (t > 0.0f && diff <= 0.0f)
        {
            // Binary search between t-coarse and t
            float lo = t - coarse;
            float hi = t;
            for (int i = 0; i < 16; ++i)
            {
                float mid = (lo + hi) * 0.5f;
                XMVECTOR mp  = XMVectorAdd(orig, XMVectorScale(dir, mid));
                XMFLOAT3 mpf; XMStoreFloat3(&mpf, mp);
                float mh = GetHeightAt(mpf.x, mpf.z);
                if (mpf.y - mh > 0.0f)
                    lo = mid;
                else
                    hi = mid;
            }
            float finalT = (lo + hi) * 0.5f;
            XMVECTOR fp  = XMVectorAdd(orig, XMVectorScale(dir, finalT));
            XMFLOAT3 fpf; XMStoreFloat3(&fpf, fp);
            fpf.y = GetHeightAt(fpf.x, fpf.z);
            hitPos = fpf;
            return true;
        }
        prevDiff = diff;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Render
// ---------------------------------------------------------------------------
void Terrain::Render(ID3D11DeviceContext* ctx, ID3D11Buffer* perFrameCB)
{
    if (!m_ready || !visible)
        return;

    // Update terrain constant buffer
    TerrainCB tcb;
    XMVECTOR  sun = XMVector3Normalize(XMVectorSet(0.6f, 1.0f, 0.4f, 0.0f));
    XMStoreFloat3(&tcb.sunDir, sun);
    tcb.maxHeight = heightScale;
    m_terrainCB.Update(ctx, tcb);

    m_shader.Bind(ctx);
    ctx->VSSetConstantBuffers(0, 1, &perFrameCB);
    ctx->PSSetConstantBuffers(0, 1, &perFrameCB);
    ctx->PSSetConstantBuffers(1, 1, m_terrainCB.GetAddressOf());

    ctx->RSSetState(wireframe ? m_rsWireframe.Get() : m_rsSolid.Get());
    ctx->IASetInputLayout(m_inputLayout.Get());
    ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    UINT stride = sizeof(Vertex), offset = 0;
    ctx->IASetVertexBuffers(0, 1, m_vb.GetAddressOf(), &stride, &offset);
    ctx->IASetIndexBuffer(m_ib.Get(), DXGI_FORMAT_R32_UINT, 0);

    ctx->DrawIndexed(m_indexCount, 0, 0);

    ctx->RSSetState(nullptr);
}

// ---------------------------------------------------------------------------
// Shutdown
// ---------------------------------------------------------------------------
void Terrain::Shutdown()
{
    m_vb.Reset();
    m_ib.Reset();
    m_inputLayout.Reset();
    m_rsSolid.Reset();
    m_rsWireframe.Reset();
}
