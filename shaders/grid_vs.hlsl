// grid_vs.hlsl
// Draws a fullscreen triangle; per-pixel ray-plane intersection is done in the PS.

#include "common.hlsli"

struct VSOutput
{
    float4 clipPos   : SV_POSITION;
    float3 nearPoint : TEXCOORD0;
    float3 farPoint  : TEXCOORD1;
};

// Unproject a clip-space point to world space
float3 UnprojectPoint(float x, float y, float z)
{
    float4 v = mul(float4(x, y, z, 1.0), invViewProj);
    return v.xyz / v.w;
}

VSOutput main(uint vid : SV_VertexID)
{
    // Fullscreen triangle — covers the entire NDC cube with 3 vertices
    static const float2 verts[3] =
    {
        float2(-1.0, -1.0),
        float2(-1.0,  3.0),
        float2( 3.0, -1.0)
    };

    float2 p = verts[vid];

    VSOutput o;
    o.clipPos   = float4(p, 0.0, 1.0);
    o.nearPoint = UnprojectPoint(p.x, p.y, 0.0); // near plane in world space
    o.farPoint  = UnprojectPoint(p.x, p.y, 1.0); // far  plane in world space
    return o;
}
