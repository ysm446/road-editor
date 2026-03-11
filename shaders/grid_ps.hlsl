// grid_ps.hlsl
// Computes infinite XZ-plane grid via ray-plane intersection.
// Outputs custom depth so the grid integrates with the depth buffer.

#include "common.hlsli"

struct PSInput
{
    float4 clipPos   : SV_POSITION;
    float3 nearPoint : TEXCOORD0;
    float3 farPoint  : TEXCOORD1;
};

struct PSOutput
{
    float4 color : SV_TARGET;
    float  depth : SV_DEPTH;
};

// Returns a grid color for a world-space position at the given cell scale.
// Axis lines are coloured red (X) and blue (Z).
float4 GridColor(float3 pos, float scale)
{
    float2 coord = pos.xz / scale;
    float2 d     = fwidth(coord);
    float2 a     = abs(frac(coord - 0.5) - 0.5) / max(d, 1e-4);
    float  gridLine = min(a.x, a.y);
    float  alpha    = 1.0 - saturate(gridLine);

    float4 color = float4(0.4, 0.4, 0.4, alpha * 0.75);

    // X axis (red)
    if (abs(pos.z) < d.y * scale)
        color = float4(0.9, 0.2, 0.2, alpha);
    // Z axis (blue)
    if (abs(pos.x) < d.x * scale)
        color = float4(0.2, 0.2, 0.9, alpha);

    return color;
}

PSOutput main(PSInput input)
{
    PSOutput o;

    float denom = input.farPoint.y - input.nearPoint.y;

    // Reject rays nearly parallel to the XZ plane
    [branch]
    if (abs(denom) < 1e-4)
    {
        discard;
    }

    // Ray-plane (y = 0) intersection parameter
    float t = -input.nearPoint.y / denom;

    // Reject rays pointing away from the plane
    [branch]
    if (t <= 0.0)
    {
        discard;
    }

    float3 worldPos = input.nearPoint + t * (input.farPoint - input.nearPoint);

    // Distance-based fade so the grid disappears at the horizon
    float dist = length(worldPos.xz - cameraPos.xz);
    float fade = 1.0 - saturate(dist / 200.0);

    // Overlay two grid scales: 1 m and 10 m
    float4 c1    = GridColor(worldPos, 1.0);
    float4 c2    = GridColor(worldPos, 10.0);
    float4 color = c1 + c2 * 0.6;
    color.a     *= fade;

    [branch]
    if (color.a < 0.005)
        discard;

    // Write the correct NDC depth so the grid occludes / is occluded properly
    float4 clipPos = mul(float4(worldPos, 1.0), viewProj);
    o.depth        = saturate(clipPos.z / clipPos.w);
    o.color        = color;

    return o;
}
