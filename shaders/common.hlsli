// common.hlsli — shared per-frame constant buffer

cbuffer PerFrame : register(b0)
{
    row_major float4x4 viewProj;      // view * projection
    row_major float4x4 invViewProj;   // inverse of viewProj
    float3             cameraPos;
    float              time;
    float              gridBaseScale;
    float              gridFadeDistance;
    float2             padding;
};
