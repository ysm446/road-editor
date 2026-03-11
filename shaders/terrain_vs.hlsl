// terrain_vs.hlsl

#include "common.hlsli"

struct VSInput
{
    float3 position : POSITION;
    float3 normal   : NORMAL;
    float2 uv       : TEXCOORD;
};

struct VSOutput
{
    float4 clipPos  : SV_POSITION;
    float3 worldPos : TEXCOORD0;
    float3 normal   : TEXCOORD1;
    float2 uv       : TEXCOORD2;
};

VSOutput main(VSInput v)
{
    VSOutput o;
    o.worldPos = v.position;
    o.clipPos  = mul(float4(v.position, 1.0), viewProj);
    o.normal   = v.normal;
    o.uv       = v.uv;
    return o;
}
