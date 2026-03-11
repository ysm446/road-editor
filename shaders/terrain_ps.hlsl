// terrain_ps.hlsl
// Altitude-based colour with Lambert lighting.

#include "common.hlsli"

cbuffer TerrainParams : register(b1)
{
    float3 sunDir;    // normalised, pointing toward light source
    float  maxHeight;
};

struct PSInput
{
    float4 clipPos  : SV_POSITION;
    float3 worldPos : TEXCOORD0;
    float3 normal   : TEXCOORD1;
    float2 uv       : TEXCOORD2;
};

float4 main(PSInput input) : SV_TARGET
{
    float3 n = normalize(input.normal);

    // Altitude [0, 1] relative to the terrain's maximum height
    float t = saturate(input.worldPos.y / max(maxHeight, 0.001));

    // Colour ramp: grass -> rock -> snow
    float3 colGrass = float3(0.30, 0.52, 0.18);
    float3 colRock  = float3(0.50, 0.42, 0.30);
    float3 colSnow  = float3(0.90, 0.90, 0.95);

    float3 albedo;
    if (t < 0.5)
        albedo = lerp(colGrass, colRock,  saturate(t * 2.0));
    else
        albedo = lerp(colRock,  colSnow,  saturate((t - 0.5) * 2.0));

    // Lambert
    float3 sun    = normalize(sunDir);
    float  NdotL  = saturate(dot(n, sun));
    float3 ambient = albedo * 0.25;
    float3 diffuse = albedo * NdotL;

    return float4(ambient + diffuse, 1.0);
}
