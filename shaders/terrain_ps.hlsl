// terrain_ps.hlsl
// Altitude-based colour with Lambert lighting.

#include "common.hlsli"

cbuffer TerrainParams : register(b1)
{
    float3 sunDir;    // normalised, pointing toward light source
    float  maxHeight;
    int    colorMode;
    float3 _padding;
};

Texture2D terrainTexture : register(t0);
SamplerState terrainSampler : register(s0);

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
    float3 albedo;
    if (colorMode == 0)
    {
        albedo = float3(0.62, 0.62, 0.64);
    }
    else if (colorMode == 3)
    {
        albedo = terrainTexture.Sample(terrainSampler, input.uv).rgb;
    }
    else if (colorMode == 2)
    {
        float slope = saturate(1.0 - abs(n.y));
        float3 colLow  = float3(0.30, 0.70, 0.32);
        float3 colMid  = float3(0.98, 0.91, 0.24);
        float3 colHigh = float3(0.96, 0.60, 0.12);
        float3 colPeak = float3(0.90, 0.22, 0.17);

        if (slope < 0.33)
            albedo = lerp(colLow, colMid, saturate(slope / 0.33));
        else if (slope < 0.66)
            albedo = lerp(colMid, colHigh, saturate((slope - 0.33) / 0.33));
        else
            albedo = lerp(colHigh, colPeak, saturate((slope - 0.66) / 0.34));
    }
    else
    {
        // Altitude [0, 1] relative to the terrain's maximum height
        float t = saturate(input.worldPos.y / max(maxHeight, 0.001));

        // Colour ramp: grass -> rock -> snow
        float3 colGrass = float3(0.30, 0.52, 0.18);
        float3 colRock  = float3(0.50, 0.42, 0.30);
        float3 colSnow  = float3(0.90, 0.90, 0.95);

        if (t < 0.5)
            albedo = lerp(colGrass, colRock,  saturate(t * 2.0));
        else
            albedo = lerp(colRock,  colSnow,  saturate((t - 0.5) * 2.0));
    }

    // Lambert
    float3 sun    = normalize(sunDir);
    float  NdotL  = saturate(dot(n, sun));
    float3 ambient = albedo * 0.25;
    float3 diffuse = albedo * NdotL;

    return float4(ambient + diffuse, 1.0);
}
