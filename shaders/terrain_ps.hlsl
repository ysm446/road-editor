// terrain_ps.hlsl
// Altitude-based colour with Lambert lighting.

#include "common.hlsli"

cbuffer TerrainParams : register(b1)
{
    float3 sunDir;    // normalised, pointing toward light source
    float  minHeight;
    float  maxHeight;
    int    colorMode;
    int    lightingMode;
    float  shadowStrength;
    float2 shadowMapTexelSize;
    float  shadowBias;
    float  opacity;
};

Texture2D terrainTexture : register(t0);
SamplerState terrainSampler : register(s0);
Texture2D shadowMap : register(t1);
SamplerComparisonState shadowSampler : register(s1);

cbuffer ShadowParams : register(b2)
{
    row_major float4x4 lightViewProj;
}

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
    bool applyDesaturation = false;
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
        applyDesaturation = true;
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
        applyDesaturation = true;
        // Altitude [0, 1] across the terrain's offset-adjusted height range.
        float t = saturate((input.worldPos.y - minHeight) / max(maxHeight - minHeight, 0.001));

        // Colour ramp: grass -> rock -> snow
        float3 colGrass = float3(0.30, 0.52, 0.18);
        float3 colRock  = float3(0.50, 0.42, 0.30);
        float3 colSnow  = float3(0.90, 0.90, 0.95);

        if (t < 0.5)
            albedo = lerp(colGrass, colRock,  saturate(t * 2.0));
        else
            albedo = lerp(colRock,  colSnow,  saturate((t - 0.5) * 2.0));
    }

    if (applyDesaturation)
    {
        const float luminance = dot(albedo, float3(0.299, 0.587, 0.114));
        albedo = lerp(float3(luminance, luminance, luminance), albedo, 0.82);
    }

    float shadow = 1.0;
    if (lightingMode == 1)
    {
        const float4 lightClip = mul(float4(input.worldPos, 1.0), lightViewProj);
        const float invW = rcp(max(lightClip.w, 1e-5));
        const float3 shadowCoord = lightClip.xyz * invW;
        const float2 shadowUv = float2(shadowCoord.x * 0.5 + 0.5, shadowCoord.y * -0.5 + 0.5);
        const bool inShadowBounds =
            shadowCoord.z >= 0.0 && shadowCoord.z <= 1.0 &&
            shadowUv.x >= 0.0 && shadowUv.x <= 1.0 &&
            shadowUv.y >= 0.0 && shadowUv.y <= 1.0;
        if (inShadowBounds)
        {
            float visibility = 0.0;
            [unroll]
            for (int y = -1; y <= 1; ++y)
            {
                [unroll]
                for (int x = -1; x <= 1; ++x)
                {
                    const float2 offset = float2(x, y) * shadowMapTexelSize;
                    visibility += shadowMap.SampleCmpLevelZero(
                        shadowSampler,
                        shadowUv + offset,
                        shadowCoord.z - shadowBias);
                }
            }
            visibility /= 9.0;
            shadow = lerp(1.0 - shadowStrength, 1.0, visibility);
        }
    }

    // Half-Lambert for softer wrap lighting without specular highlights
    float3 sun    = normalize(sunDir);
    float  NdotL  = dot(n, sun);
    float  halfLambert = saturate(NdotL * 0.5 + 0.5);
    halfLambert *= halfLambert;
    float3 ambient = albedo * ((lightingMode == 1) ? 0.32 : 0.25);
    float3 diffuse = albedo * halfLambert * shadow;

    return float4(ambient + diffuse, saturate(opacity));
}
