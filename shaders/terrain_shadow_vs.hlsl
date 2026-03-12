cbuffer ShadowFrame : register(b0)
{
    row_major float4x4 lightViewProj;
};

struct VSInput
{
    float3 position : POSITION;
    float3 normal   : NORMAL;
    float2 uv       : TEXCOORD;
};

float4 main(VSInput v) : SV_POSITION
{
    return mul(float4(v.position, 1.0), lightViewProj);
}
