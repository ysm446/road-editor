#include "common.hlsli"

struct VSIn
{
    float3 pos   : POSITION;
    float4 color : COLOR;
};

struct VSOut
{
    float4 posH  : SV_POSITION;
    float4 color : COLOR;
};

VSOut main(VSIn input)
{
    VSOut o;
    o.posH  = mul(float4(input.pos, 1.0f), viewProj);
    o.color = input.color;
    return o;
}
