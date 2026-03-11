struct PSIn
{
    float4 posH  : SV_POSITION;
    float4 color : COLOR;
};

float4 main(PSIn input) : SV_TARGET
{
    return input.color;
}
