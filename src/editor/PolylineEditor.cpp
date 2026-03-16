#include "PolylineEditor.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

using namespace DirectX;

namespace
{
constexpr size_t kMaxUndoStates = 128;
constexpr int kPreviewCurveSubdivisions = 12;
constexpr float kVerticalCurveDefaultLength = 50.0f;
constexpr float kBankAngleDefaultTargetSpeed = 40.0f;
constexpr float kLaneSectionDefaultWidth = 3.0f;
constexpr float kPreviewClothoidAngleRatio = 0.2f;
constexpr float kMinIntersectionSpacingMeters = 10.0f;
constexpr float kBankCurvatureStepMeters = 10.0f;

std::vector<float> BuildPolylineArcLengths(const std::vector<XMFLOAT3>& points);
XMFLOAT3 SamplePolylineAtDistance(const std::vector<XMFLOAT3>& points,
                                  const std::vector<float>& cumulativeLengths,
                                  float distance);
bool IsParametricEditModeHelper(EditorMode mode);

float Distance3(XMFLOAT3 a, XMFLOAT3 b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

XMFLOAT3 Lerp3(XMFLOAT3 a, XMFLOAT3 b, float t)
{
    return
    {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    };
}

XMFLOAT2 Lerp2(XMFLOAT2 a, XMFLOAT2 b, float t)
{
    return
    {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t
    };
}

XMFLOAT4 Lerp4(XMFLOAT4 a, XMFLOAT4 b, float t)
{
    return
    {
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
        a.w + (b.w - a.w) * t
    };
}

XMFLOAT3 HsvToRgb(float hue, float saturation, float value)
{
    const float wrappedHue = hue - floorf(hue);
    const float clampedSaturation = std::clamp(saturation, 0.0f, 1.0f);
    const float clampedValue = std::clamp(value, 0.0f, 1.0f);
    if (clampedSaturation <= 1e-6f)
        return { clampedValue, clampedValue, clampedValue };

    const float h = wrappedHue * 6.0f;
    const int sector = static_cast<int>(floorf(h));
    const float f = h - static_cast<float>(sector);
    const float p = clampedValue * (1.0f - clampedSaturation);
    const float q = clampedValue * (1.0f - clampedSaturation * f);
    const float t = clampedValue * (1.0f - clampedSaturation * (1.0f - f));

    switch (sector % 6)
    {
    case 0: return { clampedValue, t, p };
    case 1: return { q, clampedValue, p };
    case 2: return { p, clampedValue, t };
    case 3: return { p, q, clampedValue };
    case 4: return { t, p, clampedValue };
    default: return { clampedValue, p, q };
    }
}

float DistanceXZ3(XMFLOAT3 a, XMFLOAT3 b)
{
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return sqrtf(dx * dx + dz * dz);
}

float Distance2(XMFLOAT2 a, XMFLOAT2 b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return sqrtf(dx * dx + dy * dy);
}

float Dot3(XMFLOAT3 a, XMFLOAT3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float Dot2(XMFLOAT2 a, XMFLOAT2 b)
{
    return a.x * b.x + a.y * b.y;
}

float DotXZ(XMFLOAT3 a, XMFLOAT3 b)
{
    return a.x * b.x + a.z * b.z;
}

XMFLOAT3 Add3(XMFLOAT3 a, XMFLOAT3 b)
{
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

XMFLOAT3 Sub3(XMFLOAT3 a, XMFLOAT3 b)
{
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

XMFLOAT3 Scale3(XMFLOAT3 v, float s)
{
    return { v.x * s, v.y * s, v.z * s };
}

XMFLOAT3 Cross3(XMFLOAT3 a, XMFLOAT3 b)
{
    return
    {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

XMFLOAT2 Add2(XMFLOAT2 a, XMFLOAT2 b)
{
    return { a.x + b.x, a.y + b.y };
}

XMFLOAT2 Sub2(XMFLOAT2 a, XMFLOAT2 b)
{
    return { a.x - b.x, a.y - b.y };
}

XMFLOAT2 Scale2(XMFLOAT2 v, float s)
{
    return { v.x * s, v.y * s };
}

XMFLOAT2 Rotate2(XMFLOAT2 v, float angle)
{
    const float c = cosf(angle);
    const float s = sinf(angle);
    return
    {
        v.x * c - v.y * s,
        v.x * s + v.y * c
    };
}

bool IsFiniteFloat(float value)
{
    return std::isfinite(value);
}

bool IsFinitePoint3(XMFLOAT3 p)
{
    return IsFiniteFloat(p.x) && IsFiniteFloat(p.y) && IsFiniteFloat(p.z);
}

bool HasAnyFinitePoint(const std::vector<XMFLOAT3>& points)
{
    for (const XMFLOAT3& point : points)
    {
        if (IsFinitePoint3(point))
            return true;
    }
    return false;
}

XMFLOAT3 MakeInvalidPoint3()
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    return { nan, nan, nan };
}

float Length3(XMFLOAT3 v)
{
    return sqrtf(Dot3(v, v));
}

float Length2(XMFLOAT2 v)
{
    return sqrtf(Dot2(v, v));
}

XMFLOAT3 Normalize3(XMFLOAT3 v)
{
    const float len = Length3(v);
    if (len <= 1e-6f)
        return { 0.0f, 0.0f, 0.0f };
    return Scale3(v, 1.0f / len);
}

XMFLOAT2 Normalize2(XMFLOAT2 v)
{
    const float len = Length2(v);
    if (len <= 1e-6f)
        return { 0.0f, 0.0f };
    return Scale2(v, 1.0f / len);
}

float ComputeCurvatureRadiusXZ(XMFLOAT3 p0, XMFLOAT3 p1, XMFLOAT3 p2)
{
    constexpr float eps = 1e-6f;
    p0.y = 0.0f;
    p1.y = 0.0f;
    p2.y = 0.0f;

    const float chordLength = Distance3(p2, p0);
    if (chordLength < eps)
        return 0.0f;

    const XMFLOAT3 chordDir = Normalize3(Sub3(p2, p0));
    const XMFLOAT3 projected = Add3(p0, Scale3(chordDir, Dot3(Sub3(p1, p0), chordDir)));
    const float h = Distance3(projected, p1);
    if (h < 1e-4f)
        return 0.0f;

    const float radius = (chordLength * chordLength * 0.25f + h * h) / (2.0f * h);
    return radius > eps ? radius : 0.0f;
}

XMFLOAT3 ComputeHalfVectorXZ(XMFLOAT3 p0, XMFLOAT3 p1, XMFLOAT3 p2)
{
    constexpr float eps = 1e-6f;
    p0.y = 0.0f;
    p1.y = 0.0f;
    p2.y = 0.0f;

    const XMFLOAT3 e0 = Sub3(p1, p0);
    const XMFLOAT3 e1 = Sub3(p1, p2);
    const float len0 = Length3(e0);
    const float len1 = Length3(e1);
    if (len0 < eps || len1 < eps)
        return { 0.0f, 0.0f, 0.0f };

    const XMFLOAT3 v0 = Scale3(e0, 1.0f / len0);
    const XMFLOAT3 v1 = Scale3(e1, 1.0f / len1);
    const XMFLOAT3 sum = Add3(v0, v1);
    if (Length3(sum) < eps)
        return { 0.0f, 0.0f, 0.0f };

    return Normalize3(sum);
}

float ComputeBankAngleRadians(float radius, float targetSpeedKmh, float friction)
{
    constexpr float eps = 1e-6f;
    constexpr float g = 9.8f;
    if (radius <= eps)
        return 0.0f;

    const float speedMps = targetSpeedKmh * (1000.0f / 3600.0f);
    const float a = speedMps * speedMps / radius;
    float theta = atan2f(a - g * friction, g + a * friction);
    if (theta < 0.0f)
        theta = 0.0f;
    return theta;
}

struct EvaluatedLaneSection
{
    float offsetCenter = 0.0f;
    float widthLeft2 = 0.0f;
    float widthLeft1 = 0.0f;
    float widthCenter = 0.0f;
    float widthRight1 = 0.0f;
    float widthRight2 = 0.0f;
};

float EvaluateInterpolatedBankAngleRadians(const Road& road,
                                           const std::vector<XMFLOAT3>& curve,
                                           const std::vector<float>& arcLengths,
                                           float distance)
{
    constexpr float eps = 1e-5f;
    if (curve.size() < 2 || arcLengths.empty())
        return 0.0f;

    const float totalLength = arcLengths.back();
    const XMFLOAT3 autoP0 = SamplePolylineAtDistance(
        curve,
        arcLengths,
        (std::max)(0.0f, distance - kBankCurvatureStepMeters));
    const XMFLOAT3 autoP1 = SamplePolylineAtDistance(curve, arcLengths, distance);
    const XMFLOAT3 autoP2 = SamplePolylineAtDistance(
        curve,
        arcLengths,
        (std::min)(totalLength, distance + kBankCurvatureStepMeters));
    const float autoRadius = ComputeCurvatureRadiusXZ(autoP0, autoP1, autoP2);
    struct ResolvedBankPoint
    {
        float distance = 0.0f;
        float value = 0.0f;
    };
    struct EvaluatedBankPoint
    {
        float distance = 0.0f;
        float autoAngleRadians = 0.0f;
        bool overrideBank = false;
        float overrideAngleRadians = 0.0f;
    };

    auto evaluateLayerAt = [eps](float targetDistance, const std::vector<ResolvedBankPoint>& points, float fallbackValue) -> float
    {
        if (points.empty())
            return fallbackValue;
        if (targetDistance <= points.front().distance + eps)
            return points.front().value;
        if (targetDistance >= points.back().distance - eps)
            return points.back().value;

        for (size_t i = 1; i < points.size(); ++i)
        {
            if (targetDistance > points[i].distance)
                continue;

            const float span = points[i].distance - points[i - 1].distance;
            if (span <= eps)
                return points[i].value;
            const float t = (targetDistance - points[i - 1].distance) / span;
            return points[i - 1].value + (points[i].value - points[i - 1].value) * t;
        }

        return points.back().value;
    };

    std::vector<ResolvedBankPoint> speedPoints;
    std::vector<EvaluatedBankPoint> evaluatedPoints;
    speedPoints.reserve(road.bankAngle.size());
    evaluatedPoints.reserve(road.bankAngle.size());
    for (const BankAnglePoint& point : road.bankAngle)
    {
        const float pointDistance = std::clamp(point.uCoord, 0.0f, 1.0f) * totalLength;
        speedPoints.push_back({ pointDistance, (std::max)(0.0f, point.targetSpeed) });
        evaluatedPoints.push_back({
            pointDistance,
            0.0f,
            point.overrideBank,
            XMConvertToRadians(std::clamp(point.bankAngle, 0.0f, 90.0f))
        });
    }

    std::sort(
        speedPoints.begin(),
        speedPoints.end(),
        [](const ResolvedBankPoint& a, const ResolvedBankPoint& b)
        {
            return a.distance < b.distance;
        });
    std::sort(
        evaluatedPoints.begin(),
        evaluatedPoints.end(),
        [](const EvaluatedBankPoint& a, const EvaluatedBankPoint& b)
        {
            return a.distance < b.distance;
        });

    const float interpolatedSpeed = evaluateLayerAt(
        distance,
        speedPoints,
        (std::max)(0.0f, road.defaultTargetSpeed));
    const float autoAngleRadians = ComputeBankAngleRadians(
        autoRadius,
        interpolatedSpeed,
        (std::max)(0.0f, road.defaultFriction));

    if (evaluatedPoints.empty())
        return autoAngleRadians;

    for (EvaluatedBankPoint& point : evaluatedPoints)
    {
        const XMFLOAT3 p0 = SamplePolylineAtDistance(
            curve,
            arcLengths,
            (std::max)(0.0f, point.distance - kBankCurvatureStepMeters));
        const XMFLOAT3 p1 = SamplePolylineAtDistance(curve, arcLengths, point.distance);
        const XMFLOAT3 p2 = SamplePolylineAtDistance(
            curve,
            arcLengths,
            (std::min)(totalLength, point.distance + kBankCurvatureStepMeters));
        const float radius = ComputeCurvatureRadiusXZ(p0, p1, p2);
        const float pointSpeed = evaluateLayerAt(
            point.distance,
            speedPoints,
            (std::max)(0.0f, road.defaultTargetSpeed));
        point.autoAngleRadians = ComputeBankAngleRadians(
            radius,
            pointSpeed,
            (std::max)(0.0f, road.defaultFriction));
    }

    if (evaluatedPoints.front().overrideBank &&
        distance <= evaluatedPoints.front().distance + eps)
    {
        return evaluatedPoints.front().overrideAngleRadians;
    }
    if (evaluatedPoints.back().overrideBank &&
        distance >= evaluatedPoints.back().distance - eps)
    {
        return evaluatedPoints.back().overrideAngleRadians;
    }

    for (size_t i = 1; i < evaluatedPoints.size(); ++i)
    {
        const EvaluatedBankPoint& prev = evaluatedPoints[i - 1];
        const EvaluatedBankPoint& next = evaluatedPoints[i];
        if (distance < prev.distance - eps || distance > next.distance + eps)
            continue;
        if (!prev.overrideBank && !next.overrideBank)
            break;

        const float startAngle = prev.overrideBank ? prev.overrideAngleRadians : prev.autoAngleRadians;
        const float endAngle = next.overrideBank ? next.overrideAngleRadians : next.autoAngleRadians;
        const float span = next.distance - prev.distance;
        if (span <= eps)
            return endAngle;
        const float t = std::clamp((distance - prev.distance) / span, 0.0f, 1.0f);
        return startAngle + (endAngle - startAngle) * t;
    }

    return autoAngleRadians;
}

BankFrameSample EvaluateBankFrameAtDistance(const Road& road,
                                            const std::vector<XMFLOAT3>& curve,
                                            const std::vector<float>& arcLengths,
                                            float distance)
{
    const float totalLength = arcLengths.empty() ? 0.0f : arcLengths.back();
    const float clampedDistance = std::clamp(distance, 0.0f, totalLength);
    const float prevDistance = (std::max)(0.0f, clampedDistance - kBankCurvatureStepMeters);
    const float nextDistance = (std::min)(totalLength, clampedDistance + kBankCurvatureStepMeters);
    const XMFLOAT3 p0 = SamplePolylineAtDistance(curve, arcLengths, prevDistance);
    const XMFLOAT3 p1 = SamplePolylineAtDistance(curve, arcLengths, clampedDistance);
    const XMFLOAT3 p2 = SamplePolylineAtDistance(curve, arcLengths, nextDistance);

    XMFLOAT3 tangent = Normalize3(Sub3(p2, p0));
    if (Length3(tangent) <= 1e-5f)
        tangent = Normalize3(Sub3(p2, p1));
    if (Length3(tangent) <= 1e-5f)
        tangent = Normalize3(Sub3(p1, p0));
    if (Length3(tangent) <= 1e-5f)
        tangent = { 0.0f, 0.0f, 1.0f };

    const XMFLOAT3 halfVector = ComputeHalfVectorXZ(p0, p1, p2);
    XMFLOAT3 baseLeft = Normalize3(Cross3({ 0.0f, 1.0f, 0.0f }, tangent));
    if (Length3(baseLeft) <= 1e-5f)
        baseLeft = { 1.0f, 0.0f, 0.0f };
    XMFLOAT3 baseUp = Normalize3(Cross3(tangent, baseLeft));
    if (Length3(baseUp) <= 1e-5f)
        baseUp = { 0.0f, 1.0f, 0.0f };

    const float localHalfX = DotXZ(halfVector, baseLeft);
    const float sign = localHalfX < 0.0f ? -1.0f : 1.0f;
    const float angleRadians = EvaluateInterpolatedBankAngleRadians(road, curve, arcLengths, clampedDistance);
    XMFLOAT3 bankLeft = Normalize3(Add3(
        Scale3(baseLeft, cosf(angleRadians)),
        Scale3(baseUp, sign * sinf(angleRadians))));
    XMFLOAT3 bankUp = Normalize3(Cross3(tangent, bankLeft));
    if (Length3(bankUp) <= 1e-5f)
        bankUp = baseUp;

    return { p1, bankLeft, bankUp, angleRadians };
}

EvaluatedLaneSection EvaluateLaneSectionAtDistance(const Road& road,
                                                   const std::vector<float>& arcLengths,
                                                   float distance)
{
    auto fromPoint = [](const LaneSectionPoint& point) -> EvaluatedLaneSection
    {
        return {
            point.offsetCenter,
            point.useLaneLeft2 ? (std::max)(0.0f, point.widthLaneLeft2) : 0.0f,
            point.useLaneLeft1 ? (std::max)(0.0f, point.widthLaneLeft1) : 0.0f,
            point.useLaneCenter ? (std::max)(0.0f, point.widthLaneCenter) : 0.0f,
            point.useLaneRight1 ? (std::max)(0.0f, point.widthLaneRight1) : 0.0f,
            point.useLaneRight2 ? (std::max)(0.0f, point.widthLaneRight2) : 0.0f
        };
    };

    auto lerpSection = [](const EvaluatedLaneSection& a, const EvaluatedLaneSection& b, float t) -> EvaluatedLaneSection
    {
        return {
            a.offsetCenter + (b.offsetCenter - a.offsetCenter) * t,
            a.widthLeft2 + (b.widthLeft2 - a.widthLeft2) * t,
            a.widthLeft1 + (b.widthLeft1 - a.widthLeft1) * t,
            a.widthCenter + (b.widthCenter - a.widthCenter) * t,
            a.widthRight1 + (b.widthRight1 - a.widthRight1) * t,
            a.widthRight2 + (b.widthRight2 - a.widthRight2) * t
        };
    };

    if (road.laneSections.empty() || arcLengths.empty())
    {
        return {
            0.0f,
            0.0f,
            (std::max)(0.0f, road.defaultWidthLaneLeft1),
            (std::max)(0.0f, road.defaultWidthLaneCenter),
            (std::max)(0.0f, road.defaultWidthLaneRight1),
            0.0f
        };
    }

    const float totalLength = arcLengths.back();
    std::vector<std::pair<float, EvaluatedLaneSection>> sections;
    sections.reserve(road.laneSections.size());
    for (const LaneSectionPoint& point : road.laneSections)
        sections.push_back({ std::clamp(point.uCoord, 0.0f, 1.0f) * totalLength, fromPoint(point) });

    std::sort(
        sections.begin(),
        sections.end(),
        [](const auto& a, const auto& b)
        {
            return a.first < b.first;
        });

    if (distance <= sections.front().first)
        return sections.front().second;
    if (distance >= sections.back().first)
        return sections.back().second;

    for (size_t i = 1; i < sections.size(); ++i)
    {
        if (distance > sections[i].first)
            continue;
        const float span = sections[i].first - sections[i - 1].first;
        if (span <= 1e-5f)
            return sections[i].second;
        const float t = (distance - sections[i - 1].first) / span;
        return lerpSection(sections[i - 1].second, sections[i].second, t);
    }

    return sections.back().second;
}

bool NearlyEqual3(XMFLOAT3 a, XMFLOAT3 b, float epsilon = 1e-3f)
{
    return Distance3(a, b) <= epsilon;
}

void AppendUniquePoint(std::vector<XMFLOAT3>& samples, XMFLOAT3 point)
{
    if (!samples.empty() && NearlyEqual3(samples.back(), point))
        return;
    samples.push_back(point);
}

XMFLOAT4 PreviewGradeColor(float gradePercent, float redThresholdPercent)
{
    const float safeThreshold = (std::max)(0.1f, redThresholdPercent);
    const float ratio = gradePercent / safeThreshold;
    if (ratio > 1.0f)
    {
        const XMFLOAT3 warningColor = HsvToRgb(0.9f, 1.0f, 0.8f);
        return { warningColor.x, warningColor.y, warningColor.z, 1.0f };
    }

    const float clampedRatio = std::clamp(ratio, 0.0f, 1.0f);
    const float hue = 0.5f - clampedRatio * 0.45f;
    const XMFLOAT3 color = HsvToRgb(hue, clampedRatio, 0.8f);
    return { color.x, color.y, color.z, 1.0f };
}

XMFLOAT3 QuadraticBezier(XMFLOAT3 p0, XMFLOAT3 p1, XMFLOAT3 p2, float t)
{
    const float u = 1.0f - t;
    const float uu = u * u;
    const float tt = t * t;
    return
    {
        uu * p0.x + 2.0f * u * t * p1.x + tt * p2.x,
        uu * p0.y + 2.0f * u * t * p1.y + tt * p2.y,
        uu * p0.z + 2.0f * u * t * p1.z + tt * p2.z
    };
}

struct PreviewPlaneFrame
{
    XMFLOAT3 origin = { 0.0f, 0.0f, 0.0f };
    XMFLOAT3 xAxis = { 1.0f, 0.0f, 0.0f };
    XMFLOAT3 yAxis = { 0.0f, 1.0f, 0.0f };
    XMFLOAT3 zAxis = { 0.0f, 0.0f, 1.0f };
};

struct VerticalGuidePoint
{
    float x = 0.0f;
    float y = 0.0f;
    float vcl = 0.0f;
};

std::vector<PreviewCurvePoint> BuildRoadVerticalPreviewCurveDetailed(const Road& road, float sampleInterval);
std::vector<PreviewCurvePoint> BuildRoadVerticalPreviewCurveDetailed(
    const Road& road,
    const std::vector<XMFLOAT3>& baseCurve,
    float sampleInterval);
std::vector<float> BuildPolylineArcLengths(const std::vector<XMFLOAT3>& points);
XMFLOAT3 SamplePolylineAtDistance(const std::vector<XMFLOAT3>& points,
                                  const std::vector<float>& cumulativeLengths,
                                  float distance);

struct VerticalPreviewSegment
{
    VerticalGuidePoint p0;
    VerticalGuidePoint p1;
    VerticalGuidePoint p2;
    float curveStartX = 0.0f;
    float curveStartY = 0.0f;
    float curveEndX = 0.0f;
    float curveEndY = 0.0f;
    float curveLength = 0.0f;
    float slopeIn = 0.0f;
    float slopeOut = 0.0f;
    bool validCurve = false;
    PreviewCurveSegmentKind curveKind = PreviewCurveSegmentKind::Other;
};

void AppendUniquePreviewPoint(
    std::vector<PreviewCurvePoint>& samples,
    XMFLOAT3 point,
    PreviewCurveSegmentKind kind)
{
    if (!samples.empty() && NearlyEqual3(samples.back().pos, point))
    {
        samples.back().kind = kind;
        return;
    }
    samples.push_back({ point, kind });
}

bool BuildPreviewPlaneFrame(XMFLOAT3 p0, XMFLOAT3 p1, XMFLOAT3 p2, PreviewPlaneFrame& outFrame)
{
    const XMFLOAT3 seg0 = Sub3(p1, p0);
    const XMFLOAT3 seg1 = Sub3(p2, p1);
    const float segLen0 = Length3(seg0);
    const float segLen1 = Length3(seg1);
    if (segLen0 <= 1e-4f || segLen1 <= 1e-4f)
        return false;

    const XMFLOAT3 xAxis = Scale3(seg0, 1.0f / segLen0);
    const XMFLOAT3 zAxis = Normalize3(Cross3(xAxis, seg1));
    if (Length3(zAxis) <= 1e-4f)
        return false;

    const XMFLOAT3 yAxis = Normalize3(Cross3(zAxis, xAxis));
    if (Length3(yAxis) <= 1e-4f)
        return false;

    outFrame.origin = p0;
    outFrame.xAxis = xAxis;
    outFrame.yAxis = yAxis;
    outFrame.zAxis = zAxis;
    return true;
}

XMFLOAT3 WorldToPlane(const PreviewPlaneFrame& frame, XMFLOAT3 point)
{
    const XMFLOAT3 relative = Sub3(point, frame.origin);
    return
    {
        Dot3(relative, frame.xAxis),
        Dot3(relative, frame.yAxis),
        Dot3(relative, frame.zAxis)
    };
}

XMFLOAT3 PlaneToWorld(const PreviewPlaneFrame& frame, XMFLOAT3 point)
{
    return Add3(
        frame.origin,
        Add3(
            Add3(Scale3(frame.xAxis, point.x), Scale3(frame.yAxis, point.y)),
            Scale3(frame.zAxis, point.z)));
}

XMFLOAT2 CrossPointXY(XMFLOAT2 p0, XMFLOAT2 p1, XMFLOAT2 p2, XMFLOAT2 p3)
{
    const float a1 = p0.y - p1.y;
    const float b1 = p1.x - p0.x;
    const float c1 = (p1.x - p0.x) * p0.y - (p1.y - p0.y) * p0.x;

    const float a2 = p2.y - p3.y;
    const float b2 = p3.x - p2.x;
    const float c2 = (p3.x - p2.x) * p2.y - (p3.y - p2.y) * p2.x;

    const float denom = a1 * b2 - a2 * b1;
    if (fabsf(denom) < 1e-6f)
    {
        const XMFLOAT2 dir = Sub2(p1, p0);
        const float dirLen2 = Dot2(dir, dir);
        if (dirLen2 < 1e-6f)
            return p2;
        const float t = Dot2(Sub2(p2, p0), dir) / dirLen2;
        return Add2(p0, Scale2(dir, t));
    }

    return
    {
        (c1 * b2 - c2 * b1) / denom,
        (a1 * c2 - a2 * c1) / denom
    };
}

float ClothoidX(float tau, float A)
{
    return A / sqrtf(2.0f) * 2.0f * sqrtf((std::max)(tau, 0.0f)) *
        (1.0f - (0.1f * powf(tau, 2.0f) + (1.0f / 216.0f) * powf(tau, 4.0f) -
                  (1.0f / 9360.0f) * powf(tau, 6.0f)));
}

float ClothoidY(float tau, float A)
{
    return A / sqrtf(2.0f) * (2.0f / 3.0f) * sqrtf((std::max)(tau, 0.0f)) * tau *
        (1.0f - (powf(tau, 2.0f) / 14.0f) + (powf(tau, 4.0f) / 440.0f) -
         (powf(tau, 6.0f) / 25200.0f));
}

void AppendQuadraticBezierSamples(
    std::vector<XMFLOAT3>& samples,
    XMFLOAT3 p0, XMFLOAT3 p1, XMFLOAT3 p2,
    int subdivisions)
{
    AppendUniquePoint(samples, p0);
    for (int step = 1; step <= subdivisions; ++step)
    {
        const float t = static_cast<float>(step) / static_cast<float>(subdivisions);
        AppendUniquePoint(samples, QuadraticBezier(p0, p1, p2, t));
    }
}

void AppendBezierFallbackGuide(
    std::vector<XMFLOAT3>& samples,
    XMFLOAT3 p0, XMFLOAT3 p1, XMFLOAT3 p2,
    float sampleInterval)
{
    const float approxLength = Distance3(p0, p1) + Distance3(p1, p2);
    const int subdivisions = (std::max)(
        kPreviewCurveSubdivisions,
        static_cast<int>(ceilf(approxLength / sampleInterval)));
    AppendQuadraticBezierSamples(samples, p0, p1, p2, subdivisions);
}

void AppendBezierFallbackGuideDetailed(
    std::vector<PreviewCurvePoint>& samples,
    XMFLOAT3 p0, XMFLOAT3 p1, XMFLOAT3 p2,
    float sampleInterval)
{
    const float approxLength = Distance3(p0, p1) + Distance3(p1, p2);
    const int subdivisions = (std::max)(
        kPreviewCurveSubdivisions,
        static_cast<int>(ceilf(approxLength / sampleInterval)));
    AppendUniquePreviewPoint(samples, p0, PreviewCurveSegmentKind::Other);
    for (int step = 1; step <= subdivisions; ++step)
    {
        const float t = static_cast<float>(step) / static_cast<float>(subdivisions);
        AppendUniquePreviewPoint(
            samples,
            QuadraticBezier(p0, p1, p2, t),
            PreviewCurveSegmentKind::Other);
    }
}

void AppendClothoidGuideCurve(
    std::vector<XMFLOAT3>& samples,
    XMFLOAT3 p0, XMFLOAT3 p1, XMFLOAT3 p2,
    float sampleInterval)
{
    const XMFLOAT3 seg0 = Sub3(p1, p0);
    const XMFLOAT3 seg1 = Sub3(p2, p1);
    const float segLen0 = Length3(seg0);
    const float segLen1 = Length3(seg1);
    if (segLen0 <= 1e-4f || segLen1 <= 1e-4f)
    {
        AppendBezierFallbackGuide(samples, p0, p1, p2, sampleInterval);
        return;
    }

    const XMFLOAT3 v0World = Scale3(seg0, 1.0f / segLen0);
    const XMFLOAT3 v1World = Scale3(seg1, 1.0f / segLen1);
    const float worldDot = std::clamp(Dot3(v0World, v1World), -1.0f, 1.0f);
    if (Length3(Cross3(v0World, v1World)) < 0.001f &&
        fabsf(1.0f - fabsf(worldDot)) < 0.001f)
    {
        AppendBezierFallbackGuide(samples, p0, p1, p2, sampleInterval);
        return;
    }

    PreviewPlaneFrame frame;
    if (!BuildPreviewPlaneFrame(p0, p1, p2, frame))
    {
        AppendBezierFallbackGuide(samples, p0, p1, p2, sampleInterval);
        return;
    }

    const XMFLOAT3 localP1_3 = WorldToPlane(frame, p1);
    const XMFLOAT3 localP2_3 = WorldToPlane(frame, p2);
    const XMFLOAT2 localP1 = { localP1_3.x, localP1_3.y };
    const XMFLOAT2 localP2 = { localP2_3.x, localP2_3.y };
    const float localSegLen0 = Length2(localP1);
    const float localSegLen1 = Distance2(localP2, localP1);
    if (localSegLen0 <= 1e-4f || localSegLen1 <= 1e-4f)
    {
        AppendBezierFallbackGuide(samples, p0, p1, p2, sampleInterval);
        return;
    }

    const XMFLOAT2 v0 = Normalize2(localP1);
    const XMFLOAT2 v1 = Normalize2(Sub2(localP2, localP1));
    const float I = acosf(std::clamp(Dot2(v0, v1), -1.0f, 1.0f));
    if (I < 0.001f)
    {
        AppendBezierFallbackGuide(samples, p0, p1, p2, sampleInterval);
        return;
    }

    const float tau1 = I * kPreviewClothoidAngleRatio * 0.5f;
    const float tau2 = tau1;
    const float theta = I - (tau1 + tau2);
    const float A1 = sqrtf(tau1 * 2.0f);
    const float A2 = sqrtf(tau2 * 2.0f);
    const float L1 = A1 * A1;
    const float L2 = A2 * A2;

    const XMFLOAT2 clothoidEnd0 = { ClothoidX(tau1, A1), ClothoidY(tau1, A1) };
    const XMFLOAT2 clothoidEnd1 = { ClothoidX(tau2, A2), ClothoidY(tau2, A2) };
    const XMFLOAT2 vecTau1 = { cosf(tau1), sinf(tau1) };
    const XMFLOAT2 center = Add2(clothoidEnd0, { -vecTau1.y, vecTau1.x });
    const XMFLOAT2 diff = Sub2(clothoidEnd0, center);
    const float startRot = atan2f(diff.y, diff.x);
    const float endRot = startRot + theta;
    const XMFLOAT2 arcEnd = Add2({ cosf(endRot), sinf(endRot) }, center);

    const XMFLOAT2 mirroredClothoidStart = Add2(
        arcEnd,
        Rotate2({ clothoidEnd1.x, -clothoidEnd1.y }, I));

    const XMFLOAT2 localGuideP0 = { 0.0f, 0.0f };
    const XMFLOAT2 localGuideP2 = mirroredClothoidStart;
    const XMFLOAT2 localGuideP1 = CrossPointXY(
        { 0.0f, 0.0f },
        { 1.0f, 0.0f },
        mirroredClothoidStart,
        Add2(mirroredClothoidStart, Sub2(localP1, localP2)));

    const float localLen0 = (std::max)(Distance2(localGuideP1, localGuideP0), 1e-4f);
    const float localLen1 = (std::max)(Distance2(localGuideP1, localGuideP2), 1e-4f);
    const float localRatio = localLen0 / localLen1;
    const float ratio = localSegLen0 / localSegLen1;

    float scale = 1.0f;
    float offset = 0.0f;
    if (ratio > localRatio)
    {
        scale = localSegLen1 / localLen1;
        offset = localSegLen0 - localLen0 * scale;
    }
    else
    {
        scale = localSegLen0 / localLen0;
    }

    const float localSampleLength =
        (std::max)(sampleInterval / (std::max)(scale, 1e-4f), 1e-4f);
    auto fitPoint = [scale, offset, ratio, localRatio](XMFLOAT2 point) -> XMFLOAT2
    {
        XMFLOAT2 result = Scale2(point, scale);
        if (ratio > localRatio)
            result.x += offset;
        return result;
    };
    auto appendLocalPoint = [&samples, &frame, &fitPoint](XMFLOAT2 point)
    {
        const XMFLOAT2 fitted = fitPoint(point);
        AppendUniquePoint(samples, PlaneToWorld(frame, { fitted.x, fitted.y, 0.0f }));
    };

    AppendUniquePoint(samples, p0);

    const int clothoidSampleCount0 =
        (std::max)(static_cast<int>(ceilf(L1 / localSampleLength)), 2);
    for (int i = 0; i < clothoidSampleCount0; ++i)
    {
        const float t = tau1 * static_cast<float>(i) /
            static_cast<float>((std::max)(clothoidSampleCount0 - 1, 1));
        appendLocalPoint({ ClothoidX(t, A1), ClothoidY(t, A1) });
    }

    const float arcLength = theta;
    const int arcSampleCount =
        (std::max)(static_cast<int>(ceilf(arcLength / localSampleLength)), 2);
    for (int i = 1; i <= arcSampleCount; ++i)
    {
        const float angle =
            startRot + theta * static_cast<float>(i) / static_cast<float>(arcSampleCount);
        appendLocalPoint(Add2({ cosf(angle), sinf(angle) }, center));
    }

    const int clothoidSampleCount1 =
        (std::max)(static_cast<int>(ceilf(L2 / localSampleLength)), 2);
    std::vector<XMFLOAT2> secondClothoid;
    secondClothoid.reserve(clothoidSampleCount1);
    for (int i = 0; i < clothoidSampleCount1; ++i)
    {
        const float t = tau2 * static_cast<float>(i) /
            static_cast<float>((std::max)(clothoidSampleCount1 - 1, 1));
        const XMFLOAT2 sample = { ClothoidX(t, A2), ClothoidY(t, A2) };
        secondClothoid.push_back(Add2(
            arcEnd,
            Rotate2({ clothoidEnd1.x - sample.x, sample.y - clothoidEnd1.y }, I)));
    }
    std::reverse(secondClothoid.begin(), secondClothoid.end());
    for (const XMFLOAT2& point : secondClothoid)
        appendLocalPoint(point);

    AppendUniquePoint(samples, p2);
}

void AppendClothoidGuideCurveDetailed(
    std::vector<PreviewCurvePoint>& samples,
    XMFLOAT3 p0, XMFLOAT3 p1, XMFLOAT3 p2,
    float sampleInterval)
{
    const XMFLOAT3 seg0 = Sub3(p1, p0);
    const XMFLOAT3 seg1 = Sub3(p2, p1);
    const float segLen0 = Length3(seg0);
    const float segLen1 = Length3(seg1);
    if (segLen0 <= 1e-4f || segLen1 <= 1e-4f)
    {
        AppendBezierFallbackGuideDetailed(samples, p0, p1, p2, sampleInterval);
        return;
    }

    const XMFLOAT3 v0World = Scale3(seg0, 1.0f / segLen0);
    const XMFLOAT3 v1World = Scale3(seg1, 1.0f / segLen1);
    const float worldDot = std::clamp(Dot3(v0World, v1World), -1.0f, 1.0f);
    if (Length3(Cross3(v0World, v1World)) < 0.001f &&
        fabsf(1.0f - fabsf(worldDot)) < 0.001f)
    {
        AppendBezierFallbackGuideDetailed(samples, p0, p1, p2, sampleInterval);
        return;
    }

    PreviewPlaneFrame frame;
    if (!BuildPreviewPlaneFrame(p0, p1, p2, frame))
    {
        AppendBezierFallbackGuideDetailed(samples, p0, p1, p2, sampleInterval);
        return;
    }

    const XMFLOAT3 localP1_3 = WorldToPlane(frame, p1);
    const XMFLOAT3 localP2_3 = WorldToPlane(frame, p2);
    const XMFLOAT2 localP1 = { localP1_3.x, localP1_3.y };
    const XMFLOAT2 localP2 = { localP2_3.x, localP2_3.y };
    const float localSegLen0 = Length2(localP1);
    const float localSegLen1 = Distance2(localP2, localP1);
    if (localSegLen0 <= 1e-4f || localSegLen1 <= 1e-4f)
    {
        AppendBezierFallbackGuideDetailed(samples, p0, p1, p2, sampleInterval);
        return;
    }

    const XMFLOAT2 v0 = Normalize2(localP1);
    const XMFLOAT2 v1 = Normalize2(Sub2(localP2, localP1));
    const float I = acosf(std::clamp(Dot2(v0, v1), -1.0f, 1.0f));
    if (I < 0.001f)
    {
        AppendBezierFallbackGuideDetailed(samples, p0, p1, p2, sampleInterval);
        return;
    }

    const float tau1 = I * kPreviewClothoidAngleRatio * 0.5f;
    const float tau2 = tau1;
    const float theta = I - (tau1 + tau2);
    const float A1 = sqrtf(tau1 * 2.0f);
    const float A2 = sqrtf(tau2 * 2.0f);
    const float L1 = A1 * A1;
    const float L2 = A2 * A2;

    const XMFLOAT2 clothoidEnd0 = { ClothoidX(tau1, A1), ClothoidY(tau1, A1) };
    const XMFLOAT2 clothoidEnd1 = { ClothoidX(tau2, A2), ClothoidY(tau2, A2) };
    const XMFLOAT2 vecTau1 = { cosf(tau1), sinf(tau1) };
    const XMFLOAT2 center = Add2(clothoidEnd0, { -vecTau1.y, vecTau1.x });
    const XMFLOAT2 diff = Sub2(clothoidEnd0, center);
    const float startRot = atan2f(diff.y, diff.x);
    const float endRot = startRot + theta;
    const XMFLOAT2 arcEnd = Add2({ cosf(endRot), sinf(endRot) }, center);

    const XMFLOAT2 mirroredClothoidStart = Add2(
        arcEnd,
        Rotate2({ clothoidEnd1.x, -clothoidEnd1.y }, I));

    const XMFLOAT2 localGuideP0 = { 0.0f, 0.0f };
    const XMFLOAT2 localGuideP2 = mirroredClothoidStart;
    const XMFLOAT2 localGuideP1 = CrossPointXY(
        { 0.0f, 0.0f },
        { 1.0f, 0.0f },
        mirroredClothoidStart,
        Add2(mirroredClothoidStart, Sub2(localP1, localP2)));

    const float localLen0 = (std::max)(Distance2(localGuideP1, localGuideP0), 1e-4f);
    const float localLen1 = (std::max)(Distance2(localGuideP1, localGuideP2), 1e-4f);
    const float localRatio = localLen0 / localLen1;
    const float ratio = localSegLen0 / localSegLen1;

    float scale = 1.0f;
    float offset = 0.0f;
    if (ratio > localRatio)
    {
        scale = localSegLen1 / localLen1;
        offset = localSegLen0 - localLen0 * scale;
    }
    else
    {
        scale = localSegLen0 / localLen0;
    }

    const float localSampleLength =
        (std::max)(sampleInterval / (std::max)(scale, 1e-4f), 1e-4f);
    auto fitPoint = [scale, offset, ratio, localRatio](XMFLOAT2 point) -> XMFLOAT2
    {
        XMFLOAT2 result = Scale2(point, scale);
        if (ratio > localRatio)
            result.x += offset;
        return result;
    };
    auto appendLocalPoint = [&samples, &frame, &fitPoint](XMFLOAT2 point, PreviewCurveSegmentKind kind)
    {
        const XMFLOAT2 fitted = fitPoint(point);
        AppendUniquePreviewPoint(samples, PlaneToWorld(frame, { fitted.x, fitted.y, 0.0f }), kind);
    };

    AppendUniquePreviewPoint(samples, p0, PreviewCurveSegmentKind::Other);

    const int clothoidSampleCount0 =
        (std::max)(static_cast<int>(ceilf(L1 / localSampleLength)), 2);
    for (int i = 0; i < clothoidSampleCount0; ++i)
    {
        const float t = tau1 * static_cast<float>(i) /
            static_cast<float>((std::max)(clothoidSampleCount0 - 1, 1));
        appendLocalPoint({ ClothoidX(t, A1), ClothoidY(t, A1) }, PreviewCurveSegmentKind::Clothoid);
    }

    const float arcLength = theta;
    const int arcSampleCount =
        (std::max)(static_cast<int>(ceilf(arcLength / localSampleLength)), 2);
    for (int i = 1; i <= arcSampleCount; ++i)
    {
        const float angle =
            startRot + theta * static_cast<float>(i) / static_cast<float>(arcSampleCount);
        appendLocalPoint(Add2({ cosf(angle), sinf(angle) }, center), PreviewCurveSegmentKind::Arc);
    }

    const int clothoidSampleCount1 =
        (std::max)(static_cast<int>(ceilf(L2 / localSampleLength)), 2);
    std::vector<XMFLOAT2> secondClothoid;
    secondClothoid.reserve(clothoidSampleCount1);
    for (int i = 0; i < clothoidSampleCount1; ++i)
    {
        const float t = tau2 * static_cast<float>(i) /
            static_cast<float>((std::max)(clothoidSampleCount1 - 1, 1));
        const XMFLOAT2 sample = { ClothoidX(t, A2), ClothoidY(t, A2) };
        secondClothoid.push_back(Add2(
            arcEnd,
            Rotate2({ clothoidEnd1.x - sample.x, sample.y - clothoidEnd1.y }, I)));
    }
    std::reverse(secondClothoid.begin(), secondClothoid.end());
    for (const XMFLOAT2& point : secondClothoid)
        appendLocalPoint(point, PreviewCurveSegmentKind::Clothoid);

    AppendUniquePreviewPoint(samples, p2, PreviewCurveSegmentKind::Other);
}

std::vector<PreviewCurvePoint> BuildRoadPreviewCurveDetailed(const Road& road, float sampleInterval)
{
    std::vector<PreviewCurvePoint> samples;
    const int pointCount = static_cast<int>(road.points.size());
    if (pointCount < 2)
        return samples;

    if (pointCount == 2)
    {
        samples.push_back({ road.points[0].pos, PreviewCurveSegmentKind::Other });
        samples.push_back({ road.points[1].pos, PreviewCurveSegmentKind::Other });
        return samples;
    }

    const bool closed = road.closed && pointCount >= 3;
    const int edgeCount = closed ? pointCount : pointCount - 1;
    std::vector<XMFLOAT3> edgeMidpoints(edgeCount);

    for (int edgeIndex = 0; edgeIndex < edgeCount; ++edgeIndex)
    {
        const int startIndex = edgeIndex;
        const int endIndex = (edgeIndex + 1) % pointCount;
        float t = 0.5f;

        if (closed || (edgeIndex > 0 && edgeIndex < edgeCount - 1))
        {
            const int prevEdgeIndex = closed ? (edgeIndex - 1 + edgeCount) % edgeCount : edgeIndex - 1;
            const int nextEdgeIndex = closed ? (edgeIndex + 1) % edgeCount : edgeIndex + 1;
            const int prevStartIndex = prevEdgeIndex;
            const int prevEndIndex = (prevEdgeIndex + 1) % pointCount;
            const int nextStartIndex = nextEdgeIndex;
            const int nextEndIndex = (nextEdgeIndex + 1) % pointCount;
            const float prevLen = Distance3(
                road.points[prevStartIndex].pos,
                road.points[prevEndIndex].pos);
            const float nextLen = Distance3(
                road.points[nextStartIndex].pos,
                road.points[nextEndIndex].pos);
            const float sum = prevLen + nextLen;
            if (sum > 1e-5f)
                t = prevLen / sum;
        }

        edgeMidpoints[edgeIndex] = Lerp3(
            road.points[startIndex].pos,
            road.points[endIndex].pos,
            t);
    }

    if (closed)
    {
        AppendUniquePreviewPoint(samples, edgeMidpoints.back(), PreviewCurveSegmentKind::Other);
        for (int pointIndex = 0; pointIndex < pointCount; ++pointIndex)
        {
            AppendClothoidGuideCurveDetailed(
                samples,
                edgeMidpoints[(pointIndex - 1 + edgeCount) % edgeCount],
                road.points[pointIndex].pos,
                edgeMidpoints[pointIndex],
                sampleInterval);
        }
        if (!samples.empty())
            samples.push_back(samples.front());
        return samples;
    }

    AppendUniquePreviewPoint(samples, road.points.front().pos, PreviewCurveSegmentKind::Other);
    for (int pointIndex = 1; pointIndex < pointCount - 1; ++pointIndex)
    {
        AppendClothoidGuideCurveDetailed(
            samples,
            edgeMidpoints[pointIndex - 1],
            road.points[pointIndex].pos,
            edgeMidpoints[pointIndex],
            sampleInterval);
    }
    AppendUniquePreviewPoint(samples, road.points.back().pos, PreviewCurveSegmentKind::Other);
    return samples;
}

std::vector<XMFLOAT3> BuildRoadPreviewCurve(const Road& road, float sampleInterval)
{
    const std::vector<PreviewCurvePoint> detailed = BuildRoadPreviewCurveDetailed(road, sampleInterval);
    std::vector<XMFLOAT3> samples;
    samples.reserve(detailed.size());
    for (const PreviewCurvePoint& point : detailed)
        samples.push_back(point.pos);
    return samples;
}

std::vector<XMFLOAT3> ExtractPreviewCurvePositions(const std::vector<PreviewCurvePoint>& detailed)
{
    std::vector<XMFLOAT3> samples;
    samples.reserve(detailed.size());
    for (const PreviewCurvePoint& point : detailed)
        samples.push_back(point.pos);
    return samples;
}

std::vector<float> BuildPolylineArcLengths(const std::vector<XMFLOAT3>& points)
{
    std::vector<float> cumulativeLengths(points.size(), 0.0f);
    for (size_t i = 1; i < points.size(); ++i)
        cumulativeLengths[i] = cumulativeLengths[i - 1] + Distance3(points[i - 1], points[i]);
    return cumulativeLengths;
}

XMFLOAT3 SamplePolylineAtNormalizedDistance(const std::vector<XMFLOAT3>& points,
                                           const std::vector<float>& cumulativeLengths,
                                           float uCoord)
{
    if (points.empty())
        return { 0.0f, 0.0f, 0.0f };
    if (points.size() == 1 || cumulativeLengths.empty())
        return points.front();

    const float totalLength = cumulativeLengths.back();
    if (totalLength <= 1e-5f)
        return points.front();

    const float targetLength = std::clamp(uCoord, 0.0f, 1.0f) * totalLength;
    for (size_t i = 1; i < points.size(); ++i)
    {
        if (targetLength > cumulativeLengths[i])
            continue;

        const float segmentLength = cumulativeLengths[i] - cumulativeLengths[i - 1];
        if (segmentLength <= 1e-5f)
            return points[i];

        const float t = (targetLength - cumulativeLengths[i - 1]) / segmentLength;
        return Lerp3(points[i - 1], points[i], t);
    }

    return points.back();
}

float ComputePolylineLength(const std::vector<XMFLOAT3>& points)
{
    float length = 0.0f;
    for (size_t i = 1; i < points.size(); ++i)
        length += Distance3(points[i - 1], points[i]);
    return length;
}

XMFLOAT3 SamplePolylineAtDistance(const std::vector<XMFLOAT3>& points,
                                  const std::vector<float>& cumulativeLengths,
                                  float distance)
{
    if (points.empty())
        return { 0.0f, 0.0f, 0.0f };
    if (points.size() == 1 || cumulativeLengths.empty())
        return points.front();

    const float totalLength = cumulativeLengths.back();
    if (totalLength <= 1e-5f)
        return points.front();

    const float targetLength = std::clamp(distance, 0.0f, totalLength);
    for (size_t i = 1; i < points.size(); ++i)
    {
        if (targetLength > cumulativeLengths[i])
            continue;

        const float segmentLength = cumulativeLengths[i] - cumulativeLengths[i - 1];
        if (segmentLength <= 1e-5f)
            return points[i];

        const float t = (targetLength - cumulativeLengths[i - 1]) / segmentLength;
        return Lerp3(points[i - 1], points[i], t);
    }

    return points.back();
}

void AppendVerticalPreviewWorldPoint(std::vector<PreviewCurvePoint>& samples,
                                     const std::vector<XMFLOAT3>& baseCurve,
                                     const std::vector<float>& cumulativeLengths,
                                     float distance,
                                     float height,
                                     PreviewCurveSegmentKind kind)
{
    XMFLOAT3 worldPoint = SamplePolylineAtDistance(baseCurve, cumulativeLengths, distance);
    worldPoint.y = height;
    AppendUniquePreviewPoint(samples, worldPoint, kind);
}

VerticalPreviewSegment BuildVerticalPreviewSegment(VerticalGuidePoint p0,
                                                   VerticalGuidePoint p1,
                                                   VerticalGuidePoint p2)
{
    VerticalPreviewSegment segment;
    segment.p0 = p0;
    segment.p1 = p1;
    segment.p2 = p2;

    const float eps = 1e-5f;
    const float dx0 = p1.x - p0.x;
    const float dx1 = p2.x - p1.x;
    if (fabsf(dx0) < eps || fabsf(dx1) < eps)
        return segment;

    const float i1 = (p1.y - p0.y) / dx0;
    const float i2 = (p2.y - p1.y) / dx1;
    float L = (std::max)(0.0f, p1.vcl);
    L = (std::min)(L, fabsf(dx0) * 2.0f);
    L = (std::min)(L, fabsf(dx1) * 2.0f);
    if (L < eps)
        return segment;

    segment.validCurve = true;
    segment.slopeIn = i1;
    segment.slopeOut = i2;
    segment.curveLength = L;
    segment.curveStartX = p1.x - L * 0.5f;
    segment.curveStartY = p0.y + i1 * (segment.curveStartX - p0.x);
    segment.curveEndX = segment.curveStartX + L;
    segment.curveEndY = segment.curveStartY + i1 * L - ((i1 - i2) / (2.0f * L)) * L * L;
    segment.curveKind =
        (i2 > i1) ? PreviewCurveSegmentKind::VerticalCurveSag : PreviewCurveSegmentKind::VerticalCurveCrest;
    return segment;
}

float SampleVerticalPreviewHeight(const VerticalPreviewSegment& segment, float distance)
{
    if (!segment.validCurve)
    {
        const float dx = segment.p2.x - segment.p0.x;
        if (fabsf(dx) <= 1e-5f)
            return segment.p0.y;
        const float t = (distance - segment.p0.x) / dx;
        const float clampedT = std::clamp(t, 0.0f, 1.0f);
        return segment.p0.y + (segment.p2.y - segment.p0.y) * clampedT;
    }

    if (distance <= segment.curveStartX)
        return segment.p0.y + segment.slopeIn * (distance - segment.p0.x);
    if (distance >= segment.curveEndX)
        return segment.p2.y - segment.slopeOut * (segment.p2.x - distance);

    const float x = distance - segment.curveStartX;
    return segment.curveStartY + segment.slopeIn * x -
        ((segment.slopeIn - segment.slopeOut) / (2.0f * segment.curveLength)) * x * x;
}

PreviewCurveSegmentKind SampleVerticalPreviewKind(const VerticalPreviewSegment& segment, float distance)
{
    if (!segment.validCurve)
        return PreviewCurveSegmentKind::Other;
    if (distance >= segment.curveStartX && distance <= segment.curveEndX)
        return segment.curveKind;
    return PreviewCurveSegmentKind::Other;
}

std::vector<PreviewCurvePoint> BuildRoadVerticalPreviewCurveDetailed(const Road& road, float sampleInterval)
{
    const std::vector<XMFLOAT3> baseCurve = BuildRoadPreviewCurve(road, sampleInterval);
    return BuildRoadVerticalPreviewCurveDetailed(road, baseCurve, sampleInterval);
}

std::vector<PreviewCurvePoint> BuildRoadVerticalPreviewCurveDetailed(
    const Road& road,
    const std::vector<XMFLOAT3>& baseCurve,
    float sampleInterval)
{
    std::vector<PreviewCurvePoint> samples;
    if (baseCurve.size() < 2)
        return samples;

    const std::vector<float> cumulativeLengths = BuildPolylineArcLengths(baseCurve);
    if (cumulativeLengths.empty())
        return samples;

    const float totalLength = cumulativeLengths.back();
    if (totalLength <= 1e-5f)
        return samples;

    if (road.verticalCurve.empty())
    {
        const float sampleStep = (std::max)(sampleInterval, 1.0f);
        const int sampleCount =
            (std::max)(static_cast<int>(ceilf(totalLength / sampleStep)), 2);
        for (int step = 0; step <= sampleCount; ++step)
        {
            const float t = static_cast<float>(step) / static_cast<float>(sampleCount);
            const float distance = totalLength * t;
            const float height = baseCurve.front().y + (baseCurve.back().y - baseCurve.front().y) * t;
            AppendVerticalPreviewWorldPoint(
                samples,
                baseCurve,
                cumulativeLengths,
                distance,
                height,
                PreviewCurveSegmentKind::Other);
        }
        return samples;
    }

    std::vector<VerticalGuidePoint> guidePoints;
    guidePoints.reserve(road.verticalCurve.size() + 2);
    guidePoints.push_back({ 0.0f, baseCurve.front().y, 0.0f });

    std::vector<VerticalCurvePoint> verticalCurvePoints = road.verticalCurve;
    std::sort(
        verticalCurvePoints.begin(),
        verticalCurvePoints.end(),
        [](const VerticalCurvePoint& a, const VerticalCurvePoint& b)
        {
            return a.uCoord < b.uCoord;
        });

    for (const VerticalCurvePoint& curvePoint : verticalCurvePoints)
    {
        const float distance = std::clamp(curvePoint.uCoord, 0.0f, 1.0f) * totalLength;
        const XMFLOAT3 basePoint = SamplePolylineAtDistance(baseCurve, cumulativeLengths, distance);
        guidePoints.push_back({ distance, basePoint.y + curvePoint.offset, curvePoint.vcl });
    }

    guidePoints.push_back({ totalLength, baseCurve.back().y, 0.0f });
    if (guidePoints.size() < 3)
        return samples;

    std::vector<VerticalPreviewSegment> segments;
    segments.reserve(guidePoints.size() - 2);
    for (size_t i = 1; i + 1 < guidePoints.size(); ++i)
    {
        const bool isFirstCurve = (i == 1);
        const bool isLastCurve = (i + 2 == guidePoints.size());
        const VerticalGuidePoint segmentStart = isFirstCurve
            ? guidePoints.front()
            : VerticalGuidePoint
            {
                (guidePoints[i - 1].x + guidePoints[i].x) * 0.5f,
                (guidePoints[i - 1].y + guidePoints[i].y) * 0.5f,
                0.0f
            };
        const VerticalGuidePoint segmentEnd = isLastCurve
            ? guidePoints.back()
            : VerticalGuidePoint
            {
                (guidePoints[i].x + guidePoints[i + 1].x) * 0.5f,
                (guidePoints[i].y + guidePoints[i + 1].y) * 0.5f,
                0.0f
            };
        segments.push_back(BuildVerticalPreviewSegment(segmentStart, guidePoints[i], segmentEnd));
    }

    const float sampleStep = (std::max)(sampleInterval, 1.0f);
    for (size_t segmentIndex = 0; segmentIndex < segments.size(); ++segmentIndex)
    {
        const VerticalPreviewSegment& segment = segments[segmentIndex];
        const float segmentStartX = segment.p0.x;
        const float segmentEndX = segment.p2.x;
        const float segmentLength = segmentEndX - segmentStartX;
        if (segmentLength <= 1e-5f)
            continue;

        const int sampleCount =
            (std::max)(static_cast<int>(ceilf(segmentLength / sampleStep)), 2);
        const int startStep = (segmentIndex == 0) ? 0 : 1;
        for (int step = startStep; step <= sampleCount; ++step)
        {
            const float t = static_cast<float>(step) / static_cast<float>(sampleCount);
            const float distance = segmentStartX + (segmentEndX - segmentStartX) * t;
            const float height = SampleVerticalPreviewHeight(segment, distance);
            const PreviewCurveSegmentKind kind = SampleVerticalPreviewKind(segment, distance);
            AppendVerticalPreviewWorldPoint(samples, baseCurve, cumulativeLengths, distance, height, kind);
        }
    }

    return samples;
}

float ComputeAverageAbsoluteGradePercent(const std::vector<XMFLOAT3>& points)
{
    float totalHorizontalDistance = 0.0f;
    float totalAbsoluteRise = 0.0f;
    for (size_t i = 1; i < points.size(); ++i)
    {
        const float dx = points[i].x - points[i - 1].x;
        const float dz = points[i].z - points[i - 1].z;
        const float horizontalDistance = sqrtf(dx * dx + dz * dz);
        if (horizontalDistance <= 1e-4f)
            continue;

        totalHorizontalDistance += horizontalDistance;
        totalAbsoluteRise += fabsf(points[i].y - points[i - 1].y);
    }

    if (totalHorizontalDistance <= 1e-4f)
        return 0.0f;

    return (totalAbsoluteRise / totalHorizontalDistance) * 100.0f;
}

float ComputeMaxAbsoluteGradePercent(const std::vector<XMFLOAT3>& points)
{
    float maxGradePercent = 0.0f;
    for (size_t i = 1; i < points.size(); ++i)
    {
        const float dx = points[i].x - points[i - 1].x;
        const float dz = points[i].z - points[i - 1].z;
        const float horizontalDistance = sqrtf(dx * dx + dz * dz);
        if (horizontalDistance <= 1e-4f)
            continue;

        const float gradePercent =
            (fabsf(points[i].y - points[i - 1].y) / horizontalDistance) * 100.0f;
        maxGradePercent = (std::max)(maxGradePercent, gradePercent);
    }

    return maxGradePercent;
}
}

void PolylineEditor::InvalidateAllPreviewCaches()
{
    m_roadPreviewCaches.clear();
}

void PolylineEditor::InvalidateRoadPreviewCache(int roadIndex)
{
    if (roadIndex < 0)
        return;
    EnsureRoadPreviewCacheSize();
    if (roadIndex >= static_cast<int>(m_roadPreviewCaches.size()))
        return;
    m_roadPreviewCaches[roadIndex] = RoadPreviewCache();
}

void PolylineEditor::InvalidateAllGradeColorCaches()
{
    EnsureRoadPreviewCacheSize();
    for (RoadPreviewCache& cache : m_roadPreviewCaches)
    {
        cache.verticalGradeColorsValid = false;
        cache.verticalGradeColors.clear();
    }
}

void PolylineEditor::InvalidateRoadGradeColorCache(int roadIndex)
{
    if (roadIndex < 0)
        return;
    EnsureRoadPreviewCacheSize();
    if (roadIndex >= static_cast<int>(m_roadPreviewCaches.size()))
        return;
    RoadPreviewCache& cache = m_roadPreviewCaches[roadIndex];
    cache.verticalGradeColorsValid = false;
    cache.verticalGradeColors.clear();
}

void PolylineEditor::InvalidateAllTerrainClearanceCaches()
{
    EnsureRoadPreviewCacheSize();
    for (RoadPreviewCache& cache : m_roadPreviewCaches)
    {
        cache.terrainClearanceValid = false;
        cache.terrainClearanceSamples.clear();
    }
}

void PolylineEditor::InvalidateAllBankCaches()
{
    EnsureRoadPreviewCacheSize();
    for (RoadPreviewCache& cache : m_roadPreviewCaches)
    {
        cache.bankFrameSamplesValid = false;
        cache.bankFrameSamples.clear();
        cache.bankPreviewColorsValid = false;
        cache.bankPreviewColors.clear();
    }
}

void PolylineEditor::SetPreviewSampleInterval(float value)
{
    const float clampedValue = (std::max)(1.0f, value);
    if (fabsf(m_previewSampleInterval - clampedValue) <= 1e-5f)
        return;
    m_previewSampleInterval = clampedValue;
    InvalidateAllPreviewCaches();
}

void PolylineEditor::EnsureRoadPreviewCacheSize() const
{
    if (!m_network)
    {
        m_roadPreviewCaches.clear();
        return;
    }
    if (m_roadPreviewCaches.size() != m_network->roads.size())
        m_roadPreviewCaches.resize(m_network->roads.size());
}

const std::vector<PreviewCurvePoint>& PolylineEditor::GetRoadPreviewCurveDetailedCached(int roadIndex) const
{
    static const std::vector<PreviewCurvePoint> kEmpty;
    if (!m_network || roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return kEmpty;

    EnsureRoadPreviewCacheSize();
    RoadPreviewCache& cache = m_roadPreviewCaches[roadIndex];
    if (!cache.previewDetailedValid)
    {
        cache.previewDetailed = BuildRoadPreviewCurveDetailed(
            m_network->roads[roadIndex],
            m_previewSampleInterval);
        cache.previewDetailedValid = true;
        cache.previewPositionsValid = false;
        cache.previewArcLengthsValid = false;
        cache.metricsValid = false;
    }
    return cache.previewDetailed;
}

const std::vector<XMFLOAT3>& PolylineEditor::GetRoadPreviewCurveCached(int roadIndex) const
{
    static const std::vector<XMFLOAT3> kEmpty;
    if (!m_network || roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return kEmpty;

    EnsureRoadPreviewCacheSize();
    RoadPreviewCache& cache = m_roadPreviewCaches[roadIndex];
    if (!cache.previewPositionsValid)
    {
        cache.previewPositions = ExtractPreviewCurvePositions(GetRoadPreviewCurveDetailedCached(roadIndex));
        cache.previewPositionsValid = true;
    }
    return cache.previewPositions;
}

const std::vector<float>& PolylineEditor::GetRoadPreviewCurveArcLengthsCached(int roadIndex) const
{
    static const std::vector<float> kEmpty;
    if (!m_network || roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return kEmpty;

    EnsureRoadPreviewCacheSize();
    RoadPreviewCache& cache = m_roadPreviewCaches[roadIndex];
    if (!cache.previewArcLengthsValid)
    {
        cache.previewArcLengths = BuildPolylineArcLengths(GetRoadPreviewCurveCached(roadIndex));
        cache.previewArcLengthsValid = true;
    }
    return cache.previewArcLengths;
}

const std::vector<PreviewCurvePoint>& PolylineEditor::GetRoadVerticalPreviewCurveDetailedCached(int roadIndex) const
{
    static const std::vector<PreviewCurvePoint> kEmpty;
    if (!m_network || roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return kEmpty;

    EnsureRoadPreviewCacheSize();
    RoadPreviewCache& cache = m_roadPreviewCaches[roadIndex];
    if (!cache.verticalDetailedValid)
    {
        cache.verticalDetailed =
            BuildRoadVerticalPreviewCurveDetailed(
                m_network->roads[roadIndex],
                GetRoadPreviewCurveCached(roadIndex),
                m_previewSampleInterval);
        cache.verticalDetailedValid = true;
        cache.verticalPositionsValid = false;
        cache.verticalArcLengthsValid = false;
        cache.bankFrameSamplesValid = false;
        cache.verticalGradeColorsValid = false;
        cache.lanePreviewLinesValid = false;
        cache.metricsValid = false;
    }
    return cache.verticalDetailed;
}

const std::vector<XMFLOAT3>& PolylineEditor::GetRoadVerticalPreviewCurveCached(int roadIndex) const
{
    static const std::vector<XMFLOAT3> kEmpty;
    if (!m_network || roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return kEmpty;

    EnsureRoadPreviewCacheSize();
    RoadPreviewCache& cache = m_roadPreviewCaches[roadIndex];
    if (!cache.verticalPositionsValid)
    {
        cache.verticalPositions =
            ExtractPreviewCurvePositions(GetRoadVerticalPreviewCurveDetailedCached(roadIndex));
        cache.verticalPositionsValid = true;
    }
    return cache.verticalPositions;
}

const std::vector<float>& PolylineEditor::GetRoadVerticalPreviewCurveArcLengthsCached(int roadIndex) const
{
    static const std::vector<float> kEmpty;
    if (!m_network || roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return kEmpty;

    EnsureRoadPreviewCacheSize();
    RoadPreviewCache& cache = m_roadPreviewCaches[roadIndex];
    if (!cache.verticalArcLengthsValid)
    {
        cache.verticalArcLengths = BuildPolylineArcLengths(GetRoadVerticalPreviewCurveCached(roadIndex));
        cache.verticalArcLengthsValid = true;
    }
    return cache.verticalArcLengths;
}

const std::vector<XMFLOAT3>& PolylineEditor::GetRoadParametricPreviewCurveCached(int roadIndex) const
{
    if (m_mode == EditorMode::BankAngleEdit || m_mode == EditorMode::LaneEdit)
        return GetRoadVerticalPreviewCurveCached(roadIndex);
    return GetRoadPreviewCurveCached(roadIndex);
}

const std::vector<float>& PolylineEditor::GetRoadParametricPreviewCurveArcLengthsCached(int roadIndex) const
{
    if (m_mode == EditorMode::BankAngleEdit || m_mode == EditorMode::LaneEdit)
        return GetRoadVerticalPreviewCurveArcLengthsCached(roadIndex);
    return GetRoadPreviewCurveArcLengthsCached(roadIndex);
}

const std::vector<BankFrameSample>& PolylineEditor::GetRoadBankFrameSamplesCached(int roadIndex) const
{
    static const std::vector<BankFrameSample> kEmpty;
    if (!m_network || roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return kEmpty;

    EnsureRoadPreviewCacheSize();
    RoadPreviewCache& cache = m_roadPreviewCaches[roadIndex];
    if (!cache.bankFrameSamplesValid)
    {
        cache.bankFrameSamples.clear();
        const Road& road = m_network->roads[roadIndex];
        const std::vector<XMFLOAT3>& curve = GetRoadVerticalPreviewCurveCached(roadIndex);
        const std::vector<float>& arcLengths = GetRoadVerticalPreviewCurveArcLengthsCached(roadIndex);
        if (curve.size() >= 2 && !arcLengths.empty())
        {
            const float totalLength = arcLengths.back();
            const auto appendSample = [&](float distance)
            {
                const float clampedDistance = std::clamp(distance, 0.0f, totalLength);
                const float prevDistance = (std::max)(0.0f, clampedDistance - kBankCurvatureStepMeters);
                const float nextDistance = (std::min)(totalLength, clampedDistance + kBankCurvatureStepMeters);
                const XMFLOAT3 p0 = SamplePolylineAtDistance(curve, arcLengths, prevDistance);
                const XMFLOAT3 p1 = SamplePolylineAtDistance(curve, arcLengths, clampedDistance);
                const XMFLOAT3 p2 = SamplePolylineAtDistance(curve, arcLengths, nextDistance);

                XMFLOAT3 tangent = Normalize3(Sub3(p2, p0));
                if (Length3(tangent) <= 1e-5f)
                    tangent = Normalize3(Sub3(p2, p1));
                if (Length3(tangent) <= 1e-5f)
                    tangent = Normalize3(Sub3(p1, p0));
                if (Length3(tangent) <= 1e-5f)
                    tangent = { 0.0f, 0.0f, 1.0f };

                const XMFLOAT3 halfVector = ComputeHalfVectorXZ(p0, p1, p2);
                XMFLOAT3 baseLeft = Normalize3(Cross3({ 0.0f, 1.0f, 0.0f }, tangent));
                if (Length3(baseLeft) <= 1e-5f)
                    baseLeft = { 1.0f, 0.0f, 0.0f };
                XMFLOAT3 baseUp = Normalize3(Cross3(tangent, baseLeft));
                if (Length3(baseUp) <= 1e-5f)
                    baseUp = { 0.0f, 1.0f, 0.0f };

                const float localHalfX = DotXZ(halfVector, baseLeft);
                const float sign = localHalfX < 0.0f ? -1.0f : 1.0f;
                const float angleRadians = EvaluateInterpolatedBankAngleRadians(road, curve, arcLengths, clampedDistance);
                XMFLOAT3 bankLeft = Normalize3(Add3(
                    Scale3(baseLeft, cosf(angleRadians)),
                    Scale3(baseUp, sign * sinf(angleRadians))));
                XMFLOAT3 bankUp = Normalize3(Cross3(tangent, bankLeft));
                if (Length3(bankUp) <= 1e-5f)
                    bankUp = baseUp;

                cache.bankFrameSamples.push_back({ p1, bankLeft, bankUp, angleRadians });
            };

            const float spacing = (std::max)(0.5f, m_bankVectorInterval);
            for (float distance = 0.0f; distance <= totalLength + 1e-4f; distance += spacing)
                appendSample(distance);

            if (cache.bankFrameSamples.empty() ||
                Distance3(cache.bankFrameSamples.back().pos, curve.back()) > 1e-3f)
            {
                appendSample(totalLength);
            }
        }
        cache.bankFrameSamplesValid = true;
    }
    return cache.bankFrameSamples;
}

const std::vector<unsigned int>& PolylineEditor::GetRoadBankPreviewColorsCached(int roadIndex) const
{
    static const std::vector<unsigned int> kEmpty;
    if (!m_network || roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return kEmpty;

    EnsureRoadPreviewCacheSize();
    RoadPreviewCache& cache = m_roadPreviewCaches[roadIndex];
    if (!cache.bankPreviewColorsValid)
    {
        cache.bankPreviewColors.clear();
        const std::vector<PreviewCurvePoint>& detailed = GetRoadVerticalPreviewCurveDetailedCached(roadIndex);
        const std::vector<XMFLOAT3>& curve = GetRoadVerticalPreviewCurveCached(roadIndex);
        const std::vector<float>& arcLengths = GetRoadVerticalPreviewCurveArcLengthsCached(roadIndex);
        if (detailed.size() >= 2 && !curve.empty() && !arcLengths.empty())
        {
            cache.bankPreviewColors.reserve(detailed.size() - 1);
            const float totalLength = arcLengths.back();
            for (size_t sampleIndex = 0; sampleIndex + 1 < detailed.size(); ++sampleIndex)
            {
                const float midDistance = (std::min)(
                    totalLength,
                    0.5f * (arcLengths[(std::min)(sampleIndex, arcLengths.size() - 1)] +
                            arcLengths[(std::min)(sampleIndex + 1, arcLengths.size() - 1)]));
                const float angleDegrees = XMConvertToDegrees(
                    fabsf(EvaluateInterpolatedBankAngleRadians(
                        m_network->roads[roadIndex],
                        curve,
                        arcLengths,
                        midDistance)));
                const XMFLOAT4 bankColor = PreviewGradeColor(angleDegrees, m_bankAngleColorMaxDegrees);
                cache.bankPreviewColors.push_back(IM_COL32(
                    static_cast<int>(bankColor.x * 255.0f + 0.5f),
                    static_cast<int>(bankColor.y * 255.0f + 0.5f),
                    static_cast<int>(bankColor.z * 255.0f + 0.5f),
                    static_cast<int>(bankColor.w * 255.0f + 0.5f)));
            }
        }
        cache.bankPreviewColorsValid = true;
    }
    return cache.bankPreviewColors;
}

const std::vector<LanePreviewLine>& PolylineEditor::GetRoadLanePreviewLinesCached(int roadIndex) const
{
    static const std::vector<LanePreviewLine> kEmpty;
    if (!m_network || roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return kEmpty;

    EnsureRoadPreviewCacheSize();
    RoadPreviewCache& cache = m_roadPreviewCaches[roadIndex];
    if (!cache.lanePreviewLinesValid)
    {
        cache.lanePreviewLines.clear();
        const Road& road = m_network->roads[roadIndex];
        const std::vector<XMFLOAT3>& curve = GetRoadVerticalPreviewCurveCached(roadIndex);
        const std::vector<float>& arcLengths = GetRoadVerticalPreviewCurveArcLengthsCached(roadIndex);
        if (curve.size() >= 2 && arcLengths.size() == curve.size())
        {
            cache.lanePreviewLines.resize(static_cast<size_t>(LanePreviewLineKind::Count));

            auto appendBreak = [&](LanePreviewLineKind kind)
            {
                std::vector<XMFLOAT3>& points =
                    cache.lanePreviewLines[static_cast<size_t>(kind)].points;
                if (!points.empty() && IsFinitePoint3(points.back()))
                    points.push_back(MakeInvalidPoint3());
            };

            auto pushPoint = [&](LanePreviewLineKind kind, XMFLOAT3 point)
            {
                std::vector<XMFLOAT3>& points =
                    cache.lanePreviewLines[static_cast<size_t>(kind)].points;
                if (!points.empty() && !IsFinitePoint3(points.back()))
                {
                    if (!points.empty() && points.size() >= 2 &&
                        !IsFinitePoint3(points[points.size() - 2]))
                    {
                        points.pop_back();
                    }
                    points.push_back(point);
                    return;
                }
                AppendUniquePoint(points, point);
            };

            for (size_t sampleIndex = 0; sampleIndex < curve.size(); ++sampleIndex)
            {
                const float distance = arcLengths[sampleIndex];
                const BankFrameSample frame = EvaluateBankFrameAtDistance(road, curve, arcLengths, distance);
                const EvaluatedLaneSection section = EvaluateLaneSectionAtDistance(road, arcLengths, distance);

                const float centerOffset = section.offsetCenter;
                const float centerLeftBoundary = centerOffset + section.widthCenter * 0.5f;
                const float centerRightBoundary = centerOffset - section.widthCenter * 0.5f;
                const float left1Outer = centerLeftBoundary + section.widthLeft1;
                const float left2Outer = left1Outer + section.widthLeft2;
                const float right1Outer = centerRightBoundary - section.widthRight1;
                const float right2Outer = right1Outer - section.widthRight2;

                pushPoint(LanePreviewLineKind::Centerline, Add3(frame.pos, Scale3(frame.left, centerOffset)));

                if (section.widthLeft2 > 1e-4f)
                    pushPoint(LanePreviewLineKind::LeftOuter2, Add3(frame.pos, Scale3(frame.left, left2Outer)));
                else
                    appendBreak(LanePreviewLineKind::LeftOuter2);

                if (section.widthLeft1 > 1e-4f)
                {
                    const XMFLOAT3 point = Add3(frame.pos, Scale3(frame.left, left1Outer));
                    if (section.widthLeft2 > 1e-4f)
                    {
                        pushPoint(LanePreviewLineKind::LeftBoundary1, point);
                        appendBreak(LanePreviewLineKind::LeftOuter1);
                    }
                    else
                    {
                        pushPoint(LanePreviewLineKind::LeftOuter1, point);
                        appendBreak(LanePreviewLineKind::LeftBoundary1);
                    }
                }
                else
                {
                    appendBreak(LanePreviewLineKind::LeftOuter1);
                    appendBreak(LanePreviewLineKind::LeftBoundary1);
                }

                if (section.widthCenter > 1e-4f)
                {
                    pushPoint(LanePreviewLineKind::CenterLeftBoundary,
                        Add3(frame.pos, Scale3(frame.left, centerLeftBoundary)));
                    pushPoint(LanePreviewLineKind::CenterRightBoundary,
                        Add3(frame.pos, Scale3(frame.left, centerRightBoundary)));
                }
                else
                {
                    appendBreak(LanePreviewLineKind::CenterLeftBoundary);
                    appendBreak(LanePreviewLineKind::CenterRightBoundary);
                }

                if (section.widthRight1 > 1e-4f)
                {
                    const XMFLOAT3 point = Add3(frame.pos, Scale3(frame.left, right1Outer));
                    if (section.widthRight2 > 1e-4f)
                    {
                        pushPoint(LanePreviewLineKind::RightBoundary1, point);
                        appendBreak(LanePreviewLineKind::RightOuter1);
                    }
                    else
                    {
                        pushPoint(LanePreviewLineKind::RightOuter1, point);
                        appendBreak(LanePreviewLineKind::RightBoundary1);
                    }
                }
                else
                {
                    appendBreak(LanePreviewLineKind::RightOuter1);
                    appendBreak(LanePreviewLineKind::RightBoundary1);
                }

                if (section.widthRight2 > 1e-4f)
                    pushPoint(LanePreviewLineKind::RightOuter2, Add3(frame.pos, Scale3(frame.left, right2Outer)));
                else
                    appendBreak(LanePreviewLineKind::RightOuter2);
            }
        }
        cache.lanePreviewLinesValid = true;
    }
    return cache.lanePreviewLines;
}

const std::vector<unsigned int>& PolylineEditor::GetRoadVerticalGradeColorsCached(int roadIndex) const
{
    static const std::vector<unsigned int> kEmpty;
    if (!m_network || roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return kEmpty;

    EnsureRoadPreviewCacheSize();
    RoadPreviewCache& cache = m_roadPreviewCaches[roadIndex];
    if (!cache.verticalGradeColorsValid)
    {
        const std::vector<PreviewCurvePoint>& detailed = GetRoadVerticalPreviewCurveDetailedCached(roadIndex);
        cache.verticalGradeColors.clear();
        if (detailed.size() >= 2)
        {
            cache.verticalGradeColors.reserve(detailed.size() - 1);
            for (size_t sampleIndex = 0; sampleIndex + 1 < detailed.size(); ++sampleIndex)
            {
                unsigned int segmentColor = IM_COL32(255, 255, 255, 210);
                const float horizontalDistance = DistanceXZ3(
                    detailed[sampleIndex].pos,
                    detailed[sampleIndex + 1].pos);
                if (horizontalDistance > 1e-4f)
                {
                    const float dy = detailed[sampleIndex + 1].pos.y - detailed[sampleIndex].pos.y;
                    const float gradePercent = fabsf(dy) / horizontalDistance * 100.0f;
                    const XMFLOAT4 gradeColor =
                        PreviewGradeColor(gradePercent, m_roadGradeRedThresholdPercent);
                    segmentColor = IM_COL32(
                        static_cast<int>(gradeColor.x * 255.0f + 0.5f),
                        static_cast<int>(gradeColor.y * 255.0f + 0.5f),
                        static_cast<int>(gradeColor.z * 255.0f + 0.5f),
                        static_cast<int>(gradeColor.w * 255.0f + 0.5f));
                }
                cache.verticalGradeColors.push_back(segmentColor);
            }
        }
        cache.verticalGradeColorsValid = true;
    }
    return cache.verticalGradeColors;
}

const std::vector<TerrainClearanceSample>& PolylineEditor::GetRoadTerrainClearanceSamplesCached(int roadIndex) const
{
    static const std::vector<TerrainClearanceSample> kEmpty;
    if (!m_network || roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return kEmpty;
    if (m_terrain == nullptr || !m_terrain->IsReady())
        return kEmpty;

    EnsureRoadPreviewCacheSize();
    RoadPreviewCache& cache = m_roadPreviewCaches[roadIndex];
    if (!cache.terrainClearanceValid)
    {
        cache.terrainClearanceSamples.clear();
        const std::vector<XMFLOAT3>& verticalPreviewCurve = GetRoadVerticalPreviewCurveCached(roadIndex);
        if (verticalPreviewCurve.size() >= 2 && m_roadTerrainClearanceInterval > 1e-4f)
        {
            const std::vector<float> cumulativeLengths = BuildPolylineArcLengths(verticalPreviewCurve);
            const float totalLength = cumulativeLengths.empty() ? 0.0f : cumulativeLengths.back();
            if (totalLength > 1e-4f)
            {
                const float spacing = (std::max)(0.5f, m_roadTerrainClearanceInterval);
                const auto appendSample = [&](float distance)
                {
                    TerrainClearanceSample sample;
                    sample.curvePos = SamplePolylineAtDistance(
                        verticalPreviewCurve,
                        cumulativeLengths,
                        distance);
                    sample.terrainPos = sample.curvePos;
                    sample.terrainPos.y = m_terrain->GetHeightAt(sample.curvePos.x, sample.curvePos.z);
                    sample.color = (sample.terrainPos.y > sample.curvePos.y)
                        ? IM_COL32(90, 170, 255, 215)
                        : IM_COL32(255, 160, 60, 215);
                    cache.terrainClearanceSamples.push_back(sample);
                };

                for (float distance = 0.0f; distance <= totalLength + 1e-4f; distance += spacing)
                    appendSample(distance);

                const float remainder = fmodf(totalLength, spacing);
                if (remainder > 1e-3f && totalLength > spacing)
                    appendSample(totalLength);
            }
        }
        cache.terrainClearanceValid = true;
    }
    return cache.terrainClearanceSamples;
}

void PolylineEditor::SetShowRoadGradeGradient(bool show)
{
    if (m_showRoadGradeGradient == show)
        return;
    m_showRoadGradeGradient = show;
    if (show)
        InvalidateAllGradeColorCaches();
}

void PolylineEditor::SetRoadGradeRedThresholdPercent(float value)
{
    if (fabsf(m_roadGradeRedThresholdPercent - value) <= 1e-5f)
        return;
    m_roadGradeRedThresholdPercent = value;
    InvalidateAllGradeColorCaches();
}

void PolylineEditor::SetShowRoadTerrainClearance(bool show)
{
    if (m_showRoadTerrainClearance == show)
        return;
    m_showRoadTerrainClearance = show;
    if (show)
        InvalidateAllTerrainClearanceCaches();
}

void PolylineEditor::SetRoadTerrainClearanceInterval(float value)
{
    const float clampedValue = (std::max)(0.5f, value);
    if (fabsf(m_roadTerrainClearanceInterval - clampedValue) <= 1e-5f)
        return;
    m_roadTerrainClearanceInterval = clampedValue;
    InvalidateAllTerrainClearanceCaches();
}

void PolylineEditor::SetBankVectorInterval(float value)
{
    const float clampedValue = (std::max)(0.5f, value);
    if (fabsf(m_bankVectorInterval - clampedValue) <= 1e-5f)
        return;
    m_bankVectorInterval = clampedValue;
    InvalidateAllBankCaches();
}

void PolylineEditor::SetBankAngleColorMaxDegrees(float value)
{
    const float clampedValue = (std::max)(0.1f, value);
    if (fabsf(m_bankAngleColorMaxDegrees - clampedValue) <= 1e-5f)
        return;
    m_bankAngleColorMaxDegrees = clampedValue;
    EnsureRoadPreviewCacheSize();
    for (RoadPreviewCache& cache : m_roadPreviewCaches)
    {
        cache.bankPreviewColorsValid = false;
        cache.bankPreviewColors.clear();
    }
}

const RoadPreviewMetrics& PolylineEditor::GetRoadPreviewMetricsCached(int roadIndex) const
{
    static const RoadPreviewMetrics kEmpty;
    if (!m_network || roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return kEmpty;

    EnsureRoadPreviewCacheSize();
    RoadPreviewCache& cache = m_roadPreviewCaches[roadIndex];
    if (!cache.metricsValid)
    {
        const std::vector<XMFLOAT3>& previewCurve = GetRoadVerticalPreviewCurveCached(roadIndex);
        RoadPreviewMetrics metrics;
        if (!previewCurve.empty())
        {
            XMFLOAT3 center = { 0.0f, 0.0f, 0.0f };
            for (const XMFLOAT3& point : previewCurve)
            {
                center.x += point.x;
                center.y += point.y;
                center.z += point.z;
            }
            const float invCount = 1.0f / static_cast<float>(previewCurve.size());
            center.x *= invCount;
            center.y *= invCount;
            center.z *= invCount;

            metrics.valid = true;
            metrics.center = center;
            metrics.length = ComputePolylineLength(previewCurve);
            metrics.averageGradePercent = ComputeAverageAbsoluteGradePercent(previewCurve);
            metrics.maxGradePercent = ComputeMaxAbsoluteGradePercent(previewCurve);
        }
        cache.metrics = metrics;
        cache.metricsValid = true;
    }
    return cache.metrics;
}

bool PolylineEditor::ConsumeStatusMessage(std::string& outMessage)
{
    if (m_statusMessage.empty())
        return false;

    outMessage = m_statusMessage;
    m_statusMessage.clear();
    return true;
}

bool PolylineEditor::Load(const char* path)
{
    if (!m_network->LoadFromFile(path))
        return false;

    InvalidateAllPreviewCaches();
    ClearHistory();
    SanitizeSelection();
    return true;
}

bool PolylineEditor::GetFocusTarget(XMFLOAT3& outTarget) const
{
    if (!m_network)
        return false;

    auto tryGetParametricPointFocusTarget = [this, &outTarget](int roadIndex, float uCoord) -> bool
    {
        if (roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
            return false;

        const std::vector<XMFLOAT3>& previewCurve = GetRoadParametricPreviewCurveCached(roadIndex);
        if (previewCurve.empty())
            return false;

        const std::vector<float>& cumulativeLengths = GetRoadPreviewCurveArcLengthsCached(roadIndex);
        outTarget = SamplePolylineAtNormalizedDistance(previewCurve, cumulativeLengths, uCoord);
        return true;
    };

    VerticalCurveRef selectedVerticalCurve;
    if (GetPrimarySelectedVerticalCurvePoint(selectedVerticalCurve))
    {
        const Road& road = m_network->roads[selectedVerticalCurve.roadIndex];
        if (selectedVerticalCurve.curveIndex >= 0 &&
            selectedVerticalCurve.curveIndex < static_cast<int>(road.verticalCurve.size()) &&
            tryGetParametricPointFocusTarget(
                selectedVerticalCurve.roadIndex,
                road.verticalCurve[selectedVerticalCurve.curveIndex].uCoord))
        {
            return true;
        }
    }

    BankAngleRef selectedBankAngle;
    if (GetPrimarySelectedBankAnglePoint(selectedBankAngle))
    {
        const Road& road = m_network->roads[selectedBankAngle.roadIndex];
        if (selectedBankAngle.pointIndex >= 0 &&
            selectedBankAngle.pointIndex < static_cast<int>(road.bankAngle.size()) &&
            tryGetParametricPointFocusTarget(
                selectedBankAngle.roadIndex,
                road.bankAngle[selectedBankAngle.pointIndex].uCoord))
        {
            return true;
        }
    }

    LaneSectionRef selectedLaneSection;
    if (GetPrimarySelectedLaneSectionPoint(selectedLaneSection))
    {
        const Road& road = m_network->roads[selectedLaneSection.roadIndex];
        if (selectedLaneSection.pointIndex >= 0 &&
            selectedLaneSection.pointIndex < static_cast<int>(road.laneSections.size()) &&
            tryGetParametricPointFocusTarget(
                selectedLaneSection.roadIndex,
                road.laneSections[selectedLaneSection.pointIndex].uCoord))
        {
            return true;
        }
    }

    if (m_activeRoad >= 0 &&
        m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        const Road& road = m_network->roads[m_activeRoad];
        if (m_activePoint >= 0 &&
            m_activePoint < static_cast<int>(road.points.size()))
        {
            outTarget = road.points[m_activePoint].pos;
            return true;
        }

        if (!road.points.empty())
        {
            XMFLOAT3 center = { 0.0f, 0.0f, 0.0f };
            for (const RoadPoint& point : road.points)
            {
                center.x += point.pos.x;
                center.y += point.pos.y;
                center.z += point.pos.z;
            }

            const float invCount = 1.0f / static_cast<float>(road.points.size());
            outTarget = { center.x * invCount, center.y * invCount, center.z * invCount };
            return true;
        }
    }

    if (m_activeIntersection >= 0 &&
        m_activeIntersection < static_cast<int>(m_network->intersections.size()))
    {
        outTarget = m_network->intersections[m_activeIntersection].pos;
        return true;
    }

    return false;
}

bool PolylineEditor::GetPrimaryRoadForPathfinding(int& outRoadIndex) const
{
    if (!m_network)
        return false;

    if (m_selectedPoints.size() == 1)
    {
        const PointRef& pointRef = m_selectedPoints.front();
        if (pointRef.roadIndex >= 0 &&
            pointRef.roadIndex < static_cast<int>(m_network->roads.size()))
        {
            outRoadIndex = pointRef.roadIndex;
            return true;
        }
    }

    if (m_selectedRoads.size() == 1)
    {
        const int roadIndex = m_selectedRoads.front();
        if (roadIndex >= 0 && roadIndex < static_cast<int>(m_network->roads.size()))
        {
            outRoadIndex = roadIndex;
            return true;
        }
    }

    if (m_activeRoad >= 0 && m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        outRoadIndex = m_activeRoad;
        return true;
    }

    return false;
}

void PolylineEditor::CollectSelectedRoadIndices(std::vector<int>& outRoadIndices) const
{
    outRoadIndices.clear();
    if (!m_network)
        return;

    outRoadIndices = m_selectedRoads;
    for (const PointRef& pointRef : m_selectedPoints)
    {
        if (pointRef.roadIndex >= 0 &&
            pointRef.roadIndex < static_cast<int>(m_network->roads.size()))
        {
            outRoadIndices.push_back(pointRef.roadIndex);
        }
    }

    if (outRoadIndices.empty() &&
        m_activeRoad >= 0 &&
        m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        outRoadIndices.push_back(m_activeRoad);
    }

    std::sort(outRoadIndices.begin(), outRoadIndices.end());
    outRoadIndices.erase(
        std::unique(outRoadIndices.begin(), outRoadIndices.end()),
        outRoadIndices.end());
}

void PolylineEditor::CollectSelectedIntersectionIndices(std::vector<int>& outIntersectionIndices) const
{
    outIntersectionIndices.clear();
    if (!m_network)
        return;

    outIntersectionIndices = m_selectedIntersections;
    if (outIntersectionIndices.empty() &&
        m_activeIntersection >= 0 &&
        m_activeIntersection < static_cast<int>(m_network->intersections.size()))
    {
        outIntersectionIndices.push_back(m_activeIntersection);
    }

    std::sort(outIntersectionIndices.begin(), outIntersectionIndices.end());
    outIntersectionIndices.erase(
        std::unique(outIntersectionIndices.begin(), outIntersectionIndices.end()),
        outIntersectionIndices.end());
}

bool PolylineEditor::SelectAllPointsOnSelectedRoads()
{
    if (!m_network)
        return false;

    std::vector<int> roadIndices;
    CollectSelectedRoadIndices(roadIndices);
    if (roadIndices.empty())
        return false;

    std::vector<PointRef> pointRefs;
    for (int roadIndex : roadIndices)
    {
        if (roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
            continue;

        const Road& road = m_network->roads[roadIndex];
        for (int pointIndex = 0; pointIndex < static_cast<int>(road.points.size()); ++pointIndex)
            pointRefs.push_back({ roadIndex, pointIndex });
    }

    if (pointRefs.empty())
        return false;

    m_selectedPoints = std::move(pointRefs);
    m_activeRoad = m_selectedPoints.front().roadIndex;
    m_activePoint = m_selectedPoints.front().pointIndex;
    return true;
}

bool PolylineEditor::DisconnectSelectedRoadEndpoints()
{
    if (!m_network)
        return false;

    bool disconnectedAny = false;
    for (const PointRef& pointRef : m_selectedPoints)
    {
        if (pointRef.roadIndex < 0 ||
            pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
            continue;

        Road& road = m_network->roads[pointRef.roadIndex];
        if (road.points.empty() ||
            pointRef.pointIndex < 0 ||
            pointRef.pointIndex >= static_cast<int>(road.points.size()))
            continue;

        if (pointRef.pointIndex == 0 && !road.startIntersectionId.empty())
        {
            const int intersectionIndex = FindIntersectionIndexById(road.startIntersectionId);
            if (intersectionIndex < 0 || !IsIntersectionSelected(intersectionIndex))
            {
                road.startIntersectionId.clear();
                disconnectedAny = true;
            }
        }

        const int lastPointIndex = static_cast<int>(road.points.size()) - 1;
        if (pointRef.pointIndex == lastPointIndex && !road.endIntersectionId.empty())
        {
            const int intersectionIndex = FindIntersectionIndexById(road.endIntersectionId);
            if (intersectionIndex < 0 || !IsIntersectionSelected(intersectionIndex))
            {
                road.endIntersectionId.clear();
                disconnectedAny = true;
            }
        }
    }

    return disconnectedAny;
}

bool PolylineEditor::CopySelectedRoads()
{
    if (!m_network)
        return false;

    std::vector<int> roadIndices;
    std::vector<int> intersectionIndices;
    CollectSelectedRoadIndices(roadIndices);
    CollectSelectedIntersectionIndices(intersectionIndices);
    if (roadIndices.empty() && intersectionIndices.empty())
    {
        m_statusMessage = u8"コピーする前に道路または交差点を選択してください";
        return false;
    }

    RoadClipboard clipboard;
    XMFLOAT3 anchor = { 0.0f, 0.0f, 0.0f };
    int anchorCount = 0;
    for (int roadIndex : roadIndices)
    {
        if (roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
            continue;

        const Road& road = m_network->roads[roadIndex];
        clipboard.roads.push_back(road);
        for (const RoadPoint& point : road.points)
        {
            anchor.x += point.pos.x;
            anchor.y += point.pos.y;
            anchor.z += point.pos.z;
            ++anchorCount;
        }
    }

    for (int intersectionIndex : intersectionIndices)
    {
        if (intersectionIndex < 0 ||
            intersectionIndex >= static_cast<int>(m_network->intersections.size()))
            continue;

        const Intersection& intersection = m_network->intersections[intersectionIndex];
        clipboard.intersections.push_back(intersection);
        anchor.x += intersection.pos.x;
        anchor.y += intersection.pos.y;
        anchor.z += intersection.pos.z;
        ++anchorCount;
    }

    if (clipboard.roads.empty() && clipboard.intersections.empty())
    {
        m_statusMessage = u8"コピーする対象が選択されていません";
        return false;
    }
    if (anchorCount == 0)
    {
        m_statusMessage = u8"選択した項目にコピー可能な位置情報がありません";
        return false;
    }

    const float invPointCount = 1.0f / static_cast<float>(anchorCount);
    clipboard.anchor =
    {
        anchor.x * invPointCount,
        anchor.y * invPointCount,
        anchor.z * invPointCount
    };
    m_roadClipboard = std::move(clipboard);
    const int copiedCount = static_cast<int>(m_roadClipboard.roads.size() + m_roadClipboard.intersections.size());
    m_statusMessage = copiedCount > 1 ? u8"選択項目をコピーしました" : u8"項目をコピーしました";
    return true;
}

bool PolylineEditor::PasteCopiedRoadsAtCursor()
{
    if (!m_network)
        return false;
    if (m_roadClipboard.roads.empty() && m_roadClipboard.intersections.empty())
    {
        m_statusMessage = u8"貼り付ける前に道路または交差点をコピーしてください";
        return false;
    }
    if (!m_hasCursorPos)
    {
        m_statusMessage = u8"貼り付ける前にカーソルを地面上へ移動してください";
        return false;
    }

    m_network->EnsureDefaultGroup();
    PushUndoState();

    const XMFLOAT3 targetAnchor = m_cursorPos;
    const XMFLOAT3 delta =
    {
        targetAnchor.x - m_roadClipboard.anchor.x,
        targetAnchor.y - m_roadClipboard.anchor.y,
        targetAnchor.z - m_roadClipboard.anchor.z
    };

    std::vector<int> pastedRoads;
    std::vector<int> pastedIntersections;
    pastedRoads.reserve(m_roadClipboard.roads.size());
    pastedIntersections.reserve(m_roadClipboard.intersections.size());
    std::unordered_map<std::string, std::string> intersectionIdMap;

    for (const Intersection& copiedIntersection : m_roadClipboard.intersections)
    {
        const std::string pastedName = copiedIntersection.name.empty()
            ? std::string("Intersection Copy")
            : copiedIntersection.name + " Copy";
        const int newIntersectionIndex = m_network->AddIntersection(copiedIntersection.pos, pastedName);
        if (newIntersectionIndex < 0 ||
            newIntersectionIndex >= static_cast<int>(m_network->intersections.size()))
            continue;

        Intersection& newIntersection = m_network->intersections[newIntersectionIndex];
        newIntersection.name = pastedName;
        newIntersection.groupId =
            FindGroupIndexById(copiedIntersection.groupId) >= 0
            ? copiedIntersection.groupId
            : m_network->intersections[newIntersectionIndex].groupId;
        newIntersection.type = copiedIntersection.type;
        newIntersection.entryDist = copiedIntersection.entryDist;
        newIntersection.pos.x += delta.x;
        newIntersection.pos.z += delta.z;
        if (m_terrain && m_terrain->IsReady())
            newIntersection.pos.y = m_terrain->GetHeightAt(newIntersection.pos.x, newIntersection.pos.z);
        else
            newIntersection.pos.y += delta.y;

        intersectionIdMap[copiedIntersection.id] = newIntersection.id;
        pastedIntersections.push_back(newIntersectionIndex);
    }

    for (const Road& copiedRoad : m_roadClipboard.roads)
    {
        const std::string pastedName = copiedRoad.name.empty()
            ? std::string("Road Copy")
            : copiedRoad.name + " Copy";
        const int newRoadIndex = m_network->AddRoad(pastedName);
        if (newRoadIndex < 0 || newRoadIndex >= static_cast<int>(m_network->roads.size()))
            continue;

        Road& newRoad = m_network->roads[newRoadIndex];
        newRoad.name = pastedName;
        newRoad.groupId =
            FindGroupIndexById(copiedRoad.groupId) >= 0
            ? copiedRoad.groupId
            : m_network->roads[newRoadIndex].groupId;
        newRoad.closed = copiedRoad.closed;
        newRoad.defaultWidthLaneLeft1 = copiedRoad.defaultWidthLaneLeft1;
        newRoad.defaultWidthLaneRight1 = copiedRoad.defaultWidthLaneRight1;
        newRoad.defaultWidthLaneCenter = copiedRoad.defaultWidthLaneCenter;
        newRoad.defaultFriction = copiedRoad.defaultFriction;
        newRoad.defaultTargetSpeed = copiedRoad.defaultTargetSpeed;
        newRoad.verticalCurve = copiedRoad.verticalCurve;
        newRoad.bankAngle = copiedRoad.bankAngle;
        newRoad.laneSections = copiedRoad.laneSections;
        const auto startIt = intersectionIdMap.find(copiedRoad.startIntersectionId);
        const auto endIt = intersectionIdMap.find(copiedRoad.endIntersectionId);
        newRoad.startIntersectionId =
            startIt != intersectionIdMap.end() ? startIt->second : std::string();
        newRoad.endIntersectionId =
            endIt != intersectionIdMap.end() ? endIt->second : std::string();
        newRoad.points.clear();
        newRoad.points.reserve(copiedRoad.points.size());

        for (const RoadPoint& copiedPoint : copiedRoad.points)
        {
            RoadPoint newPoint = copiedPoint;
            newPoint.pos.x += delta.x;
            newPoint.pos.z += delta.z;
            if (m_terrain && m_terrain->IsReady())
                newPoint.pos.y = m_terrain->GetHeightAt(newPoint.pos.x, newPoint.pos.z);
            else
                newPoint.pos.y += delta.y;
            newRoad.points.push_back(newPoint);
        }

        pastedRoads.push_back(newRoadIndex);
    }

    if (pastedRoads.empty() && pastedIntersections.empty())
    {
        m_statusMessage = u8"貼り付けに失敗しました";
        return false;
    }

    m_selectedRoads = pastedRoads;
    m_activeRoad = pastedRoads.empty() ? -1 : pastedRoads.front();
    m_activePoint = -1;
    m_selectedPoints.clear();
    m_selectedIntersections = pastedIntersections;
    m_activeIntersection = pastedIntersections.empty() ? -1 : pastedIntersections.front();
    m_dragging = false;
    m_activeGizmoAxis = GizmoAxis::None;
    m_hoverSnapIntersection = -1;
    if (!pastedRoads.empty())
        SetActiveGroupById(m_network->roads[m_activeRoad].groupId);
    else if (!pastedIntersections.empty())
        SetActiveGroupById(m_network->intersections[m_activeIntersection].groupId);

    const int pastedCount = static_cast<int>(pastedRoads.size() + pastedIntersections.size());
    m_statusMessage = pastedCount > 1 ? u8"選択項目を貼り付けました" : u8"項目を貼り付けました";
    return true;
}

PolylineEditor::EditorSnapshot PolylineEditor::CaptureSnapshot() const
{
    EditorSnapshot snapshot;
    snapshot.network = *m_network;
    snapshot.mode = m_mode;
    snapshot.activeRoad = m_activeRoad;
    snapshot.activePoint = m_activePoint;
    snapshot.selectedRoads = m_selectedRoads;
    snapshot.selectedPoints = m_selectedPoints;
    snapshot.selectedVerticalCurvePoints = m_selectedVerticalCurvePoints;
    snapshot.selectedBankAnglePoints = m_selectedBankAnglePoints;
    snapshot.selectedLaneSectionPoints = m_selectedLaneSectionPoints;
    snapshot.selectedIntersections = m_selectedIntersections;
    snapshot.activeGroup = m_activeGroup;
    snapshot.activeIntersection = m_activeIntersection;
    snapshot.hoverSnapIntersection = m_hoverSnapIntersection;
    snapshot.defaultWidth = m_defaultWidth;
    snapshot.snapToTerrain = m_snapToTerrain;
    snapshot.snapToPoints = m_snapToPoints;
    snapshot.rotateYMode = m_rotateYMode;
    snapshot.scaleXZMode = m_scaleXZMode;
    return snapshot;
}

void PolylineEditor::RestoreSnapshot(const EditorSnapshot& snapshot)
{
    *m_network = snapshot.network;
    InvalidateAllPreviewCaches();
    m_mode = snapshot.mode;
    m_activeRoad = snapshot.activeRoad;
    m_activePoint = snapshot.activePoint;
    m_selectedRoads = snapshot.selectedRoads;
    m_selectedPoints = snapshot.selectedPoints;
    m_selectedVerticalCurvePoints = snapshot.selectedVerticalCurvePoints;
    m_selectedBankAnglePoints = snapshot.selectedBankAnglePoints;
    m_selectedLaneSectionPoints = snapshot.selectedLaneSectionPoints;
    m_selectedIntersections = snapshot.selectedIntersections;
    m_activeGroup = snapshot.activeGroup;
    m_activeIntersection = snapshot.activeIntersection;
    m_hoverSnapIntersection = snapshot.hoverSnapIntersection;
    m_defaultWidth = snapshot.defaultWidth;
    m_snapToTerrain = snapshot.snapToTerrain;
    m_snapToPoints = snapshot.snapToPoints;
    m_rotateYMode = snapshot.rotateYMode;
    m_scaleXZMode = snapshot.scaleXZMode;

    m_dragging = false;
    m_activeGizmoAxis = GizmoAxis::None;
    m_verticalCurveDragging = false;
    m_verticalCurveDragRoad = -1;
    m_verticalCurveDragPoint = -1;
    m_bankAngleDragging = false;
    m_bankAngleDragRoad = -1;
    m_bankAngleDragPoint = -1;
    m_laneSectionDragging = false;
    m_laneSectionDragRoad = -1;
    m_laneSectionDragPoint = -1;
    m_pointDragStartPositions.clear();
    m_intersectionDragStartPositions.clear();
    m_marqueeSelecting = false;
    SanitizeSelection();
}

void PolylineEditor::PushUndoState()
{
    if (!m_network)
        return;

    if (m_undoStack.size() >= kMaxUndoStates)
        m_undoStack.erase(m_undoStack.begin());

    m_undoStack.push_back(CaptureSnapshot());
    m_redoStack.clear();
}

void PolylineEditor::Undo()
{
    if (m_undoStack.empty() || !m_network)
        return;

    m_redoStack.push_back(CaptureSnapshot());
    RestoreSnapshot(m_undoStack.back());
    m_undoStack.pop_back();
    m_statusMessage = u8"元に戻しました";
}

void PolylineEditor::Redo()
{
    if (m_redoStack.empty() || !m_network)
        return;

    m_undoStack.push_back(CaptureSnapshot());
    RestoreSnapshot(m_redoStack.back());
    m_redoStack.pop_back();
    m_statusMessage = u8"やり直しました";
}

void PolylineEditor::ClearHistory()
{
    m_undoStack.clear();
    m_redoStack.clear();
}

void PolylineEditor::QueuePropertyRevealForRoad(int roadIndex)
{
    m_propertyRevealRoad = roadIndex;
    m_propertyRevealIntersection = -1;
    m_propertyRevealGroup = -1;
    if (roadIndex >= 0 &&
        roadIndex < static_cast<int>(m_network->roads.size()))
    {
        m_propertyRevealGroup = FindGroupIndexById(m_network->roads[roadIndex].groupId);
    }
}

void PolylineEditor::QueuePropertyRevealForIntersection(int intersectionIndex)
{
    m_propertyRevealRoad = -1;
    m_propertyRevealIntersection = intersectionIndex;
    m_propertyRevealGroup = -1;
    if (intersectionIndex >= 0 &&
        intersectionIndex < static_cast<int>(m_network->intersections.size()))
    {
        m_propertyRevealGroup = FindGroupIndexById(m_network->intersections[intersectionIndex].groupId);
    }
}

void PolylineEditor::ClearPropertyReveal()
{
    m_propertyRevealRoad = -1;
    m_propertyRevealIntersection = -1;
    m_propertyRevealGroup = -1;
}

void PolylineEditor::ResetState()
{
    m_mode = EditorMode::Navigate;
    m_activeRoad = -1;
    m_activePoint = -1;
    m_selectedRoads.clear();
    m_selectedPoints.clear();
    m_selectedVerticalCurvePoints.clear();
    m_selectedBankAnglePoints.clear();
    m_selectedLaneSectionPoints.clear();
    m_selectedIntersections.clear();
    m_verticalCurveDragging = false;
    m_verticalCurveDragRoad = -1;
    m_verticalCurveDragPoint = -1;
    m_bankAngleDragging = false;
    m_bankAngleDragRoad = -1;
    m_bankAngleDragPoint = -1;
    m_laneSectionDragging = false;
    m_laneSectionDragRoad = -1;
    m_laneSectionDragPoint = -1;
    m_activeGroup = -1;
    m_activeIntersection = -1;
    m_hoverSnapIntersection = -1;
    m_dragging = false;
    m_activeGizmoAxis = GizmoAxis::None;
    m_dragOffset = { 0, 0, 0 };
    m_axisDragStartPos = { 0, 0, 0 };
    m_axisDragStartMouse = { 0, 0 };
    m_planeDragStartHit = { 0, 0, 0 };
    m_planeDragNormal = { 0, 0, 1 };
    m_pointDragStartPositions.clear();
    m_intersectionDragStartPositions.clear();
    m_marqueeSelecting = false;
    m_marqueeStart = { 0, 0 };
    m_marqueeEnd = { 0, 0 };
    m_marqueeAdditive = false;
    m_marqueeSubtractive = false;
    m_hasCursorPos = false;
    m_cursorPos = { 0, 0, 0 };
    m_prevLButton = false;
    m_prevWKey = false;
    m_prevEKey = false;
    m_prevRKey = false;
    m_prevVKey = false;
    m_prevUndoShortcut = false;
    m_prevRedoShortcut = false;
    m_prevCopyShortcut = false;
    m_prevPasteShortcut = false;
    m_rotateYMode = false;
    m_scaleXZMode = false;
    m_defaultWidth = 3.0f;
    m_snapToTerrain = true;
    m_snapToPoints = false;
    m_statusMessage.clear();
    ClearHistory();
}

void PolylineEditor::SetShowRoadGuidelines(bool show)
{
    m_showRoadGuidelines = show;
    if (m_showRoadGuidelines)
        return;

    if (m_mode == EditorMode::PolylineDraw)
        CancelRoad();
    m_mode = EditorMode::Navigate;
    ClearRoadSelection();
    ClearPointSelection();
    m_activeRoad = -1;
    m_activePoint = -1;
    m_dragging = false;
    m_activeGizmoAxis = GizmoAxis::None;
    m_hoverSnapIntersection = -1;
    m_rotateYMode = false;
    m_scaleXZMode = false;
    m_verticalCurveDragging = false;
    m_verticalCurveDragRoad = -1;
    m_verticalCurveDragPoint = -1;
    m_bankAngleDragging = false;
    m_bankAngleDragRoad = -1;
    m_bankAngleDragPoint = -1;
    m_laneSectionDragging = false;
    m_laneSectionDragRoad = -1;
    m_laneSectionDragPoint = -1;
    m_pointDragStartPositions.clear();
    m_intersectionDragStartPositions.clear();
}

void PolylineEditor::SanitizeSelection()
{
    if (m_activeGroup < 0 || m_activeGroup >= static_cast<int>(m_network->groups.size()))
        m_activeGroup = m_network->groups.empty() ? -1 : 0;

    m_selectedRoads.erase(
        std::remove_if(
            m_selectedRoads.begin(),
            m_selectedRoads.end(),
            [this](int roadIndex)
            {
                return roadIndex < 0 ||
                    roadIndex >= static_cast<int>(m_network->roads.size());
            }),
        m_selectedRoads.end());

    m_selectedPoints.erase(
        std::remove_if(
            m_selectedPoints.begin(),
            m_selectedPoints.end(),
            [this](const PointRef& pointRef)
            {
                if (pointRef.roadIndex < 0 ||
                    pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                    return true;

                const int pointCount = static_cast<int>(
                    m_network->roads[pointRef.roadIndex].points.size());
                return pointRef.pointIndex < 0 || pointRef.pointIndex >= pointCount;
            }),
        m_selectedPoints.end());

    m_selectedVerticalCurvePoints.erase(
        std::remove_if(
            m_selectedVerticalCurvePoints.begin(),
            m_selectedVerticalCurvePoints.end(),
            [this](const VerticalCurveRef& curveRef)
            {
                if (curveRef.roadIndex < 0 ||
                    curveRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                {
                    return true;
                }

                const int curveCount = static_cast<int>(
                    m_network->roads[curveRef.roadIndex].verticalCurve.size());
                return curveRef.curveIndex < 0 || curveRef.curveIndex >= curveCount;
            }),
        m_selectedVerticalCurvePoints.end());

    m_selectedBankAnglePoints.erase(
        std::remove_if(
            m_selectedBankAnglePoints.begin(),
            m_selectedBankAnglePoints.end(),
            [this](const BankAngleRef& bankRef)
            {
                if (bankRef.roadIndex < 0 ||
                    bankRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                {
                    return true;
                }

                const int bankCount = static_cast<int>(
                    m_network->roads[bankRef.roadIndex].bankAngle.size());
                return bankRef.pointIndex < 0 || bankRef.pointIndex >= bankCount;
            }),
        m_selectedBankAnglePoints.end());

    m_selectedLaneSectionPoints.erase(
        std::remove_if(
            m_selectedLaneSectionPoints.begin(),
            m_selectedLaneSectionPoints.end(),
            [this](const LaneSectionRef& laneRef)
            {
                if (laneRef.roadIndex < 0 ||
                    laneRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                {
                    return true;
                }

                const int laneCount = static_cast<int>(
                    m_network->roads[laneRef.roadIndex].laneSections.size());
                return laneRef.pointIndex < 0 || laneRef.pointIndex >= laneCount;
            }),
        m_selectedLaneSectionPoints.end());

    PointRef primaryPoint;
    if (GetPrimarySelectedPoint(primaryPoint))
    {
        m_activeRoad = primaryPoint.roadIndex;
        m_activePoint = primaryPoint.pointIndex;
    }
    else
    {
        m_activePoint = -1;
        if (!m_selectedVerticalCurvePoints.empty())
        {
            const VerticalCurveRef& primaryCurve = m_selectedVerticalCurvePoints.front();
            m_activeRoad = primaryCurve.roadIndex;
        }
        else if (!m_selectedBankAnglePoints.empty())
        {
            const BankAngleRef& primaryBank = m_selectedBankAnglePoints.front();
            m_activeRoad = primaryBank.roadIndex;
        }
        else if (!m_selectedLaneSectionPoints.empty())
        {
            const LaneSectionRef& primaryLane = m_selectedLaneSectionPoints.front();
            m_activeRoad = primaryLane.roadIndex;
        }
        const bool hasCurveSelection =
            !m_selectedVerticalCurvePoints.empty() ||
            !m_selectedBankAnglePoints.empty() ||
            !m_selectedLaneSectionPoints.empty();
        if (m_activeRoad < 0 || m_activeRoad >= static_cast<int>(m_network->roads.size()) ||
            (!hasCurveSelection && !IsRoadSelected(m_activeRoad)))
        {
            m_activeRoad = m_selectedRoads.empty() ? -1 : m_selectedRoads.front();
        }
    }

    if (m_activeIntersection < 0 ||
        m_activeIntersection >= static_cast<int>(m_network->intersections.size()))
    {
        m_activeIntersection = -1;
    }
    m_selectedIntersections.erase(
        std::remove_if(
            m_selectedIntersections.begin(),
            m_selectedIntersections.end(),
            [this](int intersectionIndex)
            {
                return intersectionIndex < 0 ||
                    intersectionIndex >= static_cast<int>(m_network->intersections.size());
            }),
        m_selectedIntersections.end());
    if (m_activeIntersection >= 0 && !IsIntersectionSelected(m_activeIntersection))
        m_activeIntersection = m_selectedIntersections.empty() ? -1 : m_selectedIntersections.front();

    if (m_hoverSnapIntersection < 0 ||
        m_hoverSnapIntersection >= static_cast<int>(m_network->intersections.size()))
    {
        m_hoverSnapIntersection = -1;
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static float Dist2D(XMFLOAT2 a, XMFLOAT2 b)
{
    float dx = a.x - b.x, dy = a.y - b.y;
    return sqrtf(dx * dx + dy * dy);
}

static float DistPointToSegment2D(XMFLOAT2 p, XMFLOAT2 a, XMFLOAT2 b)
{
    float abx = b.x - a.x;
    float aby = b.y - a.y;
    float ab2 = abx * abx + aby * aby;
    if (ab2 <= 1e-6f)
        return Dist2D(p, a);

    float apx = p.x - a.x;
    float apy = p.y - a.y;
    float t = (apx * abx + apy * aby) / ab2;
    t = std::clamp(t, 0.0f, 1.0f);
    XMFLOAT2 q = { a.x + abx * t, a.y + aby * t };
    return Dist2D(p, q);
}

static XMFLOAT3 AxisDirection(PolylineEditor::GizmoAxis axis)
{
    switch (axis)
    {
    case PolylineEditor::GizmoAxis::Center: return { 0.0f, 0.0f, 0.0f };
    case PolylineEditor::GizmoAxis::X: return { 1.0f, 0.0f, 0.0f };
    case PolylineEditor::GizmoAxis::Y: return { 0.0f, 1.0f, 0.0f };
    case PolylineEditor::GizmoAxis::Z: return { 0.0f, 0.0f, 1.0f };
    default:                           return { 0.0f, 0.0f, 0.0f };
    }
}

static float NormalizeAngleDelta(float angle)
{
    while (angle > XM_PI)
        angle -= XM_2PI;
    while (angle < -XM_PI)
        angle += XM_2PI;
    return angle;
}

static XMFLOAT3 RotateAroundY(XMFLOAT3 point, XMFLOAT3 pivot, float radians)
{
    const float dx = point.x - pivot.x;
    const float dz = point.z - pivot.z;
    const float c = cosf(radians);
    const float s = sinf(radians);
    return
    {
        pivot.x + dx * c - dz * s,
        point.y,
        pivot.z + dx * s + dz * c
    };
}

static XMFLOAT3 ScaleAroundXZ(XMFLOAT3 point, XMFLOAT3 pivot, float scale)
{
    return
    {
        pivot.x + (point.x - pivot.x) * scale,
        point.y,
        pivot.z + (point.z - pivot.z) * scale
    };
}

static bool IntersectRayPlane(XMFLOAT3 rayOrigin, XMFLOAT3 rayDir,
                              XMFLOAT3 planePoint, XMFLOAT3 planeNormal,
                              XMFLOAT3& outHit)
{
    const XMVECTOR ro = XMLoadFloat3(&rayOrigin);
    const XMVECTOR rd = XMVector3Normalize(XMLoadFloat3(&rayDir));
    const XMVECTOR pp = XMLoadFloat3(&planePoint);
    const XMVECTOR pn = XMVector3Normalize(XMLoadFloat3(&planeNormal));

    const float denom = XMVectorGetX(XMVector3Dot(rd, pn));
    if (fabsf(denom) < 1e-5f)
        return false;

    const float t = XMVectorGetX(XMVector3Dot(XMVectorSubtract(pp, ro), pn)) / denom;
    if (t < 0.0f)
        return false;

    XMStoreFloat3(&outHit, XMVectorAdd(ro, XMVectorScale(rd, t)));
    return true;
}

// Project a world point to screen pixels (returns {-1,-1} if behind camera)
static XMFLOAT2 WorldToScreen(XMFLOAT3 world, XMMATRIX viewProj,
                               int vpW, int vpH)
{
    XMVECTOR pw  = XMVectorSet(world.x, world.y, world.z, 1.0f);
    XMVECTOR ph  = XMVector4Transform(pw, viewProj);

    float w = XMVectorGetW(ph);
    if (w <= 0.0f)
        return { -1.0f, -1.0f };

    float ndcX =  XMVectorGetX(ph) / w;
    float ndcY =  XMVectorGetY(ph) / w;

    return {
        (ndcX * 0.5f + 0.5f) * vpW,
        (1.0f - (ndcY * 0.5f + 0.5f)) * vpH
    };
}

// ---------------------------------------------------------------------------
// ScreenToRay
// ---------------------------------------------------------------------------
void PolylineEditor::ScreenToRay(int vpW, int vpH, XMFLOAT2 px,
                                  XMMATRIX invVP,
                                  XMFLOAT3& outOrigin,
                                  XMFLOAT3& outDir) const
{
    // NDC [-1,1]
    float ndcX =  (px.x / vpW) * 2.0f - 1.0f;
    float ndcY = -((px.y / vpH) * 2.0f - 1.0f);

    XMVECTOR nearH = XMVector4Transform(
        XMVectorSet(ndcX, ndcY, 0.0f, 1.0f), invVP);
    XMVECTOR farH  = XMVector4Transform(
        XMVectorSet(ndcX, ndcY, 1.0f, 1.0f), invVP);

    XMVECTOR nearW = XMVectorScale(nearH, 1.0f / XMVectorGetW(nearH));
    XMVECTOR farW  = XMVectorScale(farH,  1.0f / XMVectorGetW(farH));

    XMVECTOR dir   = XMVector3Normalize(XMVectorSubtract(farW, nearW));

    XMStoreFloat3(&outOrigin, nearW);
    XMStoreFloat3(&outDir,    dir);
}

// ---------------------------------------------------------------------------
// FindNearestPoint
// ---------------------------------------------------------------------------
void PolylineEditor::FindNearestPoint(int vpW, int vpH, XMFLOAT2 px,
                                       XMMATRIX viewProj,
                                       int& outRoad, int& outPt) const
{
    outRoad = outPt = -1;
    const float threshold = 12.0f; // pixels
    float bestDist = threshold;

    for (int r = 0; r < static_cast<int>(m_network->roads.size()); ++r)
    {
        const Road& road = m_network->roads[r];
        if (!IsRoadGuidelineVisible(road))
            continue;
        for (int p = 0; p < static_cast<int>(road.points.size()); ++p)
        {
            XMFLOAT2 sp = WorldToScreen(road.points[p].pos, viewProj, vpW, vpH);
            if (sp.x < 0)
                continue;
            float d = Dist2D(px, sp);
            if (d < bestDist)
            {
                bestDist = d;
                outRoad  = r;
                outPt    = p;
            }
        }
    }
}

int PolylineEditor::FindNearestPointOnRoad(
    int roadIndex, int vpW, int vpH, XMFLOAT2 px, XMMATRIX viewProj) const
{
    if (roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return -1;

    const Road& road = m_network->roads[roadIndex];
    if (!IsRoadGuidelineVisible(road))
        return -1;
    const float threshold = 12.0f;
    float bestDist = threshold;
    int bestPt = -1;

    for (int p = 0; p < static_cast<int>(road.points.size()); ++p)
    {
        const XMFLOAT2 sp = WorldToScreen(road.points[p].pos, viewProj, vpW, vpH);
        if (sp.x < 0.0f)
            continue;

        const float d = Dist2D(px, sp);
        if (d < bestDist)
        {
            bestDist = d;
            bestPt = p;
        }
    }

    return bestPt;
}

int PolylineEditor::FindNearestSegmentOnRoad(
    int roadIndex, int vpW, int vpH, XMFLOAT2 px, XMMATRIX viewProj) const
{
    if (roadIndex < 0 || roadIndex >= static_cast<int>(m_network->roads.size()))
        return -1;

    const Road& road = m_network->roads[roadIndex];
    if (!IsRoadGuidelineVisible(road))
        return -1;
    const float threshold = 10.0f;
    float bestDist = threshold;
    int bestSegment = -1;

    for (int p = 0; p + 1 < static_cast<int>(road.points.size()); ++p)
    {
        const XMFLOAT2 a = WorldToScreen(road.points[p].pos, viewProj, vpW, vpH);
        const XMFLOAT2 b = WorldToScreen(road.points[p + 1].pos, viewProj, vpW, vpH);
        if (a.x < 0.0f || b.x < 0.0f)
            continue;

        const float d = DistPointToSegment2D(px, a, b);
        if (d < bestDist)
        {
            bestDist = d;
            bestSegment = p;
        }
    }

    return bestSegment;
}

int PolylineEditor::FindNearestRoad(
    int vpW, int vpH, XMFLOAT2 px, XMMATRIX viewProj) const
{
    const float threshold = 10.0f;
    float bestDist = threshold;
    int bestRoad = -1;

    for (int r = 0; r < static_cast<int>(m_network->roads.size()); ++r)
    {
        const Road& road = m_network->roads[r];
        if (!(IsParametricEditModeHelper(m_mode) ? IsRoadVisible(road) : IsRoadGuidelineVisible(road)))
            continue;
        for (int p = 0; p + 1 < static_cast<int>(road.points.size()); ++p)
        {
            const XMFLOAT2 a = WorldToScreen(road.points[p].pos, viewProj, vpW, vpH);
            const XMFLOAT2 b = WorldToScreen(road.points[p + 1].pos, viewProj, vpW, vpH);
            if (a.x < 0.0f || b.x < 0.0f)
                continue;

            const float d = DistPointToSegment2D(px, a, b);
            if (d < bestDist)
            {
                bestDist = d;
                bestRoad = r;
            }
        }
    }

    return bestRoad;
}

bool PolylineEditor::FindNearestPreviewCurveLocation(
    int vpW, int vpH, XMFLOAT2 px, XMMATRIX viewProj,
    int& outRoadIndex, float& outUCoord) const
{
    outRoadIndex = -1;
    outUCoord = 0.0f;

    const float threshold = 14.0f;
    float bestDist = threshold;

    for (int roadIndex = 0; roadIndex < static_cast<int>(m_network->roads.size()); ++roadIndex)
    {
        const Road& road = m_network->roads[roadIndex];
        if (!IsRoadVisible(road))
            continue;

        const std::vector<XMFLOAT3>& previewCurve = GetRoadParametricPreviewCurveCached(roadIndex);
        if (previewCurve.size() < 2)
            continue;

        const std::vector<float>& cumulativeLengths = GetRoadParametricPreviewCurveArcLengthsCached(roadIndex);
        const float totalLength = cumulativeLengths.empty() ? 0.0f : cumulativeLengths.back();
        if (totalLength <= 1e-5f)
            continue;

        for (int pointIndex = 0; pointIndex + 1 < static_cast<int>(previewCurve.size()); ++pointIndex)
        {
            const XMFLOAT2 a = WorldToScreen(previewCurve[pointIndex], viewProj, vpW, vpH);
            const XMFLOAT2 b = WorldToScreen(previewCurve[pointIndex + 1], viewProj, vpW, vpH);
            if (a.x < 0.0f || b.x < 0.0f)
                continue;

            const float abx = b.x - a.x;
            const float aby = b.y - a.y;
            const float ab2 = abx * abx + aby * aby;
            if (ab2 <= 1e-6f)
                continue;

            const float apx = px.x - a.x;
            const float apy = px.y - a.y;
            const float segmentT = std::clamp((apx * abx + apy * aby) / ab2, 0.0f, 1.0f);
            const XMFLOAT2 projected =
            {
                a.x + abx * segmentT,
                a.y + aby * segmentT
            };
            const float d = Dist2D(px, projected);
            if (d >= bestDist)
                continue;

            const float segmentLength = cumulativeLengths[pointIndex + 1] - cumulativeLengths[pointIndex];
            const float sampleLength = cumulativeLengths[pointIndex] + segmentLength * segmentT;

            bestDist = d;
            outRoadIndex = roadIndex;
            outUCoord = sampleLength / totalLength;
        }
    }

    return outRoadIndex >= 0;
}

int PolylineEditor::FindNearestIntersection(
    int vpW, int vpH, XMFLOAT2 px, XMMATRIX viewProj) const
{
    const float threshold = 14.0f;
    float bestDist = threshold;
    int best = -1;

    for (int i = 0; i < static_cast<int>(m_network->intersections.size()); ++i)
    {
        if (!IsIntersectionVisible(m_network->intersections[i]))
            continue;
        XMFLOAT2 sp = WorldToScreen(m_network->intersections[i].pos, viewProj, vpW, vpH);
        if (sp.x < 0.0f)
            continue;

        const float d = Dist2D(px, sp);
        if (d < bestDist)
        {
            bestDist = d;
            best = i;
        }
    }

    return best;
}

int PolylineEditor::FindIntersectionWithinDistance(XMFLOAT3 pos,
                                                   float maxDistance) const
{
    if (!m_network)
        return -1;

    const float safeMaxDistance = (std::max)(0.0f, maxDistance);
    for (int i = 0; i < static_cast<int>(m_network->intersections.size()); ++i)
    {
        if (DistanceXZ3(m_network->intersections[i].pos, pos) < safeMaxDistance)
            return i;
    }

    return -1;
}

int PolylineEditor::FindGroupIndexById(const std::string& id) const
{
    return m_network ? m_network->FindGroupIndexById(id) : -1;
}

bool PolylineEditor::IsRoadVisible(const Road& road) const
{
    const RoadGroup* group = m_network->FindGroupById(road.groupId);
    return group == nullptr || group->visible;
}

bool PolylineEditor::IsRoadGuidelineVisible(const Road& road) const
{
    if (m_mode == EditorMode::VerticalCurveEdit ||
        m_mode == EditorMode::BankAngleEdit ||
        m_mode == EditorMode::LaneEdit)
        return false;
    return m_showRoadGuidelines && IsRoadVisible(road);
}

namespace
{
bool IsParametricEditModeHelper(EditorMode mode)
{
    return mode == EditorMode::VerticalCurveEdit ||
           mode == EditorMode::BankAngleEdit ||
           mode == EditorMode::LaneEdit;
}
}

bool PolylineEditor::IsIntersectionVisible(const Intersection& intersection) const
{
    const RoadGroup* group = m_network->FindGroupById(intersection.groupId);
    return group == nullptr || group->visible;
}

void PolylineEditor::SetActiveGroupById(const std::string& id)
{
    m_activeGroup = FindGroupIndexById(id);
    if (m_activeGroup < 0 && !m_network->groups.empty())
        m_activeGroup = 0;
}

int PolylineEditor::FindIntersectionIndexById(const std::string& id) const
{
    if (id.empty())
        return -1;

    for (int i = 0; i < static_cast<int>(m_network->intersections.size()); ++i)
    {
        if (m_network->intersections[i].id == id)
            return i;
    }
    return -1;
}

bool PolylineEditor::IsSelectedRoadEndpoint() const
{
    if (m_selectedPoints.size() != 1)
        return false;

    PointRef selectedPoint;
    if (!GetPrimarySelectedPoint(selectedPoint))
        return false;

    const Road& road = m_network->roads[selectedPoint.roadIndex];
    const int last = static_cast<int>(road.points.size()) - 1;
    return selectedPoint.pointIndex == 0 || selectedPoint.pointIndex == last;
}

bool PolylineEditor::GetSelectedRoadConnectionId(std::string& outId) const
{
    if (!IsSelectedRoadEndpoint())
        return false;

    PointRef selectedPoint;
    if (!GetPrimarySelectedPoint(selectedPoint))
        return false;

    const Road& road = m_network->roads[selectedPoint.roadIndex];
    outId = (selectedPoint.pointIndex == 0) ? road.startIntersectionId : road.endIntersectionId;
    return !outId.empty();
}

bool PolylineEditor::AutoCreateIntersections()
{
    return AutoCreateIntersectionsFromEndpoints();
}

bool PolylineEditor::FindPointSnapTarget(XMFLOAT3 movingPos,
                                         XMMATRIX viewProj,
                                         int vpW, int vpH,
                                         const PointRef* movingPoint,
                                         int movingIntersectionIndex,
                                         XMFLOAT3& outTarget) const
{
    constexpr float kSnapThresholdPx = 18.0f;
    const XMFLOAT2 movingScreen = WorldToScreen(movingPos, viewProj, vpW, vpH);
    if (movingScreen.x < 0.0f)
        return false;

    bool found = false;
    float bestDistance = kSnapThresholdPx;

    for (int roadIndex = 0; roadIndex < static_cast<int>(m_network->roads.size()); ++roadIndex)
    {
        const Road& road = m_network->roads[roadIndex];
        if (!IsRoadVisible(road))
            continue;

        for (int pointIndex = 0; pointIndex < static_cast<int>(road.points.size()); ++pointIndex)
        {
            if (movingPoint != nullptr && roadIndex == movingPoint->roadIndex)
                continue;
            if (IsPointSelected(roadIndex, pointIndex))
                continue;

            const XMFLOAT2 candidateScreen = WorldToScreen(road.points[pointIndex].pos, viewProj, vpW, vpH);
            if (candidateScreen.x < 0.0f)
                continue;

            const float distance = Dist2D(movingScreen, candidateScreen);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                outTarget = road.points[pointIndex].pos;
                found = true;
            }
        }
    }

    for (int intersectionIndex = 0; intersectionIndex < static_cast<int>(m_network->intersections.size()); ++intersectionIndex)
    {
        const Intersection& intersection = m_network->intersections[intersectionIndex];
        if (!IsIntersectionVisible(intersection))
            continue;
        if (intersectionIndex == movingIntersectionIndex || IsIntersectionSelected(intersectionIndex))
            continue;

        const XMFLOAT2 candidateScreen = WorldToScreen(intersection.pos, viewProj, vpW, vpH);
        if (candidateScreen.x < 0.0f)
            continue;

        const float distance = Dist2D(movingScreen, candidateScreen);
        if (distance < bestDistance)
        {
            bestDistance = distance;
            outTarget = intersection.pos;
            found = true;
        }
    }

    return found;
}

bool PolylineEditor::AutoCreateIntersectionsFromEndpoints()
{
    if (!m_network)
        return false;

    struct EndpointRef
    {
        int roadIndex = -1;
        int pointIndex = -1;
        XMFLOAT3 pos = { 0.0f, 0.0f, 0.0f };
        std::string groupId;
    };

    constexpr float kEndpointClusterDistance = 5.0f;
    constexpr float kExistingIntersectionDistance = 5.0f;

    std::vector<EndpointRef> endpoints;
    for (int roadIndex = 0; roadIndex < static_cast<int>(m_network->roads.size()); ++roadIndex)
    {
        const Road& road = m_network->roads[roadIndex];
        if (!IsRoadVisible(road) || road.points.size() < 2)
            continue;

        if (road.startIntersectionId.empty())
        {
            endpoints.push_back(
                { roadIndex, 0, road.points.front().pos, road.groupId });
        }

        const int lastPointIndex = static_cast<int>(road.points.size()) - 1;
        if (road.endIntersectionId.empty())
        {
            endpoints.push_back(
                { roadIndex, lastPointIndex, road.points.back().pos, road.groupId });
        }
    }

    if (endpoints.size() < 2)
    {
        m_statusMessage = u8"重なっている道路端点は見つかりませんでした";
        return false;
    }

    std::vector<int> clusterAssignment(endpoints.size(), -1);
    std::vector<std::vector<int>> clusters;
    for (size_t seedIndex = 0; seedIndex < endpoints.size(); ++seedIndex)
    {
        if (clusterAssignment[seedIndex] >= 0)
            continue;

        const int clusterIndex = static_cast<int>(clusters.size());
        clusters.push_back({});
        clusterAssignment[seedIndex] = clusterIndex;
        clusters.back().push_back(static_cast<int>(seedIndex));

        bool expanded = true;
        while (expanded)
        {
            expanded = false;
            for (size_t candidateIndex = 0; candidateIndex < endpoints.size(); ++candidateIndex)
            {
                if (clusterAssignment[candidateIndex] >= 0)
                    continue;

                for (int memberIndex : clusters.back())
                {
                    if (Distance3(endpoints[candidateIndex].pos, endpoints[memberIndex].pos) <= kEndpointClusterDistance)
                    {
                        clusterAssignment[candidateIndex] = clusterIndex;
                        clusters.back().push_back(static_cast<int>(candidateIndex));
                        expanded = true;
                        break;
                    }
                }
            }
        }
    }

    struct PlannedIntersection
    {
        XMFLOAT3 pos = { 0.0f, 0.0f, 0.0f };
        std::string groupId;
        std::vector<int> endpointIndices;
    };

    std::vector<PlannedIntersection> planned;
    for (const std::vector<int>& cluster : clusters)
    {
        if (cluster.size() < 2)
            continue;

        XMFLOAT3 centroid = { 0.0f, 0.0f, 0.0f };
        for (int endpointIndex : cluster)
        {
            centroid.x += endpoints[endpointIndex].pos.x;
            centroid.y += endpoints[endpointIndex].pos.y;
            centroid.z += endpoints[endpointIndex].pos.z;
        }
        const float invCount = 1.0f / static_cast<float>(cluster.size());
        centroid.x *= invCount;
        centroid.y *= invCount;
        centroid.z *= invCount;

        bool hasExistingIntersection = false;
        for (const Intersection& intersection : m_network->intersections)
        {
            if (Distance3(intersection.pos, centroid) <= kExistingIntersectionDistance)
            {
                hasExistingIntersection = true;
                break;
            }
        }
        if (hasExistingIntersection)
            continue;

        PlannedIntersection plan;
        plan.pos = centroid;
        plan.endpointIndices = cluster;
        plan.groupId = endpoints[cluster.front()].groupId;
        planned.push_back(std::move(plan));
    }

    if (planned.empty())
    {
        m_statusMessage = u8"新しい交差点を作成する必要はありませんでした";
        return false;
    }

    PushUndoState();
    m_network->EnsureDefaultGroup();

    int createdCount = 0;
    for (const PlannedIntersection& plan : planned)
    {
        const int intersectionIndex = m_network->AddIntersection(plan.pos, "Intersection");
        if (intersectionIndex < 0 ||
            intersectionIndex >= static_cast<int>(m_network->intersections.size()))
        {
            continue;
        }

        Intersection& intersection = m_network->intersections[intersectionIndex];
        if (!plan.groupId.empty())
            intersection.groupId = plan.groupId;
        intersection.pos = plan.pos;

        for (int endpointIndex : plan.endpointIndices)
        {
            const EndpointRef& endpoint = endpoints[endpointIndex];
            if (endpoint.roadIndex < 0 ||
                endpoint.roadIndex >= static_cast<int>(m_network->roads.size()))
            {
                continue;
            }

            Road& road = m_network->roads[endpoint.roadIndex];
            if (endpoint.pointIndex < 0 ||
                endpoint.pointIndex >= static_cast<int>(road.points.size()))
            {
                continue;
            }

            road.points[endpoint.pointIndex].pos = intersection.pos;
            if (endpoint.pointIndex == 0)
                road.startIntersectionId = intersection.id;
            else if (endpoint.pointIndex == static_cast<int>(road.points.size()) - 1)
                road.endIntersectionId = intersection.id;
        }

        ++createdCount;
    }

    if (createdCount <= 0)
    {
        m_statusMessage = u8"交差点の作成に失敗しました";
        return false;
    }

    InvalidateAllPreviewCaches();
    m_statusMessage = createdCount == 1
        ? u8"交差点を1件作成しました"
        : std::to_string(createdCount) + u8" 件の交差点を作成しました";
    return true;
}

bool PolylineEditor::ConnectSelectedIntersectionsWithRoad()
{
    if (!m_network)
        return false;

    std::vector<int> intersectionIndices;
    CollectSelectedIntersectionIndices(intersectionIndices);
    if (intersectionIndices.size() != 2)
    {
        m_statusMessage = u8"交差点をちょうど2つ選択してください";
        return false;
    }

    const int firstIndex = intersectionIndices[0];
    const int secondIndex = intersectionIndices[1];
    if (firstIndex < 0 ||
        firstIndex >= static_cast<int>(m_network->intersections.size()) ||
        secondIndex < 0 ||
        secondIndex >= static_cast<int>(m_network->intersections.size()) ||
        firstIndex == secondIndex)
    {
        m_statusMessage = u8"交差点をちょうど2つ選択してください";
        return false;
    }

    const Intersection& firstIntersection = m_network->intersections[firstIndex];
    const Intersection& secondIntersection = m_network->intersections[secondIndex];
    if (Distance3(firstIntersection.pos, secondIntersection.pos) <= 1e-4f)
    {
        m_statusMessage = u8"選択した交差点同士が近すぎて接続できません";
        return false;
    }

    PushUndoState();
    m_network->EnsureDefaultGroup();

    const int newRoadIndex = m_network->AddRoad(
        "Road " + std::to_string(m_network->roads.size()));
    if (newRoadIndex < 0 || newRoadIndex >= static_cast<int>(m_network->roads.size()))
    {
        m_statusMessage = u8"道路の作成に失敗しました";
        return false;
    }

    Road& newRoad = m_network->roads[newRoadIndex];
    newRoad.groupId = !firstIntersection.groupId.empty()
        ? firstIntersection.groupId
        : newRoad.groupId;
    newRoad.startIntersectionId = firstIntersection.id;
    newRoad.endIntersectionId = secondIntersection.id;

    RoadPoint startPoint;
    startPoint.pos = firstIntersection.pos;
    startPoint.width = m_defaultWidth;
    RoadPoint endPoint;
    endPoint.pos = secondIntersection.pos;
    endPoint.width = m_defaultWidth;
    newRoad.points.push_back(startPoint);
    newRoad.points.push_back(endPoint);

    SelectSingleRoad(newRoadIndex);
    ClearPointSelection();
    ClearIntersectionSelection();
    m_activeIntersection = -1;
    m_activeGizmoAxis = GizmoAxis::None;
    m_hoverSnapIntersection = -1;
    m_dragging = false;
    InvalidateRoadPreviewCache(newRoadIndex);
    m_statusMessage = u8"交差点間に道路を作成しました";
    return true;
}

void PolylineEditor::SetSelectedRoadConnectionId(const std::string& intersectionId)
{
    if (!IsSelectedRoadEndpoint())
        return;

    PointRef selectedPoint;
    if (!GetPrimarySelectedPoint(selectedPoint))
        return;

    Road& road = m_network->roads[selectedPoint.roadIndex];
    if (selectedPoint.pointIndex == 0)
        road.startIntersectionId = intersectionId;
    else
        road.endIntersectionId = intersectionId;
}

void PolylineEditor::ClearSelectedRoadConnection()
{
    SetSelectedRoadConnectionId("");
}

bool PolylineEditor::SplitSelectedRoadAtPoint()
{
    if (!m_network || m_selectedPoints.size() != 1)
        return false;

    PointRef selectedPoint;
    if (!GetPrimarySelectedPoint(selectedPoint))
        return false;
    if (selectedPoint.roadIndex < 0 ||
        selectedPoint.roadIndex >= static_cast<int>(m_network->roads.size()))
        return false;

    const int sourceRoadIndex = selectedPoint.roadIndex;
    const Road& sourceRoad = m_network->roads[sourceRoadIndex];
    const int splitIndex = selectedPoint.pointIndex;
    if (splitIndex <= 0 || splitIndex >= static_cast<int>(sourceRoad.points.size()) - 1)
    {
        m_statusMessage = u8"道路を分割するには中間点を選択してください";
        return false;
    }

    PushUndoState();

    const XMFLOAT3 splitPos = sourceRoad.points[splitIndex].pos;
    const std::string originalStartIntersectionId = sourceRoad.startIntersectionId;
    const std::string originalEndIntersectionId = sourceRoad.endIntersectionId;
    const std::string originalName = sourceRoad.name;
    const std::string originalGroupId = sourceRoad.groupId;
    const bool originalClosed = sourceRoad.closed;
    const float originalDefaultWidthLaneLeft1 = sourceRoad.defaultWidthLaneLeft1;
    const float originalDefaultWidthLaneRight1 = sourceRoad.defaultWidthLaneRight1;
    const float originalDefaultWidthLaneCenter = sourceRoad.defaultWidthLaneCenter;
    const float originalDefaultFriction = sourceRoad.defaultFriction;
    const float originalDefaultTargetSpeed = sourceRoad.defaultTargetSpeed;
    const std::vector<RoadPoint> originalPoints = sourceRoad.points;

    const int newIntersectionIndex = m_network->AddIntersection(splitPos, "Intersection");
    if (newIntersectionIndex < 0 ||
        newIntersectionIndex >= static_cast<int>(m_network->intersections.size()))
        return false;

    Intersection& splitIntersection = m_network->intersections[newIntersectionIndex];
    splitIntersection.groupId = originalGroupId;
    splitIntersection.pos = splitPos;

    const std::string splitIntersectionId = splitIntersection.id;

    const int newRoadIndex = m_network->AddRoad(originalName + " B");
    if (newRoadIndex < 0 || newRoadIndex >= static_cast<int>(m_network->roads.size()))
        return false;

    Road& newRoad = m_network->roads[newRoadIndex];
    newRoad.groupId = originalGroupId;
    newRoad.closed = false;
    newRoad.startIntersectionId = splitIntersectionId;
    newRoad.endIntersectionId = originalEndIntersectionId;
    newRoad.defaultWidthLaneLeft1 = originalDefaultWidthLaneLeft1;
    newRoad.defaultWidthLaneRight1 = originalDefaultWidthLaneRight1;
    newRoad.defaultWidthLaneCenter = originalDefaultWidthLaneCenter;
    newRoad.defaultFriction = originalDefaultFriction;
    newRoad.defaultTargetSpeed = originalDefaultTargetSpeed;
    newRoad.points.assign(originalPoints.begin() + splitIndex, originalPoints.end());
    newRoad.points.front().pos = splitPos;

    Road& updatedSourceRoad = m_network->roads[sourceRoadIndex];
    updatedSourceRoad.groupId = originalGroupId;
    updatedSourceRoad.name = originalName + " A";
    updatedSourceRoad.startIntersectionId = originalStartIntersectionId;
    updatedSourceRoad.closed = false;
    updatedSourceRoad.endIntersectionId = splitIntersectionId;
    updatedSourceRoad.defaultWidthLaneLeft1 = originalDefaultWidthLaneLeft1;
    updatedSourceRoad.defaultWidthLaneRight1 = originalDefaultWidthLaneRight1;
    updatedSourceRoad.defaultWidthLaneCenter = originalDefaultWidthLaneCenter;
    updatedSourceRoad.defaultFriction = originalDefaultFriction;
    updatedSourceRoad.defaultTargetSpeed = originalDefaultTargetSpeed;
    updatedSourceRoad.points.assign(originalPoints.begin(), originalPoints.begin() + splitIndex + 1);
    updatedSourceRoad.points.back().pos = splitPos;

    m_activeRoad = sourceRoadIndex;
    ClearPointSelection();
    SelectSingleIntersection(newIntersectionIndex);
    m_activeGizmoAxis = GizmoAxis::None;
    m_hoverSnapIntersection = -1;
    m_dragging = false;
    m_statusMessage = originalClosed
        ? u8"閉じた道路を2本の道路に分割しました"
        : u8"道路を分割して新しい交差点を作成しました";
    return true;
}

bool PolylineEditor::MergeSelectedRoads()
{
    constexpr float kMergeEndpointDistance = 5.0f;

    if (!m_network || m_selectedRoads.size() != 2)
        return false;

    const int roadAIndex = m_selectedRoads[0];
    const int roadBIndex = m_selectedRoads[1];
    if (roadAIndex < 0 || roadAIndex >= static_cast<int>(m_network->roads.size()) ||
        roadBIndex < 0 || roadBIndex >= static_cast<int>(m_network->roads.size()) ||
        roadAIndex == roadBIndex)
        return false;

    const Road& roadA = m_network->roads[roadAIndex];
    const Road& roadB = m_network->roads[roadBIndex];
    if (roadA.points.size() < 2 || roadB.points.size() < 2)
    {
        m_statusMessage = u8"結合する両方の道路に少なくとも2点必要です";
        return false;
    }

    auto endpointsMatch = [kMergeEndpointDistance](const std::string& lhsId,
                                                   const std::string& rhsId,
                                                   XMFLOAT3 lhsPos,
                                                   XMFLOAT3 rhsPos)
    {
        if (!lhsId.empty() && !rhsId.empty() && lhsId == rhsId)
            return true;
        return Distance3(lhsPos, rhsPos) <= kMergeEndpointDistance;
    };

    bool reverseA = false;
    bool reverseB = false;
    bool foundConnection = false;
    XMFLOAT3 mergePoint = {};
    if (endpointsMatch(roadA.endIntersectionId, roadB.startIntersectionId, roadA.points.back().pos, roadB.points.front().pos))
    {
        reverseA = false;
        reverseB = false;
        foundConnection = true;
        mergePoint =
        {
            0.5f * (roadA.points.back().pos.x + roadB.points.front().pos.x),
            0.5f * (roadA.points.back().pos.y + roadB.points.front().pos.y),
            0.5f * (roadA.points.back().pos.z + roadB.points.front().pos.z)
        };
    }
    else if (endpointsMatch(roadA.endIntersectionId, roadB.endIntersectionId, roadA.points.back().pos, roadB.points.back().pos))
    {
        reverseA = false;
        reverseB = true;
        foundConnection = true;
        mergePoint =
        {
            0.5f * (roadA.points.back().pos.x + roadB.points.back().pos.x),
            0.5f * (roadA.points.back().pos.y + roadB.points.back().pos.y),
            0.5f * (roadA.points.back().pos.z + roadB.points.back().pos.z)
        };
    }
    else if (endpointsMatch(roadA.startIntersectionId, roadB.startIntersectionId, roadA.points.front().pos, roadB.points.front().pos))
    {
        reverseA = true;
        reverseB = false;
        foundConnection = true;
        mergePoint =
        {
            0.5f * (roadA.points.front().pos.x + roadB.points.front().pos.x),
            0.5f * (roadA.points.front().pos.y + roadB.points.front().pos.y),
            0.5f * (roadA.points.front().pos.z + roadB.points.front().pos.z)
        };
    }
    else if (endpointsMatch(roadA.startIntersectionId, roadB.endIntersectionId, roadA.points.front().pos, roadB.points.back().pos))
    {
        reverseA = true;
        reverseB = true;
        foundConnection = true;
        mergePoint =
        {
            0.5f * (roadA.points.front().pos.x + roadB.points.back().pos.x),
            0.5f * (roadA.points.front().pos.y + roadB.points.back().pos.y),
            0.5f * (roadA.points.front().pos.z + roadB.points.back().pos.z)
        };
    }

    if (!foundConnection)
    {
        m_statusMessage = u8"道路を結合するには端点同士が5m以内にある必要があります";
        return false;
    }

    PushUndoState();

    std::vector<RoadPoint> pointsA = roadA.points;
    std::vector<RoadPoint> pointsB = roadB.points;
    if (reverseA)
        std::reverse(pointsA.begin(), pointsA.end());
    if (reverseB)
        std::reverse(pointsB.begin(), pointsB.end());
    pointsA.back().pos = mergePoint;
    pointsB.front().pos = mergePoint;

    Road mergedRoad = roadA;
    mergedRoad.points = pointsA;
    mergedRoad.points.insert(mergedRoad.points.end(), pointsB.begin() + 1, pointsB.end());
    mergedRoad.startIntersectionId = reverseA ? roadA.endIntersectionId : roadA.startIntersectionId;
    mergedRoad.endIntersectionId = reverseB ? roadB.startIntersectionId : roadB.endIntersectionId;
    mergedRoad.closed = false;

    const int keepIndex = (std::min)(roadAIndex, roadBIndex);
    const int removeIndex = (std::max)(roadAIndex, roadBIndex);
    m_network->roads[keepIndex] = mergedRoad;
    m_network->RemoveRoad(removeIndex);

    SelectSingleRoad(keepIndex);
    ClearPointSelection();
    ClearIntersectionSelection();
    m_activeIntersection = -1;
    m_statusMessage = u8"道路を結合しました";
    return true;
}

int PolylineEditor::FindSnapIntersectionForSelectedEndpoint(
    int vpW, int vpH, XMMATRIX viewProj) const
{
    if (!IsSelectedRoadEndpoint())
        return -1;

    PointRef selectedPoint;
    if (!GetPrimarySelectedPoint(selectedPoint))
        return -1;

    const Road& road = m_network->roads[selectedPoint.roadIndex];
    const XMFLOAT3 pointPos = road.points[selectedPoint.pointIndex].pos;
    const XMFLOAT2 pointScreen = WorldToScreen(pointPos, viewProj, vpW, vpH);
    if (pointScreen.x < 0.0f)
        return -1;

    const float threshold = 16.0f;
    float bestDist = threshold;
    int best = -1;

    for (int i = 0; i < static_cast<int>(m_network->intersections.size()); ++i)
    {
        const XMFLOAT2 isecScreen = WorldToScreen(
            m_network->intersections[i].pos, viewProj, vpW, vpH);
        if (isecScreen.x < 0.0f)
            continue;

        const float d = Dist2D(pointScreen, isecScreen);
        if (d < bestDist)
        {
            bestDist = d;
            best = i;
        }
    }

    return best;
}

void PolylineEditor::SnapSelectedEndpointToIntersection(int intersectionIndex)
{
    if (!IsSelectedRoadEndpoint() ||
        intersectionIndex < 0 ||
        intersectionIndex >= static_cast<int>(m_network->intersections.size()))
        return;

    PointRef selectedPoint;
    if (!GetPrimarySelectedPoint(selectedPoint))
        return;

    Road& road = m_network->roads[selectedPoint.roadIndex];
    road.points[selectedPoint.pointIndex].pos = m_network->intersections[intersectionIndex].pos;
    SetSelectedRoadConnectionId(m_network->intersections[intersectionIndex].id);
}

void PolylineEditor::SyncRoadConnectionsForIntersection(int intersectionIndex)
{
    if (intersectionIndex < 0 ||
        intersectionIndex >= static_cast<int>(m_network->intersections.size()))
        return;

    const Intersection& isec = m_network->intersections[intersectionIndex];
    for (Road& road : m_network->roads)
    {
        if (!road.startIntersectionId.empty() &&
            road.startIntersectionId == isec.id &&
            !road.points.empty())
        {
            road.points.front().pos = isec.pos;
        }
        if (!road.endIntersectionId.empty() &&
            road.endIntersectionId == isec.id &&
            !road.points.empty())
        {
            road.points.back().pos = isec.pos;
        }
    }
}

bool PolylineEditor::GetActiveGizmoPivot(XMFLOAT3& outPivot) const
{
    if (m_mode == EditorMode::PointEdit &&
        (!m_selectedPoints.empty() || !m_selectedIntersections.empty()))
    {
        XMFLOAT3 center = { 0.0f, 0.0f, 0.0f };
        int validCount = 0;
        for (const PointRef& pointRef : m_selectedPoints)
        {
            if (pointRef.roadIndex < 0 ||
                pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                continue;

            const Road& road = m_network->roads[pointRef.roadIndex];
            if (pointRef.pointIndex < 0 || pointRef.pointIndex >= static_cast<int>(road.points.size()))
                continue;

            const XMFLOAT3& pos = road.points[pointRef.pointIndex].pos;
            center.x += pos.x;
            center.y += pos.y;
            center.z += pos.z;
            ++validCount;
        }
        for (int intersectionIndex : m_selectedIntersections)
        {
            if (intersectionIndex < 0 ||
                intersectionIndex >= static_cast<int>(m_network->intersections.size()))
                continue;
            const XMFLOAT3& pos = m_network->intersections[intersectionIndex].pos;
            center.x += pos.x;
            center.y += pos.y;
            center.z += pos.z;
            ++validCount;
        }
        if (validCount > 0)
        {
            const float invCount = 1.0f / static_cast<float>(validCount);
            outPivot = { center.x * invCount, center.y * invCount, center.z * invCount };
            return true;
        }
    }

    if (m_mode == EditorMode::IntersectionEdit &&
        m_activeIntersection >= 0 &&
        m_activeIntersection < static_cast<int>(m_network->intersections.size()))
    {
        outPivot = m_network->intersections[m_activeIntersection].pos;
        return true;
    }

    return false;
}

bool PolylineEditor::IsRoadSelected(int roadIndex) const
{
    return std::find(
        m_selectedRoads.begin(),
        m_selectedRoads.end(),
        roadIndex) != m_selectedRoads.end();
}

bool PolylineEditor::IsPointSelected(int roadIndex, int pointIndex) const
{
    return std::find_if(
        m_selectedPoints.begin(),
        m_selectedPoints.end(),
        [roadIndex, pointIndex](const PointRef& pointRef)
        {
            return pointRef.roadIndex == roadIndex && pointRef.pointIndex == pointIndex;
        }) != m_selectedPoints.end();
}

bool PolylineEditor::GetPrimarySelectedPoint(PointRef& outPoint) const
{
    if (m_selectedPoints.empty())
        return false;

    outPoint = m_selectedPoints.front();
    return true;
}

bool PolylineEditor::IsVerticalCurveSelected(int roadIndex, int curveIndex) const
{
    return std::find_if(
        m_selectedVerticalCurvePoints.begin(),
        m_selectedVerticalCurvePoints.end(),
        [roadIndex, curveIndex](const VerticalCurveRef& curveRef)
        {
            return curveRef.roadIndex == roadIndex && curveRef.curveIndex == curveIndex;
        }) != m_selectedVerticalCurvePoints.end();
}

bool PolylineEditor::GetPrimarySelectedVerticalCurvePoint(VerticalCurveRef& outPoint) const
{
    if (m_selectedVerticalCurvePoints.empty())
        return false;

    outPoint = m_selectedVerticalCurvePoints.front();
    return true;
}

bool PolylineEditor::IsBankAngleSelected(int roadIndex, int pointIndex) const
{
    return std::find_if(
        m_selectedBankAnglePoints.begin(),
        m_selectedBankAnglePoints.end(),
        [roadIndex, pointIndex](const BankAngleRef& bankRef)
        {
            return bankRef.roadIndex == roadIndex && bankRef.pointIndex == pointIndex;
        }) != m_selectedBankAnglePoints.end();
}

bool PolylineEditor::GetPrimarySelectedBankAnglePoint(BankAngleRef& outPoint) const
{
    if (m_selectedBankAnglePoints.empty())
        return false;

    outPoint = m_selectedBankAnglePoints.front();
    return true;
}

bool PolylineEditor::IsLaneSectionSelected(int roadIndex, int pointIndex) const
{
    return std::find_if(
        m_selectedLaneSectionPoints.begin(),
        m_selectedLaneSectionPoints.end(),
        [roadIndex, pointIndex](const LaneSectionRef& laneRef)
        {
            return laneRef.roadIndex == roadIndex && laneRef.pointIndex == pointIndex;
        }) != m_selectedLaneSectionPoints.end();
}

bool PolylineEditor::GetPrimarySelectedLaneSectionPoint(LaneSectionRef& outPoint) const
{
    if (m_selectedLaneSectionPoints.empty())
        return false;

    outPoint = m_selectedLaneSectionPoints.front();
    return true;
}

bool PolylineEditor::IsIntersectionSelected(int intersectionIndex) const
{
    return std::find(
        m_selectedIntersections.begin(),
        m_selectedIntersections.end(),
        intersectionIndex) != m_selectedIntersections.end();
}

void PolylineEditor::ClearRoadSelection()
{
    m_selectedRoads.clear();
    m_activeRoad = -1;
    ClearVerticalCurveSelection();
    ClearBankAngleSelection();
    ClearLaneSectionSelection();
}

void PolylineEditor::SelectSingleRoad(int roadIndex)
{
    m_selectedRoads.clear();
    m_selectedVerticalCurvePoints.clear();
    m_selectedBankAnglePoints.clear();
    m_selectedLaneSectionPoints.clear();
    if (roadIndex >= 0)
        m_selectedRoads.push_back(roadIndex);
    m_activeRoad = roadIndex;
    m_activePoint = -1;
    QueuePropertyRevealForRoad(roadIndex);
}

void PolylineEditor::ToggleRoadSelection(int roadIndex)
{
    if (roadIndex < 0)
        return;

    std::vector<int>::iterator it = std::find(
        m_selectedRoads.begin(),
        m_selectedRoads.end(),
        roadIndex);
    if (it != m_selectedRoads.end())
    {
        m_selectedRoads.erase(it);
        m_selectedVerticalCurvePoints.clear();
        m_selectedBankAnglePoints.clear();
        m_selectedLaneSectionPoints.clear();
        m_activeRoad = m_selectedRoads.empty() ? -1 : m_selectedRoads.front();
        return;
    }

    m_selectedRoads.push_back(roadIndex);
    m_selectedVerticalCurvePoints.clear();
    m_selectedBankAnglePoints.clear();
    m_selectedLaneSectionPoints.clear();
    m_activeRoad = roadIndex;
    m_activePoint = -1;
    QueuePropertyRevealForRoad(roadIndex);
}

void PolylineEditor::ClearPointSelection()
{
    m_selectedPoints.clear();
    m_activePoint = -1;
}

void PolylineEditor::ClearVerticalCurveSelection()
{
    m_selectedVerticalCurvePoints.clear();
}

void PolylineEditor::ClearBankAngleSelection()
{
    m_selectedBankAnglePoints.clear();
}

void PolylineEditor::ClearLaneSectionSelection()
{
    m_selectedLaneSectionPoints.clear();
}

void PolylineEditor::ClearIntersectionSelection()
{
    m_selectedIntersections.clear();
    m_activeIntersection = -1;
}

void PolylineEditor::SelectSinglePoint(int roadIndex, int pointIndex)
{
    m_selectedPoints.clear();
    m_selectedVerticalCurvePoints.clear();
    m_selectedBankAnglePoints.clear();
    m_selectedLaneSectionPoints.clear();
    if (roadIndex >= 0 && pointIndex >= 0)
        m_selectedPoints.push_back({ roadIndex, pointIndex });
    m_activeRoad = roadIndex;
    m_activePoint = pointIndex;
    QueuePropertyRevealForRoad(roadIndex);
}

void PolylineEditor::TogglePointSelection(int roadIndex, int pointIndex)
{
    if (roadIndex < 0 || pointIndex < 0)
        return;

    std::vector<PointRef>::iterator it = std::find_if(
        m_selectedPoints.begin(),
        m_selectedPoints.end(),
        [roadIndex, pointIndex](const PointRef& pointRef)
        {
            return pointRef.roadIndex == roadIndex && pointRef.pointIndex == pointIndex;
        });
    if (it != m_selectedPoints.end())
    {
        m_selectedPoints.erase(it);
        PointRef primaryPoint;
        if (GetPrimarySelectedPoint(primaryPoint))
        {
            m_activeRoad = primaryPoint.roadIndex;
            m_activePoint = primaryPoint.pointIndex;
        }
        else
        {
            m_activePoint = -1;
        }
        return;
    }

    m_selectedVerticalCurvePoints.clear();
    m_selectedBankAnglePoints.clear();
    m_selectedLaneSectionPoints.clear();
    m_selectedPoints.push_back({ roadIndex, pointIndex });
    m_activeRoad = roadIndex;
    m_activePoint = pointIndex;
    QueuePropertyRevealForRoad(roadIndex);
}

void PolylineEditor::SelectSingleVerticalCurvePoint(int roadIndex, int curveIndex)
{
    m_selectedVerticalCurvePoints.clear();
    m_selectedBankAnglePoints.clear();
    m_selectedLaneSectionPoints.clear();
    if (roadIndex >= 0 && curveIndex >= 0)
        m_selectedVerticalCurvePoints.push_back({ roadIndex, curveIndex });
    m_selectedPoints.clear();
    m_selectedIntersections.clear();
    m_activeRoad = roadIndex;
    m_activePoint = -1;
    m_activeIntersection = -1;
    QueuePropertyRevealForRoad(roadIndex);
}

void PolylineEditor::SelectSingleBankAnglePoint(int roadIndex, int pointIndex)
{
    m_selectedBankAnglePoints.clear();
    if (roadIndex >= 0 && pointIndex >= 0)
        m_selectedBankAnglePoints.push_back({ roadIndex, pointIndex });
    m_selectedPoints.clear();
    m_selectedVerticalCurvePoints.clear();
    m_selectedLaneSectionPoints.clear();
    m_selectedIntersections.clear();
    m_activeRoad = roadIndex;
    m_activePoint = -1;
    m_activeIntersection = -1;
    QueuePropertyRevealForRoad(roadIndex);
}

void PolylineEditor::SelectSingleLaneSectionPoint(int roadIndex, int pointIndex)
{
    m_selectedLaneSectionPoints.clear();
    if (roadIndex >= 0 && pointIndex >= 0)
        m_selectedLaneSectionPoints.push_back({ roadIndex, pointIndex });
    m_selectedPoints.clear();
    m_selectedVerticalCurvePoints.clear();
    m_selectedBankAnglePoints.clear();
    m_selectedIntersections.clear();
    m_activeRoad = roadIndex;
    m_activePoint = -1;
    m_activeIntersection = -1;
    QueuePropertyRevealForRoad(roadIndex);
}

void PolylineEditor::SelectSingleIntersection(int intersectionIndex)
{
    m_selectedIntersections.clear();
    m_selectedVerticalCurvePoints.clear();
    m_selectedBankAnglePoints.clear();
    m_selectedLaneSectionPoints.clear();
    if (intersectionIndex >= 0)
        m_selectedIntersections.push_back(intersectionIndex);
    m_activeIntersection = intersectionIndex;
    QueuePropertyRevealForIntersection(intersectionIndex);
}

void PolylineEditor::ToggleIntersectionSelection(int intersectionIndex)
{
    if (intersectionIndex < 0)
        return;

    std::vector<int>::iterator it = std::find(
        m_selectedIntersections.begin(),
        m_selectedIntersections.end(),
        intersectionIndex);
    if (it != m_selectedIntersections.end())
    {
        m_selectedIntersections.erase(it);
        m_activeIntersection = m_selectedIntersections.empty() ? -1 : m_selectedIntersections.front();
        return;
    }

    m_selectedIntersections.push_back(intersectionIndex);
    m_activeIntersection = intersectionIndex;
    QueuePropertyRevealForIntersection(intersectionIndex);
}

void PolylineEditor::ApplyMarqueeSelection(
    int vpW, int vpH, XMMATRIX viewProj, bool addToSelection, bool removeFromSelection)
{
    const float minX = (std::min)(m_marqueeStart.x, m_marqueeEnd.x);
    const float maxX = (std::max)(m_marqueeStart.x, m_marqueeEnd.x);
    const float minY = (std::min)(m_marqueeStart.y, m_marqueeEnd.y);
    const float maxY = (std::max)(m_marqueeStart.y, m_marqueeEnd.y);
    const bool pointEditSelection = (m_mode == EditorMode::PointEdit);

    auto isScreenPointInside = [&](const XMFLOAT2& pointScreen) -> bool
    {
        return pointScreen.x >= minX && pointScreen.x <= maxX &&
               pointScreen.y >= minY && pointScreen.y <= maxY;
    };
    auto cross2D = [](const XMFLOAT2& a, const XMFLOAT2& b, const XMFLOAT2& c) -> float
    {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    };
    auto segmentsIntersect = [&](const XMFLOAT2& a0, const XMFLOAT2& a1,
                                 const XMFLOAT2& b0, const XMFLOAT2& b1) -> bool
    {
        const float ab0 = cross2D(a0, a1, b0);
        const float ab1 = cross2D(a0, a1, b1);
        const float cd0 = cross2D(b0, b1, a0);
        const float cd1 = cross2D(b0, b1, a1);
        return ((ab0 <= 0.0f && ab1 >= 0.0f) || (ab0 >= 0.0f && ab1 <= 0.0f)) &&
               ((cd0 <= 0.0f && cd1 >= 0.0f) || (cd0 >= 0.0f && cd1 <= 0.0f));
    };
    auto segmentIntersectsRect = [&](const XMFLOAT2& a, const XMFLOAT2& b) -> bool
    {
        if (isScreenPointInside(a) || isScreenPointInside(b))
            return true;

        const float segMinX = (std::min)(a.x, b.x);
        const float segMaxX = (std::max)(a.x, b.x);
        const float segMinY = (std::min)(a.y, b.y);
        const float segMaxY = (std::max)(a.y, b.y);
        if (segMaxX < minX || segMinX > maxX || segMaxY < minY || segMinY > maxY)
            return false;

        const XMFLOAT2 rect[4] =
        {
            { minX, minY },
            { maxX, minY },
            { maxX, maxY },
            { minX, maxY }
        };
        for (int edgeIndex = 0; edgeIndex < 4; ++edgeIndex)
        {
            const XMFLOAT2& r0 = rect[edgeIndex];
            const XMFLOAT2& r1 = rect[(edgeIndex + 1) % 4];
            if (segmentsIntersect(a, b, r0, r1))
                return true;
        }
        return false;
    };

    if (!addToSelection && !removeFromSelection)
    {
        if (pointEditSelection)
        {
            m_selectedPoints.clear();
            m_selectedIntersections.clear();
        }
        else
        {
            m_selectedRoads.clear();
            m_selectedPoints.clear();
            m_selectedIntersections.clear();
        }
    }

    if (pointEditSelection)
    {
        for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
        {
            const Road& road = m_network->roads[ri];
            if (!IsRoadGuidelineVisible(road))
                continue;

            for (int pi = 0; pi < static_cast<int>(road.points.size()); ++pi)
            {
                const XMFLOAT2 pointScreen = WorldToScreen(road.points[pi].pos, viewProj, vpW, vpH);
                if (pointScreen.x < 0.0f || !isScreenPointInside(pointScreen))
                    continue;

                if (removeFromSelection)
                {
                    auto it = std::remove_if(
                        m_selectedPoints.begin(),
                        m_selectedPoints.end(),
                        [ri, pi](const PointRef& pointRef)
                        {
                            return pointRef.roadIndex == ri && pointRef.pointIndex == pi;
                        });
                    m_selectedPoints.erase(it, m_selectedPoints.end());
                }
                else if (!IsPointSelected(ri, pi))
                {
                    m_selectedPoints.push_back({ ri, pi });
                }
            }
        }

        PointRef primaryPoint;
        if (GetPrimarySelectedPoint(primaryPoint))
        {
            m_activeRoad = primaryPoint.roadIndex;
            m_activePoint = primaryPoint.pointIndex;
        }
        else
        {
            m_activeRoad = -1;
            m_activePoint = -1;
        }
        m_activeIntersection = -1;
        m_selectedIntersections.clear();
        return;
    }

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadGuidelineVisible(road))
            continue;

        bool selectedByRect = false;
        for (int pi = 0; pi + 1 < static_cast<int>(road.points.size()); ++pi)
        {
            const XMFLOAT2 a = WorldToScreen(road.points[pi].pos, viewProj, vpW, vpH);
            const XMFLOAT2 b = WorldToScreen(road.points[pi + 1].pos, viewProj, vpW, vpH);
            if (a.x < 0.0f || b.x < 0.0f)
                continue;
            if (segmentIntersectsRect(a, b))
            {
                selectedByRect = true;
                break;
            }
        }
        if (!selectedByRect && road.closed && road.points.size() >= 2)
        {
            const XMFLOAT2 a = WorldToScreen(road.points.back().pos, viewProj, vpW, vpH);
            const XMFLOAT2 b = WorldToScreen(road.points.front().pos, viewProj, vpW, vpH);
            if (a.x >= 0.0f && b.x >= 0.0f && segmentIntersectsRect(a, b))
                selectedByRect = true;
        }
        if (!selectedByRect)
            continue;

        if (removeFromSelection)
        {
            auto it = std::remove(m_selectedRoads.begin(), m_selectedRoads.end(), ri);
            m_selectedRoads.erase(it, m_selectedRoads.end());
        }
        else if (!IsRoadSelected(ri))
        {
            m_selectedRoads.push_back(ri);
        }
    }

    for (int ii = 0; ii < static_cast<int>(m_network->intersections.size()); ++ii)
    {
        const Intersection& isec = m_network->intersections[ii];
        if (!IsIntersectionVisible(isec))
            continue;

        const XMFLOAT2 pointScreen = WorldToScreen(isec.pos, viewProj, vpW, vpH);
        if (pointScreen.x < 0.0f || !isScreenPointInside(pointScreen))
            continue;

        if (removeFromSelection)
        {
            auto it = std::remove(
                m_selectedIntersections.begin(),
                m_selectedIntersections.end(),
                ii);
            m_selectedIntersections.erase(it, m_selectedIntersections.end());
        }
        else if (!IsIntersectionSelected(ii))
        {
            m_selectedIntersections.push_back(ii);
        }
    }

    m_selectedPoints.clear();
    m_activePoint = -1;
    m_activeRoad = m_selectedRoads.empty() ? -1 : m_selectedRoads.front();
    m_activeIntersection = m_selectedIntersections.empty() ? -1 : m_selectedIntersections.front();
}

PolylineEditor::GizmoAxis PolylineEditor::PickGizmoAxis(
    int vpW, int vpH, XMFLOAT2 px, XMMATRIX viewProj) const
{
    XMFLOAT3 pivotPos;
    if (!GetActiveGizmoPivot(pivotPos))
        return GizmoAxis::None;

    XMFLOAT2 pivotScreen = WorldToScreen(pivotPos, viewProj, vpW, vpH);
    if (pivotScreen.x < 0.0f)
        return GizmoAxis::None;

    if (m_scaleXZMode)
    {
        const float worldRadius = ComputeScaleGizmoRadius(pivotPos, viewProj, vpW, vpH);
        constexpr int kSegments = 4;
        constexpr float kThreshold = 10.0f;
        float bestDist = kThreshold;
        XMFLOAT3 corners[kSegments + 1] =
        {
            { pivotPos.x + worldRadius, pivotPos.y, pivotPos.z + worldRadius },
            { pivotPos.x - worldRadius, pivotPos.y, pivotPos.z + worldRadius },
            { pivotPos.x - worldRadius, pivotPos.y, pivotPos.z - worldRadius },
            { pivotPos.x + worldRadius, pivotPos.y, pivotPos.z - worldRadius },
            { pivotPos.x + worldRadius, pivotPos.y, pivotPos.z + worldRadius }
        };
        XMFLOAT2 screenCorners[kSegments] = {};
        bool validQuad = true;
        for (int i = 0; i < kSegments; ++i)
        {
            screenCorners[i] = WorldToScreen(corners[i], viewProj, vpW, vpH);
            if (screenCorners[i].x < 0.0f)
            {
                validQuad = false;
                break;
            }
        }
        if (validQuad)
        {
            bool inside = false;
            for (int i = 0, j = kSegments - 1; i < kSegments; j = i++)
            {
                const XMFLOAT2 a = screenCorners[i];
                const XMFLOAT2 b = screenCorners[j];
                const bool intersects =
                    ((a.y > px.y) != (b.y > px.y)) &&
                    (px.x < (b.x - a.x) * (px.y - a.y) / ((b.y - a.y) + 1e-6f) + a.x);
                if (intersects)
                    inside = !inside;
            }
            if (inside)
                return GizmoAxis::ScaleXZ;
        }
        for (int i = 0; i < kSegments; ++i)
        {
            const XMFLOAT2 s0 = WorldToScreen(corners[i], viewProj, vpW, vpH);
            const XMFLOAT2 s1 = WorldToScreen(corners[i + 1], viewProj, vpW, vpH);
            if (s0.x < 0.0f || s1.x < 0.0f)
                continue;
            bestDist = (std::min)(bestDist, DistPointToSegment2D(px, s0, s1));
        }
        return bestDist < kThreshold ? GizmoAxis::ScaleXZ : GizmoAxis::None;
    }

    if (m_rotateYMode)
    {
        const float worldRadius = ComputeRotationGizmoRadius(pivotPos, viewProj, vpW, vpH);
        constexpr int kSegments = 48;
        constexpr float kThreshold = 10.0f;
        float bestDist = kThreshold;
        for (int segmentIndex = 0; segmentIndex < kSegments; ++segmentIndex)
        {
            const float t0 = XM_2PI * static_cast<float>(segmentIndex) / static_cast<float>(kSegments);
            const float t1 = XM_2PI * static_cast<float>(segmentIndex + 1) / static_cast<float>(kSegments);
            const XMFLOAT3 p0 =
            {
                pivotPos.x + cosf(t0) * worldRadius,
                pivotPos.y,
                pivotPos.z + sinf(t0) * worldRadius
            };
            const XMFLOAT3 p1 =
            {
                pivotPos.x + cosf(t1) * worldRadius,
                pivotPos.y,
                pivotPos.z + sinf(t1) * worldRadius
            };
            const XMFLOAT2 s0 = WorldToScreen(p0, viewProj, vpW, vpH);
            const XMFLOAT2 s1 = WorldToScreen(p1, viewProj, vpW, vpH);
            if (s0.x < 0.0f || s1.x < 0.0f)
                continue;

            bestDist = (std::min)(bestDist, DistPointToSegment2D(px, s0, s1));
        }
        return bestDist < kThreshold ? GizmoAxis::RotateY : GizmoAxis::None;
    }

    const float threshold = 10.0f;

    if (Dist2D(px, pivotScreen) <= threshold)
        return GizmoAxis::Center;

    struct Candidate
    {
        GizmoAxis axis;
        float dist;
    };

    Candidate best = { GizmoAxis::None, threshold };

    for (GizmoAxis axis : { GizmoAxis::X, GizmoAxis::Y, GizmoAxis::Z })
    {
        XMFLOAT3 dir = AxisDirection(axis);
        const float axisLen = ComputeGizmoAxisLength(pivotPos, axis, viewProj, vpW, vpH);
        XMFLOAT3 end =
        {
            pivotPos.x + dir.x * axisLen,
            pivotPos.y + dir.y * axisLen,
            pivotPos.z + dir.z * axisLen
        };

        XMFLOAT2 endScreen = WorldToScreen(end, viewProj, vpW, vpH);
        if (endScreen.x < 0.0f)
            continue;

        float d = DistPointToSegment2D(px,
            pivotScreen,
            endScreen);
        if (d < best.dist)
            best = { axis, d };
    }

    return best.axis;
}

float PolylineEditor::ComputeGizmoAxisLength(
    XMFLOAT3 pivot, GizmoAxis axis, XMMATRIX viewProj, int vpW, int vpH) const
{
    const float targetPixels = 72.0f;
    const XMFLOAT3 dir = AxisDirection(axis);
    const XMFLOAT2 pivotScreen = WorldToScreen(pivot, viewProj, vpW, vpH);
    if (pivotScreen.x < 0.0f)
        return 2.0f;

    const XMFLOAT3 unitEnd =
    {
        pivot.x + dir.x,
        pivot.y + dir.y,
        pivot.z + dir.z
    };
    const XMFLOAT2 unitScreen = WorldToScreen(unitEnd, viewProj, vpW, vpH);
    if (unitScreen.x < 0.0f)
        return 2.0f;

    const float pixelsPerUnit = Dist2D(pivotScreen, unitScreen);
    if (pixelsPerUnit <= 1e-3f)
        return 2.0f;

    return std::clamp(targetPixels / pixelsPerUnit, 0.5f, 1000.0f);
}

float PolylineEditor::ComputeRotationGizmoRadius(
    XMFLOAT3 pivot, XMMATRIX viewProj, int vpW, int vpH) const
{
    const float radiusX = ComputeGizmoAxisLength(pivot, GizmoAxis::X, viewProj, vpW, vpH);
    const float radiusZ = ComputeGizmoAxisLength(pivot, GizmoAxis::Z, viewProj, vpW, vpH);
    return (std::max)(0.5f, 0.8f * (radiusX + radiusZ) * 0.5f);
}

float PolylineEditor::ComputeScaleGizmoRadius(
    XMFLOAT3 pivot, XMMATRIX viewProj, int vpW, int vpH) const
{
    const float radiusX = ComputeGizmoAxisLength(pivot, GizmoAxis::X, viewProj, vpW, vpH);
    const float radiusZ = ComputeGizmoAxisLength(pivot, GizmoAxis::Z, viewProj, vpW, vpH);
    return (std::max)(0.5f, 0.55f * (radiusX + radiusZ) * 0.5f);
}

// ---------------------------------------------------------------------------
// SetMode
// ---------------------------------------------------------------------------
void PolylineEditor::SetMode(EditorMode mode)
{
    if (!m_showRoadGuidelines &&
        (mode == EditorMode::PointEdit || mode == EditorMode::PolylineDraw))
    {
        mode = EditorMode::Navigate;
    }

    // Cancel any in-progress draw
    if (m_mode == EditorMode::PolylineDraw && mode != EditorMode::PolylineDraw)
        CancelRoad();

    m_mode = mode;
    if (mode == EditorMode::PointEdit)
    {
        if (m_selectedPoints.empty() && !m_selectedRoads.empty())
            SelectAllPointsOnSelectedRoads();

        if (m_selectedPoints.empty() && m_selectedIntersections.empty())
            SelectAllPointsOnSelectedRoads();
    }
    else
    {
        m_rotateYMode = false;
        m_scaleXZMode = false;
        ClearPointSelection();
        if (mode != EditorMode::VerticalCurveEdit)
            ClearVerticalCurveSelection();
        if (mode != EditorMode::BankAngleEdit)
            ClearBankAngleSelection();
        if (mode != EditorMode::LaneEdit)
            ClearLaneSectionSelection();
        if (mode != EditorMode::VerticalCurveEdit)
        {
            m_verticalCurveDragging = false;
            m_verticalCurveDragRoad = -1;
            m_verticalCurveDragPoint = -1;
        }
        if (mode != EditorMode::BankAngleEdit)
        {
            m_bankAngleDragging = false;
            m_bankAngleDragRoad = -1;
            m_bankAngleDragPoint = -1;
        }
        if (mode != EditorMode::LaneEdit)
        {
            m_laneSectionDragging = false;
            m_laneSectionDragRoad = -1;
            m_laneSectionDragPoint = -1;
        }
        m_dragging    = false;
        m_activeGizmoAxis = GizmoAxis::None;
        m_pointDragStartPositions.clear();
        m_intersectionDragStartPositions.clear();
    }
    if (mode != EditorMode::IntersectionEdit)
        m_activeIntersection = -1;

    if (IsParametricEditModeHelper(mode))
    {
        const bool activeRoadValid =
            m_activeRoad >= 0 &&
            m_activeRoad < static_cast<int>(m_network->roads.size());
        if (activeRoadValid)
        {
            SelectSingleRoad(m_activeRoad);
        }
        else if (m_selectedRoads.size() == 1)
        {
            SelectSingleRoad(m_selectedRoads.front());
        }
        else
        {
            ClearRoadSelection();
        }
    }
}

void PolylineEditor::StartNewRoad()
{
    if (!m_showRoadGuidelines)
        return;

    m_network->EnsureDefaultGroup();
    PushUndoState();
    m_activeRoad  = m_network->AddRoad("Road " +
        std::to_string(m_network->roads.size()));
    if (m_activeGroup >= 0 &&
        m_activeGroup < static_cast<int>(m_network->groups.size()) &&
        m_activeRoad >= 0 &&
        m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        m_network->roads[m_activeRoad].groupId = m_network->groups[m_activeGroup].id;
    }
    m_activePoint = -1;
    SelectSingleRoad(m_activeRoad);
    m_selectedPoints.clear();
    InvalidateRoadPreviewCache(m_activeRoad);
    m_mode        = EditorMode::PolylineDraw;
}

void PolylineEditor::ConfirmRoad()
{
    if (m_mode == EditorMode::PolylineDraw)
    {
        // Remove incomplete road if fewer than 2 points
        if (m_activeRoad >= 0 &&
            m_activeRoad < static_cast<int>(m_network->roads.size()))
        {
            if (!m_network->roads[m_activeRoad].IsValid())
            {
                PushUndoState();
                m_network->RemoveRoad(m_activeRoad);
                InvalidateAllPreviewCaches();
            }
        }
        m_activeRoad  = -1;
        ClearRoadSelection();
        ClearPointSelection();
        m_mode        = EditorMode::Navigate;
    }
}

void PolylineEditor::CancelRoad()
{
    if (m_mode == EditorMode::PolylineDraw)
    {
        if (m_activeRoad >= 0 &&
            m_activeRoad < static_cast<int>(m_network->roads.size()))
        {
            PushUndoState();
            m_network->RemoveRoad(m_activeRoad);
            InvalidateAllPreviewCaches();
        }
        m_activeRoad  = -1;
        ClearRoadSelection();
        ClearPointSelection();
        m_mode        = EditorMode::Navigate;
    }
}

// ---------------------------------------------------------------------------
// Update
// ---------------------------------------------------------------------------
void PolylineEditor::Update(int vpW, int vpH,
                             XMFLOAT2 mousePos,
                             XMMATRIX invViewProj,
                             bool wantMouse)
{
    SanitizeSelection();

    bool alt = (GetAsyncKeyState(VK_MENU) & 0x8000) != 0;
    bool lDown = (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;
    bool lClick = lDown && !m_prevLButton;  // rising edge
    bool wDown = (GetAsyncKeyState('W') & 0x8000) != 0;
    bool wPress = wDown && !m_prevWKey;
    bool eDown = (GetAsyncKeyState('E') & 0x8000) != 0;
    bool ePress = eDown && !m_prevEKey;
    bool rDown = (GetAsyncKeyState('R') & 0x8000) != 0;
    bool rPress = rDown && !m_prevRKey;
    bool vDown = (GetAsyncKeyState('V') & 0x8000) != 0;
    bool vPress = vDown && !m_prevVKey;
    bool oneDown = (GetAsyncKeyState('1') & 0x8000) != 0;
    bool onePress = oneDown && !m_prev1Key;
    bool twoDown = (GetAsyncKeyState('2') & 0x8000) != 0;
    bool twoPress = twoDown && !m_prev2Key;
    bool threeDown = (GetAsyncKeyState('3') & 0x8000) != 0;
    bool threePress = threeDown && !m_prev3Key;
    bool fourDown = (GetAsyncKeyState('4') & 0x8000) != 0;
    bool fourPress = fourDown && !m_prev4Key;
    bool fiveDown = (GetAsyncKeyState('5') & 0x8000) != 0;
    bool fivePress = fiveDown && !m_prev5Key;
    bool deleteDown = (GetAsyncKeyState(VK_DELETE) & 0x8000) != 0;
    bool deletePress = deleteDown && !m_prevDeleteKey;

    if (wantMouse || alt)
    {
        m_hasCursorPos = false;
        m_hoverSnapIntersection = -1;
        m_prevLButton  = lDown;
        m_prevWKey     = wDown;
        m_prevEKey     = eDown;
        m_prevRKey     = rDown;
        m_prevVKey     = vDown;
        m_prev1Key     = oneDown;
        m_prev2Key     = twoDown;
        m_prev3Key     = threeDown;
        m_prev4Key     = fourDown;
        m_prev5Key     = fiveDown;
        m_prevDeleteKey = deleteDown;
        return;
    }
    m_prevLButton = lDown;
    m_prevWKey = wDown;
    m_prevEKey = eDown;
    m_prevRKey = rDown;
    m_prevVKey = vDown;
    m_prev1Key = oneDown;
    m_prev2Key = twoDown;
    m_prev3Key = threeDown;
    m_prev4Key = fourDown;
    m_prev5Key = fiveDown;
    m_prevDeleteKey = deleteDown;

    // Compute ray
    XMFLOAT3 rayOrig, rayDir;
    ScreenToRay(vpW, vpH, mousePos, invViewProj, rayOrig, rayDir);

    // Terrain intersection for cursor preview and placement
    XMFLOAT3 hitPos = {};
    bool hasHit = false;
    if (m_terrain && m_terrain->IsReady())
    {
        hasHit = m_terrain->Raycast(rayOrig, rayDir, hitPos);
    }
    else
    {
        hasHit = IntersectRayPlane(
            rayOrig, rayDir,
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f },
            hitPos);
    }
    m_hasCursorPos = hasHit;
    m_cursorPos    = hitPos;

    const bool ctrlDown = (GetAsyncKeyState(VK_CONTROL) & 0x8000) != 0;
    const bool zDown = (GetAsyncKeyState('Z') & 0x8000) != 0;
    const bool yDown = (GetAsyncKeyState('Y') & 0x8000) != 0;
    const bool cDown = (GetAsyncKeyState('C') & 0x8000) != 0;
    const bool aDown = (GetAsyncKeyState('A') & 0x8000) != 0;
    const bool pasteVDown = (GetAsyncKeyState('V') & 0x8000) != 0;
    const bool undoShortcut = ctrlDown && zDown;
    const bool redoShortcut = ctrlDown && yDown;
    const bool copyShortcut = ctrlDown && cDown;
    const bool selectAllShortcut = ctrlDown && aDown;
    const bool pasteShortcut = ctrlDown && pasteVDown;
    const auto hasActiveCurveEditRoad = [&]() -> bool
    {
        return m_activeRoad >= 0 &&
               m_activeRoad < static_cast<int>(m_network->roads.size()) &&
               IsParametricEditModeHelper(m_mode);
    };
    const auto selectCurveEditRoad = [&](int roadIndex)
    {
        SelectSingleRoad(roadIndex);
        ClearPointSelection();
        ClearIntersectionSelection();
        if (m_mode == EditorMode::VerticalCurveEdit)
            ClearVerticalCurveSelection();
        else if (m_mode == EditorMode::BankAngleEdit)
            ClearBankAngleSelection();
        else if (m_mode == EditorMode::LaneEdit)
            ClearLaneSectionSelection();
        if (roadIndex >= 0 && roadIndex < static_cast<int>(m_network->roads.size()))
            m_statusMessage = std::string(u8"編集中の道路: ") + m_network->roads[roadIndex].name;
    };
    const auto collectSelectedIntersections = [&]() -> std::vector<int>
    {
        std::vector<int> intersectionsToDelete = m_selectedIntersections;
        if (m_activeIntersection >= 0 &&
            m_activeIntersection < static_cast<int>(m_network->intersections.size()))
        {
            intersectionsToDelete.push_back(m_activeIntersection);
        }
        std::sort(intersectionsToDelete.begin(), intersectionsToDelete.end(), std::greater<int>());
        intersectionsToDelete.erase(
            std::unique(intersectionsToDelete.begin(), intersectionsToDelete.end()),
            intersectionsToDelete.end());
        return intersectionsToDelete;
    };
    const auto deleteIntersectionsByIndex = [&](const std::vector<int>& intersectionsToDelete)
    {
        for (int intersectionIndex : intersectionsToDelete)
        {
            if (intersectionIndex < 0 ||
                intersectionIndex >= static_cast<int>(m_network->intersections.size()))
            {
                continue;
            }

            const std::string removedId = m_network->intersections[intersectionIndex].id;
            for (Road& road : m_network->roads)
            {
                if (road.startIntersectionId == removedId)
                    road.startIntersectionId.clear();
                if (road.endIntersectionId == removedId)
                    road.endIntersectionId.clear();
            }
            m_network->RemoveIntersection(intersectionIndex);
        }
    };
    const auto deleteSelectedIntersections = [&]() -> bool
    {
        const std::vector<int> intersectionsToDelete = collectSelectedIntersections();
        if (intersectionsToDelete.empty())
            return false;

        PushUndoState();
        deleteIntersectionsByIndex(intersectionsToDelete);

        InvalidateAllPreviewCaches();
        ClearIntersectionSelection();
        m_activeIntersection = -1;
        m_hoverSnapIntersection = -1;
        m_statusMessage = intersectionsToDelete.size() > 1
            ? u8"交差点を削除しました"
            : u8"交差点を削除しました";
        return true;
    };
    if (!ImGui::GetIO().WantTextInput)
    {
        if (undoShortcut && !m_prevUndoShortcut)
        {
            Undo();
            m_prevUndoShortcut = undoShortcut;
            m_prevRedoShortcut = redoShortcut;
            return;
        }
        if (redoShortcut && !m_prevRedoShortcut)
        {
            Redo();
            m_prevUndoShortcut = undoShortcut;
            m_prevRedoShortcut = redoShortcut;
            return;
        }
        if (copyShortcut && !m_prevCopyShortcut)
        {
            CopySelectedRoads();
            m_prevCopyShortcut = copyShortcut;
            m_prevPasteShortcut = pasteShortcut;
            m_prevSelectAllShortcut = selectAllShortcut;
            return;
        }
        if (selectAllShortcut && !m_prevSelectAllShortcut)
        {
            bool handled = false;
            if (!m_selectedPoints.empty())
            {
                const int parentRoadIndex = m_selectedPoints.front().roadIndex;
                if (parentRoadIndex >= 0 &&
                    parentRoadIndex < static_cast<int>(m_network->roads.size()))
                {
                    SelectSingleRoad(parentRoadIndex);
                    handled = SelectAllPointsOnSelectedRoads();
                    if (handled)
                        SetMode(EditorMode::PointEdit);
                }
            }
            m_prevSelectAllShortcut = selectAllShortcut;
            m_prevCopyShortcut = copyShortcut;
            m_prevPasteShortcut = pasteShortcut;
            if (handled)
                return;
        }
        if (pasteShortcut && !m_prevPasteShortcut)
        {
            PasteCopiedRoadsAtCursor();
            m_prevCopyShortcut = copyShortcut;
            m_prevPasteShortcut = pasteShortcut;
            m_prevSelectAllShortcut = selectAllShortcut;
            return;
        }
    }
    m_prevUndoShortcut = undoShortcut;
    m_prevRedoShortcut = redoShortcut;
    m_prevCopyShortcut = copyShortcut;
    m_prevPasteShortcut = pasteShortcut;
    m_prevSelectAllShortcut = selectAllShortcut;

    if (!ImGui::GetIO().WantTextInput && !ctrlDown)
    {
        if (onePress)
        {
            SetMode(EditorMode::Navigate);
            return;
        }
        if (twoPress && m_mode != EditorMode::Pathfinding)
        {
            SetMode(EditorMode::PointEdit);
            return;
        }
        if (threePress)
        {
            SetMode(EditorMode::VerticalCurveEdit);
            return;
        }
        if (fourPress)
        {
            SetMode(EditorMode::BankAngleEdit);
            return;
        }
        if (fivePress)
        {
            SetMode(EditorMode::LaneEdit);
            return;
        }
    }

    if (wPress && m_mode != EditorMode::Pathfinding)
    {
        m_rotateYMode = false;
        m_scaleXZMode = false;
        if (!m_selectedRoads.empty() ||
            !m_selectedPoints.empty() ||
            !m_selectedIntersections.empty() ||
            m_mode == EditorMode::PointEdit)
        {
            SetMode(EditorMode::PointEdit);
        }
        else if (m_activeIntersection >= 0)
        {
            SetMode(EditorMode::IntersectionEdit);
        }
    }

    if (ePress && m_mode != EditorMode::Pathfinding)
    {
        m_rotateYMode = true;
        m_scaleXZMode = false;
        if (!m_selectedRoads.empty() ||
            !m_selectedPoints.empty() ||
            !m_selectedIntersections.empty() ||
            m_mode == EditorMode::PointEdit)
        {
            SetMode(EditorMode::PointEdit);
        }
    }

    if (rPress && m_mode != EditorMode::Pathfinding)
    {
        m_rotateYMode = false;
        m_scaleXZMode = true;
        if (!m_selectedRoads.empty() ||
            !m_selectedPoints.empty() ||
            !m_selectedIntersections.empty() ||
            m_mode == EditorMode::PointEdit)
        {
            SetMode(EditorMode::PointEdit);
        }
    }

    if (m_mode == EditorMode::Pathfinding)
    {
        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
            SetMode(EditorMode::Navigate);
        return;
    }

    if (m_mode == EditorMode::VerticalCurveEdit)
    {
        const XMMATRIX viewProj = XMMatrixInverse(nullptr, invViewProj);

        if (m_verticalCurveDragging && lDown)
        {
            int roadIndex = -1;
            float uCoord = 0.0f;
            if (FindNearestPreviewCurveLocation(vpW, vpH, mousePos, viewProj, roadIndex, uCoord) &&
                roadIndex == m_verticalCurveDragRoad &&
                m_verticalCurveDragRoad >= 0 &&
                m_verticalCurveDragRoad < static_cast<int>(m_network->roads.size()))
            {
                std::vector<VerticalCurvePoint>& points = m_network->roads[m_verticalCurveDragRoad].verticalCurve;
                if (m_verticalCurveDragPoint >= 0 &&
                    m_verticalCurveDragPoint < static_cast<int>(points.size()))
                {
                    points[m_verticalCurveDragPoint].uCoord = std::clamp(uCoord, 0.0f, 1.0f);
                    InvalidateRoadPreviewCache(m_verticalCurveDragRoad);
                }
            }
            return;
        }

        if (m_verticalCurveDragging && !lDown)
        {
            if (m_verticalCurveDragRoad >= 0 &&
                m_verticalCurveDragRoad < static_cast<int>(m_network->roads.size()))
            {
                std::vector<VerticalCurvePoint>& points = m_network->roads[m_verticalCurveDragRoad].verticalCurve;
                if (m_verticalCurveDragPoint >= 0 &&
                    m_verticalCurveDragPoint < static_cast<int>(points.size()))
                {
                    const VerticalCurvePoint movedPoint = points[m_verticalCurveDragPoint];
                    std::sort(
                        points.begin(),
                        points.end(),
                        [](const VerticalCurvePoint& a, const VerticalCurvePoint& b)
                        {
                            return a.uCoord < b.uCoord;
                        });

                    int newIndex = -1;
                    for (int i = 0; i < static_cast<int>(points.size()); ++i)
                    {
                        if (fabsf(points[i].uCoord - movedPoint.uCoord) <= 1e-5f &&
                            fabsf(points[i].vcl - movedPoint.vcl) <= 1e-5f &&
                            fabsf(points[i].offset - movedPoint.offset) <= 1e-5f)
                        {
                            newIndex = i;
                            break;
                        }
                    }
                    SelectSingleRoad(m_verticalCurveDragRoad);
                    SelectSingleVerticalCurvePoint(m_verticalCurveDragRoad, newIndex);
                }
            }

            m_verticalCurveDragging = false;
            m_verticalCurveDragRoad = -1;
            m_verticalCurveDragPoint = -1;
            return;
        }

        if (deletePress &&
            !m_selectedVerticalCurvePoints.empty())
        {
            PushUndoState();
            std::vector<VerticalCurveRef> curvesToDelete = m_selectedVerticalCurvePoints;
            std::sort(
                curvesToDelete.begin(),
                curvesToDelete.end(),
                [](const VerticalCurveRef& a, const VerticalCurveRef& b)
                {
                    if (a.roadIndex != b.roadIndex)
                        return a.roadIndex > b.roadIndex;
                    return a.curveIndex > b.curveIndex;
                });
            curvesToDelete.erase(
                std::unique(
                    curvesToDelete.begin(),
                    curvesToDelete.end(),
                    [](const VerticalCurveRef& a, const VerticalCurveRef& b)
                    {
                        return a.roadIndex == b.roadIndex && a.curveIndex == b.curveIndex;
                    }),
                curvesToDelete.end());

            for (const VerticalCurveRef& curveRef : curvesToDelete)
            {
                if (curveRef.roadIndex < 0 ||
                    curveRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                {
                    continue;
                }

                std::vector<VerticalCurvePoint>& points =
                    m_network->roads[curveRef.roadIndex].verticalCurve;
                if (curveRef.curveIndex < 0 ||
                    curveRef.curveIndex >= static_cast<int>(points.size()))
                {
                    continue;
                }

                points.erase(points.begin() + curveRef.curveIndex);
                InvalidateRoadPreviewCache(curveRef.roadIndex);
            }

            ClearVerticalCurveSelection();
            SanitizeSelection();
            m_statusMessage = u8"縦断曲線ポイントを削除しました";
            return;
        }

        if (lClick)
        {
            const int clickedRoad = FindNearestRoad(vpW, vpH, mousePos, viewProj);
            if (!hasActiveCurveEditRoad())
            {
                if (clickedRoad >= 0)
                {
                    selectCurveEditRoad(clickedRoad);
                    return;
                }
                m_statusMessage = u8"縦断曲線を編集する道路を選ぶには道路ポリラインをクリックしてください";
                return;
            }
            if (clickedRoad >= 0 && clickedRoad != m_activeRoad)
            {
                selectCurveEditRoad(clickedRoad);
                return;
            }

            int hitRoadIndex = -1;
            int hitCurveIndex = -1;
            float bestPointDistance = 14.0f;

            for (int roadIndex = m_activeRoad; roadIndex <= m_activeRoad; ++roadIndex)
            {
                const Road& road = m_network->roads[roadIndex];
                if (!IsRoadVisible(road))
                    continue;

                const std::vector<XMFLOAT3>& previewCurve = GetRoadParametricPreviewCurveCached(roadIndex);
                if (previewCurve.size() < 2)
                    continue;

                const std::vector<float>& cumulativeLengths = GetRoadPreviewCurveArcLengthsCached(roadIndex);
                for (int curveIndex = 0; curveIndex < static_cast<int>(road.verticalCurve.size()); ++curveIndex)
                {
                    XMFLOAT3 curvePos = SamplePolylineAtNormalizedDistance(
                        previewCurve,
                        cumulativeLengths,
                        road.verticalCurve[curveIndex].uCoord);

                    const XMFLOAT2 screenPos = WorldToScreen(curvePos, viewProj, vpW, vpH);
                    if (screenPos.x < 0.0f)
                        continue;

                    const float d = Dist2D(mousePos, screenPos);
                    if (d < bestPointDistance)
                    {
                        bestPointDistance = d;
                        hitRoadIndex = roadIndex;
                        hitCurveIndex = curveIndex;
                    }
                }
            }

            if (hitRoadIndex >= 0 && hitCurveIndex >= 0)
            {
                PushUndoState();
                SelectSingleRoad(hitRoadIndex);
                SelectSingleVerticalCurvePoint(hitRoadIndex, hitCurveIndex);
                ClearPointSelection();
                ClearIntersectionSelection();
                m_verticalCurveDragging = true;
                m_verticalCurveDragRoad = hitRoadIndex;
                m_verticalCurveDragPoint = hitCurveIndex;
                return;
            }

            int roadIndex = -1;
            float uCoord = 0.0f;
            if (FindNearestPreviewCurveLocation(vpW, vpH, mousePos, viewProj, roadIndex, uCoord) &&
                roadIndex == m_activeRoad)
            {
                PushUndoState();
                VerticalCurvePoint point;
                point.uCoord = uCoord;
                point.vcl = kVerticalCurveDefaultLength;
                point.offset = 0.0f;

                std::vector<VerticalCurvePoint>& points = m_network->roads[roadIndex].verticalCurve;
                points.push_back(point);
                InvalidateRoadPreviewCache(roadIndex);
                std::sort(
                    points.begin(),
                    points.end(),
                    [](const VerticalCurvePoint& a, const VerticalCurvePoint& b)
                    {
                        return a.uCoord < b.uCoord;
                    });

                int curveIndex = -1;
                for (int i = 0; i < static_cast<int>(points.size()); ++i)
                {
                    if (fabsf(points[i].uCoord - point.uCoord) <= 1e-5f &&
                        fabsf(points[i].vcl - point.vcl) <= 1e-5f &&
                        fabsf(points[i].offset - point.offset) <= 1e-5f)
                    {
                        curveIndex = i;
                        break;
                    }
                }

                SelectSingleRoad(roadIndex);
                SelectSingleVerticalCurvePoint(roadIndex, curveIndex);
                ClearPointSelection();
                ClearIntersectionSelection();
                m_statusMessage = u8"縦断曲線ポイントを追加しました";
                return;
            }
        }

        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
        {
            m_verticalCurveDragging = false;
            m_verticalCurveDragRoad = -1;
            m_verticalCurveDragPoint = -1;
            SetMode(EditorMode::Navigate);
        }

        return;
    }

    if (m_mode == EditorMode::BankAngleEdit)
    {
        const XMMATRIX viewProj = XMMatrixInverse(nullptr, invViewProj);

        if (m_bankAngleDragging && lDown)
        {
            int roadIndex = -1;
            float uCoord = 0.0f;
            if (FindNearestPreviewCurveLocation(vpW, vpH, mousePos, viewProj, roadIndex, uCoord) &&
                roadIndex == m_bankAngleDragRoad &&
                m_bankAngleDragRoad >= 0 &&
                m_bankAngleDragRoad < static_cast<int>(m_network->roads.size()))
            {
                std::vector<BankAnglePoint>& points = m_network->roads[m_bankAngleDragRoad].bankAngle;
                if (m_bankAngleDragPoint >= 0 &&
                    m_bankAngleDragPoint < static_cast<int>(points.size()))
                {
                    points[m_bankAngleDragPoint].uCoord = std::clamp(uCoord, 0.0f, 1.0f);
                    InvalidateRoadPreviewCache(m_bankAngleDragRoad);
                }
            }
            return;
        }

        if (m_bankAngleDragging && !lDown)
        {
            if (m_bankAngleDragRoad >= 0 &&
                m_bankAngleDragRoad < static_cast<int>(m_network->roads.size()))
            {
                std::vector<BankAnglePoint>& points = m_network->roads[m_bankAngleDragRoad].bankAngle;
                if (m_bankAngleDragPoint >= 0 &&
                    m_bankAngleDragPoint < static_cast<int>(points.size()))
                {
                    const BankAnglePoint movedPoint = points[m_bankAngleDragPoint];
                    std::sort(
                        points.begin(),
                        points.end(),
                        [](const BankAnglePoint& a, const BankAnglePoint& b)
                        {
                            return a.uCoord < b.uCoord;
                        });

                    int newIndex = -1;
                    for (int i = 0; i < static_cast<int>(points.size()); ++i)
                    {
                        if (fabsf(points[i].uCoord - movedPoint.uCoord) <= 1e-5f &&
                            fabsf(points[i].targetSpeed - movedPoint.targetSpeed) <= 1e-5f &&
                            points[i].overrideBank == movedPoint.overrideBank &&
                            fabsf(points[i].bankAngle - movedPoint.bankAngle) <= 1e-5f)
                        {
                            newIndex = i;
                            break;
                        }
                    }
                    SelectSingleRoad(m_bankAngleDragRoad);
                    SelectSingleBankAnglePoint(m_bankAngleDragRoad, newIndex);
                }
            }

            m_bankAngleDragging = false;
            m_bankAngleDragRoad = -1;
            m_bankAngleDragPoint = -1;
            return;
        }

        if (deletePress &&
            !m_selectedBankAnglePoints.empty())
        {
            PushUndoState();
            std::vector<BankAngleRef> pointsToDelete = m_selectedBankAnglePoints;
            std::sort(
                pointsToDelete.begin(),
                pointsToDelete.end(),
                [](const BankAngleRef& a, const BankAngleRef& b)
                {
                    if (a.roadIndex != b.roadIndex)
                        return a.roadIndex > b.roadIndex;
                    return a.pointIndex > b.pointIndex;
                });
            pointsToDelete.erase(
                std::unique(
                    pointsToDelete.begin(),
                    pointsToDelete.end(),
                    [](const BankAngleRef& a, const BankAngleRef& b)
                    {
                        return a.roadIndex == b.roadIndex && a.pointIndex == b.pointIndex;
                    }),
                pointsToDelete.end());

            for (const BankAngleRef& pointRef : pointsToDelete)
            {
                if (pointRef.roadIndex < 0 ||
                    pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                {
                    continue;
                }

                std::vector<BankAnglePoint>& points =
                    m_network->roads[pointRef.roadIndex].bankAngle;
                if (pointRef.pointIndex < 0 ||
                    pointRef.pointIndex >= static_cast<int>(points.size()))
                {
                    continue;
                }

                points.erase(points.begin() + pointRef.pointIndex);
                InvalidateRoadPreviewCache(pointRef.roadIndex);
            }

            ClearBankAngleSelection();
            SanitizeSelection();
            m_statusMessage = u8"バンク角ポイントを削除しました";
            return;
        }

        if (lClick)
        {
            const int clickedRoad = FindNearestRoad(vpW, vpH, mousePos, viewProj);
            if (!hasActiveCurveEditRoad())
            {
                if (clickedRoad >= 0)
                {
                    selectCurveEditRoad(clickedRoad);
                    return;
                }
                m_statusMessage = u8"バンク角を編集する道路を選ぶには道路ポリラインをクリックしてください";
                return;
            }
            if (clickedRoad >= 0 && clickedRoad != m_activeRoad)
            {
                selectCurveEditRoad(clickedRoad);
                return;
            }

            int hitRoadIndex = -1;
            int hitPointIndex = -1;
            float bestPointDistance = 14.0f;

            for (int roadIndex = m_activeRoad; roadIndex <= m_activeRoad; ++roadIndex)
            {
                const Road& road = m_network->roads[roadIndex];
                if (!IsRoadVisible(road))
                    continue;

                const std::vector<XMFLOAT3>& previewCurve = GetRoadParametricPreviewCurveCached(roadIndex);
                if (previewCurve.size() < 2)
                    continue;

                const std::vector<float>& cumulativeLengths = GetRoadParametricPreviewCurveArcLengthsCached(roadIndex);
                for (int pointIndex = 0; pointIndex < static_cast<int>(road.bankAngle.size()); ++pointIndex)
                {
                    XMFLOAT3 bankPos = SamplePolylineAtNormalizedDistance(
                        previewCurve,
                        cumulativeLengths,
                        road.bankAngle[pointIndex].uCoord);

                    const XMFLOAT2 screenPos = WorldToScreen(bankPos, viewProj, vpW, vpH);
                    if (screenPos.x < 0.0f)
                        continue;

                    const float d = Dist2D(mousePos, screenPos);
                    if (d < bestPointDistance)
                    {
                        bestPointDistance = d;
                        hitRoadIndex = roadIndex;
                        hitPointIndex = pointIndex;
                    }
                }
            }

            if (hitRoadIndex >= 0 && hitPointIndex >= 0)
            {
                PushUndoState();
                SelectSingleRoad(hitRoadIndex);
                SelectSingleBankAnglePoint(hitRoadIndex, hitPointIndex);
                ClearPointSelection();
                ClearIntersectionSelection();
                m_bankAngleDragging = true;
                m_bankAngleDragRoad = hitRoadIndex;
                m_bankAngleDragPoint = hitPointIndex;
                return;
            }

            int roadIndex = -1;
            float uCoord = 0.0f;
            if (FindNearestPreviewCurveLocation(vpW, vpH, mousePos, viewProj, roadIndex, uCoord) &&
                roadIndex == m_activeRoad)
            {
                PushUndoState();
                BankAnglePoint point;
                point.uCoord = uCoord;
                point.targetSpeed = kBankAngleDefaultTargetSpeed;
                point.overrideBank = false;
                point.bankAngle = 0.0f;

                std::vector<BankAnglePoint>& points = m_network->roads[roadIndex].bankAngle;
                points.push_back(point);
                InvalidateRoadPreviewCache(roadIndex);
                std::sort(
                    points.begin(),
                    points.end(),
                    [](const BankAnglePoint& a, const BankAnglePoint& b)
                    {
                        return a.uCoord < b.uCoord;
                    });

                int pointIndex = -1;
                for (int i = 0; i < static_cast<int>(points.size()); ++i)
                {
                    if (fabsf(points[i].uCoord - point.uCoord) <= 1e-5f &&
                        fabsf(points[i].targetSpeed - point.targetSpeed) <= 1e-5f &&
                        points[i].overrideBank == point.overrideBank &&
                        fabsf(points[i].bankAngle - point.bankAngle) <= 1e-5f)
                    {
                        pointIndex = i;
                        break;
                    }
                }

                SelectSingleRoad(roadIndex);
                SelectSingleBankAnglePoint(roadIndex, pointIndex);
                ClearPointSelection();
                ClearIntersectionSelection();
                m_statusMessage = u8"バンク角ポイントを追加しました";
                return;
            }
        }

        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
        {
            m_bankAngleDragging = false;
            m_bankAngleDragRoad = -1;
            m_bankAngleDragPoint = -1;
            SetMode(EditorMode::Navigate);
        }

        return;
    }

    if (m_mode == EditorMode::LaneEdit)
    {
        const XMMATRIX viewProj = XMMatrixInverse(nullptr, invViewProj);

        if (m_laneSectionDragging && lDown)
        {
            int roadIndex = -1;
            float uCoord = 0.0f;
            if (FindNearestPreviewCurveLocation(vpW, vpH, mousePos, viewProj, roadIndex, uCoord) &&
                roadIndex == m_laneSectionDragRoad &&
                m_laneSectionDragRoad >= 0 &&
                m_laneSectionDragRoad < static_cast<int>(m_network->roads.size()))
            {
                std::vector<LaneSectionPoint>& points = m_network->roads[m_laneSectionDragRoad].laneSections;
                if (m_laneSectionDragPoint >= 0 &&
                    m_laneSectionDragPoint < static_cast<int>(points.size()))
                {
                    points[m_laneSectionDragPoint].uCoord = std::clamp(uCoord, 0.0f, 1.0f);
                    InvalidateRoadPreviewCache(m_laneSectionDragRoad);
                }
            }
            return;
        }

        if (m_laneSectionDragging && !lDown)
        {
            if (m_laneSectionDragRoad >= 0 &&
                m_laneSectionDragRoad < static_cast<int>(m_network->roads.size()))
            {
                std::vector<LaneSectionPoint>& points = m_network->roads[m_laneSectionDragRoad].laneSections;
                if (m_laneSectionDragPoint >= 0 &&
                    m_laneSectionDragPoint < static_cast<int>(points.size()))
                {
                    const LaneSectionPoint movedPoint = points[m_laneSectionDragPoint];
                    std::sort(
                        points.begin(),
                        points.end(),
                        [](const LaneSectionPoint& a, const LaneSectionPoint& b)
                        {
                            return a.uCoord < b.uCoord;
                        });

                    int newIndex = -1;
                    for (int i = 0; i < static_cast<int>(points.size()); ++i)
                    {
                        if (fabsf(points[i].uCoord - movedPoint.uCoord) <= 1e-5f &&
                            points[i].useLaneLeft2 == movedPoint.useLaneLeft2 &&
                            fabsf(points[i].widthLaneLeft2 - movedPoint.widthLaneLeft2) <= 1e-5f &&
                            points[i].useLaneLeft1 == movedPoint.useLaneLeft1 &&
                            fabsf(points[i].widthLaneLeft1 - movedPoint.widthLaneLeft1) <= 1e-5f &&
                            points[i].useLaneCenter == movedPoint.useLaneCenter &&
                            fabsf(points[i].widthLaneCenter - movedPoint.widthLaneCenter) <= 1e-5f &&
                            points[i].useLaneRight1 == movedPoint.useLaneRight1 &&
                            fabsf(points[i].widthLaneRight1 - movedPoint.widthLaneRight1) <= 1e-5f &&
                            points[i].useLaneRight2 == movedPoint.useLaneRight2 &&
                            fabsf(points[i].widthLaneRight2 - movedPoint.widthLaneRight2) <= 1e-5f &&
                            fabsf(points[i].offsetCenter - movedPoint.offsetCenter) <= 1e-5f)
                        {
                            newIndex = i;
                            break;
                        }
                    }
                    SelectSingleRoad(m_laneSectionDragRoad);
                    SelectSingleLaneSectionPoint(m_laneSectionDragRoad, newIndex);
                }
            }

            m_laneSectionDragging = false;
            m_laneSectionDragRoad = -1;
            m_laneSectionDragPoint = -1;
            return;
        }

        if (deletePress &&
            !m_selectedLaneSectionPoints.empty())
        {
            PushUndoState();
            std::vector<LaneSectionRef> pointsToDelete = m_selectedLaneSectionPoints;
            std::sort(
                pointsToDelete.begin(),
                pointsToDelete.end(),
                [](const LaneSectionRef& a, const LaneSectionRef& b)
                {
                    if (a.roadIndex != b.roadIndex)
                        return a.roadIndex > b.roadIndex;
                    return a.pointIndex > b.pointIndex;
                });
            pointsToDelete.erase(
                std::unique(
                    pointsToDelete.begin(),
                    pointsToDelete.end(),
                    [](const LaneSectionRef& a, const LaneSectionRef& b)
                    {
                        return a.roadIndex == b.roadIndex && a.pointIndex == b.pointIndex;
                    }),
                pointsToDelete.end());

            for (const LaneSectionRef& pointRef : pointsToDelete)
            {
                if (pointRef.roadIndex < 0 ||
                    pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                {
                    continue;
                }

                std::vector<LaneSectionPoint>& points =
                    m_network->roads[pointRef.roadIndex].laneSections;
                if (pointRef.pointIndex < 0 ||
                    pointRef.pointIndex >= static_cast<int>(points.size()))
                {
                    continue;
                }

                points.erase(points.begin() + pointRef.pointIndex);
                InvalidateRoadPreviewCache(pointRef.roadIndex);
            }

            ClearLaneSectionSelection();
            SanitizeSelection();
            m_statusMessage = u8"車線セクションポイントを削除しました";
            return;
        }

        if (lClick)
        {
            const int clickedRoad = FindNearestRoad(vpW, vpH, mousePos, viewProj);
            if (!hasActiveCurveEditRoad())
            {
                if (clickedRoad >= 0)
                {
                    selectCurveEditRoad(clickedRoad);
                    return;
                }
                m_statusMessage = u8"車線を編集する道路を選ぶには道路ポリラインをクリックしてください";
                return;
            }
            if (clickedRoad >= 0 && clickedRoad != m_activeRoad)
            {
                selectCurveEditRoad(clickedRoad);
                return;
            }

            int hitRoadIndex = -1;
            int hitPointIndex = -1;
            float bestPointDistance = 14.0f;

            for (int roadIndex = m_activeRoad; roadIndex <= m_activeRoad; ++roadIndex)
            {
                const Road& road = m_network->roads[roadIndex];
                if (!IsRoadVisible(road))
                    continue;

                const std::vector<XMFLOAT3>& previewCurve = GetRoadParametricPreviewCurveCached(roadIndex);
                if (previewCurve.size() < 2)
                    continue;

                const std::vector<float>& cumulativeLengths = GetRoadParametricPreviewCurveArcLengthsCached(roadIndex);
                for (int pointIndex = 0; pointIndex < static_cast<int>(road.laneSections.size()); ++pointIndex)
                {
                    XMFLOAT3 lanePos = SamplePolylineAtNormalizedDistance(
                        previewCurve,
                        cumulativeLengths,
                        road.laneSections[pointIndex].uCoord);

                    const XMFLOAT2 screenPos = WorldToScreen(lanePos, viewProj, vpW, vpH);
                    if (screenPos.x < 0.0f)
                        continue;

                    const float d = Dist2D(mousePos, screenPos);
                    if (d < bestPointDistance)
                    {
                        bestPointDistance = d;
                        hitRoadIndex = roadIndex;
                        hitPointIndex = pointIndex;
                    }
                }
            }

            if (hitRoadIndex >= 0 && hitPointIndex >= 0)
            {
                PushUndoState();
                SelectSingleRoad(hitRoadIndex);
                SelectSingleLaneSectionPoint(hitRoadIndex, hitPointIndex);
                ClearPointSelection();
                ClearIntersectionSelection();
                m_laneSectionDragging = true;
                m_laneSectionDragRoad = hitRoadIndex;
                m_laneSectionDragPoint = hitPointIndex;
                return;
            }

            int roadIndex = -1;
            float uCoord = 0.0f;
            if (FindNearestPreviewCurveLocation(vpW, vpH, mousePos, viewProj, roadIndex, uCoord) &&
                roadIndex == m_activeRoad)
            {
                PushUndoState();
                LaneSectionPoint point;
                point.uCoord = uCoord;
                point.widthLaneLeft2 = kLaneSectionDefaultWidth;
                point.widthLaneLeft1 = kLaneSectionDefaultWidth;
                point.widthLaneCenter = 0.0f;
                point.widthLaneRight1 = kLaneSectionDefaultWidth;
                point.widthLaneRight2 = kLaneSectionDefaultWidth;
                point.offsetCenter = 0.0f;

                std::vector<LaneSectionPoint>& points = m_network->roads[roadIndex].laneSections;
                points.push_back(point);
                InvalidateRoadPreviewCache(roadIndex);
                std::sort(
                    points.begin(),
                    points.end(),
                    [](const LaneSectionPoint& a, const LaneSectionPoint& b)
                    {
                        return a.uCoord < b.uCoord;
                    });

                int pointIndex = -1;
                for (int i = 0; i < static_cast<int>(points.size()); ++i)
                {
                    if (fabsf(points[i].uCoord - point.uCoord) <= 1e-5f &&
                        points[i].useLaneLeft2 == point.useLaneLeft2 &&
                        fabsf(points[i].widthLaneLeft2 - point.widthLaneLeft2) <= 1e-5f &&
                        points[i].useLaneLeft1 == point.useLaneLeft1 &&
                        fabsf(points[i].widthLaneLeft1 - point.widthLaneLeft1) <= 1e-5f &&
                        points[i].useLaneCenter == point.useLaneCenter &&
                        fabsf(points[i].widthLaneCenter - point.widthLaneCenter) <= 1e-5f &&
                        points[i].useLaneRight1 == point.useLaneRight1 &&
                        fabsf(points[i].widthLaneRight1 - point.widthLaneRight1) <= 1e-5f &&
                        points[i].useLaneRight2 == point.useLaneRight2 &&
                        fabsf(points[i].widthLaneRight2 - point.widthLaneRight2) <= 1e-5f &&
                        fabsf(points[i].offsetCenter - point.offsetCenter) <= 1e-5f)
                    {
                        pointIndex = i;
                        break;
                    }
                }

                SelectSingleRoad(roadIndex);
                SelectSingleLaneSectionPoint(roadIndex, pointIndex);
                ClearPointSelection();
                ClearIntersectionSelection();
                m_statusMessage = u8"車線セクションポイントを追加しました";
                return;
            }
        }

        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
        {
            m_laneSectionDragging = false;
            m_laneSectionDragRoad = -1;
            m_laneSectionDragPoint = -1;
            SetMode(EditorMode::Navigate);
        }

        return;
    }

    if (m_marqueeSelecting)
    {
        m_marqueeEnd = mousePos;
        if (!lDown)
        {
            const XMMATRIX viewProj = XMMatrixInverse(nullptr, invViewProj);
            const float width = fabsf(m_marqueeEnd.x - m_marqueeStart.x);
            const float height = fabsf(m_marqueeEnd.y - m_marqueeStart.y);
            if (width >= 4.0f || height >= 4.0f)
            {
                ApplyMarqueeSelection(vpW, vpH, viewProj, m_marqueeAdditive, m_marqueeSubtractive);
                if (m_mode == EditorMode::Navigate && !m_selectedPoints.empty())
                    SetMode(EditorMode::PointEdit);
            }
            else if (!m_marqueeAdditive && !m_marqueeSubtractive)
            {
                m_activeRoad = -1;
                ClearPointSelection();
                ClearIntersectionSelection();
            }
            if (m_mode == EditorMode::PointEdit &&
                m_selectedPoints.empty() &&
                m_selectedIntersections.empty())
            {
                m_mode = EditorMode::Navigate;
            }
            m_marqueeSelecting = false;
        }
        return;
    }

    if (m_mode == EditorMode::Navigate)
    {
        const XMMATRIX viewProj = XMMatrixInverse(nullptr, invViewProj);
        if (deletePress)
        {
            const std::vector<int> intersectionsToDelete = collectSelectedIntersections();
            std::vector<int> roadsToDelete = m_selectedRoads;
            std::sort(roadsToDelete.begin(), roadsToDelete.end(), std::greater<int>());
            roadsToDelete.erase(std::unique(roadsToDelete.begin(), roadsToDelete.end()), roadsToDelete.end());

            const bool shouldDeleteRoads =
                !roadsToDelete.empty() &&
                m_activePoint < 0 &&
                m_selectedPoints.empty();
            const bool shouldDeleteIntersections = !intersectionsToDelete.empty();
            if (shouldDeleteRoads || shouldDeleteIntersections)
            {
                PushUndoState();

                if (shouldDeleteRoads)
                {
                    for (int roadIndex : roadsToDelete)
                    {
                        if (roadIndex >= 0 && roadIndex < static_cast<int>(m_network->roads.size()))
                            m_network->RemoveRoad(roadIndex);
                    }
                }

                if (shouldDeleteIntersections)
                    deleteIntersectionsByIndex(intersectionsToDelete);

                InvalidateAllPreviewCaches();
                ClearRoadSelection();
                ClearPointSelection();
                ClearIntersectionSelection();
                m_activeIntersection = -1;
                m_hoverSnapIntersection = -1;
                if (shouldDeleteRoads && shouldDeleteIntersections)
                    m_statusMessage = u8"道路と交差点を削除しました";
                else if (shouldDeleteRoads)
                    m_statusMessage = roadsToDelete.size() > 1 ? u8"道路を削除しました" : u8"道路を削除しました";
                else
                    m_statusMessage = intersectionsToDelete.size() > 1 ? u8"交差点を削除しました" : u8"交差点を削除しました";
                return;
            }
        }

        if (lClick)
        {
            const int selIntersection = FindNearestIntersection(vpW, vpH, mousePos, viewProj);
            if (selIntersection >= 0)
            {
                if (ctrlDown)
                    ToggleIntersectionSelection(selIntersection);
                else
                {
                    SelectSingleIntersection(selIntersection);
                    ClearPointSelection();
                }
                SetActiveGroupById(m_network->intersections[selIntersection].groupId);
                ClearRoadSelection();
                return;
            }

            if (m_activeRoad >= 0)
            {
                const int selPoint = FindNearestPointOnRoad(
                    m_activeRoad, vpW, vpH, mousePos, viewProj);
                if (selPoint >= 0)
                {
                    if (ctrlDown)
                        TogglePointSelection(m_activeRoad, selPoint);
                    else
                    {
                        SelectSinglePoint(m_activeRoad, selPoint);
                        ClearIntersectionSelection();
                    }
                    SetMode(EditorMode::PointEdit);
                    return;
                }
            }

            const int selRoad = FindNearestRoad(vpW, vpH, mousePos, viewProj);
            if (selRoad >= 0)
            {
                if (ctrlDown)
                    ToggleRoadSelection(selRoad);
                else
                    SelectSingleRoad(selRoad);
                SetActiveGroupById(m_network->roads[selRoad].groupId);
                ClearPointSelection();
                ClearIntersectionSelection();
                return;
            }

            ClearRoadSelection();
            m_dragging = false;
            m_activeGizmoAxis = GizmoAxis::None;
            m_hoverSnapIntersection = -1;
            m_marqueeSelecting = true;
            m_marqueeStart = mousePos;
            m_marqueeEnd = mousePos;
            m_marqueeAdditive = ctrlDown && (GetAsyncKeyState(VK_SHIFT) & 0x8000) == 0;
            m_marqueeSubtractive = ctrlDown && (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;
        }
        return;
    }

    // --- PolylineDraw mode ---
    if (m_mode == EditorMode::PolylineDraw)
    {
        if (lClick && hasHit &&
            m_activeRoad >= 0 &&
            m_activeRoad < static_cast<int>(m_network->roads.size()))
        {
            PushUndoState();
            RoadPoint rp;
            rp.pos   = hitPos;
            rp.width = m_defaultWidth;
            m_network->roads[m_activeRoad].points.push_back(rp);
            InvalidateRoadPreviewCache(m_activeRoad);
        }

        // Enter -> confirm, Esc -> cancel
        if (GetAsyncKeyState(VK_RETURN) & 0x8000)
            ConfirmRoad();
        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
            CancelRoad();

        return;
    }

    if (m_mode == EditorMode::IntersectionDraw)
    {
        if (lClick && hasHit)
        {
            if (FindIntersectionWithinDistance(hitPos, kMinIntersectionSpacingMeters) >= 0)
            {
                m_statusMessage = u8"別の交差点から10m以内には交差点を配置できません";
                return;
            }

            m_network->EnsureDefaultGroup();
            PushUndoState();
            m_activeIntersection = m_network->AddIntersection(hitPos);
            if (m_activeGroup >= 0 &&
                m_activeGroup < static_cast<int>(m_network->groups.size()) &&
                m_activeIntersection >= 0 &&
                m_activeIntersection < static_cast<int>(m_network->intersections.size()))
            {
                m_network->intersections[m_activeIntersection].groupId =
                    m_network->groups[m_activeGroup].id;
            }
            ClearPointSelection();
            m_statusMessage = u8"交差点を配置しました";
        }

        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
            SetMode(EditorMode::Navigate);

        return;
    }

    // --- PointEdit mode ---
    if (m_mode == EditorMode::PointEdit)
    {
        // Rebuild viewProj from invViewProj (inverse)
        XMMATRIX viewProj = XMMatrixInverse(nullptr, invViewProj);

        if (vPress && !ImGui::GetIO().WantTextInput)
        {
            if (SplitSelectedRoadAtPoint())
                return;
        }

        if (m_dragging && lDown)
        {
            if (m_activeGizmoAxis != GizmoAxis::None)
            {
                if ((!m_selectedPoints.empty() || !m_selectedIntersections.empty()) &&
                    m_pointDragStartPositions.size() == m_selectedPoints.size() &&
                    m_intersectionDragStartPositions.size() == m_selectedIntersections.size())
                {
                    XMFLOAT3 pivotStart = m_axisDragStartPos;
                    XMFLOAT3 delta = { 0.0f, 0.0f, 0.0f };
                    float rotationDelta = 0.0f;
                    float scaleFactor = 1.0f;
                    std::vector<XMFLOAT3> pointPositions(m_selectedPoints.size());
                    std::vector<XMFLOAT3> intersectionPositions(m_selectedIntersections.size());
                    if (m_activeGizmoAxis == GizmoAxis::RotateY)
                    {
                        const XMFLOAT2 pivotScreen =
                            WorldToScreen(m_axisDragStartPos, viewProj, vpW, vpH);
                        if (pivotScreen.x >= 0.0f)
                        {
                            const float currentAngle =
                                atan2f(mousePos.y - pivotScreen.y, mousePos.x - pivotScreen.x);
                            rotationDelta = NormalizeAngleDelta(currentAngle - m_rotateDragStartAngle);
                        }
                    }
                    else if (m_activeGizmoAxis == GizmoAxis::ScaleXZ)
                    {
                        const XMFLOAT2 pivotScreen =
                            WorldToScreen(m_axisDragStartPos, viewProj, vpW, vpH);
                        if (pivotScreen.x >= 0.0f)
                        {
                            const float currentDistance = Dist2D(mousePos, pivotScreen);
                            scaleFactor = currentDistance / (std::max)(m_scaleDragStartDistance, 1.0f);
                            scaleFactor = std::clamp(scaleFactor, 0.1f, 20.0f);
                        }
                    }
                    else if (m_snapToTerrain &&
                             hasHit &&
                             m_activeGizmoAxis == GizmoAxis::Center)
                    {
                        delta =
                        {
                            hitPos.x - pivotStart.x,
                            hitPos.y - pivotStart.y,
                            hitPos.z - pivotStart.z
                        };
                    }
                    else if (m_activeGizmoAxis == GizmoAxis::Center)
                    {
                        XMFLOAT3 planeHit;
                        if (IntersectRayPlane(rayOrig, rayDir, m_axisDragStartPos,
                                              m_planeDragNormal, planeHit))
                        {
                            delta =
                            {
                                planeHit.x - m_planeDragStartHit.x,
                                planeHit.y - m_planeDragStartHit.y,
                                planeHit.z - m_planeDragStartHit.z
                            };
                        }
                    }
                    else
                    {
                        XMFLOAT3 axisDir = AxisDirection(m_activeGizmoAxis);
                        const float axisLen = ComputeGizmoAxisLength(
                            m_axisDragStartPos, m_activeGizmoAxis, viewProj, vpW, vpH);
                        const XMFLOAT2 startScreen =
                            WorldToScreen(m_axisDragStartPos, viewProj, vpW, vpH);
                        const XMFLOAT2 endScreen = WorldToScreen(
                            {
                                m_axisDragStartPos.x + axisDir.x * axisLen,
                                m_axisDragStartPos.y + axisDir.y * axisLen,
                                m_axisDragStartPos.z + axisDir.z * axisLen
                            },
                            viewProj, vpW, vpH);

                        const float axisScreenLen = Dist2D(startScreen, endScreen);
                        if (axisScreenLen > 1e-3f)
                        {
                            const XMFLOAT2 axisScreenDir =
                            {
                                (endScreen.x - startScreen.x) / axisScreenLen,
                                (endScreen.y - startScreen.y) / axisScreenLen
                            };
                            const XMFLOAT2 mouseDelta =
                            {
                                mousePos.x - m_axisDragStartMouse.x,
                                mousePos.y - m_axisDragStartMouse.y
                            };
                            const float deltaPixels =
                                mouseDelta.x * axisScreenDir.x +
                                mouseDelta.y * axisScreenDir.y;
                            const float deltaWorld = (deltaPixels / axisScreenLen) * axisLen;
                            delta = { axisDir.x * deltaWorld, axisDir.y * deltaWorld, axisDir.z * deltaWorld };
                        }
                    }

                    for (size_t selectionIndex = 0; selectionIndex < m_selectedPoints.size(); ++selectionIndex)
                    {
                        const PointRef& pointRef = m_selectedPoints[selectionIndex];
                        if (pointRef.roadIndex < 0 ||
                            pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                            continue;
                        Road& road = m_network->roads[pointRef.roadIndex];
                        const int pointIndex = pointRef.pointIndex;
                        if (pointIndex < 0 || pointIndex >= static_cast<int>(road.points.size()))
                            continue;

                        XMFLOAT3 newPos = m_pointDragStartPositions[selectionIndex];
                        if (m_activeGizmoAxis == GizmoAxis::RotateY)
                        {
                            newPos = RotateAroundY(newPos, pivotStart, rotationDelta);
                            if (m_snapToTerrain &&
                                m_terrain && m_terrain->IsReady())
                            {
                                newPos.y = m_terrain->GetHeightAt(newPos.x, newPos.z);
                            }
                        }
                        else if (m_activeGizmoAxis == GizmoAxis::ScaleXZ)
                        {
                            newPos = ScaleAroundXZ(newPos, pivotStart, scaleFactor);
                            if (m_snapToTerrain &&
                                m_terrain && m_terrain->IsReady())
                            {
                                newPos.y = m_terrain->GetHeightAt(newPos.x, newPos.z);
                            }
                        }
                        else
                        {
                            newPos =
                            {
                                m_pointDragStartPositions[selectionIndex].x + delta.x,
                                m_pointDragStartPositions[selectionIndex].y + delta.y,
                                m_pointDragStartPositions[selectionIndex].z + delta.z
                            };

                            if (m_snapToTerrain &&
                                m_terrain && m_terrain->IsReady() &&
                                m_activeGizmoAxis != GizmoAxis::Y)
                            {
                                newPos.y = m_terrain->GetHeightAt(newPos.x, newPos.z);
                            }
                        }

                        pointPositions[selectionIndex] = newPos;
                    }

                    for (size_t selectionIndex = 0; selectionIndex < m_selectedIntersections.size(); ++selectionIndex)
                    {
                        const int intersectionIndex = m_selectedIntersections[selectionIndex];
                        if (intersectionIndex < 0 ||
                            intersectionIndex >= static_cast<int>(m_network->intersections.size()) ||
                            selectionIndex >= m_intersectionDragStartPositions.size())
                            continue;

                        XMFLOAT3 newPos = m_intersectionDragStartPositions[selectionIndex];
                        if (m_activeGizmoAxis == GizmoAxis::RotateY)
                        {
                            newPos = RotateAroundY(newPos, pivotStart, rotationDelta);
                            if (m_snapToTerrain &&
                                m_terrain && m_terrain->IsReady())
                            {
                                newPos.y = m_terrain->GetHeightAt(newPos.x, newPos.z);
                            }
                        }
                        else if (m_activeGizmoAxis == GizmoAxis::ScaleXZ)
                        {
                            newPos = ScaleAroundXZ(newPos, pivotStart, scaleFactor);
                            if (m_snapToTerrain &&
                                m_terrain && m_terrain->IsReady())
                            {
                                newPos.y = m_terrain->GetHeightAt(newPos.x, newPos.z);
                            }
                        }
                        else
                        {
                            newPos =
                            {
                                m_intersectionDragStartPositions[selectionIndex].x + delta.x,
                                m_intersectionDragStartPositions[selectionIndex].y + delta.y,
                                m_intersectionDragStartPositions[selectionIndex].z + delta.z
                            };

                            if (m_snapToTerrain &&
                                m_terrain && m_terrain->IsReady() &&
                                m_activeGizmoAxis != GizmoAxis::Y)
                            {
                                newPos.y = m_terrain->GetHeightAt(newPos.x, newPos.z);
                            }
                        }

                        intersectionPositions[selectionIndex] = newPos;
                    }

                    if (m_snapToPoints && m_activeGizmoAxis == GizmoAxis::Center)
                    {
                        XMFLOAT3 snapSource = {};
                        XMFLOAT3 snapTarget = {};
                        bool hasSnapTarget = false;

                        PointRef primaryPoint;
                        if (GetPrimarySelectedPoint(primaryPoint))
                        {
                            for (size_t selectionIndex = 0; selectionIndex < m_selectedPoints.size(); ++selectionIndex)
                            {
                                if (m_selectedPoints[selectionIndex].roadIndex == primaryPoint.roadIndex &&
                                    m_selectedPoints[selectionIndex].pointIndex == primaryPoint.pointIndex)
                                {
                                    snapSource = pointPositions[selectionIndex];
                                    hasSnapTarget = FindPointSnapTarget(
                                        snapSource, viewProj, vpW, vpH, &primaryPoint, -1, snapTarget);
                                    break;
                                }
                            }
                        }
                        else if (!m_selectedIntersections.empty() && !intersectionPositions.empty())
                        {
                            const int primaryIntersectionIndex = m_selectedIntersections.front();
                            snapSource = intersectionPositions.front();
                            hasSnapTarget = FindPointSnapTarget(
                                snapSource, viewProj, vpW, vpH, nullptr, primaryIntersectionIndex, snapTarget);
                        }

                        if (hasSnapTarget)
                        {
                            const XMFLOAT3 snapDelta =
                            {
                                snapTarget.x - snapSource.x,
                                snapTarget.y - snapSource.y,
                                snapTarget.z - snapSource.z
                            };
                            for (XMFLOAT3& position : pointPositions)
                            {
                                position.x += snapDelta.x;
                                position.y += snapDelta.y;
                                position.z += snapDelta.z;
                            }
                            for (XMFLOAT3& position : intersectionPositions)
                            {
                                position.x += snapDelta.x;
                                position.y += snapDelta.y;
                                position.z += snapDelta.z;
                            }
                        }
                    }

                    for (size_t selectionIndex = 0; selectionIndex < m_selectedPoints.size(); ++selectionIndex)
                    {
                        const PointRef& pointRef = m_selectedPoints[selectionIndex];
                        if (pointRef.roadIndex < 0 ||
                            pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                            continue;
                        Road& road = m_network->roads[pointRef.roadIndex];
                        const int pointIndex = pointRef.pointIndex;
                        if (pointIndex < 0 || pointIndex >= static_cast<int>(road.points.size()))
                            continue;
                        road.points[pointIndex].pos = pointPositions[selectionIndex];
                        InvalidateRoadPreviewCache(pointRef.roadIndex);
                    }

                    for (size_t selectionIndex = 0; selectionIndex < m_selectedIntersections.size(); ++selectionIndex)
                    {
                        const int intersectionIndex = m_selectedIntersections[selectionIndex];
                        if (intersectionIndex < 0 ||
                            intersectionIndex >= static_cast<int>(m_network->intersections.size()))
                            continue;
                        m_network->intersections[intersectionIndex].pos = intersectionPositions[selectionIndex];
                        SyncRoadConnectionsForIntersection(intersectionIndex);
                    }
                    if (!m_selectedIntersections.empty())
                        InvalidateAllPreviewCaches();

                    if (m_selectedPoints.size() == 1 && IsSelectedRoadEndpoint())
                        m_hoverSnapIntersection = FindSnapIntersectionForSelectedEndpoint(vpW, vpH, viewProj);
                    else
                        m_hoverSnapIntersection = -1;
                }
            }
        }
        else if (m_dragging && !lDown)
        {
            if (IsSelectedRoadEndpoint())
            {
                if (m_hoverSnapIntersection >= 0)
                {
                    PushUndoState();
                    SnapSelectedEndpointToIntersection(m_hoverSnapIntersection);
                    m_statusMessage = u8"道路の端点を接続しました";
                }
                else
                {
                    PushUndoState();
                    ClearSelectedRoadConnection();
                }
            }
            m_dragging = false;
            m_activeGizmoAxis = GizmoAxis::None;
            m_hoverSnapIntersection = -1;
            m_pointDragStartPositions.clear();
            m_intersectionDragStartPositions.clear();
        }
        else if (lClick)
        {
            GizmoAxis gizmoAxis = PickGizmoAxis(vpW, vpH, mousePos, viewProj);
            if (gizmoAxis != GizmoAxis::None)
            {
                if (!m_selectedPoints.empty() || !m_selectedIntersections.empty())
                {
                    m_activeGizmoAxis  = gizmoAxis;
                    if (!GetActiveGizmoPivot(m_axisDragStartPos))
                        return;
                    m_axisDragStartMouse = mousePos;
                    const XMFLOAT2 pivotScreen =
                        WorldToScreen(m_axisDragStartPos, viewProj, vpW, vpH);
                    m_rotateDragStartAngle =
                        atan2f(mousePos.y - pivotScreen.y, mousePos.x - pivotScreen.x);
                    m_scaleDragStartDistance = (std::max)(1.0f, Dist2D(mousePos, pivotScreen));
                    m_planeDragNormal = rayDir;
                    if (!IntersectRayPlane(rayOrig, rayDir, m_axisDragStartPos,
                                           m_planeDragNormal, m_planeDragStartHit))
                    {
                        m_planeDragStartHit = m_axisDragStartPos;
                    }
                    m_pointDragStartPositions.clear();
                    for (const PointRef& pointRef : m_selectedPoints)
                    {
                        if (pointRef.roadIndex < 0 ||
                            pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                            continue;
                        const Road& road = m_network->roads[pointRef.roadIndex];
                        if (pointRef.pointIndex >= 0 &&
                            pointRef.pointIndex < static_cast<int>(road.points.size()))
                        {
                            m_pointDragStartPositions.push_back(road.points[pointRef.pointIndex].pos);
                        }
                    }
                    m_intersectionDragStartPositions.clear();
                    for (int intersectionIndex : m_selectedIntersections)
                    {
                        if (intersectionIndex >= 0 &&
                            intersectionIndex < static_cast<int>(m_network->intersections.size()))
                        {
                            m_intersectionDragStartPositions.push_back(
                                m_network->intersections[intersectionIndex].pos);
                        }
                    }
                    PushUndoState();
                    DisconnectSelectedRoadEndpoints();
                    m_dragging         = true;
                }
            }
            else
            {
                // Try to select a point
                int selRoad, selPt;
                FindNearestPoint(vpW, vpH, mousePos, viewProj, selRoad, selPt);
                if (selRoad >= 0)
                {
                    SetActiveGroupById(m_network->roads[selRoad].groupId);
                    if (ctrlDown)
                        TogglePointSelection(selRoad, selPt);
                    else
                    {
                        SelectSinglePoint(selRoad, selPt);
                        ClearIntersectionSelection();
                    }
                    m_activeIntersection = -1;
                    m_dragging         = false;
                    m_activeGizmoAxis  = GizmoAxis::None;
                    m_hoverSnapIntersection = -1;
                }
                else
                {
                    const int selIntersection = FindNearestIntersection(vpW, vpH, mousePos, viewProj);
                    if (selIntersection >= 0)
                    {
                        SetActiveGroupById(m_network->intersections[selIntersection].groupId);
                        if (ctrlDown)
                            ToggleIntersectionSelection(selIntersection);
                        else
                        {
                            SelectSingleIntersection(selIntersection);
                            ClearPointSelection();
                        }
                        m_dragging = false;
                        m_activeGizmoAxis = GizmoAxis::None;
                        m_hoverSnapIntersection = -1;
                    }
                    else if (m_activeRoad >= 0 &&
                             m_activeRoad < static_cast<int>(m_network->roads.size()) &&
                             hasHit)
                    {
                        Road& road = m_network->roads[m_activeRoad];
                        const int segmentIndex =
                            FindNearestSegmentOnRoad(m_activeRoad, vpW, vpH, mousePos, viewProj);
                        if (segmentIndex >= 0)
                        {
                            PushUndoState();
                            RoadPoint newPoint;
                            newPoint.pos = hitPos;
                            if (segmentIndex >= 0 &&
                                segmentIndex + 1 < static_cast<int>(road.points.size()))
                            {
                                newPoint.width =
                                    0.5f * (road.points[segmentIndex].width +
                                            road.points[segmentIndex + 1].width);
                            }
                            else
                            {
                                newPoint.width = m_defaultWidth;
                            }

                            road.points.insert(road.points.begin() + segmentIndex + 1, newPoint);
                            InvalidateRoadPreviewCache(m_activeRoad);
                            SelectSinglePoint(m_activeRoad, segmentIndex + 1);
                            ClearIntersectionSelection();
                            m_dragging = false;
                            m_activeGizmoAxis = GizmoAxis::None;
                            m_hoverSnapIntersection = -1;
                            m_statusMessage = u8"ポイントを挿入しました";
                        }
                        else
                        {
                            m_dragging         = false;
                            m_activeGizmoAxis  = GizmoAxis::None;
                            m_hoverSnapIntersection = -1;
                            m_marqueeSelecting = true;
                            m_marqueeStart = mousePos;
                            m_marqueeEnd = mousePos;
                            m_marqueeAdditive = ctrlDown && (GetAsyncKeyState(VK_SHIFT) & 0x8000) == 0;
                            m_marqueeSubtractive = ctrlDown && (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;
                        }
                    }
                    else
                    {
                        m_dragging         = false;
                        m_activeGizmoAxis  = GizmoAxis::None;
                        m_hoverSnapIntersection = -1;
                        m_marqueeSelecting = true;
                        m_marqueeStart = mousePos;
                        m_marqueeEnd = mousePos;
                        m_marqueeAdditive = ctrlDown && (GetAsyncKeyState(VK_SHIFT) & 0x8000) == 0;
                        m_marqueeSubtractive = ctrlDown && (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;
                    }
                }
            }
        }

        // Delete selected point
        if (deletePress &&
            !m_selectedPoints.empty())
        {
            PushUndoState();
            std::vector<PointRef> pointsToDelete = m_selectedPoints;
            std::sort(
                pointsToDelete.begin(),
                pointsToDelete.end(),
                [](const PointRef& lhs, const PointRef& rhs)
                {
                    if (lhs.roadIndex != rhs.roadIndex)
                        return lhs.roadIndex > rhs.roadIndex;
                    return lhs.pointIndex > rhs.pointIndex;
                });
            pointsToDelete.erase(
                std::unique(
                    pointsToDelete.begin(),
                    pointsToDelete.end(),
                    [](const PointRef& lhs, const PointRef& rhs)
                    {
                        return lhs.roadIndex == rhs.roadIndex && lhs.pointIndex == rhs.pointIndex;
                    }),
                pointsToDelete.end());
            bool removedAny = false;
            for (const PointRef& pointRef : pointsToDelete)
            {
                if (pointRef.roadIndex < 0 || pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                    continue;
                Road& road = m_network->roads[pointRef.roadIndex];
                const int pointIndex = pointRef.pointIndex;
                if (pointIndex < 0 || pointIndex >= static_cast<int>(road.points.size()))
                    continue;
                if (pointIndex == 0)
                    road.startIntersectionId.clear();
                if (pointIndex == static_cast<int>(road.points.size()) - 1)
                    road.endIntersectionId.clear();
                road.points.erase(road.points.begin() + pointIndex);
                InvalidateRoadPreviewCache(pointRef.roadIndex);
                removedAny = true;
            }
            if (removedAny)
                m_statusMessage = pointsToDelete.size() > 1 ? u8"ポイントを削除しました" : u8"ポイントを削除しました";
            ClearPointSelection();
            SanitizeSelection();
            m_activeGizmoAxis = GizmoAxis::None;
            m_hoverSnapIntersection = -1;
            m_pointDragStartPositions.clear();
        }
    }

    if (m_mode == EditorMode::IntersectionEdit)
    {
        XMMATRIX viewProj = XMMatrixInverse(nullptr, invViewProj);

        if (m_dragging && lDown)
        {
            if (m_activeGizmoAxis != GizmoAxis::None &&
                m_activeIntersection >= 0 &&
                m_activeIntersection < static_cast<int>(m_network->intersections.size()))
            {
                Intersection& isec = m_network->intersections[m_activeIntersection];
                XMFLOAT3 newPos = m_axisDragStartPos;

                if (m_snapToTerrain &&
                    hasHit &&
                    m_activeGizmoAxis == GizmoAxis::Center)
                {
                    newPos = hitPos;
                }
                else if (m_activeGizmoAxis == GizmoAxis::Center)
                {
                    XMFLOAT3 planeHit;
                    if (IntersectRayPlane(rayOrig, rayDir, m_axisDragStartPos,
                                          m_planeDragNormal, planeHit))
                    {
                        newPos =
                        {
                            m_axisDragStartPos.x + (planeHit.x - m_planeDragStartHit.x),
                            m_axisDragStartPos.y + (planeHit.y - m_planeDragStartHit.y),
                            m_axisDragStartPos.z + (planeHit.z - m_planeDragStartHit.z)
                        };
                    }
                }
                else
                {
                    XMFLOAT3 axisDir = AxisDirection(m_activeGizmoAxis);
                    const float axisLen = ComputeGizmoAxisLength(
                        m_axisDragStartPos, m_activeGizmoAxis, viewProj, vpW, vpH);
                    const XMFLOAT2 startScreen =
                        WorldToScreen(m_axisDragStartPos, viewProj, vpW, vpH);
                    const XMFLOAT2 endScreen = WorldToScreen(
                        {
                            m_axisDragStartPos.x + axisDir.x * axisLen,
                            m_axisDragStartPos.y + axisDir.y * axisLen,
                            m_axisDragStartPos.z + axisDir.z * axisLen
                        },
                        viewProj, vpW, vpH);
                    const float axisScreenLen = Dist2D(startScreen, endScreen);
                    if (axisScreenLen > 1e-3f)
                    {
                        const XMFLOAT2 axisScreenDir =
                        {
                            (endScreen.x - startScreen.x) / axisScreenLen,
                            (endScreen.y - startScreen.y) / axisScreenLen
                        };
                        const XMFLOAT2 mouseDelta =
                        {
                            mousePos.x - m_axisDragStartMouse.x,
                            mousePos.y - m_axisDragStartMouse.y
                        };
                        const float deltaPixels =
                            mouseDelta.x * axisScreenDir.x +
                            mouseDelta.y * axisScreenDir.y;
                        const float deltaWorld = (deltaPixels / axisScreenLen) * axisLen;
                        newPos =
                        {
                            m_axisDragStartPos.x + axisDir.x * deltaWorld,
                            m_axisDragStartPos.y + axisDir.y * deltaWorld,
                            m_axisDragStartPos.z + axisDir.z * deltaWorld
                        };
                    }
                }

                if (m_snapToTerrain &&
                    m_terrain && m_terrain->IsReady() &&
                    m_activeGizmoAxis != GizmoAxis::Y)
                    newPos.y = m_terrain->GetHeightAt(newPos.x, newPos.z);

                if (m_snapToPoints && m_activeGizmoAxis == GizmoAxis::Center)
                {
                    XMFLOAT3 snapTarget = {};
                    if (FindPointSnapTarget(newPos, viewProj, vpW, vpH, nullptr, m_activeIntersection, snapTarget))
                        newPos = snapTarget;
                }

                isec.pos = newPos;
                SyncRoadConnectionsForIntersection(m_activeIntersection);
            }
        }
        else if (m_dragging && !lDown)
        {
            m_dragging = false;
            m_activeGizmoAxis = GizmoAxis::None;
        }
        else if (lClick)
        {
            GizmoAxis gizmoAxis = PickGizmoAxis(vpW, vpH, mousePos, viewProj);
            if (gizmoAxis != GizmoAxis::None &&
                m_activeIntersection >= 0 &&
                m_activeIntersection < static_cast<int>(m_network->intersections.size()))
            {
                m_activeGizmoAxis = gizmoAxis;
                m_axisDragStartPos = m_network->intersections[m_activeIntersection].pos;
                m_axisDragStartMouse = mousePos;
                m_planeDragNormal = rayDir;
                if (!IntersectRayPlane(rayOrig, rayDir, m_axisDragStartPos,
                                       m_planeDragNormal, m_planeDragStartHit))
                {
                    m_planeDragStartHit = m_axisDragStartPos;
                }
                PushUndoState();
                m_dragging = true;
            }
            else
            {
                m_activeIntersection = FindNearestIntersection(vpW, vpH, mousePos, viewProj);
                if (m_activeIntersection >= 0)
                    SetActiveGroupById(m_network->intersections[m_activeIntersection].groupId);
                ClearPointSelection();
                m_activeGizmoAxis = GizmoAxis::None;
                m_dragging = false;
                if (m_activeIntersection < 0)
                    m_mode = EditorMode::Navigate;
            }
        }

        if (deletePress &&
            (!m_selectedIntersections.empty() ||
             (m_activeIntersection >= 0 &&
              m_activeIntersection < static_cast<int>(m_network->intersections.size()))))
        {
            deleteSelectedIntersections();
        }
    }
}

// ---------------------------------------------------------------------------
// DrawNetwork / DrawOverlay
// ---------------------------------------------------------------------------

void PolylineEditor::DrawNetwork(DebugDraw& dd, XMMATRIX viewProj, int vpW, int vpH) const
{
    const_cast<PolylineEditor*>(this)->SanitizeSelection();

    static const XMFLOAT4 colorRoad     = { 0.72f, 0.72f, 0.75f, 1.0f };
    static const XMFLOAT4 colorSelected = { 1.0f, 1.0f, 1.0f, 1.0f };
    static const XMFLOAT4 colorCursor   = { 0.2f, 1.0f, 0.4f, 0.4f };
    static const XMFLOAT4 colorAxisX    = { 1.0f, 0.2f, 0.2f, 1.0f };
    static const XMFLOAT4 colorAxisY    = { 0.2f, 1.0f, 0.2f, 1.0f };
    static const XMFLOAT4 colorAxisZ    = { 0.2f, 0.6f, 1.0f, 1.0f };
    static const XMFLOAT4 colorRotateY  = { 1.0f, 0.85f, 0.2f, 1.0f };
    static const XMFLOAT4 colorScaleXZ  = { 1.0f, 0.45f, 0.9f, 1.0f };
    static const XMFLOAT4 colorPivot    = { 1.0f, 1.0f, 1.0f, 0.9f };
    static const XMFLOAT4 colorIntersection = { 0.3f, 0.95f, 1.0f, 1.0f };
    static const XMFLOAT4 colorIntersectionSelected = { 1.0f, 0.4f, 0.2f, 1.0f };
    static const XMFLOAT4 colorConnection = { 0.9f, 0.9f, 0.3f, 0.9f };
    static const XMFLOAT4 colorSnapCandidate = { 1.0f, 1.0f, 0.2f, 1.0f };
    static const XMFLOAT4 colorPreview = { 0.45f, 0.95f, 0.95f, 0.9f };

    auto drawRoadLines = [&](int roadIndex, XMFLOAT4 color)
    {
        const Road& road = m_network->roads[roadIndex];
        for (int pi = 0; pi + 1 < static_cast<int>(road.points.size()); ++pi)
            dd.AddLine(road.points[pi].pos, road.points[pi + 1].pos, color);

        if (road.closed && road.points.size() >= 2)
            dd.AddLine(road.points.back().pos, road.points.front().pos, color);

        const int startIsec = FindIntersectionIndexById(road.startIntersectionId);
        const int endIsec   = FindIntersectionIndexById(road.endIntersectionId);
        if (startIsec >= 0 && !road.points.empty() &&
            IsIntersectionVisible(m_network->intersections[startIsec]))
            dd.AddLine(road.points.front().pos, m_network->intersections[startIsec].pos, colorConnection);
        if (endIsec >= 0 && !road.points.empty() &&
            IsIntersectionVisible(m_network->intersections[endIsec]))
            dd.AddLine(road.points.back().pos, m_network->intersections[endIsec].pos, colorConnection);
    };

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadGuidelineVisible(road) || IsRoadSelected(ri) || ri == m_activeRoad)
            continue;
        drawRoadLines(ri, colorRoad);
    }

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadGuidelineVisible(road) || (!IsRoadSelected(ri) && ri != m_activeRoad))
            continue;
        drawRoadLines(ri, colorSelected);
    }

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadVisible(road))
            continue;

    }

    if (m_showRoadGuidelines)
    {
        for (int ii = 0; ii < static_cast<int>(m_network->intersections.size()); ++ii)
        {
            const Intersection& isec = m_network->intersections[ii];
            if (!IsIntersectionVisible(isec))
                continue;
            const XMFLOAT4 col = (ii == m_activeIntersection)
                || IsIntersectionSelected(ii)
                ? colorIntersectionSelected
                : colorIntersection;
            const float r = (std::max)(1.5f, isec.entryDist * 0.35f);
            dd.AddLine({ isec.pos.x - r, isec.pos.y, isec.pos.z },
                       { isec.pos.x + r, isec.pos.y, isec.pos.z }, col);
            dd.AddLine({ isec.pos.x, isec.pos.y, isec.pos.z - r },
                       { isec.pos.x, isec.pos.y, isec.pos.z + r }, col);
            dd.AddLine({ isec.pos.x - r * 0.7f, isec.pos.y, isec.pos.z - r * 0.7f },
                       { isec.pos.x + r * 0.7f, isec.pos.y, isec.pos.z + r * 0.7f }, col);
            dd.AddLine({ isec.pos.x - r * 0.7f, isec.pos.y, isec.pos.z + r * 0.7f },
                       { isec.pos.x + r * 0.7f, isec.pos.y, isec.pos.z - r * 0.7f }, col);

            if (ii == m_hoverSnapIntersection)
            {
                const float rr = r * 1.5f;
                dd.AddLine({ isec.pos.x - rr, isec.pos.y, isec.pos.z }, { isec.pos.x + rr, isec.pos.y, isec.pos.z }, colorSnapCandidate);
                dd.AddLine({ isec.pos.x, isec.pos.y, isec.pos.z - rr }, { isec.pos.x, isec.pos.y, isec.pos.z + rr }, colorSnapCandidate);
            }
        }
    }

    // Preview segment from cursor to last placed point
    if (m_hasCursorPos &&
        m_mode == EditorMode::PolylineDraw &&
        m_activeRoad >= 0 &&
        m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        const Road& road = m_network->roads[m_activeRoad];
        if (!road.points.empty())
            dd.AddLine(road.points.back().pos, m_cursorPos, colorCursor);
    }

    if (m_mode == EditorMode::PointEdit)
    {
        XMFLOAT3 pivot;
        if (GetActiveGizmoPivot(pivot))
        {
            const XMFLOAT3 p = pivot;
            dd.AddLine({ p.x - 0.15f, p.y, p.z }, { p.x + 0.15f, p.y, p.z }, colorPivot);
            dd.AddLine({ p.x, p.y - 0.15f, p.z }, { p.x, p.y + 0.15f, p.z }, colorPivot);
            dd.AddLine({ p.x, p.y, p.z - 0.15f }, { p.x, p.y, p.z + 0.15f }, colorPivot);
            if (m_rotateYMode)
            {
                constexpr int kSegments = 48;
                const float radius = ComputeRotationGizmoRadius(pivot, viewProj, vpW, vpH);
                XMFLOAT3 prev =
                {
                    p.x + radius,
                    p.y,
                    p.z
                };
                for (int segmentIndex = 1; segmentIndex <= kSegments; ++segmentIndex)
                {
                    const float t = XM_2PI * static_cast<float>(segmentIndex) / static_cast<float>(kSegments);
                    const XMFLOAT3 next =
                    {
                        p.x + cosf(t) * radius,
                        p.y,
                        p.z + sinf(t) * radius
                    };
                    dd.AddLine(prev, next, colorRotateY);
                    prev = next;
                }
            }
            else if (m_scaleXZMode)
            {
                const float radius = ComputeScaleGizmoRadius(pivot, viewProj, vpW, vpH);
                XMFLOAT3 corners[5] =
                {
                    { p.x + radius, p.y, p.z + radius },
                    { p.x - radius, p.y, p.z + radius },
                    { p.x - radius, p.y, p.z - radius },
                    { p.x + radius, p.y, p.z - radius },
                    { p.x + radius, p.y, p.z + radius }
                };
                for (int i = 0; i < 4; ++i)
                    dd.AddLine(corners[i], corners[i + 1], colorScaleXZ);
            }
            else
            {
                const float axisLenX = ComputeGizmoAxisLength(pivot, GizmoAxis::X, viewProj, vpW, vpH);
                const float axisLenY = ComputeGizmoAxisLength(pivot, GizmoAxis::Y, viewProj, vpW, vpH);
                const float axisLenZ = ComputeGizmoAxisLength(pivot, GizmoAxis::Z, viewProj, vpW, vpH);
                dd.AddLine(p, { p.x + axisLenX, p.y, p.z }, colorAxisX);
                dd.AddLine(p, { p.x, p.y + axisLenY, p.z }, colorAxisY);
                dd.AddLine(p, { p.x, p.y, p.z + axisLenZ }, colorAxisZ);
            }
        }
    }

    if (m_mode == EditorMode::IntersectionEdit &&
        m_activeIntersection >= 0 &&
        m_activeIntersection < static_cast<int>(m_network->intersections.size()))
    {
        const Intersection& isec = m_network->intersections[m_activeIntersection];
        const float axisLenX = ComputeGizmoAxisLength(isec.pos, GizmoAxis::X, viewProj, vpW, vpH);
        const float axisLenY = ComputeGizmoAxisLength(isec.pos, GizmoAxis::Y, viewProj, vpW, vpH);
        const float axisLenZ = ComputeGizmoAxisLength(isec.pos, GizmoAxis::Z, viewProj, vpW, vpH);
        const XMFLOAT3 p = isec.pos;
        dd.AddLine({ p.x - 0.15f, p.y, p.z }, { p.x + 0.15f, p.y, p.z }, colorPivot);
        dd.AddLine({ p.x, p.y - 0.15f, p.z }, { p.x, p.y + 0.15f, p.z }, colorPivot);
        dd.AddLine({ p.x, p.y, p.z - 0.15f }, { p.x, p.y, p.z + 0.15f }, colorPivot);
        dd.AddLine(p, { p.x + axisLenX, p.y, p.z }, colorAxisX);
        dd.AddLine(p, { p.x, p.y + axisLenY, p.z }, colorAxisY);
        dd.AddLine(p, { p.x, p.y, p.z + axisLenZ }, colorAxisZ);
    }
}

// Project a world-space point to screen pixels. Returns false if behind camera.
static bool WorldToScreen(XMFLOAT3 world, XMMATRIX viewProj,
                           int vpW, int vpH, ImVec2& out)
{
    XMVECTOR h = XMVector4Transform(
        XMVectorSet(world.x, world.y, world.z, 1.0f), viewProj);
    float w = XMVectorGetW(h);
    if (w <= 0.0f) return false;
    out.x = ( XMVectorGetX(h) / w * 0.5f + 0.5f) * vpW;
    out.y = (-XMVectorGetY(h) / w * 0.5f + 0.5f) * vpH;
    return true;
}

void PolylineEditor::DrawOverlay(XMMATRIX viewProj, int vpW, int vpH) const
{
    const_cast<PolylineEditor*>(this)->SanitizeSelection();

    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    auto colorFromFloat3 = [](const XMFLOAT3& color, int alpha) -> ImU32
    {
        const float clampedR = (std::max)(0.0f, (std::min)(1.0f, color.x));
        const float clampedG = (std::max)(0.0f, (std::min)(1.0f, color.y));
        const float clampedB = (std::max)(0.0f, (std::min)(1.0f, color.z));
        return IM_COL32(
            static_cast<int>(clampedR * 255.0f + 0.5f),
            static_cast<int>(clampedG * 255.0f + 0.5f),
            static_cast<int>(clampedB * 255.0f + 0.5f),
            alpha);
    };
    const float kRadius    = (std::max)(1.0f, m_roadVertexScreenRadius);
    const float kRoadThickness = (std::max)(1.0f, m_roadLineThickness);
    const float kPreviewCurveThickness = (std::max)(1.0f, m_previewCurveThickness);
    const float kSelectedRoadThickness = (std::max)(1.0f, m_selectedRoadLineThickness);
    const ImU32 colPoint    = colorFromFloat3(m_roadVertexColor, 220);
    const ImU32 colRoad     = IM_COL32(184, 184, 191, 255);
    const ImU32 colRoadSel  = colorFromFloat3(m_selectedRoadColor, 255);
    const ImU32 colPointSel = IM_COL32(255, 140,  60, 255);
    const ImU32 colCursor   = IM_COL32( 60, 255, 110, 220);
    const ImU32 colAxisX    = IM_COL32(255,  80,  80, 255);
    const ImU32 colAxisY    = IM_COL32( 80, 255, 120, 255);
    const ImU32 colAxisZ    = IM_COL32( 80, 150, 255, 255);
    const ImU32 colScaleXZ  = IM_COL32(255, 110, 220, 255);
    const ImU32 colIsec     = colorFromFloat3(m_intersectionCircleColor, 255);
    const ImU32 colIsecSel  = IM_COL32(255, 120, 80, 255);
    const ImU32 colSnap     = IM_COL32(255, 240, 80, 255);
    const ImU32 colVertical = IM_COL32(255, 190, 70, 245);
    const ImU32 colVerticalSel = IM_COL32(255, 110, 40, 255);
    const ImU32 colBank = IM_COL32(110, 210, 255, 245);
    const ImU32 colBankSel = IM_COL32(40, 140, 255, 255);
    const ImU32 colBankUp = IM_COL32(120, 255, 120, 220);
    const ImU32 colBankLeft = IM_COL32(120, 210, 255, 220);
    const ImU32 colLane = IM_COL32(150, 255, 120, 245);
    const ImU32 colLaneSel = IM_COL32(70, 210, 60, 255);
    const ImU32 colLaneCenterline = IM_COL32(120, 220, 255, 220);
    const ImU32 colLaneBoundary = IM_COL32(180, 255, 90, 210);
    const ImU32 colLaneOutline = IM_COL32(190, 110, 255, 230);
    bool showClothoidPreview = true;
    bool showVerticalPreview = true;
    switch (m_mode)
    {
    case EditorMode::Navigate:
        showClothoidPreview = false;
        showVerticalPreview = true;
        break;
    case EditorMode::PointEdit:
        showClothoidPreview = true;
        showVerticalPreview = false;
        break;
    case EditorMode::VerticalCurveEdit:
        showClothoidPreview = true;
        showVerticalPreview = true;
        break;
    case EditorMode::BankAngleEdit:
    case EditorMode::LaneEdit:
        showClothoidPreview = false;
        showVerticalPreview = true;
        break;
    default:
        showClothoidPreview = true;
        showVerticalPreview = true;
        break;
    }

    auto drawRoadOverlay = [&](const Road& road, ImU32 color, float thickness)
    {
        for (int pi = 0; pi + 1 < static_cast<int>(road.points.size()); ++pi)
        {
            ImVec2 a;
            ImVec2 b;
            if (!WorldToScreen(road.points[pi].pos, viewProj, vpW, vpH, a) ||
                !WorldToScreen(road.points[pi + 1].pos, viewProj, vpW, vpH, b))
                continue;
            dl->AddLine(a, b, color, thickness);
        }

        if (road.closed && road.points.size() >= 2)
        {
            ImVec2 a;
            ImVec2 b;
            if (WorldToScreen(road.points.back().pos, viewProj, vpW, vpH, a) &&
                WorldToScreen(road.points.front().pos, viewProj, vpW, vpH, b))
            {
                dl->AddLine(a, b, color, thickness);
            }
        }
    };

    const bool showCurveEditSelectionGuides = IsParametricEditModeHelper(m_mode);
    if (showCurveEditSelectionGuides)
    {
        const ImU32 colGuideInactive = IM_COL32(140, 140, 150, 120);
        const ImU32 colGuideActive = IM_COL32(255, 220, 120, 235);
        for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
        {
            const Road& road = m_network->roads[ri];
            if (!IsRoadVisible(road))
                continue;
            const bool active = (ri == m_activeRoad);
            drawRoadOverlay(
                road,
                active ? colGuideActive : colGuideInactive,
                active ? (kSelectedRoadThickness + 2.5f) : (kRoadThickness + 0.75f));
        }
    }

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadGuidelineVisible(road) || IsRoadSelected(ri) || ri == m_activeRoad)
            continue;
        drawRoadOverlay(road, colRoad, kRoadThickness);
    }

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadVisible(road))
            continue;

        if (showClothoidPreview)
        {
            const std::vector<PreviewCurvePoint>& previewCurveDetailed = GetRoadPreviewCurveDetailedCached(ri);
            for (int sampleIndex = 0; sampleIndex + 1 < static_cast<int>(previewCurveDetailed.size()); ++sampleIndex)
            {
                ImVec2 a;
                ImVec2 b;
                if (!WorldToScreen(previewCurveDetailed[sampleIndex].pos, viewProj, vpW, vpH, a) ||
                    !WorldToScreen(previewCurveDetailed[sampleIndex + 1].pos, viewProj, vpW, vpH, b))
                    continue;

                ImU32 segmentColor = IM_COL32(255, 255, 255, 230);
                if (m_mode != EditorMode::VerticalCurveEdit)
                {
                    if (previewCurveDetailed[sampleIndex].kind == PreviewCurveSegmentKind::Arc ||
                        previewCurveDetailed[sampleIndex + 1].kind == PreviewCurveSegmentKind::Arc)
                    {
                        segmentColor = IM_COL32(128, 255, 0, 230);
                    }
                    else if (previewCurveDetailed[sampleIndex].kind == PreviewCurveSegmentKind::Clothoid ||
                             previewCurveDetailed[sampleIndex + 1].kind == PreviewCurveSegmentKind::Clothoid)
                    {
                        segmentColor = IM_COL32(255, 128, 0, 230);
                    }
                }

                dl->AddLine(a, b, segmentColor, kPreviewCurveThickness);
            }
        }

        if (showVerticalPreview)
        {
            const std::vector<PreviewCurvePoint>& verticalPreviewCurveDetailed =
                GetRoadVerticalPreviewCurveDetailedCached(ri);
            const std::vector<unsigned int>* verticalGradeColors = nullptr;
            const std::vector<unsigned int>* bankPreviewColors = nullptr;
            if (m_showRoadGradeGradient)
                verticalGradeColors = &GetRoadVerticalGradeColorsCached(ri);
            if (m_mode == EditorMode::BankAngleEdit)
                bankPreviewColors = &GetRoadBankPreviewColorsCached(ri);
            for (int sampleIndex = 0; sampleIndex + 1 < static_cast<int>(verticalPreviewCurveDetailed.size()); ++sampleIndex)
            {
                ImVec2 a;
                ImVec2 b;
                if (!WorldToScreen(verticalPreviewCurveDetailed[sampleIndex].pos, viewProj, vpW, vpH, a) ||
                    !WorldToScreen(verticalPreviewCurveDetailed[sampleIndex + 1].pos, viewProj, vpW, vpH, b))
                    continue;

                ImU32 segmentColor = IM_COL32(255, 255, 255, 210);
                if (verticalPreviewCurveDetailed[sampleIndex].kind == PreviewCurveSegmentKind::VerticalCurveCrest ||
                    verticalPreviewCurveDetailed[sampleIndex + 1].kind == PreviewCurveSegmentKind::VerticalCurveCrest)
                {
                    segmentColor = IM_COL32(255, 170, 40, 245);
                }
                else if (verticalPreviewCurveDetailed[sampleIndex].kind == PreviewCurveSegmentKind::VerticalCurveSag ||
                         verticalPreviewCurveDetailed[sampleIndex + 1].kind == PreviewCurveSegmentKind::VerticalCurveSag)
                {
                    segmentColor = IM_COL32(70, 190, 255, 245);
                }
                if (verticalGradeColors != nullptr &&
                    sampleIndex < static_cast<int>(verticalGradeColors->size()))
                {
                    segmentColor = (*verticalGradeColors)[sampleIndex];
                }
                if (bankPreviewColors != nullptr &&
                    sampleIndex < static_cast<int>(bankPreviewColors->size()))
                {
                    segmentColor = (*bankPreviewColors)[sampleIndex];
                }

                dl->AddLine(a, b, segmentColor, kPreviewCurveThickness + 1.0f);
            }

            if (m_showRoadTerrainClearance &&
                m_terrain != nullptr &&
                m_terrain->IsReady())
            {
                const std::vector<TerrainClearanceSample>& clearanceSamples =
                    GetRoadTerrainClearanceSamplesCached(ri);
                for (const TerrainClearanceSample& sample : clearanceSamples)
                {
                    ImVec2 a;
                    ImVec2 b;
                    if (!WorldToScreen(sample.curvePos, viewProj, vpW, vpH, a) ||
                        !WorldToScreen(sample.terrainPos, viewProj, vpW, vpH, b))
                        continue;

                    dl->AddLine(a, b, sample.color, (std::max)(1.0f, kPreviewCurveThickness));
                }
            }
        }

        if (m_mode != EditorMode::VerticalCurveEdit || road.verticalCurve.empty())
            continue;

        const std::vector<XMFLOAT3>& previewCurve = GetRoadPreviewCurveCached(ri);
        const std::vector<float>& cumulativeLengths = GetRoadPreviewCurveArcLengthsCached(ri);
        for (int curveIndex = 0; curveIndex < static_cast<int>(road.verticalCurve.size()); ++curveIndex)
        {
            XMFLOAT3 curvePos = SamplePolylineAtNormalizedDistance(
                previewCurve,
                cumulativeLengths,
                road.verticalCurve[curveIndex].uCoord);

            ImVec2 screenPos;
            if (!WorldToScreen(curvePos, viewProj, vpW, vpH, screenPos))
                continue;

            const bool selected = IsVerticalCurveSelected(ri, curveIndex);
            dl->AddCircleFilled(
                screenPos,
                selected ? (kRadius + 3.0f) : (kRadius + 1.5f),
                selected ? colVerticalSel : colVertical,
                20);
            dl->AddCircle(
                screenPos,
                selected ? (kRadius + 4.5f) : (kRadius + 2.5f),
                IM_COL32(255, 245, 230, selected ? 255 : 190),
                20,
                1.2f);
        }
    }

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadVisible(road) || m_mode != EditorMode::BankAngleEdit)
            continue;

        const std::vector<BankFrameSample>& bankFrameSamples = GetRoadBankFrameSamplesCached(ri);
        for (const BankFrameSample& sample : bankFrameSamples)
        {
            ImVec2 origin;
            ImVec2 upEnd;
            ImVec2 leftEnd;
            if (!WorldToScreen(sample.pos, viewProj, vpW, vpH, origin) ||
                !WorldToScreen(
                    Add3(sample.pos, Scale3(sample.up, (std::max)(0.5f, m_bankVectorInterval))),
                    viewProj, vpW, vpH, upEnd) ||
                !WorldToScreen(
                    Add3(sample.pos, Scale3(sample.left, (std::max)(0.5f, m_bankVectorInterval))),
                    viewProj, vpW, vpH, leftEnd))
            {
                continue;
            }

            const float angleDegrees = XMConvertToDegrees(fabsf(sample.angleRadians));
            const XMFLOAT4 bankColor = PreviewGradeColor(angleDegrees, m_bankAngleColorMaxDegrees);
            const ImU32 sampleColor = IM_COL32(
                static_cast<int>(bankColor.x * 255.0f + 0.5f),
                static_cast<int>(bankColor.y * 255.0f + 0.5f),
                static_cast<int>(bankColor.z * 255.0f + 0.5f),
                static_cast<int>(bankColor.w * 255.0f + 0.5f));
            dl->AddLine(origin, upEnd, sampleColor, 1.5f);
            dl->AddLine(origin, leftEnd, sampleColor, 1.5f);
        }

        const std::vector<XMFLOAT3>& previewCurve = GetRoadParametricPreviewCurveCached(ri);
        if (previewCurve.size() < 2)
            continue;
        const std::vector<float>& cumulativeLengths = GetRoadParametricPreviewCurveArcLengthsCached(ri);
        for (int pointIndex = 0; pointIndex < static_cast<int>(road.bankAngle.size()); ++pointIndex)
        {
            XMFLOAT3 bankPos = SamplePolylineAtNormalizedDistance(
                previewCurve,
                cumulativeLengths,
                road.bankAngle[pointIndex].uCoord);

            ImVec2 screenPos;
            if (!WorldToScreen(bankPos, viewProj, vpW, vpH, screenPos))
                continue;

            const bool selected = IsBankAngleSelected(ri, pointIndex);
            dl->AddRectFilled(
                ImVec2(screenPos.x - (selected ? 5.0f : 4.0f), screenPos.y - (selected ? 5.0f : 4.0f)),
                ImVec2(screenPos.x + (selected ? 5.0f : 4.0f), screenPos.y + (selected ? 5.0f : 4.0f)),
                selected ? colBankSel : colBank,
                2.0f);
            dl->AddRect(
                ImVec2(screenPos.x - (selected ? 7.0f : 6.0f), screenPos.y - (selected ? 7.0f : 6.0f)),
                ImVec2(screenPos.x + (selected ? 7.0f : 6.0f), screenPos.y + (selected ? 7.0f : 6.0f)),
                IM_COL32(235, 248, 255, selected ? 255 : 190),
                2.0f,
                0,
                1.2f);
        }
    }

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadVisible(road) || m_mode != EditorMode::LaneEdit)
            continue;

        const std::vector<LanePreviewLine>& lanePreviewLines = GetRoadLanePreviewLinesCached(ri);
        for (size_t lineIndex = 0; lineIndex < lanePreviewLines.size(); ++lineIndex)
        {
            const std::vector<XMFLOAT3>& linePoints = lanePreviewLines[lineIndex].points;
            if (linePoints.size() < 2)
                continue;

            ImU32 lineColor = colLaneBoundary;
            if (lineIndex == static_cast<size_t>(LanePreviewLineKind::Centerline))
                lineColor = colLaneCenterline;
            else if (lineIndex == static_cast<size_t>(LanePreviewLineKind::LeftOuter2) ||
                     lineIndex == static_cast<size_t>(LanePreviewLineKind::LeftOuter1) ||
                     lineIndex == static_cast<size_t>(LanePreviewLineKind::RightOuter1) ||
                     lineIndex == static_cast<size_t>(LanePreviewLineKind::RightOuter2))
                lineColor = colLaneOutline;

            for (size_t pointIndex = 0; pointIndex + 1 < linePoints.size(); ++pointIndex)
            {
                if (!IsFinitePoint3(linePoints[pointIndex]) ||
                    !IsFinitePoint3(linePoints[pointIndex + 1]))
                    continue;

                ImVec2 a;
                ImVec2 b;
                if (!WorldToScreen(linePoints[pointIndex], viewProj, vpW, vpH, a) ||
                    !WorldToScreen(linePoints[pointIndex + 1], viewProj, vpW, vpH, b))
                    continue;
                dl->AddLine(a, b, lineColor, kPreviewCurveThickness);
            }
        }

        const std::vector<XMFLOAT3>& previewCurve = GetRoadParametricPreviewCurveCached(ri);
        if (previewCurve.size() < 2)
            continue;
        const std::vector<float>& cumulativeLengths = GetRoadParametricPreviewCurveArcLengthsCached(ri);
        for (int pointIndex = 0; pointIndex < static_cast<int>(road.laneSections.size()); ++pointIndex)
        {
            XMFLOAT3 lanePos = SamplePolylineAtNormalizedDistance(
                previewCurve,
                cumulativeLengths,
                road.laneSections[pointIndex].uCoord);

            ImVec2 screenPos;
            if (!WorldToScreen(lanePos, viewProj, vpW, vpH, screenPos))
                continue;

            const bool selected = IsLaneSectionSelected(ri, pointIndex);
            const float radius = selected ? 6.0f : 5.0f;
            dl->AddTriangleFilled(
                ImVec2(screenPos.x, screenPos.y - radius - 1.0f),
                ImVec2(screenPos.x - radius, screenPos.y + radius - 1.0f),
                ImVec2(screenPos.x + radius, screenPos.y + radius - 1.0f),
                selected ? colLaneSel : colLane);
            dl->AddTriangle(
                ImVec2(screenPos.x, screenPos.y - radius - 3.0f),
                ImVec2(screenPos.x - radius - 1.5f, screenPos.y + radius),
                ImVec2(screenPos.x + radius + 1.5f, screenPos.y + radius),
                IM_COL32(235, 255, 230, selected ? 255 : 190),
                1.2f);
        }
    }

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadGuidelineVisible(road) || (!IsRoadSelected(ri) && ri != m_activeRoad))
            continue;
        drawRoadOverlay(road, colRoadSel, kSelectedRoadThickness);
    }

    if (m_mode != EditorMode::Navigate)
    {
        for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
        {
            const Road& road = m_network->roads[ri];
            if (!IsRoadGuidelineVisible(road))
                continue;
            for (int pi = 0; pi < static_cast<int>(road.points.size()); ++pi)
            {
                ImVec2 sp;
                if (!WorldToScreen(road.points[pi].pos, viewProj, vpW, vpH, sp))
                    continue;
                const bool pointSelected = IsPointSelected(ri, pi);
                const bool roadSelected = IsRoadSelected(ri) || ri == m_activeRoad;
                ImU32 col = colPoint;
                if (roadSelected)
                    col = colRoadSel;
                if (pointSelected)
                    col = colPointSel;
                const float radius = pointSelected ? (kRadius + 2.0f) : kRadius;
                dl->AddCircleFilled(sp, radius, col, 20);
            }
        }
    }

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        if (!IsRoadVisible(road) || !(m_showRoadNames || m_showRoadPreviewMetrics) || road.points.empty())
            continue;

        const RoadPreviewMetrics& metrics = GetRoadPreviewMetricsCached(ri);
        const XMFLOAT3 center = metrics.valid ? metrics.center : road.points.front().pos;

        ImVec2 labelPos;
        if (WorldToScreen(center, viewProj, vpW, vpH, labelPos))
        {
            const ImVec2 textPos(labelPos.x + 10.0f, labelPos.y - 8.0f);
            if (m_showRoadNames && !road.name.empty())
                dl->AddText(textPos, IM_COL32(255, 215, 90, 255), road.name.c_str());

            if (m_showRoadPreviewMetrics)
            {
                char metricsBuf[128] = {};
                sprintf_s(
                    metricsBuf,
                    "%.0fm / %.1f%%(%.1f%%)",
                    std::round(metrics.length),
                    metrics.averageGradePercent,
                    metrics.maxGradePercent);
                const float metricsOffsetY = (m_showRoadNames && !road.name.empty()) ? 10.0f : 0.0f;
                dl->AddText(
                    ImVec2(textPos.x, textPos.y + metricsOffsetY),
                    IM_COL32(180, 245, 255, 255),
                    metricsBuf);
            }
        }
    }

    if (m_marqueeSelecting)
    {
        const float minX = (std::min)(m_marqueeStart.x, m_marqueeEnd.x);
        const float maxX = (std::max)(m_marqueeStart.x, m_marqueeEnd.x);
        const float minY = (std::min)(m_marqueeStart.y, m_marqueeEnd.y);
        const float maxY = (std::max)(m_marqueeStart.y, m_marqueeEnd.y);
        dl->AddRect(
            ImVec2(minX, minY),
            ImVec2(maxX, maxY),
            IM_COL32(255, 220, 80, 220),
            0.0f,
            0,
            1.5f);
        dl->AddRectFilled(
            ImVec2(minX, minY),
            ImVec2(maxX, maxY),
            IM_COL32(255, 220, 80, 35));
    }

    if (m_showRoadGuidelines)
    {
        for (int ii = 0; ii < static_cast<int>(m_network->intersections.size()); ++ii)
        {
            const Intersection& isec = m_network->intersections[ii];
            if (!IsIntersectionVisible(isec))
                continue;
            ImVec2 sp;
            if (!WorldToScreen(isec.pos, viewProj, vpW, vpH, sp))
                continue;
            const bool selected = (ii == m_activeIntersection) || IsIntersectionSelected(ii);
            const ImU32 col = selected ? colIsecSel : colIsec;
            const float radiusPx = (std::max)(2.0f, m_intersectionScreenGizmoRadius);
            const float crossPx = radiusPx * 0.8f;
            dl->AddCircle(sp, radiusPx, col, 24, 2.0f);
            dl->AddLine(ImVec2(sp.x - crossPx, sp.y), ImVec2(sp.x + crossPx, sp.y), col, 2.0f);
            dl->AddLine(ImVec2(sp.x, sp.y - crossPx), ImVec2(sp.x, sp.y + crossPx), col, 2.0f);
        }

        if (m_hoverSnapIntersection >= 0 &&
            m_hoverSnapIntersection < static_cast<int>(m_network->intersections.size()))
        {
            ImVec2 sp;
            if (WorldToScreen(m_network->intersections[m_hoverSnapIntersection].pos,
                              viewProj, vpW, vpH, sp))
            {
                dl->AddCircle(sp, 22.0f, colSnap, 32, 3.0f);
                dl->AddCircle(sp, 30.0f, IM_COL32(255, 240, 80, 120), 32, 2.0f);
                dl->AddText(ImVec2(sp.x + 18.0f, sp.y - 24.0f), colSnap, "SNAP");
            }
        }
    }

    if (m_showIntersectionNames)
    {
        for (int ii = 0; ii < static_cast<int>(m_network->intersections.size()); ++ii)
        {
            const Intersection& isec = m_network->intersections[ii];
            if (!IsIntersectionVisible(isec))
                continue;
            ImVec2 sp;
            if (!WorldToScreen(isec.pos, viewProj, vpW, vpH, sp))
                continue;
            const bool selected = (ii == m_activeIntersection) || IsIntersectionSelected(ii);
            const ImU32 col = selected ? colIsecSel : colIsec;
            const float radiusPx = (std::max)(2.0f, m_intersectionScreenGizmoRadius);
            const float crossPx = radiusPx * 0.8f;
            dl->AddText(ImVec2(sp.x + radiusPx + 2.0f, sp.y - crossPx), col, isec.name.c_str());
        }
    }

    if (m_mode == EditorMode::PointEdit)
    {
        XMFLOAT3 p;
        if (GetActiveGizmoPivot(p))
        {
            ImVec2 pivot;
            if (WorldToScreen(p, viewProj, vpW, vpH, pivot))
            {
                dl->AddCircle(pivot, 8.0f, IM_COL32(255,255,255,220), 24, 2.0f);
                dl->AddCircleFilled(pivot, 3.0f, IM_COL32(255,255,255,220), 16);
                if (m_rotateYMode)
                {
                    constexpr int kSegments = 64;
                    std::vector<ImVec2> ringPoints;
                    ringPoints.reserve(kSegments + 1);
                    const float radius = ComputeRotationGizmoRadius(p, viewProj, vpW, vpH);
                    for (int segmentIndex = 0; segmentIndex <= kSegments; ++segmentIndex)
                    {
                        const float t = XM_2PI * static_cast<float>(segmentIndex) / static_cast<float>(kSegments);
                        ImVec2 ringPoint;
                        if (!WorldToScreen(
                                { p.x + cosf(t) * radius, p.y, p.z + sinf(t) * radius },
                                viewProj,
                                vpW,
                                vpH,
                                ringPoint))
                            continue;
                        ringPoints.push_back(ringPoint);
                    }
                    if (ringPoints.size() >= 2)
                        dl->AddPolyline(ringPoints.data(), static_cast<int>(ringPoints.size()), IM_COL32(255, 215, 60, 240), ImDrawFlags_None, 2.0f);
                    dl->AddText(ImVec2(pivot.x + 10.0f, pivot.y - 22.0f), IM_COL32(255, 215, 60, 240), "RY");
                }
                else if (m_scaleXZMode)
                {
                    const float radius = ComputeScaleGizmoRadius(p, viewProj, vpW, vpH);
                    ImVec2 corners[4];
                    bool valid = true;
                    const XMFLOAT3 cornersWorld[4] =
                    {
                        { p.x + radius, p.y, p.z + radius },
                        { p.x - radius, p.y, p.z + radius },
                        { p.x - radius, p.y, p.z - radius },
                        { p.x + radius, p.y, p.z - radius }
                    };
                    for (int i = 0; i < 4; ++i)
                        valid = valid && WorldToScreen(cornersWorld[i], viewProj, vpW, vpH, corners[i]);
                    if (valid)
                        dl->AddPolyline(corners, 4, colScaleXZ, ImDrawFlags_Closed, 2.0f);
                    dl->AddText(ImVec2(pivot.x + 10.0f, pivot.y - 22.0f), colScaleXZ, "SXZ");
                }
                else
                {
                    struct AxisOverlay
                    {
                        XMFLOAT3 end;
                        ImU32 color;
                        const char* label;
                    };
                    AxisOverlay axes[] =
                    {
                        { { p.x + ComputeGizmoAxisLength(p, GizmoAxis::X, viewProj, vpW, vpH), p.y, p.z }, colAxisX, "X" },
                        { { p.x, p.y + ComputeGizmoAxisLength(p, GizmoAxis::Y, viewProj, vpW, vpH), p.z }, colAxisY, "Y" },
                        { { p.x, p.y, p.z + ComputeGizmoAxisLength(p, GizmoAxis::Z, viewProj, vpW, vpH) }, colAxisZ, "Z" },
                    };

                    for (const AxisOverlay& axis : axes)
                    {
                        ImVec2 end;
                        if (!WorldToScreen(axis.end, viewProj, vpW, vpH, end))
                            continue;
                        dl->AddLine(pivot, end, axis.color, 2.0f);
                        dl->AddText(ImVec2(end.x + 4.0f, end.y - 8.0f), axis.color, axis.label);
                    }
                }
            }
        }
    }

    if (m_mode == EditorMode::IntersectionEdit &&
        m_activeIntersection >= 0 &&
        m_activeIntersection < static_cast<int>(m_network->intersections.size()))
    {
        const Intersection& isec = m_network->intersections[m_activeIntersection];
        const XMFLOAT3 p = isec.pos;
        ImVec2 pivot;
        if (WorldToScreen(p, viewProj, vpW, vpH, pivot))
        {
            dl->AddCircle(pivot, 8.0f, IM_COL32(255,255,255,220), 24, 2.0f);
            dl->AddCircleFilled(pivot, 3.0f, IM_COL32(255,255,255,220), 16);

            struct AxisOverlay
            {
                XMFLOAT3 end;
                ImU32 color;
                const char* label;
            };
            AxisOverlay axes[] =
            {
                { { p.x + ComputeGizmoAxisLength(p, GizmoAxis::X, viewProj, vpW, vpH), p.y, p.z }, colAxisX, "X" },
                { { p.x, p.y + ComputeGizmoAxisLength(p, GizmoAxis::Y, viewProj, vpW, vpH), p.z }, colAxisY, "Y" },
                { { p.x, p.y, p.z + ComputeGizmoAxisLength(p, GizmoAxis::Z, viewProj, vpW, vpH) }, colAxisZ, "Z" },
            };

            for (const AxisOverlay& axis : axes)
            {
                ImVec2 end;
                if (!WorldToScreen(axis.end, viewProj, vpW, vpH, end))
                    continue;
                dl->AddLine(pivot, end, axis.color, 2.0f);
                dl->AddText(ImVec2(end.x + 4.0f, end.y - 8.0f), axis.color, axis.label);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// DrawUI
// ---------------------------------------------------------------------------
void PolylineEditor::DrawUI(ID3D11Device* /*device*/,
                            bool* showRoadEditorWindow,
                            bool* showPropertiesWindow,
                            SavedImGuiWindowLayout* roadEditorLayout,
                            SavedImGuiWindowLayout* propertiesLayout,
                            bool applyManagedLayouts)
{
    m_network->EnsureDefaultGroup();
    SanitizeSelection();

    if (showRoadEditorWindow != nullptr && *showRoadEditorWindow)
    {
        const ImGuiViewport* mainViewport = ImGui::GetMainViewport();
        const ImVec2 workPos = mainViewport->WorkPos;
        const ImVec2 workSize = mainViewport->WorkSize;
        const ImVec2 defaultPos(workPos.x + 16.0f, workPos.y + 40.0f);
        const ImVec2 defaultSize(360.0f, (std::max)(360.0f, workSize.y - 120.0f));
        if (roadEditorLayout != nullptr)
        {
            if (applyManagedLayouts)
            {
                const ImVec2 pos = roadEditorLayout->valid
                    ? ImVec2(roadEditorLayout->x, roadEditorLayout->y)
                    : defaultPos;
                const ImVec2 size = roadEditorLayout->valid
                    ? ImVec2(roadEditorLayout->width, roadEditorLayout->height)
                    : defaultSize;
                ImGui::SetNextWindowPos(pos, ImGuiCond_Always);
                ImGui::SetNextWindowSize(size, ImGuiCond_Always);
            }
            else if (roadEditorLayout->valid)
            {
                ImGui::SetNextWindowPos(
                    ImVec2(roadEditorLayout->x, roadEditorLayout->y),
                    ImGuiCond_FirstUseEver);
                ImGui::SetNextWindowSize(
                    ImVec2(roadEditorLayout->width, roadEditorLayout->height),
                    ImGuiCond_FirstUseEver);
            }
            else
            {
                ImGui::SetNextWindowPos(defaultPos, ImGuiCond_FirstUseEver);
                ImGui::SetNextWindowSize(defaultSize, ImGuiCond_FirstUseEver);
            }
        }
        ImGui::Begin(u8"\u9053\u8DEF\u30A8\u30C7\u30A3\u30BF", showRoadEditorWindow);
        if (roadEditorLayout != nullptr)
        {
            const ImVec2 pos = ImGui::GetWindowPos();
            const ImVec2 size = ImGui::GetWindowSize();
            roadEditorLayout->valid = true;
            roadEditorLayout->x = pos.x;
            roadEditorLayout->y = pos.y;
            roadEditorLayout->width = size.x;
            roadEditorLayout->height = size.y;
        }

        // Mode toolbar
        {
        bool navActive  = (m_mode == EditorMode::Navigate);
        bool drawActive = (m_mode == EditorMode::PolylineDraw);
        bool editActive = (m_mode == EditorMode::PointEdit);
        bool verticalActive = (m_mode == EditorMode::VerticalCurveEdit);
        bool bankActive = (m_mode == EditorMode::BankAngleEdit);
        bool laneActive = (m_mode == EditorMode::LaneEdit);
        bool isecDrawActive = (m_mode == EditorMode::IntersectionDraw);
        bool isecEditActive = (m_mode == EditorMode::IntersectionEdit);
        bool pathActive = (m_mode == EditorMode::Pathfinding);

        const auto drawModeToggle = [](const char* label, bool active, const ImVec2& size = ImVec2(0.0f, 0.0f))
        {
            const ImVec4 activeColor = ImVec4(0.20f, 0.55f, 0.32f, 1.0f);
            const ImVec4 idleColor = ImVec4(0.22f, 0.24f, 0.28f, 1.0f);
            const ImVec4 hoverColor = active
                ? ImVec4(0.24f, 0.62f, 0.37f, 1.0f)
                : ImVec4(0.28f, 0.31f, 0.36f, 1.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 7.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, active ? 1.0f : 0.0f);
            ImGui::PushStyleColor(ImGuiCol_Button, active ? activeColor : idleColor);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, hoverColor);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, active ? activeColor : hoverColor);
            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.42f, 0.78f, 0.56f, active ? 0.95f : 0.0f));
            ImGui::PushStyleColor(ImGuiCol_Text, active ? ImVec4(1.0f, 1.0f, 1.0f, 1.0f)
                                                        : ImVec4(0.84f, 0.87f, 0.90f, 1.0f));
            const bool pressed = ImGui::Button(label, size);
            ImGui::PopStyleColor(5);
            ImGui::PopStyleVar(2);
            return pressed;
        };

        const float toggleSpacing = ImGui::GetStyle().ItemSpacing.x;
        const float toggleWidth = (ImGui::GetContentRegionAvail().x - toggleSpacing * 4.0f) / 5.0f;

        if (drawModeToggle(u8"\u30AA\u30D6\u30B8\u30A7\u30AF\u30C8", navActive, ImVec2(toggleWidth, 0.0f)))
            SetMode(EditorMode::Navigate);

        ImGui::SameLine();

        ImGui::BeginDisabled(!m_showRoadGuidelines);
        if (drawModeToggle(u8"\u30DD\u30A4\u30F3\u30C8\u7DE8\u96C6", editActive, ImVec2(toggleWidth, 0.0f)))
            SetMode(EditorMode::PointEdit);
        ImGui::EndDisabled();

        ImGui::SameLine();

        if (drawModeToggle(u8"\u7E26\u65AD\u66F2\u7DDA\u7DE8\u96C6", verticalActive, ImVec2(toggleWidth, 0.0f)))
            SetMode(EditorMode::VerticalCurveEdit);

        ImGui::SameLine();

        if (drawModeToggle(u8"\u30D0\u30F3\u30AF\u89D2\u7DE8\u96C6", bankActive, ImVec2(toggleWidth, 0.0f)))
            SetMode(EditorMode::BankAngleEdit);

        ImGui::SameLine();

        if (drawModeToggle(u8"\u8ECA\u7DDA\u7DE8\u96C6", laneActive, ImVec2(toggleWidth, 0.0f)))
            SetMode(EditorMode::LaneEdit);

        ImGui::NewLine();

        ImGui::BeginDisabled(!m_showRoadGuidelines);
        if (drawActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button(u8"\u9053\u8DEF\u4F5C\u6210")) StartNewRoad();
        if (drawActive) ImGui::PopStyleColor();
        ImGui::EndDisabled();

        ImGui::SameLine();

        if (isecDrawActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button(u8"\u4EA4\u5DEE\u70B9\u4F5C\u6210")) SetMode(EditorMode::IntersectionDraw);
        if (isecDrawActive) ImGui::PopStyleColor();

        ImGui::SameLine();

        if (isecEditActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button(u8"\u4EA4\u5DEE\u70B9\u7DE8\u96C6")) SetMode(EditorMode::IntersectionEdit);
        if (isecEditActive) ImGui::PopStyleColor();

        ImGui::SameLine();

        if (pathActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button(u8"\u7D4C\u8DEF\u63A2\u7D22"))
        {
            if (pathActive)
                SetMode(EditorMode::Navigate);
            else
                SetMode(EditorMode::Pathfinding);
        }
        if (pathActive) ImGui::PopStyleColor();
    }

    ImGui::Separator();

    // Mode hint
    switch (m_mode)
    {
    case EditorMode::Navigate:
        ImGui::TextDisabled(u8"\u30AA\u30D6\u30B8\u30A7\u30AF\u30C8\u9078\u629E\u30E2\u30FC\u30C9");
        ImGui::TextDisabled(u8"\u9053\u8DEF\u3092\u30AF\u30EA\u30C3\u30AF: \u9053\u8DEF\u9078\u629E");
        ImGui::TextDisabled(u8"Ctrl+\u30AF\u30EA\u30C3\u30AF: \u9053\u8DEF\u3092\u8907\u6570\u9078\u629E");
        ImGui::TextDisabled(u8"\u9078\u629E\u4E2D\u9053\u8DEF\u306E\u30DD\u30A4\u30F3\u30C8\u306F\u76F4\u63A5\u9078\u629E\u53EF\u80FD");
        ImGui::TextDisabled(u8"\u7A7A\u304D\u9818\u57DF\u3092\u30C9\u30E9\u30C3\u30B0: \u77E9\u5F62\u9078\u629E");
        ImGui::TextDisabled(u8"W \u30AD\u30FC: \u79FB\u52D5\u30AE\u30BA\u30E2");
        ImGui::TextDisabled(u8"E \u30AD\u30FC: Y\u56DE\u8EE2\u30AE\u30BA\u30E2");
        ImGui::TextDisabled(u8"R \u30AD\u30FC: XZ\u62E1\u5927\u30AE\u30BA\u30E2");
        break;
    case EditorMode::PolylineDraw:
        ImGui::TextColored(ImVec4(0.2f,1,0.4f,1), u8"\u4F5C\u6210\u4E2D: %s",
            (m_activeRoad >= 0 &&
             m_activeRoad < static_cast<int>(m_network->roads.size()))
            ? m_network->roads[m_activeRoad].name.c_str() : "");
        ImGui::TextDisabled(u8"\u5DE6\u30AF\u30EA\u30C3\u30AF: \u30DD\u30A4\u30F3\u30C8\u8FFD\u52A0");
        ImGui::TextDisabled(u8"Enter: \u78BA\u5B9A  Esc: \u30AD\u30E3\u30F3\u30BB\u30EB");
        ImGui::SliderFloat(u8"\u5E45", &m_defaultWidth, 0.5f, 20.0f);
        break;
    case EditorMode::PointEdit:
        ImGui::TextColored(ImVec4(1,0.8f,0.2f,1), u8"\u30DD\u30A4\u30F3\u30C8\u7DE8\u96C6");
        ImGui::TextDisabled(u8"\u30DD\u30A4\u30F3\u30C8\u3092\u30AF\u30EA\u30C3\u30AF: \u9078\u629E");
        ImGui::TextDisabled(u8"Ctrl+\u30AF\u30EA\u30C3\u30AF: \u8FFD\u52A0/\u89E3\u9664");
        ImGui::TextDisabled(u8"\u7A7A\u304D\u9818\u57DF\u3092\u30C9\u30E9\u30C3\u30B0: \u77E9\u5F62\u9078\u629E");
        ImGui::TextDisabled(u8"Ctrl+Shift \u77E9\u5F62: \u9078\u629E\u89E3\u9664");
        ImGui::TextDisabled(u8"\u4E2D\u592E\u30C9\u30E9\u30C3\u30B0: \u30B9\u30AF\u30EA\u30FC\u30F3\u5E73\u9762\u3067\u79FB\u52D5");
        ImGui::TextDisabled(u8"X/Y/Z \u8EF8\u30AF\u30EA\u30C3\u30AF: \u8EF8\u65B9\u5411\u306B\u79FB\u52D5");
        ImGui::TextDisabled(u8"E: Y\u56DE\u8EE2\u30EA\u30F3\u30B0");
        ImGui::TextDisabled(u8"R: XZ\u62E1\u5927\u30EA\u30F3\u30B0");
        ImGui::TextDisabled(u8"\u7AEF\u70B9\u304C\u4EA4\u5DEE\u70B9\u4ED8\u8FD1: \u30B9\u30CA\u30C3\u30D7/\u63A5\u7D9A");
        ImGui::TextDisabled(u8"Delete: \u30DD\u30A4\u30F3\u30C8\u524A\u9664");
        break;
    case EditorMode::VerticalCurveEdit:
        ImGui::TextColored(ImVec4(0.95f,0.55f,0.25f,1), u8"\u7E26\u65AD\u66F2\u7DDA\u7DE8\u96C6");
        if (m_activeRoad >= 0 && m_activeRoad < static_cast<int>(m_network->roads.size()))
            ImGui::TextColored(ImVec4(1.0f,0.9f,0.55f,1), u8"\u7DE8\u96C6\u5BFE\u8C61: %s", m_network->roads[m_activeRoad].name.c_str());
        else
            ImGui::TextDisabled(u8"\u307E\u305A\u9053\u8DEF\u30DD\u30EA\u30E9\u30A4\u30F3\u3092\u30AF\u30EA\u30C3\u30AF\u3057\u3066\u7DE8\u96C6\u5BFE\u8C61\u3092\u9078\u629E");
        ImGui::TextDisabled(u8"\u5225\u306E\u9053\u8DEF\u30DD\u30EA\u30E9\u30A4\u30F3\u3092\u30AF\u30EA\u30C3\u30AF: \u7DE8\u96C6\u5BFE\u8C61\u3092\u5207\u308A\u66FF\u3048");
        ImGui::TextDisabled(u8"\u30D7\u30EC\u30D3\u30E5\u30FC\u30AB\u30FC\u30D6\u3092\u30AF\u30EA\u30C3\u30AF: \u7E26\u65AD\u66F2\u7DDA\u30DD\u30A4\u30F3\u30C8\u8FFD\u52A0");
        ImGui::TextDisabled(u8"\u65E2\u5B58\u30DD\u30A4\u30F3\u30C8\u3092\u30AF\u30EA\u30C3\u30AF: \u9078\u629E/\u30C9\u30E9\u30C3\u30B0");
        ImGui::TextDisabled(u8"Delete: \u9078\u629E\u4E2D\u30DD\u30A4\u30F3\u30C8\u524A\u9664");
        ImGui::TextDisabled(u8"\u30D7\u30ED\u30D1\u30C6\u30A3\u3067 u_coord / vcl / offset \u3092\u7DE8\u96C6");
        break;
    case EditorMode::BankAngleEdit:
        ImGui::TextColored(ImVec4(0.35f,0.78f,1.0f,1), u8"\u30D0\u30F3\u30AF\u89D2\u7DE8\u96C6");
        if (m_activeRoad >= 0 && m_activeRoad < static_cast<int>(m_network->roads.size()))
            ImGui::TextColored(ImVec4(1.0f,0.9f,0.55f,1), u8"\u7DE8\u96C6\u5BFE\u8C61: %s", m_network->roads[m_activeRoad].name.c_str());
        else
            ImGui::TextDisabled(u8"\u307E\u305A\u9053\u8DEF\u30DD\u30EA\u30E9\u30A4\u30F3\u3092\u30AF\u30EA\u30C3\u30AF\u3057\u3066\u7DE8\u96C6\u5BFE\u8C61\u3092\u9078\u629E");
        ImGui::TextDisabled(u8"\u5225\u306E\u9053\u8DEF\u30DD\u30EA\u30E9\u30A4\u30F3\u3092\u30AF\u30EA\u30C3\u30AF: \u7DE8\u96C6\u5BFE\u8C61\u3092\u5207\u308A\u66FF\u3048");
        ImGui::TextDisabled(u8"\u30D7\u30EC\u30D3\u30E5\u30FC\u30AB\u30FC\u30D6\u3092\u30AF\u30EA\u30C3\u30AF: \u30D0\u30F3\u30AF\u89D2\u30DD\u30A4\u30F3\u30C8\u8FFD\u52A0");
        ImGui::TextDisabled(u8"\u65E2\u5B58\u30DD\u30A4\u30F3\u30C8\u3092\u30AF\u30EA\u30C3\u30AF: \u9078\u629E/\u30C9\u30E9\u30C3\u30B0");
        ImGui::TextDisabled(u8"Delete: \u9078\u629E\u4E2D\u30DD\u30A4\u30F3\u30C8\u524A\u9664");
        ImGui::TextDisabled(u8"\u30D7\u30ED\u30D1\u30C6\u30A3\u3067 u_coord / \u60F3\u5B9A\u901F\u5EA6 / \u30D0\u30F3\u30AF\u89D2 \u3092\u7DE8\u96C6");
        break;
    case EditorMode::LaneEdit:
        ImGui::TextColored(ImVec4(0.48f,0.92f,0.38f,1), u8"\u8ECA\u7DDA\u7DE8\u96C6");
        if (m_activeRoad >= 0 && m_activeRoad < static_cast<int>(m_network->roads.size()))
            ImGui::TextColored(ImVec4(1.0f,0.9f,0.55f,1), u8"\u7DE8\u96C6\u5BFE\u8C61: %s", m_network->roads[m_activeRoad].name.c_str());
        else
            ImGui::TextDisabled(u8"\u307E\u305A\u9053\u8DEF\u30DD\u30EA\u30E9\u30A4\u30F3\u3092\u30AF\u30EA\u30C3\u30AF\u3057\u3066\u7DE8\u96C6\u5BFE\u8C61\u3092\u9078\u629E");
        ImGui::TextDisabled(u8"\u5225\u306E\u9053\u8DEF\u30DD\u30EA\u30E9\u30A4\u30F3\u3092\u30AF\u30EA\u30C3\u30AF: \u7DE8\u96C6\u5BFE\u8C61\u3092\u5207\u308A\u66FF\u3048");
        ImGui::TextDisabled(u8"\u30D7\u30EC\u30D3\u30E5\u30FC\u30AB\u30FC\u30D6\u3092\u30AF\u30EA\u30C3\u30AF: \u8ECA\u7DDA\u30BB\u30AF\u30B7\u30E7\u30F3\u30DD\u30A4\u30F3\u30C8\u8FFD\u52A0");
        ImGui::TextDisabled(u8"\u65E2\u5B58\u30DD\u30A4\u30F3\u30C8\u3092\u30AF\u30EA\u30C3\u30AF: \u9078\u629E/\u30C9\u30E9\u30C3\u30B0");
        ImGui::TextDisabled(u8"Delete: \u9078\u629E\u4E2D\u30DD\u30A4\u30F3\u30C8\u524A\u9664");
        ImGui::TextDisabled(u8"\u30D7\u30ED\u30D1\u30C6\u30A3\u3067 u_coord / \u8ECA\u7DDA\u4F7F\u7528 / \u5E45\u3092\u7DE8\u96C6");
        break;
    case EditorMode::IntersectionDraw:
        ImGui::TextColored(ImVec4(0.2f,0.9f,1.0f,1), u8"\u4EA4\u5DEE\u70B9\u4F5C\u6210");
        ImGui::TextDisabled(u8"\u5DE6\u30AF\u30EA\u30C3\u30AF: \u4EA4\u5DEE\u70B9\u914D\u7F6E");
        ImGui::TextDisabled(u8"Esc: \u30AA\u30D6\u30B8\u30A7\u30AF\u30C8\u30E2\u30FC\u30C9\u3078\u623B\u308B");
        break;
    case EditorMode::IntersectionEdit:
        ImGui::TextColored(ImVec4(0.2f,0.9f,1.0f,1), u8"\u4EA4\u5DEE\u70B9\u7DE8\u96C6");
        ImGui::TextDisabled(u8"\u30AF\u30EA\u30C3\u30AF: \u4EA4\u5DEE\u70B9\u9078\u629E");
        ImGui::TextDisabled(u8"Delete: \u4EA4\u5DEE\u70B9\u524A\u9664");
        break;
    case EditorMode::Pathfinding:
        ImGui::TextColored(ImVec4(1.0f,0.7f,0.2f,1), u8"\u7D4C\u8DEF\u63A2\u7D22");
        ImGui::TextDisabled(u8"\u5730\u5F62\u4E0A\u306E\u59CB\u70B9\u3068\u7D42\u70B9\u3092\u8ABF\u6574");
        ImGui::TextDisabled(u8"\u30B0\u30EA\u30C3\u30C9\u9593\u9694\u3068\u6700\u5927\u52FE\u914D\u3092\u8ABF\u6574");
        ImGui::TextDisabled(u8"\u30D7\u30EC\u30D3\u30E5\u30FC\u3092\u78BA\u8A8D\u3057\u3066\u9053\u8DEF\u3078\u9069\u7528");
        break;
    }

    ImGui::Separator();
    bool snapToTerrain = m_snapToTerrain;
    if (ImGui::Checkbox(u8"\u5730\u5F62\u306B\u30B9\u30CA\u30C3\u30D7", &snapToTerrain))
    {
        PushUndoState();
        m_snapToTerrain = snapToTerrain;
        if (m_snapToTerrain && m_terrain && m_terrain->IsReady())
        {
            if (!m_selectedPoints.empty())
            {
                for (const PointRef& pointRef : m_selectedPoints)
                {
                    if (pointRef.roadIndex < 0 ||
                        pointRef.roadIndex >= static_cast<int>(m_network->roads.size()))
                        continue;
                    Road& road = m_network->roads[pointRef.roadIndex];
                    const int pointIndex = pointRef.pointIndex;
                    if (pointIndex < 0 || pointIndex >= static_cast<int>(road.points.size()))
                        continue;
                    road.points[pointIndex].pos.y =
                        m_terrain->GetHeightAt(road.points[pointIndex].pos.x,
                                               road.points[pointIndex].pos.z);
                }
            }

            for (int intersectionIndex : m_selectedIntersections)
            {
                if (intersectionIndex < 0 ||
                    intersectionIndex >= static_cast<int>(m_network->intersections.size()))
                    continue;
                Intersection& isec = m_network->intersections[intersectionIndex];
                isec.pos.y = m_terrain->GetHeightAt(isec.pos.x, isec.pos.z);
                SyncRoadConnectionsForIntersection(intersectionIndex);
            }
        }
    }
    bool snapToPoints = m_snapToPoints;
    if (ImGui::Checkbox(u8"\u30DD\u30A4\u30F3\u30C8\u306B\u30B9\u30CA\u30C3\u30D7", &snapToPoints))
    {
        PushUndoState();
        m_snapToPoints = snapToPoints;
    }
    ImGui::TextDisabled(m_snapToTerrain
        ? u8"\u4E2D\u592E/XZ \u30AE\u30BA\u30E2\u79FB\u52D5\u306F\u5730\u5F62\u3078\u30B9\u30CA\u30C3\u30D7\u3057\u307E\u3059"
        : u8"\u30DD\u30A4\u30F3\u30C8\u3068\u4EA4\u5DEE\u70B9\u306F 3D \u7A7A\u9593\u5185\u3092\u81EA\u7531\u306B\u79FB\u52D5\u3057\u307E\u3059");
        ImGui::TextDisabled(m_snapToPoints
            ? u8"\u4E2D\u592E\u79FB\u52D5\u4E2D\u306F\u8FD1\u304F\u306E\u9802\u70B9\u307E\u305F\u306F\u4EA4\u5DEE\u70B9\u3078\u5438\u7740\u3057\u307E\u3059"
            : u8"\u30DD\u30A4\u30F3\u30C8\u30B9\u30CA\u30C3\u30D7\u306F\u7121\u52B9\u3067\u3059");
        ImGui::End();
    }

    if (showPropertiesWindow != nullptr && *showPropertiesWindow)
    {
        const ImGuiViewport* mainViewport = ImGui::GetMainViewport();
        const ImVec2 workPos = mainViewport->WorkPos;
        const ImVec2 workSize = mainViewport->WorkSize;
        const ImVec2 defaultPos(
            workPos.x + workSize.x - 396.0f,
            workPos.y + 40.0f);
        const ImVec2 defaultSize(380.0f, (std::max)(420.0f, workSize.y - 120.0f));
        if (propertiesLayout != nullptr)
        {
            if (applyManagedLayouts)
            {
                const ImVec2 pos = propertiesLayout->valid
                    ? ImVec2(propertiesLayout->x, propertiesLayout->y)
                    : defaultPos;
                const ImVec2 size = propertiesLayout->valid
                    ? ImVec2(propertiesLayout->width, propertiesLayout->height)
                    : defaultSize;
                ImGui::SetNextWindowPos(pos, ImGuiCond_Always);
                ImGui::SetNextWindowSize(size, ImGuiCond_Always);
            }
            else if (propertiesLayout->valid)
            {
                ImGui::SetNextWindowPos(
                    ImVec2(propertiesLayout->x, propertiesLayout->y),
                    ImGuiCond_FirstUseEver);
                ImGui::SetNextWindowSize(
                    ImVec2(propertiesLayout->width, propertiesLayout->height),
                    ImGuiCond_FirstUseEver);
            }
            else
            {
                ImGui::SetNextWindowPos(defaultPos, ImGuiCond_FirstUseEver);
                ImGui::SetNextWindowSize(defaultSize, ImGuiCond_FirstUseEver);
            }
        }
        ImGui::Begin(u8"\u30D7\u30ED\u30D1\u30C6\u30A3", showPropertiesWindow);
        if (propertiesLayout != nullptr)
        {
            const ImVec2 pos = ImGui::GetWindowPos();
            const ImVec2 size = ImGui::GetWindowSize();
            propertiesLayout->valid = true;
            propertiesLayout->x = pos.x;
            propertiesLayout->y = pos.y;
            propertiesLayout->width = size.x;
            propertiesLayout->height = size.y;
        }

        const bool hasPendingPropertyReveal =
            m_propertyRevealGroup >= 0 ||
            m_propertyRevealRoad >= 0 ||
            m_propertyRevealIntersection >= 0;
        bool propertyRevealHandled = false;

        ImGui::Separator();
        ImGui::Text(u8"\u30B0\u30EB\u30FC\u30D7 (%d)", static_cast<int>(m_network->groups.size()));
    if (ImGui::Button(u8"\u30B0\u30EB\u30FC\u30D7\u8FFD\u52A0"))
    {
        PushUndoState();
        const int newIndex = m_network->AddGroup(
            "Group " + std::to_string(m_network->groups.size()));
        m_activeGroup = newIndex;
        m_statusMessage = u8"グループを作成しました";
    }

    ImGui::BeginChild("GroupTree", ImVec2(0, 210), true);
    int groupToDelete = -1;
    for (int gi = 0; gi < static_cast<int>(m_network->groups.size()); ++gi)
    {
        RoadGroup& group = m_network->groups[gi];
        ImGui::PushID(group.id.c_str());

        const bool revealGroup = hasPendingPropertyReveal && m_propertyRevealGroup == gi;
        const bool revealRoadSection =
            revealGroup && m_propertyRevealRoad >= 0 &&
            m_propertyRevealRoad < static_cast<int>(m_network->roads.size()) &&
            m_network->roads[m_propertyRevealRoad].groupId == group.id;
        const bool revealIntersectionSection =
            revealGroup && m_propertyRevealIntersection >= 0 &&
            m_propertyRevealIntersection < static_cast<int>(m_network->intersections.size()) &&
            m_network->intersections[m_propertyRevealIntersection].groupId == group.id;

        if (revealGroup)
            ImGui::SetNextItemOpen(true, ImGuiCond_Always);

        ImGuiTreeNodeFlags groupFlags =
            ImGuiTreeNodeFlags_DefaultOpen |
            ImGuiTreeNodeFlags_OpenOnArrow |
            ((gi == m_activeGroup) ? ImGuiTreeNodeFlags_Selected : 0);
        const bool groupOpen = ImGui::TreeNodeEx("Group", groupFlags, "%s", group.name.c_str());
        if (revealGroup)
            ImGui::SetScrollHereY(0.2f);
        if (ImGui::IsItemClicked())
            m_activeGroup = gi;

        ImGui::SameLine();
        bool groupVisible = group.visible;
        if (ImGui::Checkbox("##visible", &groupVisible))
        {
            PushUndoState();
            group.visible = groupVisible;
        }
        ImGui::SameLine();
        bool groupLocked = group.locked;
        if (ImGui::Checkbox("##locked", &groupLocked))
        {
            PushUndoState();
            group.locked = groupLocked;
        }

        if (ImGui::BeginPopupContextItem("GroupContext"))
        {
            if (ImGui::MenuItem(u8"\u30B0\u30EB\u30FC\u30D7\u524A\u9664", nullptr, false,
                                static_cast<int>(m_network->groups.size()) > 1))
            {
                groupToDelete = gi;
            }
            ImGui::EndPopup();
        }

        if (groupOpen)
        {
            if (revealRoadSection)
                ImGui::SetNextItemOpen(true, ImGuiCond_Always);
            if (ImGui::TreeNodeEx("Roads", ImGuiTreeNodeFlags_DefaultOpen,
                                  u8"\u9053\u8DEF (%d)",
                                  static_cast<int>(std::count_if(
                                      m_network->roads.begin(),
                                      m_network->roads.end(),
                                      [&group](const Road& road)
                                      {
                                          return road.groupId == group.id;
                                      }))))
            {
                ImGui::BeginDisabled(!m_showRoadGuidelines);
                for (int i = 0; i < static_cast<int>(m_network->roads.size()); ++i)
                {
                    Road& road = m_network->roads[i];
                    if (road.groupId != group.id)
                        continue;

                    const bool selected = IsRoadSelected(i);
                    ImGui::PushID(road.id.empty() ? i : std::hash<std::string>{}(road.id));
                    if (ImGui::Selectable(road.name.c_str(), selected))
                    {
                        if (ImGui::GetIO().KeyCtrl)
                            ToggleRoadSelection(i);
                        else
                            SelectSingleRoad(i);
                        ClearPointSelection();
                        ClearIntersectionSelection();
                        m_activeIntersection = -1;
                        m_activeGroup = gi;
                    }
                    if (m_propertyRevealRoad == i)
                    {
                        ImGui::SetScrollHereY(0.25f);
                        propertyRevealHandled = true;
                    }
                    if (selected && ImGui::BeginPopupContextItem())
                    {
                        if (ImGui::MenuItem(u8"\u9053\u8DEF\u524A\u9664"))
                        {
                            PushUndoState();
                            m_network->RemoveRoad(i);
                            SanitizeSelection();
                        }
                        ImGui::EndPopup();
                    }
                    ImGui::PopID();
                }
                ImGui::EndDisabled();
                ImGui::TreePop();
            }

            if (revealIntersectionSection)
                ImGui::SetNextItemOpen(true, ImGuiCond_Always);
            if (ImGui::TreeNodeEx("Intersections", ImGuiTreeNodeFlags_DefaultOpen,
                                  u8"\u4EA4\u5DEE\u70B9 (%d)",
                                  static_cast<int>(std::count_if(
                                      m_network->intersections.begin(),
                                      m_network->intersections.end(),
                                      [&group](const Intersection& intersection)
                                      {
                                          return intersection.groupId == group.id;
                                      }))))
            {
                for (int i = 0; i < static_cast<int>(m_network->intersections.size()); ++i)
                {
                    Intersection& isec = m_network->intersections[i];
                    if (isec.groupId != group.id)
                        continue;

                    const bool selected = (i == m_activeIntersection);
                    ImGui::PushID(isec.id.empty() ? i : std::hash<std::string>{}(isec.id));
                    if (ImGui::Selectable(isec.name.c_str(), selected))
                    {
                        SelectSingleIntersection(i);
                        ClearRoadSelection();
                        ClearPointSelection();
                        m_activeGroup = gi;
                    }
                    if (m_propertyRevealIntersection == i)
                    {
                        ImGui::SetScrollHereY(0.25f);
                        propertyRevealHandled = true;
                    }
                    if (selected && ImGui::BeginPopupContextItem())
                    {
                        if (ImGui::MenuItem(u8"\u4EA4\u5DEE\u70B9\u524A\u9664"))
                        {
                            PushUndoState();
                            m_network->RemoveIntersection(i);
                            if (m_activeIntersection >= static_cast<int>(m_network->intersections.size()))
                                m_activeIntersection = -1;
                        }
                        ImGui::EndPopup();
                    }
                    ImGui::PopID();
                }
                ImGui::TreePop();
            }
            ImGui::TreePop();
        }

        ImGui::PopID();
    }
    ImGui::EndChild();
    if (groupToDelete >= 0)
    {
        PushUndoState();
        m_network->RemoveGroup(groupToDelete);
        SanitizeSelection();
        m_statusMessage = u8"グループを削除しました";
    }

    if (propertyRevealHandled)
        ClearPropertyReveal();

    if (m_activeGroup >= 0 &&
        m_activeGroup < static_cast<int>(m_network->groups.size()))
    {
        RoadGroup& group = m_network->groups[m_activeGroup];
        char groupNameBuf[128] = {};
        strncpy_s(groupNameBuf, sizeof(groupNameBuf), group.name.c_str(), _TRUNCATE);
        ImGui::Text(u8"\u30A2\u30AF\u30C6\u30A3\u30D6\u30B0\u30EB\u30FC\u30D7");
        if (ImGui::InputText(u8"\u540D\u524D##group", groupNameBuf, sizeof(groupNameBuf)))
        {
            PushUndoState();
            group.name = groupNameBuf;
        }
        bool groupVisible = group.visible;
        if (ImGui::Checkbox(u8"\u8868\u793A##group", &groupVisible))
        {
            PushUndoState();
            group.visible = groupVisible;
        }
        ImGui::SameLine();
        bool groupLocked = group.locked;
        if (ImGui::Checkbox(u8"\u30ED\u30C3\u30AF##group", &groupLocked))
        {
            PushUndoState();
            group.locked = groupLocked;
        }
    }

    if (m_activeRoad >= 0 &&
        m_activeRoad < static_cast<int>(m_network->roads.size()))
    {
        ImGui::BeginDisabled(!m_showRoadGuidelines);
        Road& road = m_network->roads[m_activeRoad];
        ImGui::Separator();
        ImGui::Text(u8"\u9053\u8DEF");
        if (m_selectedRoads.size() > 1)
            ImGui::TextDisabled(u8"\u9078\u629E\u4E2D\u306E\u9053\u8DEF: %d", static_cast<int>(m_selectedRoads.size()));

        if (m_selectedRoads.size() == 2)
        {
            if (ImGui::Button(u8"\u9078\u629E\u4E2D\u306E\u9053\u8DEF\u3092\u30DE\u30FC\u30B8"))
                MergeSelectedRoads();
        }

        char roadNameBuf[128] = {};
        strncpy_s(roadNameBuf, sizeof(roadNameBuf), road.name.c_str(), _TRUNCATE);
        if (ImGui::InputText(u8"\u540D\u524D##road", roadNameBuf, sizeof(roadNameBuf)))
        {
            PushUndoState();
            road.name = roadNameBuf;
        }

        float defaultWidthLaneLeft1 = road.defaultWidthLaneLeft1;
        if (ImGui::InputFloat(u8"\u30C7\u30D5\u30A9\u30EB\u30C8\u5DE6\u8ECA\u7DDA\u5E45", &defaultWidthLaneLeft1, 0.1f, 1.0f, "%.2f"))
        {
            PushUndoState();
            road.defaultWidthLaneLeft1 = (std::max)(0.0f, defaultWidthLaneLeft1);
            InvalidateRoadPreviewCache(m_activeRoad);
        }

        float defaultWidthLaneRight1 = road.defaultWidthLaneRight1;
        if (ImGui::InputFloat(u8"\u30C7\u30D5\u30A9\u30EB\u30C8\u53F3\u8ECA\u7DDA\u5E45", &defaultWidthLaneRight1, 0.1f, 1.0f, "%.2f"))
        {
            PushUndoState();
            road.defaultWidthLaneRight1 = (std::max)(0.0f, defaultWidthLaneRight1);
            InvalidateRoadPreviewCache(m_activeRoad);
        }

        float defaultWidthLaneCenter = road.defaultWidthLaneCenter;
        if (ImGui::InputFloat(u8"\u30C7\u30D5\u30A9\u30EB\u30C8\u4E2D\u592E\u8ECA\u7DDA\u5E45", &defaultWidthLaneCenter, 0.1f, 1.0f, "%.2f"))
        {
            PushUndoState();
            road.defaultWidthLaneCenter = (std::max)(0.0f, defaultWidthLaneCenter);
            InvalidateRoadPreviewCache(m_activeRoad);
        }

        float defaultFriction = road.defaultFriction;
        if (ImGui::InputFloat(u8"\u30C7\u30D5\u30A9\u30EB\u30C8\u6469\u64E6\u4FC2\u6570", &defaultFriction, 0.01f, 0.05f, "%.3f"))
        {
            PushUndoState();
            road.defaultFriction = (std::max)(0.0f, defaultFriction);
            InvalidateRoadPreviewCache(m_activeRoad);
        }

        float defaultTargetSpeed = road.defaultTargetSpeed;
        if (ImGui::InputFloat(u8"\u30C7\u30D5\u30A9\u30EB\u30C8\u60F3\u5B9A\u901F\u5EA6(km/h)", &defaultTargetSpeed, 1.0f, 10.0f, "%.1f"))
        {
            PushUndoState();
            road.defaultTargetSpeed = (std::max)(0.0f, defaultTargetSpeed);
            InvalidateRoadPreviewCache(m_activeRoad);
        }

        const RoadGroup* currentGroup = m_network->FindGroupById(road.groupId);
        const char* preview = currentGroup ? currentGroup->name.c_str() : "<none>";
        if (ImGui::BeginCombo(u8"\u30B0\u30EB\u30FC\u30D7##road", preview))
        {
            for (int gi = 0; gi < static_cast<int>(m_network->groups.size()); ++gi)
            {
                const RoadGroup& group = m_network->groups[gi];
                const bool selected = (road.groupId == group.id);
                if (ImGui::Selectable(group.name.c_str(), selected))
                {
                    PushUndoState();
                    road.groupId = group.id;
                    m_activeGroup = gi;
                }
                if (selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        if (m_mode == EditorMode::VerticalCurveEdit)
        {
            ImGui::Separator();
            ImGui::Text(u8"\u7E26\u65AD\u66F2\u7DDA");
            if (road.verticalCurve.empty())
            {
                ImGui::TextDisabled(u8"\u307E\u3060\u30DD\u30A4\u30F3\u30C8\u306F\u3042\u308A\u307E\u305B\u3093");
            }
            else
            {
                for (int curveIndex = 0; curveIndex < static_cast<int>(road.verticalCurve.size()); ++curveIndex)
                {
                    VerticalCurvePoint& point = road.verticalCurve[curveIndex];
                    char label[96] = {};
                    sprintf_s(label, "U=%.3f / VCL=%.1f / dz=%.2f", point.uCoord, point.vcl, point.offset);
                    const bool selected = IsVerticalCurveSelected(m_activeRoad, curveIndex);
                    if (ImGui::Selectable(label, selected))
                        SelectSingleVerticalCurvePoint(m_activeRoad, curveIndex);
                }
            }
        }

        if (m_mode == EditorMode::BankAngleEdit)
        {
            ImGui::Separator();
            ImGui::Text(u8"\u30D0\u30F3\u30AF\u89D2");
            if (road.bankAngle.empty())
            {
                ImGui::TextDisabled(u8"\u307E\u3060\u30DD\u30A4\u30F3\u30C8\u306F\u3042\u308A\u307E\u305B\u3093");
            }
            else
            {
                for (int pointIndex = 0; pointIndex < static_cast<int>(road.bankAngle.size()); ++pointIndex)
                {
                    BankAnglePoint& point = road.bankAngle[pointIndex];
                    char label[128] = {};
                    sprintf_s(
                        label,
                        "U=%.3f / %.0fkm / %s / %.1fdeg",
                        point.uCoord,
                        point.targetSpeed,
                        point.overrideBank ? "manual" : "auto",
                        point.bankAngle);
                    const bool selected = IsBankAngleSelected(m_activeRoad, pointIndex);
                    if (ImGui::Selectable(label, selected))
                        SelectSingleBankAnglePoint(m_activeRoad, pointIndex);
                }
            }
        }

        if (m_mode == EditorMode::LaneEdit)
        {
            ImGui::Separator();
            ImGui::Text(u8"\u8ECA\u7DDA\u30BB\u30AF\u30B7\u30E7\u30F3");
            if (road.laneSections.empty())
            {
                ImGui::TextDisabled(u8"\u307E\u3060\u30DD\u30A4\u30F3\u30C8\u306F\u3042\u308A\u307E\u305B\u3093");
            }
            else
            {
                for (int pointIndex = 0; pointIndex < static_cast<int>(road.laneSections.size()); ++pointIndex)
                {
                    LaneSectionPoint& point = road.laneSections[pointIndex];
                    char label[160] = {};
                    sprintf_s(
                        label,
                        "U=%.3f / L2:%s %.1f / L1:%s %.1f / C:%s %.1f / R1:%s %.1f / R2:%s %.1f",
                        point.uCoord,
                        point.useLaneLeft2 ? "on" : "off",
                        point.widthLaneLeft2,
                        point.useLaneLeft1 ? "on" : "off",
                        point.widthLaneLeft1,
                        point.useLaneCenter ? "on" : "off",
                        point.widthLaneCenter,
                        point.useLaneRight1 ? "on" : "off",
                        point.widthLaneRight1,
                        point.useLaneRight2 ? "on" : "off",
                        point.widthLaneRight2);
                    const bool selected = IsLaneSectionSelected(m_activeRoad, pointIndex);
                    if (ImGui::Selectable(label, selected))
                        SelectSingleLaneSectionPoint(m_activeRoad, pointIndex);
                }
            }
        }

        ImGui::EndDisabled();
    }

    VerticalCurveRef selectedVerticalCurve;
    if (m_mode == EditorMode::VerticalCurveEdit &&
        GetPrimarySelectedVerticalCurvePoint(selectedVerticalCurve) &&
        selectedVerticalCurve.roadIndex >= 0 &&
        selectedVerticalCurve.roadIndex < static_cast<int>(m_network->roads.size()))
    {
        Road& road = m_network->roads[selectedVerticalCurve.roadIndex];
        if (selectedVerticalCurve.curveIndex >= 0 &&
            selectedVerticalCurve.curveIndex < static_cast<int>(road.verticalCurve.size()))
        {
            ImGui::Separator();
            ImGui::Text(u8"\u7E26\u65AD\u66F2\u7DDA\u30DD\u30A4\u30F3\u30C8 %d", selectedVerticalCurve.curveIndex);

            VerticalCurvePoint currentPoint = road.verticalCurve[selectedVerticalCurve.curveIndex];
            float uCoord = currentPoint.uCoord;
            if (ImGui::SliderFloat(u8"u_coord", &uCoord, 0.0f, 1.0f, "%.6f"))
            {
                PushUndoState();
                currentPoint.uCoord = std::clamp(uCoord, 0.0f, 1.0f);
                road.verticalCurve[selectedVerticalCurve.curveIndex] = currentPoint;
                InvalidateRoadPreviewCache(selectedVerticalCurve.roadIndex);
                std::sort(
                    road.verticalCurve.begin(),
                    road.verticalCurve.end(),
                    [](const VerticalCurvePoint& a, const VerticalCurvePoint& b)
                    {
                        return a.uCoord < b.uCoord;
                    });
                int newIndex = -1;
                for (int i = 0; i < static_cast<int>(road.verticalCurve.size()); ++i)
                {
                    if (fabsf(road.verticalCurve[i].uCoord - currentPoint.uCoord) <= 1e-5f &&
                        fabsf(road.verticalCurve[i].vcl - currentPoint.vcl) <= 1e-5f &&
                        fabsf(road.verticalCurve[i].offset - currentPoint.offset) <= 1e-5f)
                    {
                        newIndex = i;
                        break;
                    }
                }
                SelectSingleVerticalCurvePoint(selectedVerticalCurve.roadIndex, newIndex);
                if (newIndex >= 0)
                {
                    selectedVerticalCurve.curveIndex = newIndex;
                    currentPoint = road.verticalCurve[newIndex];
                }
            }

            float vcl = currentPoint.vcl;
            if (ImGui::InputFloat(u8"vcl", &vcl, 1.0f, 10.0f, "%.2f"))
            {
                PushUndoState();
                road.verticalCurve[selectedVerticalCurve.curveIndex].vcl = (std::max)(0.0f, vcl);
                InvalidateRoadPreviewCache(selectedVerticalCurve.roadIndex);
            }

            float offset = currentPoint.offset;
            if (ImGui::InputFloat(u8"offset", &offset, 0.1f, 1.0f, "%.2f"))
            {
                PushUndoState();
                road.verticalCurve[selectedVerticalCurve.curveIndex].offset = offset;
                InvalidateRoadPreviewCache(selectedVerticalCurve.roadIndex);
            }

            if (ImGui::Button(u8"\u7E26\u65AD\u66F2\u7DDA\u30DD\u30A4\u30F3\u30C8\u3092\u524A\u9664"))
            {
                PushUndoState();
                road.verticalCurve.erase(road.verticalCurve.begin() + selectedVerticalCurve.curveIndex);
                InvalidateRoadPreviewCache(selectedVerticalCurve.roadIndex);
                ClearVerticalCurveSelection();
                SanitizeSelection();
                m_statusMessage = u8"縦断曲線ポイントを削除しました";
            }
        }
    }

    BankAngleRef selectedBankAngle;
    if (m_mode == EditorMode::BankAngleEdit &&
        GetPrimarySelectedBankAnglePoint(selectedBankAngle) &&
        selectedBankAngle.roadIndex >= 0 &&
        selectedBankAngle.roadIndex < static_cast<int>(m_network->roads.size()))
    {
        Road& road = m_network->roads[selectedBankAngle.roadIndex];
        if (selectedBankAngle.pointIndex >= 0 &&
            selectedBankAngle.pointIndex < static_cast<int>(road.bankAngle.size()))
        {
            ImGui::Separator();
            ImGui::Text(u8"\u30D0\u30F3\u30AF\u89D2\u30DD\u30A4\u30F3\u30C8 %d", selectedBankAngle.pointIndex);

            BankAnglePoint currentPoint = road.bankAngle[selectedBankAngle.pointIndex];
            float uCoord = currentPoint.uCoord;
            if (ImGui::SliderFloat(u8"u_coord##bank", &uCoord, 0.0f, 1.0f, "%.6f"))
            {
                PushUndoState();
                currentPoint.uCoord = std::clamp(uCoord, 0.0f, 1.0f);
                road.bankAngle[selectedBankAngle.pointIndex] = currentPoint;
                InvalidateRoadPreviewCache(selectedBankAngle.roadIndex);
                std::sort(
                    road.bankAngle.begin(),
                    road.bankAngle.end(),
                    [](const BankAnglePoint& a, const BankAnglePoint& b)
                    {
                        return a.uCoord < b.uCoord;
                    });
                int newIndex = -1;
                for (int i = 0; i < static_cast<int>(road.bankAngle.size()); ++i)
                {
                    if (fabsf(road.bankAngle[i].uCoord - currentPoint.uCoord) <= 1e-5f &&
                        fabsf(road.bankAngle[i].targetSpeed - currentPoint.targetSpeed) <= 1e-5f &&
                        road.bankAngle[i].overrideBank == currentPoint.overrideBank &&
                        fabsf(road.bankAngle[i].bankAngle - currentPoint.bankAngle) <= 1e-5f)
                    {
                        newIndex = i;
                        break;
                    }
                }
                SelectSingleBankAnglePoint(selectedBankAngle.roadIndex, newIndex);
                if (newIndex >= 0)
                {
                    selectedBankAngle.pointIndex = newIndex;
                    currentPoint = road.bankAngle[newIndex];
                }
            }

            float targetSpeed = currentPoint.targetSpeed;
            if (ImGui::InputFloat(u8"\u60F3\u5B9A\u901F\u5EA6(km)##bank", &targetSpeed, 1.0f, 10.0f, "%.1f"))
            {
                PushUndoState();
                road.bankAngle[selectedBankAngle.pointIndex].targetSpeed = (std::max)(0.0f, targetSpeed);
                InvalidateRoadPreviewCache(selectedBankAngle.roadIndex);
            }

            bool overrideBank = currentPoint.overrideBank;
            if (ImGui::Checkbox(u8"\u30D0\u30F3\u30AF\u89D2\u8A2D\u5B9A##bank", &overrideBank))
            {
                PushUndoState();
                road.bankAngle[selectedBankAngle.pointIndex].overrideBank = overrideBank;
                InvalidateRoadPreviewCache(selectedBankAngle.roadIndex);
            }

            float bankAngle = currentPoint.bankAngle;
            ImGui::BeginDisabled(!overrideBank);
            if (ImGui::SliderFloat(u8"\u30D0\u30F3\u30AF\u89D2##bank", &bankAngle, -90.0f, 90.0f, "%.1f"))
            {
                PushUndoState();
                road.bankAngle[selectedBankAngle.pointIndex].bankAngle = std::clamp(bankAngle, -90.0f, 90.0f);
                InvalidateRoadPreviewCache(selectedBankAngle.roadIndex);
            }
            ImGui::EndDisabled();

            if (ImGui::Button(u8"\u30D0\u30F3\u30AF\u89D2\u30DD\u30A4\u30F3\u30C8\u3092\u524A\u9664"))
            {
                PushUndoState();
                road.bankAngle.erase(road.bankAngle.begin() + selectedBankAngle.pointIndex);
                InvalidateRoadPreviewCache(selectedBankAngle.roadIndex);
                ClearBankAngleSelection();
                SanitizeSelection();
                m_statusMessage = u8"バンク角ポイントを削除しました";
            }
        }
    }

    LaneSectionRef selectedLaneSection;
    if (m_mode == EditorMode::LaneEdit &&
        GetPrimarySelectedLaneSectionPoint(selectedLaneSection) &&
        selectedLaneSection.roadIndex >= 0 &&
        selectedLaneSection.roadIndex < static_cast<int>(m_network->roads.size()))
    {
        Road& road = m_network->roads[selectedLaneSection.roadIndex];
        if (selectedLaneSection.pointIndex >= 0 &&
            selectedLaneSection.pointIndex < static_cast<int>(road.laneSections.size()))
        {
            ImGui::Separator();
            ImGui::Text(u8"\u8ECA\u7DDA\u30BB\u30AF\u30B7\u30E7\u30F3\u30DD\u30A4\u30F3\u30C8 %d", selectedLaneSection.pointIndex);

            LaneSectionPoint currentPoint = road.laneSections[selectedLaneSection.pointIndex];
            float uCoord = currentPoint.uCoord;
            if (ImGui::SliderFloat(u8"u_coord##lane", &uCoord, 0.0f, 1.0f, "%.6f"))
            {
                PushUndoState();
                currentPoint.uCoord = std::clamp(uCoord, 0.0f, 1.0f);
                road.laneSections[selectedLaneSection.pointIndex] = currentPoint;
                InvalidateRoadPreviewCache(selectedLaneSection.roadIndex);
                std::sort(
                    road.laneSections.begin(),
                    road.laneSections.end(),
                    [](const LaneSectionPoint& a, const LaneSectionPoint& b)
                    {
                        return a.uCoord < b.uCoord;
                    });
                int newIndex = -1;
                for (int i = 0; i < static_cast<int>(road.laneSections.size()); ++i)
                {
                    if (fabsf(road.laneSections[i].uCoord - currentPoint.uCoord) <= 1e-5f &&
                        road.laneSections[i].useLaneLeft2 == currentPoint.useLaneLeft2 &&
                        fabsf(road.laneSections[i].widthLaneLeft2 - currentPoint.widthLaneLeft2) <= 1e-5f &&
                        road.laneSections[i].useLaneLeft1 == currentPoint.useLaneLeft1 &&
                        fabsf(road.laneSections[i].widthLaneLeft1 - currentPoint.widthLaneLeft1) <= 1e-5f &&
                        road.laneSections[i].useLaneCenter == currentPoint.useLaneCenter &&
                        fabsf(road.laneSections[i].widthLaneCenter - currentPoint.widthLaneCenter) <= 1e-5f &&
                        road.laneSections[i].useLaneRight1 == currentPoint.useLaneRight1 &&
                        fabsf(road.laneSections[i].widthLaneRight1 - currentPoint.widthLaneRight1) <= 1e-5f &&
                        road.laneSections[i].useLaneRight2 == currentPoint.useLaneRight2 &&
                        fabsf(road.laneSections[i].widthLaneRight2 - currentPoint.widthLaneRight2) <= 1e-5f &&
                        fabsf(road.laneSections[i].offsetCenter - currentPoint.offsetCenter) <= 1e-5f)
                    {
                        newIndex = i;
                        break;
                    }
                }
                SelectSingleLaneSectionPoint(selectedLaneSection.roadIndex, newIndex);
                if (newIndex >= 0)
                {
                    selectedLaneSection.pointIndex = newIndex;
                    currentPoint = road.laneSections[newIndex];
                }
            }

            bool useLaneLeft2 = currentPoint.useLaneLeft2;
            if (ImGui::Checkbox(u8"\u5DE6\u5074\u7B2C2\u8ECA\u7DDA\u3092\u4F7F\u3046", &useLaneLeft2))
            {
                PushUndoState();
                road.laneSections[selectedLaneSection.pointIndex].useLaneLeft2 = useLaneLeft2;
                InvalidateRoadPreviewCache(selectedLaneSection.roadIndex);
            }
            float widthLaneLeft2 = currentPoint.widthLaneLeft2;
            ImGui::BeginDisabled(!useLaneLeft2);
            if (ImGui::InputFloat(u8"\u5E45##laneLeft2", &widthLaneLeft2, 0.1f, 1.0f, "%.2f"))
            {
                PushUndoState();
                road.laneSections[selectedLaneSection.pointIndex].widthLaneLeft2 = (std::max)(0.0f, widthLaneLeft2);
                InvalidateRoadPreviewCache(selectedLaneSection.roadIndex);
            }
            ImGui::EndDisabled();

            bool useLaneLeft1 = currentPoint.useLaneLeft1;
            if (ImGui::Checkbox(u8"\u5DE6\u5074\u7B2C1\u8ECA\u7DDA\u3092\u4F7F\u3046", &useLaneLeft1))
            {
                PushUndoState();
                road.laneSections[selectedLaneSection.pointIndex].useLaneLeft1 = useLaneLeft1;
                InvalidateRoadPreviewCache(selectedLaneSection.roadIndex);
            }
            float widthLaneLeft1 = currentPoint.widthLaneLeft1;
            ImGui::BeginDisabled(!useLaneLeft1);
            if (ImGui::InputFloat(u8"\u5E45##laneLeft1", &widthLaneLeft1, 0.1f, 1.0f, "%.2f"))
            {
                PushUndoState();
                road.laneSections[selectedLaneSection.pointIndex].widthLaneLeft1 = (std::max)(0.0f, widthLaneLeft1);
                InvalidateRoadPreviewCache(selectedLaneSection.roadIndex);
            }
            ImGui::EndDisabled();

            float widthLaneCenter = currentPoint.widthLaneCenter;
            if (ImGui::InputFloat(u8"\u4E2D\u592E\u7DDA\u5E45", &widthLaneCenter, 0.1f, 1.0f, "%.2f"))
            {
                PushUndoState();
                road.laneSections[selectedLaneSection.pointIndex].widthLaneCenter = (std::max)(0.0f, widthLaneCenter);
                InvalidateRoadPreviewCache(selectedLaneSection.roadIndex);
            }

            bool useLaneRight1 = currentPoint.useLaneRight1;
            if (ImGui::Checkbox(u8"\u53F3\u5074\u7B2C1\u8ECA\u7DDA\u3092\u4F7F\u3046", &useLaneRight1))
            {
                PushUndoState();
                road.laneSections[selectedLaneSection.pointIndex].useLaneRight1 = useLaneRight1;
                InvalidateRoadPreviewCache(selectedLaneSection.roadIndex);
            }
            float widthLaneRight1 = currentPoint.widthLaneRight1;
            ImGui::BeginDisabled(!useLaneRight1);
            if (ImGui::InputFloat(u8"\u5E45##laneRight1", &widthLaneRight1, 0.1f, 1.0f, "%.2f"))
            {
                PushUndoState();
                road.laneSections[selectedLaneSection.pointIndex].widthLaneRight1 = (std::max)(0.0f, widthLaneRight1);
                InvalidateRoadPreviewCache(selectedLaneSection.roadIndex);
            }
            ImGui::EndDisabled();

            bool useLaneRight2 = currentPoint.useLaneRight2;
            if (ImGui::Checkbox(u8"\u53F3\u5074\u7B2C2\u8ECA\u7DDA\u3092\u4F7F\u3046", &useLaneRight2))
            {
                PushUndoState();
                road.laneSections[selectedLaneSection.pointIndex].useLaneRight2 = useLaneRight2;
                InvalidateRoadPreviewCache(selectedLaneSection.roadIndex);
            }
            float widthLaneRight2 = currentPoint.widthLaneRight2;
            ImGui::BeginDisabled(!useLaneRight2);
            if (ImGui::InputFloat(u8"\u5E45##laneRight2", &widthLaneRight2, 0.1f, 1.0f, "%.2f"))
            {
                PushUndoState();
                road.laneSections[selectedLaneSection.pointIndex].widthLaneRight2 = (std::max)(0.0f, widthLaneRight2);
                InvalidateRoadPreviewCache(selectedLaneSection.roadIndex);
            }
            ImGui::EndDisabled();

            float offsetCenter = currentPoint.offsetCenter;
            if (ImGui::InputFloat(u8"\u4E2D\u592E\u7DDA\u306E\u30AA\u30D5\u30BB\u30C3\u30C8\u5E45", &offsetCenter, 0.1f, 1.0f, "%.2f"))
            {
                PushUndoState();
                road.laneSections[selectedLaneSection.pointIndex].offsetCenter = offsetCenter;
                InvalidateRoadPreviewCache(selectedLaneSection.roadIndex);
            }

            if (ImGui::Button(u8"\u8ECA\u7DDA\u30BB\u30AF\u30B7\u30E7\u30F3\u30DD\u30A4\u30F3\u30C8\u3092\u524A\u9664"))
            {
                PushUndoState();
                road.laneSections.erase(road.laneSections.begin() + selectedLaneSection.pointIndex);
                InvalidateRoadPreviewCache(selectedLaneSection.roadIndex);
                ClearLaneSectionSelection();
                SanitizeSelection();
                m_statusMessage = u8"車線セクションポイントを削除しました";
            }
        }
    }

    // Selected point info
    PointRef selectedPoint;
    if (GetPrimarySelectedPoint(selectedPoint) &&
        selectedPoint.roadIndex >= 0 &&
        selectedPoint.roadIndex < static_cast<int>(m_network->roads.size()))
    {
        ImGui::BeginDisabled(!m_showRoadGuidelines);
        Road& road = m_network->roads[selectedPoint.roadIndex];
        if (selectedPoint.pointIndex < static_cast<int>(road.points.size()))
        {
            RoadPoint& rp = road.points[selectedPoint.pointIndex];
            ImGui::Text(u8"\u30DD\u30A4\u30F3\u30C8 %d:%d", selectedPoint.roadIndex, selectedPoint.pointIndex);
            float pointPos[3] = { rp.pos.x, rp.pos.y, rp.pos.z };
            if (ImGui::InputFloat3(u8"\u4F4D\u7F6E", pointPos))
            {
                PushUndoState();
                rp.pos = { pointPos[0], pointPos[1], pointPos[2] };
                if (m_snapToTerrain && m_terrain && m_terrain->IsReady())
                    rp.pos.y = m_terrain->GetHeightAt(rp.pos.x, rp.pos.z);
                InvalidateRoadPreviewCache(selectedPoint.roadIndex);
            }
            if (IsSelectedRoadEndpoint())
            {
                std::string connectionId;
                if (GetSelectedRoadConnectionId(connectionId))
                    ImGui::Text(u8"\u63A5\u7D9A\u5148: %s", connectionId.c_str());
                else
                    ImGui::TextDisabled(u8"\u63A5\u7D9A\u5148: \u306A\u3057");

                if (ImGui::Button(u8"\u7AEF\u70B9\u306E\u63A5\u7D9A\u3092\u89E3\u9664"))
                {
                    PushUndoState();
                    ClearSelectedRoadConnection();
                    m_statusMessage = u8"道路の端点を切り離しました";
                }
            }
        }
        ImGui::EndDisabled();
    }

    if (m_activeIntersection >= 0 &&
        m_activeIntersection < static_cast<int>(m_network->intersections.size()))
    {
        Intersection& isec = m_network->intersections[m_activeIntersection];
        char nameBuf[128] = {};
        strncpy_s(nameBuf, sizeof(nameBuf), isec.name.c_str(), _TRUNCATE);
        static const char* kIntersectionTypes[] = { "intersection", "roundabout" };
        int typeIndex = (isec.type == "roundabout") ? 1 : 0;
        ImGui::Text(u8"\u4EA4\u5DEE\u70B9");
        if (ImGui::InputText(u8"\u540D\u524D##isec", nameBuf, sizeof(nameBuf)))
        {
            PushUndoState();
            isec.name = nameBuf;
        }
        const RoadGroup* currentGroup = m_network->FindGroupById(isec.groupId);
        const char* preview = currentGroup ? currentGroup->name.c_str() : "<none>";
        if (ImGui::BeginCombo(u8"\u30B0\u30EB\u30FC\u30D7##isec", preview))
        {
            for (int gi = 0; gi < static_cast<int>(m_network->groups.size()); ++gi)
            {
                const RoadGroup& group = m_network->groups[gi];
                const bool selected = (isec.groupId == group.id);
                if (ImGui::Selectable(group.name.c_str(), selected))
                {
                    PushUndoState();
                    isec.groupId = group.id;
                    m_activeGroup = gi;
                }
                if (selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        if (ImGui::Combo("Type##isec", &typeIndex, kIntersectionTypes, IM_ARRAYSIZE(kIntersectionTypes)))
        {
            PushUndoState();
            isec.type = kIntersectionTypes[typeIndex];
        }
        float isecPos[3] = { isec.pos.x, isec.pos.y, isec.pos.z };
        if (ImGui::InputFloat3("Pos##isec", isecPos))
        {
            PushUndoState();
            isec.pos = { isecPos[0], isecPos[1], isecPos[2] };
            if (m_snapToTerrain && m_terrain && m_terrain->IsReady())
                isec.pos.y = m_terrain->GetHeightAt(isec.pos.x, isec.pos.z);
            SyncRoadConnectionsForIntersection(m_activeIntersection);
            InvalidateAllPreviewCaches();
        }
        float isecEntryDist = isec.entryDist;
        if (ImGui::SliderFloat(u8"\u63A5\u7D9A\u8DDD\u96E2 (m)##isec", &isecEntryDist, 1.0f, 20.0f))
        {
            PushUndoState();
            isec.entryDist = isecEntryDist;
        }
    }

        ImGui::End();
    }
}
