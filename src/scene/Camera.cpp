#include "Camera.h"
#include <algorithm>
#include <cmath>

using namespace DirectX;

void Camera::Initialize(float distance, float azimuth, float elevation)
{
    m_distance  = distance;
    m_azimuth   = azimuth;
    m_elevation = elevation;

    GetCursorPos(&m_lastMouse);
}

DirectX::XMFLOAT3 Camera::GetPosition() const
{
    float cosE = cosf(m_elevation);
    return {
        m_target.x + m_distance * cosE * sinf(m_azimuth),
        m_target.y + m_distance * sinf(m_elevation),
        m_target.z + m_distance * cosE * cosf(m_azimuth)
    };
}

DirectX::XMMATRIX Camera::GetViewMatrix() const
{
    XMFLOAT3 pos = GetPosition();
    XMVECTOR eye = XMLoadFloat3(&pos);
    XMVECTOR tgt = XMLoadFloat3(&m_target);
    XMVECTOR up  = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);

    // Avoid gimbal lock when looking straight up/down
    if (fabsf(m_elevation) > 1.56f)
        up = XMVectorSet(0.0f, 0.0f, (m_elevation > 0.0f) ? -1.0f : 1.0f, 0.0f);

    return XMMatrixLookAtRH(eye, tgt, up);
}

DirectX::XMMATRIX Camera::GetProjMatrix(float aspect, float fovY,
                                         float nearZ, float farZ) const
{
    return XMMatrixPerspectiveFovRH(fovY, aspect, nearZ, farZ);
}

void Camera::SetOrbitState(XMFLOAT3 target, float distance,
                           float azimuth, float elevation)
{
    m_target    = target;
    m_distance  = std::clamp(distance, 0.5f, 10000.0f);
    m_azimuth   = azimuth;
    m_elevation = std::clamp(elevation, -1.55f, 1.55f);
}

void Camera::HandleInput(bool wantMouse)
{
    POINT cur;
    GetCursorPos(&cur);

    float dx = static_cast<float>(cur.x - m_lastMouse.x);
    float dy = static_cast<float>(cur.y - m_lastMouse.y);
    m_lastMouse = cur;

    if (wantMouse)
        return;

    // Maya-style: Alt must be held for all camera operations
    bool alt = (GetAsyncKeyState(VK_MENU)   & 0x8000) != 0;
    bool lmb = (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;
    bool mmb = (GetAsyncKeyState(VK_MBUTTON) & 0x8000) != 0;

    if (!alt)
        return;

    if (lmb)
    {
        // Alt + LMB : Orbit (rotate)
        const float sensitivity = 0.005f;
        m_azimuth   -= dx * sensitivity;
        m_elevation += dy * sensitivity;
        m_elevation  = std::clamp(m_elevation, -1.55f, 1.55f);
    }
    else if (mmb)
    {
        // Alt + MMB : Pan
        XMFLOAT3 pos  = GetPosition();
        XMVECTOR eye  = XMLoadFloat3(&pos);
        XMVECTOR tgt  = XMLoadFloat3(&m_target);
        XMVECTOR fwd  = XMVector3Normalize(XMVectorSubtract(tgt, eye));
        XMVECTOR up   = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
        XMVECTOR right   = XMVector3Normalize(XMVector3Cross(fwd, up));
        XMVECTOR upOrtho = XMVector3Normalize(XMVector3Cross(right, fwd));

        float panScale = m_distance * 0.001f;
        XMVECTOR delta = XMVectorAdd(
            XMVectorScale(right,    -dx * panScale),
            XMVectorScale(upOrtho,   dy * panScale));

        XMFLOAT3 d;
        XMStoreFloat3(&d, delta);
        m_target.x += d.x;
        m_target.y += d.y;
        m_target.z += d.z;
    }
}

void Camera::OnScroll(float wheelDelta)
{
    m_distance *= powf(0.88f, wheelDelta);
    m_distance  = std::clamp(m_distance, 0.5f, 10000.0f);
}
