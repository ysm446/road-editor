#pragma once

#include <DirectXMath.h>
#include <Windows.h>

// Orbit camera (Maya-style controls):
//
//  Alt + LMB drag : rotate (azimuth / elevation)
//  Alt + MMB drag : pan target
//  Scroll wheel   : zoom (distance)
class Camera
{
public:
    void Initialize(float distance  = 20.0f,
                    float azimuth   = 0.785f,   // 45 degrees
                    float elevation = 0.4f);

    // Call once per frame; wantMouse = ImGui::GetIO().WantCaptureMouse
    void HandleInput(bool wantMouse);

    // Call from WM_MOUSEWHEEL handler
    void OnScroll(float wheelDelta);

    DirectX::XMMATRIX   GetViewMatrix() const;
    DirectX::XMMATRIX   GetProjMatrix(float aspect,
                                      float fovY  = DirectX::XM_PIDIV4,
                                      float nearZ = 0.1f,
                                      float farZ  = 10000.0f) const;
    DirectX::XMFLOAT3   GetPosition()  const;
    DirectX::XMFLOAT3   GetTarget()    const { return m_target; }
    float               GetDistance()  const { return m_distance; }
    float               GetAzimuth()   const { return m_azimuth; }
    float               GetElevation() const { return m_elevation; }
    void SetOrbitState(DirectX::XMFLOAT3 target,
                       float distance,
                       float azimuth,
                       float elevation);

private:
    DirectX::XMFLOAT3 m_target    = { 0.0f, 0.0f, 0.0f };
    float             m_distance  = 20.0f;
    float             m_azimuth   = 0.785f;
    float             m_elevation = 0.4f;

    POINT m_lastMouse = { 0, 0 };
};
