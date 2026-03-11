# CLAUDE.md - Road Editor Project Instructions

## Build

```bash
# Configure (Visual Studio 2026)
cmake -B build -S . -G "Visual Studio 18 2026" -A x64

# Build
cmake --build build --config Debug
cmake --build build --config Release

# Run
build/Debug/RoadEditor.exe
```

FetchContent が初回に以下を自動取得する:
- Dear ImGui (docking branch)
- nlohmann/json v3.11.3
- stb (master)

## Project Structure

```
src/
  main.cpp              # WinMain entry point
  App.h / App.cpp       # Window, main loop, render orchestration
  renderer/
    D3D11Context        # Device, SwapChain, RTV, DSV management
    Shader              # HLSL runtime compile (D3DCompileFromFile)
    Buffer.h            # ConstantBuffer<T> template (RAII)
  scene/
    Camera              # Orbit camera (azimuth/elevation/distance)
    Grid                # Infinite XZ grid via fullscreen triangle
  editor/               # (Phase 3+) polyline / road editing
  ui/
    ImGuiLayer          # ImGui init, BeginFrame/EndFrame
shaders/
  common.hlsli          # cbuffer PerFrame (viewProj, invViewProj, cameraPos, time)
  grid_vs.hlsl          # Fullscreen triangle, near/far unproject
  grid_ps.hlsl          # Ray-plane intersection, grid pattern, custom SV_DEPTH
```

## Coding Conventions

- **Language**: C++17
- **Encoding**: ASCII only in source files (Windows CP932 environment)
- **String literals**: `L"..."` for Win32 API (UNICODE defined)
- **COM objects**: `Microsoft::WRL::ComPtr<T>` - never raw pointers
- **Matrix upload**: `row_major` keyword in HLSL; no transpose needed
- **Constant buffer size**: must be multiple of 16 bytes
- **Error handling**: return `bool` from Initialize(); HRESULT checked with `FAILED()`
- **Debug layer**: enabled automatically in Debug builds (`D3D11_CREATE_DEVICE_DEBUG`)

## Constant Buffer Layout

```cpp
struct PerFrameData          // slot b0
{
    XMFLOAT4X4 viewProj;     // 64 bytes
    XMFLOAT4X4 invViewProj;  // 64 bytes
    XMFLOAT3   cameraPos;    // 12 bytes
    float      time;         //  4 bytes
};                           // 144 bytes total
```

```hlsl
// common.hlsli
cbuffer PerFrame : register(b0)
{
    row_major float4x4 viewProj;
    row_major float4x4 invViewProj;
    float3 cameraPos;
    float  time;
};
```

## Render Order (plan.md Phase design)

1. Terrain (opaque)
2. Road mesh (opaque, depth bias)
3. Grid (alpha blend)
4. Polyline / control points (overlay)
5. ImGui (front-most)

## Camera Controls

| Operation | Input |
|-----------|-------|
| Rotate | Middle mouse button drag |
| Pan target | Shift + MMB drag |
| Zoom | Scroll wheel |

## Phase Status

| Phase | Status | Content |
|-------|--------|---------|
| Phase 1 | Done | D3D11 + ImGui + Orbit camera + Grid |
| Phase 2 | Pending | Heightmap terrain |
| Phase 3 | Pending | Polyline road placement |
| Phase 4 | Pending | Road mesh generation |
