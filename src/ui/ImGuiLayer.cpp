#include "ImGuiLayer.h"
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"

bool ImGuiLayer::Initialize(HWND hwnd, ID3D11Device* device, ID3D11DeviceContext* ctx)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImFontConfig fontCfg = {};
    fontCfg.OversampleH = 2;
    fontCfg.OversampleV = 2;
    fontCfg.PixelSnapH = false;
    const char* fontCandidates[] =
    {
        "C:/Windows/Fonts/meiryo.ttc",
        "C:/Windows/Fonts/YuGothM.ttc",
        "C:/Windows/Fonts/msgothic.ttc"
    };
    for (const char* fontPath : fontCandidates)
    {
        if (GetFileAttributesA(fontPath) != INVALID_FILE_ATTRIBUTES)
        {
            io.Fonts->AddFontFromFileTTF(
                fontPath,
                18.0f,
                &fontCfg,
                io.Fonts->GetGlyphRangesJapanese());
            break;
        }
    }

    ImGui::StyleColorsDark();

    // Tweak colours slightly for the dark background
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding   = 4.0f;
    style.FrameRounding    = 3.0f;
    style.GrabRounding     = 3.0f;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.1f, 0.12f, 0.92f);

    if (!ImGui_ImplWin32_Init(hwnd))        return false;
    if (!ImGui_ImplDX11_Init(device, ctx))  return false;

    return true;
}

void ImGuiLayer::BeginFrame()
{
    ImGui_ImplDX11_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
}

void ImGuiLayer::EndFrame()
{
    ImGui::Render();
    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
}

void ImGuiLayer::Shutdown()
{
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
}
