#include "PolylineEditor.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>

using namespace DirectX;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static float Dist2D(XMFLOAT2 a, XMFLOAT2 b)
{
    float dx = a.x - b.x, dy = a.y - b.y;
    return sqrtf(dx * dx + dy * dy);
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

// ---------------------------------------------------------------------------
// SetMode
// ---------------------------------------------------------------------------
void PolylineEditor::SetMode(EditorMode mode)
{
    // Cancel any in-progress draw
    if (m_mode == EditorMode::PolylineDraw && mode != EditorMode::PolylineDraw)
        CancelRoad();

    m_mode = mode;
    if (mode != EditorMode::PointEdit)
    {
        m_activePoint = -1;
        m_dragging    = false;
    }
}

void PolylineEditor::StartNewRoad()
{
    m_activeRoad  = m_network->AddRoad("Road " +
        std::to_string(m_network->roads.size()));
    m_activePoint = -1;
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
                m_network->RemoveRoad(m_activeRoad);
        }
        m_activeRoad  = -1;
        m_activePoint = -1;
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
            m_network->RemoveRoad(m_activeRoad);
        }
        m_activeRoad  = -1;
        m_activePoint = -1;
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
    bool alt = (GetAsyncKeyState(VK_MENU) & 0x8000) != 0;

    if (wantMouse || m_mode == EditorMode::Navigate || alt)
    {
        m_hasCursorPos = false;
        m_prevLButton  = (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;
        return;
    }

    bool lDown = (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;
    bool lClick = lDown && !m_prevLButton;  // rising edge
    m_prevLButton = lDown;

    // Compute ray
    XMFLOAT3 rayOrig, rayDir;
    ScreenToRay(vpW, vpH, mousePos, invViewProj, rayOrig, rayDir);

    // Terrain intersection for cursor preview and placement
    XMFLOAT3 hitPos = {};
    bool hasHit = m_terrain && m_terrain->IsReady() &&
                  m_terrain->Raycast(rayOrig, rayDir, hitPos);
    m_hasCursorPos = hasHit;
    m_cursorPos    = hitPos;

    // --- PolylineDraw mode ---
    if (m_mode == EditorMode::PolylineDraw)
    {
        if (lClick && hasHit &&
            m_activeRoad >= 0 &&
            m_activeRoad < static_cast<int>(m_network->roads.size()))
        {
            RoadPoint rp;
            rp.pos   = hitPos;
            rp.width = m_defaultWidth;
            m_network->roads[m_activeRoad].points.push_back(rp);
        }

        // Enter -> confirm, Esc -> cancel
        if (GetAsyncKeyState(VK_RETURN) & 0x8000)
            ConfirmRoad();
        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
            CancelRoad();

        return;
    }

    // --- PointEdit mode ---
    if (m_mode == EditorMode::PointEdit)
    {
        // Rebuild viewProj from invViewProj (inverse)
        XMMATRIX viewProj = XMMatrixInverse(nullptr, invViewProj);

        if (m_dragging && lDown)
        {
            // Move selected point to terrain hit
            if (hasHit &&
                m_activeRoad  >= 0 &&
                m_activePoint >= 0 &&
                m_activeRoad  < static_cast<int>(m_network->roads.size()))
            {
                Road& road = m_network->roads[m_activeRoad];
                if (m_activePoint < static_cast<int>(road.points.size()))
                    road.points[m_activePoint].pos = hitPos;
            }
        }
        else if (m_dragging && !lDown)
        {
            m_dragging = false;
        }
        else if (lClick)
        {
            // Try to select a point
            int selRoad, selPt;
            FindNearestPoint(vpW, vpH, mousePos, viewProj, selRoad, selPt);
            if (selRoad >= 0)
            {
                m_activeRoad  = selRoad;
                m_activePoint = selPt;
                m_dragging    = true;
            }
            else
            {
                m_activePoint = -1;
                m_dragging    = false;
            }
        }

        // Delete selected point
        if ((GetAsyncKeyState(VK_DELETE) & 0x8000) &&
            m_activeRoad  >= 0 &&
            m_activePoint >= 0 &&
            m_activeRoad  < static_cast<int>(m_network->roads.size()))
        {
            Road& road = m_network->roads[m_activeRoad];
            if (m_activePoint < static_cast<int>(road.points.size()))
            {
                road.points.erase(road.points.begin() + m_activePoint);
                m_activePoint = -1;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// DrawNetwork / DrawOverlay
// ---------------------------------------------------------------------------

void PolylineEditor::DrawNetwork(DebugDraw& dd) const
{
    static const XMFLOAT4 colorRoad     = { 1.0f, 0.8f, 0.1f, 1.0f };
    static const XMFLOAT4 colorSelected = { 1.0f, 0.3f, 0.3f, 1.0f };
    static const XMFLOAT4 colorCursor   = { 0.2f, 1.0f, 0.4f, 0.4f };

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        XMFLOAT4 col = (ri == m_activeRoad) ? colorSelected : colorRoad;

        for (int pi = 0; pi + 1 < static_cast<int>(road.points.size()); ++pi)
            dd.AddLine(road.points[pi].pos, road.points[pi + 1].pos, col);

        if (road.closed && road.points.size() >= 2)
            dd.AddLine(road.points.back().pos, road.points.front().pos, col);
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
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const float kRadius    = 4.0f;
    const ImU32 colPoint    = IM_COL32(255, 255, 255, 220);
    const ImU32 colSelected = IM_COL32(255,  80,  80, 255);
    const ImU32 colCursor   = IM_COL32( 60, 255, 110, 220);

    for (int ri = 0; ri < static_cast<int>(m_network->roads.size()); ++ri)
    {
        const Road& road = m_network->roads[ri];
        for (int pi = 0; pi < static_cast<int>(road.points.size()); ++pi)
        {
            ImVec2 sp;
            if (!WorldToScreen(road.points[pi].pos, viewProj, vpW, vpH, sp))
                continue;
            bool isActive = (ri == m_activeRoad && pi == m_activePoint);
            ImU32 col = isActive ? colSelected : colPoint;
            float r   = isActive ? kRadius + 2.0f : kRadius;
            dl->AddCircleFilled(sp, r, col, 20);
        }
    }

}

// ---------------------------------------------------------------------------
// DrawUI
// ---------------------------------------------------------------------------
void PolylineEditor::DrawUI(ID3D11Device* /*device*/)
{
    ImGui::SetNextWindowPos(ImVec2(10, 360), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(280, 320), ImGuiCond_FirstUseEver);
    ImGui::Begin("Road Editor");

    // Mode toolbar
    {
        bool navActive  = (m_mode == EditorMode::Navigate);
        bool drawActive = (m_mode == EditorMode::PolylineDraw);
        bool editActive = (m_mode == EditorMode::PointEdit);

        if (navActive)  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button("Navigate")) SetMode(EditorMode::Navigate);
        if (navActive)  ImGui::PopStyleColor();

        ImGui::SameLine();

        if (drawActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button("Draw Road")) StartNewRoad();
        if (drawActive) ImGui::PopStyleColor();

        ImGui::SameLine();

        if (editActive) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f,0.6f,0.2f,1));
        if (ImGui::Button("Edit Points")) SetMode(EditorMode::PointEdit);
        if (editActive) ImGui::PopStyleColor();
    }

    ImGui::Separator();

    // Mode hint
    switch (m_mode)
    {
    case EditorMode::Navigate:
        ImGui::TextDisabled("Camera navigation only");
        break;
    case EditorMode::PolylineDraw:
        ImGui::TextColored(ImVec4(0.2f,1,0.4f,1), "Drawing: %s",
            (m_activeRoad >= 0 &&
             m_activeRoad < static_cast<int>(m_network->roads.size()))
            ? m_network->roads[m_activeRoad].name.c_str() : "");
        ImGui::TextDisabled("Left click: add point");
        ImGui::TextDisabled("Enter: confirm  Esc: cancel");
        ImGui::SliderFloat("Width", &m_defaultWidth, 0.5f, 20.0f);
        break;
    case EditorMode::PointEdit:
        ImGui::TextColored(ImVec4(1,0.8f,0.2f,1), "Edit Points");
        ImGui::TextDisabled("Click: select/drag point");
        ImGui::TextDisabled("Delete: remove point");
        break;
    }

    ImGui::Separator();

    // Road list
    ImGui::Text("Roads (%d)", static_cast<int>(m_network->roads.size()));
    ImGui::BeginChild("RoadList", ImVec2(0, 110), true);
    for (int i = 0; i < static_cast<int>(m_network->roads.size()); ++i)
    {
        Road& road = m_network->roads[i];
        bool selected = (i == m_activeRoad);
        if (ImGui::Selectable(road.name.c_str(), selected))
        {
            m_activeRoad  = i;
            m_activePoint = -1;
        }
        if (selected && ImGui::BeginPopupContextItem())
        {
            if (ImGui::MenuItem("Delete Road"))
            {
                m_network->RemoveRoad(i);
                if (m_activeRoad >= static_cast<int>(m_network->roads.size()))
                    m_activeRoad = -1;
            }
            ImGui::EndPopup();
        }
    }
    ImGui::EndChild();

    // Selected point info
    if (m_activeRoad  >= 0 &&
        m_activePoint >= 0 &&
        m_activeRoad  < static_cast<int>(m_network->roads.size()))
    {
        Road& road = m_network->roads[m_activeRoad];
        if (m_activePoint < static_cast<int>(road.points.size()))
        {
            RoadPoint& rp = road.points[m_activePoint];
            ImGui::Text("Point %d", m_activePoint);
            ImGui::InputFloat3("Pos", &rp.pos.x);
            ImGui::SliderFloat("Width##pt", &rp.width, 0.5f, 20.0f);
        }
    }

    ImGui::Separator();

    // Save / load
    ImGui::InputText("Path", m_filePath, sizeof(m_filePath));
    if (ImGui::Button("Save"))
    {
        if (!Save(m_filePath))
            ImGui::OpenPopup("SaveError");
    }
    ImGui::SameLine();
    if (ImGui::Button("Load"))
    {
        if (!Load(m_filePath))
            ImGui::OpenPopup("LoadError");
    }
    if (ImGui::BeginPopup("SaveError"))
    {
        ImGui::Text("Save failed: %s", m_filePath);
        if (ImGui::Button("OK")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    if (ImGui::BeginPopup("LoadError"))
    {
        ImGui::Text("Load failed: %s", m_filePath);
        if (ImGui::Button("OK")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    ImGui::End();
}
