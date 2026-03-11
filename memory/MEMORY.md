# Road Editor Project Memory

## Build
- Visual Studio 2026: `cmake -B build -S . -G "Visual Studio 18 2026" -A x64`
- FetchContent で ImGui(docking), nlohmann/json, stb を自動取得

## Known Issues / Lessons Learned
- `line` は HLSL の予約キーワード (geometry shader primitive type) — 変数名に使えない
- Terrain 三角形の巻き順: v00,v01,v10 / v10,v01,v11 が正 (D3D11 default CW front face)
  - 法線は XMVector3Cross(tz, tx) で上向きになる (tz=Z接線, tx=X接線)
- D3D11 デバッグレイヤーは Graphics Tools 未インストール時に失敗する → フォールバック済み (D3D11Context.cpp)
- ソースファイルは ASCII only (Windows CP932環境のため)

## Phase Status
- Phase 1: 完了 (D3D11 + ImGui + Camera + Grid)
- Phase 2: 完了 (Terrain heightmap + Lambert lighting + wireframe)
- Phase 3: 完了 (PolylineEditor + DebugDraw + RoadNetwork + Terrain raycast)
- Phase 4: 未実装

## Phase 3 New Files
- src/editor/EditorState.h    -- EditorMode enum
- src/editor/RoadData.h/.cpp  -- RoadPoint, Road, RoadNetwork (nlohmann/json)
- src/editor/PolylineEditor.h/.cpp -- click-to-add, select/drag/delete, Enter/Esc
- src/renderer/DebugDraw.h/.cpp -- dynamic LINELIST renderer
- shaders/line_vs.hlsl / line_ps.hlsl -- simple colored line shaders
- Terrain::GetHeightAt() / Raycast() added to Terrain.h/.cpp
