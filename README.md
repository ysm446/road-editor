# Road Network Editor

DirectX 11 + Dear ImGui によるスタンドアロンの道路ネットワークエディタ。
Houdini の道路エディタ HDA で培った設計思想をベースに、段階的に実装を進めるプロジェクト。

## 現在の機能

- 無限グリッド表示
- オービットカメラ操作
- ハイトマップ画像からの地形生成
- Terrain パネルからの分割数、幅、奥行き、高さの調整
- 地形へのレイキャストを使ったポリライン編集の基礎実装
- 道路データの JSON 保存・読み込み

## 技術スタック

| 項目 | 選定 |
|------|------|
| グラフィックスAPI | DirectX 11 |
| UI | Dear ImGui (docking branch) |
| 数学 | DirectXMath |
| ビルド | CMake 3.20+ |
| 言語 | C++17 |
| シェーダー | HLSL Shader Model 5.0 |

## 必要環境

- Windows 10/11
- Visual Studio 2022 / 2026 (MSVC)
- CMake 3.20 以上
- DirectX 11 対応 GPU

## ビルド手順

```bash
# 1. リポジトリをクローン
git clone https://github.com/<your-name>/directx-viewer.git
cd directx-viewer

# 2. CMake 構成 (Visual Studio 2026 の場合)
cmake -B build -S . -G "Visual Studio 18 2026" -A x64

# Visual Studio 2022 の場合
cmake -B build -S . -G "Visual Studio 17 2022" -A x64

# 3. ビルド
cmake --build build --config Debug

# 4. 実行
build\Debug\RoadEditor.exe
```

初回ビルド時に CMake FetchContent が以下を自動取得します（インターネット接続が必要）。

| ライブラリ | バージョン |
|-----------|-----------|
| Dear ImGui | docking branch |
| nlohmann/json | v3.11.3 |
| stb | master |

## カメラ操作

| 操作 | 入力 |
|------|------|
| 回転 | Alt + 左ドラッグ |
| パン | Alt + 中ドラッグ |
| ズーム | スクロールホイール |

## Terrain の使い方

- `Browse` でハイトマップ画像を選択して `Load` を押すと地形を生成します。
- `Divisions X / Y` はワイヤーフレーム格子のセル数です。`0` を指定すると画像解像度に合わせて `image size - 1` が使われます。
- `Size W / D / H (m)` は地形の幅、奥行き、高さです。
- 地形ロード後も同じ UI を使い続けられ、`Divisions` や `Size` を変更するとその地形が再構築されます。
- `Wireframe` をオンにすると地形メッシュをワイヤーフレーム表示できます。

## 実装状況

- [x] Phase 1: D3D11 初期化 / Dear ImGui 統合 / オービットカメラ / 無限グリッド
- [x] Phase 2: ハイトマップ地形表示
- [~] Phase 3: ポリライン道路配置・編集 / JSON 保存・読み込み
- [ ] Phase 4: スプライン補間による道路メッシュ自動生成

## プロジェクト構成

```
RoadEditor/
├── CMakeLists.txt
├── data/
│   ├── heightmap.png
│   └── roads.json
├── shaders/
│   ├── common.hlsli
│   ├── grid_vs.hlsl
│   ├── grid_ps.hlsl
│   ├── line_vs.hlsl
│   ├── line_ps.hlsl
│   ├── terrain_vs.hlsl
│   └── terrain_ps.hlsl
├── src/
│   ├── main.cpp
│   ├── App.h / App.cpp
│   ├── editor/
│   │   ├── EditorState.h
│   │   ├── PolylineEditor.h / .cpp
│   │   └── RoadData.h / .cpp
│   ├── renderer/
│   │   ├── D3D11Context.h / .cpp
│   │   ├── Shader.h / .cpp
│   │   ├── Buffer.h
│   │   └── DebugDraw.h / .cpp
│   ├── scene/
│   │   ├── Camera.h / .cpp
│   │   ├── Grid.h / .cpp
│   │   └── Terrain.h / .cpp
│   └── ui/
│       └── ImGuiLayer.h / .cpp
```

## メモ

- 初回ロード前でも Terrain パネルの `Divisions` と `Size` を設定できます。
- ロード後も UI は切り替わらず、同じ項目で地形パラメータを編集できます。
- `build\Debug\RoadEditor.exe` の出力先は環境や CMake ジェネレータによって変わる場合があります。

## ライセンス

MIT
