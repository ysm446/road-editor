# Road Network Editor

DirectX 11 + Dear ImGui によるスタンドアロンの道路ネットワークエディタ。
Houdini の道路エディタ HDA で培った設計思想をベースに、段階的に実装を進めるプロジェクト。

## スクリーンショット

*Phase 1: グリッド表示 + オービットカメラ*

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
| 回転 | 中ボタンドラッグ |
| パン | Shift + 中ボタンドラッグ |
| ズーム | スクロールホイール |

## 実装フェーズ

- [x] **Phase 1**: D3D11 初期化 / Dear ImGui 統合 / オービットカメラ / 無限グリッド
- [ ] **Phase 2**: ハイトマップ地形表示
- [ ] **Phase 3**: ポリライン道路配置・編集 / JSON 保存・読み込み
- [ ] **Phase 4**: スプライン補間による道路メッシュ自動生成

## プロジェクト構成

```
RoadEditor/
├── CMakeLists.txt
├── shaders/
│   ├── common.hlsli
│   ├── grid_vs.hlsl
│   └── grid_ps.hlsl
├── src/
│   ├── main.cpp
│   ├── App.h / App.cpp
│   ├── renderer/
│   │   ├── D3D11Context.h / .cpp
│   │   ├── Shader.h / .cpp
│   │   └── Buffer.h
│   ├── scene/
│   │   ├── Camera.h / .cpp
│   │   └── Grid.h / .cpp
│   └── ui/
│       └── ImGuiLayer.h / .cpp
└── data/                   # (Phase 2+) heightmap images
```

## ライセンス

MIT
