# Road Network Editor

DirectX 11 + Dear ImGui によるスタンドアロンの道路ネットワークエディタ。
Houdini の道路エディタ HDA で培った設計思想をベースに、段階的に実装を進めるプロジェクト。

## 現在の機能

- 無限グリッド表示
- 右手座標系のオービットカメラ操作
- 画面左下の XYZ 軸表示
- ハイトマップ画像からの地形生成
- Terrain パネルからの Divisions X / Z、Size X / Z / Y、Offset X / Z の調整
- Terrain 未ロード時の XZ 平面配置
- 道路ポリラインと交差点の作成、編集、削除
- ポイント挿入、複数選択、矩形選択、gizmo 移動
- 各道路ポリラインに対する 2 次ベジェ曲線プレビュー表示
- 道路と交差点のグループ管理、表示 / 非表示、ロック
- 道路名 / 交差点名の表示切り替えと永続化
- Undo / Redo (`Ctrl+Z` / `Ctrl+Y`)
- UUID ベースの道路データ JSON 保存・読み込み
- グループや表示設定を含むプロジェクト初期化 (`File -> New Project`)

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
git clone https://github.com/<your-name>/road-editor.git
cd road-editor

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
| フォーカス | `F` |

`F` は選択中の道路、道路ポイント、交差点にフォーカスします。

## Terrain の使い方

- `Browse` でハイトマップ画像を選択して `Load` を押すと地形を生成します。
- `Divisions X / Z` はワイヤーフレーム格子のセル数です。`0` を指定すると画像解像度に合わせて `image size - 1` が使われます。
- `Size X / Z / Y (m)` は地形の幅、奥行き、高さです。
- `Offset X / Z (m)` は地形全体の水平オフセットです。
- 地形ロード後も同じ UI を使い続けられ、`Divisions` や `Size` を変更するとその地形が再構築されます。
- `Wireframe` をオンにすると地形メッシュをワイヤーフレーム表示できます。
- `Clear Height Field` を押すとハイトフィールドを解除し、XZ 平面ベースの編集に戻せます。

## エディタ操作

| 操作 | 入力 |
|------|------|
| 道路選択 | クリック |
| 交差点選択 | クリック |
| 道路ポイント選択 | 道路選択後にポイントをクリック |
| ポイント / 交差点の追加選択 | `Ctrl + Click` |
| ポイント / 交差点の矩形選択 | 空いた場所をドラッグ |
| 矩形の追加選択 | `Ctrl + ドラッグ` |
| 矩形の解除選択 | `Ctrl + Shift + ドラッグ` |
| gizmo 編集に入る | `W` |
| Undo / Redo | `Ctrl + Z` / `Ctrl + Y` |
| 選択道路の削除 | `Delete` |
| 選択交差点の削除 | `Delete` |
| 選択ポイントの削除 | `Delete` |

- `Navigate` と `PointEdit` の両方で矩形選択が使えます。
- `PointEdit` では、道路ポイントと交差点を混在選択したまま同時移動できます。
- 道路ポイント間のラインをクリックすると、その位置に新しいポイントを挿入できます。
- 単一の道路端点を選択して移動した場合は、近い交差点へスナップ / 接続できます。
- 各道路には、制御点として道路ポイントを使った 2 次ベジェ曲線プレビューが重ねて表示されます。
- ベジェ曲線は道路ポイント自体は通らず、各エッジ上の中間点を通ります。
- 中間点は隣接エッジ長に比例した位置に置かれます。

## グループと表示

- 道路と交差点はグループに所属できます。
- グループはツリー UI で管理でき、表示 / 非表示とロックを切り替えられます。
- `View` メニューから `Road Names` と `Intersection Names` を切り替えできます。
- 表示設定は次回起動時のために保存されます。

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
- ハイトフィールド未ロード時は `y=0` の XZ 平面に道路と交差点を配置します。
- ベジェプレビューは道路メッシュ生成ではなく、現時点では編集用の可視化です。
- 道路、交差点、グループは UUID を持ち、JSON 上でもそのまま保存されます。
- `build\Debug\RoadEditor.exe` の出力先は環境や CMake ジェネレータによって変わる場合があります。

## ライセンス

MIT
