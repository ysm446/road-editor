# Road Editor

DirectX 11 と Dear ImGui で構築した道路編集ツールです。  
ハイトマップ地形の上で道路ポリラインと交差点を編集し、プロジェクト単位で保存できます。

## 主な機能

- ハイトマップ地形の読み込み
- 地形サイズ、分割数、オフセットの調整
- 道路と交差点の作成、編集、保存、読込
- 道路端点の交差点スナップ
- Undo / Redo
- 最近開いたプロジェクト
- 背景、グリッド、エディタ表示の調整
- 旧フォーマット道路 / 交差点 JSON の読込互換

## 動作環境

- Windows 10 / 11
- Visual Studio 2022 または 2026
- CMake 3.20 以上
- DirectX 11 対応 GPU

## ビルド

### Visual Studio 2026

```powershell
cmake -B build -S . -G "Visual Studio 18 2026" -A x64
cmake --build build --config Debug
```

### Visual Studio 2022

```powershell
cmake -B build -S . -G "Visual Studio 17 2022" -A x64
cmake --build build --config Debug
```

### `cmake` が PATH に無い場合

```powershell
& 'C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' --build build --config Debug
```

### 実行

```powershell
.\build\Debug\RoadEditor.exe
```

## 配布フォルダ

配布用の出力は [`dist/RoadEditor`](/d:/GitHub/road-editor/dist/RoadEditor) にまとめています。

主な内容:

- [`RoadEditor.exe`](/d:/GitHub/road-editor/dist/RoadEditor/RoadEditor.exe)
- [`shaders`](/d:/GitHub/road-editor/dist/RoadEditor/shaders)
- [`data`](/d:/GitHub/road-editor/dist/RoadEditor/data)

## ファイル形式

このツールでは中身は JSON ですが、用途ごとに拡張子を分けています。

- プロジェクト: `.roadproj`
- 道路ネットワーク: `.roadnet`

既存の `.json` も読込互換がありますが、新規保存は上記拡張子を使います。

サンプル:

- [`data/project.roadproj`](/d:/GitHub/road-editor/data/project.roadproj)
- [`data/roads.roadnet`](/d:/GitHub/road-editor/data/roads.roadnet)
- [`data/testtrack/testtrack_project.roadproj`](/d:/GitHub/road-editor/data/testtrack/testtrack_project.roadproj)
- [`data/testtrack/testtrack_road.roadnet`](/d:/GitHub/road-editor/data/testtrack/testtrack_road.roadnet)

## メニュー構成

上部メニュー:

- `ファイル`
- `編集`
- `表示`
- `ウインドウ`
- `設定`

`ウインドウ` から次のウィンドウを表示 / 非表示できます。

- 地形
- カメラ
- 道路エディタ
- プロパティ

各ウィンドウは右上の閉じるボタンでも非表示にできます。

## ファイルメニュー

- `新規プロジェクト`
- `プロジェクトを開く...`
- `最近開いたプロジェクト`
- `プロジェクトを保存`
- `名前を付けて保存...`
- `道路を開く...`
- `道路を保存`
- `道路を別名で保存...`

## 編集メニュー

- `Undo`
- `Redo`
- `交差点を自動作成`

`交差点を自動作成` は、近接した未接続の道路端点から交差点を生成します。

## 表示メニュー

- 道路名
- 交差点名
- 道路プレビュー情報
- 道路勾配グラデーション
- グリッド
- FPS

FPS はメニューバー右端に表示されます。

## 設定メニュー

### 背景

- グリッドサイズ
- フォグ距離
- 背景色

### エディタ表示

道路:

- 道路の太さ
- プレビューカーブの太さ
- 選択時の道路の太さ
- 頂点サイズ
- 色
- 選択時の色

交差点:

- 交差点サークルサイズ
- 色

設定内容は [`data/view_settings.json`](/d:/GitHub/road-editor/data/view_settings.json) に保存されます。

## 地形ウィンドウ

主な項目:

- 表示
- ワイヤー
- 表示モード
- ライティング
- 地形テクスチャー
- ハイトマップ
- 分割数 X / Z
- サイズ X / Y / Z (m)
- オフセット X / Y / Z (m)

初期値:

- 分割数: `1024 x 1024`
- サイズ: `1024 x 1024 x 1024`

ハイトマップと地形テクスチャーのパス表示は、プロジェクトファイル基準の相対パスです。  
内部では絶対パスへ解決して読み込みます。

ライティングは Half-Lambert です。スペキュラーは使っていません。

## 道路エディタ

主なモード:

- オブジェクト選択
- ポイント編集
- 道路作成
- 交差点作成
- 交差点編集
- 経路探索

補助機能:

- 地形にスナップ
- ポイントにスナップ

ポイントスナップでは、道路頂点や交差点を移動中に近くのポイントへ吸着します。  
道路頂点を移動する場合、同じ道路内の他頂点には吸着しません。

## プロパティ

`プロパティ` ウィンドウでは、グループ、道路、ポイント、交差点の詳細を編集できます。

交差点プロパティには `接続距離 (m)` があります。  
保存キーは `entryDist`、既定値は `8.0` です。

## 保存仕様

### プロジェクトファイル

プロジェクト保存時、次のパスはプロジェクトファイル基準の相対パスで保存されます。

- `terrain.path`
- `terrain.texturePath`
- `roads.path`

### 道路ファイル

道路頂点は `point[].pos` 配列で保存します。  
`points[]` の `x / y / z` 形式は新規保存では出力しません。

旧データ読込時は、次のような項目も保持します。

- `bankAngle`
- `laneSection`
- `verticalCurve`
- ポイントごとの追加パラメータ

### 表示設定

次の設定は [`data/view_settings.json`](/d:/GitHub/road-editor/data/view_settings.json) に保存されます。

- 背景設定
- エディタ表示設定
- 最近開いたプロジェクト
- ウィンドウの開閉状態
- 等高線間隔

## 旧フォーマット統合

[`data/test/new_curve.json`](/d:/GitHub/road-editor/data/test/new_curve.json) には、旧フォーマットの道路 / 交差点データを統合したサンプルがあります。

## ショートカット

カメラ:

- 回転: `Alt + 左ドラッグ`
- パン: `Alt + 中ドラッグ`
- ズーム: マウスホイール
- フォーカス: `F`

編集:

- Undo: `Ctrl + Z`
- Redo: `Ctrl + Y`
- 削除: `Delete`
- 移動 gizmo: `W`
- Y 回転 gizmo: `E`
- XZ 拡大 gizmo: `R`

## 主な実装ファイル

- [`src/App.cpp`](/d:/GitHub/road-editor/src/App.cpp)
- [`src/editor/PolylineEditor.cpp`](/d:/GitHub/road-editor/src/editor/PolylineEditor.cpp)
- [`src/editor/RoadData.cpp`](/d:/GitHub/road-editor/src/editor/RoadData.cpp)
- [`src/scene/Terrain.cpp`](/d:/GitHub/road-editor/src/scene/Terrain.cpp)
- [`shaders/terrain_ps.hlsl`](/d:/GitHub/road-editor/shaders/terrain_ps.hlsl)
- [`shaders/grid_ps.hlsl`](/d:/GitHub/road-editor/shaders/grid_ps.hlsl)

## ライセンス

MIT
