# Road Editor

DirectX 11 と Dear ImGui で作られた道路編集ツールです。  
ハイトマップ地形の上で道路ポリラインと交差点を編集し、JSON 形式で保存できます。

## 主な機能

- ハイトマップの読み込み
- 地形サイズ、分割数、オフセットの調整
- 道路の作成、編集、保存、読込
- 交差点の作成、編集、保存
- 道路端点の交差点スナップ
- 近いポイントへのスナップ
- グループ管理
- Undo / Redo
- 最近開いたプロジェクト
- 背景、グリッド、エディタ表示の調整
- 旧フォーマット道路 / 交差点 JSON の読込互換

## 動作環境

- Windows 10 / 11
- Visual Studio 2022 または 2026
- CMake 3.20 以上
- DirectX 11 が動作する GPU

## ビルド

### CMake が PATH に通っている場合

```powershell
cmake -B build -S . -G "Visual Studio 18 2026" -A x64
cmake --build build --config Debug
```

Visual Studio 2022 を使う場合:

```powershell
cmake -B build -S . -G "Visual Studio 17 2022" -A x64
cmake --build build --config Debug
```

### PowerShell で `cmake` が見つからない場合

この環境では Visual Studio 同梱の `cmake.exe` を直接呼べます。

```powershell
& 'C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' --build build --config Debug
```

初回構成から行う場合:

```powershell
& 'C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' -B build -S . -G "Visual Studio 18 2026" -A x64
& 'C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' --build build --config Debug
```

実行ファイル:

```powershell
.\build\Debug\RoadEditor.exe
```

## 画面構成

上部メニュー:

- `ファイル`
- `編集`
- `表示`
- `ウインドウ`
- `設定`

`ウインドウ` メニューから次のウィンドウを開閉できます。

- `地形`
- `カメラ`
- `道路エディタ`
- `プロパティ`

各ウィンドウは右上の閉じるボタンでも閉じられます。  
開閉状態は `data/view_settings.json` に保存されます。

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

`交差点を自動作成` は、未接続の道路端点が近接している場所に交差点が無い場合、新しい交差点を作成して端点を接続します。

## 表示メニュー

- 道路名
- 交差点名
- 道路プレビュー情報
- 道路勾配グラデーション
- グリッド
- FPS

## 設定メニュー

- `背景`
  - グリッドサイズ
  - フォグ距離
  - 背景色
- `エディタ表示`
  - 道路
    - 道路の太さ
    - 選択時の道路の太さ
    - 頂点サイズ
    - 色
    - 選択時の色
  - 交差点
    - 交差点サークルサイズ
    - 色

## 地形ウィンドウ

主な項目:

- 表示
- ワイヤー
- 表示モード
- ライティング
- 地形テクスチャ
- ハイトマップ
- 分割数 X / Z
- サイズ X / Z / Y (m)
- オフセット X / Y / Z (m)
- 等高線表示

初期値:

- 分割数: `1024 x 1024`
- サイズ: `1024 x 1024 x 1024`

標準ライティングは Half-Lambert です。スペキュラーは入っていません。

## 道路エディタウィンドウ

主に編集モード切り替えと編集補助を置いています。

- オブジェクト
- ポイント編集
- 道路作成
- 交差点作成
- 交差点編集
- 経路探索
- 地形にスナップ
- ポイントにスナップ

`ポイントにスナップ` を有効にすると、道路頂点や交差点の移動中に近いポイントへ吸着します。  
道路頂点を動かす場合は、同じ道路内の頂点には吸着しません。

## プロパティウィンドウ

次の詳細編集をまとめています。

- グループ
- 道路
- 選択中ポイント
- 交差点

交差点プロパティには `接続距離 (m)` があります。  
内部保存名は `entryDist` で、既定値は `8.0` です。

## 保存仕様

### プロジェクトファイル

プロジェクト保存時、次のパスはプロジェクトファイル基準の相対パスで保存されます。

- `terrain.path`
- `terrain.texturePath`
- `roads.path`

読み込み時は、プロジェクトファイルの場所から相対解決します。

### 道路 / 交差点 JSON

道路データは UUID ベースの JSON で保存されます。  
旧フォーマットに含まれる次の情報も保持して保存できます。

- `bankAngle`
- `laneSection`
- `verticalCurve`
- 各ポイントの追加パラメータ

### 表示設定

次の内容は `data/view_settings.json` に保存されます。

- 背景設定
- エディタ表示設定
- 最近開いたプロジェクト
- 各ウィンドウの開閉状態

## 操作

カメラ:

- 回転: `Alt + 左ドラッグ`
- パン: `Alt + 中ドラッグ`
- ズーム: マウスホイール
- フォーカス: `F`

編集:

- Undo: `Ctrl + Z`
- Redo: `Ctrl + Y`
- 削除: `Delete`
- gizmo 移動: `W`
- Y 回転 gizmo: `E`
- XZ 拡大 gizmo: `R`

## 旧フォーマット統合メモ

`data/test` 以下の旧道路 / 交差点データは、現行形式へ統合した JSON を作成できます。  
作成済みサンプル:

- `data/test/new_curve.json`

## 主要ファイル

- `src/App.cpp`
- `src/editor/PolylineEditor.cpp`
- `src/editor/RoadData.cpp`
- `src/scene/Terrain.cpp`
- `shaders/terrain_ps.hlsl`
- `shaders/grid_ps.hlsl`

## ライセンス

MIT
