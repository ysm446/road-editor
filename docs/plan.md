# DirectX 11 Road Network Editor — プロジェクト計画書

## 概要

Houdiniの道路エディタHDAで培った設計思想をベースに、DirectX 11 + Dear ImGui によるスタンドアロンの道路ネットワークエディタを構築する。Claude Codeを活用し、フェーズごとに段階的に実装を進める。

---

## 技術スタック

| 項目 | 選定 | 備考 |
|------|------|------|
| グラフィックスAPI | DirectX 11 | 安定性・情報量が豊富 |
| UIフレームワーク | Dear ImGui | 軽量・ツール向き、D3D11バックエンド公式サポート |
| 数学ライブラリ | DirectXMath | D3D11と自然に統合 |
| ビルドシステム | CMake | Claude Codeとの相性が良い |
| 言語 | C++17 | |
| シェーダー | HLSL (Shader Model 5.0) | D3DCompile でランタイムコンパイル |

### 外部ライブラリ

- **Dear ImGui** (docking branch) — UI全般、ビューポート統合
- **DirectXTex** (任意) — テクスチャ読み込み
- **stb_image.h** — 軽量な画像読み込み（ハイトマップ等）
- **nlohmann/json** — 道路データのJSON保存/読み込み

---

## プロジェクト構成

```
RoadEditor/
├── CMakeLists.txt
├── extern/
│   ├── imgui/                    # Dear ImGui ソース + D3D11バックエンド
│   │   ├── imgui.h / imgui.cpp
│   │   ├── imgui_impl_win32.h / .cpp
│   │   ├── imgui_impl_dx11.h / .cpp
│   │   └── ...
│   ├── stb/                      # stb_image.h
│   └── json/                     # nlohmann/json.hpp
├── src/
│   ├── main.cpp                  # WinMain, ウィンドウ作成, メインループ
│   ├── App.h / App.cpp           # アプリケーション管理クラス
│   ├── renderer/
│   │   ├── D3D11Context.h / .cpp # デバイス・スワップチェーン・レンダーターゲット管理
│   │   ├── Shader.h / .cpp       # HLSLコンパイル・シェーダー管理
│   │   ├── Buffer.h / .cpp       # 頂点/インデックス/定数バッファのラッパー
│   │   └── DebugDraw.h / .cpp    # ライン・ポイント描画ユーティリティ
│   ├── scene/
│   │   ├── Camera.h / .cpp       # オービットカメラ（回転・ズーム・パン）
│   │   ├── Grid.h / .cpp         # 無限グリッド描画
│   │   └── Terrain.h / .cpp      # ハイトフィールドメッシュ描画
│   ├── editor/
│   │   ├── EditorState.h         # エディタモード管理（enum + ステートパターン）
│   │   ├── PolylineEditor.h / .cpp   # ポリライン配置・編集
│   │   ├── ControlPointEditor.h / .cpp # コントロールポイント操作
│   │   ├── RoadGenerator.h / .cpp     # ポリライン→道路メッシュ変換
│   │   └── RoadData.h / .cpp     # 道路データ構造・JSON入出力
│   └── ui/
│       ├── ImGuiLayer.h / .cpp   # ImGui初期化・フレーム管理
│       ├── ViewportPanel.h / .cpp # 3Dビューポートパネル
│       ├── PropertyPanel.h / .cpp # 選択オブジェクトのプロパティ表示
│       ├── LayerPanel.h / .cpp    # 道路レイヤー管理
│       └── ToolBar.h / .cpp       # ツールバー（モード切り替え）
├── shaders/
│   ├── common.hlsli              # 共通の定数バッファ定義
│   ├── grid_vs.hlsl / grid_ps.hlsl
│   ├── terrain_vs.hlsl / terrain_ps.hlsl
│   ├── road_vs.hlsl / road_ps.hlsl
│   └── line_vs.hlsl / line_ps.hlsl  # デバッグライン描画
└── data/
    └── sample_heightmap.png      # テスト用ハイトマップ
```

---

## 実装フェーズ

### Phase 1: 基盤構築 — D3D11 + ImGui + カメラ

**目標**: ウィンドウが開き、3Dビューポート内でオービットカメラ操作ができる状態

#### タスク

1. **Win32ウィンドウ作成**
   - WinMain でウィンドウクラス登録・作成
   - メッセージループ（WM_SIZE でリサイズ対応）
   - ImGui の WndProc フック統合

2. **D3D11 デバイス初期化** (`D3D11Context`)
   - ID3D11Device / ID3D11DeviceContext 作成
   - IDXGISwapChain 作成（DXGI_FORMAT_R8G8B8A8_UNORM）
   - レンダーターゲットビュー / 深度ステンシルビュー作成
   - ウィンドウリサイズ時のバッファ再作成

3. **Dear ImGui 統合** (`ImGuiLayer`)
   - imgui_impl_win32 + imgui_impl_dx11 初期化
   - Docking 有効化（ImGuiConfigFlags_DockingEnable）
   - フレーム開始/終了/レンダリングのラッパー

4. **オービットカメラ** (`Camera`)
   - パラメータ: target, distance, azimuth, elevation
   - マウス操作: 中ボタンドラッグで回転、Shift+中ボタンでパン、ホイールでズーム
   - View / Projection 行列の計算（DirectXMath）
   - ImGuiがマウスをキャプチャしている時は入力無視

5. **グリッド描画** (`Grid`)
   - XZ平面上の無限風グリッド（シェーダーベース）
   - カメラ距離に応じてグリッド密度を変更
   - フェードアウトで地平線を自然に処理

6. **定数バッファ構造**
   ```hlsl
   cbuffer PerFrame : register(b0) {
       float4x4 viewProj;
       float3   cameraPos;
       float    time;
   };
   ```

#### 完了条件
- ウィンドウが起動し、グリッドが表示される
- マウスでカメラ操作（回転・パン・ズーム）ができる
- ImGuiのデモウィンドウが表示される
- ウィンドウリサイズが正しく動作する

---

### Phase 2: 地形表示

**目標**: ハイトマップ画像から地形メッシュを生成・描画する

#### タスク

1. **ハイトマップ読み込み**
   - stb_image でグレースケール画像読み込み
   - 高さスケール・水平スケールのパラメータ化

2. **地形メッシュ生成** (`Terrain`)
   - ハイトマップからグリッドメッシュ（頂点 + インデックス）生成
   - 法線計算（隣接ピクセルからの差分法）
   - 頂点フォーマット: position(float3), normal(float3), uv(float2)

3. **地形シェーダー** (`terrain.hlsl`)
   - 高度ベースのカラーリング（低地=緑、高地=茶→白）
   - 法線による基本的なランバートライティング
   - ワイヤフレーム/ソリッド/ワイヤフレーム+ソリッドの切り替え

4. **地形 ImGui パネル**
   - 高さスケール・水平スケールのスライダー
   - 描画モード切り替え（ソリッド/ワイヤフレーム）
   - ハイトマップファイルパス指定

#### 完了条件
- ハイトマップ画像から3D地形が表示される
- カメラで地形を自由に閲覧できる
- ImGuiパネルで地形パラメータを調整できる

---

### Phase 3: ポリライン編集 — 道路配置

**目標**: 地形上にマウスクリックでポリラインを配置し、コントロールポイントを編集できる

#### タスク

1. **レイキャスト** 
   - スクリーン座標 → ワールドレイ変換
   - レイ vs 地形メッシュの交差判定（三角形単位）
   - ヒットポイントのワールド座標取得

2. **エディタモード管理** (`EditorState`)
   - Houdini ViewerState に倣ったモード設計:
     ```cpp
     enum class EditorMode {
         Navigate,       // カメラ操作のみ
         PolylineDraw,   // ポリライン描画（クリックで点追加）
         PointEdit,      // コントロールポイント移動
         // 将来拡張:
         // VerticalCurve, BankAngle, LaneEdit
     };
     ```
   - モード切り替えはツールバーまたはキーボードショートカット

3. **ポリライン編集** (`PolylineEditor`)
   - クリックでポイント追加（地形上にスナップ）
   - ポイント選択（近接判定）・ドラッグ移動
   - ポイント削除（Delete キー）
   - Enter/ダブルクリックでポリライン確定
   - Esc でキャンセル

4. **道路データ構造** (`RoadData`)
   ```cpp
   struct RoadPoint {
       DirectX::XMFLOAT3 position;
       float width = 6.0f;       // 車線幅（デフォルト6m）
       float bankAngle = 0.0f;   // バンク角
       // 将来: superelevation, vertical curve params
   };

   struct Road {
       std::string id;
       std::vector<RoadPoint> points;
       int laneCount = 2;
       // メタデータ
   };

   struct RoadNetwork {
       std::vector<Road> roads;
       // JSON 保存/読み込み
       void SaveToJson(const std::string& path);
       void LoadFromJson(const std::string& path);
   };
   ```

5. **ポリライン描画**
   - GL_LINES 相当のライン描画（DebugDraw利用）
   - コントロールポイントのビルボード描画
   - 選択状態のハイライト（色変更）

6. **ImGui パネル**
   - ToolBar: モード切り替えボタン
   - PropertyPanel: 選択ポイント/道路のプロパティ編集
   - LayerPanel: 道路一覧・表示/非表示・選択

#### 完了条件
- 地形上にクリックでポリラインを配置できる
- コントロールポイントの選択・移動・削除ができる
- 複数の道路ポリラインを管理できる
- JSON で保存/読み込みができる

---

### Phase 4: 道路メッシュ生成

**目標**: ポリラインから道路メッシュを自動生成し、地形に沿って描画する

#### タスク

1. **ポリライン → スプライン変換**
   - Catmull-Rom または B-Spline 補間
   - 等間隔リサンプリング（道路メッシュ解像度に合わせて）

2. **道路断面生成**
   - スプライン上の各サンプル点で断面ポリゴンを配置
   - Frenet-Serret フレーム or 最小回転フレーム（RMF）で方向決定
   - Houdiniの Up vector smoothing の知見を活用:
     - カーブUVベースのガウシアンサンプリングでUp vectorを平滑化
     - スイッチバック道路でも正しく動作するようにする

3. **メッシュ生成** (`RoadGenerator`)
   - 断面ポリゴンの連結 → 三角形メッシュ
   - UV座標の生成（u: 道路横断方向, v: 道路長手方向）
   - 車線幅・バンク角パラメータの反映
   - 地形との交差処理（道路メッシュを地形に沿わせる or 切土/盛土）

4. **道路シェーダー** (`road.hlsl`)
   - アスファルト風テクスチャリング
   - 車線マーキング（UVベースのプロシージャル描画）
   - 基本ライティング

5. **部分再計算システム**（Houdiniの設計を移植）
   - 道路IDごとのハッシュ比較
   - 変更があった道路のみメッシュを再生成
   - キャッシュ管理

#### 完了条件
- ポリラインから道路メッシュが自動生成される
- 車線幅・バンク角を変更するとメッシュが更新される
- 地形に沿った道路が描画される
- 変更のない道路はキャッシュから再利用される

---

## Phase 間で共通の設計方針

### 入力処理のルール
- ImGui が入力をキャプチャしている間は3Dビューポートの入力を無効化
  ```cpp
  if (!ImGui::GetIO().WantCaptureMouse) {
      // 3Dビューポートのマウス入力を処理
  }
  ```
- モードごとにマウス/キーボードの振る舞いを分離（ステートパターン）

### レンダリング順序
1. 地形（不透明）
2. 道路メッシュ（不透明、深度バイアスで地形より手前に）
3. グリッド（半透明）
4. ポリライン・コントロールポイント（オーバーレイ）
5. ImGui（最前面）

### Undo/Redo（将来対応）
- コマンドパターンでの実装を想定
- 各編集操作を `ICommand` として記録
- Phase 3 以降で段階的に導入

---

## Claude Code への指示方法

各フェーズの作業を Claude Code に依頼する際のプロンプト例:

### Phase 1 の開始
```
このプロジェクト計画書（ROAD_EDITOR_PLAN.md）に従って、Phase 1 を実装してください。
まず CMakeLists.txt とプロジェクト構成を作成し、
その後 Win32ウィンドウ + D3D11初期化 + ImGui統合 + オービットカメラ + グリッド描画を
順番に実装してください。
```

### 個別タスクの依頼
```
Phase 2 の地形シェーダーを実装してください。
terrain_vs.hlsl / terrain_ps.hlsl を作成し、
高度ベースカラーリングとランバートライティングを入れてください。
ワイヤフレーム切り替えも対応してください。
```

### デバッグ依頼
```
D3D11のデバッグレイヤーでエラーが出ています:
[エラーメッセージ]
D3D11Context.cpp の該当箇所を修正してください。
```

---

## 参考: Houdini道路エディタとの対応関係

| Houdini (ViewerState + HDA) | DirectX エディタ |
|-----|-----|
| ViewerState のモード管理 | EditorState enum + ステートパターン |
| hou.session によるコールバック間通信 | App クラスの共有ステート |
| Multiparm ベースのレイヤーパネル | ImGui の LayerPanel |
| VEX による道路メッシュ生成 | C++ の RoadGenerator |
| JSON ベースの道路データ管理 | RoadData + nlohmann/json |
| カーブUVベースのガウシアンサンプリング | 同ロジックをC++に移植 |
| Per-road-ID ハッシュ比較による部分再計算 | 同システムをC++に移植 |

---

## 注意事項

- Dear ImGui は **docking ブランチ** を使用すること（ビューポート分割に必要）
- D3D11 のデバッグレイヤーは開発中常に有効にする（`D3D11_CREATE_DEVICE_DEBUG`）
- シェーダーは `D3DCompileFromFile` でランタイムコンパイル（開発中はホットリロード対応可）
- DirectXMath は行優先（row-major）だが、HLSL はデフォルト列優先（column-major）なので、定数バッファ送信時に転置するか `row_major` 指定する