# xpbd-body MCP 化調査（Phase 1）

## 概要

`xpbd-body` は VRM 上半身向けの XPBD 能動ラグドール物理エンジンである。motion-engine の L3（動力学）層として、運動学的に生成された目標ポーズを、質量・重力・接触・筋肉の柔らかさを持つ物理状態に変換する。さらに M4 では観測軌道から制御を逆推定する（analysis by synthesis）。

- **種類**: library（npm package、依存ゼロ、HTTP API / CLI / MCP なし）
- **規模**: index.js 341 行 + inverse.js 496 行 + test.mjs 391 行、28 export、runtime dependency 0
- **決定論的**: 固定サブステップ、`Math.random` 不使用、ヘッドレス Node で単体テスト済み（48 passed / 0 failed）
- **バージョン**: 0.4.0（M1〜M4 実装済み）

## 判定と理由

**判定: `library-serve`**（ライブラリを新規サーバ化して volta 参加）

### 採用の根拠

1. **エージェントが直接呼べると価値が高い能力が複数ある**:
   - ポーズ→物理追従シミュレーション（`simulate`）
   - 観測軌道→制御推定（`estimateControl`、逆動力学）
   - 物理可能性判定（`checkFeasible`、ROM / トルク / 残差の 3 つの観点）
2. **他サービスとの組み合わせで価値が出る**: motion-engine のポーズを入力→物理追従→keiko-engine の稽古ループで制御推定、というパイプラインが想定されており、MCP 化することでこの連携がエージェント経由で組める
3. **既存 API が MCP に適した性質を持つ**: plain-data in/out、副作用なし、決定論的。tool の入出力としてそのまま使える
4. **常駐プロセスの価値がある**: `makeRig` / `makeUpperBody` で構築したリグ（World + Body + Constraint のオブジェクトグラフ）を再利用でき、`estimateControl` は数百回の forward simulate を回すため起動コストを償却できる
5. **重複しない**: volta カタログに xpbd-body / motion-engine / keiko-engine は未登録

## 公開候補

| kind | name | io | 副作用 | 長時間 | 対応 |
|------|------|----|--------|--------|------|
| tool | `simulate` | `{ rig_opts, control_trajectory, dt, steps, substeps? } → { frames:[{ t, rel, pos, torque }] }` | read | No | `inverse.js:simulate()` |
| tool | `estimate_control` | `{ rig_opts, observed_trajectory, opts } → { control, residual, feasible, violations, unique, evals }` | read | **Yes** | `inverse.js:estimateControl()` |
| tool | `check_feasible` | `{ rig_opts, observed_trajectory, simulated?, opts } → { feasible, violations }` | read | No | `inverse.js:checkFeasible()` |
| resource | `spec` | `xpbd://spec` — 能力の機械可読仕様 | - | - | - |
| resource | `guide` | `xpbd://guide` — 使い方 | - | - | - |
| resource | `inverse-dynamics` | `xpbd://inverse-dynamics` — 逆動力学の理論と限界 | - | - | `docs/inverse-dynamics.md` |
| resource | `theory-to-model` | `xpbd://theory-to-model` — XPBD 理論→実装マッピング | - | - | `docs/from-theory-to-model.md` |
| skill | `xpbd-body-mcp-usage` | VRM 上半身の制御・逆動力学を組み合わせる手順 | - | - | locality: service |

### tool 設計の補足

- **`simulate`**: 1フレームあたり 3.6ms（3-bone arm, substeps=8）〜43ms（full upper body, substeps=20）。通常は 30 秒以内で完了するが、長い軌道（120フレーム×20サブステップ）では境界付近に来る。当面は同期的でよいが、長時間化する場合は job 型にする。
- **`estimate_control`**: 合成往復検証で 178 回の forward simulate、~400ms（3-bone arm）。実データでは数千回の評価が必要になり **30 秒を超える可能性が高い**。**job 型化が必須**（`estimate_control_start` → `estimate_control_status` → `estimate_control_result`）。
- **`check_feasible`**: 観測軌道の ROM / トルク / 残差チェック。軽量（1回の simulate または既存結果で判定可能）。

## 組み合わせ例

1. **ポーズ→物理追従→描画**: `motion-engine` のポーズ生成 → `xpbd__simulate` で物理追従 → `showcase__*` 等でレンダリング。目標ポーズを物理的に自然な動きに変換して可視化。

2. **観測→制御推定→稽古ループ**: 動画解析（2Dポーズ推定→3Dポーズ変換）→ `xpbd__estimate_control` で制御推定 → `keiko-engine` の稽古ループで動きの記述子を取得。

3. **物理可能性チェック→フィードバック**: `xpbd__check_feasible` で観測動作の物理可能性を判定 → `feasible: false` なら `violations` で原因（ROM / torque / residual）を特定 → リギング調整 or モーション修正へフィードバック。

## 依存と協調

| 相手 repo | 方向 | 能力 | 現在あるか | 備考 |
|-----------|------|------|-----------|------|
| motion-engine | depends_on | 目標ポーズ生成（L2 運動学層）。xpbd-body は `{bone:[x,y,z]}` Euler XYZ ポーズを入力として受け取る | No | volta カタログ未登録。MCP 化されていない。Phase 2 で協調必要 |
| keiko-engine | provides_to | 逆動力学の制御推定結果（control + residual + feasible）。keiko-engine の稽古ループが `estimateControl` を消費する | No | volta カタログ未登録。Phase 2 で協調必要 |
| sotai-engine | depends_on | 関節可動域（ROM）データ。`makeRig` の `rom` オプションに sotai-engine の ROM が入る | No | README で言及。存在・MCP 化ともに未確認 |

### 協調の必要性

3 つの依存リポジトリ（motion-engine, keiko-engine, sotai-engine）がいずれも volta カタログに未登録・MCP 化されていない。end-to-end のパイプラインを組むには Phase 2 でこれらを協調させる必要がある。特にポーズデータの schema（`{bone:[x,y,z]}` Euler XYZ）を共通フォーマットとして定義し、サービス間で整合させる必要がある。

## ライブラリのサーバ化

| 項目 | 内容 |
|------|------|
| needed | true |
| runtime | node |
| estimated_effort | M |

### 新規に実装する必要があるもの

1. **healthz エンドポイント**（`GET /healthz` → 200）
2. **PORT 環境変数対応**（`0.0.0.0` bind、ファサードは LAN 越しに来る）
3. **MCP サーバ実装**（Streamable HTTP `/mcp`）
4. **volta.service.json manifest**（リポジトリ root に配置）
5. **systemd user unit または docker 設定**
6. **長時間 tool の job 型化**（`estimate_control_start` → `estimate_control_status` → `estimate_control_result`）

### 設計の方向性

- リグ（World + Body chain + Motor constraints）の構築はサーバ側で保持し、tool 呼び出しで再利用できるようにする（セッションまたはリグ ID で管理）
- `simulate` は同期的に結果を返す（通常 30 秒以内）
- `estimateControl` は job 型にする。`estimate_control_start` でジョブ ID を返し、`estimate_control_status` で進捗（evals 数 / 現在の best residual）、`estimate_control_result` で最終結果
- リグの構築オプション（skeleton, mass, compliance, profile, rom, maxTorque）は tool の引数または事前設定として渡す

## リスク

- **長時間処理**: `estimateControl` は数百回の forward simulate を実行する。3-bone arm で 400ms だが、full upper body + 実データでは数分かかる可能性がある。job 型化が必須。
- **物理パラメータの未較正**: mass / compliance / maxTorque は engine units であり、人体の物理量に較正されていない。エージェントが結果を解釈する際に「これは相対的な指標であり、絶対値ではない」ことを理解する必要がある。market research でも課題として挙がっている。
- **逆問題の不良設定性**: `unique: false`。複数の制御が同じ動きを生む。`estimateControl` が返すのは「正解」ではなく「残差と平滑化正則化が選んだ 1 つの推定」。`smooth` パラメータを変えれば別の equally valid な答えが出る。
- **依存の未 MCP 化**: motion-engine / keiko-engine / sotai-engine がいずれも未 MCP 化のため、現時点では end-to-end の組み合わせができない。Phase 2 で協調が必要。

## 持ち主への質問

1. **ポーズ schema**: motion-engine のポーズ形式（`{bone:[x,y,z]}` Euler XYZ）を MCP 越しにそのまま渡してよいか、それとも共通 schema を定義すべきか？
2. **estimateControl のタイムアウト**: job 型化のタイムアウトをどの程度に設定すべきか？ 3-bone arm で 400ms、full upper body で数秒だが、実データではもっと長くなる可能性がある。
3. **sotai-engine**: 実在するか？ 存する場合、ROM データを resource として公開すべきか、xpbd-body 側に埋め込むか？
4. **物理パラメータの較正**: 誰が・どう行うか？ market research でも未較正が課題として挙がっている。
