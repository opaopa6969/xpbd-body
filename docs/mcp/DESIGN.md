# xpbd-body MCP 化設計（Phase 2）

> Phase 1 survey: `docs/mcp/survey.json` / `docs/mcp/SURVEY.md` 参照。割当表: `MCPIFY-phase2-plan.md` 行 #6 — namespace `xpbd`, port `9204`。

## 1. namespace と種別

- **namespace**: `xpbd`
- **種別**: `library-serve`（純粋 JS ライブラリを新規 MCP サーバ化して volta 参加）
- **runtime**: node
- **port**: 9204（割当表 #6。`volta__machine_ports` で空き確認済み）

## 2. tools 表

| name | 目的 | 入力 schema（要点） | 出力の形 | 副作用 | dry-run | job 型 | 所要時間 | min_role |
|------|------|---------------------|----------|--------|---------|--------|----------|----------|
| `simulate` | 目標ポーズを物理追従させる（前向きシミュレーション） | `{ rig_opts: { skeleton?, mass?, compliance?, profile? }, control_trajectory: [{pose:{bone:[x,y,z]}, compliance?}], dt?, steps?, substeps? }` | `{ schemaVersion, dt, steps, bones, frames:[{ t, rel:{bone:quat}, pos:{bone:[x,y,z]}, torque:{bone:number} }] }` | read | — | No | <30s | MEMBER |
| `estimate_control_start` | 観測軌道から制御を逆推定する（analysis by synthesis）を job として開始 | `{ rig_opts, observed_trajectory, opts:{bones?,knots?,method?,iters?,seed?} }` | `{ job_id, status: "queued" }` | read | — | **Yes** | 数秒〜数分 | MEMBER |
| `estimate_control_status` | estimate_control ジョブの進捗を取得 | `{ job_id }` | `{ job_id, status, evals, best_residual, elapsed_ms }` | read | — | — | — | MEMBER |
| `estimate_control_result` | estimate_control ジョブの最終結果を取得 | `{ job_id }` | `{ control, residual, feasible, violations, unique, evals, method }` または `{ status: "running" }` | read | — | — | — | MEMBER |
| `check_feasible` | 観測軌道の物理可能性を判定 | `{ rig_opts, observed_trajectory, simulated?, opts:{tol?,residualTol?} }` | `{ feasible, violations }` | read | — | No | <5s | MEMBER |

### 設計判断

- **`estimate_control` は job 型化**（`start` → `status` → `result`）。3-bone arm で 400ms だが full upper body + 実データでは 30 秒超の可能性が高い。job 管理はインメモリ Map（セッション ID = jobId）。
- **`simulate` / `check_feasible` は同期**。通常 30 秒以内。境界付近の長い軌道はユーザーが `steps` で調整可能。
- **副作用なし**: すべて read-only。リグ（World + Body + Constraint）は tool 呼び出し内で都度構築し、セッションをまたがない（stateless で堅牢）。
- **rig_opts**: `{ skeleton?, mass?, compliance?, profile?, gravity?, linDamp?, angDamp?, dt?, substeps?, rom?, maxTorque?, seed? }`。省略時は `makeRig` のデフォルト。
- **observed_trajectory / control_trajectory**: `{ schemaVersion, dt, steps, bones, frames:[{ t, rel:{bone:quat}, pos?:{bone:[x,y,z]} }] }` 形式。`simulate` の出力をそのまま `estimate_control` / `check_feasible` の入力に渡せる。

## 3. resources 表

| uri | 内容 | mime |
|-----|------|------|
| `xpbd://spec` | 能力仕様（JSON。tools/list から自動生成 + compositions/depends_on 手書き） | application/json |
| `xpbd://guide` | 使い方ガイド | text/markdown |
| `xpbd://inverse-dynamics` | 逆動力学の理論と限界（`docs/inverse-dynamics.md`） | text/markdown |
| `xpbd://theory-to-model` | XPBD 理論→実装マッピング（`docs/from-theory-to-model.md`） | text/markdown |

### spec resource の形

```jsonc
{
  "namespace": "xpbd",
  "name": "xpbd-body MCP server",
  "version": "0.4.0",
  "summary": "VRM 上半身向け XPBD 能動ラグドール物理エンジン。ポーズ→物理追従、逆動力学、物理可能性判定。",
  "capabilities": [
    { "kind": "tool", "name": "simulate", "summary": "目標ポーズを物理追従させる", "input": "{ rig_opts, control_trajectory, dt?, steps?, substeps? }", "output": "{ schemaVersion, dt, steps, bones, frames }", "side_effect": "read", "long_running": false, "dry_run": false, "min_role": "MEMBER" },
    { "kind": "tool", "name": "estimate_control_start", "summary": "観測軌道から制御を逆推定（job 開始）", "input": "{ rig_opts, observed_trajectory, opts }", "output": "{ job_id, status }", "side_effect": "read", "long_running": true, "dry_run": false, "min_role": "MEMBER" },
    { "kind": "tool", "name": "estimate_control_status", "summary": "推定 job の進捗", "input": "{ job_id }", "output": "{ job_id, status, evals, best_residual }", "side_effect": "read", "long_running": false, "dry_run": false, "min_role": "MEMBER" },
    { "kind": "tool", "name": "estimate_control_result", "summary": "推定 job の結果", "input": "{ job_id }", "output": "{ control, residual, feasible, violations }", "side_effect": "read", "long_running": false, "dry_run": false, "min_role": "MEMBER" },
    { "kind": "tool", "name": "check_feasible", "summary": "物理可能性判定", "input": "{ rig_opts, observed_trajectory, simulated?, opts }", "output": "{ feasible, violations }", "side_effect": "read", "long_running": false, "dry_run": false, "min_role": "MEMBER" }
  ],
  "compositions": [
    { "title": "ポーズ→物理追従→描画", "flow": ["motion__pose", "xpbd__simulate", "showcase__render"], "note": "motion-engine のポーズを xpbd-body で物理追従させ、レンダラーで可視化" },
    { "title": "観測→制御推定→稽古ループ", "flow": ["xpbd__estimate_control_start", "xpbd__estimate_control_result", "keiko__train"], "note": "観測軌道から制御を逆推定し、keiko-engine の稽古ループで動きの記述子を得る" },
    { "title": "物理可能性チェック→フィードバック", "flow": ["xpbd__check_feasible", "sotai__rom_lookup"], "note": "feasible:false なら violations で原因を特定し、ROM データと照合してリギング修正" }
  ],
  "depends_on": [
    { "namespace": "motion", "capability": "motion__pose（目標ポーズ生成）" },
    { "namespace": "keiko", "capability": "keiko__train（制御推定結果の消費）" },
    { "namespace": "sotai", "capability": "sotai__rom_lookup（関節可動域データ）" }
  ],
  "health": "/healthz",
  "docs": ["xpbd://guide", "xpbd://inverse-dynamics", "xpbd://theory-to-model"]
}
```

## 4. prompts / skills

### skill: `xpbd-body-mcp-usage`

- **用途**: VRM 上半身の制御・逆動力学を MCP tool として組み合わせる手順
- **locality**: `service`
- **applies_when**: xpbd-body MCP server を使ってシミュレーション or 逆動力学を行うとき
- **requires**: `xpbd` namespace が catalog に存在すること
- **min_role**: MEMBER
- **配置**: `docs/skills/xpbd-body-mcp-usage/SKILL.md` + resource `skill://xpbd-body-mcp-usage`

## 5. 組み合わせ例

1. **ポーズ→物理追従→描画**: `motion__pose`（目標ポーズ生成）→ `xpbd__simulate`（物理追従）→ `showcase__render`（可視化）。目標ポーズを物理的に自然な動きに変換して可視化する。

2. **観測→制御推定→稽古ループ**: 動画解析（2Dポーズ推定→3Dポーズ変換）→ `xpbd__estimate_control_start` → `xpbd__estimate_control_result`（制御推定）→ `keiko__train`（稽古ループで動きの記述子を取得）。

3. **物理可能性チェック→フィードバック**: `xpbd__check_feasible` で観測動作の物理可能性を判定 → `feasible:false` なら `violations` で原因（ROM/torque/residual）を特定 → `sotai__rom_lookup` で ROM データと照合しリギング調整。

## 6. 依存と協調

| 相手 repo | 方向 | 依存する/提供する入口 | 合意したいこと | issue-hub |
|-----------|------|-----------------------|----------------|-----------|
| motion-engine | depends_on | `motion__pose`（目標ポーズ生成）。xpbd-body は `{bone:[x,y,z]}` Euler XYZ ポーズを入力として受け取る | ポーズ形式 `{bone:[x,y,z]}` Euler XYZ を共通フォーマットとして確認 | issue-hub に登録 |
| keiko-engine | provides_to | 逆動力学の制御推定結果（`control` + `residual` + `feasible`）。keiko-engine の稽古ループが `estimateControl` の結果を消費 | 制御推定結果の schema（`xpbd-body/control@1`）を keiko-engine 側で受理可能か確認 | issue-hub に登録 |
| sotai-engine | depends_on | `sotai__rom_lookup`（関節可動域データ）。`makeRig` の `rom` オプションに sotai-engine の ROM が入る | ROM データの形式（`{bone:[lo,hi]×3}` Euler XYZ）を確認 | issue-hub に登録 |

いずれも相手 repo は未 MCP 化のため、暫定仕様で実装を進め、相手の MCP 化が進んだ時点で schema を調整する。

## 7. 非対応にした候補

Phase 1 からの差分なし。`makeArm` / `makeUpperBody` / `World` / `Body` 等の低レベル API は tool にせず、`rig_opts` 経由でオプションとして渡す（1 tool = 1 つのはっきりした操作の原則）。`snapshot` / `restore` は `simulate` 内部で使用し、外部には公開しない（サーバが stateless なため不要）。

## 8. 参加方法

### volta.service.json

```jsonc
{
  "id": "xpbd-body",
  "name": "xpbd-body MCP",
  "description": "VRM 上半身向け XPBD 能動ラグドール物理エンジン。ポーズ→物理追従、逆動力学、物理可能性判定。",
  "type": "node",
  "hostname": "xpbd.unlaxer.org",
  "port": 9204,
  "host": "192.168.1.50",
  "runtime": "systemd",
  "exec_start": "/home/opa/xpbd-body/run.sh",
  "user": "opa",
  "auth": "minRole:MEMBER",
  "health_check": "/healthz",
  "tags": ["mcp", "xpbd", "physics", "ragdoll", "inverse-dynamics"],
  "repo_url": "https://github.com/opaopa6969/xpbd-body",
  "mcp": {
    "enabled": true,
    "port": 9204,
    "path": "/mcp",
    "namespace": "xpbd",
    "min_role": "MEMBER",
    "timeoutMs": 110000,
    "description": "VRM 上半身 XPBD 物理エンジン"
  }
}
```

- **ホスト**: 192.168.1.50（prod）
- **runtime**: systemd user unit
- **auth**: minRole:MEMBER（送信元 IP 制限は gateway 側で処理）
- **ポート**: 9204（割当表 #6、空き確認済み）

## 9. テスト方針

e2e テスト（`mcp/test.mjs`）:
1. サーバ起動 → `GET /healthz` が 200
2. MCP クライアント接続 → `tools/list` に 5 tool が含まれる
3. `simulate` を最小構成（3-bone arm 相当）で実行 → frames が返る
4. `estimate_control_start` → `estimate_control_status` → `estimate_control_result` の job ライフサイクル
5. `check_feasible` で feasible / infeasible 両方を確認
6. `xpbd://spec` resource が JSON で返る
7. `xpbd://guide` resource が markdown で返る

使用ライブラリ: `@modelcontextprotocol/sdk` の `Client` + `StreamableHTTPClientTransport`。
