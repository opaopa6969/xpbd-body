---
name: xpbd-body-mcp-usage
description: VRM 上半身の制御・逆動力学を xpbd-body MCP tool として組み合わせる手順（ポーズ→物理追従、観測→制御推定、物理可能性判定）
volta:
  version: 1
  namespace: xpbd
  locality: service
  tags: [xpbd, physics, inverse-dynamics, ragdoll]
  applies_when:
    - service.id: xpbd-body
  requires:
    tools: [xpbd__simulate, xpbd__estimate_control_start, xpbd__estimate_control_result, xpbd__check_feasible]
    resources: [xpbd://spec, xpbd://guide]
  min_role: member
  export: allowed
---
# xpbd-body MCP で制御・逆動力学を回す

VRM 上半身の XPBD 物理エンジンを MCP tool として使う。詳細は `xpbd://guide` と `xpbd://spec` を読む。ここは判断基準と順序だけ。

## tools
- `xpbd__simulate`: 目標ポーズ → 物理追従（前向き）。同期・30 秒以内・決定論的。
- `xpbd__estimate_control_start` → `xpbd__estimate_control_status` → `xpbd__estimate_control_result`: 観測軌道 → 制御推定（job 型・30 秒超あり）。
- `xpbd__check_feasible`: 観測軌道の物理可能性判定（ROM / トルク / 残差）。同期。

## 順序
1. **目的を決める**: 物理追従（simulate）/ 制御推定（estimate_control）/ 可能性判定（check_feasible）のどれか。
2. **rig_opts を組む**: `makeRig` のオプション（mass/compliance/dt/substeps/rom/maxTorque/seed）。省略時はデフォルト。`rom` は sotai-engine 由由を想定（`{bone:[lo,hi]×3}` Euler XYZ）。
3. **simulate**: `control_trajectory=[{pose:{bone:[x,y,z]}}]`（Euler XYZ radians, child-in-parent）→ `frames=[{t,rel,pos,torque}]`。
4. **estimate_control**: `observed_trajectory`（`simulate` の出力をそのまま渡せる）→ `start` で `job_id` → `status` で poll → `result` で `control` + `residual` + `feasible` + `violations`。
5. **check_feasible**: `observed_trajectory` → `feasible` + `violations`。`feasible:false` なら `violations` で原因（rom/torque/residual）を特定。

## 判断基準
- **逆問題は ill-posed**（`unique: false`）。推定結果は「正解」ではなく「1 つの推定」。複数 seed で安定性を確認すること。
- **物理パラメータは engine units**（人体較正なし）。`mass/compliance/maxTorque` の解釈に注意。
- **長時間の estimate_control**: full upper body + 実データでは数分かかる可能性。job 型なので `status` で poll すること。
- **入力形式**: ポーズは `{bone:[x,y,z]}` Euler XYZ（motion-engine 由由を想定）。軌道は `{schemaVersion,dt,steps,bones,frames}`。

## 組み合わせ
1. `motion__pose`（目標ポーズ）→ `xpbd__simulate`（物理追従）→ レンダラー（`showcase__render` 等）で可視化
2. 動画解析 → `xpbd__estimate_control_start` → `xpbd__estimate_control_result` → `keiko__train`（稽古ループ）
3. `xpbd__check_feasible` → `feasible:false` なら `violations` で原因特定 → `sotai__rom_lookup` で ROM 照合 → リギング修正
