# xpbd-body MCP 化 STATUS（Phase 2）

> 最終更新: 2026-08-22T02:53Z

## 進捗

| 項目 | 状態 | 備考 |
|------|------|------|
| Phase 1 survey | 完了 | `docs/mcp/survey.json` / `docs/mcp/SURVEY.md` decision=library-serve |
| DESIGN.md | 完了 | `docs/mcp/DESIGN.md` namespace=xpbd, port=9204 |
| MCP サーバ実装 | 完了 | `mcp/server.mjs` (Streamable HTTP /mcp + /healthz) |
| テスト | 完了 | `mcp/test.mjs` 14 passed, 0 failed / `node test.mjs` 48 passed |
| volta.service.json | 完了 | root に配置 |
| run.sh / systemd unit | 完了 | `run.sh`（Node 20 via nvm）, `deploy/xpbd-body.service` |
| skill | 完了 | `docs/skills/xpbd-body-mcp-usage/SKILL.md` + resource `skill://xpbd-body-mcp-usage` |
| issue-hub 協調 | 完了 | #241(motion) #242(keiko) #243(sotai) にコメント追記 |
| dry-run svc_add | 完了 | exists=false, 新規登録, exit=0 |
| dry-run gateway_routes_diff | 完了 | xpbd 1 件のみ追加確認（保護1・温存2は既存設定） |
| svc_add confirm | 完了 | previous=null（新規）, exit=0, warnings は既存サービスの既知のもののみ |
| gateway_routes_apply | 完了 | [新規] xpbd.unlaxer.org -> http://192.168.1.50:9204, SIGHUP 済み |
| prod 配置・起動 | 完了 | git clone + npm install + systemd enable --now, Node 20.20.1 で稼働 |
| healthz 200 | 完了 | https://xpbd.unlaxer.org/healthz → 200 `{"ok":true,"name":"xpbd-body","version":"0.4.0"}` |
| backend_status ready | 完了 | catalog__backend_status: xpbd status=ready, tools=5, connectedAt=2026-08-22T02:53:08Z |
| audit | 完了 | 9 ok / 0 ng / 2 skip / 1 unknown（content_encoding: healthz は identity 確認済み） |

## dry-run 結果

### svc_add (dry-run → confirm)
- dry-run: `exists: false`（新規）, `exit: 0`
- confirm: `previous: null`（新規登録成功）, `exit: 0`
- エントリ: host=192.168.1.50, port=9204, runtime=systemd, namespace=xpbd, min_role=MEMBER, health_check=/healthz
- MCP 項: enabled=true, transport=http, port=9204, path=/mcp, namespace=xpbd, timeoutMs=110000
- warnings: 自分に関するものは無し（全て既存サービスの既知の警告）

### gateway_routes_diff (svc_add 後)
- 既存 routing: 65 件 / services.json から導出: 64 件 / マージ後: 66 件
- [新規] xpbd.unlaxer.org -> http://192.168.1.50:9204（自分の 1 件のみ）
- [保護] auth.unlaxer.org.public（既存の保護設定）
- 温存（2件）: adoyose-admin, mahjong-mcp（既存の手動設定）
- → 自分の 1 件以外を含まない → confirm で apply

### gateway_routes_apply
- job_id: 1ced66d8027f459081c003ff7980ee00
- state: done, exit: 0
- バックアップ: /home/opa/volta-gateway/volta-gateway.yaml.bak-generated-20260822-025049
- SIGHUP 済み（瞬断なし）

## 協調 issue（issue-hub）

| issue | 相手 | 方向 | 状態 |
|-------|------|------|------|
| #241 | motion-engine | depends_on (ポーズ形式) | コメント追記済み（暫定仕様で進行） |
| #242 | keiko-engine | provides_to (制御推定結果 schema) | コメント追記済み（暫定仕様で進行） |
| #243 | sotai-engine | depends_on (ROM データ) | sotai から回答あり（暫定合意成立） |

## 停止条件のチェック

- [x] テストが通る: 14/14 passed (mcp:test), 48/48 passed (test)
- [x] ポート 9204 は割当表どおり（machine_ports で空き確認済み）
- [x] svc_add は新規（既存サービス上書きなし）
- [x] gateway_routes_diff が自分の 1 件のみ（保護/温存は既存設定）
- [x] healthz が 200（https://xpbd.unlaxer.org/healthz）
- [x] backend_status が ready（catalog__backend_status: xpbd ready, tools=5）
- [x] audit 0 ng（9 ok / 0 ng / 2 skip / 1 unknown）

## デプロイで発生した問題と対応

1. **Node 18 で `crypto is not defined`**: prod のデフォルト Node が v18.19.1。`@modelcontextprotocol/sdk` が `globalThis.crypto` を使うため Node 20+ が必要。run.sh で nvm の Node 20 を使うように修正して解決。
2. **git clone が main をチェックアウト**: prod で `git clone` すると main が選択される。`git checkout release/v0.4.0` で切り替え。今後の更新は `git pull` で release/v0.4.0 を追跡。

## tools（5）

| tool | 説明 |
|------|------|
| `xpbd__simulate` | 目標ポーズを物理追従させる（前向きシミュレーション） |
| `xpbd__estimate_control_start` | 観測軌道から制御を逆推定（job 開始） |
| `xpbd__estimate_control_status` | 推定 job の進捗取得 |
| `xpbd__estimate_control_result` | 推定 job の最終結果取得 |
| `xpbd__check_feasible` | 物理可能性判定（ROM/torque/residual） |

## resources

- `xpbd://spec` — 能力仕様（JSON）
- `xpbd://guide` — 使い方ガイド
- `xpbd://inverse-dynamics` — 逆動力学の理論と限界
- `xpbd://theory-to-model` — XPBD 理論→実装マッピング
- `skill://xpbd-body-mcp-usage` — 組み合わせ手順（SKILL.md）

## 未決事項

- motion-engine のポーズ形式（{bone:[x,y,z]} Euler XYZ）の最終確認（#241）
- keiko-engine が制御推定結果 schema を受理可能か（#242）
- 物理パラメータの較正（market research で未較正が課題）
- prod のデフォルト Node が v18 のため、run.sh で nvm Node 20 を明示使用中。恒久対応として prod のデフォルト Node を 20+ に上げるか検討
