# xpbd-body MCP 化 STATUS（Phase 2）

> 最終更新: 2026-08-22

## 進捗

| 項目 | 状態 | 備考 |
|------|------|------|
| Phase 1 survey | 完了 | `docs/mcp/survey.json` / `docs/mcp/SURVEY.md` decision=library-serve |
| DESIGN.md | 完了 | `docs/mcp/DESIGN.md` namespace=xpbd, port=9204 |
| MCP サーバ実装 | 完了 | `mcp/server.mjs` (Streamable HTTP /mcp + /healthz) |
| テスト | 完了 | `mcp/test.mjs` 14 passed, 0 failed / `node test.mjs` 48 passed |
| volta.service.json | 完了 | root に配置 |
| run.sh / systemd unit | 完了 | `run.sh`, `deploy/xpbd-body.service` |
| skill | 完了 | `docs/skills/xpbd-body-mcp-usage/SKILL.md` + resource `skill://xpbd-body-mcp-usage` |
| issue-hub 協調 | 完了 | #241(motion) #242(keiko) #243(sotai) にコメント追記 |
| dry-run svc_add | 完了 | exists=false, 新規登録, exit=0 |
| dry-run gateway_routes_diff | 完了（svc_add 前） | xpbd ルート未出現（services.json 未更新のため）。温存2件・保護1件は既存設定 |
| svc_add confirm | 進行中 | ↓ |
| gateway_routes_apply | 未 | svc_add 後に再確認 |
| healthz 200 | 未 | |
| backend_status ready | 未 | |

## dry-run 結果

### svc_add (dry-run)
- `exists: false`（新規）
- `exit: 0`
- エントリ内容: host=192.168.1.50, port=9204, runtime=systemd, namespace=xpbd, min_role=MEMBER, health_check=/healthz
- MCP 項: enabled=true, transport=http, port=9204, path=/mcp, namespace=xpbd, timeoutMs=110000

### gateway_routes_diff (svc_add 前・dry-run)
- 既存 routing: 65 件 / services.json から導出: 63 件 / マージ後: 65 件
- 変更: `[保護] auth.unlaxer.org.public`（既存の保護設定、自分の変更ではない）
- 温存（2件）: `adoyose-admin`, `mahjong-mcp`（services.json に対応が無い手動設定の温存）
- xpbd.unlaxer.org は services.json 未更新のため未出現
- → svc_add(confirm=true) 後に再度 gateway_routes_diff を確認し、自分の 1 件のみ追加されることを確認する

## 協調 issue（issue-hub）

| issue | 相手 | 方向 | 状態 |
|-------|------|------|------|
| #241 | motion-engine | depends_on (ポーズ形式) | コメント追記済み（暫定仕様で進行） |
| #242 | keiko-engine | provides_to (制御推定結果 schema) | コメント追記済み（暫定仕様で進行） |
| #243 | sotai-engine | depends_on (ROM データ) | sotai から回答あり（暫定合意成立） |

## 停止条件のチェック

- [x] テストが通る: 14/14 passed (mcp:test), 48/48 passed (test)
- [x] ポート 9204 は割当表どおり（machine_ports で空き確認済み）
- [x] svc_add dry-run は新規（既存サービス上書きなし）
- [ ] gateway_routes_diff が自分の 1 件のみ（svc_add 後に確認）
- [ ] healthz が 200（デプロイ後に確認）

## 未決事項

- motion-engine のポーズ形式（{bone:[x,y,z]} Euler XYZ）の最終確認（#241）
- keiko-engine が制御推定結果 schema を受理可能か（#242）
- 物理パラメータの較正（market research で未較正が課題）
