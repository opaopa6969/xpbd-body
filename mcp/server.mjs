#!/usr/bin/env node
// xpbd-body MCP server — Streamable HTTP /mcp + /healthz
// Tools: simulate, estimate_control_start/status/result, check_feasible
// Resources: xpbd://spec, xpbd://guide, xpbd://inverse-dynamics, xpbd://theory-to-model
// Spec: docs/mcp/DESIGN.md

import http from 'node:http';
import { randomUUID } from 'node:crypto';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { McpServer } from '@modelcontextprotocol/sdk/server/mcp.js';
import { StreamableHTTPServerTransport } from '@modelcontextprotocol/sdk/server/streamableHttp.js';
import { z } from 'zod';

import { makeRig, simulate, estimateControl, checkFeasible, SCHEMA } from '../inverse.js';

const __dirname = dirname(fileURLToPath(import.meta.url));
const PKG = JSON.parse(readFileSync(join(__dirname, '..', 'package.json'), 'utf8'));
const VERSION = PKG.version;
const NAMESPACE = 'xpbd';

function log(...a) { process.stderr.write('[xpbd-mcp] ' + a.map((x) => typeof x === 'string' ? x : JSON.stringify(x)).join(' ') + '\n'); }

// ── zod schemas ──────────────────────────────────────────────────────────

const rigOptsSchema = z.object({
  skeleton: z.array(z.object({
    name: z.string(),
    parent: z.string().nullable(),
    off: z.array(z.number()).length(3),
    fixed: z.boolean().optional(),
  })).optional(),
  mass: z.number().positive().optional(),
  compliance: z.number().positive().optional(),
  profile: z.object({
    mass: z.number().positive().optional(),
    bulk: z.number().min(0).optional(),
    selfCollision: z.boolean().optional(),
  }).optional(),
  gravity: z.array(z.number()).length(3).optional(),
  linDamp: z.number().optional(),
  angDamp: z.number().optional(),
  dt: z.number().positive().optional(),
  substeps: z.number().int().positive().optional(),
  rom: z.record(z.string(), z.array(z.array(z.number()).length(2)).length(3)).optional(),
  maxTorque: z.record(z.string(), z.number()).optional(),
  seed: z.number().int().optional(),
  bones: z.array(z.string()).optional(),
}).optional();

const trajectorySchema = z.object({
  schemaVersion: z.string().optional(),
  dt: z.number().positive().optional(),
  steps: z.number().int().positive().optional(),
  bones: z.array(z.string()).optional(),
  frames: z.array(z.object({
    t: z.number().optional(),
    rel: z.record(z.string(), z.array(z.number()).length(4)).optional(),
    pos: z.record(z.string(), z.array(z.number()).length(3)).optional(),
    torque: z.record(z.string(), z.number()).optional(),
  })),
});

const controlTrajectorySchema = z.array(z.object({
  pose: z.record(z.string(), z.array(z.number()).length(3)).optional(),
  compliance: z.record(z.string(), z.number()).optional(),
}));

// ── job management ───────────────────────────────────────────────────────

const jobs = new Map();

function estimateControlAsync(rig, observed, opts) {
  const jobId = randomUUID();
  const job = {
    id: jobId,
    status: 'running',
    evals: 0,
    best_residual: null,
    elapsed_ms: 0,
    result: null,
    error: null,
    startedAt: Date.now(),
  };
  jobs.set(jobId, job);
  setImmediate(() => {
    const t0 = Date.now();
    try {
      const result = estimateControl(rig, observed, opts);
      job.result = result;
      job.status = 'done';
      job.evals = result.evals;
      job.best_residual = result.residual.rmsAngle;
      job.elapsed_ms = Date.now() - t0;
    } catch (e) {
      job.error = String(e?.message || e);
      job.status = 'error';
      job.elapsed_ms = Date.now() - t0;
    }
  });
  return jobId;
}

// ── server factory ───────────────────────────────────────────────────────

function createServer() {
  const server = new McpServer({ name: 'xpbd-body', version: VERSION });

  // ── tool: simulate ───────────────────────────────────────────────────
  server.tool(
    'simulate',
    '目標ポーズを物理追従させる（前向きシミュレーション）。副作用なし・決定論的。入力: rig_opts + control_trajectory → 出力: pose trajectory (frames with rel/pos/torque per bone)',
    {
      rig_opts: rigOptsSchema,
      control_trajectory: controlTrajectorySchema,
      dt: z.number().positive().optional(),
      steps: z.number().int().positive().optional(),
      substeps: z.number().int().positive().optional(),
    },
    async (args) => {
      const rig = makeRig(args.rig_opts || {});
      const dt = args.dt || rig.dt;
      const steps = args.steps || (args.control_trajectory ? args.control_trajectory.length : 0);
      const substeps = args.substeps != null ? args.substeps : rig.substeps;
      const result = simulate(rig, args.control_trajectory, dt, steps, { substeps });
      return { content: [{ type: 'text', text: JSON.stringify(result) }] };
    },
  );

  // ── tool: estimate_control_start ───────────────────────────────────
  server.tool(
    'estimate_control_start',
    '観測軌道から制御を逆推定する（analysis by synthesis）を job として開始。job_id を返す。30秒超の可能性があるため job 型。estimate_control_status で進捗確認、estimate_control_result で結果取得。',
    {
      rig_opts: rigOptsSchema,
      observed_trajectory: trajectorySchema,
      opts: z.object({
        bones: z.array(z.string()).optional(),
        knots: z.number().int().positive().optional(),
        method: z.enum(['coord', 'cem']).optional(),
        iters: z.number().int().positive().optional(),
        seed: z.number().int().optional(),
        smooth: z.number().optional(),
        wAngle: z.number().optional(),
        wPos: z.number().optional(),
        weights: z.record(z.string(), z.number()).optional(),
        step0: z.number().optional(),
        pop: z.number().int().positive().optional(),
        sigma0: z.number().optional(),
      }).optional(),
    },
    async (args) => {
      const rig = makeRig(args.rig_opts || {});
      const observed = args.observed_trajectory;
      const opts = args.opts || {};
      const jobId = estimateControlAsync(rig, observed, opts);
      return { content: [{ type: 'text', text: JSON.stringify({ job_id: jobId, status: 'queued' }) }] };
    },
  );

  // ── tool: estimate_control_status ──────────────────────────────────
  server.tool(
    'estimate_control_status',
    'estimate_control ジョブの進捗を取得。status: running/done/error。evals: 評価回数。best_residual: 最良残差(rad rms)。',
    {
      job_id: z.string().uuid(),
    },
    async (args) => {
      const job = jobs.get(args.job_id);
      if (!job) return { content: [{ type: 'text', text: JSON.stringify({ error: 'job not found' }) }] };
      return { content: [{ type: 'text', text: JSON.stringify({
        job_id: job.id, status: job.status, evals: job.evals, best_residual: job.best_residual, elapsed_ms: job.elapsed_ms,
      }) }] };
    },
  );

  // ── tool: estimate_control_result ──────────────────────────────────
  server.tool(
    'estimate_control_result',
    'estimate_control ジョブの最終結果を取得。status が done の場合は control/residual/feasible/violations を返す。running 中は status:running を返す。',
    {
      job_id: z.string().uuid(),
    },
    async (args) => {
      const job = jobs.get(args.job_id);
      if (!job) return { content: [{ type: 'text', text: JSON.stringify({ error: 'job not found' }) }] };
      if (job.status === 'running') return { content: [{ type: 'text', text: JSON.stringify({ job_id: job.id, status: 'running' }) }] };
      if (job.status === 'error') return { content: [{ type: 'text', text: JSON.stringify({ job_id: job.id, status: 'error', error: job.error }) }] };
      return { content: [{ type: 'text', text: JSON.stringify(job.result) }] };
    },
  );

  // ── tool: check_feasible ────────────────────────────────────────────
  server.tool(
    'check_feasible',
    '観測軌道の物理可能性を判定。ROM（関節可動域）/ トルク / 残差の3観点からチェック。feasible:false の場合 violations で原因を特定。',
    {
      rig_opts: rigOptsSchema,
      observed_trajectory: trajectorySchema,
      simulated: z.object({
        schemaVersion: z.string().optional(),
        dt: z.number().positive().optional(),
        steps: z.number().int().positive().optional(),
        bones: z.array(z.string()).optional(),
        frames: z.array(z.object({
          t: z.number().optional(),
          rel: z.record(z.string(), z.array(z.number()).length(4)).optional(),
          pos: z.record(z.string(), z.array(z.number()).length(3)).optional(),
          torque: z.record(z.string(), z.number()).optional(),
        })),
      }).optional(),
      opts: z.object({
        tol: z.number().optional(),
        residualTol: z.number().optional(),
        rom: z.record(z.string(), z.array(z.array(z.number()).length(2)).length(3)).optional(),
        maxTorque: z.record(z.string(), z.number()).optional(),
        residual: z.number().optional(),
      }).optional(),
    },
    async (args) => {
      const rig = makeRig(args.rig_opts || {});
      const observed = args.observed_trajectory;
      const result = checkFeasible(rig, observed, args.simulated || null, args.opts || {});
      return { content: [{ type: 'text', text: JSON.stringify(result) }] };
    },
  );

  // ── resources ───────────────────────────────────────────────────────
  server.resource('spec', 'xpbd://spec', { mimeType: 'application/json', description: 'xpbd-body capability spec' }, async () => {
    const spec = buildSpec();
    return { contents: [{ uri: 'xpbd://spec', mimeType: 'application/json', text: JSON.stringify(spec, null, 2) }] };
  });

  server.resource('guide', 'xpbd://guide', { mimeType: 'text/markdown', description: 'xpbd-body usage guide' }, async () => {
    const guide = buildGuide();
    return { contents: [{ uri: 'xpbd://guide', mimeType: 'text/markdown', text: guide }] };
  });

  server.resource('inverse-dynamics', 'xpbd://inverse-dynamics', { mimeType: 'text/markdown', description: 'inverse dynamics theory and limits' }, async () => {
    const text = readDoc('inverse-dynamics.md');
    return { contents: [{ uri: 'xpbd://inverse-dynamics', mimeType: 'text/markdown', text }] };
  });

  server.resource('theory-to-model', 'xpbd://theory-to-model', { mimeType: 'text/markdown', description: 'XPBD theory to implementation mapping' }, async () => {
    const text = readDoc('from-theory-to-model.md');
    return { contents: [{ uri: 'xpbd://theory-to-model', mimeType: 'text/markdown', text }] };
  });

  server.resource('xpbd-body-mcp-usage', 'skill://xpbd-body-mcp-usage', { mimeType: 'text/markdown', description: 'xpbd-body MCP usage skill (SKILL.md)' }, async () => {
    const text = readDoc('skills/xpbd-body-mcp-usage/SKILL.md');
    return { contents: [{ uri: 'skill://xpbd-body-mcp-usage', mimeType: 'text/markdown', text }] };
  });

  return server;
}

// ── spec builder ─────────────────────────────────────────────────────────

function buildSpec() {
  return {
    namespace: NAMESPACE,
    name: 'xpbd-body MCP server',
    version: VERSION,
    summary: 'VRM 上半身向け XPBD 能動ラグドール物理エンジン。ポーズ→物理追従、逆動力学、物理可能性判定。',
    capabilities: [
      { kind: 'tool', name: 'simulate', summary: '目標ポーズを物理追従させる（前向きシミュレーション）', input: '{ rig_opts, control_trajectory, dt?, steps?, substeps? }', output: '{ schemaVersion, dt, steps, bones, frames:[{ t, rel, pos, torque }] }', side_effect: 'read', long_running: false, dry_run: false, min_role: 'MEMBER' },
      { kind: 'tool', name: 'estimate_control_start', summary: '観測軌道から制御を逆推定（job 開始）', input: '{ rig_opts, observed_trajectory, opts }', output: '{ job_id, status }', side_effect: 'read', long_running: true, dry_run: false, min_role: 'MEMBER' },
      { kind: 'tool', name: 'estimate_control_status', summary: '推定 job の進捗取得', input: '{ job_id }', output: '{ job_id, status, evals, best_residual }', side_effect: 'read', long_running: false, dry_run: false, min_role: 'MEMBER' },
      { kind: 'tool', name: 'estimate_control_result', summary: '推定 job の最終結果取得', input: '{ job_id }', output: '{ control, residual, feasible, violations }', side_effect: 'read', long_running: false, dry_run: false, min_role: 'MEMBER' },
      { kind: 'tool', name: 'check_feasible', summary: '物理可能性判定（ROM/torque/residual）', input: '{ rig_opts, observed_trajectory, simulated?, opts }', output: '{ feasible, violations }', side_effect: 'read', long_running: false, dry_run: false, min_role: 'MEMBER' },
    ],
    compositions: [
      { title: 'ポーズ→物理追従→描画', flow: ['motion__pose', 'xpbd__simulate', 'showcase__render'], note: 'motion-engine のポーズを xpbd-body で物理追従させ、レンダラーで可視化' },
      { title: '観測→制御推定→稽古ループ', flow: ['xpbd__estimate_control_start', 'xpbd__estimate_control_result', 'keiko__train'], note: '観測軌道から制御を逆推定し、keiko-engine の稽古ループで動きの記述子を得る' },
      { title: '物理可能性チェック→フィードバック', flow: ['xpbd__check_feasible', 'sotai__rom_lookup'], note: 'feasible:false なら violations で原因を特定し、ROM データと照合してリギング修正' },
    ],
    depends_on: [
      { namespace: 'motion', capability: 'motion__pose（目標ポーズ生成）' },
      { namespace: 'keiko', capability: 'keiko__train（制御推定結果の消費）' },
      { namespace: 'sotai', capability: 'sotai__rom_lookup（関節可動域データ）' },
    ],
    health: '/healthz',
    docs: ['xpbd://guide', 'xpbd://inverse-dynamics', 'xpbd://theory-to-model'],
  };
}

// ── guide builder ────────────────────────────────────────────────────────

function buildGuide() {
  return `# xpbd-body MCP Guide

## namespace
\`xpbd\` — VRM 上半身向け XPBD 能動ラグドール物理エンジン

## tools

### simulate
目標ポーズを物理追従させる（前向きシミュレーション）。
- 入力: \`{ rig_opts, control_trajectory, dt?, steps?, substeps? }\`
- 出力: \`{ schemaVersion, dt, steps, bones, frames:[{ t, rel, pos, torque }] }\`
- 副作用なし・決定論的。通常 30 秒以内。

### estimate_control_start / status / result
観測軌道から制御を逆推定する（analysis by synthesis）。job 型。
1. \`estimate_control_start\` → \`{ job_id }\`
2. \`estimate_control_status\` → \`{ status, evals, best_residual }\`
3. \`estimate_control_result\` → \`{ control, residual, feasible, violations, unique, method }\`

注意: 逆問題は ill-posed（\`unique: false\`）。推定結果は「正解」ではなく「1つの推定」。

### check_feasible
観測軌道の物理可能性を判定。ROM / トルク / 残差の 3 観点。
- \`feasible: false\` の場合 \`violations\` で原因（rom/torque/residual）を特定。

## 入力形式

### rig_opts
\`{ skeleton?, mass?, compliance?, profile?, gravity?, dt?, substeps?, rom?, maxTorque?, seed? }\`
省略時は \`makeRig\` のデフォルト（DEFAULT_ROM, DEFAULT_MAX_TORQUE）。

### control_trajectory
\`[{ pose: { bone: [x, y, z] }, compliance?: { bone: number } }]\`
ポーズは Euler XYZ（ラジアン）, child-in-parent フレーム。

### observed_trajectory
\`{ schemaVersion?, dt?, steps?, bones?, frames: [{ t?, rel: { bone: [x,y,z,w] }, pos?: { bone: [x,y,z] }, torque? }] }\`
\`simulate\` の出力をそのまま入力に渡せる。

## 組み合わせ例
1. \`motion__pose\` → \`xpbd__simulate\` → レンダラーで可視化
2. 動画解析 → \`xpbd__estimate_control_start\` → \`xpbd__estimate_control_result\` → \`keiko__train\`
3. \`xpbd__check_feasible\` → \`feasible:false\` なら violations で原因特定 → リギング修正

## 制限
- 物理パラメータ（mass/compliance/maxTorque）は engine units であり人体に較正されていない
- 逆問題は ill-posed。複数の制御が同じ動きを生む
- 依存ライブラリ（motion-engine, keiko-engine, sotai-engine）は未 MCP 化（暫定仕様で運用）
`;
}

// ── doc reader ────────────────────────────────────────────────────────────

function readDoc(name) {
  try {
    return readFileSync(join(__dirname, '..', 'docs', name), 'utf8');
  } catch {
    return `# ${name}\n\nDocument not found.`;
  }
}

// ── HTTP server ──────────────────────────────────────────────────────────

function serveHttp(port) {
  const transports = new Map();
  const httpServer = http.createServer(async (req, res) => {
    res.setHeader('content-encoding', 'identity');
    const url = new URL(req.url, `http://${req.headers.host || 'localhost'}`);
    try {
      if (url.pathname === '/healthz') {
        res.writeHead(200, { 'content-type': 'application/json' });
        return res.end(JSON.stringify({ ok: true, name: 'xpbd-body', version: VERSION }));
      }
      if (url.pathname !== '/mcp') {
        res.writeHead(404, { 'content-type': 'application/json' });
        return res.end(JSON.stringify({ error: 'not found' }));
      }
      const sid = req.headers['mcp-session-id'];
      if (sid && transports.has(sid)) {
        return await transports.get(sid).handleRequest(req, res);
      }
      if (req.method === 'POST' && !sid) {
        const transport = new StreamableHTTPServerTransport({
          sessionIdGenerator: () => randomUUID(),
          enableJsonResponse: true,
          onsessioninitialized: (id) => { transports.set(id, transport); log('session open', { sid: id }); },
          onsessionclosed: (id) => { transports.delete(id); log('session closed', { sid: id }); },
        });
        const server = createServer();
        transport.onclose = () => {
          if (transport.sessionId) transports.delete(transport.sessionId);
          server.close().catch(() => {});
        };
        await server.connect(transport);
        return await transport.handleRequest(req, res);
      }
      res.writeHead(sid ? 404 : 400, { 'content-type': 'application/json' });
      return res.end(JSON.stringify({ error: sid ? 'unknown session' : 'missing mcp-session-id' }));
    } catch (e) {
      log('request failed', { path: url.pathname, error: String(e?.stack || e) });
      if (!res.headersSent) { res.writeHead(500); res.end(JSON.stringify({ error: 'internal error' })); }
      else res.end();
    }
  });
  httpServer.listen(port, '0.0.0.0', () => log('http listening', { url: `http://0.0.0.0:${port}/mcp` }));
  return httpServer;
}

// ── entry ─────────────────────────────────────────────────────────────────

const argv = process.argv.slice(2);
const port = Number(process.env.PORT || argv[0] || 9204);
serveHttp(port);
log('started', { version: VERSION, port });
