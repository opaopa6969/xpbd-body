// e2e test for xpbd-body MCP server
// Starts the server, runs healthz + tools/list + each tool via MCP client.

import { spawn } from 'node:child_process';
import { Client } from '@modelcontextprotocol/sdk/client/index.js';
import { StreamableHTTPClientTransport } from '@modelcontextprotocol/sdk/client/streamableHttp.js';

const PORT = 9304; // test port (avoid colliding with prod 9204)
const SERVER_URL = `http://127.0.0.1:${PORT}/mcp`;

let pass = 0, fail = 0;
function ok(name) { pass++; console.log(`  ✓ ${name}`); }
function ng(name, e) { fail++; console.error(`  ✗ ${name}: ${String(e?.message || e)}`); }

function assert(cond, msg) { if (!cond) throw new Error(msg || 'assertion failed'); }

const server = spawn('node', ['mcp/server.mjs', String(PORT)], {
  env: { ...process.env, PORT: String(PORT) },
  stdio: ['pipe', 'pipe', 'inherit'],
});
server.unref();

async function waitForHealth(url, timeoutMs = 5000) {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) {
    try {
      const res = await fetch(url);
      if (res.ok) return;
    } catch {}
    await new Promise((r) => setTimeout(r, 200));
  }
  throw new Error(`healthz not ready after ${timeoutMs}ms`);
}

async function main() {
  await waitForHealth(`http://127.0.0.1:${PORT}/healthz`);
  ok('healthz 200');

  // healthz body
  {
    const res = await fetch(`http://127.0.0.1:${PORT}/healthz`);
    const body = await res.json();
    assert(body.ok === true, `expected ok:true, got ${JSON.stringify(body)}`);
    assert(body.name === 'xpbd-body', `expected name:xpbd-body, got ${body.name}`);
    assert(body.version === '0.4.0', `expected version:0.4.0, got ${body.version}`);
    ok('healthz body { ok, name, version }');
  }

  // MCP client
  const transport = new StreamableHTTPClientTransport(new URL(SERVER_URL));
  const client = new Client({ name: 'test-client', version: '0.1.0' });
  await client.connect(transport);
  ok('MCP client connected');

  // tools/list
  {
    const { tools } = await client.listTools();
    const names = tools.map((t) => t.name).sort();
    const expected = ['check_feasible', 'estimate_control_result', 'estimate_control_start', 'estimate_control_status', 'simulate'];
    assert(JSON.stringify(names) === JSON.stringify(expected), `tools mismatch: ${JSON.stringify(names)}`);
    ok(`tools/list: ${tools.length} tools`);
  }

  // simulate — use real UPPER_BODY bone names (spine, chest)
  {
    const rig_opts = { mass: 1.0, compliance: 0.0006, dt: 1 / 60, substeps: 8, bones: ['spine', 'chest'] };
    const control_trajectory = [
      { pose: { spine: [0.1, 0, 0], chest: [0.1, 0, 0] } },
      { pose: { spine: [0.2, 0, 0], chest: [0.2, 0, 0] } },
    ];
    const result = await client.callTool({ name: 'simulate', arguments: { rig_opts, control_trajectory, steps: 2 } });
    const parsed = JSON.parse(result.content[0].text);
    assert(parsed.schemaVersion === 'xpbd-body/pose-trajectory@1', `bad schemaVersion: ${parsed.schemaVersion}`);
    assert(parsed.frames.length === 2, `expected 2 frames, got ${parsed.frames.length}`);
    assert(parsed.frames[0].rel && parsed.frames[0].pos, 'frame missing rel/pos');
    ok(`simulate: ${parsed.frames.length} frames, schema ${parsed.schemaVersion}`);
  }

  // estimate_control job lifecycle
  let jobId;
  {
    const rig_opts = { mass: 1.0, compliance: 0.0006, dt: 1 / 60, substeps: 8, bones: ['spine', 'chest'] };
    const control = [{ pose: { spine: [0.1, 0, 0], chest: [0.1, 0, 0] } }];
    // generate observed trajectory via simulate first
    const simResult = await client.callTool({ name: 'simulate', arguments: { rig_opts, control_trajectory: control, steps: 1 } });
    const observed = JSON.parse(simResult.content[0].text);

    const result = await client.callTool({
      name: 'estimate_control_start',
      arguments: { rig_opts, observed_trajectory: observed, opts: { iters: 2 } },
    });
    const parsed = JSON.parse(result.content[0].text);
    assert(parsed.job_id, 'no job_id');
    assert(parsed.status === 'queued', `expected status:queued, got ${parsed.status}`);
    jobId = parsed.job_id;
    ok(`estimate_control_start: job ${jobId.slice(0, 8)}`);
  }

  // poll status
  {
    let done = false;
    for (let i = 0; i < 30; i++) {
      const result = await client.callTool({ name: 'estimate_control_status', arguments: { job_id: jobId } });
      const parsed = JSON.parse(result.content[0].text);
      assert(parsed.status, 'no status');
      if (parsed.status === 'done' || parsed.status === 'error') {
        done = true;
        assert(parsed.status === 'done', `job failed: ${parsed.error || ''}`);
        assert(parsed.evals > 0, `no evals: ${parsed.evals}`);
        ok(`estimate_control_status: ${parsed.status}, ${parsed.evals} evals, residual ${parsed.best_residual?.toFixed(6)}`);
        break;
      }
      await new Promise((r) => setTimeout(r, 500));
    }
    assert(done, 'job did not complete');
  }

  // result
  {
    const result = await client.callTool({ name: 'estimate_control_result', arguments: { job_id: jobId } });
    const parsed = JSON.parse(result.content[0].text);
    assert(parsed.control, 'no control in result');
    assert(parsed.control.schemaVersion === 'xpbd-body/control@1', `bad control schema: ${parsed.control.schemaVersion}`);
    assert(parsed.residual, 'no residual');
    assert(typeof parsed.feasible === 'boolean', `bad feasible: ${parsed.feasible}`);
    assert(parsed.unique === false, `unique should be false`);
    assert(parsed.method === 'coord', `bad method: ${parsed.method}`);
    ok(`estimate_control_result: method=${parsed.method}, feasible=${parsed.feasible}, evals=${parsed.evals}, residual=${parsed.residual.rmsAngle.toFixed(6)}`);
  }

  // check_feasible
  {
    const rig_opts = { mass: 1.0, compliance: 0.0006, dt: 1 / 60, substeps: 8, bones: ['spine', 'chest'] };
    const control = [{ pose: { spine: [0.1, 0, 0], chest: [0.1, 0, 0] } }];
    const simResult = await client.callTool({ name: 'simulate', arguments: { rig_opts, control_trajectory: control, steps: 1 } });
    const observed = JSON.parse(simResult.content[0].text);

    const result = await client.callTool({ name: 'check_feasible', arguments: { rig_opts, observed_trajectory: observed } });
    const parsed = JSON.parse(result.content[0].text);
    assert(typeof parsed.feasible === 'boolean', `bad feasible: ${parsed.feasible}`);
    assert(Array.isArray(parsed.violations), 'violations not array');
    ok(`check_feasible: feasible=${parsed.feasible}, ${parsed.violations.length} violations`);
  }

  // resources: spec
  {
    const result = await client.readResource({ uri: 'xpbd://spec' });
    const text = result.contents[0].text;
    const spec = JSON.parse(text);
    assert(spec.namespace === 'xpbd', `bad namespace: ${spec.namespace}`);
    assert(spec.capabilities.length === 5, `expected 5 capabilities, got ${spec.capabilities.length}`);
    assert(spec.compositions.length === 3, `expected 3 compositions, got ${spec.compositions.length}`);
    assert(spec.depends_on.length === 3, `expected 3 depends_on, got ${spec.depends_on.length}`);
    ok(`resource xpbd://spec: ${spec.namespace}, ${spec.capabilities.length} capabilities`);
  }

  // resources: guide
  {
    const result = await client.readResource({ uri: 'xpbd://guide' });
    const text = result.contents[0].text;
    assert(text.includes('# xpbd-body MCP Guide'), `guide missing title`);
    assert(text.includes('simulate'), `guide missing simulate`);
    ok(`resource xpbd://guide: ${text.length} chars`);
  }

  // resources: inverse-dynamics
  {
    const result = await client.readResource({ uri: 'xpbd://inverse-dynamics' });
    const text = result.contents[0].text;
    assert(text.includes('Inverse dynamics'), `inverse-dynamics missing title`);
    ok(`resource xpbd://inverse-dynamics: ${text.length} chars`);
  }

  // resources: theory-to-model
  {
    const result = await client.readResource({ uri: 'xpbd://theory-to-model' });
    const text = result.contents[0].text;
    assert(text.includes('From theory to'), `theory-to-model missing title`);
    ok(`resource xpbd://theory-to-model: ${text.length} chars`);
  }

  // resource: skill://xpbd-body-mcp-usage
  {
    const result = await client.readResource({ uri: 'skill://xpbd-body-mcp-usage' });
    const text = result.contents[0].text;
    assert(text.includes('xpbd-body-mcp-usage'), `skill resource missing name`);
    assert(text.includes('xpbd__simulate'), `skill resource missing tool ref`);
    ok(`resource skill://xpbd-body-mcp-usage: ${text.length} chars`);
  }

  await transport.close();
  console.log(`\n${pass} passed, ${fail} failed`);
  process.exit(fail === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(`FATAL: ${e?.stack || e}`);
  process.exit(1);
});
