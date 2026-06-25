// xpbd M1 tests — proves the active ragdoll: real gravity/mass, tracks a target
// pose (the "muscle"), stays connected, reacts to a push, and is deterministic.
//   node test.mjs
import { World, makeArm, q4, v3 } from './index.js';

let pass = 0, fail = 0;
const ok = (c, m) => { if (c) pass++; else { fail++; console.error('  ✗ ' + m); } };

const upperDirY = (arm) => q4.rot(arm.upper.q, [0, -1, 0])[1];   // -1 = straight down, →0 = lifted to horizontal
const LIFT = q4.axisAngle([0, 0, 1], 1.3);                       // raise the upper arm toward horizontal
const targetDirY = q4.rot(LIFT, [0, -1, 0])[1];

function simulate({ compliance = 0.00003, mass = 1.2, target = LIFT, frames = 200, kickAt = -1 } = {}) {
  const world = new World();
  const arm = makeArm(world, { mass, compliance });
  const dt = 1 / 60; const trace = [];
  let maxGap = 0, finite = true;
  for (let i = 0; i < frames; i++) {
    arm.setTarget(target, [0, 0, 0, 1]);
    if (i === kickAt) arm.lower.v = v3.add(arm.lower.v, [0, 0, 22]);   // shove the forearm
    world.step(dt);
    const elbowU = arm.upper.worldPoint([0, -arm.Lupper / 2, 0]);
    const elbowL = arm.lower.worldPoint([0, arm.Llower / 2, 0]);
    maxGap = Math.max(maxGap, v3.len(v3.sub(elbowU, elbowL)));
    const hp = arm.handPos();
    if (![...hp, ...arm.upper.q].every(Number.isFinite)) finite = false;
    trace.push(hp);
  }
  return { arm, trace, maxGap, finite, settledDirY: upperDirY(arm) };
}

// 1) stable + finite over a long run (no blow-up under stiff motor + gravity)
{
  const r = simulate({ frames: 600 });
  ok(r.finite, 'simulation stays finite over 10s (stable)');
}

// 2) the elbow joint stays ATTACHED (positional constraint holds)
{
  const r = simulate({ frames: 400 });
  ok(r.maxGap < 0.01, 'elbow joint stays connected (maxGap=' + r.maxGap.toFixed(5) + ' m)');
}

// 3) a STRONG muscle tracks the target (arm lifts to ~the target orientation)
{
  const r = simulate({ compliance: 0.00003 });
  ok(Math.abs(r.settledDirY - targetDirY) < 0.15, 'strong muscle tracks the lifted target (dirY=' + r.settledDirY.toFixed(3) + ' vs ' + targetDirY.toFixed(3) + ')');
}

// 4) a WEAK muscle SAGS under gravity (mass is real — it can't hold the pose)
{
  const strong = simulate({ compliance: 0.00003 }).settledDirY;
  const weak = simulate({ compliance: 0.05 }).settledDirY;
  ok(weak < strong - 0.1, 'weak muscle sags below the strong one (weak=' + weak.toFixed(3) + ' < strong=' + strong.toFixed(3) + ')');
}

// 5) WEIGHT: a heavier arm sags more than a lighter one (same muscle)
{
  const light = simulate({ compliance: 0.015, mass: 0.5 }).settledDirY;
  const heavy = simulate({ compliance: 0.015, mass: 4.0 }).settledDirY;
  ok(heavy < light - 0.03, 'heavier arm sags more (heavy=' + heavy.toFixed(3) + ' < light=' + light.toFixed(3) + ')');
}

// 6) REACTION: a shove perturbs the hand, then the muscle pulls it back (recovers)
{
  const r = simulate({ compliance: 0.0012, kickAt: 120, frames: 360 });
  const base = r.trace[118];                    // just before the kick
  let peak = 0; for (let i = 121; i < 175; i++) peak = Math.max(peak, v3.len(v3.sub(r.trace[i], base)));
  const back = v3.len(v3.sub(r.trace[359], base));   // long after
  ok(peak > 0.03, 'a shove visibly perturbs the arm (peak=' + peak.toFixed(3) + ')');
  ok(back < peak * 0.5, 'the muscle pulls it back toward the pose (residual=' + back.toFixed(3) + ')');
}

// 7) deterministic (fixed step, no Math.random)
{
  const a = simulate({ kickAt: 50 }).trace, b = simulate({ kickAt: 50 }).trace;
  let same = true;
  for (let i = 0; i < a.length; i++) for (let k = 0; k < 3; k++) if (a[i][k] !== b[i][k]) same = false;
  ok(same, 'simulation is deterministic across runs');
}

console.log(`xpbd M1: ${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
