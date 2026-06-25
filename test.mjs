// xpbd M1 tests — proves the active ragdoll: real gravity/mass, tracks a target
// pose (the "muscle"), stays connected, reacts to a push, and is deterministic.
//   node test.mjs
import { World, Body, Contact, GroundContact, BoxContact, makeArm, makeUpperBody, qFromEulerXYZ, UPPER_BODY, q4, v3 } from './index.js';

let pass = 0, fail = 0;
const ok = (c, m) => { if (c) pass++; else { fail++; console.error('  ✗ ' + m); } };
// shortest angle between two unit quats (radians)
const qAngle = (a, b) => { const d = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]); return 2 * Math.acos(Math.min(1, d)); };

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

// ===== M2: full upper-body chain driven by a motion-engine pose =====
const offOf = {}; for (const b of UPPER_BODY) offOf[b.name] = b;
function simBody({ pose = {}, compliance = 0.00006, frames = 300, kick = null } = {}) {
  const world = new World();
  const ub = makeUpperBody(world, { compliance });
  const dt = 1 / 60; let maxGap = 0, finite = true; const trace = [];
  for (let i = 0; i < frames; i++) {
    ub.setPose(pose);
    if (kick && i === kick.at) ub.bodies[kick.bone].v = v3.add(ub.bodies[kick.bone].v, kick.v);
    world.step(dt);
    for (const b of UPPER_BODY) {
      if (!b.parent) continue;
      const jp = v3.add(ub.bodies[b.parent].p, q4.rot(ub.bodies[b.parent].q, b.off));
      maxGap = Math.max(maxGap, v3.len(v3.sub(jp, ub.bodies[b.name].p)));
    }
    if (!ub.bodies.head.p.every(Number.isFinite)) finite = false;
    trace.push({ head: ub.bodies.head.q.slice(), chest: ub.bodies.chest.q.slice(), rua: ub.bodies.rightUpperArm.q.slice(), rh: ub.bodies.rightHand.p.slice() });
  }
  return { ub, maxGap, finite, trace };
}
// relative orientation child-in-parent (the bone's local rotation)
const relOf = (ub, name) => q4.mul(q4.conj(ub.bodies[offOf[name].parent].q), ub.bodies[name].q);

// 8) the whole chain TRACKS a motion-engine pose (relative orientations match)
{
  const POSE = { rightUpperArm: [0, 0, -1.0], head: [0.2, 0.3, 0], chest: [0, 0.2, 0] };
  const { ub, finite } = simBody({ pose: POSE, compliance: 0.00004, frames: 320 });
  let maxErr = 0;
  for (const name in POSE) maxErr = Math.max(maxErr, qAngle(relOf(ub, name), qFromEulerXYZ(POSE[name])));
  ok(finite, 'upper-body chain stays finite');
  ok(maxErr < 0.2, 'chain tracks the motion-engine pose (maxErr=' + maxErr.toFixed(3) + ' rad)');
}

// 9) every joint in the chain stays connected
{
  const r = simBody({ pose: { rightUpperArm: [0, 0, -1.0] }, frames: 320 });
  ok(r.maxGap < 0.012, 'all upper-body joints stay connected (maxGap=' + r.maxGap.toFixed(5) + ' m)');
}

// 10) a shove on the head PROPAGATES down the chain, then recovers
{
  const r = simBody({ pose: {}, compliance: 0.003, frames: 360, kick: { at: 150, bone: 'head', v: [0, 0, 18] } });
  const chestBase = r.trace[148].chest;
  let peak = 0; for (let i = 151; i < 185; i++) peak = Math.max(peak, qAngle(chestBase, r.trace[i].chest));
  const headBase = r.trace[148].head, headEnd = r.trace[359].head;
  ok(peak > 0.02, 'the shove propagates down to the chest (peak Δ=' + peak.toFixed(3) + ')');
  ok(qAngle(headBase, headEnd) < 0.06, 'the chain recovers to the pose after the shove');
}

// 11) a WEAK muscle lets a raised arm sag under gravity (held pose vs strong)
{
  const POSE = { rightUpperArm: [0, 0, -1.3] };          // hold the right arm out/up
  const strong = simBody({ pose: POSE, compliance: 0.00006, frames: 280 }).trace[279].rh[1];
  const weak = simBody({ pose: POSE, compliance: 0.01, frames: 280 }).trace[279].rh[1];
  ok(Math.abs(strong - weak) > 0.05, 'muscle strength changes where the held arm ends up (strong y=' + strong.toFixed(3) + ', weak y=' + weak.toFixed(3) + ')');
}

// 12) the upper-body chain is deterministic
{
  const a = simBody({ pose: { head: [0.2, 0, 0] }, kick: { at: 60, bone: 'chest', v: [0, 0, 5] } }).trace.map((t) => t.rh);
  const b = simBody({ pose: { head: [0.2, 0, 0] }, kick: { at: 60, bone: 'chest', v: [0, 0, 5] } }).trace.map((t) => t.rh);
  let same = true; for (let i = 0; i < a.length; i++) for (let k = 0; k < 3; k++) if (a[i][k] !== b[i][k]) same = false;
  ok(same, 'upper-body chain is deterministic');
}

// ===== M3: contacts / self-collision =====

// 13) GROUND contact: a falling body settles on the plane and never tunnels through
{
  const world = new World();
  const b = world.add(new Body({ pos: [0, 1.0, 0], mass: 1, cr: 0.08 }));
  world.constrain(GroundContact(b, { y: 0 }));
  const dt = 1 / 60; let minY = Infinity, finite = true;
  for (let i = 0; i < 240; i++) { world.step(dt); minY = Math.min(minY, b.p[1]); if (!b.p.every(Number.isFinite)) finite = false; }
  ok(finite, 'ground contact stays finite');
  ok(minY > -0.01, 'body never tunnels below the plane (minY=' + minY.toFixed(4) + ')');
  ok(Math.abs(b.p[1] - 0.08) < 0.02, 'body rests on the surface at y≈cr (y=' + b.p[1].toFixed(4) + ')');
}

// 14) BOX contact: a body dropped onto a box top lands on it, not inside it
{
  const world = new World();
  const b = world.add(new Body({ pos: [0, 0.6, 0], mass: 1, cr: 0.04 }));
  world.constrain(BoxContact(b, [-0.2, -0.2, -0.2], [0.2, 0.2, 0.2]));
  const dt = 1 / 60;
  for (let i = 0; i < 300; i++) world.step(dt);
  // either it rests on the top face (y≈0.24) — it started centered above, so it should
  ok(b.p[1] > 0.2, 'body lands on/above the box, not inside it (y=' + b.p[1].toFixed(4) + ')');
}

// helper: drive the right hand toward the body's own spine and measure how close it gets
function handToSpineMin({ selfCollision, bulk = 0, frames = 320 }) {
  const world = new World();
  const ub = makeUpperBody(world, { compliance: 0.00004, profile: { selfCollision, bulk } });
  // a pose that swings the right arm hard ACROSS the chest toward the left/center
  const POSE = { rightUpperArm: [0, -0.2, 1.3], rightLowerArm: [0, -1.6, 0] };
  const dt = 1 / 60; let minDist = Infinity, maxGap = 0, finite = true;
  for (let i = 0; i < frames; i++) {
    ub.setPose(POSE);
    world.step(dt);
    const d = v3.len(v3.sub(ub.bodies.rightHand.p, ub.bodies.spine.p));
    if (i > frames * 0.6) minDist = Math.min(minDist, d);   // measure once settled
    for (const b of UPPER_BODY) {
      if (!b.parent) continue;
      const jp = v3.add(ub.bodies[b.parent].p, q4.rot(ub.bodies[b.parent].q, b.off));
      maxGap = Math.max(maxGap, v3.len(v3.sub(jp, ub.bodies[b.name].p)));
    }
    if (!ub.bodies.rightHand.p.every(Number.isFinite)) finite = false;
  }
  return { minDist, maxGap, finite, ub };
}

// 15) SELF-COLLISION: the hand penetrates the trunk without it, rides around WITH it
{
  const off = handToSpineMin({ selfCollision: false });
  const on = handToSpineMin({ selfCollision: true, bulk: 0.5 });
  ok(on.finite, 'self-collision body stays finite');
  const wanted = on.ub.bodies.rightHand.cr + on.ub.bodies.spine.cr;
  ok(on.minDist > off.minDist + 0.01, 'self-collision keeps the hand farther from the trunk (off=' + off.minDist.toFixed(3) + ' → on=' + on.minDist.toFixed(3) + ' m)');
  ok(on.minDist > wanted - 0.03, 'the hand rides at ~the collider sum, not through the trunk (minDist=' + on.minDist.toFixed(3) + ' vs want≈' + wanted.toFixed(3) + ')');
}

// 16) self-collision does NOT tear the chain apart (joints stay connected)
{
  const on = handToSpineMin({ selfCollision: true, bulk: 0.5 });
  ok(on.maxGap < 0.02, 'joints stay connected with self-collision on (maxGap=' + on.maxGap.toFixed(5) + ' m)');
}

// 17) contacts are deterministic
{
  const a = handToSpineMin({ selfCollision: true, bulk: 0.5 }).minDist;
  const b = handToSpineMin({ selfCollision: true, bulk: 0.5 }).minDist;
  ok(a === b, 'self-collision is deterministic across runs');
}

// 18) BodyProfile bulk default reproduces the M2 body (no behavior drift)
{
  // a default-profile body must move identically to one built with no profile at all
  const sim = (profile) => {
    const world = new World();
    const ub = makeUpperBody(world, { compliance: 0.00006, profile });
    const dt = 1 / 60; const tr = [];
    for (let i = 0; i < 120; i++) { ub.setPose({ rightUpperArm: [0, 0, -1.0] }); world.step(dt); tr.push(ub.bodies.rightHand.p.slice()); }
    return tr;
  };
  const base = sim(null), def = sim({});
  let same = true; for (let i = 0; i < base.length; i++) for (let k = 0; k < 3; k++) if (base[i][k] !== def[i][k]) same = false;
  ok(same, 'default profile reproduces the M2 body exactly');
}

console.log(`xpbd M1+M2+M3: ${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
