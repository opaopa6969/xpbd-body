// xpbd-body/inverse — the ANALYSIS-BY-SYNTHESIS substrate (M4).
//
// The active ragdoll in index.js is already a FORWARD model: control (motor
// targets + compliance) → trajectory. What an estimation loop needs on top of it
// is (a) a way to rewind the world so it can try a different control from the
// SAME initial state, (b) a side-effect-free `simulate()` it can call hundreds of
// times, and (c) the inverse: `estimateControl()` — given an observed pose
// trajectory, search for the control that would have produced it.
//
//   simulate(rig, control, dt, steps)          → poseTrajectory      (#2)
//   estimateControl(rig, observed, opts)       → { control, residual, feasible }  (#3)
//
// Pure, dependency-free, deterministic: no Math.random anywhere (the CEM sampler
// takes its PRNG seed from the rig/opts), fixed substeps, plain-data in/out with
// a schemaVersion so keiko-engine can consume it.
//
// HONESTY NOTE (see docs/inverse-dynamics.md): the inverse problem is ILL-POSED.
// Many controls produce the same visible motion; contact forces are unobservable;
// a monocular observation has no depth. estimateControl does NOT return "the"
// control — it returns ONE control, the one picked by the residual + smoothing
// regulariser, together with its residual and a feasibility verdict.

import { World, makeUpperBody, qFromEulerXYZ, UPPER_BODY, v3, q4 } from './index.js';

export const SCHEMA = Object.freeze({
  snapshot: 'xpbd-body/snapshot@1',
  rig: 'xpbd-body/rig@1',
  pose: 'xpbd-body/pose-trajectory@1',
  control: 'xpbd-body/control@1',
  estimate: 'xpbd-body/control-estimate@1',
});

// ---------------------------------------------------------------- math helpers

// shortest angle between two unit quaternions (radians, always ≥0)
export const qAngle = (a, b) => {
  const d = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
  return 2 * Math.acos(Math.min(1, d));
};

// inverse of qFromEulerXYZ (three.js 'XYZ' convention) — needed to seed / read
// the control in the same {bone:[x,y,z]} space motion-engine poses use.
export function eulerXYZFromQ(q) {
  const [x, y, z, w] = q;
  const m11 = 1 - 2 * (y * y + z * z), m12 = 2 * (x * y - z * w), m13 = 2 * (x * z + y * w);
  const m22 = 1 - 2 * (x * x + z * z), m23 = 2 * (y * z - x * w);
  const m32 = 2 * (y * z + x * w), m33 = 1 - 2 * (x * x + y * y);
  const ey = Math.asin(Math.max(-1, Math.min(1, m13)));
  if (Math.abs(m13) < 0.9999999) return [Math.atan2(-m23, m33), ey, Math.atan2(-m12, m11)];
  return [Math.atan2(m32, m22), ey, 0];   // gimbal-locked: fold z into x
}

// seeded PRNG (mulberry32). The estimator NEVER calls Math.random — the seed comes
// from the rig or opts, so a run is byte-reproducible.
export function makePRNG(seed = 1) {
  let a = (seed >>> 0) || 1;
  const rnd = () => {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  rnd.normal = () => {                       // Box–Muller, deterministic
    const u = Math.max(1e-12, rnd()), v = rnd();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  };
  return rnd;
}

// ------------------------------------------------------- snapshot / restore (#2)
// The whole simulation state as plain data: body kinematics + the motor CONTROL
// state (rest target + compliance). Deep-copied both ways, so a snapshot is inert
// — hold it, run anything, restore, and you are bit-for-bit back where you were.

export function snapshot(world) {
  return {
    schemaVersion: SCHEMA.snapshot,
    gravity: world.gravity.slice(), linDamp: world.linDamp, angDamp: world.angDamp,
    bodies: world.bodies.map((b) => ({ p: b.p.slice(), q: b.q.slice(), v: b.v.slice(), w: b.w.slice(), pp: b.pp.slice(), pq: b.pq.slice() })),
    // constraint order is stable (insertion order), so index-addressing the motors is safe
    motors: world.constraints.map((c, i) => (c.motor ? { i, rest: c.rest.slice(), compliance: c.compliance } : null)).filter(Boolean),
  };
}

export function restore(world, snap) {
  world.gravity = snap.gravity.slice(); world.linDamp = snap.linDamp; world.angDamp = snap.angDamp;
  for (let i = 0; i < world.bodies.length; i++) {
    const b = world.bodies[i], s = snap.bodies[i];
    b.p = s.p.slice(); b.q = s.q.slice(); b.v = s.v.slice(); b.w = s.w.slice();
    b.pp = s.pp.slice(); b.pq = s.pq.slice();
  }
  for (const m of snap.motors) {
    const c = world.constraints[m.i];
    c.rest = m.rest.slice(); c.compliance = m.compliance;
  }
  return world;
}

// ---------------------------------------------------------------------- the rig
// A rig bundles everything the estimation loop needs to be able to answer
// "what would this control have looked like": the world, the driven bones, and the
// PRIORS that make the inverse problem tractable at all — joint range of motion
// and motor torque ceilings. (sotai-engine ROMs drop straight in here.)

const rom3 = (x, y, z) => [x, y, z];
// mirror every left* entry onto right*. A ROM (array of [lo,hi] per axis) mirrors by
// flipping the y/z ranges (the axes that swap sign across the sagittal plane); a
// scalar (a torque ceiling) just copies.
const SYM = (base) => {
  const out = {};
  for (const k in base) {
    out[k] = base[k];
    if (!k.startsWith('left')) continue;
    const r = 'right' + k.slice(4);
    out[r] = Array.isArray(base[k]) ? base[k].map((rng, ax) => (ax === 0 ? rng.slice() : [-rng[1], -rng[0]])) : base[k];
  }
  return out;
};
// Radians, child-in-parent Euler XYZ. Deliberately GENEROUS (a prior, not a corset):
// the point is to reject the physically absurd (an elbow bending backwards), not to
// police style. Elbows cannot hyperextend — that is what makes an impossible
// observation impossible.
export const DEFAULT_ROM = Object.freeze(SYM({
  spine: rom3([-0.5, 0.6], [-0.6, 0.6], [-0.4, 0.4]),
  chest: rom3([-0.5, 0.6], [-0.7, 0.7], [-0.4, 0.4]),
  neck: rom3([-0.7, 0.7], [-0.9, 0.9], [-0.5, 0.5]),
  head: rom3([-0.6, 0.6], [-0.8, 0.8], [-0.4, 0.4]),
  leftShoulder: rom3([-0.4, 0.4], [-0.5, 0.5], [-0.5, 0.5]),
  leftUpperArm: rom3([-2.0, 2.0], [-1.7, 1.7], [-2.4, 2.4]),
  leftLowerArm: rom3([-2.7, 0.4], [-2.7, 2.7], [-0.6, 0.6]),   // x ≫ 0 = elbow bending the wrong way
  leftHand: rom3([-1.2, 1.2], [-0.6, 0.6], [-1.0, 1.0]),
}));

// SUSTAINED motor-torque ceiling per bone, in ENGINE units (see torqueOf). These are
// PRIORS, not measurements of a real human, and they are deliberately PERMISSIVE:
// measured against this body, an ordinary reach or a held pose lands at ~10 (soft
// muscle) to ~600 (very stiff muscle), so these ceilings only trip on an observation
// that is asking for something absurd. A calibrated avatar overrides them per rig —
// tighten them and the same machinery models a WEAK body that cannot hold a pose.
export const DEFAULT_MAX_TORQUE = Object.freeze(SYM({
  spine: 1500, chest: 1500, neck: 1000, head: 800,
  leftShoulder: 1500, leftUpperArm: 2000, leftLowerArm: 1200, leftHand: 800,
}));

// The motor is a compliant torsional spring: compliance α = radians per unit torque, so
// the torque it is CURRENTLY exerting is its tracking error over its compliance.
// Substep-independent, which is what makes it usable as a feasibility statistic. In
// steady state it equals the torque gravity is demanding, which is the whole point.
export const torqueOf = (motor) => qAngle(motor.rest, q4.mul(q4.conj(motor.A.q), motor.B.q)) / Math.max(1e-9, motor.compliance);

// SUSTAINED torque = the median over the trajectory. Deliberately not the peak: a step
// change in the motor target spikes the torque for a few frames (that is the simulator's
// transient, not the body's effort), and a ceiling on the peak would just measure how
// abruptly the control was authored.
const medianTorque = (simulated) => {
  const per = {};
  for (const f of simulated.frames) for (const b in (f.torque || {})) (per[b] = per[b] || []).push(f.torque[b]);
  const out = {};
  for (const b in per) { const s = per[b].sort((x, y) => x - y); out[b] = s[Math.floor(s.length / 2)]; }
  return out;
};

/**
 * Build a rig: world + upper body + priors + a snapshot of the rest state.
 * `bones` = the bones the control actually drives (default: every jointed bone).
 */
export function makeRig({
  skeleton = UPPER_BODY, mass = 1.0, compliance = 0.0008, profile = null,
  gravity, linDamp, angDamp, dt = 1 / 60, substeps = 20,
  rom = DEFAULT_ROM, maxTorque = DEFAULT_MAX_TORQUE, seed = 1, bones = null,
} = {}) {
  const wopt = {};
  if (gravity) wopt.gravity = gravity;
  if (linDamp != null) wopt.linDamp = linDamp;
  if (angDamp != null) wopt.angDamp = angDamp;
  const world = new World(wopt);
  const body = makeUpperBody(world, { skeleton, mass, compliance, profile });
  const driven = bones || skeleton.filter((b) => b.parent).map((b) => b.name);
  const rig = {
    schemaVersion: SCHEMA.rig,
    world, body, skeleton, bones: driven, motors: body.motors,
    rom, maxTorque, compliance, dt, substeps, seed,
    snapshot: () => snapshot(world),
    restore: (s) => restore(world, s),
  };
  rig.initial = snapshot(world);          // the rest state every estimation trial starts from
  return rig;
}

// ------------------------------------------------------------- control encoding
// A control trajectory is plain data: one frame per step, each a motor target pose
// (+ optional per-bone compliance). Short trajectories HOLD their last frame, so a
// constant pose is just `[{ pose: {...} }]`.
const frameOf = (traj, i) => {
  if (!traj || traj.length === 0) return {};
  const f = traj[Math.min(i, traj.length - 1)];
  return f && f.pose ? f : { pose: f };     // accept a bare {bone:[x,y,z]} map too
};

// ------------------------------------------------------------ forward model (#2)
/**
 * Roll the rig forward under a control trajectory and return the pose trajectory.
 * SIDE-EFFECT FREE: snapshots the world on entry and restores it on exit, so the
 * caller's live simulation (and the next trial of an estimation loop) is untouched.
 * Deterministic: same start state + same control → byte-identical output.
 *
 * @returns { schemaVersion, dt, steps, bones, frames:[{ t, rel:{bone:quat}, pos:{bone:[x,y,z]}, torque:{bone:number} }] }
 */
export function simulate(rig, controlTrajectory, dt = rig.dt, steps = 0, opts = {}) {
  const { world, body } = rig;
  const substeps = opts.substeps != null ? opts.substeps : rig.substeps;
  const n = steps || (controlTrajectory ? controlTrajectory.length : 0);
  const from = opts.from || snapshot(world);
  const keep = snapshot(world);           // what we must hand back untouched
  restore(world, from);

  const bones = opts.bones || rig.bones;
  const frames = [];
  for (let i = 0; i < n; i++) {
    const f = frameOf(controlTrajectory, i);
    for (const name of rig.bones) {
      const m = rig.motors[name];
      if (!m) continue;
      const e = f.pose && f.pose[name];
      m.rest = e ? qFromEulerXYZ(e) : [0, 0, 0, 1];
      const c = f.compliance && f.compliance[name];
      if (c != null) m.compliance = c;
    }
    world.step(dt, substeps);
    const rel = {}, pos = {}, torque = {};
    for (const name of bones) {
      const m = rig.motors[name];
      if (!m) continue;
      rel[name] = q4.mul(q4.conj(m.A.q), m.B.q);         // the bone's local rotation (what a renderer applies)
      pos[name] = m.B.p.slice();
      torque[name] = torqueOf(m);
    }
    frames.push({ t: (i + 1) * dt, rel, pos, torque });
  }

  restore(world, keep);
  return { schemaVersion: SCHEMA.pose, dt, steps: n, bones: bones.slice(), frames };
}

// --------------------------------------------------------------- feasibility (#3)
/**
 * Is this observation physically possible for THIS body? Three prongs, each catching a
 * different way an observation can be a lie:
 *   ROM       the observed joint angles are outside the joint's range of motion. An
 *             elbow bending backwards is not a hard control problem, it is impossible.
 *   TORQUE    reproducing it needs a sustained muscle pull past the motor's ceiling.
 *   RESIDUAL  the body simply cannot get there — the best control the estimator found
 *             still misses by a mile. THIS is what "collapses under its own weight"
 *             looks like: a weak muscle cannot exert a big torque, so the torque prong
 *             stays quiet by construction; what betrays it is that the arm sags anyway.
 * Every violation carries the excess, so a caller can name the joint and the amount.
 */
export function checkFeasible(rig, observed, simulated = null, opts = {}) {
  const tol = opts.tol != null ? opts.tol : 0.05;                    // rad / relative slack
  const residualTol = opts.residualTol != null ? opts.residualTol : 0.25;   // rad, rms joint angle
  const violations = [];
  const rom = opts.rom || rig.rom, maxT = opts.maxTorque || rig.maxTorque;
  const AX = ['x', 'y', 'z'];

  for (let i = 0; i < observed.frames.length; i++) {
    const rel = observed.frames[i].rel || {};
    for (const name in rel) {
      const lim = rom && rom[name];
      if (!lim) continue;
      const e = eulerXYZFromQ(rel[name]);
      for (let a = 0; a < 3; a++) {
        const [lo, hi] = lim[a];
        const excess = e[a] < lo - tol ? e[a] - (lo - tol) : e[a] > hi + tol ? e[a] - (hi + tol) : 0;
        if (excess !== 0) violations.push({ type: 'rom', bone: name, axis: AX[a], frame: i, value: e[a], limit: [lo, hi], excess });
      }
    }
  }
  if (simulated) {
    const sus = medianTorque(simulated);
    for (const name in sus) {
      const lim = maxT && maxT[name];
      if (lim == null) continue;
      if (sus[name] > lim * (1 + tol)) violations.push({ type: 'torque', bone: name, value: sus[name], limit: lim, excess: sus[name] - lim });
    }
  }
  if (opts.residual != null && opts.residual > residualTol) {
    violations.push({ type: 'residual', value: opts.residual, limit: residualTol, excess: opts.residual - residualTol });
  }
  return { feasible: violations.length === 0, violations };
}

// ---------------------------------------------------------- objective + encoding
// The control is parameterised by KNOTS (a few Euler triples per bone, linearly
// interpolated over the frames) instead of one free target per frame. Two reasons:
// it collapses the search space from thousands of dims to dozens, and it is itself
// a smoothness prior — muscles do not teleport.
const expand = (bones, knots, K, steps) => {
  const traj = [];
  for (let i = 0; i < steps; i++) {
    const u = K === 1 ? 0 : (i * (K - 1)) / Math.max(1, steps - 1);
    const k0 = Math.min(K - 1, Math.floor(u)), k1 = Math.min(K - 1, k0 + 1), s = u - k0;
    const pose = {};
    for (let b = 0; b < bones.length; b++) {
      const a = knots[b][k0], c = knots[b][k1];
      pose[bones[b]] = [a[0] + (c[0] - a[0]) * s, a[1] + (c[1] - a[1]) * s, a[2] + (c[2] - a[2]) * s];
    }
    traj.push({ pose });
  }
  return traj;
};

const clampToRom = (rig, bone, e) => {
  const lim = rig.rom && rig.rom[bone];
  if (!lim) return e;
  return [0, 1, 2].map((a) => Math.max(lim[a][0], Math.min(lim[a][1], e[a])));
};

// residual: weighted sum of squared joint-angle error (+ optional position error),
// plus a smoothing regulariser on the control's curvature (2nd difference of the
// knots). The regulariser is what PICKS ONE control out of the many that fit.
function makeCost(rig, observed, bones, K, cfg) {
  const steps = observed.frames.length;
  return (knots) => {
    const traj = expand(bones, knots, K, steps);
    const sim = simulate(rig, traj, cfg.dt, steps, { substeps: cfg.substeps, from: cfg.from, bones });
    let ang = 0, posE = 0;
    for (let i = 0; i < steps; i++) {
      const o = observed.frames[i], s = sim.frames[i];
      for (const b of bones) {
        if (!o.rel || !o.rel[b]) continue;
        const th = qAngle(o.rel[b], s.rel[b]);
        ang += (cfg.weights[b] != null ? cfg.weights[b] : 1) * th * th;
        if (cfg.wPos > 0 && o.pos && o.pos[b]) {
          const d = v3.sub(o.pos[b], s.pos[b]);
          posE += v3.dot(d, d);
        }
      }
    }
    let smooth = 0;
    for (let b = 0; b < bones.length; b++) for (let k = 1; k < K - 1; k++) for (let a = 0; a < 3; a++) {
      const d2 = knots[b][k + 1][a] - 2 * knots[b][k][a] + knots[b][k - 1][a];
      smooth += d2 * d2;
    }
    const cost = cfg.wAngle * ang + cfg.wPos * posE + cfg.smooth * smooth;
    return { cost, ang, posE, smooth, sim };
  };
}

const cloneKnots = (k) => k.map((b) => b.map((e) => e.slice()));

// ---------------------------------------------------------- control estimation (#3)
/**
 * estimateControl(rig, observedPoseTrajectory, opts) → { control, residual, feasible }
 *
 * Analysis by synthesis: guess a control, run the FORWARD model, look at the gap,
 * fix the guess. Gradient-free (the simulator is a black box with contacts in it —
 * there is no clean derivative), seeded, deterministic.
 *
 * opts:
 *   bones      which bones the control drives (default rig.bones ∩ observed.bones)
 *   knots      control knots per bone (default ≈ steps/24, min 2, max 6)
 *   method     'coord' (coordinate descent, default) | 'cem' (cross-entropy method)
 *   iters      sweeps (coord) / generations (cem)
 *   seed       PRNG seed for 'cem' (default rig.seed) — Math.random is never used
 *   smooth     weight of the control-smoothness regulariser (default 0.02)
 *   wAngle/wPos/weights   objective weights
 *   step0      initial coordinate-descent step, rad (default 0.25)
 */
export function estimateControl(rig, observed, opts = {}) {
  const steps = observed.frames.length;
  if (!steps) throw new Error('estimateControl: empty observation');
  const obsBones = observed.bones || Object.keys(observed.frames[0].rel || {});
  const bones = (opts.bones || rig.bones).filter((b) => obsBones.includes(b) && rig.motors[b]);
  if (!bones.length) throw new Error('estimateControl: no observed bone is driven by this rig');

  const K = Math.max(1, opts.knots || Math.min(6, Math.max(2, Math.round(steps / 24))));
  const cfg = {
    dt: opts.dt || observed.dt || rig.dt,
    substeps: opts.substeps != null ? opts.substeps : rig.substeps,
    from: opts.from || rig.initial,
    wAngle: opts.wAngle != null ? opts.wAngle : 1,
    wPos: opts.wPos != null ? opts.wPos : 0,
    smooth: opts.smooth != null ? opts.smooth : 0.02,
    weights: opts.weights || {},
  };
  const cost = makeCost(rig, observed, bones, K, cfg);

  // seed the search from the OBSERVATION itself: the pose you see is already a good
  // first guess at the pose the muscle was aiming for (it differs by the sag).
  const knotFrame = (k) => (K === 1 ? 0 : Math.round((k * (steps - 1)) / (K - 1)));
  let best = bones.map((b) => {
    const row = [];
    for (let k = 0; k < K; k++) {
      const rel = observed.frames[knotFrame(k)].rel[b];
      row.push(clampToRom(rig, b, rel ? eulerXYZFromQ(rel) : [0, 0, 0]));
    }
    return row;
  });
  let bestR = cost(best);
  let evals = 1;

  const method = opts.method || 'coord';
  if (method === 'cem') {
    const rnd = makePRNG(opts.seed != null ? opts.seed : rig.seed);
    const gens = opts.iters != null ? opts.iters : 12;
    const pop = opts.pop || 24, elite = Math.max(2, Math.round(pop * 0.25));
    // sigma0 matters more than anything else here: the search STARTS from a good guess
    // (the observation itself), so a wide cloud just samples worse controls forever and
    // CEM never moves. Start narrow and let the elite variance widen it if it needs to.
    const s0 = opts.sigma0 != null ? opts.sigma0 : 0.01;
    let mean = cloneKnots(best);
    let sigma = bones.map(() => Array.from({ length: K }, () => [s0, s0, s0]));
    for (let g = 0; g < gens; g++) {
      const samples = [];
      for (let s = 0; s < pop; s++) {
        const cand = mean.map((row, b) => row.map((e, k) => clampToRom(rig, bones[b], e.map((v, a) => v + sigma[b][k][a] * rnd.normal()))));
        const r = cost(cand); evals++;
        samples.push({ cand, r });
        if (r.cost < bestR.cost) { best = cloneKnots(cand); bestR = r; }
      }
      samples.sort((x, y) => x.r.cost - y.r.cost);
      const top = samples.slice(0, elite);
      mean = bones.map((_, b) => Array.from({ length: K }, (_, k) => [0, 1, 2].map((a) => top.reduce((s, t) => s + t.cand[b][k][a], 0) / elite)));
      sigma = bones.map((_, b) => Array.from({ length: K }, (_, k) => [0, 1, 2].map((a) => {
        const m = mean[b][k][a];
        const varr = top.reduce((s, t) => s + (t.cand[b][k][a] - m) ** 2, 0) / elite;
        return Math.max(1e-3, Math.sqrt(varr));         // floor so it never fully collapses
      })));
    }
  } else {
    // coordinate descent / pattern search: one knot-axis at a time, ± a shrinking step.
    // Boring, gradient-free, and reliable on a black-box simulator.
    const sweeps = opts.iters != null ? opts.iters : 6;
    let step = opts.step0 != null ? opts.step0 : 0.25;
    for (let s = 0; s < sweeps; s++) {
      let improved = false;
      for (let b = 0; b < bones.length; b++) for (let k = 0; k < K; k++) for (let a = 0; a < 3; a++) {
        for (const sign of [1, -1]) {
          const cand = cloneKnots(best);
          const e = cand[b][k].slice();
          e[a] += sign * step;
          cand[b][k] = clampToRom(rig, bones[b], e);
          if (cand[b][k][a] === best[b][k][a]) continue;   // clamped flat: nothing to try
          const r = cost(cand); evals++;
          if (r.cost < bestR.cost - 1e-12) { best = cand; bestR = r; improved = true; break; }
        }
      }
      step *= improved ? 0.6 : 0.35;
      if (step < 1e-4) break;
    }
  }

  // residual, reported in the units a human can argue with: radians of joint angle.
  const sim = bestR.sim;
  const perBone = {}; let sumSq = 0, maxErr = 0, n = 0;
  for (const b of bones) { perBone[b] = { rms: 0, max: 0 }; }
  for (let i = 0; i < steps; i++) {
    for (const b of bones) {
      const o = observed.frames[i].rel[b];
      if (!o) continue;
      const th = qAngle(o, sim.frames[i].rel[b]);
      perBone[b].rms += th * th; perBone[b].max = Math.max(perBone[b].max, th);
      sumSq += th * th; maxErr = Math.max(maxErr, th); n++;
    }
  }
  for (const b of bones) perBone[b].rms = Math.sqrt(perBone[b].rms / Math.max(1, steps));
  const rmsAngle = Math.sqrt(sumSq / Math.max(1, n));

  const feas = checkFeasible(rig, observed, sim, Object.assign({}, opts, { residual: rmsAngle }));
  const frames = expand(bones, best, K, steps);

  return {
    schemaVersion: SCHEMA.estimate,
    control: {
      schemaVersion: SCHEMA.control,
      dt: cfg.dt, steps, bones: bones.slice(), knots: K,
      knotFrames: Array.from({ length: K }, (_, k) => knotFrame(k)),
      knotValues: bones.reduce((o, b, i) => (o[b] = best[i].map((e) => e.slice()), o), {}),
      frames,                                    // feed straight back into simulate()
    },
    residual: {
      rmsAngle,                                     // rad
      maxAngle: maxErr,
      perBone,
      cost: bestR.cost,
      smoothness: bestR.smooth,
    },
    feasible: feas.feasible,
    violations: feas.violations,
    evals,
    method,
    // the inverse problem is ill-posed: this is ONE control consistent with the
    // observation (the one the smoothing prior picked), not THE control.
    unique: false,
  };
}
