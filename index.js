// xpbd-body — a tiny from-scratch articulated rigid-body simulator (XPBD) for an
// ACTIVE RAGDOLL upper body. The L3 dynamics layer for motion-engine: it makes a
// body with real mass + gravity physically TRACK a target pose. Pure, dep-free,
// deterministic (fixed substeps, no Math.random) — same 流儀 as motion-engine.
//
// Why XPBD: position-based, so joint attachment, angular motors ("muscles"),
// joint limits and (later) contacts are all the SAME kind of constraint. Stable
// under stiff motors via substepping. This is the substrate L3 needs because
// contacts / self-collision (the "arm goes around the belly") drop straight in.
//
// M1 scope: rigid bodies + gravity + positional ATTACH joints + angular MOTOR
// constraints that pull each bone toward a target orientation (the L2 pose
// becomes the motor target → the body physically TRACKS it but has real mass,
// gravity, momentum and reacts to pushes). Isotropic inertia (sphere approx);
// real inertia tensors + contacts are M2/M3.

// ----- vec3 / quat (plain arrays) -----
export const v3 = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  len: (a) => Math.hypot(a[0], a[1], a[2]),
  norm: (a) => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; },
};
export const q4 = {
  mul: (a, b) => [
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
  ],
  conj: (q) => [-q[0], -q[1], -q[2], q[3]],
  norm: (q) => { const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return [q[0] / l, q[1] / l, q[2] / l, q[3] / l]; },
  rot: (q, p) => {
    const tx = 2 * (q[1] * p[2] - q[2] * p[1]);
    const ty = 2 * (q[2] * p[0] - q[0] * p[2]);
    const tz = 2 * (q[0] * p[1] - q[1] * p[0]);
    return [p[0] + q[3] * tx + (q[1] * tz - q[2] * ty), p[1] + q[3] * ty + (q[2] * tx - q[0] * tz), p[2] + q[3] * tz + (q[0] * ty - q[1] * tx)];
  },
  axisAngle: (ax, a) => { const n = v3.norm(ax); const h = a / 2; const s = Math.sin(h); return [n[0] * s, n[1] * s, n[2] * s, Math.cos(h)]; },
};

// ----- rigid body -----
export class Body {
  constructor({ pos = [0, 0, 0], q = [0, 0, 0, 1], mass = 1, radius = 0.06, fixed = false } = {}) {
    this.p = pos.slice(); this.q = q.slice();
    this.v = [0, 0, 0]; this.w = [0, 0, 0];
    this.invM = fixed ? 0 : 1 / mass;
    // isotropic inertia of a solid sphere I = 2/5 m r^2
    this.invI = fixed ? 0 : 1 / (0.4 * mass * radius * radius);
    this.pp = this.p.slice(); this.pq = this.q.slice();
  }
  worldPoint(r) { return v3.add(this.p, q4.rot(this.q, r)); }
  // apply an angular impulse vector (world) as a small quaternion correction
  applyDRot(aimp) {
    const dq = q4.mul([aimp[0], aimp[1], aimp[2], 0], this.q);
    this.q = q4.norm([this.q[0] + 0.5 * dq[0], this.q[1] + 0.5 * dq[1], this.q[2] + 0.5 * dq[2], this.q[3] + 0.5 * dq[3]]);
  }
}

// ----- constraints -----
// keep local point rA on body A coincident with local point rB on body B (a ball
// joint). compliance 0 = rigid. This is what keeps the elbow attached.
export function Attach(A, rA, B, rB, compliance = 0) {
  return {
    joint: true,                                  // re-iterated to converge the chain
    solve(h) {
      const rAw = q4.rot(A.q, rA), rBw = q4.rot(B.q, rB);
      const C = v3.sub(v3.add(B.p, rBw), v3.add(A.p, rAw));   // gap (want 0)
      const c = v3.len(C); if (c < 1e-12) return;
      const n = v3.scale(C, 1 / c);
      const cAn = v3.cross(rAw, n), cBn = v3.cross(rBw, n);
      const wA = A.invM + A.invI * v3.dot(cAn, cAn);
      const wB = B.invM + B.invI * v3.dot(cBn, cBn);
      const at = compliance / (h * h);
      const dl = -c / (wA + wB + at);
      const P = v3.scale(n, dl);
      A.p = v3.add(A.p, v3.scale(P, -A.invM));
      B.p = v3.add(B.p, v3.scale(P, B.invM));
      A.applyDRot(v3.scale(v3.cross(rAw, P), -A.invI));
      B.applyDRot(v3.scale(v3.cross(rBw, P), B.invI));
    },
  };
}

// drive B's orientation RELATIVE to A toward `rest` (a quaternion, child-in-parent
// frame). compliance is the muscle softness: small = stiff/strong, large = weak
// (sags under gravity). This is the PD "muscle" as a compliant XPBD constraint.
export function Motor(A, B, rest, compliance = 0.0005) {
  return {
    rest,
    solve(h) {
      const qRel = q4.mul(q4.conj(A.q), B.q);             // current child-in-parent
      let qErr = q4.mul(this.rest, q4.conj(qRel));        // rotation needed (parent frame)
      if (qErr[3] < 0) qErr = [-qErr[0], -qErr[1], -qErr[2], -qErr[3]];
      const theta = q4.rot(A.q, [2 * qErr[0], 2 * qErr[1], 2 * qErr[2]]);   // → world
      const ang = v3.len(theta); if (ang < 1e-9) return;
      const axis = v3.scale(theta, 1 / ang);
      const at = compliance / (h * h);
      const dl = ang / (A.invI + B.invI + at);
      const corr = v3.scale(axis, dl);
      A.applyDRot(v3.scale(corr, -A.invI));
      B.applyDRot(v3.scale(corr, B.invI));
    },
  };
}

// ----- world -----
export class World {
  constructor({ gravity = [0, -9.81, 0], linDamp = 0.999, angDamp = 0.985 } = {}) {
    this.gravity = gravity; this.linDamp = linDamp; this.angDamp = angDamp;
    this.bodies = []; this.constraints = [];
  }
  add(b) { this.bodies.push(b); return b; }
  constrain(c) { this.constraints.push(c); return c; }

  // fixed-timestep XPBD: substep for stability under stiff motors. Deterministic.
  step(dt, substeps = 20) {
    const h = dt / substeps;
    for (let s = 0; s < substeps; s++) {
      for (const b of this.bodies) {
        if (b.invM === 0) continue;
        b.v = v3.add(b.v, v3.scale(this.gravity, h));
        b.pp = b.p.slice(); b.p = v3.add(b.p, v3.scale(b.v, h));
        b.pq = b.q.slice();
        const w = b.w; const dq = q4.mul([w[0], w[1], w[2], 0], b.q);
        b.q = q4.norm([b.q[0] + 0.5 * h * dq[0], b.q[1] + 0.5 * h * dq[1], b.q[2] + 0.5 * h * dq[2], b.q[3] + 0.5 * h * dq[3]]);
      }
      for (const c of this.constraints) c.solve(h);
      // extra Gauss-Seidel passes over the JOINTS only (not motors): a chain
      // needs iteration to satisfy all attachments at once, and we don't want to
      // over-stiffen the muscles by re-solving them too.
      for (let k = 0; k < 3; k++) for (const c of this.constraints) if (c.joint) c.solve(h);
      for (const b of this.bodies) {
        if (b.invM === 0) continue;
        b.v = v3.scale(v3.scale(v3.sub(b.p, b.pp), 1 / h), this.linDamp);
        let dq = q4.mul(b.q, q4.conj(b.pq));
        if (dq[3] < 0) dq = [-dq[0], -dq[1], -dq[2], -dq[3]];
        b.w = v3.scale([2 / h * dq[0], 2 / h * dq[1], 2 / h * dq[2]], this.angDamp);
      }
    }
  }
}

// ----- a 2-bone active-ragdoll arm (shoulder fixed → upper → lower) -----
// Bones are along local -Y (length L). Anchored at `shoulder`. Returns handles +
// a setTarget(qUpper, qLower) that updates the motor rest orientations (the L2
// pose). Call world.step(dt) each frame; read upper.q / lower.q for the result.
export function makeArm(world, { shoulder = [0, 1.4, 0], Lupper = 0.26, Llower = 0.24, mass = 1.2, compliance = 0.0006 } = {}) {
  const anchor = world.add(new Body({ pos: shoulder, fixed: true }));
  const upper = world.add(new Body({ pos: v3.add(shoulder, [0, -Lupper / 2, 0]), mass }));
  const lower = world.add(new Body({ pos: v3.add(shoulder, [0, -Lupper - Llower / 2, 0]), mass: mass * 0.8 }));
  // joints: shoulder (anchor↔upper top) and elbow (upper bottom↔lower top)
  // motors first, then joints LAST, so each substep ends with the bones
  // re-connected (the motor rotates a bone about its COM, displacing the joint;
  // solving Attach afterwards pulls them back together → no drift).
  const mU = world.constrain(Motor(anchor, upper, [0, 0, 0, 1], compliance));
  const mL = world.constrain(Motor(upper, lower, [0, 0, 0, 1], compliance));
  world.constrain(Attach(anchor, [0, 0, 0], upper, [0, Lupper / 2, 0]));
  world.constrain(Attach(upper, [0, -Lupper / 2, 0], lower, [0, Llower / 2, 0]));
  return {
    anchor, upper, lower, mU, mL, Lupper, Llower,
    setTarget(qU, qL) { mU.rest = qU.slice(); mL.rest = qL.slice(); },
    handPos() { return lower.worldPoint([0, -Llower / 2, 0]); },
  };
}
