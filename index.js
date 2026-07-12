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
//
// M3 scope: CONTACTS as one-sided positional constraints (same XPBD frame as the
// joints, so they re-iterate together and stay stable under stiff motors):
//   GroundContact   keep a body above a horizontal plane (卓=面: the table top).
//   BoxContact      keep a body out of an axis-aligned box (牌=小箱, 卓縁).
//   Contact         sphere↔sphere push-apart — the body's own SELF-COLLISION
//                   ("肉で腕が回り込む"): limbs ride AROUND the torso, they don't
//                   pass through it. A BodyProfile `bulk` inflates the torso
//                   colliders → a heavier build's arms swing wider around the belly.

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
  constructor({ pos = [0, 0, 0], q = [0, 0, 0, 1], mass = 1, radius = 0.06, cr = null, fixed = false } = {}) {
    this.p = pos.slice(); this.q = q.slice();
    this.v = [0, 0, 0]; this.w = [0, 0, 0];
    this.invM = fixed ? 0 : 1 / mass;
    // isotropic inertia of a solid sphere I = 2/5 m r^2
    this.invI = fixed ? 0 : 1 / (0.4 * mass * radius * radius);
    this.cr = cr != null ? cr : radius;   // collision radius (M3 contacts); defaults to the physics radius
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
    motor: true,                                          // tagged so snapshot/restore can find the CONTROL state
    A, B, rest, compliance,                               // `compliance` is a live field: the control may vary it per frame
    solve(h) {
      const qRel = q4.mul(q4.conj(A.q), B.q);             // current child-in-parent
      let qErr = q4.mul(this.rest, q4.conj(qRel));        // rotation needed (parent frame)
      if (qErr[3] < 0) qErr = [-qErr[0], -qErr[1], -qErr[2], -qErr[3]];
      const theta = q4.rot(A.q, [2 * qErr[0], 2 * qErr[1], 2 * qErr[2]]);   // → world
      const ang = v3.len(theta); if (ang < 1e-9) return;
      const axis = v3.scale(theta, 1 / ang);
      const at = this.compliance / (h * h);
      const dl = ang / (A.invI + B.invI + at);
      const corr = v3.scale(axis, dl);
      A.applyDRot(v3.scale(corr, -A.invI));
      B.applyDRot(v3.scale(corr, B.invI));
    },
  };
}

// ----- contacts (M3) -----
// One-sided positional constraints: they do NOTHING until a body penetrates, then
// project it out along the contact normal. Positional-only (no torque) — a body
// is treated as a sphere of radius `b.cr` about its COM, which is all the upper
// body needs to keep limbs out of the torso and off the table.

// keep sphere `B` (radius B.cr) above the horizontal plane y = `y`. compliance 0
// = rigid floor. This is the table top / 卓面.
export function GroundContact(B, { y = 0, compliance = 0 } = {}) {
  return {
    contact: true,
    solve(h) {
      if (B.invM === 0) return;
      const pen = (y + B.cr) - B.p[1];          // >0 → below the surface
      if (pen <= 0) return;
      const at = compliance / (h * h);
      B.p[1] += pen / (1 + at);                  // invM cancels (single free body)
    },
  };
}

// keep sphere `B` out of the axis-aligned box [min,max] (grown by B.cr): the tile
// box / table-edge box. Pushes out along the least-penetrating axis (the face the
// body is closest to escaping through) — the standard AABB pop-out.
export function BoxContact(B, min, max, { compliance = 0 } = {}) {
  return {
    contact: true,
    solve(h) {
      if (B.invM === 0) return;
      const r = B.cr;
      const lo = [min[0] - r, min[1] - r, min[2] - r];
      const hi = [max[0] + r, max[1] + r, max[2] + r];
      for (let i = 0; i < 3; i++) if (B.p[i] <= lo[i] || B.p[i] >= hi[i]) return;   // outside → no contact
      let axis = 0, depth = Infinity, dir = 1;
      for (let i = 0; i < 3; i++) {
        const dLo = B.p[i] - lo[i], dHi = hi[i] - B.p[i];
        if (dLo < depth) { depth = dLo; axis = i; dir = -1; }
        if (dHi < depth) { depth = dHi; axis = i; dir = 1; }
      }
      const at = compliance / (h * h);
      B.p[axis] += (dir * depth) / (1 + at);
    },
  };
}

// sphere↔sphere push-apart: keep COMs at least (A.cr + B.cr) apart. This is the
// SELF-COLLISION primitive — register it between a limb body and a torso body and
// the limb can no longer pass through the trunk; under a stiff reach it slides
// AROUND instead. Split by inverse mass so a fixed/heavy part barely moves.
export function Contact(A, B, { compliance = 0 } = {}) {
  return {
    contact: true,
    solve(h) {
      const wsum = A.invM + B.invM;
      if (wsum === 0) return;
      const d = v3.sub(A.p, B.p);
      const dist = v3.len(d);
      const dMin = A.cr + B.cr;
      if (dist >= dMin || dist < 1e-9) return;  // not overlapping
      const n = v3.scale(d, 1 / dist);
      const C = dMin - dist;                     // penetration depth (>0)
      const at = compliance / (h * h);
      const dl = C / (wsum + at);
      const P = v3.scale(n, dl);
      A.p = v3.add(A.p, v3.scale(P, A.invM));
      B.p = v3.sub(B.p, v3.scale(P, B.invM));
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
      // extra Gauss-Seidel passes over the JOINTS + CONTACTS (not motors): a chain
      // needs iteration to satisfy all attachments at once, and contacts must be
      // re-resolved after the joints pull bodies back (or a limb the joint just
      // moved would be left penetrating the torso). Motors are left out so we
      // don't over-stiffen the muscles by re-solving them.
      for (let k = 0; k < 3; k++) for (const c of this.constraints) if (c.joint || c.contact) c.solve(h);
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

// ----- M2: full pelvis-anchored upper-body chain -----
// three.js 'XYZ' Euler → quat, so a motion-engine pose ({bone:[x,y,z]}) drops in
// as the motor targets with no conversion surprises.
export function qFromEulerXYZ(e) {
  const c1 = Math.cos(e[0] / 2), c2 = Math.cos(e[1] / 2), c3 = Math.cos(e[2] / 2);
  const s1 = Math.sin(e[0] / 2), s2 = Math.sin(e[1] / 2), s3 = Math.sin(e[2] / 2);
  return [s1 * c2 * c3 + c1 * s2 * s3, c1 * s2 * c3 - s1 * c2 * s3, c1 * c2 * s3 + s1 * s2 * c3, c1 * c2 * c3 - s1 * s2 * s3];
}

// a default upper-body skeleton (bone → parent, rest offset from parent origin).
// `off` is the child's joint position in the parent's local frame at rest. Real
// integration measures these from the VRM; these are plausible defaults for tests.
export const UPPER_BODY = Object.freeze([
  { name: 'hips', parent: null, off: [0, 1.0, 0], fixed: true },
  { name: 'spine', parent: 'hips', off: [0, 0.12, 0] },
  { name: 'chest', parent: 'spine', off: [0, 0.14, 0] },
  { name: 'neck', parent: 'chest', off: [0, 0.15, 0] },
  { name: 'head', parent: 'neck', off: [0, 0.08, 0] },
  { name: 'leftShoulder', parent: 'chest', off: [0.06, 0.10, 0] },
  { name: 'leftUpperArm', parent: 'leftShoulder', off: [0.10, 0, 0] },
  { name: 'leftLowerArm', parent: 'leftUpperArm', off: [0, -0.26, 0] },
  { name: 'leftHand', parent: 'leftLowerArm', off: [0, -0.24, 0] },
  { name: 'rightShoulder', parent: 'chest', off: [-0.06, 0.10, 0] },
  { name: 'rightUpperArm', parent: 'rightShoulder', off: [-0.10, 0, 0] },
  { name: 'rightLowerArm', parent: 'rightUpperArm', off: [0, -0.26, 0] },
  { name: 'rightHand', parent: 'rightLowerArm', off: [0, -0.24, 0] },
]);

// BodyProfile (M3): per-avatar physical character. `mass` scales every bone's
// mass (heavier = sags more under the same muscle); `bulk` inflates the torso
// colliders (a wider build to ride limbs around); `selfCollision` opts the trunk
// vs forearm/hand contacts in. Defaults reproduce the M2 body exactly.
export const DEFAULT_PROFILE = Object.freeze({ mass: 1.0, bulk: 0, selfCollision: false });

// per-bone collision radii (m, about the COM). The trunk bones are fat targets
// limbs must go around; arms are thin. `bulk` adds to the trunk only.
const COLLIDER_R = Object.freeze({
  hips: 0.13, spine: 0.12, chest: 0.13, neck: 0.05, head: 0.10,
  leftShoulder: 0.05, rightShoulder: 0.05,
  leftUpperArm: 0.05, rightUpperArm: 0.05,
  leftLowerArm: 0.045, rightLowerArm: 0.045, leftHand: 0.05, rightHand: 0.05,
});
const TRUNK = Object.freeze(['hips', 'spine', 'chest']);

/**
 * Build a pelvis-anchored active-ragdoll upper body. Each bone is a rigid body
 * jointed to its parent (Attach) and driven by a compliant Motor toward a target
 * orientation. Feed a motion-engine pose via setPose() each frame; step the world;
 * read bodies[name].q for the physical result (relative-to-parent = the bone's
 * local rotation the renderer applies).
 *
 * `profile` (BodyProfile) gives the body physical character — mass / bulk /
 * self-collision (see DEFAULT_PROFILE). Omit it and the body is identical to M2.
 */
export function makeUpperBody(world, { skeleton = UPPER_BODY, mass = 1.0, compliance = 0.0008, profile = null } = {}) {
  const prof = Object.assign({}, DEFAULT_PROFILE, profile);
  const bulk = prof.bulk || 0;
  const bodies = {}, motors = {}, worldPos = {}, parentOf = {};
  for (const b of skeleton) {
    const base = b.parent ? worldPos[b.parent] : [0, 0, 0];
    worldPos[b.name] = v3.add(base, b.off);
    let cr = COLLIDER_R[b.name] != null ? COLLIDER_R[b.name] : 0.04;
    if (TRUNK.includes(b.name)) cr += bulk * 0.08;      // a heavier build → a wider trunk
    const body = world.add(new Body({ pos: worldPos[b.name], mass: b.fixed ? 1 : mass * prof.mass, fixed: !!b.fixed, cr }));
    bodies[b.name] = body; parentOf[b.name] = b.parent;
    if (b.parent) {
      motors[b.name] = world.constrain(Motor(bodies[b.parent], body, [0, 0, 0, 1], compliance));
      world.constrain(Attach(bodies[b.parent], b.off, body, [0, 0, 0]));
    }
  }
  // self-collision: forearms + hands ride AROUND the trunk, not through it. Opt-in
  // via the profile so the default body stays byte-for-byte the M2 body.
  if (prof.selfCollision) {
    const limbs = ['leftLowerArm', 'leftHand', 'rightLowerArm', 'rightHand'];
    for (const t of TRUNK) for (const l of limbs) {
      if (bodies[l] && bodies[t]) world.constrain(Contact(bodies[l], bodies[t]));
    }
  }
  return {
    bodies, motors, parentOf, profile: prof, skeleton,
    setPose(poseEuler) {
      for (const b of skeleton) {
        if (!b.parent || !motors[b.name]) continue;
        const e = poseEuler && poseEuler[b.name];
        motors[b.name].rest = e ? qFromEulerXYZ(e) : [0, 0, 0, 1];
      }
    },
  };
}
