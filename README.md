**English** · [日本語](./README.ja.md)

# xpbd-body

> A tiny from-scratch **XPBD active-ragdoll** for VRM upper bodies — real gravity, mass, momentum and contact, that **tracks a target pose**.

The L3 dynamics layer for [motion-engine](https://github.com/opaopa6969/motion-engine): where motion-engine *generates* a target pose kinematically (L2), `xpbd-body` makes a body with **real mass and gravity physically track it** via compliant "muscle" motors — so it sags when weak, reacts when shoved, and respects contact. **Pure, dependency-free, deterministic** (fixed substeps, no `Math.random`) — so it runs headless and is unit-tested in Node, and stays compatible with deterministic replay.

```js
import { World, makeArm, q4 } from 'xpbd-body';

const world = new World();
const arm = makeArm(world, { mass: 1.2, compliance: 0.0006 });   // shoulder→upper→lower

// each frame: feed the L2 pose as the motor targets, step, read the result
arm.setTarget(qUpper, qLower);     // child-in-parent quaternions (e.g. from motion-engine)
world.step(1 / 60);
const handWorld = arm.handPos();   // arm.upper.q / arm.lower.q drive the VRM bones
```

## Why XPBD

Position-based dynamics: joint **attachment**, angular **motors** (the "muscles"), joint limits and **contacts** are all the *same kind* of constraint, and it stays stable under stiff motors via substepping. That uniformity is why it's the right substrate for what's next — self-collision ("the arm goes around the belly") drops in as another contact constraint.

The whole thing is four primitives: **`World` / `Body` / `Attach` (joint) / `Motor` (muscle)**.

## API

- `new World({ gravity?, linDamp?, angDamp? })` → `step(dt, substeps=20)`, `add(body)`, `constrain(c)`.
- `new Body({ pos, q, mass, radius, cr?, fixed })` — rigid body (isotropic inertia for now). `cr` = collision radius for M3 contacts (defaults to `radius`).
- `Attach(A, rA, B, rB, compliance=0)` — ball joint keeping local points coincident (re-iterated to hold a chain).
- `Motor(A, B, restQuat, compliance)` — drive B's orientation relative to A toward `restQuat`; compliance = muscle softness (small = strong, large = sags).
- `GroundContact(B, { y, compliance })` / `BoxContact(B, min, max, { compliance })` / `Contact(A, B, { compliance })` — one-sided contact constraints (M3): keep a body above a plane (table top), out of an AABB (tile / table edge), or apart from another body (self-collision). They no-op until penetration, then project out; re-iterated with the joints.
- `makeArm(world, opts)` → `{ upper, lower, setTarget(qU,qL), handPos() }` — a 2-bone active-ragdoll arm.
- `makeUpperBody(world, { ..., profile })` → `{ bodies, motors, setPose(poseEuler) }` — a pelvis-anchored upper-body chain (M2). `setPose` takes a motion-engine pose (`{bone:[x,y,z]}`) as the motor targets; read `bodies[name].q` for the physical result. A `profile` (BodyProfile: `{ mass, bulk, selfCollision }`, see `DEFAULT_PROFILE`) gives the body physical character — `mass` scales sag, `bulk` widens the trunk, `selfCollision` rides the forearms/hands around it instead of through it. Omit it and the body is byte-for-byte the M2 body. `qFromEulerXYZ`, `UPPER_BODY` exported too.

## Inverse dynamics (M4) — `xpbd-body/inverse`

The active ragdoll is already a **forward model** (control → trajectory). M4 adds the machinery to run it *backwards*: given an observed pose trajectory, estimate the **control** that produced it. That is what [keiko-engine](https://github.com/opaopa6969/keiko-engine) needs so its motion descriptor is the estimated control, not raw 2D positions. See **[docs/inverse-dynamics.md](./docs/inverse-dynamics.md)**.

```js
import { makeRig, simulate, estimateControl } from 'xpbd-body/inverse';

const rig = makeRig({ compliance: 0.00006 });        // body + priors (ROM, torque ceilings)
const traj = simulate(rig, control, 1 / 60, 120);    // control → trajectory. pure, deterministic
const { control, residual, feasible } = estimateControl(rig, observedTrajectory);
```

- `snapshot(world)` / `restore(world, snap)` — plain-data dump/restore of every body **and the motor control state**, so an estimation loop can rewind and try a different control from a bit-identical start.
- `simulate(rig, controlTrajectory, dt, steps)` → `poseTrajectory` — the forward model. **Side-effect free** (snapshots and restores the world) and **deterministic**. The incremental `world.step(dt)` API is untouched.
- `estimateControl(rig, observed, opts)` → `{ control, residual, feasible, violations, unique }` — analysis by synthesis: guess a control, run it forward, look at the gap, fix the guess. Gradient-free (coordinate descent, or a seeded CEM), no `Math.random` anywhere. `feasible: false` names the joint that broke its **range of motion**, blew its **torque ceiling**, or the **residual** that says this body simply cannot do that.

**The inverse problem is ill-posed** and the API says so (`unique: false`): many controls produce the same visible motion, contact forces are unobservable, a monocular observation has no depth. `estimateControl` returns *one* control — the one the residual and the smoothing regulariser picked — not *the* control. [The limits are documented.](./docs/inverse-dynamics.md)

## Test

```sh
node test.mjs     # or: npm test
npm run mcp:test  # e2e for the MCP server (starts it, runs tools + resources)
```

Headless proof of the active ragdoll: stable under stiff motors, joints stay connected, a strong muscle tracks the target, a weak/heavier arm sags under gravity, a shove perturbs then recovers, and it's deterministic. Plus the M4 inverse layer: a **synthetic round trip** (known control → trajectory → estimate → back to the control within 0.003 rad rms), rewind-and-retry, and all three infeasibility prongs.

## MCP

This library is also an **MCP server** (namespace `xpbd`, on [volta](https://github.com/opaopa6969/volta-mcp)). It exposes the forward model and the inverse layer as tools so other MCP services can compose with them.

- **Spec**: `xpbd://spec` (machine-readable capability list). **Guide**: `xpbd://guide`.
- **Tools**: `simulate` (pose → physics-follow), `estimate_control_start/status/result` (observed → control, job-typed), `check_feasible` (physical feasibility).
- **Start locally**: `PORT=9204 npm run mcp:start` → `curl http://127.0.0.1:9204/healthz`.
- **Design**: `docs/mcp/DESIGN.md`. **Status**: `docs/mcp/STATUS.md`. **Skill**: `docs/skills/xpbd-body-mcp-usage/SKILL.md`.

## Status

**M1** XPBD core + 2-bone active-ragdoll arm. **M2** full pelvis-anchored upper-body chain (`makeUpperBody` + `setPose`) driven by motion-engine poses. **M3** contacts as one-sided XPBD constraints — ground/plane (table), AABB box (tile/edge), and sphere↔sphere self-collision driven by a `BodyProfile` (`bulk` widens the trunk so limbs ride around it). **M4** (this) inverse dynamics — snapshot/restore, a pure forward model, and gradient-free control estimation. Roadmap: contact friction · joint limits as XPBD constraints (not just a post-hoc feasibility prior) · keiko-engine integration.

## License

MIT
