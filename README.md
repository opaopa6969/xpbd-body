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
- `new Body({ pos, q, mass, radius, fixed })` — rigid body (isotropic inertia for now).
- `Attach(A, rA, B, rB, compliance=0)` — ball joint keeping local points coincident (re-iterated to hold a chain).
- `Motor(A, B, restQuat, compliance)` — drive B's orientation relative to A toward `restQuat`; compliance = muscle softness (small = strong, large = sags).
- `makeArm(world, opts)` → `{ upper, lower, setTarget(qU,qL), handPos() }` — a 2-bone active-ragdoll arm.
- `makeUpperBody(world, opts)` → `{ bodies, setPose(poseEuler) }` — a pelvis-anchored upper-body chain (M2). `setPose` takes a motion-engine pose (`{bone:[x,y,z]}`) as the motor targets; read `bodies[name].q` for the physical result. `qFromEulerXYZ`, `UPPER_BODY` exported too.

## Test

```sh
node test.mjs     # or: npm test
```

Headless proof of the active ragdoll: stable under stiff motors, joints stay connected, a strong muscle tracks the target, a weak/heavier arm sags under gravity, a shove perturbs then recovers, and it's deterministic.

## Status

**M1** XPBD core + 2-bone active-ragdoll arm. **M2** (this) full pelvis-anchored upper-body chain (`makeUpperBody` + `setPose`) driven by motion-engine poses. Roadmap: M3 contacts + bulk self-collision · M4 integrate into a host as an opt-in physical mode.

## License

MIT
