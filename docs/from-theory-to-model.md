**English** · [日本語](./from-theory-to-model.ja.md)

# From theory to the engine/model — the xpbd-body design document

This document records how the **theory (position-based physics / constraints / substeps / compliance)** told in `edu/theory.md` was turned into the actual **algorithm → data structures → implementation policy → tests** of `index.js`. The goal is to map theory and implementation one-to-one.

Premise: xpbd-body is motion-engine's **L3 (dynamics) layer**. L2 kinematically generates a "target pose," and L3 makes a **body with real mass and gravity** **physically track** that pose. If it's weak, it sags; if pushed, it reacts; it respects contact.

---

## 1. Theory → choices

The theory section made four claims. Each one became a design decision as-is.

| Theory | Why it's nice | Consequence for the implementation |
| --- | --- | --- |
| **Position-based** (fix position directly, not force) | Doesn't diverge even with stiff muscles | Integration is velocity-Verlet-like; correction is applied directly to position |
| **A single concept: "constraint"** | Joints, muscles, and contacts all solve the same way | One array of objects that each have `solve(h)`, run through in a single loop |
| **Compliance** | Muscle softness = a single number | Just add `α = compliance / h²` to each constraint's denominator |
| **Substeps** | Slicing finely is stable | `World.step` divides `dt` into `substeps` |

The root decision was to abandon "force → acceleration" and go all in on "move the position → pull it back with constraints." That's what buys the simplicity of **solving joints, muscles, and the ground in the same loop**.

---

## 2. Algorithm

### 2.1 Dividing dt into substeps (`World.step`)

One frame's `dt` is split into `substeps` (default 20) small steps of `h = dt / substeps`. Each small step does:

1. **Predict**: add gravity to velocity, and advance position/orientation by `h` (`p += v·h`; orientation from angular velocity). Keep the previous position `pp` / orientation `pq` before advancing.
2. **Constraint projection**: call `solve(h)` on every constraint, correcting the drift directly on position/orientation.
3. **Recompute velocity**: derive velocity from the difference between the "corrected position" and the "position kept before prediction" (`v = (p − pp) / h`). Damping (`linDamp`/`angDamp`) is applied here.

The key is step 3. **Velocity isn't integrated — it's derived afterward from the change in position.** This is the heart of position-based (PBD/XPBD): whatever amount the constraints warped things by is properly reflected back into velocity (which is what naturally produces "gets pushed, then recovers").

### 2.2 Iterating constraint projection (Gauss–Seidel)

The "whack-a-mole" problem from the theory section: a chain can't satisfy all its constraints simultaneously in a single pass. So:

- First, solve **all constraints** once (motors are solved here too).
- Then solve **only the joints and contacts** for 3 additional passes (`if (c.joint || c.contact)`).

Joints get extra iterations so the whole chain of attachments converges together. Contacts get extra iterations so that **penetration gets resolved again after a joint has pulled a bone back** (preventing the order-dependent case where "the limb the joint just moved is left sunk into the torso").

**Excluding motors from the extra iterations** is a deliberate design choice. If motors were solved repeatedly too, the muscle would become excessively stiff and break the meaning of compliance. Hence the asymmetry: "solve the muscle once, but converge the bone connections and contacts thoroughly."

### 2.3 How Attach (joints) is solved

A ball joint. It keeps a local point `rA` on A coincident with a local point `rB` on B.

- Measure the world-space gap `C` between the two points (length `c`, direction `n`).
- Compute each body's **effective mass** `w = invM + invI·|r×n|²` (how easily it translates, plus how easily it rotates around that point).
- The XPBD correction is `Δλ = −c / (wA + wB + α)`, with `α = compliance / h²`.
- The correction vector `P = n·Δλ` is applied to translation (distributed via `invM`) and rotation (`r×P` distributed via `invI`).

When `compliance = 0`, `α = 0` and the joint is rigid. Having `α` in the denominator makes the **stiffness independent of step size and constant** — that's the reason this is **X**PBD (extended), not plain PBD.

### 2.4 How Motor (muscle) is solved

An angular constraint that drives B's orientation relative to its parent A toward a target `rest`.

- Current relative orientation: `qRel = conj(A.q)·B.q`.
- Rotation error to the target: `qErr = rest · conj(qRel)`. Flip the sign if `qErr[3] < 0`, to take the shortest path.
- Rotate the imaginary part of the error quaternion (×2) into world space to get a **rotation vector `θ`** (axis × angle).
- `Δλ = ang / (A.invI + B.invI + α)` is applied along the axis, distributed to both bodies via `invI`.

The structure mirrors Attach almost exactly (the positional version becomes a rotational one). Fitting into the **same XPBD shape** is the concrete embodiment of the theory section's "one kind of constraint for everything." `compliance` is literally the muscle strength: small = strong, large = loses to gravity and sags.

### 2.5 How contacts are solved (one-sided constraints)

`GroundContact` / `BoxContact` / `Contact` are all **one-sided**. They **do nothing** (early return) until penetration occurs. Once penetration happens, they push out along the normal:

- Ground: for the plane `y`, `pen = (y + cr) − p.y`. If positive, `p.y += pen / (1 + α)`.
- Box: the AABB is inflated by `cr`; if outside, do nothing. If inside, push out from the **shallowest face**.
- Contact (sphere↔sphere): if the COM distance is less than `A.cr + B.cr`, push apart along the normal, distributed via `invM` — this is what produces self-collision (an arm riding around the torso).

Contacts are position-only (no torque), treating each body as a sphere of radius `cr` centered at its COM. This is a necessary-and-sufficient approximation for keeping the upper body afloat above a table and limbs from piercing the torso.

---

## 3. Data structures

The theory's four concepts map exactly onto four primitives.

- **`World`** … the whole. Holds `bodies[]` and `constraints[]`, driven by `step(dt, substeps)`. Keeps `gravity` / `linDamp` / `angDamp`.
- **`Body`** … a single bone. `p` (position) / `q` (orientation) / `v`, `w` (linear/angular velocity) / `invM`, `invI` (inverse mass/inertia) / `cr` (collision radius). If `fixed`, `invM=invI=0` and it's immovable (shoulder/pelvis anchors). Inertia is **isotropic** for now (sphere approximation, `I = 2/5·m·r²`).
- **`Attach`** / **`Motor`** / **`*Contact`** … all just constraint objects that expose `{ solve(h) }`. Flags `joint:true` / `contact:true` mark which ones are targeted by the extra iterations.

**The essence of the design**: every constraint shares the same `solve(h)` interface. `World` just loops through the array without knowing what's inside. A new constraint (friction, joint limits, a new kind of contact) drops in with just one more `solve(h)` — the theory section's "everything is the same kind" is what gives the API its shape.

Assembly functions:

- `makeArm` … a 2-bone active ragdoll: anchor → upper arm → forearm.
- `makeUpperBody` … a pelvis-anchored upper-body chain (`UPPER_BODY` skeleton + `BodyProfile`). `setPose(poseEuler)` feeds a motion-engine pose into each motor's `rest`.

Motors are registered **first**, and Attach **last**, and that order matters: a motor rotates a bone around its COM, shifting the joint point → then Attach pulls it back → each substep ends "connected" (no drift).

`BodyProfile` (`{ mass, bulk, selfCollision }`) is the avatar's build. `mass` controls how much it sags, `bulk` fattens the torso collider so the arms ride around it, `selfCollision` enables contact between the torso and forearms/hands. The default reproduces the M2 body byte-for-byte (behavior is unchanged if no profile is given).

---

## 4. Implementation policy

Strictly follows the same conventions as motion-engine.

- **Pure / zero dependencies**: no external libraries. vec3/quat are also hand-written (`v3` / `q4`). Runs headless in Node.
- **Determinism**: `Math.random` is forbidden. No randomness, wall-clock time, or environment dependence of any kind. Same input → same output. Compatible with deterministic replay.
- **Fixed substeps**: `step` divides `dt` into a fixed count (default 20). Never variable-step — for reproducibility and replay compatibility.
- **No side effects**: constraints only touch `World`'s own state (a body's `p`/`q`). No I/O, no logging.
- **Isotropic inertia, deliberately**: a sphere approximation for now. Real inertia tensors and friction/joint limits are future work. The design is kept extensible so those only require adding a `solve(h)`.

These aren't decoration — they guarantee **headless unit-testability** and **replay producing the same picture**. Those two properties are prioritized over correctness as a general-purpose physics engine.

---

## 5. Test policy

`test.mjs` (`npm test`) verifies the active ragdoll as a **headless proof**, property by property. The policy is to check **behavioral properties**, not rounded numeric values.

Properties verified:

1. **Stability**: doesn't diverge even under stiff motors (position/velocity stay finite). The regression test for the theory section's "doesn't explode."
2. **Connectivity**: joints stay attached (Attach's drift stays below a threshold). The chain doesn't fall apart.
3. **Tracking**: a strong muscle (small compliance) properly tracks the target pose (final pose is close enough to the target).
4. **Sag**: a weak/heavy arm droops under gravity (raising compliance and mass lowers the hand). Proof that compliance's meaning actually takes effect.
5. **Push and recover**: an external poke disturbs it momentarily, and it returns to the target over time. Proof that velocity is correctly reconstructed under the position-based scheme.
6. **Determinism**: running twice from the same initial conditions matches **bit-for-bit**. Guarantees the exclusion of `Math.random` and the fixed substep count.

Every one of these tests "the constraint (= property) that must be satisfied," not "the correct number." This is itself the test-side version of the theory section's philosophy — decide what promises must be kept, and fix things when they're broken.

---

## Appendix: theory-speak ↔ code mapping table

| Theory-section term | Code |
| --- | --- |
| Fix position directly, not force → position | Predict → `solve` → derive velocity, inside `World.step` |
| A string of beads (joints stick together) | `Attach` |
| Muscle (be at this angle) | `Motor`'s `rest` |
| Muscle softness | `compliance` (→ `α = compliance/h²`) |
| Don't sink in (table, tile, belly) | `GroundContact` / `BoxContact` / `Contact` |
| Fix it repeatedly, in small steps | `substeps` division + extra iterations for `joint`/`contact` |
| Converging the whack-a-mole | The Gauss–Seidel 3-pass loop |
| Assembling a body | `makeArm` / `makeUpperBody` |
| Build (weight · girth) | `BodyProfile` = `{ mass, bulk, selfCollision }` |
