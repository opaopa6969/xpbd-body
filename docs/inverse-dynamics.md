**English** · [日本語](./inverse-dynamics.ja.md)

# Inverse dynamics — the analysis-by-synthesis substrate (M4)

> `import { makeRig, simulate, estimateControl } from 'xpbd-body/inverse'`

The internet's 2D reference footage shows you **where the joints went**. It does not show you the **control** that put them there — the muscle tension, the joint torque, the timing, the coordination. But the body's physical constraints (range of motion, mass, inertia, how a muscle actually pulls) are a strong enough prior that you can **run the control backwards out of the positions**. That is classical inverse dynamics, and it is what [keiko-engine](https://github.com/opaopa6969/keiko-engine)'s 稽古ループ needs so that its motion descriptor is *the estimated control*, not the raw 2D positions.

xpbd-body already owned half the problem. The active ragdoll — compliant `Motor`s ("muscles") driving a real, heavy, gravity-bound chain — **is** the forward model: control → trajectory. M4 adds the two things an estimation loop needs on top of it.

```
                 ┌──────────── the forward model (already existed) ────────────┐
   control  ───► │  Motor targets → XPBD substeps → mass, gravity, contacts    │ ───► trajectory
                 └────────────────────────────────────────────────────────────┘
                                              ▲                        │
        estimateControl:  fix the guess ──────┘        compare ◄───────┘  observed trajectory
```

## The API

### `snapshot(world)` / `restore(world, snap)` — rewind

Plain-data dump of every body's `p / q / v / w` **and the motor control state** (`rest` target + `compliance`). The estimation loop lives or dies on this: it must try control A, rewind, try control B, from a bit-identical initial state. A snapshot is inert — hold it as long as you like.

### `makeRig(opts)` — the body plus its priors

The world, the driven bones, and the two priors that make the inverse problem tractable at all: **`rom`** (per-bone joint range of motion, Euler XYZ; sotai-engine ROMs drop straight in) and **`maxTorque`** (per-bone sustained muscle-tension ceiling). Plus `rig.initial`, the rest-state snapshot every estimation trial starts from.

### `simulate(rig, controlTrajectory, dt, steps)` → `poseTrajectory` — the forward model *(issue #2)*

Pure. **Side-effect free**: it snapshots the world on entry and restores it on exit, so the caller's live simulation is untouched. **Deterministic**: same start state + same control → byte-identical output. The existing incremental `world.step(dt)` API is untouched — real-time use does not go through here.

A control frame is `{ pose: { bone: [x, y, z] }, compliance?: { bone: number } }` — the same Euler-XYZ pose a motion-engine frame is. A short trajectory holds its last frame. Each output frame carries `rel` (the bone's local rotation = what a renderer applies), `pos`, and `torque` (what the muscle is pulling at, right now).

### `estimateControl(rig, observed, opts)` → `{ control, residual, feasible }` *(issue #3)*

Guess a control, run it forward, look at the gap, fix the guess. Gradient-free — the simulator is a black box with contacts in it, there is no clean derivative — and **deterministic** (a seeded PRNG; `Math.random` appears nowhere in this repo).

- **Parameterisation.** The control is a handful of **knots** per bone (Euler triples, linearly interpolated over the frames), not one free target per frame. This collapses the search from thousands of dimensions to dozens, and is itself a smoothness prior: muscles do not teleport. `knots` is the bias/variance knob.
- **Initialisation.** The search starts *from the observation itself*. The pose you see is already a good guess at the pose the muscle was aiming for — the two differ by the sag, which is exactly what the search then has to find. This matters more than the optimiser.
- **Objective.** Weighted sum of squared joint-angle error against the observation (plus optional position error), **plus a smoothing regulariser** on the control's curvature.
- **Optimiser.** `'coord'` (coordinate descent / pattern search with a shrinking step — the default, and the more accurate of the two here) or `'cem'` (cross-entropy method, seeded; useful when the landscape is rough). CEM's `sigma0` is the parameter that matters: the search starts from a good guess, so a wide sampling cloud just draws worse controls forever.

## Feasibility — three ways an observation can be a lie

`feasible: false` comes with `violations[]`, each naming the joint and the excess.

| prong | what it catches |
|---|---|
| **`rom`** | The observed joint angles are outside the joint's range. An elbow bending backwards is not a hard control problem — it is impossible. |
| **`torque`** | Reproducing it needs a *sustained* muscle pull past the motor's ceiling. |
| **`residual`** | The body simply cannot get there: the best control found still misses by a mile. |

The third prong is the subtle one, and it is where **"the pose collapses under its own weight"** actually shows up. A weak muscle *cannot exert a large torque* — so a sagging body keeps the torque prong quiet **by construction**. What betrays it is that no control reproduces the observation at all. If you only check the torque ceiling, gravity collapse slips straight through. (This surprised us; the test is #25.)

Sustained torque is the **median** over the trajectory, not the peak. A step change in the motor target spikes the torque for a few frames — that is the simulator's transient, not the body's effort, and a ceiling on the peak would only be measuring how abruptly the control was authored.

Torque is read off the motor as a compliant torsional spring: `τ = tracking error / compliance`. Substep-independent, and in steady state it equals the torque gravity is demanding — which is the whole point. The units are **engine units**, not newton-metres: this body's inertia is a sphere approximation and its compliances are not calibrated against a human. `DEFAULT_MAX_TORQUE` is therefore deliberately permissive (ordinary motion lands at ~10 for a soft muscle, ~600 for a very stiff one). Tighten it per rig and the same machinery models a *weak* body.

## The limits — read this before trusting a number

**The inverse problem is ill-posed.** This is not a caveat to be engineered away; it is the shape of the problem.

- **Many controls produce the same motion.** A stiff muscle aiming exactly at the pose and a slack muscle aiming well past it can settle in the same place. `estimateControl` does **not** return *the* control. It returns **one** control — the one the residual and the smoothing regulariser picked out of the family that fits. It reports `unique: false` to say so out loud. Change `smooth` and you will get a different, equally valid answer.
- **Contact forces are unobservable.** If the body is leaning on a table, the table is doing some of the work, and nothing in a pose trajectory says how much. The estimator will attribute the table's support to the muscles and hand back a control that is wrong in a way the residual cannot see.
- **A monocular observation has no depth.** Lift a 2D video into 3D joint angles and the depth is a *guess*. Everything downstream inherits it. xpbd-body takes a 3D pose trajectory and asks no questions about where it came from — the depth ambiguity is upstream, and it does not go away by being ignored.
- **The knots bound what is recoverable.** A control with more structure than the knots can represent gets smoothed away. This is a deliberate prior, but it is a prior.

Use the residual. A control with a large residual is not a poor estimate of the true control — it is a statement that **this body cannot do that**.

## Measured

Node 20, one core.

| | full upper body (13 bodies) | 3-bone arm |
|---|---|---|
| forward `simulate`, 2 s @ 60 Hz, substeps=20 | 43 ms | 9.5 ms |
| …substeps=8 | 14.5 ms | 3.6 ms |
| …substeps=4 | 8.8 ms | 2.4 ms |

Substeps are the dial. 20 is the real-time default; an estimation loop can usually drop to 8 and pay ~3× less for a residual that stays in the third decimal.

**Synthetic round trip** (3-bone arm, 1 s, substeps=8, knots=3, 5 sweeps of coordinate descent) — take a known control, generate its trajectory, throw the control away, estimate it back:

- joint-angle residual: **rms 0.0029 rad, max 0.0074 rad** (0.17° / 0.42°)
- error against the **true control**: **rms 0.0029 rad, max 0.0074 rad**
- 178 forward simulations, ~400 ms total
- CEM (8 generations × 16 samples, seeded): rms 0.0043 rad, 129 sims

The control-space error being the same size as the pose residual is the good case, not the general one: this observation was *generated by this body*, so a control that reproduces the motion is close to the control that made it. Against real footage, expect the pose residual to stay small while the control error is bounded by nothing but the priors above.

## Tests

`node test.mjs` — tests 19–28. Determinism (byte-identical trajectories), purity (`simulate` leaves the world untouched), snapshot → detour → restore → the original control reproduces the first run exactly, the synthetic round trip, seeded CEM reproducibility, and all three infeasibility prongs.
