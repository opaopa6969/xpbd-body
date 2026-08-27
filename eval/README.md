# Inverse evaluation

`npm run eval` runs a small, dependency-free regression dataset for the inverse
estimator. Its three cases cover a fixed 0.015 rad noise distribution, one-third
temporal occlusion, and active torso/arm self-contact. The contact motion exceeds
the rig's ROM and torque priors, so it is also the fixed negative case used to
measure the false-feasible rate.

The fixtures are handcrafted and rebuilt from constants with seed `20260828`.
The runner evaluates them twice and requires both `datasetFingerprint` and
`metricsFingerprint` to match. Wall-clock `durationMs` is intentionally
observational; CI enforces only a generous 30 second ceiling to catch major
performance regressions without treating scheduler jitter as physics nondeterminism.

Control error is the RMS shortest quaternion angle between the known Euler-XYZ motor
target and the estimated motor target. This joint-angle definition matches the
estimator's residual units and does not add an end-effector weighting policy.

The JSON report records, per case, residual, feasible verdict, control error,
evaluation count, duration, and case-specific noise/occlusion/contact metadata. CI
runs the report after the unit suite and fails when a checked limit regresses.
