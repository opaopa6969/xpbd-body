import { q4, qFromEulerXYZ } from '../index.js';
import { makePRNG, makeRig, qAngle, simulate } from '../inverse.js';

export const DATASET_VERSION = 'xpbd-body/inverse-eval@1';
export const SEED = 20260828;
export const BONES = Object.freeze(['rightUpperArm', 'rightLowerArm']);
export const ARM_SKELETON = Object.freeze([
  { name: 'chest', parent: null, off: [0, 1.3, 0], fixed: true },
  { name: 'rightUpperArm', parent: 'chest', off: [-0.16, 0.1, 0] },
  { name: 'rightLowerArm', parent: 'rightUpperArm', off: [0, -0.26, 0] },
  { name: 'rightHand', parent: 'rightLowerArm', off: [0, -0.24, 0] },
]);

const STEPS = 36;
const DT = 1 / 60;

const makeArmRig = (overrides = {}) => makeRig({
  skeleton: ARM_SKELETON,
  bones: BONES,
  compliance: 0.00006,
  substeps: 8,
  seed: SEED,
  ...overrides,
});

const reach = (steps = STEPS) => Array.from({ length: steps }, (_, i) => {
  const s = i / (steps - 1);
  return { pose: {
    rightUpperArm: [0.1 * s, -0.3 * s, 0.9 * s],
    rightLowerArm: [0, -1.0 * s, 0],
  } };
});

const crossChest = (steps = STEPS) => Array.from({ length: steps }, (_, i) => {
  const s = i / (steps - 1);
  return { pose: {
    rightUpperArm: [0, -0.2 * s, 1.3 * s],
    rightLowerArm: [0, -1.6 * s, 0],
  } };
});

const cloneTrajectory = (trajectory) => JSON.parse(JSON.stringify(trajectory));

function noisyObservation(rig, truth, sigmaRad) {
  const observed = cloneTrajectory(simulate(rig, truth, DT, truth.length));
  const random = makePRNG(SEED);
  let sumSq = 0;
  let samples = 0;
  for (const frame of observed.frames) {
    for (const bone of BONES) {
      const axis = [random.normal(), random.normal(), random.normal()];
      const length = Math.hypot(...axis) || 1;
      const angle = Math.max(-3 * sigmaRad, Math.min(3 * sigmaRad, random.normal() * sigmaRad));
      frame.rel[bone] = q4.mul(frame.rel[bone], q4.axisAngle(axis.map((v) => v / length), angle));
      sumSq += angle * angle;
      samples++;
    }
  }
  return { observed, noiseRmsRad: Math.sqrt(sumSq / samples) };
}

function occludedObservation(rig, truth) {
  const observed = cloneTrajectory(simulate(rig, truth, DT, truth.length));
  let missingSamples = 0;
  for (let i = 0; i < observed.frames.length; i++) {
    // Keep knot frames 0/18/35 visible while masking one third of the samples.
    if (i % 3 !== 1) continue;
    delete observed.frames[i].rel.rightLowerArm;
    delete observed.frames[i].pos.rightLowerArm;
    missingSamples++;
  }
  return { observed, missingSamples };
}

function contactObservation(rig, truth) {
  const observed = cloneTrajectory(simulate(rig, truth, DT, truth.length));
  const withoutContact = simulate(makeArmRig(), truth, DT, truth.length);
  let maxPositionDeltaM = 0;
  for (let i = 0; i < observed.frames.length; i++) {
    for (const bone of BONES) {
      const a = observed.frames[i].pos[bone];
      const b = withoutContact.frames[i].pos[bone];
      maxPositionDeltaM = Math.max(maxPositionDeltaM, Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]));
    }
  }
  return { observed, maxPositionDeltaM };
}

// Handcrafted case definitions are deliberately small so every PR can run them in CI.
// Observations are rebuilt from fixed constants and a fixed seed; no fixture depends on
// wall-clock time, process state, Math.random, or an external package.
export function buildDataset() {
  const truth = reach();

  const noiseRig = makeArmRig();
  const noisy = noisyObservation(noiseRig, truth, 0.015);

  const occlusionRig = makeArmRig();
  const occluded = occludedObservation(occlusionRig, truth);

  const contactRig = makeArmRig({ profile: { selfCollision: true, bulk: 0.5 } });
  const contactTruth = crossChest();
  const contact = contactObservation(contactRig, contactTruth);

  return [
    {
      id: 'noise-0.015rad',
      category: 'noise',
      rig: noiseRig,
      truth,
      observed: noisy.observed,
      expectedFeasible: true,
      metadata: { sigmaRad: 0.015, noiseRmsRad: noisy.noiseRmsRad },
    },
    {
      id: 'occlusion-one-third',
      category: 'occlusion',
      rig: occlusionRig,
      truth,
      observed: occluded.observed,
      expectedFeasible: true,
      metadata: { missingSamples: occluded.missingSamples, totalSamples: STEPS * BONES.length },
    },
    {
      id: 'self-contact-cross-chest',
      category: 'contact',
      rig: contactRig,
      truth: contactTruth,
      observed: contact.observed,
      expectedFeasible: false,
      metadata: {
        selfCollision: true,
        bulk: 0.5,
        maxPositionDeltaM: contact.maxPositionDeltaM,
        expectedViolations: ['rom', 'torque'],
      },
    },
  ];
}

export function controlError(truth, estimate) {
  let sumSq = 0;
  let maxRad = 0;
  let samples = 0;
  for (let i = 0; i < truth.length; i++) {
    for (const bone of BONES) {
      const expectedEuler = truth[i].pose[bone];
      const actualEuler = estimate.control.frames[i].pose[bone];
      const expectedQ = qFromEulerXYZ(expectedEuler);
      const actualQ = qFromEulerXYZ(actualEuler);
      const error = qAngle(expectedQ, actualQ);
      sumSq += error * error;
      maxRad = Math.max(maxRad, error);
      samples++;
    }
  }
  return { rmsRad: Math.sqrt(sumSq / samples), maxRad };
}
