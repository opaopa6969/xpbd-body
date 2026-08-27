import { performance } from 'node:perf_hooks';
import { buildDataset, controlError, DATASET_VERSION, SEED } from './dataset.mjs';
import { estimateControl } from '../inverse.js';

const LIMITS = Object.freeze({
  maxResidualRmsRad: 0.08,
  maxControlErrorRmsRad: 0.12,
  maxFalseFeasibleRate: 0,
  minContactPositionDeltaM: 0.001,
  maxTotalDurationMs: 30_000,
});

const stableCase = ({ id, category, truth, observed, expectedFeasible, metadata }) => ({
  id,
  category,
  truth,
  observed,
  expectedFeasible,
  metadata,
});

const stableResult = ({ durationMs, ...result }) => result;

const fingerprint = (value) => {
  let hash = 0x811c9dc5;
  for (const byte of new TextEncoder().encode(JSON.stringify(value))) {
    hash ^= byte;
    hash = Math.imul(hash, 0x01000193);
  }
  return (hash >>> 0).toString(16).padStart(8, '0');
};

const firstDataset = buildDataset();
const secondDataset = buildDataset();
const datasetFingerprint = fingerprint(firstDataset.map(stableCase));
const deterministicDataset = datasetFingerprint === fingerprint(secondDataset.map(stableCase));

const started = performance.now();
const evaluate = (fixture) => {
  const caseStarted = performance.now();
  const estimate = estimateControl(fixture.rig, fixture.observed, {
    bones: fixture.observed.bones,
    knots: 3,
    iters: 4,
    seed: SEED,
  });
  const error = controlError(fixture.truth, estimate);
  return {
    id: fixture.id,
    category: fixture.category,
    expectedFeasible: fixture.expectedFeasible,
    feasible: estimate.feasible,
    residualRmsRad: estimate.residual.rmsAngle,
    residualMaxRad: estimate.residual.maxAngle,
    controlErrorRmsRad: error.rmsRad,
    controlErrorMaxRad: error.maxRad,
    violationTypes: [...new Set(estimate.violations.map((violation) => violation.type))].sort(),
    evals: estimate.evals,
    durationMs: performance.now() - caseStarted,
    metadata: fixture.metadata,
  };
};
const results = firstDataset.map(evaluate);
const repeatedResults = secondDataset.map(evaluate);
const metricsFingerprint = fingerprint(results.map(stableResult));
const deterministicMetrics = metricsFingerprint === fingerprint(repeatedResults.map(stableResult));
const totalDurationMs = performance.now() - started;

const negativeResults = results.filter((result) => !result.expectedFeasible);
const falseFeasibleCount = negativeResults.filter((result) => result.feasible).length;
const falseFeasibleRate = falseFeasibleCount / Math.max(1, negativeResults.length);
const failures = [];

if (!deterministicDataset) failures.push('dataset fingerprint changed between identical builds');
if (!deterministicMetrics) failures.push('evaluation metrics changed between identical runs');
for (const result of results) {
  if (result.feasible !== result.expectedFeasible) failures.push(`${result.id}: feasible=${result.feasible}, expected=${result.expectedFeasible}`);
  if (result.expectedFeasible && result.residualRmsRad > LIMITS.maxResidualRmsRad) failures.push(`${result.id}: residual rms ${result.residualRmsRad} > ${LIMITS.maxResidualRmsRad}`);
  if (result.expectedFeasible && result.controlErrorRmsRad > LIMITS.maxControlErrorRmsRad) failures.push(`${result.id}: control error rms ${result.controlErrorRmsRad} > ${LIMITS.maxControlErrorRmsRad}`);
}
const contact = results.find((result) => result.category === 'contact');
if (!contact || contact.metadata.maxPositionDeltaM < LIMITS.minContactPositionDeltaM) failures.push('contact fixture did not activate the self-contact constraint');
for (const expected of contact ? contact.metadata.expectedViolations : []) {
  if (!contact.violationTypes.includes(expected)) failures.push(`contact fixture did not report expected ${expected} violation`);
}
if (falseFeasibleRate > LIMITS.maxFalseFeasibleRate) failures.push(`false feasible rate ${falseFeasibleRate} > ${LIMITS.maxFalseFeasibleRate}`);
if (totalDurationMs > LIMITS.maxTotalDurationMs) failures.push(`total duration ${totalDurationMs}ms > ${LIMITS.maxTotalDurationMs}ms`);

const report = {
  schemaVersion: DATASET_VERSION,
  seed: SEED,
  datasetFingerprint,
  deterministicDataset,
  metricsFingerprint,
  deterministicMetrics,
  limits: LIMITS,
  falseFeasibleRate,
  totalDurationMs,
  results,
  passed: failures.length === 0,
  failures,
};

console.log(JSON.stringify(report, null, 2));
if (failures.length) process.exitCode = 1;
