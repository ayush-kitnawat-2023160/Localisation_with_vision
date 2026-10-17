# RGB-D visual-seed correction cap: two-bag evaluation — 2026-10-16

**Decision: do not promote any candidate, even the 0.35m cap.**

All figures are differences against bag-specific loop-closed proxy trajectories,
**not surveyed ground truth**. No map or visual keyframe database was fit to September 18.

| Recording / version | 3D RMSE during LiDAR loss | Recovery 20s RMSE | Worst pose step |
|---|---:|---:|---:|
| Main original curated | 2.170m | 1.317m | 2.276m |
| Main: 0.35m max GICP shift | 1.928m | 1.491m | 2.055m |
| Main: 0.15m max GICP shift | 2.599m | 1.639m | 2.408m |
| Sep18 curated best | 3.670m | 1.863m | 1.385m |
| Sep18: unchanged 0.35m setting | 3.885m | 1.425m | 1.713m |

## Interpreting the result

- The 0.35m variant improves the **main-bag outage** by 11.2% compared
  with the frozen Oct13 best, consistent with bad ICP minima sliding
  position away from a useful visual-place seed. However, it worsens
  recovery and has many 0.85m+ published pose steps.
- Tightening to 0.15m loses useful geometric correction: it regresses
  both outage and recovery on the main bag. There is no safe monotonic
  gain to obtain just by increasing/decreasing this parameter.
- With the **unchanged** 0.35m setting, the Sep18 outage becomes
  worse than the archived best despite some better recovery. This
  rules out claiming cross-bag generalisation.
- **Important confound:** the archived Sep18 curated baseline is the
  older Oct7 development image, while the new run uses the Oct13
  frozen base. Therefore the September difference is a model-version
  **plus** parameter comparison, not a controlled isolated cap effect.
- Spatial accuracy, geometry observability and heading consistency
  must be fixed at the source using distinctive static landmarks
  and independent registration constraints. The map and image-frame
  transforms are numerically orthonormal; physical extrinsics remain
  unverified to sub-0.5m field tolerance.

## Deployment and safety

- Retained curated best runs: `results/BEST_SO_FAR_20261015/` (all 8 intact).
- No current candidate meets <=0.5m outage RMSE or jump/dropout gates.
- None has a hardware-validated localizer-untrusted -> planner/DBW stop.
- Offline velocity spikes and correction steps must not be hidden by
  unverified interpolation or smoothing before the cart is tested.
- Leave the live cart code untouched; don't promote or deploy.

## Artifacts

- Full JSON audit: `reports/DEPTH_SEED_LIMIT_CROSS_BAG_VALIDATION_20261016.json`
- New trials and plots: `step3_global/depth_bounded_seed_trial_20261016/`
- Reproducibility: original bag input, exact 1x replay, fixed model,
  original map and camera calibration, original Sep18 reference only
  during offline scoring.
