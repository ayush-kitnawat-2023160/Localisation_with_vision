# Optional visual smoothing preview (NOT the tested release-candidate image)

These files are the independently built, ROS1-synthetic-tested, 230ms-lag
fixed-window pose filter used only for side-by-side RViz inspection.

Separate Docker image:
    fast_livo2_failsafe:visual-fixed-lag-shadow-dev-20261016

The single-image ROS replay in this release DID NOT use the shadow smoother
for its published navigation pose; source/core metrics in EIGHT_FINAL_METRICS
refer to the actual Step-3 global pose without the optional sidecar.
This folder is OUTSIDE the frozen source snapshot code/ on purpose.

ROS preview:
    /fast_livo2/step3_global_pose_smoothed_preview
Numeric-only warning:
    /fast_livo2/step3_visual_smoother_numeric_unsafe

NEVER remap the vehicle planner, controller, or DBW to the preview topic.
A 230ms delayed pose and smoothness in XY do NOT prove accurate localization.
Failures above 0.85m remain, and this pipeline has not met the GT target.

Previous before/after offline diagnostics are in:
    results/FIXED_LAG_SMOOTHING_DEV_20261016/
