# ALiVE FAST-LIVO2 failsafe — current consolidated development release (2026-10-16)

## Where the CLEAN codebase is
  release/ALIVE_FASTLIVO2_OFFLINE_RC_20261016/code/

The source was extracted from the **specific immutable Docker image** actually
used for eight serial 1x ROS bag replays, instead of copying a working directory
with partially integrated C++ experimental edits. The files mounted at runtime
(launch, configs, sensor bridge) were overlaid into the clean source snapshot.
Use the release SOURCE/sha256 manifest to confirm integrity.

## Where the NEW final 1x comparisons are
  results/FINAL_SINGLE_IMAGE_RC_20261016/

Once the eight actual replay simulations complete, use
  scripts/finalize_single_build_rc_20261016.py
which refuses incomplete runs and produces a verified table, 8-map-plot panel,
8 error-timeline panel, 8 jump panel and clean source+report archive.

The main map proxy remains the original main bag loop-closed reference.
The September 18 reference was rebuilt on 16 Oct 2026 from LiDAR/PCD data:
  results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/

The outdated September 8/Oct 8 baseline contains a geometrically false
right-side branch and must not be used. Older Oct9 already corrected that
corner; the new Oct16 reference is versioned and now the default runner input.

## What's NOT claimed
- This is **not** a vehicle safety certificate or an autonomous-ready release.
- The reference paths are diagnostic same-map proxies, not independent RTK/GT.
- Historical best-of-8 results were made using different experimental builds;
  the one-build replay suite is the authoritative release-candidate validation.
- The current launch is for ROSBAG testing: use_sim_time=true and taskset CPU
  affinity / core numbering are laptop-specific. Not a cart NUC deploy launch.
- A ROS stop-required Boolean isn't verified at the golf cart actual VAT/DBW
  brake and motor command arbiter.
- The separate smoothing sidecar produces a **shadow-only** preview at 230ms
  delay; it does not replace the trusted navigation state or prove accuracy.

## Safe cleanup policy
Original historical plots, sensor raw bag files, registrations, and reference
files were NOT deleted, moved or rewritten. They are research audit evidence.
The release folder is the cleaned usable subset, separate from older trials.
Never run a destructive Docker prune on the cart ROS Noetic/Jazzy hosts.
