# New ZED X/VLP-16 maps — first mapping pass (19 October 2026)

> Update: this note records the earlier partial `campus_full_raw_v2` pass. The later complete LiDAR-only map, held-out relocalization results, and separate loop-closure candidate are documented in [LIDAR_GLOBAL_LOCALIZATION_VALIDATION_20261019.md](LIDAR_GLOBAL_LOCALIZATION_VALIDATION_20261019.md).

## Safety and provenance
Only `rosbag/looooo/localization_full.bag` was consumed. None of the other three bags have been evaluated. The first pass produced a **RAW local FAST-LIVO2 odometry map**, NOT a loop-closed, surveyed, Lanelet-aligned or driving-qualified map. Do not enable `/pose` as planning authority from this build.

Source observations: input duration ~226 s, output trajectory duration ~127.8 s, 1,197 trajectory samples. Repeated `IMU and LiDAR not synced!` messages appear in `outputs/full_20261019/launch.log`. This first replay did not fully process the source bag. The current map must be REBUILT after fixing replay pacing, buffering, and LiDAR/IMU time association. Its Z span of ~19.8 m also needs investigation. The previous saved-map and ROS bags are not mixed into the generated map.

## Current generated files

`maps/campus_full_raw_v2/`:

- `pcd/0.pcd ...` — 165 LiDAR-frame keyframes, compatible in layout with upstream FAST-LOCALIZATION.
- `pose.json` — matching T_map_lidar poses (tx ty tz qw qx qy qz).
- `global_map.pcd` — raw LiDAR cloud preview (226,203 points).
- `keyframes.csv`, `gps_measurements.csv` — location metadata, GPS only coarse/untrusted.
- `visual/images/`, `visual/landmarks/`, `visual/keyframes.csv` — 122 ZED X RGB-D visual keyframes with ORB descriptors and anchored 3D landmarks.
- `visual/landmarks_preview.pcd` — 23,069 visual landmark preview points.
- `cache/scan_context.bin`, `cache/local_submaps/` — precomputed local matcher cache, built from current raw keyframes.
- `quality.json`, `map_overview.png`, `visual_keyframes_contact_sheet.jpg`, `visual_neighbor_probe.json` — diagnostic summaries.

**Current results:** median neighboring LiDAR scan overlap at 0.8 m = ~0.739 (structural check only); visual within-map adjacent-feature matching = 10/38 (26.3%), FAILED provisional 70% target. Neither proves held-out global accuracy or valid loop closure.

## Pipeline code added

- `scripts/build_fresh_campus_maps.py` — bag + FAST-LIVO2 odometry -> LiDAR PCD/keyframe pose DB + GPS metadata + visual image/descriptor/metric landmark DB.
- `scripts/build_step3_cache.py` — Scan Context and local-submap cache for the existing Step3 matcher.
- `scripts/inspect_fresh_maps.py`, `scripts/probe_visual_relocalization.py` — quality and mapping-bag consistency diagnostics.
- `scripts/fresh_visual_global_localizer.py` — bounded local-keyframe ORB + PnP with GPS as coarse fallback.
- `scripts/fresh_pose_authority.py` — single map-to-odom alignment observer. `/pose` publishing remains **OFF** unless explicitly enabled.
- `launch/fresh_multimodal_global_rosbag.launch` — FAST-LIVO2 + existing Step3 Scan Context/GICP + visual match candidate + pose authority (offline development only).

**Scope limitations:** This LiDAR matcher reuses the existing Step3 implementation, not an extracted/ported FAST-LOCALIZATION library. No HBA/pose-graph optimization, no XFeat integration, and no KLT/ESIKF delayed-state updates have yet been implemented. Visual ORB+PnP is a diagnostic baseline. The current runtime launch is statically parsed, not validated against a held-out bag.

## Reproducible analysis and viewing

Run map QA on the current map with an environment containing numpy/scipy/matplotlib/PIL/cv2:

    python3 scripts/inspect_fresh_maps.py --map maps/campus_full_raw_v2
    python3 scripts/probe_visual_relocalization.py --visual maps/campus_full_raw_v2/visual

Quick static images:

    xdg-open maps/campus_full_raw_v2/map_overview.png
    xdg-open maps/campus_full_raw_v2/visual_keyframes_contact_sheet.jpg

3D maps in RViz (ROS Noetic/catkin source):

    source /opt/ros/noetic/setup.bash
    source /path/to/catkin_ws/devel/setup.bash
    roslaunch fast_livo2 view_fresh_maps.launch rviz:=true lidar_pcd:=/absolute/path/to/FailSafe_FastLivo2/maps/campus_full_raw_v2/global_map.pcd visual_pcd:=/absolute/path/to/FailSafe_FastLivo2/maps/campus_full_raw_v2/visual/landmarks_preview.pcd

Set Fixed Frame=`map`, add PointCloud2 displays `/fresh_maps/lidar` and `/fresh_maps/visual`. Display Type RGB8 for visual RGB PCD. For direct PCD visualization if PCL tools are installed:

    pcl_viewer maps/campus_full_raw_v2/global_map.pcd
    pcl_viewer maps/campus_full_raw_v2/visual/landmarks_preview.pcd

## Next prerequisites before using this map

1. Reprocess the full 226-second bag at an ingest rate sustainable by the mapper; audit LiDAR point-time units and IMU/LiDAR ordering. Preserve original sensor header timestamps and record pose for the entire bag.
2. Inspect physical LiDAR↔IMU and LiDAR↔camera frame chain. The current calibration profile inherits older IMU/base assumptions.
3. Add loop detection, pose-graph optimization and optionally HBA; rebuild BOTH LiDAR and visual map coordinates from corrected poses.
4. Re-run quality checks and visual matching tests until there are enough correct verified pairs, and check relative/global geometry over repeated passes.
5. Only then start held-out testing with `localization.bag`, `localization_from2.bag`, `localization_3.bag`; do not treat the in-map visual probe as independent accuracy.
