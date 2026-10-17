# ALIVE FAST-LIVO2 + Lanelet initialization

This is the non-FLOAM localization path. FAST-LIVO2 is only a local,
camera/LiDAR/IMU 6-DoF odometry estimator. `fast_livo2_lanelet_bridge.py`
collects robust GNSS samples, projects them using the same UTM zone-43 origin
given to `lanelet2_interface_ros`, scores Lanelet centerline candidates, and
publishes one map anchor:

```text
map -> fastlivo_odom -> base_link
       static-ish       FAST-LIVO2 6DoF
```

The bridge publishes `/pose`, `/odom`, `/fast_livo2/global_trajectory`,
`/fast_livo2/gps_map_pose`, `/fast_livo2/matched_lanelet`, and
`/fast_livo2/localization_status`. It never uses a PCD for registration and
never applies a GPS update to FAST-LIVO2 after initialization. The launch can
publish `maps/full_campus.pcd` on `/apriori_map` for RViz visualization only.

RViz's **2D Pose Estimate** tool publishes `/initialpose`. A click-drag on the
PCD map creates the `map -> fastlivo_odom` anchor from the current local
FAST-LIVO2 odometry, so manual initialization is available when GNSS and the
Lanelet map do not overlap.

The UTM datum and all initialization thresholds live in
`config/lanelet_bridge.yaml`. The default map is a checked-in copy of the
ALIVE map-publisher Lanelet OSM. The road-plane anchor is
`T_map_fastlivo = T_map_base_initial * inverse(T_fastlivo_base_initial)`;
therefore later roll, pitch, z, and relative yaw remain FAST-LIVO2 output.

For visual validation, start the existing ALIVE `lanelet_map_rviz.launch` with
the same origin, then use `rviz/fast_livo2_lanelet_localization.rviz` (or add
its displays to that RViz instance).
