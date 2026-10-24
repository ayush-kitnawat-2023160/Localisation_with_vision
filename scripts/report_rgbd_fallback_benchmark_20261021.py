#!/usr/bin/env python3
"""Consolidate measured GPS-/LiDAR-free RGB-D camera odometry results."""
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np


ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/rgbd_fallback_benchmark_20261021'
PREV=ROOT/'outputs/zedx_rgbd_imu_fallback_20261020'


def read_json(path):return json.loads(path.read_text())


def fmt(v):return 'unavailable' if v is None else f'{v:.2f}'


def main():
    baseline={
        'localization.bag':read_json(PREV/'localization_gyro_fused_v2/metrics.json'),
        'localization_from2.bag':read_json(PREV/'localization_from2_gyro_fused_v2/metrics.json'),
        'localization_3.bag':read_json(PREV/'localization_3_gyro_dense_v3/metrics.json')}
    no_gyro=read_json(OUT/'dense_rgbd_visual_only_ablation/metrics.json')
    geometric={
        'localization_3.bag':read_json(OUT/'depth3d_full_local3_20261021/metrics.json'),
        'localization_from2.bag':read_json(OUT/'depth3d_full_from2_20261021/metrics.json')}
    ros=OUT/'rgbd_gyro_localization_3_ros1_profiled_20261021'
    cpu=read_json(ros/'rgbd_process_resources.json')['summary']
    ros_events=read_json(ros/'pose_events.json')
    latest=read_json(PREV/'localization_3_parallax_status_15s_ros1_20261021/events.json')
    measure=[]
    cats=Counter()
    for event in latest['statuses']:
        field={part.split('=')[0]:part.split('=')[1]
               for part in event['text'].split() if '=' in part}
        try:p=float(field['parallax_deg'])
        except (KeyError,ValueError):continue
        if np.isfinite(p):
            measure.append(p)
            cats[field['parallax_observability']]+=1

    # Baseline offline processing timings are for process() only, not the
    # full ROS1 callback, stereo-depth SDK, bag decoding, or TF consumers.
    rows=list(csv.DictReader(open(PREV/'localization_3_gyro_dense_v3/trajectory.csv')))
    core_ms=np.array([float(x['processing_ms']) for x in rows])
    gyro=baseline['localization_3.bag']
    rmse_improvement=(1-gyro['outage_position_error_vs_lio_proxy_rmse_m']
                     /no_gyro['outage_position_error_vs_lio_proxy_rmse_m'])*100
    end_improvement=(1-gyro['outage_end_error_to_lio_proxy_m']
                     /no_gyro['outage_end_error_to_lio_proxy_m'])*100
    bins=[]
    for left,right in [(0,10),(10,20),(20,30),(30,40),(40,50),(50,65)]:
        data=list(csv.DictReader(open(OUT/'depth3d_full_local3_20261021/frames.csv')))
        values=[float(x['rigid_parallax_deg']) for x in data
                if left<=float(x['sim_sec'])<right and x['rigid_parallax_deg']]
        if values:
            bins.append({'window_sec':[left,right],
                         'valid_parallax_frames':len(values),
                         'median_parallax_deg':float(np.median(values))})

    summary={
        'status':'OFFLINE_CANDIDATE_ONLY_NOT_ODOMETRY_AUTHORITY',
        'processor':'Intel Core i5-1340P, 16 logical CPU threads',
        'total_ZED_SDK_depth_GPU_cost_measured':False,
        'estimator_is_full_tightly_coupled_VIO':False,
        'baseline_bags':{k:{'frames':v['frames_processed'],
                'accepted':v['accepted_vo_frames'],
                'acceptance_fraction':v['vision_good_fraction'],
                'end_disagreement_m':v['outage_end_error_to_lio_proxy_m'],
                'rmse_disagreement_m':v['outage_position_error_vs_lio_proxy_rmse_m']}
            for k,v in baseline.items()},
        'dense_baseline_ros1_compute':{
            'poses':ros_events['candidate_count'],
            'mean_cpu_cores_including_startup':cpu['mean_cpu_cores'],
            'mean_cpu_percent_one_core':cpu['mean_cpu_percent_of_one_core'],
            'process_cpu_seconds':cpu['process_cpu_seconds_last_sample'],
            'peak_rss_mib':cpu['peak_rss_mib'],
            'pose_authority_leaks':ros_events['forbidden_pose_count'],
            'core_process_median_ms':float(np.median(core_ms)),
            'core_process_p95_ms':float(np.percentile(core_ms,95))},
        'gyro_ablation_dense_localization3':{
            'gyro_35s_end_error_m':gyro['outage_end_error_to_lio_proxy_m'],
            'no_gyro_35s_end_error_m':no_gyro['outage_end_error_to_lio_proxy_m'],
            'gyro_35s_rmse_m':gyro['outage_position_error_vs_lio_proxy_rmse_m'],
            'no_gyro_35s_rmse_m':no_gyro['outage_position_error_vs_lio_proxy_rmse_m'],
            'relative_rmse_improvement_percent':rmse_improvement,
            'relative_end_improvement_percent':end_improvement},
        'geometric_alternative':{
            k:{method:{'accepted':v[method]['accepted_frames'],
                          'total':v[method]['processed_frames'],
                          'outage_rmse_m':v[method]['outage_rmse_m'],
                          'outage_end_m':v[method]['outage_endpoint_disagreement_m'],
                          'median_per_frame_ms':v[method]['processing_ms_median'],
                          'p95_per_frame_ms':v[method]['processing_ms_p95']}
                for method in ('baseline_pnp_gyro','alternative_depth3d_ransac_gyro')}
            for k,v in geometric.items()},
        'parallax_by_time':bins,
        '15s_ROS1_parallax_readonly_test':{
            'published_odometry_candidates':latest['candidate_count'],
            'unexpected_pose_publications':latest['forbidden_pose_count'],
            'diagnostic_messages_with_finite_parallax':len(measure),
            'low_parallax_observations':cats['LOW_PARALLAX_OR_STATIONARY'],
            'measurable_parallax_observations':cats['MEASURABLE']},
        'fundamental_limitations':[
            'LiDAR-only FAST-LIVO2 comparison is not surveyed truth.',
            'Camera/body mounting disagrees between original ZED driver TF and inherited LiDAR YAML by 0.55 m, 12.77 deg.',
            'CPU/RAM measurements exclude ZED stereo SDK GPU depth generation and the real cart hardware.',
            'Parallax telemetry comes from gyro-de-rotated tracked rays and is descriptive; it does not gate odometry.',
            'Fallback has no independent wheel-speed/brake interlock or automatic odometry authority.']}
    (OUT/'FINAL_RGBD_FALLBACK_METRICS_20261021.json').write_text(json.dumps(summary,indent=2)+'\n')

    lines=[
        '# LiDAR-free ZED X RGB-D + gyro fallback: October 21, 2026',
        '', '## Conclusion', '',
        '**Keep depth-assisted KLT + PnP/RANSAC + short-interval ZED X gyro as the experimental local odometry fallback.** It is *not* tightly coupled VIO; no motion authority or `/pose` is enabled.',
        '**Remove/avoid 3D-to-3D alignment as default:** despite using depth at both frames it is slower and less accurate against the same LiDAR-only odometry proxy, especially on the difficult route.',
        '', '## CPU / RAM (measured on Intel i5-1340P laptop; NOT Jetson Orin)', '',
        '| Metric | Value |', '|---|---:|',
        f"| ROS1 candidate messages, difficult 64s bag | {ros_events['candidate_count']} |",
        f"| Mean estimator CPU occupancy | **{cpu['mean_cpu_cores']:.3f} core** ({cpu['mean_cpu_percent_of_one_core']:.1f}% of one logical core) |",
        f"| Estimator CPU-seconds including startup/shutdown | **{cpu['process_cpu_seconds_last_sample']:.2f}s** |",
        f"| Peak estimator resident RAM | **{cpu['peak_rss_mib']:.1f} MiB** |",
        f"| Core image-to-motion processing only | {np.median(core_ms):.2f}ms median, {np.percentile(core_ms,95):.2f}ms p95 |",
        f"| Unauthorized `/pose` messages | **{ros_events['forbidden_pose_count']}** |",
        '', 'Note: GPU computation for ZED stereo depth, ROS image decoding/copying, camera SDK overhead, and concurrent FAST-LIVO2 processing are **not** included in these process measurements. Our real target has an Orin and a NUC; no Orin-specific CPU/GPU number has been measured.',
        '', '## Accuracy relative to recorded LiDAR-only local odometry (NOT surveyed ground truth)', '',
        '| Recording | Accepted RGB-D+gyro estimates | 20–55s proxy RMSE | End-of-window disagreement |',
        '|---|---:|---:|---:|']
    for name,v in baseline.items():
        lines.append(f"| `{name}` | {v['accepted_vo_frames']}/{v['frames_processed']} ({v['vision_good_fraction']*100:.1f}%) | {v['outage_position_error_vs_lio_proxy_rmse_m']:.2f} m | {v['outage_end_error_to_lio_proxy_m']:.2f} m |")
    lines += ['',
      'Each camera trajectory is aligned to the reference at the simulated LiDAR-loss onset; measured disagreement is not absolute map localization accuracy. Physical camera-to-body extrinsics remain unverified. An estimator can agree with drifting LiDAR without being correct.',
      '', '## Gyro ablation: same 807 frames on `localization_3.bag`', '',
      '| Same RGB-D KLT+PnP | 35s RMSE (m) | Endpoint (m) |', '|---|---:|---:|',
      f"| **With ZED X gyro** | **{gyro['outage_position_error_vs_lio_proxy_rmse_m']:.2f}** | **{gyro['outage_end_error_to_lio_proxy_m']:.2f}** |",
      f"| Without gyro (visual-only) | {no_gyro['outage_position_error_vs_lio_proxy_rmse_m']:.2f} | {no_gyro['outage_end_error_to_lio_proxy_m']:.2f} |",
      '', f'Gyro improves disagreement RMSE by **{rmse_improvement:.1f}%** and endpoint disagreement by **{end_improvement:.1f}%** in this recording, without improving the feature-tracking acceptance count.',
      '', '## Genuine alternative tested: 3D-to-3D depth Kabsch + RANSAC + gyro', '',
      'Compared at **identical camera frames** (0.13 s sampling) using the same LiDAR reference and inherited body transform. 3D-to-3D uses depth at *both* frames, unlike PnP which needs depth only for 3D points in the first frame.',
      '', '| Route | Solver | Good frames | 20–55s RMSE | Endpoint | Median motion processing |',
      '|---|---|---:|---:|---:|---:|']
    for name,j in geometric.items():
        for method,short in [('baseline_pnp_gyro','PnP+gyro'),('alternative_depth3d_ransac_gyro','3D↔3D RANSAC+gyro')]:
            v=j[method]
            lines.append(f"| {name} | {short} | {v['accepted_frames']}/{v['processed_frames']} | {v['outage_rmse_m']:.2f} m | {v['outage_endpoint_disagreement_m']:.2f} m | {v['processing_ms_median']:.2f} ms |")
    lines += ['',
       'On `localization_3.bag`, 3D↔3D depth alignment suffers many more failed geometric updates and accumulates much more drift; on `localization_from2.bag` it tracks most frames but remains worse and slower. Reject this method as the *default*. Depth errors/holes can affect both views, while PnP requires depth only from one view.',
       '', '## Parallax — useful confidence cue, not a complete odometry method', '',
       'Gyroscope-rotation-compensated bearing parallax is measured from tracked points in accepted 3D↔3D frames. Low parallax can mean the cart is stopped, moving very slowly, or tracking distant scenery; it is not proof of sensor failure.',
       '', '| Bag seconds | Median parallax |', '|---|---:|']
    for v in bins:
        lines.append(f"| {v['window_sec'][0]}–{v['window_sec'][1]} | {v['median_parallax_deg']:.3f}° |")
    lines += ['',
       f"The additional read-only ROS1 health test emitted **{latest['candidate_count']}** candidate poses and **zero `/pose`** while classifying **{cats['LOW_PARALLAX_OR_STATIONARY']}** frames as low-parallax. It deliberately did NOT suppress their odometry: hard-gating low parallax would break stationary/slow-moving navigation.",
       'The PnP estimator now emits `parallax_deg=... parallax_observability=...` in `/fast_livo2/fallback/camera_imu_status` for *diagnostics only*. The motion estimator and trusted-output rules are unchanged.',
       '', '## Recommended improvements in priority order', '',
       '1. **Get true end-to-end compute on the Orin**: measure ZED SDK depth-generation GPU, camera rectification, node process CPU/RAM, message age, and network transfer latency to the NUC. Existing 0.55-core figure excludes SDK GPU.',
       '2. **Calibrate the physical camera-to-body and camera↔IMU transforms/timing.** Driver `base_link` is currently assumed, not measured. Record moving + stationary calibration sequences; verify gyro bias and depth uncertainty.',
       '3. **Depth confidence and feature distribution**: use multi-pixel median depth/consistency instead of single-pixel depth at tracked corner; select spatially distributed features on curbs/buildings/walls, restrict unstable foliage, moving people/cars and road-only texture. Weight PnP residuals with depth-dependent covariance.',
       '4. **Adaptive observability health, NOT hard parallax rejection**: integrate 1–2s windows of gyro-de-rotated parallax, near/far feature counts, spatial spread, inlier fraction, reprojection residual and independent speed; distinguish stopped/creeping from bad tracking. If visually unobservable while moving, request stop.',
       '5. **Temporally stable keyframe tracking**: maintain longer-lived KLT tracks across multiple frames and estimate keyframe-to-current motion rather than only successive noisy depth samples. Reinitialization must be epoch-aware after image gaps.',
       '6. **Benchmark independent visual odometry** on the same raw data: (a) ZED SDK positional tracking, ideally with SVO + IMU on Orin, (b) RTAB-Map RGB-D `rgbd_odometry` with the ROS1 Noetic `master` branch, (c) OpenCV RGB-D direct photometric alignment if hardware allows. Neither (a) nor (b) is installed or benchmarked in this run.',
       '7. **Verify failover health and rejoin**: only candidate messages until independently proven LiDAR outage and physical controller stop; do not replace FAST-LIVO2 `/aft_mapped_to_init` or publish navigation `/pose` automatically. On LiDAR recovery require a supervised map/odom epoch handoff.',
       '', '## Reproducibility (No OpenVINS)', '',
       '```bash', 'cd ~/Desktop/ALIVE_N/FailSafe_FastLivo2',
       'docker run --rm --entrypoint bash -v "$PWD:/repo:ro" fast_livo2:dev -lc "source /opt/ros/noetic/setup.bash; PYTHONPATH=/repo/scripts python3 /repo/scripts/test_zedx_rgbd_imu_odom_core.py"',
       'python3 scripts/report_rgbd_fallback_benchmark_20261021.py',
       '# Offline, candidate-only replay:',
       'PLAY_DURATION=15 RUN=<unique_name> ./scripts/run_zedx_camera_imu_no_lidar_rosbag.sh localization_3',
       '```',
       '', 'All experimental OpenVINS source/build/results and its dedicated scripts were removed at user request. Existing FAST-LIVO2 maps and production modules were preserved.']
    (OUT/'FINAL_RGBD_FALLBACK_REPORT_20261021.md').write_text('\n'.join(lines)+'\n')
    print('REPORT_DONE',OUT/'FINAL_RGBD_FALLBACK_REPORT_20261021.md',
          'gyro_gain_rmse',round(rmse_improvement,2),
          'baseline_cpu_percent_one_core',round(cpu['mean_cpu_percent_of_one_core'],2))


if __name__=='__main__':main()
