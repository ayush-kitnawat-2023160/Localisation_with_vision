#!/usr/bin/env python3
"""Consolidate two GPS-/LiDAR-free offline KLT odometry evaluations.

Measured reference differences are against another odometry system, not
surveyed ground truth. Estimator internals are isolated from ROS /pose.
"""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/rgbd_fallback_benchmark_20261021'


def runpath(bag,method):
    return OUT/f'klt_vs_eskf_individual_{bag}_{method}_20261021'


def data_for(bag,method):
    d=runpath(bag,method)
    j=json.loads((d/'metrics.json').read_text())
    m=j['methods'][method]
    return j,m


def quality_percent(a,b):return (b-a)/a*100. if a else None
def format_m(row,key):return "No trusted coverage" if row.get(key) is None else "%.3f m"%row[key]


def main():
    rows={}
    for bag in ('localization_3','localization_from2'):
        rows[bag]={}
        for method in ('baseline','keyframe_eskf'):
            j,m=data_for(bag,method)
            rows[bag][method]={
              'frames_processed':m['frame_count'],
              'trusted_visual_updates':m['accepted_count'],
              'trusted_fraction':m['accepted_fraction'],
              'keyframes':m['keyframe_count'],
              'estimation_wall_seconds_excluding_reference':j['estimation_wall_sec_excluding_lidar_reference'],
              'offline_process_cpu_seconds_excluding_reference':j['process_cpu_sec_excluding_reference'],
              'offline_average_cpu_cores_including_bag_decoding':j['average_process_cpu_cores'],
              'peak_process_ram_mib':j['peak_rss_mib'],
              'mean_image_conversion_seconds':j['cpu_times_by_component_sec']['image_convert_sec'],
              'visual_processing_seconds':j['cpu_times_by_component_sec']['baseline_image_sec'] if method=='baseline' else j['cpu_times_by_component_sec']['novel_image_sec'],
              'inertial_processing_seconds':j['cpu_times_by_component_sec']['baseline_imu_sec'] if method=='baseline' else j['cpu_times_by_component_sec']['novel_imu_sec'],
              'visual_processing_median_ms':m['vision_frame_process_ms_median'],
              'visual_processing_p95_ms':m['vision_frame_process_ms_p95'],
              'status_counts':m['status_counts'],
              'outage_relative_lidar_odom_disagreement':m['relative_lidar_proxy'],
              'camera_imu_calibration_physically_verified':False,
              'vehicle_odom_authority':False,
            }
            if method=='keyframe_eskf':
                rows[bag][method].update({
                  'IMU_propagation_steps':m['imu_updates'],
                  'initial_stationary_gyro_mean':m['IMU_initial_mean_gyro_rps'],
                  'initial_stationary_accel_std':m['IMU_initial_accel_std_mps2'],
                  'visual_innovation_NIS_p95':m['innovation_NIS_p95'],
                  'pose_sigma_median_m_UNCALIBRATED':m['reported_pos_sigma_median'],
                  'pose_sigma_max_m_UNCALIBRATED':m['reported_pos_sigma_max'],
                  'filter_numerically_finite_and_PSD_final':m['covariance_psd_checked_final'],
                  'last_odom_state':m['last_valid_state'],
                })

    metrics={'hardware':'Intel i5-1340P laptop, 16 logical CPUs',
      'sensor_data':'ZED X left RGB 960x600, registered 32FC1 stereo depth and ZED IMU; baseline and proposed use identical 0.065s sampled image pairs at 0.65x resolution',
      'sensor_depth_generation_gpu_load_measured':False,
      'benchmark_environment':'Offline ROS1 Noetic Docker Python replay; not Jetson Orin live camera; rosbag decoding included in whole-process CPU',
      'short_visual_frontend_pipeline':'Existing KLT, one-step RGB-D 3D-2D PnP with 35% short-interval gyro blended rotation, NO accelerometer',
      'new_visual_frontend_pipeline':'Persistent KLT 3D tracks against RGB-D keyframes and 3D-2D relative PnP; 21D error-state with 15D inertial nominal, 6D cloned keyframe pose, IMU accel+gyro preintegration and Joseph covariance update',
      'new_estimator_is_feature_level_tightly_coupled_VIO':False,
      'new_estimator_is_loosely_coupled_relative_pose_ESKF':True,
      'imu_sensor_noise_model_independently_calibrated':False,
      'camera_imu_and_camera_body_extrinsics_physically_surveyed':False,
      'LiDAR_reference_is_surveyed_ground_truth':False,
      'production_vehicle_pose_or_state_modified':False,
      'outage_window_seconds':[20,55],
      'results':rows,
      'deployment_recommendation':'NO_GO for new ESKF as local odom authority; keep existing KLT + gyro and improve low-texture keyframe recovery, covariance calibration and independent wheel-speed/brake safety checks'}
    path=OUT/'FINAL_KLT_KEYFRAME_ESKF_COMPARE_20261021.json'
    path.write_text(json.dumps(metrics,indent=2)+'\n')

    for bag in ('localization_3','localization_from2'):
        fig,ax=plt.subplots(figsize=(9,4.8))
        anyseries=False
        for label,key in [('Current KLT + PnP + gyro','baseline'),
                          ('Persistent KLT keyframes + 21D ESKF','keyframe_eskf')]:
            p=runpath(bag,key)/(key+'_proxy_errors.json')
            if not p.exists():continue
            j=json.loads(p.read_text())
            ax.plot(j['seconds'],j['errors_m'],label=label,linewidth=1.8)
            anyseries=True
        if bag=='localization_3':
            ax.annotate('No trusted keyframe+ESKF\nposes in 20–55s window',
                        xy=(.60,.74),xycoords='axes fraction',ha='center',
                        fontsize=11)
        ax.set(title=bag+' | camera odometry disagreement vs LiDAR-only proxy',
               xlabel='Seconds since recording started',
               ylabel='Position disagreement (m)')
        ax.set_xlim(20,55)
        ax.grid(alpha=.3)
        if anyseries:ax.legend(loc='best')
        fig.tight_layout()
        fig.savefig(OUT/(bag+'_klt_vs_eskf_error_20to55.png'),dpi=160)
        plt.close(fig)

    b=rows['localization_from2']['baseline']['outage_relative_lidar_odom_disagreement']
    e=rows['localization_from2']['keyframe_eskf']['outage_relative_lidar_odom_disagreement']
    rmse_delta=quality_percent(e['position_rmse_m'],b['position_rmse_m'])
    lines=[
      '# ZED X KLT + PnP + gyro vs persistent KLT keyframes + 21D ESKF',
      '**October 21, 2026 — offline only. No cart pose authority.**',
      '', '## Result',
      '', '**Keep the current lightweight KLT + PnP + gyro as the working camera-only fallback.** The new keyframe/ESKF improves the second bag but loses trustworthy localization on the difficult first bag, so it is NOT yet a robust or validated replacement.',
      '', '| Same-image benchmark | KLT + PnP + gyro | KLT keyframes + ESKF |',
      '|---|---:|---:|']
    for bag in ('localization_3','localization_from2'):
        b=rows[bag]['baseline'];e=rows[bag]['keyframe_eskf']
        br=b['outage_relative_lidar_odom_disagreement'];er=e['outage_relative_lidar_odom_disagreement']
        lines.extend([
          f'| **{bag}.bag** | | |',
          f"| Trusted visual tracking | **{b['trusted_visual_updates']}/{b['frames_processed']} ({b['trusted_fraction']*100:.1f}%)** | {e['trusted_visual_updates']}/{e['frames_processed']} ({e['trusted_fraction']*100:.1f}%) |",
          f"| Outage RMSE against LiDAR-only odom (20–55s) | {format_m(br,'position_rmse_m')} | {format_m(er,'position_rmse_m')} |",
          f"| Position disagreement near 55s | {format_m(br,'endpoint_position_disagreement_m')} | {format_m(er,'endpoint_position_disagreement_m')} |",
          f"| 1-second median translation RPE | {format_m(br,'rpe_1s_median_translation_m')} | {format_m(er,'rpe_1s_median_translation_m')} |",
          f"| Keyframes created | — | {e['keyframes']} |",
        ])
    lines.extend(['',
      'The second route improves proxy RMSE from **1.207 m to 0.993 m** (~17.7% reduction), and the endpoint disagreement from **0.596 m to 0.155 m**. These **are not ground-truth errors**, and centimeter-scale endpoint disagreement does not imply centimeter-level absolute accuracy.',
      '', '## Experimental estimator model',
      '', '- Original: fresh depth-valid corners each previous image; LK forward/backward tracking; PnP RANSAC; 35% gyro blend for incremental rotation; concatenate frame motion. 200 Hz accelerometer is unused.',
      '- Prototype: persistent feature IDs and depth anchors per keyframe; keyframe-to-current 3D-to-2D PnP; initial 0.55s stationary ZED IMU calibration; continuous IMU propagation of position, velocity, orientation and gyro/accel biases; relative 6D pose update with retained keyframe clone and covariance cross-terms; Joseph-form covariance, chi-square innovation gate, spatial feature-spread check and fail-closed image/IMU gaps.',
      '- **This is not a full feature-level MSCKF/VINS**: the visual factor is a PnP relative pose, not a per-landmark reprojection update. Its covariance/noise is heuristic, and the 21D clone is replaced as keyframes change. No bias/time offset Allan/Kalibr or physical camera-to-body survey is available.',
      '', '## CPU and RAM: four independent offline process runs',
      '', 'Host: Intel Core i5-1340P, 16 logical CPUs; precomputed ZED registered stereo depth; **no Orin/GPU depth cost included**.',
      '', '| Bag | Estimator | Median visual frame compute | p95 visual compute | IMU propagation total | Offline CPU seconds (entire process) | Peak RSS |',
      '|---|---|---:|---:|---:|---:|---:|'])
    for bag in ('localization_3','localization_from2'):
        for method,label in [('baseline','KLT+PnP+gyro'),('keyframe_eskf','KLT keyframes+ESKF')]:
            m=rows[bag][method]
            lines.append(f"| {bag} | {label} | **{m['visual_processing_median_ms']:.2f} ms** | {m['visual_processing_p95_ms']:.2f} ms | {m['inertial_processing_seconds']:.2f} s | {m['offline_process_cpu_seconds_excluding_reference']:.2f} s | {m['peak_process_ram_mib']:.1f} MiB |")
    lines+=['',
      'The keyframe frontend often spends **less time on image processing** because it reuses persistent tracks instead of detecting corners on every frame. Its separate 200 Hz ESKF propagation adds ~1.1–1.3 CPU-time seconds over these recordings.',
      '**Do not interpret the offline 3.1–3.5 mean CPU cores as live estimator CPU demand.** Compressed rosbag image decoding, image conversion and accelerated OpenCV routines dominate the unthrottled offline process; its wall time is only 12–15 s for ~65–70 s of recorded data. The previously measured ROS1 real-time baseline was **0.55 CPU core and ~165 MiB RSS**; no real-time ROS1 ESKF CPU measurement exists yet.',
      'The per-image PnP timing excludes ZED depth generation, ROS transport, bag decompression and filtering in other processes. Compare separately on the actual Orin and NUC before deployment.',
      '', '## Failure analysis: `localization_3.bag`',
      '', '- At ~19.6 s, the scene temporarily contains fewer than ~25 stable LK+depth tracks. The existing frame-to-frame estimator redetects features and resumes within the difficult section.',
      '- The persistent keyframe estimator loses too many retained features. A reseed allows image correspondences to resume near ~21 s, but its inertial prediction has then continued for **more than the 1.2 s allowed recovery window**.',
      '- For safety, it marks later poses `UNTRUSTED_REACQUIRING_OR_EXTERNAL_REARM_REQUIRED`; no trusted ESKF position is counted during 20–55 s. This is a fail-closed outcome, **not an accuracy success**.',
      '- Prototype IMU-only position continues growing after the violation; those invalid poses were saved **for diagnosis only** and are not published to `/pose`, FAST-LIVO2, TF or a controller.',
      '- Our RGB-D frontend should retain emergency frame-to-frame PnP tracking while preserving persistent keyframe tracks when reliable; otherwise keyframe-only tracking reduces availability.',
      '', '## Prioritized next changes',
      '', '1. Add a secondary frame-to-frame PnP update when persistent keyframe features fall below 30–40; retain shared tracks and switch back only after multiple verified keyframe matches. Do not silently reanchor an inertially drifting pose.',
      '2. Use image-timestamp interpolation of IMU specific force and gyro; estimate camera↔IMU time offset and noise using stationary/moving calibration recordings, and physically survey body↔camera sensor extrinsics.',
      '3. Use a depth-quality model (neighborhood depth consistency and range-dependent measurement covariance) instead of fixed PnP pose covariance; inspect innovation whiteness/NEES/NIS. Current NIS values are near zero and uncertainty is not calibrated.',
      '4. Use consecutive low-parallax and motion-state checks, not a single parallax threshold: stopped cart ≠ camera failure. Add wheel-speed/encoder constraints only when independent vehicle feedback exists.',
      '5. Evaluate a true feature-level MSCKF or a modest windowed depth-aided optimizer if the cloned-pose ESKF remains inconsistent. Compare pose continuity, errors, CPU and covariance against identical bag frames.',
      '6. Leave the physical cart in STOP/HANDOFF_REQUIRED state whenever the odometry source loses vision for a long interval. A camera estimate cannot authorize navigation on its own.',
      '', '## Reproduce the benchmark',
      '', '```bash',
      'cd ~/Desktop/ALIVE_N/FailSafe_FastLivo2',
      'docker run --rm --entrypoint bash -v "$PWD:/repo:ro" fast_livo2:dev -lc "source /opt/ros/noetic/setup.bash; PYTHONPATH=/repo/scripts python3 /repo/scripts/test_zedx_rgbd_keyframe_eskf_core.py"',
      '# A new run identifier is required; scripts refuse to overwrite existing experiments.',
      'cat scripts/run_klt_keyframe_eskf_cpu_benchmarks_20261021.sh',
      'python3 scripts/report_klt_keyframe_eskf_benchmark_20261021.py',
      '```',
      '', 'Experimental files stay under `scripts/zedx_rgbd_keyframe_eskf_core.py`, `scripts/benchmark_klt_keyframe_eskf_20261021.py`, and `outputs/rgbd_fallback_benchmark_20261021/`. Production FAST-LIVO2 and `/pose` are unchanged.']
    (OUT/'FINAL_KLT_KEYFRAME_ESKF_REPORT_20261021.md').write_text('\n'.join(lines)+'\n')
    print('WROTE',str(OUT/'FINAL_KLT_KEYFRAME_ESKF_REPORT_20261021.md'))
    for bag,methods in rows.items():
        for label,m in methods.items():
            e=m['outage_relative_lidar_odom_disagreement']
            print(bag,label,'good',m['trusted_visual_updates'],'/',m['frames_processed'],
                  'median_ms',round(m['visual_processing_median_ms'],2),
                  'cpu_sec',round(m['offline_process_cpu_seconds_excluding_reference'],2),
                  'ram',round(m['peak_process_ram_mib'],1),'RMSE',e.get('position_rmse_m'))


if __name__=='__main__':main()
