#!/usr/bin/env python3
"""Isolated localization-only recovery CPU budget experiment.

Source from frozen eight-case image. Cap points used for recovery voxel map
BUILD only (not for ESIKF odometry, raw inputs or saved PCD matching).
Opt-in parameters; identical normal estimator & tracking outside recovery.
"""
from pathlib import Path
import subprocess,difflib
root=Path(__file__).resolve().parents[1]
base='fast_livo2_failsafe:single-build-rc-20261016'
tag='fast_livo2_failsafe:recovery-voxel-budget-localization-dev-20261017'
target='/ws/src/Fast_Livo2/src/LIVMapper.cpp'
before=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',base,target],text=True)
code=before
def once(a,b):
 global code
 n=code.count(a)
 if n!=1:raise ValueError(f'Expected one anchor found {n}: {a[:80]}')
 code=code.replace(a,b)
once('''  nh.param<int>("failsafe/recovery_lio_warmup_scans",
                recovery_lio_warmup_scans_, 5);
''','''  nh.param<int>("failsafe/recovery_lio_warmup_scans",
                recovery_lio_warmup_scans_, 5);
  // Only recovery-map construction is work-limited. This is off by default.
  // Standard FAST-LIVO2 ESIKF scan, LiDAR and visual measurements unchanged.
  nh.param<int>("failsafe/recovery_voxel_seed_point_budget",
                recovery_voxel_seed_point_budget_, 0);
''')
anchor='''  bool seeded_recovery_map = false;
  if (rebuild_voxel_map_on_recovery_)
'''
assert code.count(anchor)==1
budget='''  // Bound expensive voxel octree construction exclusively during
  // recovery seeding and recovery warm-up. World/body points are sampled
  // with the same deterministic indices to preserve exact extrinsics and
  // estimator covariance. The complete normal LIO scan is retained.
  const auto buildRecoveryVoxelMapBudgeted = [&]() {
    const size_t N=feats_down_body->points.size();
    const size_t budget = recovery_voxel_seed_point_budget_ <= 0
        ? N : static_cast<size_t>(recovery_voxel_seed_point_budget_);
    if(N<=budget || budget<100){
      voxelmap_manager->BuildVoxelMap();
      return;
    }
    PointCloudXYZI::Ptr body(new PointCloudXYZI);
    PointCloudXYZI::Ptr world(new PointCloudXYZI);
    body->reserve(budget); world->reserve(budget);
    for(size_t k=0;k<budget;k++){
      const size_t i=k*N/budget;
      body->push_back(feats_down_body->points[i]);
      world->push_back(feats_down_world->points[i]);
    }
    auto orig_body=voxelmap_manager->feats_down_body_;
    auto orig_world=voxelmap_manager->feats_down_world_;
    const auto orig_count=voxelmap_manager->feats_down_size_;
    voxelmap_manager->feats_down_body_=body;
    voxelmap_manager->feats_down_world_=world;
    voxelmap_manager->feats_down_size_=budget;
    const double t0=omp_get_wtime();
    voxelmap_manager->BuildVoxelMap();
    const double ms=(omp_get_wtime()-t0)*1.0e3;
    voxelmap_manager->feats_down_body_=orig_body;
    voxelmap_manager->feats_down_world_=orig_world;
    voxelmap_manager->feats_down_size_=orig_count;
    ROS_INFO("[FAILSAFE RECOVERY BUDGET] %zu -> %zu map seed points; build %.1f ms",
             N,budget,ms);
  };

'''
code=code.replace(anchor,budget+anchor)
once('''    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();

    if (seeded_recovery_map)''',
'''    lidar_map_inited = true;
    if(seeded_recovery_map) buildRecoveryVoxelMapBudgeted();
    else voxelmap_manager->BuildVoxelMap();

    if (seeded_recovery_map)''')
once('''      voxelmap_manager->BuildVoxelMap();
      last_recovery_warmup_lio_time_ = warmup_time;''','''      buildRecoveryVoxelMapBudgeted();
      last_recovery_warmup_lio_time_ = warmup_time;''')
# Keep class declaration minimal and make experiment flag clear.
header='/ws/src/Fast_Livo2/include/LIVMapper.h'
hbefore=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',base,header],text=True)
assert hbefore.count('  int recovery_lio_warmup_scans_ = 5;')==1
hnew=hbefore.replace('  int recovery_lio_warmup_scans_ = 5;','  int recovery_lio_warmup_scans_ = 5;\n  int recovery_voxel_seed_point_budget_ = 0;')
work=root/'step3_global/recovery_voxel_budget_localization_dev_20261017'
work.mkdir(parents=True,exist_ok=True)
(work/'LIVMapper.cpp').write_text(code)
(work/'LIVMapper.h').write_text(hnew)
(work/'LIVMapper.diff').write_text('\n'.join(difflib.unified_diff(before.splitlines(),code.splitlines(),lineterm=''))+'\n')
(work/'Dockerfile').write_text(f'''FROM {base}
COPY LIVMapper.cpp {target}
COPY LIVMapper.h {header}
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 fastlivo_mapping'
''')
launch=(root/'launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch').read_text()
(root/'launch/fast_livo2_recovery_voxel_budget_dev_20261017.launch').write_text(launch)
# The child launch loads YAML globally, so use a separate TEST-ONLY YAML.
# Do not overwrite the accepted YAML or change any production launch.
config=(root/'config/failsafe_rosbag.yaml').read_text()
token='  imu_recovery_samples: 20\n'
assert config.count(token)==1
config=config.replace(token,token+'''  # Opt-in bounded recovery-map build only, not normal LIO.
  recovery_voxel_seed_point_budget: 1200
''')
warmup='  recovery_lio_warmup_scans: 5\n'
assert config.count(warmup)==1
config=config.replace(warmup,'  recovery_lio_warmup_scans: 3\n')
(root/'config/failsafe_recovery_voxel_budget_dev_20261017.yaml').write_text(config)
print('INPUT AND WORK OUTPUT',len(before),len(code),len(hnew))
subprocess.run(['docker','build','-t',tag,'-f',str(work/'Dockerfile'),str(work)],check=True)
print('BUILT_IMAGE',tag)
