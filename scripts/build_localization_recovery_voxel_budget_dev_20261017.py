#!/usr/bin/env python3
"""Opt-in bounded *localization* recovery-only voxel-map processing.
Recompile only FAST-LIVO2 mapper from frozen 8-case image; all maps and
global measurement frontends unmodified. No vehicle-control code touched."""
from pathlib import Path
import subprocess,difflib
ROOT=Path(__file__).resolve().parents[1]
BASE='fast_livo2_failsafe:single-build-rc-20261016'
TAG='fast_livo2_failsafe:recovery-voxel-budget-localization-dev-20261017'
path='/ws/src/Fast_Livo2/src/LIVMapper.cpp'
source=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',BASE,path],text=True)
out=source
def change(a,b):
 global out
 n=out.count(a)
 assert n==1,(n,a[:85])
 out=out.replace(a,b)
change('''  nh.param<int>("failsafe/recovery_lio_warmup_scans",
                recovery_lio_warmup_scans_, 5);''','''  nh.param<int>("failsafe/recovery_lio_warmup_scans",
                recovery_lio_warmup_scans_, 5);
  nh.param<bool>("failsafe/recovery_voxel_budget_enable",
                 recovery_voxel_budget_enable_, false);
  nh.param<int>("failsafe/recovery_voxel_max_points",
                 recovery_voxel_max_points_, 1400);
  recovery_voxel_max_points_=std::max(500,std::min(10000,recovery_voxel_max_points_));''')
# Limit ONLY local-map recovery warm-up: keep all normal scans unchanged.
# VoxelMapManager::BuildVoxelMap() computes covariance and allocates octree
# once per point; choose uniformly across the input, with offset per scan.
# Do not modify the global saved PCD map or ESIKF measurement weighting.
change('''    voxelmap_manager->BuildVoxelMap();

    if (seeded_recovery_map)''','''    if (seeded_recovery_map && recovery_voxel_budget_enable_ &&
        feats_down_body->size()>static_cast<size_t>(recovery_voxel_max_points_) &&
        feats_down_body->size()==feats_down_world->size()){
      auto body_full=voxelmap_manager->feats_down_body_;
      auto world_full=voxelmap_manager->feats_down_world_;
      PointCloudXYZI::Ptr body(new PointCloudXYZI),world(new PointCloudXYZI);
      body->reserve(recovery_voxel_max_points_);
      world->reserve(recovery_voxel_max_points_);
      const size_t stride=std::max<size_t>(1,
          (body_full->size()+recovery_voxel_max_points_-1)/recovery_voxel_max_points_);
      for(size_t i=0;i<body_full->size() && body->size()<static_cast<size_t>(recovery_voxel_max_points_);i+=stride){
        body->push_back(body_full->points[i]);world->push_back(world_full->points[i]);
      }
      voxelmap_manager->feats_down_body_=body;
      voxelmap_manager->feats_down_world_=world;
      voxelmap_manager->BuildVoxelMap();
      voxelmap_manager->feats_down_body_=body_full;
      voxelmap_manager->feats_down_world_=world_full;
      ROS_INFO("[LOCALIZATION RECOVERY BUDGET] seed local voxel map %zu/%zu points",
          body->size(),body_full->size());
    }else voxelmap_manager->BuildVoxelMap();

    if (seeded_recovery_map)''')
change('''      voxelmap_manager->BuildVoxelMap();
      last_recovery_warmup_lio_time_ = warmup_time;''','''      if(recovery_voxel_budget_enable_ &&
         feats_down_body->size()>static_cast<size_t>(recovery_voxel_max_points_) &&
         feats_down_body->size()==feats_down_world->size()){
        auto body_full=voxelmap_manager->feats_down_body_;
        auto world_full=voxelmap_manager->feats_down_world_;
        PointCloudXYZI::Ptr body(new PointCloudXYZI),world(new PointCloudXYZI);
        body->reserve(recovery_voxel_max_points_);world->reserve(recovery_voxel_max_points_);
        const size_t stride=std::max<size_t>(1,
          (body_full->size()+recovery_voxel_max_points_-1)/recovery_voxel_max_points_);
        const size_t offset=static_cast<size_t>(recovery_lio_warmup_remaining_-1)%stride;
        for(size_t i=offset;i<body_full->size() && body->size()<static_cast<size_t>(recovery_voxel_max_points_);i+=stride){
          body->push_back(body_full->points[i]);world->push_back(world_full->points[i]);
        }
        voxelmap_manager->feats_down_body_=body;
        voxelmap_manager->feats_down_world_=world;
        voxelmap_manager->BuildVoxelMap();
        voxelmap_manager->feats_down_body_=body_full;
        voxelmap_manager->feats_down_world_=world_full;
        ROS_INFO("[LOCALIZATION RECOVERY BUDGET] warm-up local voxel map %zu/%zu points",
            body->size(),body_full->size());
      }else voxelmap_manager->BuildVoxelMap();
      last_recovery_warmup_lio_time_ = warmup_time;''')
HEADER='/ws/src/Fast_Livo2/include/LIVMapper.h'
before=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',BASE,HEADER],text=True)
assert before.count('  int recovery_lio_warmup_scans_ = 5;')==1
head=before.replace('  int recovery_lio_warmup_scans_ = 5;',
'''  int recovery_lio_warmup_scans_ = 5;
  bool recovery_voxel_budget_enable_ = false;
  int recovery_voxel_max_points_ = 1400;''')
work=ROOT/'step3_global/recovery_voxel_budget_localization_dev_20261017'
work.mkdir(parents=True,exist_ok=True)
(work/'LIVMapper.cpp').write_text(out)
(work/'LIVMapper.h').write_text(head)
(work/'change.cpp.diff').write_text('\n'.join(difflib.unified_diff(source.splitlines(),out.splitlines(),lineterm=''))+'\n')
(work/'Dockerfile').write_text(f'''FROM {BASE}
COPY LIVMapper.cpp {path}
COPY LIVMapper.h {HEADER}
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 fastlivo_mapping'
''')
# Do not make fallback behavior depend on current maps or GT.
launch=(ROOT/'launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch').read_text()
insert='  <param name="step3/gyro_bias_feedback_enable" value="$(arg tracking_gyro_bias_feedback_enable)"/>'
assert launch.count(insert)==1
launch=launch.replace(insert,insert+'\n'
   '  <param name="failsafe/recovery_voxel_budget_enable" value="true"/>\n'
   '  <param name="failsafe/recovery_voxel_max_points" value="1400"/>')
(ROOT/'launch/fast_livo2_recovery_voxel_budget_dev_20261017.launch').write_text(launch)
subprocess.run(['docker','build','-t',TAG,'-f',str(work/'Dockerfile'),str(work)],check=True)
print('BUILT',TAG,flush=True)
