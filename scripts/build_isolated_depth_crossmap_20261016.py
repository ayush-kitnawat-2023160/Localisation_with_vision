#!/usr/bin/env python3
"""Build an opt-in, independently cross-checked depth-map registration trial.

The candidate comes from appearance + live depth GICP against the saved
visual voxel map. A separate 15 cm LiDAR PCD is then used to cross-check
and locally refine its XYZ; disagreement fails closed. No online LiDAR is
used during simulated LiDAR failure. Never access the reference trajectory.
"""
from pathlib import Path
import difflib
import subprocess

ROOT=Path(__file__).resolve().parents[1]
BASE='fast_livo2_failsafe:experimental-continuity-retry-20261013'
IMAGE='fast_livo2_failsafe:depth-crossmap-dev-20261016'
SOURCE='src/failsafe_map_localizer.cpp'
src=subprocess.check_output(['docker','run','--rm','--entrypoint','cat',BASE,
       '/ws/src/Fast_Livo2/'+SOURCE],text=True)
out=src
def change(before,after):
 global out
 n=out.count(before)
 if n!=1:raise RuntimeError(f'expected 1 anchor got {n}: {before[:140]}')
 out=out.replace(before,after)

change('''    pnh_.param<bool>("visual_map_enable", visual_map_enable_, false);''',
'''    pnh_.param<bool>("visual_map_enable", visual_map_enable_, false);
    // Opt-in independent saved-geometric-PCD confirmation of RGB-D/voxel
    // visual place pose. Not a second estimator and not another ROS publisher.
    pnh_.param<bool>("depth_crossmap_enable", depth_crossmap_enable_, false);''')
change('''      if (!depth_failover_only_)
      {
        if (pcl::io::loadPCDFile<PointT>(map_path_, *map_) != 0 || map_->empty())''',
'''      // Depth failover normally does not need the LiDAR map in memory;
      // opt-in geometric cross-check is the sole exception. Do not re-enable
      // any healthy map registration/publisher in this frontend.
      if (!depth_failover_only_ || depth_crossmap_enable_)
      {
        if (pcl::io::loadPCDFile<PointT>(map_path_, *map_) != 0 || map_->empty())''')
change('''  bool registerToMap(const CloudT::Ptr &source_lidar,''',
'''  bool confirmDepthWithIndependentGeometricMap(
      const CloudT::Ptr &depth_source,
      const Eigen::Matrix4f &visual_measurement,
      Eigen::Matrix4f &verified_measurement)
  {
    if(!depth_source || depth_source->size()<100 ||
       !map_ || map_->empty() || !visual_measurement.allFinite())
      return false;
    CloudT::Ptr geometric_map = localTarget(
        visual_measurement.block<3,1>(0,3), 13.0, 4.0, false);
    if(!geometric_map || geometric_map->size()<450)
      return false;
    CloudT::Ptr current(new CloudT(*depth_source));
    downsampleInPlace(current,0.35);
    downsampleInPlace(geometric_map,0.55);
    if(current->size()<90 || geometric_map->size()<300)
      return false;

    // A SECOND map, produced from LiDAR geometry, is required to agree with
    // the live depth scan. Visual-voxel GICP fitness alone is not sufficient:
    // flat/repeated roads can yield low residual at the wrong location.
    pcl::GeneralizedIterativeClosestPoint<PointT,PointT> geom;
    geom.setInputSource(current);
    geom.setInputTarget(geometric_map);
    geom.setMaximumIterations(14);
    geom.setMaxCorrespondenceDistance(1.0);
    geom.setTransformationEpsilon(1.0e-5);
    geom.setEuclideanFitnessEpsilon(1.0e-4);
    geom.setCorrespondenceRandomness(12);
    CloudT aligned;
    geom.align(aligned,visual_measurement);
    if(!geom.hasConverged())return false;
    const Eigen::Matrix4f measured=geom.getFinalTransformation();
    const Eigen::Matrix4f residual=visual_measurement.inverse()*measured;
    const double dxyz=residual.block<3,1>(0,3).norm();
    const double drot=rotationAngleDeg(residual.block<3,3>(0,0).cast<double>());
    const double fitness=geom.getFitnessScore(1.0);
    CloudT::Ptr aligned_ptr(new CloudT(aligned));
    const double overlap=overlapRatio(aligned_ptr,geometric_map,0.70);
    if(!std::isfinite(fitness) || fitness>=0.20 || overlap<0.55 ||
       dxyz>1.20 || drot>8.0){
      ROS_WARN_THROTTLE(0.75,
          "[DEPTH CROSSMAP] reject weak independent canonical-PCD verification fit=%.3f ov=%.2f dxyz=%.2fm dRot=%.1fdeg",
          fitness,overlap,dxyz,drot);
      return false;
    }

    // Map origins are the same loop-closed PCD frame. Keep camera/IMU yaw,
    // and let the separate geometric map correct *only position*. The 0.75
    // blend reduces one-off 3D minima without inventing measurements.
    verified_measurement=visual_measurement;
    verified_measurement.block<3,1>(0,3)=
        0.25f*visual_measurement.block<3,1>(0,3)+
        0.75f*measured.block<3,1>(0,3);
    ROS_INFO_THROTTLE(0.5,
        "[DEPTH CROSSMAP] accepted independent canonical-PCD match fit=%.3f ov=%.2f delta=%.2fm heading_rejected=%.1fdeg",
        fitness,overlap,dxyz,drot);
    return true;
  }

  bool registerToMap(const CloudT::Ptr &source_lidar,''')
change('''          best_score = score;
          guess = candidate_guess;
          T_wl = selected_pose;''',
'''          if(lidar_depth_fail && depth_crossmap_enable_){
            Eigen::Matrix4f double_verified;
            if(!confirmDepthWithIndependentGeometricMap(
                   source,selected_pose,double_verified))
              continue; // Reject ambiguous visual geometry; never loosen gates
            selected_pose=double_verified;
          }

          best_score = score;
          guess = candidate_guess;
          T_wl = selected_pose;''')
change('''  bool visual_map_enable_ = false;''',
'''  bool visual_map_enable_ = false;
  bool depth_crossmap_enable_ = false;''')

dst=ROOT/'step3_global/depth_crossmap_trial_20261016'
dst.mkdir(parents=True,exist_ok=True)
path=dst/SOURCE
path.parent.mkdir(parents=True,exist_ok=True)
path.write_text(out)
(dst/'isolated_change.patch').write_text('\n'.join(difflib.unified_diff(
    src.splitlines(),out.splitlines(),fromfile='frozen/'+SOURCE,
    tofile='trial/'+SOURCE,lineterm=''))+'\n')
(dst/'Dockerfile').write_text(f'''FROM {BASE}
COPY src/failsafe_map_localizer.cpp /ws/src/Fast_Livo2/src/failsafe_map_localizer.cpp
RUN /bin/bash -lc 'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && make -C /ws/build -j3 failsafe_map_localizer'
''')
original=(ROOT/'config/map_failsafe_recorded_cart.yaml').read_text()
key='visual_map_enable: true\n'
assert original.count(key)==1
(dst/'map_failsafe_depth_crossmap.yaml').write_text(
    original.replace(key,key+'# Experimental independent geometrical map confirmation; default disabled.\ndepth_crossmap_enable: true\n'))
print('Created isolated dual-map geometrical verification dev source',dst)
subprocess.run(['docker','build','-t',IMAGE,str(dst)],check=True)
