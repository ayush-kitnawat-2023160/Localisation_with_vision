#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <list>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <atomic>
#include <numeric>
#include <string>
#include <tuple>
#include <unordered_map>
#include <chrono>
#include <vector>

namespace {
using PointT = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointT>;
constexpr int kRings = 20;
constexpr int kSectors = 60;
constexpr double kRadius = 80.0;
constexpr double kLidarHeight = 2.0;
constexpr double kSectorAngle = 2.0 * M_PI / kSectors;

double wrap(double a){ while(a>M_PI)a-=2*M_PI; while(a<-M_PI)a+=2*M_PI; return a; }
double yawOf(const Eigen::Isometry3d& T){ return std::atan2(T.linear()(1,0),T.linear()(0,0)); }
geometry_msgs::Pose poseMsg(const Eigen::Isometry3d& T){ geometry_msgs::Pose p; Eigen::Quaterniond q(T.linear()); p.position.x=T.translation().x();p.position.y=T.translation().y();p.position.z=T.translation().z();p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();return p; }
bool odomPose(const nav_msgs::Odometry& m,Eigen::Isometry3d& T){ Eigen::Quaterniond q(m.pose.pose.orientation.w,m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z); if(q.norm()<1e-8)return false;q.normalize();T=Eigen::Isometry3d::Identity();T.linear()=q.toRotationMatrix();T.translation()<<m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z;return T.matrix().allFinite(); }

struct Descriptor { std::array<double,kRings*kSectors> v{}; std::array<double,kRings> ring{}; };
Descriptor makeDescriptor(const Cloud& cloud){
  Descriptor d; d.v.fill(-1000.0);
  for(const auto& p:cloud){ if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue; const double r=std::hypot(p.x,p.y); if(r>kRadius||r<1e-6)continue; double a=std::atan2(p.y,p.x); if(a<0)a+=2*M_PI; int ri=std::min(kRings-1,std::max(0,(int)std::ceil(r/kRadius*kRings)-1)); int si=std::min(kSectors-1,std::max(0,(int)std::ceil(a/(2*M_PI)*kSectors)-1)); d.v[ri*kSectors+si]=std::max(d.v[ri*kSectors+si],(double)p.z+kLidarHeight); }
  for(auto& x:d.v) if(x < -999.0) x=0.0;
  for(int r=0;r<kRings;++r){ double s=0; for(int c=0;c<kSectors;++c)s+=d.v[r*kSectors+c]; d.ring[r]=s/kSectors; }
  return d;
}

// The same sector norm is used at every circular shift. Compute it once per
// descriptor comparison, rather than recomputing 60 times. Only the dot
// products depend on the shift. Keep the original ring summation order and
// strict < tie-breaker so the selected map ID / yaw remains unchanged.
std::pair<double,int> bestDescriptorDistance(const Descriptor& q,const Descriptor& m){
  std::array<double,kSectors> q_norm2{}, m_norm2{};
  for(int c=0;c<kSectors;++c){
    double nq=0,nm=0;
    for(int r=0;r<kRings;++r){
      const double a=q.v[r*kSectors+c], b=m.v[r*kSectors+c];
      nq+=a*a; nm+=b*b;
    }
    q_norm2[c]=nq; m_norm2[c]=nm;
  }
  double best=1e9; int best_shift=0;
  for(int shift=0;shift<kSectors;++shift){
    double sim=0; int effective=0;
    for(int c=0;c<kSectors;++c){
      const int mc=(c+shift)%kSectors;
      const double nq=q_norm2[c],nm=m_norm2[mc];
      if(!(nq>1e-12&&nm>1e-12))continue;
      double dot=0;
      for(int r=0;r<kRings;++r)
        dot+=q.v[r*kSectors+c]*m.v[r*kSectors+mc];
      sim+=dot/std::sqrt(nq*nm);++effective;
    }
    const double d=effective?1.0-sim/effective:1.0;
    if(d<best){best=d;best_shift=shift;}
  }
  return {best,best_shift};
}
}

class Step3GlobalAcquirer {
 public:
  Step3GlobalAcquirer():nh_(),pnh_("~"){
    pnh_.param<std::string>("map_db",map_db_,std::string()); pnh_.param<std::string>("lidar_topic",lidar_topic_,"/fast_livo2/input/lidar"); pnh_.param<std::string>("fast_odom_topic",odom_topic_,"/aft_mapped_to_init");
    pnh_.param("query_hz",query_hz_,2.0); pnh_.param("source_voxel",source_voxel_,0.35); pnh_.param("descriptor_candidates",descriptor_candidates_,12); pnh_.param("descriptor_threshold",descriptor_threshold_,0.34); pnh_.param("geometric_candidates",geometric_candidates_,4); pnh_.param("gicp_max_correspondence",gicp_corr_,2.0); pnh_.param("gicp_max_fitness",gicp_fitness_,0.12); pnh_.param("consistency_translation",consistency_translation_,1.0); pnh_.param("consistency_yaw_deg",consistency_yaw_deg_,5.0); pnh_.param("required_consistent",required_consistent_,2);
    // OFF by default. GPS is only a bounded preference among LiDAR candidates;
    // it never produces a pose, relaxes a geometry gate, or vetoes global search.
    pnh_.param("gps_soft_prior_enable",gps_soft_prior_enable_,false);
    pnh_.param<std::string>("gps_topic",gps_topic_,"/mavros/global_position/raw/fix");
    pnh_.param("gps_max_age_sec",gps_max_age_sec_,0.65);
    pnh_.param("gps_max_covariance_m2",gps_max_covariance_m2_,100.0);
    pnh_.param("gps_preference_radius_m",gps_preference_radius_m_,25.0);
    pnh_.param("gps_sc_penalty_per_m",gps_sc_penalty_per_m_,0.0010);
    pnh_.param("gps_fitness_penalty_per_m",gps_fitness_penalty_per_m_,0.0010);
    pnh_.param("mh_enable",mh_enable_,false);
    pnh_.param("mh_scancontext_candidates",mh_sc_candidates_,16);
    pnh_.param("mh_geometric_candidates",mh_geometric_candidates_,5);
    pnh_.param("mh_max_hypotheses",mh_max_hypotheses_,4);
    pnh_.param("mh_required_hits",mh_required_hits_,3);
    pnh_.param("mh_max_fitness",mh_max_fitness_,0.15);
    pnh_.param("mh_association_xy_m",mh_association_xy_m_,3.0);
    pnh_.param("mh_association_yaw_deg",mh_association_yaw_deg_,18.0);
    pnh_.param("mh_stale_sec",mh_stale_sec_,7.0);
    pnh_.param("mh_ambiguity_separation_m",mh_ambiguity_separation_m_,6.0);
    pnh_.param("mh_candidate_diversity_m",mh_candidate_diversity_m_,8.0);
    pnh_.param("mh_min_fitness_advantage",mh_min_fitness_advantage_,0.035);
    pnh_.param("mh_max_reanchor_m",mh_max_reanchor_m_,4.0);
    pnh_.param("mh_max_reanchor_yaw_deg",mh_max_reanchor_yaw_deg_,15.0);
    pnh_.param("mh_max_auto_pose_resume_gap_sec",mh_max_auto_pose_resume_gap_sec_,2.5);
    // OFF by default; visual pose is a search proposal, never a direct
    // estimator update or authority. It must pass GICP and MH scan consensus.
    pnh_.param("mh_visual_seed_enable",mh_visual_seed_enable_,false);
    pnh_.param("mh_visual_seed_max_age_sec",mh_visual_seed_max_age_sec_,1.4);
    pnh_.param("mh_visual_seed_max_xy_variance_m2",mh_visual_seed_max_xy_variance_m2_,16.0);
    pnh_.param<std::string>("mh_visual_candidate_topic",mh_visual_candidate_topic_,
                             "/fast_livo2/visual_global_candidate");
    if(mh_enable_ && gps_soft_prior_enable_){
      ROS_WARN("[STEP3 MH] ignoring gps_soft_prior_enable: multi-hypothesis experiment is LiDAR/odometry ONLY");
      gps_soft_prior_enable_=false;
    }
    pnh_.param("tracking_hz",tracking_hz_,1.0);
    pnh_.param("tracking_candidates",tracking_candidates_,3);
    pnh_.param("tracking_submap_cache_slots",tracking_submap_cache_slots_,0);
    pnh_.param("tracking_gicp_max_iterations",tracking_gicp_max_iterations_,25);
    pnh_.param("tracking_deferred_scan_enable",tracking_deferred_scan_enable_,false);
    pnh_.param("tracking_deferred_scan_max_age_sec",tracking_deferred_scan_max_age_sec_,1.8);
    pnh_.param("tracking_continuity_retry_enable",tracking_continuity_retry_enable_,false);
    pnh_.param("tracking_continuity_retry_physical_only_enable",tracking_continuity_retry_physical_only_enable_,false);
    pnh_.param("tracking_healthy_continuity_reseed_enable",tracking_healthy_continuity_reseed_enable_,false);
    pnh_.param("tracking_healthy_continuity_reseed_min_seed_delta_m",tracking_healthy_continuity_reseed_min_seed_delta_m_,0.4);
    pnh_.param("tracking_healthy_continuity_reseed_max_fitness",tracking_healthy_continuity_reseed_max_fitness_,0.14);
    pnh_.param("tracking_continuity_retry_max_age_sec",tracking_continuity_retry_max_age_sec_,4.5);
    pnh_.param("tracking_heading_weight_m_per_rad",tracking_heading_weight_m_per_rad_,3.0);
    pnh_.param("tracking_id_free_window",tracking_id_free_window_,8.0);
    pnh_.param("tracking_id_window_per_sec",tracking_id_window_per_sec_,2.5);
    pnh_.param("tracking_id_penalty_m_per_keyframe",tracking_id_penalty_m_per_keyframe_,0.50);
    pnh_.param("tracking_max_fitness",tracking_max_fitness_,0.20);
    pnh_.param("tracking_max_translation_innovation",tracking_max_translation_innovation_,3.0);
    pnh_.param("tracking_max_yaw_innovation_deg",tracking_max_yaw_innovation_deg_,10.0);
    // Reject registrations which imply distance travelled incompatible with
    // time-aligned local FAST motion. A low GICP fitness alone can still select
    // a neighbouring road/submap and move the global anchor several metres.
    pnh_.param("tracking_healthy_motion_consistency_gate",tracking_healthy_motion_consistency_gate_,false);
    pnh_.param("tracking_healthy_motion_consistency_slack_m",tracking_healthy_motion_consistency_slack_m_,0.45);
    pnh_.param("tracking_healthy_motion_consistency_rate_mps",tracking_healthy_motion_consistency_rate_mps_,0.65);
    pnh_.param("tracking_healthy_motion_consistency_max_gap_sec",tracking_healthy_motion_consistency_max_gap_sec_,5.0);
    pnh_.param("tracking_healthy_motion_consistency_strong_fitness",tracking_healthy_motion_consistency_strong_fitness_,0.0);
    // Optional map-keyframe lateral prior for repeated, parallel road passes.
    // The reference trajectory CSV is NEVER read at runtime: this only uses
    // poses bundled in the saved PCD registration database. Disabled by default.
    pnh_.param("tracking_healthy_route_lateral_prior_enable",tracking_healthy_route_lateral_prior_enable_,false);
    pnh_.param("tracking_healthy_route_lateral_prior_gain",tracking_healthy_route_lateral_prior_gain_,0.65);
    pnh_.param("tracking_healthy_route_lateral_prior_deadband_m",tracking_healthy_route_lateral_prior_deadband_m_,0.30);
    pnh_.param("tracking_healthy_route_lateral_prior_max_step_m",tracking_healthy_route_lateral_prior_max_step_m_,1.0);
    pnh_.param("tracking_healthy_route_lateral_prior_max_offset_m",tracking_healthy_route_lateral_prior_max_offset_m_,2.60);
    pnh_.param("tracking_healthy_route_lateral_prior_min_fitness",tracking_healthy_route_lateral_prior_min_fitness_,0.08);
    pnh_.param("tracking_healthy_route_lateral_prior_max_fitness",tracking_healthy_route_lateral_prior_max_fitness_,0.14);
    // A good, route-continuous GICP match must be allowed to observe heading
    // drift. Otherwise FAST can rotate away from the road until the fixed yaw
    // gate rejects exactly the measurements needed to correct that drift.
    pnh_.param("tracking_relaxed_yaw_max_deg",tracking_relaxed_yaw_max_deg_,18.0);
    pnh_.param("tracking_relaxed_yaw_max_fitness",tracking_relaxed_yaw_max_fitness_,0.14);
    pnh_.param("tracking_relaxed_yaw_max_translation",tracking_relaxed_yaw_max_translation_,2.5);
    pnh_.param("tracking_route_continuous_max_fitness",tracking_route_continuous_max_fitness_,0.28);
    pnh_.param("tracking_route_continuous_max_translation",tracking_route_continuous_max_translation_,4.0);
    pnh_.param("tracking_route_continuous_max_yaw_deg",tracking_route_continuous_max_yaw_deg_,14.0);
    pnh_.param("tracking_gap_predict_after_sec",tracking_gap_predict_after_sec_,1.25);
    pnh_.param("tracking_gap_predict_blend_sec",tracking_gap_predict_blend_sec_,0.50);
    pnh_.param("tracking_gap_predict_max_sec",tracking_gap_predict_max_sec_,6.0);
    pnh_.param("tracking_gap_predict_max_speed_mps",tracking_gap_predict_max_speed_mps_,8.0);
    pnh_.param("tracking_gap_predict_max_yaw_rate_deg_s",tracking_gap_predict_max_yaw_rate_deg_s_,55.0);
    pnh_.param("tracking_gap_predict_velocity_alpha",tracking_gap_predict_velocity_alpha_,0.65);
    pnh_.param("tracking_reacquire_after_sec",tracking_reacquire_after_sec_,3.0);
    pnh_.param("tracking_reacquire_max_fitness",tracking_reacquire_max_fitness_,0.18);
    pnh_.param("tracking_reacquire_max_translation_innovation",tracking_reacquire_max_translation_innovation_,12.0);
    pnh_.param("tracking_reacquire_max_yaw_innovation_deg",tracking_reacquire_max_yaw_innovation_deg_,20.0);
    pnh_.param("tracking_route_reacquire_id_band",tracking_route_reacquire_id_band_,18);
    pnh_.param("tracking_route_reacquire_id_rate",tracking_route_reacquire_id_rate_,4.0);
    pnh_.param("tracking_route_reacquire_descriptor_threshold",tracking_route_reacquire_descriptor_threshold_,0.45);
    pnh_.param("tracking_route_reacquire_geometric_candidates",tracking_route_reacquire_geometric_candidates_,3);
    pnh_.param("tracking_route_reacquire_max_fitness",tracking_route_reacquire_max_fitness_,0.22);
    pnh_.param("tracking_route_reacquire_max_speed_mps",tracking_route_reacquire_max_speed_mps_,12.0);
    pnh_.param("tracking_route_reacquire_max_yaw_rate_deg_s",tracking_route_reacquire_max_yaw_rate_deg_s_,60.0);
    pnh_.param("tracking_route_reacquire_enabled",tracking_route_reacquire_enabled_,false);
    pnh_.param("tracking_global_reacquire_after_sec",tracking_global_reacquire_after_sec_,6.0);
    pnh_.param("tracking_global_reacquire_after_misses",tracking_global_reacquire_after_misses_,3);
    pnh_.param("tracking_global_reacquire_unrestricted_after_sec",tracking_global_reacquire_unrestricted_after_sec_,30.0);
    // Map keyframe IDs are storage order, not route topology: a physically
    // adjacent road section can be hundreds of IDs away at a loop/junction.
    // Keep a switch for reproducing the old behaviour, but gate by reachable
    // map-space displacement and yaw instead of arbitrary ID differences.
    pnh_.param("tracking_global_reacquire_physical_gate",tracking_global_reacquire_physical_gate_,true);
    pnh_.param("tracking_global_reacquire_position_slack_m",tracking_global_reacquire_position_slack_m_,3.0);
    pnh_.param("tracking_global_reacquire_vertical_slack_m",tracking_global_reacquire_vertical_slack_m_,3.0);
    pnh_.param("tracking_global_reacquire_recovery_vertical_rate_mps",tracking_global_reacquire_recovery_vertical_rate_mps_,0.0);
    pnh_.param("tracking_global_reacquire_recovery_vertical_max_slack_m",tracking_global_reacquire_recovery_vertical_max_slack_m_,3.0);
    pnh_.param("tracking_global_reacquire_predicted_pose_weight",tracking_global_reacquire_predicted_pose_weight_,0.001);
    pnh_.param("tracking_global_reacquire_hz",tracking_global_reacquire_hz_,0.33);
    pnh_.param("tracking_global_reacquire_descriptor_candidates",tracking_global_reacquire_descriptor_candidates_,20);
    pnh_.param("tracking_global_reacquire_descriptor_threshold",tracking_global_reacquire_descriptor_threshold_,0.38);
    pnh_.param("tracking_global_reacquire_geometric_candidates",tracking_global_reacquire_geometric_candidates_,4);
    pnh_.param("tracking_global_reacquire_max_fitness",tracking_global_reacquire_max_fitness_,0.14);
    pnh_.param("tracking_global_reacquire_required_consistent",tracking_global_reacquire_required_consistent_,2);
    pnh_.param("tracking_global_reacquire_consistency_translation",tracking_global_reacquire_consistency_translation_,3.0);
    pnh_.param("tracking_global_reacquire_consistency_yaw_deg",tracking_global_reacquire_consistency_yaw_deg_,12.0);
    pnh_.param("tracking_global_reacquire_max_speed_mps",tracking_global_reacquire_max_speed_mps_,10.0);
    pnh_.param("tracking_global_reacquire_max_yaw_rate_deg_s",tracking_global_reacquire_max_yaw_rate_deg_s_,45.0);
    pnh_.param("tracking_global_reacquire_max_id_rate",tracking_global_reacquire_max_id_rate_,6.0);
    pnh_.param("tracking_global_reacquire_hard_reset_translation",tracking_global_reacquire_hard_reset_translation_,15.0);
    pnh_.param("tracking_global_reacquire_ramp_sec",tracking_global_reacquire_ramp_sec_,3.0);
    pnh_.param("fast_local_max_speed_mps",fast_local_max_speed_mps_,15.0);
    pnh_.param("fast_local_max_vertical_speed_mps",fast_local_max_vertical_speed_mps_,6.0);
    pnh_.param("fast_local_max_pose_gap_sec",fast_local_max_pose_gap_sec_,1.0);
    pnh_.param("fast_local_recover_samples",fast_local_recover_samples_,10);
    pnh_.param("lidar_recovery_bridge_sec",lidar_recovery_bridge_sec_,2.5);
    // A single LiDAR callback can arrive before the matching FAST odometry
    // sample has been processed. Treat that as transient callback skew, not as
    // an estimator failure. The previous behavior escalated after a few such
    // misses into expensive whole-map reacquisition, which then starved FAST
    // long enough to create the very failure it was trying to recover from.
    pnh_.param("fast_odom_missing_grace_sec",fast_odom_missing_grace_sec_,1.0);
    pnh_.param("fast_odom_missing_scans_before_unstable",
               fast_odom_missing_scans_before_unstable_,6);
    pnh_.param("map_fix_fallback_follow_tau_sec",map_fix_fallback_follow_tau_sec_,0.35);
    pnh_.param("map_fix_fallback_max_extrapolation_sec",map_fix_fallback_max_extrapolation_sec_,1.2);
    pnh_.param("map_fix_fallback_max_speed_mps",map_fix_fallback_max_speed_mps_,12.0);
    pnh_.param("map_fix_fallback_max_yaw_rate_deg_s",map_fix_fallback_max_yaw_rate_deg_s_,90.0);
    pnh_.param("tracking_update_anchor",tracking_update_anchor_,true);
    pnh_.param("tracking_anchor_translation_gain",tracking_anchor_translation_gain_,1.00);
    pnh_.param("tracking_anchor_max_xy_step",tracking_anchor_max_xy_step_,0.80);
    pnh_.param("tracking_anchor_max_z_step",tracking_anchor_max_z_step_,0.15);
    pnh_.param("tracking_anchor_yaw_gain",tracking_anchor_yaw_gain_,0.75);
    pnh_.param("tracking_anchor_max_yaw_step_deg",tracking_anchor_max_yaw_step_deg_,1.5);
    pnh_.param("tracking_correction_ramp_sec",tracking_correction_ramp_sec_,2.0);
    pnh_.param("tracking_reacquire_correction_ramp_sec",tracking_reacquire_correction_ramp_sec_,3.0);
    pnh_.param("tracking_reacquire_anchor_max_xy_correction",tracking_reacquire_anchor_max_xy_correction_,2.5);
    pnh_.param("tracking_correction_deadband_xy_m",tracking_correction_deadband_xy_m_,0.0);
    pnh_.param("tracking_correction_deadband_z_m",tracking_correction_deadband_z_m_,0.0);
    pnh_.param("tracking_correction_deadband_yaw_deg",tracking_correction_deadband_yaw_deg_,0.0);
    pnh_.param("tracking_correction_filter_alpha",tracking_correction_filter_alpha_,1.0);
    pnh_.param("tracking_reacquire_correction_filter_alpha",tracking_reacquire_correction_filter_alpha_,1.0);
    pnh_.param("tracking_publish_smoothing_min_sec",tracking_publish_smoothing_min_sec_,0.20);
    pnh_.param("tracking_publish_smoothing_max_sec",tracking_publish_smoothing_max_sec_,1.50);
    pnh_.param("tracking_route_reacquire_publish_max_sec",tracking_route_reacquire_publish_max_sec_,0.65);
    pnh_.param("tracking_publish_max_correction_speed_mps",tracking_publish_max_correction_speed_mps_,6.0);
    pnh_.param("tracking_publish_max_yaw_correction_deg_s",tracking_publish_max_yaw_correction_deg_s_,35.0);
    pnh_.param("tracking_publish_esikf_residual",tracking_publish_esikf_residual_,false);
    pnh_.param("tracking_publish_relative_motion_limit_enable",tracking_publish_relative_motion_limit_enable_,false);
    // Experimental healthy-only, causal velocity observer. It follows real
    // FAST odometry plus recent verified PCD velocity, correcting the *public*
    // pose through a bounded acceleration rather than stacking pose ramps.
    pnh_.param("tracking_publish_healthy_observer_enable",tracking_publish_healthy_observer_enable_,false);
    pnh_.param("tracking_publish_healthy_correction_only_enable",tracking_publish_healthy_correction_only_enable_,false);
    pnh_.param("tracking_publish_healthy_observer_map_velocity_weight",tracking_publish_healthy_observer_map_velocity_weight_,0.40);
    pnh_.param("tracking_publish_healthy_observer_position_gain",tracking_publish_healthy_observer_position_gain_,1.5);
    pnh_.param("tracking_publish_healthy_observer_accel_limit",tracking_publish_healthy_observer_accel_limit_,4.0);
    pnh_.param("tracking_publish_healthy_observer_speed_limit",tracking_publish_healthy_observer_speed_limit_,10.0);
    pnh_.param("tracking_publish_healthy_observer_yaw_rate_limit_deg_s",tracking_publish_healthy_observer_yaw_rate_limit_deg_s_,45.0);
    pnh_.param("tracking_publish_healthy_observer_yaw_accel_limit_deg_s2",tracking_publish_healthy_observer_yaw_accel_limit_deg_s2_,100.0);
    // After a multi-second healthy PCD matching gap the first map anchor can
    // carry metres of accumulated lateral and heading discrepancy. Never
    // apply it as an abrupt sideways velocity during a vehicle turn.
    pnh_.param("tracking_healthy_gap_handoff_enable",tracking_healthy_gap_handoff_enable_,false);
    pnh_.param("tracking_healthy_gap_handoff_trigger_gap_sec",tracking_healthy_gap_handoff_trigger_gap_sec_,1.8);
    pnh_.param("tracking_healthy_gap_handoff_trigger_residual_m",tracking_healthy_gap_handoff_trigger_residual_m_,1.2);
    pnh_.param("tracking_healthy_gap_handoff_trigger_yaw_deg",tracking_healthy_gap_handoff_trigger_yaw_deg_,8.0);
    pnh_.param("tracking_healthy_gap_handoff_duration_sec",tracking_healthy_gap_handoff_duration_sec_,4.0);
    pnh_.param("tracking_healthy_gap_handoff_lateral_speed_mps",tracking_healthy_gap_handoff_lateral_speed_mps_,0.35);
    pnh_.param("tracking_healthy_gap_handoff_yaw_rate_deg_s",tracking_healthy_gap_handoff_yaw_rate_deg_s_,9.0);
    pnh_.param("tracking_healthy_gap_handoff_max_odom_dt_sec",tracking_healthy_gap_handoff_max_odom_dt_sec_,0.90);
    pnh_.param("tracking_healthy_gap_handoff_observer_map_age_sec",tracking_healthy_gap_handoff_observer_map_age_sec_,2.0);
    pnh_.param("tracking_healthy_gap_handoff_extended_local_dt_enable",tracking_healthy_gap_handoff_extended_local_dt_enable_,false);
    // When a rate-limited publisher is active, it alone should absorb map
    // anchor changes. Running a time-resetting recovery ramp as well causes
    // overlapping smoothers and a persistent map-to-published-pose backlog.
    pnh_.param("tracking_publish_single_stage_handoff_enable",tracking_publish_single_stage_handoff_enable_,false);
    pnh_.param("tracking_publish_verified_recovery_catchup_enable",tracking_publish_verified_recovery_catchup_enable_,false);
    pnh_.param("tracking_publish_verified_recovery_max_age_sec",tracking_publish_verified_recovery_max_age_sec_,2.0);
    pnh_.param("tracking_publish_verified_recovery_max_speed_mps",tracking_publish_verified_recovery_max_speed_mps_,6.0);
    pnh_.param("tracking_publish_map_motion_recovery_enable",tracking_publish_map_motion_recovery_enable_,false);
    pnh_.param("tracking_publish_map_motion_recovery_max_age_sec",tracking_publish_map_motion_recovery_max_age_sec_,1.4);
    pnh_.param("tracking_publish_depth_map_motion_enable",tracking_publish_depth_map_motion_enable_,false);
    pnh_.param("tracking_publish_depth_map_motion_max_age_sec",tracking_publish_depth_map_motion_max_age_sec_,1.4);
    pnh_.param("tracking_publish_depth_map_motion_correction_speed_mps",tracking_publish_depth_map_motion_correction_speed_mps_,4.0);
    pnh_.param("tracking_publish_lidar_outage_vertical_guard_enable",tracking_publish_lidar_outage_vertical_guard_enable_,false);
    pnh_.param("tracking_publish_lidar_outage_vertical_guard_max_mps",tracking_publish_lidar_outage_vertical_guard_max_mps_,4.0);
    pnh_.param("tracking_publish_relative_correction_speed_mps",tracking_publish_relative_correction_speed_mps_,1.5);
    pnh_.param("tracking_publish_relative_yaw_rate_deg_s",tracking_publish_relative_yaw_rate_deg_s_,12.0);
    pnh_.param("tracking_publish_relative_turn_sensitive_enable",tracking_publish_relative_turn_sensitive_enable_,false);
    pnh_.param("tracking_publish_relative_turn_rate_threshold_deg_s",tracking_publish_relative_turn_rate_threshold_deg_s_,7.0);
    pnh_.param("tracking_publish_relative_turn_hold_sec",tracking_publish_relative_turn_hold_sec_,8.0);
    pnh_.param("tracking_publish_relative_turn_correction_speed_mps",tracking_publish_relative_turn_correction_speed_mps_,1.5);
    pnh_.param("tracking_publish_relative_turn_yaw_correction_deg_s",tracking_publish_relative_turn_yaw_correction_deg_s_,12.0);
    pnh_.param("tracking_publish_post_failure_quarantine_sec",tracking_publish_post_failure_quarantine_sec_,3.0);
    pnh_.param("tracking_publish_skip_relative_limit_after_critical_recovery",tracking_publish_skip_relative_limit_after_critical_recovery_,false);
    pnh_.param("tracking_publish_post_critical_turn_correction_speed_mps",tracking_publish_post_critical_turn_correction_speed_mps_,1.5);
    pnh_.param("tracking_publish_post_critical_turn_yaw_rate_deg_s",tracking_publish_post_critical_turn_yaw_rate_deg_s_,12.0);
    pnh_.param("tracking_publish_recovery_vertical_catchup_enable",tracking_publish_recovery_vertical_catchup_enable_,false);
    pnh_.param("tracking_publish_recovery_vertical_catchup_threshold_m",tracking_publish_recovery_vertical_catchup_threshold_m_,0.8);
    pnh_.param("tracking_publish_recovery_vertical_catchup_speed_mps",tracking_publish_recovery_vertical_catchup_speed_mps_,4.0);
    pnh_.param("tracking_publish_recovery_vertical_recent_fix_sec",tracking_publish_recovery_vertical_recent_fix_sec_,1.8);
    // Opt-in ESIKF bias observation from two successive verified PCD headings.
    // No map pose is written to the local state; the body gyro bias receives
    // an error-state measurement, so the live local voxel map stays coherent.
    pnh_.param("tracking_publish_gyro_bias_measurement",tracking_publish_gyro_bias_measurement_,false);
    pnh_.param("tracking_publish_anchored_local_measurement",tracking_publish_anchored_local_measurement_,false);
    pnh_.param("depth_anchor_translation_gain",depth_anchor_translation_gain_,0.70);
    pnh_.param("depth_anchor_max_xy_step",depth_anchor_max_xy_step_,0.60);
    pnh_.param("depth_anchor_max_z_step",depth_anchor_max_z_step_,0.15);
    pnh_.param("depth_max_translation_innovation",depth_max_translation_innovation_,4.0);
    pnh_.param("depth_verified_max_speed_mps",depth_verified_max_speed_mps_,8.0);
    pnh_.param("depth_verified_jump_slack_m",depth_verified_jump_slack_m_,1.0);
    pnh_.param("depth_verified_max_gap_sec",depth_verified_max_gap_sec_,3.0);
    // Estimate map<-FAST heading drift from a short history of trusted
    // trajectory correspondences instead of trusting the yaw of one GICP pose.
    // This is the online SE(2) equivalent of fitting two 2-D trajectories.
    pnh_.param("trajectory_yaw_fit_enable",trajectory_yaw_fit_enable_,true);
    pnh_.param("trajectory_yaw_fit_window_sec",trajectory_yaw_fit_window_sec_,12.0);
    pnh_.param("trajectory_yaw_fit_min_baseline_m",trajectory_yaw_fit_min_baseline_m_,4.0);
    pnh_.param("trajectory_yaw_fit_min_pairs",trajectory_yaw_fit_min_pairs_,4);
    pnh_.param("trajectory_yaw_fit_gain",trajectory_yaw_fit_gain_,0.65);
    pnh_.param("trajectory_yaw_fit_max_step_deg",trajectory_yaw_fit_max_step_deg_,1.5);
    pnh_.param("trajectory_yaw_fit_filter_alpha",trajectory_yaw_fit_filter_alpha_,1.0);
    std::vector<double> t,r; nh_.getParam("calibration/lidar_to_base_translation",t); nh_.getParam("calibration/lidar_to_base_rotation",r); if(t.size()!=3||r.size()!=9) throw std::runtime_error("missing calibration/lidar_to_base"); body_from_lidar_=Eigen::Isometry3d::Identity(); for(int i=0;i<3;++i)body_from_lidar_.translation()[i]=t[i];for(int i=0;i<3;++i)for(int j=0;j<3;++j)body_from_lidar_.linear()(i,j)=r[3*i+j];
    loadDb();
    if(gps_soft_prior_enable_)loadGpsKeyframes();
    lock_pub_=nh_.advertise<geometry_msgs::PoseStamped>("/fast_livo2/step3_global_lock",1,true); pose_pub_=nh_.advertise<geometry_msgs::PoseStamped>("/fast_livo2/step3_global_pose",5,true); status_pub_=nh_.advertise<std_msgs::String>("/fast_livo2/step3_status",1,true);
    tracking_pub_=nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/fast_livo2/step3_tracking_correction",3);
    tracking_map_pose_pub_=nh_.advertise<geometry_msgs::PoseStamped>("/fast_livo2/step3_gicp_map_pose",3);
    mh_status_pub_=nh_.advertise<std_msgs::String>("/fast_livo2/step3_mh_status",3,true);
    mh_recovery_candidate_pub_=nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "/fast_livo2/step3_mh_recovery_candidate",3);
    failover_initial_pose_pub_=nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/fast_livo2/step3_failover_initialpose",1,true);
    odom_sub_=nh_.subscribe(odom_topic_,500,&Step3GlobalAcquirer::odomCb,this);
    mode_sub_=nh_.subscribe("/fast_livo2/failsafe_mode",10,&Step3GlobalAcquirer::modeCb,this);
    depth_map_sub_=nh_.subscribe("/fast_livo2/step3_depth_map_pose",3,&Step3GlobalAcquirer::depthMapCb,this);
    // Keep only the newest LiDAR frame. Global registration is deliberately
    // slower than the sensor, so queuing scans makes the tracker operate on
    // stale geometry and blocks odometry callbacks at 1x playback.
    lidar_sub_=nh_.subscribe(lidar_topic_,1,&Step3GlobalAcquirer::lidarCb,this);
    if(mh_enable_ && mh_visual_seed_enable_){
      mh_visual_sub_=nh_.subscribe(mh_visual_candidate_topic_,6,
                                  &Step3GlobalAcquirer::mhVisualCb,this);
      ROS_WARN("[STEP3 MH] EXPERIMENTAL visual-guided GICP proposals ON; GPS unused, visual never directly changes anchor");
    }
    if(gps_soft_prior_enable_){
      gps_sub_=nh_.subscribe(gps_topic_,10,&Step3GlobalAcquirer::gpsCb,this);
      ROS_INFO("[STEP3 GPS] enabled soft coarse candidate preference, radius=%.1fm; LiDAR-only fallback always retained",gps_preference_radius_m_);
    }
    publishStatus("SEARCHING");
  }
 private:
  ros::NodeHandle nh_,pnh_; ros::Subscriber odom_sub_,lidar_sub_,mode_sub_,depth_map_sub_,gps_sub_,mh_visual_sub_;ros::Publisher lock_pub_,pose_pub_,status_pub_,tracking_pub_,tracking_map_pose_pub_,failover_initial_pose_pub_,mh_status_pub_,mh_recovery_candidate_pub_;
  bool mh_enable_=false;
  int mh_sc_candidates_=16,mh_geometric_candidates_=5,mh_max_hypotheses_=4,mh_required_hits_=3;
  double mh_max_fitness_=0.15,mh_association_xy_m_=3.0,mh_association_yaw_deg_=18.0,mh_stale_sec_=7.0;
  double mh_ambiguity_separation_m_=6.0,mh_min_fitness_advantage_=0.035,mh_candidate_diversity_m_=8.0;
  double mh_max_reanchor_m_=4.0,mh_max_reanchor_yaw_deg_=15.0;
  double mh_max_auto_pose_resume_gap_sec_=2.5;
  // Unlike last_published_global_stamp_ (updated by the predictor even while
  // publishing is blocked), this timestamp advances ONLY on a real message.
  std::atomic<double> mh_last_actual_pose_publication_sec_{0.0};
  bool mh_visual_seed_enable_=false,mh_visual_seed_valid_=false;
  double mh_visual_seed_max_age_sec_=1.4,mh_visual_seed_max_xy_variance_m2_=16.0;
  std::string mh_visual_candidate_topic_;
  std::mutex mh_visual_mtx_;
  ros::Time mh_visual_last_stamp_;
  Eigen::Isometry3d mh_visual_last_map_body_=Eigen::Isometry3d::Identity();
  std::atomic<bool> mh_pose_untrusted_{false};
  bool gps_soft_prior_enable_=false, gps_valid_=false;
  std::string gps_topic_;
  double gps_max_age_sec_=0.65,gps_max_covariance_m2_=100.0,gps_preference_radius_m_=25.0;
  double gps_sc_penalty_per_m_=0.0010,gps_fitness_penalty_per_m_=0.0010;
  std::mutex gps_mtx_;
  ros::Time gps_stamp_;
  double gps_lat_=0.0,gps_lon_=0.0;
  std::vector<std::pair<double,double>> keyframe_gps_;
  std::string map_db_,lidar_topic_,odom_topic_; double query_hz_=2,source_voxel_=.35,descriptor_threshold_=.34,gicp_corr_=2,gicp_fitness_=.12,consistency_translation_=1,consistency_yaw_deg_=5;int descriptor_candidates_=12,geometric_candidates_=4,required_consistent_=2;
  double tracking_hz_=1.0,tracking_max_fitness_=0.20,tracking_max_translation_innovation_=3.0,tracking_max_yaw_innovation_deg_=10.0;
  bool tracking_healthy_motion_consistency_gate_=false;
  double tracking_healthy_motion_consistency_slack_m_=0.45;
  double tracking_healthy_motion_consistency_rate_mps_=0.65;
  double tracking_healthy_motion_consistency_max_gap_sec_=5.0;
  double tracking_healthy_motion_consistency_strong_fitness_=0.0;
  bool tracking_healthy_route_lateral_prior_enable_=false;
  double tracking_healthy_route_lateral_prior_gain_=0.65;
  double tracking_healthy_route_lateral_prior_deadband_m_=0.30;
  double tracking_healthy_route_lateral_prior_max_step_m_=1.0;
  double tracking_healthy_route_lateral_prior_max_offset_m_=2.60;
  double tracking_healthy_route_lateral_prior_min_fitness_=0.08;
  double tracking_healthy_route_lateral_prior_max_fitness_=0.14;
  double tracking_relaxed_yaw_max_deg_=18.0,tracking_relaxed_yaw_max_fitness_=0.14,tracking_relaxed_yaw_max_translation_=2.5;
  double tracking_route_continuous_max_fitness_=0.28,tracking_route_continuous_max_translation_=4.0,tracking_route_continuous_max_yaw_deg_=14.0;
  double tracking_gap_predict_after_sec_=1.25,tracking_gap_predict_blend_sec_=0.50,tracking_gap_predict_max_sec_=6.0;
  double tracking_gap_predict_max_speed_mps_=8.0,tracking_gap_predict_max_yaw_rate_deg_s_=55.0,tracking_gap_predict_velocity_alpha_=0.65;
  double tracking_heading_weight_m_per_rad_=3.0;
  double tracking_id_free_window_=8.0,tracking_id_window_per_sec_=2.5,tracking_id_penalty_m_per_keyframe_=0.50;
  double tracking_reacquire_after_sec_=3.0,tracking_reacquire_max_fitness_=0.18,tracking_reacquire_max_translation_innovation_=12.0,tracking_reacquire_max_yaw_innovation_deg_=20.0;
  int tracking_route_reacquire_id_band_=18,tracking_route_reacquire_geometric_candidates_=3;
  double tracking_route_reacquire_id_rate_=4.0,tracking_route_reacquire_descriptor_threshold_=0.45,tracking_route_reacquire_max_fitness_=0.22;
  double tracking_route_reacquire_max_speed_mps_=12.0,tracking_route_reacquire_max_yaw_rate_deg_s_=60.0;
  bool tracking_route_reacquire_enabled_=false;
  double tracking_global_reacquire_after_sec_=6.0,tracking_global_reacquire_unrestricted_after_sec_=30.0,tracking_global_reacquire_hz_=0.33,tracking_global_reacquire_descriptor_threshold_=0.38,tracking_global_reacquire_max_fitness_=0.14;
  bool tracking_global_reacquire_physical_gate_=true;
  double tracking_global_reacquire_position_slack_m_=3.0,tracking_global_reacquire_vertical_slack_m_=3.0;
  double tracking_global_reacquire_recovery_vertical_rate_mps_=0.0,tracking_global_reacquire_recovery_vertical_max_slack_m_=3.0;
  double tracking_global_reacquire_predicted_pose_weight_=0.001;
  double tracking_global_reacquire_consistency_translation_=3.0,tracking_global_reacquire_consistency_yaw_deg_=12.0;
  double tracking_global_reacquire_max_speed_mps_=10.0,tracking_global_reacquire_max_yaw_rate_deg_s_=45.0,tracking_global_reacquire_max_id_rate_=6.0;
  double tracking_global_reacquire_hard_reset_translation_=15.0,tracking_global_reacquire_ramp_sec_=3.0;
  double fast_local_max_speed_mps_=15.0,fast_local_max_vertical_speed_mps_=6.0,fast_local_max_pose_gap_sec_=1.0;
  int fast_local_recover_samples_=10;
  double fast_odom_missing_grace_sec_=1.0;
  int fast_odom_missing_scans_before_unstable_=6;
  int fast_odom_missing_scans_=0;
  double map_fix_fallback_follow_tau_sec_=0.35,map_fix_fallback_max_extrapolation_sec_=1.2,map_fix_fallback_max_speed_mps_=12.0,map_fix_fallback_max_yaw_rate_deg_s_=90.0;
  double tracking_anchor_translation_gain_=1.00,tracking_anchor_max_xy_step_=0.80,tracking_anchor_max_z_step_=0.15;
  double tracking_anchor_yaw_gain_=0.75,tracking_anchor_max_yaw_step_deg_=1.5;
  double tracking_correction_ramp_sec_=2.0,tracking_reacquire_correction_ramp_sec_=3.0;
  double tracking_reacquire_anchor_max_xy_correction_=2.5;
  double tracking_correction_deadband_xy_m_=0.0,tracking_correction_deadband_z_m_=0.0,tracking_correction_deadband_yaw_deg_=0.0;
  double tracking_correction_filter_alpha_=1.0,tracking_reacquire_correction_filter_alpha_=1.0;
  double tracking_publish_smoothing_min_sec_=0.20,tracking_publish_smoothing_max_sec_=1.50;
  double tracking_route_reacquire_publish_max_sec_=0.65;
  double tracking_publish_max_correction_speed_mps_=6.0,tracking_publish_max_yaw_correction_deg_s_=35.0;
  double depth_anchor_translation_gain_=0.70,depth_anchor_max_xy_step_=0.60,depth_anchor_max_z_step_=0.15,depth_max_translation_innovation_=4.0;
  double depth_verified_max_speed_mps_=8.0,depth_verified_jump_slack_m_=1.0,depth_verified_max_gap_sec_=3.0;
  double lidar_recovery_bridge_sec_=2.5;
  std::mutex depth_verified_mtx_;
  bool depth_verified_pose_valid_=false;
  Eigen::Isometry3d depth_verified_last_pose_=Eigen::Isometry3d::Identity();
  ros::Time depth_verified_last_stamp_;
  bool trajectory_yaw_fit_enable_=true;
  double trajectory_yaw_fit_window_sec_=12.0,trajectory_yaw_fit_min_baseline_m_=4.0;
  double trajectory_yaw_fit_gain_=0.65,trajectory_yaw_fit_max_step_deg_=1.5;
  double trajectory_yaw_fit_filter_alpha_=1.0;
  int trajectory_yaw_fit_min_pairs_=4;
  bool trajectory_yaw_filter_valid_=false;
  double trajectory_yaw_filtered_=0.0;
  int tracking_candidates_=3;
  int tracking_submap_cache_slots_=0;
  int tracking_gicp_max_iterations_=25;
  std::list<int> submap_lru_;
  struct CachedSubmap{Cloud::Ptr cloud;std::list<int>::iterator lru;};
  std::unordered_map<int,CachedSubmap> submap_cache_;
  uint64_t submap_cache_hits_=0,submap_cache_misses_=0;
  bool tracking_deferred_scan_enable_=false,tracking_continuity_retry_enable_=false;
  bool tracking_continuity_retry_physical_only_enable_=false;
  bool tracking_healthy_continuity_reseed_enable_=false;
  double tracking_healthy_continuity_reseed_min_seed_delta_m_=0.4;
  double tracking_healthy_continuity_reseed_max_fitness_=0.14;
  double tracking_continuity_retry_max_age_sec_=4.5;
  double tracking_deferred_scan_max_age_sec_=1.8;
  std::deque<sensor_msgs::PointCloud2ConstPtr> deferred_lidar_;
  size_t deferred_scans_processed_=0,deferred_scans_expired_=0;
  int tracking_global_reacquire_descriptor_candidates_=20,tracking_global_reacquire_geometric_candidates_=4,tracking_global_reacquire_required_consistent_=2;
  int tracking_global_reacquire_after_misses_=3;
  bool tracking_update_anchor_=true,tracking_publish_esikf_residual_=false;
  bool tracking_publish_gyro_bias_measurement_=false;
  bool tracking_publish_relative_motion_limit_enable_=false;
  bool tracking_publish_healthy_observer_enable_=false;
  bool tracking_publish_healthy_correction_only_enable_=false;
  double tracking_publish_healthy_observer_map_velocity_weight_=0.40;
  double tracking_publish_healthy_observer_position_gain_=1.5;
  double tracking_publish_healthy_observer_accel_limit_=4.0;
  double tracking_publish_healthy_observer_speed_limit_=10.0;
  double tracking_publish_healthy_observer_yaw_rate_limit_deg_s_=45.0;
  double tracking_publish_healthy_observer_yaw_accel_limit_deg_s2_=100.0;
  bool tracking_healthy_gap_handoff_enable_=false;
  double tracking_healthy_gap_handoff_trigger_gap_sec_=1.8;
  double tracking_healthy_gap_handoff_trigger_residual_m_=1.2;
  double tracking_healthy_gap_handoff_trigger_yaw_deg_=8.0;
  double tracking_healthy_gap_handoff_duration_sec_=4.0;
  double tracking_healthy_gap_handoff_lateral_speed_mps_=0.35;
  double tracking_healthy_gap_handoff_yaw_rate_deg_s_=9.0;
  double tracking_healthy_gap_handoff_max_odom_dt_sec_=0.90;
  double tracking_healthy_gap_handoff_observer_map_age_sec_=2.0;
  bool tracking_healthy_gap_handoff_extended_local_dt_enable_=false;
  std::atomic<double> healthy_gap_handoff_start_sec_{0.0};
  bool tracking_publish_single_stage_handoff_enable_=false;
  bool tracking_publish_verified_recovery_catchup_enable_=false;
  double tracking_publish_verified_recovery_max_age_sec_=2.0;
  double tracking_publish_verified_recovery_max_speed_mps_=6.0;
  bool tracking_publish_map_motion_recovery_enable_=false;
  double tracking_publish_map_motion_recovery_max_age_sec_=1.4;
  bool tracking_publish_depth_map_motion_enable_=false;
  double tracking_publish_depth_map_motion_max_age_sec_=1.4;
  double tracking_publish_depth_map_motion_correction_speed_mps_=4.0;
  bool tracking_publish_lidar_outage_vertical_guard_enable_=false;
  double tracking_publish_lidar_outage_vertical_guard_max_mps_=4.0;
  // Sensor-failure poses may come from RGB-D, map-hold, or a rebased FAST
  // local frame. Never use FAST relative motion as their publication model.
  std::atomic<bool> publish_full_sensor_mode_{true};
  std::atomic<double> publish_full_sensor_mode_since_sec_{0.0};
  // Enable faster Z-only reconvergence *only* after a real LiDAR/IMU outage,
  // and only while a recent GICP map correction supports the target altitude.
  std::atomic<bool> publish_critical_sensor_recovered_{false};
  std::atomic<double> last_verified_map_fix_stamp_sec_{0.0};
  std::mutex verified_vertical_mtx_;
  bool verified_vertical_valid_=false;
  double verified_vertical_z_m_=0.0;
  double verified_vertical_v_mps_=0.0;
  double verified_vertical_stamp_sec_=0.0;
  bool tracking_publish_recovery_vertical_catchup_enable_=false;
  double tracking_publish_recovery_vertical_catchup_threshold_m_=0.8;
  double tracking_publish_recovery_vertical_catchup_speed_mps_=4.0;
  double tracking_publish_recovery_vertical_recent_fix_sec_=1.8;
  double tracking_publish_relative_correction_speed_mps_=1.5;
  double tracking_publish_relative_yaw_rate_deg_s_=12.0;
  bool tracking_publish_relative_turn_sensitive_enable_=false;
  bool tracking_publish_skip_relative_limit_after_critical_recovery_=false;
  double tracking_publish_post_critical_turn_correction_speed_mps_=1.5;
  double tracking_publish_post_critical_turn_yaw_rate_deg_s_=12.0;
  double tracking_publish_relative_turn_rate_threshold_deg_s_=7.0;
  double tracking_publish_relative_turn_hold_sec_=8.0;
  double tracking_publish_relative_turn_correction_speed_mps_=1.5;
  double tracking_publish_relative_turn_yaw_correction_deg_s_=12.0;
  double tracking_publish_post_failure_quarantine_sec_=3.0;
  ros::Time publish_last_sharp_turn_stamp_;
  bool publish_previous_local_valid_=false;
  Eigen::Isometry3d publish_previous_local_pose_=Eigen::Isometry3d::Identity();
  ros::Time publish_previous_local_stamp_;
  bool tracking_publish_anchored_local_measurement_=false;
  bool anchored_local_initial_lock_valid_=false;
  Eigen::Isometry3d anchored_local_initial_map_fast_=Eigen::Isometry3d::Identity();
  bool gyro_bias_heading_pair_valid_=false;
  Eigen::Isometry3d gyro_bias_previous_map_body_=Eigen::Isometry3d::Identity();
  Eigen::Isometry3d gyro_bias_previous_fast_body_=Eigen::Isometry3d::Identity();
  ros::Time gyro_bias_previous_stamp_;
  Eigen::Isometry3d body_from_lidar_=Eigen::Isometry3d::Identity(); std::vector<Descriptor> desc_; std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>> map_poses_;
  std::mutex odom_mtx_;
  std::mutex anchor_mtx_;
  std::mutex mode_mtx_;
  std::mutex lidar_job_mtx_;
  std::mutex tracking_motion_mtx_;
  std::mutex trajectory_fit_mtx_;
  std::mutex recovery_bridge_mtx_;
  // [stamp_s, fast_x, fast_y, map_x, map_y]
  std::deque<std::array<double,5>> trajectory_fit_pairs_;
  std::string failsafe_mode_="FULL_LIVO";
  std::deque<std::pair<ros::Time,Eigen::Isometry3d>,Eigen::aligned_allocator<std::pair<ros::Time,Eigen::Isometry3d>>> odom_;
  ros::Time last_query_,last_tracking_query_,last_global_reacquire_query_;
  std::atomic<bool> locked_{false};
  Eigen::Isometry3d map_from_fast_=Eigen::Isometry3d::Identity(),candidate_anchor_=Eigen::Isometry3d::Identity();
  int consistent_=0;
  int last_tracking_id_=-1;
  int tracking_misses_=0;
  ros::Time last_tracking_accept_stamp_;
  bool last_tracking_map_valid_=false;
  Eigen::Isometry3d last_tracking_map_body_=Eigen::Isometry3d::Identity();
  bool last_tracking_fast_valid_=false;
  Eigen::Isometry3d last_tracking_fast_body_=Eigen::Isometry3d::Identity();
  ros::Time last_tracking_map_stamp_;
  bool tracking_motion_valid_=false;
  Eigen::Isometry3d tracking_motion_pose_=Eigen::Isometry3d::Identity();
  ros::Time tracking_motion_stamp_;
  Eigen::Vector3d tracking_motion_velocity_=Eigen::Vector3d::Zero();
  double tracking_motion_yaw_rate_=0.0;
  int global_reacquire_consistent_=0;
  Eigen::Isometry3d global_reacquire_candidate_anchor_=Eigen::Isometry3d::Identity();
  bool global_reacquire_last_map_valid_=false;
  Eigen::Isometry3d global_reacquire_last_map_body_=Eigen::Isometry3d::Identity();
  ros::Time global_reacquire_last_map_stamp_;
  int global_reacquire_last_id_=-1;
  // Used only by lidarCb under lidar_job_mtx_. Each hypothesis represents a
  // distinct map<-FAST transform supported by temporally separate LiDAR scans.
  struct MhTrack {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Isometry3d anchor=Eigen::Isometry3d::Identity();
    Eigen::Isometry3d map_body=Eigen::Isometry3d::Identity();
    ros::Time last_stamp;
    int id=-1,hits=0;
    double mean_fitness=1e9;
  };
  std::vector<MhTrack,Eigen::aligned_allocator<MhTrack>> mh_tracks_;
  ros::Time mh_last_stamp_;
  std::atomic<bool> fast_local_unstable_{false};
  std::atomic<bool> fast_local_recovery_ready_{false};
  bool fast_local_prev_valid_=false;
  ros::Time fast_local_prev_stamp_;
  Eigen::Isometry3d fast_local_prev_pose_=Eigen::Isometry3d::Identity();
  int fast_local_stable_samples_=0;
  bool map_fix_fallback_active_=false;
  bool map_fix_target_valid_=false;
  Eigen::Isometry3d map_fix_target_pose_=Eigen::Isometry3d::Identity();
  ros::Time map_fix_target_stamp_;
  Eigen::Vector3d map_fix_velocity_=Eigen::Vector3d::Zero();
  double map_fix_yaw_rate_=0.0;
  bool correction_ramp_active_=false;
  ros::Time correction_ramp_start_stamp_;
  double correction_ramp_duration_sec_=2.0;
  double correction_ramp_last_fraction_=0.0;
  Eigen::Vector3d correction_ramp_translation_=Eigen::Vector3d::Zero();
  double correction_ramp_yaw_=0.0;
  bool correction_filter_valid_=false;
  Eigen::Vector3d correction_filtered_translation_=Eigen::Vector3d::Zero();
  double correction_filtered_yaw_=0.0;
  // Large global re-localizations repair the internal map<-FAST anchor
  // immediately so subsequent matching uses the correct search centre. The
  // externally published pose is eased from the old pose to the corrected one
  // over a short window, avoiding a visible/control snap.
  bool publish_recovery_active_=false;
  bool publish_recovery_pending_=false;
  ros::Time publish_recovery_start_stamp_;
  double publish_recovery_duration_sec_=3.0;
  Eigen::Vector3d publish_recovery_translation_=Eigen::Vector3d::Zero();
  double publish_recovery_yaw_=0.0;
  bool last_published_global_pose_valid_=false;
  bool healthy_observer_velocity_valid_=false;
  Eigen::Vector3d healthy_observer_velocity_=Eigen::Vector3d::Zero();
  double healthy_observer_yaw_rate_=0.0;
  Eigen::Isometry3d last_published_global_pose_=Eigen::Isometry3d::Identity();
  ros::Time last_published_global_stamp_;
  bool lidar_recovery_bridge_active_=false;
  ros::Time lidar_recovery_bridge_until_;

  bool lidarRecoveryBridgeActive(const ros::Time& stamp){
    std::lock_guard<std::mutex> l(recovery_bridge_mtx_);
    if(!lidar_recovery_bridge_active_)return false;
    if(!lidar_recovery_bridge_until_.isZero()&&stamp>lidar_recovery_bridge_until_){
      lidar_recovery_bridge_active_=false;
      return false;
    }
    return true;
  }

  void publishStatus(const std::string&s){std_msgs::String m;m.data=s;status_pub_.publish(m);ROS_INFO_STREAM("[STEP3] "<<s);}
  void updateTrackingMotionModel(const Eigen::Isometry3d& map_body,const ros::Time& stamp){
    if(stamp.isZero()||!map_body.matrix().allFinite())return;
    std::lock_guard<std::mutex> l(tracking_motion_mtx_);
    if(tracking_motion_valid_){
      const double dt=(stamp-tracking_motion_stamp_).toSec();
      if(dt>0.05&&dt<4.0){
        Eigen::Vector3d v=(map_body.translation()-tracking_motion_pose_.translation())/dt;
        double speed=v.head<2>().norm();
        if(std::isfinite(speed)&&speed>tracking_gap_predict_max_speed_mps_&&speed>1e-9)
          v.head<2>()*=tracking_gap_predict_max_speed_mps_/speed;
        // The optional healthy observer needs actual map height motion too;
        // zeroing Z here makes it unable to counter FAST vertical drift.
        if(!tracking_publish_healthy_observer_enable_)v.z()=0.0;
        const double a=std::max(0.0,std::min(1.0,tracking_gap_predict_velocity_alpha_));
        if(v.allFinite())tracking_motion_velocity_=(1.0-a)*tracking_motion_velocity_+a*v;
        double wr=wrap(yawOf(map_body)-yawOf(tracking_motion_pose_))/dt;
        const double max_wr=tracking_gap_predict_max_yaw_rate_deg_s_*M_PI/180.0;
        if(std::isfinite(wr)){
          wr=std::max(-max_wr,std::min(max_wr,wr));
          tracking_motion_yaw_rate_=(1.0-a)*tracking_motion_yaw_rate_+a*wr;
        }
      }
    }else{
      tracking_motion_velocity_.setZero();
      tracking_motion_yaw_rate_=0.0;
    }
    tracking_motion_pose_=map_body;
    tracking_motion_stamp_=stamp;
    tracking_motion_valid_=true;
  }
  bool trackingMotionPrediction(const ros::Time& stamp,Eigen::Isometry3d& pose,double& age){
    std::lock_guard<std::mutex> l(tracking_motion_mtx_);
    if(!tracking_motion_valid_||tracking_motion_stamp_.isZero())return false;
    age=(stamp-tracking_motion_stamp_).toSec();
    if(!std::isfinite(age)||age<0.0)return false;
    const double dt=std::min(age,std::max(0.1,tracking_gap_predict_max_sec_));
    pose=tracking_motion_pose_;
    pose.translation()+=dt*tracking_motion_velocity_;
    if(std::abs(tracking_motion_yaw_rate_)>1e-9)
      pose.linear()=Eigen::AngleAxisd(dt*tracking_motion_yaw_rate_,Eigen::Vector3d::UnitZ()).toRotationMatrix()*pose.linear();
    return pose.matrix().allFinite();
  }
  void resetTrajectoryFit(){
    std::lock_guard<std::mutex> l(trajectory_fit_mtx_);
    trajectory_fit_pairs_.clear();
    trajectory_yaw_filter_valid_=false;
    trajectory_yaw_filtered_=0.0;
  }
  void addTrajectoryFitPair(const ros::Time& stamp,const Eigen::Isometry3d& fast_body,
                            const Eigen::Isometry3d& map_body){
    if(!trajectory_yaw_fit_enable_||stamp.isZero())return;
    std::lock_guard<std::mutex> l(trajectory_fit_mtx_);
    trajectory_fit_pairs_.push_back({stamp.toSec(),fast_body.translation().x(),fast_body.translation().y(),
                                     map_body.translation().x(),map_body.translation().y()});
    while(!trajectory_fit_pairs_.empty()&&
          stamp.toSec()-trajectory_fit_pairs_.front()[0]>trajectory_yaw_fit_window_sec_)
      trajectory_fit_pairs_.pop_front();
    while(trajectory_fit_pairs_.size()>40)trajectory_fit_pairs_.pop_front();
  }
  bool fittedTrajectoryYaw(double& yaw,double& fast_path,double& map_path){
    std::lock_guard<std::mutex> l(trajectory_fit_mtx_);
    if(!trajectory_yaw_fit_enable_||
       trajectory_fit_pairs_.size()<static_cast<size_t>(std::max(2,trajectory_yaw_fit_min_pairs_)))return false;
    fast_path=0.0;map_path=0.0;
    for(size_t i=1;i<trajectory_fit_pairs_.size();++i){const auto&p=trajectory_fit_pairs_[i];const auto&u=trajectory_fit_pairs_[i-1];fast_path+=std::hypot(p[1]-u[1],p[2]-u[2]);map_path+=std::hypot(p[3]-u[3],p[4]-u[4]);}
    if(std::min(fast_path,map_path)<trajectory_yaw_fit_min_baseline_m_)return false;

    // Robust 2D Procrustes HEADING fit. Translation is intentionally not used
    // to update map<-FAST here: the newest verified PCD pose remains the
    // position authority. We only estimate how the recent FAST trajectory must
    // rotate so its incoming direction overlaps the map trajectory.
    std::vector<double> w(trajectory_fit_pairs_.size(),1.0);
    auto solve=[&](double& out_yaw,Eigen::Vector2d& out_t)->bool{
      double sw=0,fx=0,fy=0,mx=0,my=0;
      for(size_t i=0;i<trajectory_fit_pairs_.size();++i){const auto&p=trajectory_fit_pairs_[i];const double wi=w[i];sw+=wi;fx+=wi*p[1];fy+=wi*p[2];mx+=wi*p[3];my+=wi*p[4];}
      if(sw<2.0)return false;fx/=sw;fy/=sw;mx/=sw;my/=sw;
      double c=0,sn=0;
      for(size_t i=0;i<trajectory_fit_pairs_.size();++i){const auto&p=trajectory_fit_pairs_[i];const double wi=w[i];const double px=p[1]-fx,py=p[2]-fy,qx=p[3]-mx,qy=p[4]-my;c+=wi*(px*qx+py*qy);sn+=wi*(px*qy-py*qx);}
      if(std::hypot(c,sn)<1e-6)return false;
      out_yaw=std::atan2(sn,c);const double co=std::cos(out_yaw),si=std::sin(out_yaw);out_t=Eigen::Vector2d(mx-(co*fx-si*fy),my-(si*fx+co*fy));return std::isfinite(out_yaw)&&out_t.allFinite();
    };
    Eigen::Vector2d t=Eigen::Vector2d::Zero();double raw_yaw=0.0;
    if(!solve(raw_yaw,t))return false;
    std::vector<double> r; r.reserve(trajectory_fit_pairs_.size());
    for(const auto&p:trajectory_fit_pairs_){const double co=std::cos(raw_yaw),si=std::sin(raw_yaw);const Eigen::Vector2d pred(co*p[1]-si*p[2]+t.x(),si*p[1]+co*p[2]+t.y());r.push_back((pred-Eigen::Vector2d(p[3],p[4])).norm());}
    std::vector<double> sorted=r;std::sort(sorted.begin(),sorted.end());const double med=sorted[sorted.size()/2];const double huber=std::max(0.30,2.5*med);
    for(size_t i=0;i<w.size();++i)w[i]=(r[i]<=huber||r[i]<1e-9)?1.0:huber/r[i];
    if(!solve(raw_yaw,t))return false;

    // Filter the fitted heading target itself. map<-FAST should change slowly;
    // this rejects one-frame GICP orientation noise while the existing max-step
    // limiter below still guarantees bounded correction rate.
    const double alpha=std::max(0.01,std::min(1.0,trajectory_yaw_fit_filter_alpha_));
    if(!trajectory_yaw_filter_valid_){trajectory_yaw_filtered_=raw_yaw;trajectory_yaw_filter_valid_=true;}
    else trajectory_yaw_filtered_=wrap(trajectory_yaw_filtered_+alpha*wrap(raw_yaw-trajectory_yaw_filtered_));
    yaw=trajectory_yaw_filtered_;
    return std::isfinite(yaw);
  }
  Eigen::Isometry3d smartAnchorAtCurrentPoint(const Eigen::Isometry3d& current_anchor,
                                               const Eigen::Isometry3d& fast_body,
                                               const Eigen::Vector3d& target_map_position,
                                               double fallback_desired_yaw,
                                               bool allow_fallback_yaw,
                                               bool& fit_used,double& applied_yaw){
    double desired_yaw=0,fp=0,mp=0;fit_used=fittedTrajectoryYaw(desired_yaw,fp,mp);
    if(!fit_used){
      if(!allow_fallback_yaw){desired_yaw=yawOf(current_anchor);}
      else desired_yaw=fallback_desired_yaw;
    }
    const double current_yaw=yawOf(current_anchor);
    const double gain=fit_used?trajectory_yaw_fit_gain_:tracking_anchor_yaw_gain_;
    const double max_step=(fit_used?trajectory_yaw_fit_max_step_deg_:tracking_anchor_max_yaw_step_deg_)*M_PI/180.0;
    applied_yaw=std::max(-max_step,std::min(max_step,gain*wrap(desired_yaw-current_yaw)));
    Eigen::Isometry3d out=current_anchor;
    out.linear()=Eigen::AngleAxisd(applied_yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix()*current_anchor.linear();
    // Rotate map<-FAST about the CURRENT vehicle location.  Recompute the
    // parent translation so the current pose stays on its trusted map point;
    // only future local motion changes direction. This avoids zig-zag/arc jumps.
    out.translation()=target_map_position-out.linear()*fast_body.translation();
    if(fit_used)ROS_INFO_THROTTLE(0.5,"[STEP3 SE2] trajectory yaw fit %.2fdeg apply %.2fdeg paths=%.1f/%.1fm",
        desired_yaw*180.0/M_PI,applied_yaw*180.0/M_PI,fp,mp);
    return out;
  }
  void updateVerifiedVerticalReference(const ros::Time& stamp,const Eigen::Isometry3d& map_body){
    const double when=stamp.toSec();
    const double z=map_body.translation().z();
    if(!std::isfinite(when)||!std::isfinite(z))return;
    std::lock_guard<std::mutex> l(verified_vertical_mtx_);
    const double dt=when-verified_vertical_stamp_sec_;
    if(verified_vertical_valid_&&dt>0.2&&dt<2.5){
      const double raw_v=(z-verified_vertical_z_m_)/dt;
      if(std::isfinite(raw_v)){
        const double bounded=std::max(-6.0,std::min(6.0,raw_v));
        verified_vertical_v_mps_=0.65*bounded+0.35*verified_vertical_v_mps_;
      }
    }else{
      verified_vertical_v_mps_=0.0;
    }
    verified_vertical_z_m_=z;
    verified_vertical_stamp_sec_=when;
    verified_vertical_valid_=true;
  }
  void loadDb(){
    if(map_db_.empty())throw std::runtime_error("~map_db empty"); std::ifstream pf(map_db_+"/pose.json"); if(!pf)throw std::runtime_error("pose.json missing"); double tx,ty,tz,qw,qx,qy,qz; while(pf>>tx>>ty>>tz>>qw>>qx>>qy>>qz){Eigen::Quaterniond q(qw,qx,qy,qz);q.normalize();Eigen::Isometry3d T=Eigen::Isometry3d::Identity();T.linear()=q.toRotationMatrix();T.translation()<<tx,ty,tz;map_poses_.push_back(T);} std::ifstream f(map_db_+"/cache/scan_context.bin",std::ios::binary); if(!f)throw std::runtime_error("scan_context.bin missing"); char magic[8];uint32_t n,r,s;f.read(magic,8);f.read((char*)&n,4);f.read((char*)&r,4);f.read((char*)&s,4);if(std::string(magic,8)!="SCDBV001"||r!=kRings||s!=kSectors||n!=map_poses_.size())throw std::runtime_error("scan context cache mismatch");desc_.resize(n);for(uint32_t i=0;i<n;++i){for(auto&x:desc_[i].v)f.read((char*)&x,sizeof(double));for(int rr=0;rr<kRings;++rr){double sum=0;for(int c=0;c<kSectors;++c)sum+=desc_[i].v[rr*kSectors+c];desc_[i].ring[rr]=sum/kSectors;}}ROS_INFO("[STEP3] loaded %zu map descriptors",desc_.size());
  }
  void loadGpsKeyframes(){
    std::ifstream f(map_db_+"/keyframes.csv");
    if(!f){ROS_WARN("[STEP3 GPS] missing keyframes.csv: LiDAR-only retrieval");return;}
    std::string line;
    std::getline(f,line); // exporter contract: gps_lat index 10, gps_lon 11
    std::vector<std::pair<double,double>> coordinates;
    while(std::getline(f,line)){
      std::stringstream ss(line);std::vector<std::string> cols;std::string c;
      while(std::getline(ss,c,','))cols.push_back(c);
      double lat=std::numeric_limits<double>::quiet_NaN(),lon=lat;
      try{if(cols.size()>=12){lat=std::stod(cols[10]);lon=std::stod(cols[11]);}}catch(...){}
      coordinates.push_back({lat,lon});
    }
    if(coordinates.size()!=map_poses_.size()){
      ROS_ERROR("[STEP3 GPS] keyframe GPS size mismatch %zu vs %zu; GPS disabled",
                coordinates.size(),map_poses_.size());return;
    }
    keyframe_gps_=std::move(coordinates);
    ROS_INFO("[STEP3 GPS] %zu map GPS entries loaded; only soft preference, no map pose authority",
             keyframe_gps_.size());
  }
  void gpsCb(const sensor_msgs::NavSatFixConstPtr& m){
    if(!m || m->status.status<sensor_msgs::NavSatStatus::STATUS_FIX ||
       !std::isfinite(m->latitude)||!std::isfinite(m->longitude)||
       std::abs(m->latitude)>90||std::abs(m->longitude)>180)return;
    double vx=m->position_covariance[0],vy=m->position_covariance[4];
    if(!std::isfinite(vx)||!std::isfinite(vy)||vx<=0||vy<=0||
       vx>gps_max_covariance_m2_||vy>gps_max_covariance_m2_)return;
    std::lock_guard<std::mutex> lock(gps_mtx_);
    gps_stamp_=m->header.stamp;gps_lat_=m->latitude;gps_lon_=m->longitude;gps_valid_=true;
  }
  std::vector<double> gpsDistancesAt(const ros::Time& stamp){
    if(!gps_soft_prior_enable_||keyframe_gps_.size()!=map_poses_.size())return {};
    double lat,lon;ros::Time last;
    {std::lock_guard<std::mutex> lock(gps_mtx_);
      if(!gps_valid_)return {};
      lat=gps_lat_;lon=gps_lon_;last=gps_stamp_;}
    const double age=(stamp-last).toSec();
    if(!std::isfinite(age)||age < -0.2 || age>gps_max_age_sec_)return {};
    const double longitude_scale=111320.0*std::cos(lat*M_PI/180.0);
    std::vector<double> distances;distances.reserve(keyframe_gps_.size());
    for(const auto& p:keyframe_gps_){
      if(!std::isfinite(p.first)||!std::isfinite(p.second))distances.push_back(1e6);
      else distances.push_back(std::hypot((p.first-lat)*111132.0,
                                          (p.second-lon)*longitude_scale));
    }
    return distances;
  }
  double gpsPenalty(int id,const std::vector<double>& ds,double per_m,double cap) const{
    if(ds.empty()||id<0||id>=(int)ds.size())return 0.0;
    return std::min(cap,std::max(0.0,ds[id]-gps_preference_radius_m_)*per_m);
  }
  void modeCb(const std_msgs::String::ConstPtr& m){
    if(!m)return;
    bool entering_lidar_fail=false,exiting_lidar_fail=false,exiting_critical_fail=false;
    {
      std::lock_guard<std::mutex> l(mode_mtx_);
      const bool was_fail=(failsafe_mode_=="VI_LIDAR_FAILED"||failsafe_mode_=="IMU_ONLY_BRIDGE");
      const bool was_critical_fail=was_fail||failsafe_mode_=="LV_IMU_FAILED";
      const bool now_fail=(m->data=="VI_LIDAR_FAILED"||m->data=="IMU_ONLY_BRIDGE");
      entering_lidar_fail=now_fail&&!was_fail;
      exiting_lidar_fail=was_fail&&!now_fail&&m->data=="FULL_LIVO";
      exiting_critical_fail=was_critical_fail&&m->data=="FULL_LIVO";
      failsafe_mode_=m->data;
    }
    if(exiting_critical_fail)publish_critical_sensor_recovered_.store(true);
    const bool now_full=(m->data=="FULL_LIVO");
    const bool was_full=publish_full_sensor_mode_.exchange(now_full);
    if(now_full&&!was_full)
      publish_full_sensor_mode_since_sec_.store(ros::Time::now().toSec());
    if(m->data!="FULL_LIVO")gyro_bias_heading_pair_valid_=false;
    if(entering_lidar_fail){
      std::lock_guard<std::mutex> dl(depth_verified_mtx_);
      depth_verified_pose_valid_=false;
      depth_verified_last_pose_=Eigen::Isometry3d::Identity();
      depth_verified_last_stamp_=ros::Time(0);
      ROS_INFO("[STEP3 DEPTH] LiDAR outage entered: waiting for verified visual/depth map fix");
    }
    if(exiting_lidar_fail){
      const ros::Time now=ros::Time::now();
      {
        std::lock_guard<std::mutex> rl(recovery_bridge_mtx_);
        lidar_recovery_bridge_active_=true;
        lidar_recovery_bridge_until_=now+ros::Duration(std::max(0.5,lidar_recovery_bridge_sec_));
      }

      // The LiDAR tracker has intentionally been silent throughout the outage,
      // so its last accepted LiDAR keyframe can be tens of seconds old.  Do not
      // interpret that expected gap as a localization loss and launch a whole-
      // map descriptor search.  The RGB-D/depth path has kept the published map
      // pose alive; returning LiDAR must refine locally around that pose first.
      last_tracking_accept_stamp_=now;
      last_tracking_id_=-1;
      tracking_misses_=0;
      global_reacquire_consistent_=0;
      global_reacquire_last_map_valid_=false;
      last_global_reacquire_query_=ros::Time();
      fast_local_unstable_.store(false);
      fast_local_recovery_ready_.store(false);
      fast_local_stable_samples_=0;

      Eigen::Isometry3d depth_pose=Eigen::Isometry3d::Identity();
      ros::Time depth_stamp;
      bool have_depth=false;
      {
        std::lock_guard<std::mutex> dl(depth_verified_mtx_);
        have_depth=depth_verified_pose_valid_;
        if(have_depth){depth_pose=depth_verified_last_pose_;depth_stamp=depth_verified_last_stamp_;}
      }
      if(have_depth&&!depth_stamp.isZero()){
        updateTrackingMotionModel(depth_pose,depth_stamp);
        last_tracking_map_body_=depth_pose;
        last_tracking_map_stamp_=depth_stamp;
        last_tracking_map_valid_=true;
      }
      resetTrajectoryFit();
      ROS_WARN("[STEP3 RECOVERY-BRIDGE] LiDAR returned; using current RGB-D/depth map pose as local-search seed for %.1fs (full-map reacquisition suppressed)",
               std::max(0.5,lidar_recovery_bridge_sec_));
    }
  }
  void depthMapCb(const geometry_msgs::PoseStamped::ConstPtr& m){
    if(!m||!locked_.load())return;
    {std::lock_guard<std::mutex>l(mode_mtx_);if(failsafe_mode_!="VI_LIDAR_FAILED"&&failsafe_mode_!="IMU_ONLY_BRIDGE")return;}
    // Never confuse causal RGB-D propagation with an independent saved-map
    // observation. V14/V15 collapsed both to frame_id="map" and Step-3 fed
    // its own prediction back into the global anchor, causing positive feedback.
    const bool verified_depth=(m->header.frame_id=="map_depth_gicp");
    if(!verified_depth){
      if(m->header.frame_id=="map_rgbd_predict")
        ROS_INFO_THROTTLE(1.0,"[STEP3 DEPTH] RGB-D prediction kept relative-only; waiting for verified map fix");
      else
        ROS_WARN_THROTTLE(1.0,"[STEP3 DEPTH] ignoring unclassified failover pose frame '%s'",m->header.frame_id.c_str());
      return;
    }
    Eigen::Quaterniond q(m->pose.orientation.w,m->pose.orientation.x,m->pose.orientation.y,m->pose.orientation.z);
    if(q.norm()<1e-8)return;q.normalize();
    Eigen::Isometry3d meas=Eigen::Isometry3d::Identity();meas.linear()=q.toRotationMatrix();meas.translation()<<m->pose.position.x,m->pose.position.y,m->pose.position.z;
    if(!meas.matrix().allFinite())return;
    // A single repeated-geometry lock must not teleport the cart to a different
    // part of the campus map. Gate successive absolute fixes by physically
    // plausible motion before they are allowed to touch the anchor.
    {
      std::lock_guard<std::mutex> dl(depth_verified_mtx_);
      if(depth_verified_pose_valid_&&!depth_verified_last_stamp_.isZero()){
        const double dt=(m->header.stamp-depth_verified_last_stamp_).toSec();
        if(dt<=0.0){ROS_WARN_THROTTLE(1.0,"[STEP3 DEPTH] rejected out-of-order verified map pose");return;}
        if(dt<=depth_verified_max_gap_sec_){
          const double motion=(meas.translation()-depth_verified_last_pose_.translation()).norm();
          const double allowed=depth_verified_jump_slack_m_+depth_verified_max_speed_mps_*dt;
          if(!std::isfinite(motion)||motion>allowed){
            ROS_WARN_THROTTLE(0.5,"[STEP3 DEPTH] rejected map-continuity jump %.2fm in %.2fs (limit %.2fm)",motion,dt,allowed);
            return;
          }
        }
      }
      depth_verified_last_pose_=meas;
      depth_verified_last_stamp_=m->header.stamp;
      depth_verified_pose_valid_=true;
    }
    Eigen::Isometry3d fast_body;if(!odomAt(m->header.stamp,fast_body)){ROS_WARN_THROTTLE(1.0,"[STEP3 DEPTH] no time-aligned FAST pose");return;}
    Eigen::Isometry3d anchor;{std::lock_guard<std::mutex>l(anchor_mtx_);anchor=map_from_fast_;}
    const Eigen::Isometry3d pred=anchor*fast_body;
    const Eigen::Vector3d residual=meas.translation()-pred.translation();
    const double innovation=residual.norm();
    if(!std::isfinite(innovation))return;
    updateTrackingMotionModel(meas,m->header.stamp);
    addTrajectoryFitPair(m->header.stamp,fast_body,meas);

    // A verified visual/depth match is an absolute map observation, but it
    // must not teleport the published vehicle pose. Apply only a bounded
    // fraction of its translational innovation on each verified fix;
    // subsequent fixes keep pulling the anchor toward the saved map. These
    // limits were already exposed as ROS params but were bypassed when the
    // authoritative visual-place path was added, which allowed 2-3 m public
    // pose steps during LiDAR outages.
    Eigen::Vector3d step=depth_anchor_translation_gain_*residual;
    const double xy=step.head<2>().norm();
    if(xy>depth_anchor_max_xy_step_&&xy>1e-9)
      step.head<2>()*=depth_anchor_max_xy_step_/xy;
    step.z()=std::max(-depth_anchor_max_z_step_,
                      std::min(depth_anchor_max_z_step_,step.z()));
    const Eigen::Vector3d target_pos=pred.translation()+step;

    bool fit_used=false;double applied_yaw=0.0;
    const Eigen::Isometry3d next=smartAnchorAtCurrentPoint(anchor,fast_body,target_pos,
                                                            yawOf(anchor),false,
                                                            fit_used,applied_yaw);
    const double publish_ramp_sec=publishSmoothingDuration(innovation,applied_yaw);
    {
      std::lock_guard<std::mutex>l(anchor_mtx_);
      map_from_fast_=next;
      correction_filter_valid_=false;
      correction_ramp_active_=false;
      correction_ramp_last_fraction_=0.0;
      if((publish_recovery_active_||publish_recovery_pending_)&&last_published_global_pose_valid_){
        const Eigen::Isometry3d new_target=map_from_fast_*fast_body;
        const double remaining_translation=(last_published_global_pose_.translation()-new_target.translation()).norm();
        const double remaining_yaw=wrap(yawOf(last_published_global_pose_)-yawOf(new_target));
        publish_recovery_duration_sec_=publishSmoothingDuration(remaining_translation,remaining_yaw);
        publish_recovery_active_=false;
        publish_recovery_pending_=true;
      }else{
        publish_recovery_duration_sec_=publish_ramp_sec;
        publish_recovery_active_=false;
        publish_recovery_pending_=last_published_global_pose_valid_;
      }
    }
    ROS_INFO("[STEP3 DEPTH VERIFIED] bounded map fix residual=%.2fm step=[%.2f %.2f %.2f] xyz=[%.2f %.2f %.2f] yawfit=%d smoothing=%.2fs",
             innovation,step.x(),step.y(),step.z(),meas.translation().x(),meas.translation().y(),meas.translation().z(),fit_used?1:0,publish_ramp_sec);
  }
  void publishFailoverSeed(const ros::Time& stamp,const Eigen::Isometry3d& map_body){
    geometry_msgs::PoseWithCovarianceStamped p;p.header.stamp=stamp;p.header.frame_id="map";p.pose.pose=poseMsg(map_body);p.pose.covariance.fill(0.0);failover_initial_pose_pub_.publish(p);
  }
  static double smoothStep01(double u){u=std::max(0.0,std::min(1.0,u));return u*u*(3.0-2.0*u);}
  double publishSmoothingDuration(double translation_m,double yaw_rad) const {
    // smoothStep01(u)=3u^2-2u^3 has a peak derivative of 1.5 at u=0.5.
    // Account for that here so configured correction-speed limits are true
    // peak limits instead of being exceeded by about 50% mid-ramp.
    constexpr double kSmoothStepPeakSlope=1.5;
    const double t_trans=kSmoothStepPeakSlope*translation_m/
        std::max(0.1,tracking_publish_max_correction_speed_mps_);
    const double t_yaw=kSmoothStepPeakSlope*std::abs(yaw_rad)*180.0/M_PI/
        std::max(1.0,tracking_publish_max_yaw_correction_deg_s_);
    return std::max(tracking_publish_smoothing_min_sec_,
                    std::min(tracking_publish_smoothing_max_sec_,std::max(t_trans,t_yaw)));
  }
  void updateMapFixFallbackTargetLocked(const Eigen::Isometry3d& pose,const ros::Time& stamp){
    if(map_fix_target_valid_){
      const double dt=(stamp-map_fix_target_stamp_).toSec();
      if(dt>0.05&&dt<3.0){
        Eigen::Vector3d v=(pose.translation()-map_fix_target_pose_.translation())/dt;
        const double speed=v.norm();
        if(std::isfinite(speed)&&speed>map_fix_fallback_max_speed_mps_&&speed>1e-9)v*=map_fix_fallback_max_speed_mps_/speed;
        if(v.allFinite())map_fix_velocity_=v;
        double wr=wrap(yawOf(pose)-yawOf(map_fix_target_pose_))/dt;
        const double max_wr=map_fix_fallback_max_yaw_rate_deg_s_*M_PI/180.0;
        if(std::isfinite(wr))map_fix_yaw_rate_=std::max(-max_wr,std::min(max_wr,wr));
      }
    }
    map_fix_target_pose_=pose;
    map_fix_target_stamp_=stamp;
    map_fix_target_valid_=true;
    map_fix_fallback_active_=true;
  }
  Eigen::Isometry3d mapFixFallbackPoseLocked(const ros::Time& stamp)const{
    Eigen::Isometry3d target=map_fix_target_pose_;
    const double age=std::max(0.0,std::min(map_fix_fallback_max_extrapolation_sec_,(stamp-map_fix_target_stamp_).toSec()));
    target.translation()+=age*map_fix_velocity_;
    if(std::abs(map_fix_yaw_rate_)>1e-9)target.linear()=Eigen::AngleAxisd(age*map_fix_yaw_rate_,Eigen::Vector3d::UnitZ()).toRotationMatrix()*target.linear();
    if(!last_published_global_pose_valid_)return target;
    const double dt=last_published_global_stamp_.isZero()?0.02:std::max(0.0,std::min(0.2,(stamp-last_published_global_stamp_).toSec()));
    const double tau=std::max(0.03,map_fix_fallback_follow_tau_sec_);
    const double a=std::max(0.0,std::min(1.0,1.0-std::exp(-dt/tau)));
    Eigen::Isometry3d out=Eigen::Isometry3d::Identity();
    Eigen::Vector3d step=a*(target.translation()-last_published_global_pose_.translation());
    const double max_step=map_fix_fallback_max_speed_mps_*std::max(0.005,dt);
    const double sn=step.norm();
    if(sn>max_step&&sn>1e-9)step*=max_step/sn;
    out.translation()=last_published_global_pose_.translation()+step;
    Eigen::Quaterniond q0(last_published_global_pose_.linear()),q1(target.linear());q0.normalize();q1.normalize();
    double dot=std::abs(q0.dot(q1));dot=std::max(0.0,std::min(1.0,dot));
    const double angle=2.0*std::acos(dot);
    const double max_angle=map_fix_fallback_max_yaw_rate_deg_s_*M_PI/180.0*std::max(0.005,dt);
    const double ar=angle>1e-9?std::min(a,max_angle/angle):a;
    out.linear()=q0.slerp(ar,q1).normalized().toRotationMatrix();
    return out;
  }
  void advanceCorrectionRampLocked(const ros::Time& stamp,const Eigen::Isometry3d& fast_body){
    if(!correction_ramp_active_)return;
    // Registration can finish well after the LiDAR scan timestamp.  Start the
    // ramp from the first odometry sample that can actually publish it, rather
    // than consuming most of the ramp while GICP is still running.
    if(correction_ramp_start_stamp_.isZero()){
      correction_ramp_start_stamp_=stamp;
      correction_ramp_last_fraction_=0.0;
      return;
    }
    const double duration=std::max(0.05,correction_ramp_duration_sec_);
    const double u=(stamp-correction_ramp_start_stamp_).toSec()/duration;
    if(u<=0.0)return;
    const double fraction=smoothStep01(u);
    const double df=std::max(0.0,fraction-correction_ramp_last_fraction_);
    if(df>1e-9){
      Eigen::Isometry3d map_body=map_from_fast_*fast_body;
      const Eigen::Vector3d dxyz=df*correction_ramp_translation_;
      const double dyaw=df*correction_ramp_yaw_;
      map_body.translation()+=dxyz;
      map_body.linear()=Eigen::AngleAxisd(dyaw,Eigen::Vector3d::UnitZ()).toRotationMatrix()*map_body.linear();
      map_from_fast_=map_body*fast_body.inverse();
      correction_ramp_last_fraction_=fraction;
      ROS_INFO_THROTTLE(0.5,"[STEP3 RAMP] %.0f%% dxyz=[%.3f %.3f %.3f] dyaw=%.3fdeg",100.0*fraction,dxyz.x(),dxyz.y(),dxyz.z(),dyaw*180.0/M_PI);
    }
    if(u>=1.0-1e-6){correction_ramp_active_=false;correction_ramp_last_fraction_=1.0;ROS_INFO("[STEP3 RAMP] completed");}
  }
  void odomCb(const nav_msgs::OdometryConstPtr&m){
    Eigen::Isometry3d T;if(!odomPose(*m,T))return;
    if(fast_local_prev_valid_){
      const double dt=(m->header.stamp-fast_local_prev_stamp_).toSec();
      if(dt>1e-3){
        // Validate the very first FAST pose after a processing gap too.  The old
        // dt<1s condition let a 16s estimator stall resume ~1900m away and that
        // corrupt pose was published once before the next sample tripped the
        // guard.  Speed over sensor time is still meaningful across a long gap:
        // a cart may move tens of metres, but never kilometres.
        const Eigen::Vector3d dp=T.translation()-fast_local_prev_pose_.translation();
        const double speed=dp.norm()/dt;
        const double vz=std::abs(dp.z())/dt;
        // A long silence from FAST is itself an unsafe handoff boundary. Even
        // if the first returned pose implies a plausible *average* speed over
        // 10-20 s, the internal ESIKF may have resumed from stale queued data.
        // Never publish that first pose directly. Quarantine FAST, keep the
        // global output on the trusted map predictor/hold state, and require a
        // fresh map rebase after sane local increments have resumed.
        const bool bad_gap=dt>std::max(0.25,fast_local_max_pose_gap_sec_);
        const bool bad_motion=!std::isfinite(speed)||!std::isfinite(vz)||
            speed>fast_local_max_speed_mps_||vz>fast_local_max_vertical_speed_mps_;
        const bool bad=bad_gap||bad_motion;
        if(bad){
          bool recovery_rebased=false;
          if(lidarRecoveryBridgeActive(m->header.stamp)){
            std::lock_guard<std::mutex> l(anchor_mtx_);
            if(last_published_global_pose_valid_){
              // During LiDAR recovery FAST-LIVO2 can intentionally rebuild and
              // rebase its local voxel state. Preserve the trusted GLOBAL pose
              // and solve a new map<-FAST transform instead of declaring the
              // cart globally lost and launching a whole-map search.
              map_from_fast_=last_published_global_pose_*T.inverse();
              correction_ramp_active_=false;
              correction_ramp_last_fraction_=0.0;
              publish_recovery_active_=false;
              publish_recovery_pending_=false;
              map_fix_fallback_active_=false;
              recovery_rebased=true;
            }
          }
          if(recovery_rebased){
            fast_local_unstable_.store(false);
            fast_local_recovery_ready_.store(false);
            fast_local_stable_samples_=0;
            last_tracking_accept_stamp_=m->header.stamp;
            last_tracking_id_=-1;
            tracking_misses_=0;
            global_reacquire_consistent_=0;
            global_reacquire_last_map_valid_=false;
            ROS_WARN("[STEP3 RECOVERY-BRIDGE] local FAST frame changed during LiDAR rejoin; rebased map<-FAST to preserve published global pose (dt=%.2fs speed=%.1fm/s)",dt,speed);
          }else{
            if(!fast_local_unstable_.load()){
              if(bad_gap)
                ROS_ERROR("[STEP3 FAST-GUARD] FAST pose gap %.2fs exceeds %.2fs; quarantining resumed local state before publication and forcing trusted map rebase",dt,fast_local_max_pose_gap_sec_);
              else
                ROS_ERROR("[STEP3 FAST-GUARD] implausible FAST local motion %.1fm/s vertical %.1fm/s over dt=%.2fs; rejecting bad handoff before publication and forcing relocalization",speed,vz,dt);
            }
            fast_local_unstable_.store(true);
            fast_local_recovery_ready_.store(false);
            fast_local_stable_samples_=0;
          }
        }else if(fast_local_unstable_.load()){
          if(!fast_local_recovery_ready_.load()&&++fast_local_stable_samples_>=std::max(1,fast_local_recover_samples_)){
            fast_local_recovery_ready_.store(true);
            ROS_WARN("[STEP3 FAST-GUARD] FAST local increments sane again; state still quarantined until trusted map rebase");
          }
        }
      }
    }
    fast_local_prev_pose_=T;fast_local_prev_stamp_=m->header.stamp;fast_local_prev_valid_=true;
    {std::lock_guard<std::mutex>l(odom_mtx_);odom_.push_back({m->header.stamp,T});while(odom_.size()>4000)odom_.pop_front();}
    if(locked_.load()){
      Eigen::Isometry3d tracking_gap_pose=Eigen::Isometry3d::Identity();
      double tracking_gap_age=0.0;
      const bool tracking_gap_pose_valid=trackingMotionPrediction(m->header.stamp,tracking_gap_pose,tracking_gap_age)&&
          tracking_gap_age>=tracking_gap_predict_after_sec_;
      bool map_motion_recovery_publishing=false;
      bool depth_map_motion_publishing=false;
      Eigen::Isometry3d global_pose;
      {
        std::lock_guard<std::mutex>l(anchor_mtx_);
        if(fast_local_unstable_.load()&&map_fix_fallback_active_&&map_fix_target_valid_){
          global_pose=mapFixFallbackPoseLocked(m->header.stamp);
          correction_ramp_active_=false;
          publish_recovery_active_=false;
        }else if(fast_local_unstable_.load()&&last_published_global_pose_valid_){
          global_pose=last_published_global_pose_;
          correction_ramp_active_=false;
          publish_recovery_active_=false;
        }else{
          advanceCorrectionRampLocked(m->header.stamp,T);
          global_pose=map_from_fast_*T;
          // Following a LiDAR/IMU outage, FAST's incremental Z/heading may
          // remain biased even while direct PCD geometry registers correctly.
          // Use the map-space trajectory inferred from successive trusted
          // GICP observations for short, *fresh* intervals after reacquisition
          // instead of integrating those suspect FAST increments indefinitely.
          // This affects ONLY publication. The ESIKF and internal matching
          // anchor remain untouched, and map prediction is never allowed to
          // substitute for a missing/outdated GICP match.
          if(tracking_publish_map_motion_recovery_enable_ &&
             publish_critical_sensor_recovered_.load() &&
             publish_full_sensor_mode_.load() &&
             m->header.stamp.toSec()-publish_full_sensor_mode_since_sec_.load()>=
                 tracking_publish_post_failure_quarantine_sec_){
            Eigen::Isometry3d map_motion_target;
            double map_age=0.0;
            if(trackingMotionPrediction(m->header.stamp,map_motion_target,map_age) &&
               map_age<=tracking_publish_map_motion_recovery_max_age_sec_){
              global_pose=map_motion_target;
              map_motion_recovery_publishing=true;
              ROS_INFO_THROTTLE(1.0,"[STEP3 MAP-MOTION HANDOFF] trusted map predictor age %.2fs replaces biased FAST propagation in public pose",
                                map_age);
            }
          }
          // During the LiDAR outage, a fresh *independently registered*
          // RGB-D/PCD map pose is more trustworthy for GLOBAL motion than
          // the temporarily drifting local FAST-LIVO2 propagation. Never
          // use an RGB-D prediction or a stale pre-outage LiDAR registration
          // as this absolute source. This path only changes publication.
          if(tracking_publish_depth_map_motion_enable_){
            bool lidar_failed=false;
            {
              std::lock_guard<std::mutex> ml(mode_mtx_);
              lidar_failed=(failsafe_mode_=="VI_LIDAR_FAILED" || failsafe_mode_=="IMU_ONLY_BRIDGE");
            }
            if(lidar_failed){
              bool depth_fresh=false;
              {
                std::lock_guard<std::mutex> dl(depth_verified_mtx_);
                const double age=(m->header.stamp-depth_verified_last_stamp_).toSec();
                depth_fresh=depth_verified_pose_valid_ && !depth_verified_last_stamp_.isZero() &&
                    age>=0.0 && age<=tracking_publish_depth_map_motion_max_age_sec_;
              }
              if(depth_fresh){
                Eigen::Isometry3d map_target;
                double map_age=0.0;
                if(trackingMotionPrediction(m->header.stamp,map_target,map_age) &&
                   map_age<=tracking_publish_depth_map_motion_max_age_sec_){
                  global_pose=map_target;
                  depth_map_motion_publishing=true;
                  ROS_INFO_THROTTLE(1.0,"[STEP3 DEPTH-MAP HANDOFF] verified depth map age %.2fs; using map motion over drifting FAST output",map_age);
                }
              }
            }
          }
          if(tracking_gap_pose_valid){
            const double u=std::max(0.0,std::min(1.0,
                (tracking_gap_age-tracking_gap_predict_after_sec_)/std::max(0.05,tracking_gap_predict_blend_sec_)));
            const double a=u*u*(3.0-2.0*u);
            global_pose.translation()=(1.0-a)*global_pose.translation()+a*tracking_gap_pose.translation();
            Eigen::Quaterniond q0(global_pose.linear()),q1(tracking_gap_pose.linear());q0.normalize();q1.normalize();
            global_pose.linear()=q0.slerp(a,q1).normalized().toRotationMatrix();
            ROS_WARN_THROTTLE(0.5,"[STEP3 GAP-PRED] %.2fs since trusted map fix; blending %.0f%% to map-motion predictor",
                              tracking_gap_age,100.0*a);
          }
        }
        // The relative-motion publisher already follows the NEW map anchor
        // with bounded corrections from the previously published pose. A
        // second timestamp-based ramp resets every ~1 s GICP fix and keeps
        // reintroducing the same stale-map offset. Use only one handoff stage
        // when local increments and the sensor mode are both trustworthy.
        const double publish_local_dt=publish_previous_local_valid_?
            (m->header.stamp-publish_previous_local_stamp_).toSec():0.0;
        const bool single_stage_handoff=
            tracking_publish_single_stage_handoff_enable_ &&
            publish_critical_sensor_recovered_.load() &&
            tracking_publish_relative_motion_limit_enable_ &&
            !(tracking_publish_skip_relative_limit_after_critical_recovery_&&
              publish_critical_sensor_recovered_.load()) &&
            publish_full_sensor_mode_.load() &&
            (m->header.stamp.toSec()-publish_full_sensor_mode_since_sec_.load()>=
             tracking_publish_post_failure_quarantine_sec_) &&
            !fast_local_unstable_.load() && last_published_global_pose_valid_ &&
            publish_previous_local_valid_ && publish_local_dt>0.001 && publish_local_dt<=0.25;
        if(single_stage_handoff || (depth_map_motion_publishing && last_published_global_pose_valid_)){
          if(publish_recovery_pending_ || publish_recovery_active_)
            ROS_INFO_THROTTLE(1.0,"[STEP3 SINGLE-HANDOFF] verified map anchor followed by bounded local-motion correction; suppressing duplicate time ramp");
          publish_recovery_pending_=false;
          publish_recovery_active_=false;
        }
        const double observer_dt=publish_previous_local_valid_?
          (m->header.stamp-publish_previous_local_stamp_).toSec():0.0;
        const double verified_age=m->header.stamp.toSec()-last_verified_map_fix_stamp_sec_.load();
        const double handoff_elapsed=m->header.stamp.toSec()-healthy_gap_handoff_start_sec_.load();
        const bool healthy_gap_handoff_active=tracking_healthy_gap_handoff_enable_ &&
          handoff_elapsed>=0.0 && handoff_elapsed<tracking_healthy_gap_handoff_duration_sec_ &&
          publish_full_sensor_mode_.load() && !publish_critical_sensor_recovered_.load();
        // Continue using the same FAST-relative publisher through a short
        // PCD matching gap. Without this, at 2.0s GICP age the observer
        // DISENGAGES and the old pose ramp takes over; the next valid match
        // at ~2.5s makes the observer re-engage, causing the loop-exit kink.
        const double healthy_map_age_max=tracking_healthy_gap_handoff_enable_?
            std::max(2.0,tracking_healthy_gap_handoff_observer_map_age_sec_):2.0;
        const bool healthy_observer_active=tracking_publish_healthy_observer_enable_ &&
          publish_full_sensor_mode_.load() && !fast_local_unstable_.load() &&
          last_published_global_pose_valid_ && publish_previous_local_valid_ &&
          observer_dt>0.001 &&
          observer_dt<=((healthy_gap_handoff_active ||
             (tracking_healthy_gap_handoff_enable_ && tracking_healthy_gap_handoff_extended_local_dt_enable_))?
            std::max(0.25,tracking_healthy_gap_handoff_max_odom_dt_sec_):0.25) &&
          verified_age>=0.0 && verified_age<=healthy_map_age_max;
        if(healthy_observer_active){
          // All sensor poses remain internal to FAST-LIVO2. We do not modify
          // its ESIKF/voxel state or the exact map-search anchor. Suppress the
          // older recovery ramp and per-turn correction limiter ONLY while
          // the opt-in healthy velocity observer owns the public output.
          publish_recovery_pending_=false;
          publish_recovery_active_=false;
          const Eigen::Isometry3d relative=publish_previous_local_pose_.inverse()*T;
          const Eigen::Isometry3d predicted=last_published_global_pose_*relative;
          if(tracking_publish_healthy_correction_only_enable_){
            // FAST local increments are already physically smooth and turn
            // with the cart. Preserve them verbatim; accelerate/limit ONLY
            // the additional global correction. Filtering the TOTAL velocity
            // proved to lag sharp 40-60s turns by several metres.
            const Eigen::Vector3d error=global_pose.translation()-predicted.translation();
            Eigen::Vector3d desired=tracking_publish_healthy_observer_position_gain_*error;
            const double cap=std::max(0.1,tracking_publish_healthy_observer_speed_limit_);
            if(desired.norm()>cap)desired*=cap/desired.norm();
            Eigen::Vector2d lateral=Eigen::Vector2d(-predicted.linear()(1,0),predicted.linear()(0,0));
            if(lateral.norm()>1e-6)lateral.normalize();
            const double handoff_u=healthy_gap_handoff_active?
                smoothStep01(handoff_elapsed/std::max(0.2,tracking_healthy_gap_handoff_duration_sec_)):1.0;
            const double lateral_cap=tracking_healthy_gap_handoff_lateral_speed_mps_+
                (cap-tracking_healthy_gap_handoff_lateral_speed_mps_)*handoff_u;
            if(healthy_gap_handoff_active){
              // Preserve the FAST forward displacement verbatim. Limit only
              // the ADDITIONAL sideways velocity due to a new PCD anchor.
              // This is independent of map position, heading and map scoring.
              const double component=desired.head<2>().dot(lateral);
              desired.head<2>()+=lateral*(std::max(-lateral_cap,
                  std::min(lateral_cap,component))-component);
            }
            if(!healthy_observer_velocity_valid_){
              healthy_observer_velocity_.setZero();
              healthy_observer_yaw_rate_=0.0;
              healthy_observer_velocity_valid_=true;
            }
            Eigen::Vector3d change=desired-healthy_observer_velocity_;
            const double max_change=std::max(0.1,tracking_publish_healthy_observer_accel_limit_)*observer_dt;
            if(change.norm()>max_change)change*=max_change/change.norm();
            healthy_observer_velocity_+=change;
            if(healthy_gap_handoff_active){
              const double actual=healthy_observer_velocity_.head<2>().dot(lateral);
              healthy_observer_velocity_.head<2>()+=lateral*(std::max(-lateral_cap,
                  std::min(lateral_cap,actual))-actual);
            }
            global_pose.translation()=predicted.translation()+observer_dt*healthy_observer_velocity_;

            const double yaw_error=wrap(yawOf(global_pose)-yawOf(predicted));
            const double rate_cap=(healthy_gap_handoff_active?
                tracking_healthy_gap_handoff_yaw_rate_deg_s_+
                (tracking_publish_healthy_observer_yaw_rate_limit_deg_s_-
                 tracking_healthy_gap_handoff_yaw_rate_deg_s_)*handoff_u:
                tracking_publish_healthy_observer_yaw_rate_limit_deg_s_)*M_PI/180.0;
            const double desired_rate=std::max(-rate_cap,std::min(rate_cap,1.5*yaw_error));
            const double max_rate_change=tracking_publish_healthy_observer_yaw_accel_limit_deg_s2_*
                M_PI/180.0*observer_dt;
            healthy_observer_yaw_rate_+=std::max(-max_rate_change,std::min(max_rate_change,
                desired_rate-healthy_observer_yaw_rate_));
            global_pose.linear()=Eigen::AngleAxisd(healthy_observer_yaw_rate_*observer_dt,
                Eigen::Vector3d::UnitZ()).toRotationMatrix()*predicted.linear();
            ROS_INFO_THROTTLE(1.0,"[STEP3 HEALTHY-CORRECTION] map_age=%.2fs target_residual=%.2fm corr_speed=%.2fm/s",
                verified_age,error.norm(),healthy_observer_velocity_.norm());
            if(healthy_gap_handoff_active)
              ROS_INFO_THROTTLE(0.5,"[STEP3 GAP-HANDOFF] elapsed=%.2fs lateral_cap=%.2fm/s yaw_cap=%.1fdeg/s backlog=%.2fm local_dt=%.3fs",
                  handoff_elapsed,lateral_cap,rate_cap*180.0/M_PI,error.norm(),observer_dt);
          }else{
          const Eigen::Vector3d v_fast=(predicted.translation()-last_published_global_pose_.translation())/observer_dt;
          Eigen::Vector3d map_velocity=Eigen::Vector3d::Zero();
          bool map_velocity_valid=false;
          {
            std::lock_guard<std::mutex> motion_lock(tracking_motion_mtx_);
            map_velocity_valid=tracking_motion_valid_ &&
                !tracking_motion_stamp_.isZero() &&
                (m->header.stamp-tracking_motion_stamp_).toSec()<=2.0;
            if(map_velocity_valid)map_velocity=tracking_motion_velocity_;
          }
          const double w=map_velocity_valid?
            std::max(0.0,std::min(0.90,tracking_publish_healthy_observer_map_velocity_weight_)):0.0;
          const Eigen::Vector3d error=global_pose.translation()-predicted.translation();
          Eigen::Vector3d v_target=(1.0-w)*v_fast+w*map_velocity+
              std::max(0.0,tracking_publish_healthy_observer_position_gain_)*error;
          const double v_cap=std::max(0.5,tracking_publish_healthy_observer_speed_limit_);
          if(v_target.norm()>v_cap)v_target*=v_cap/v_target.norm();
          if(!healthy_observer_velocity_valid_){
            healthy_observer_velocity_=v_fast;
            healthy_observer_yaw_rate_=wrap(yawOf(relative))/observer_dt;
            healthy_observer_velocity_valid_=true;
          }
          Eigen::Vector3d change=v_target-healthy_observer_velocity_;
          const double dv_cap=std::max(0.1,tracking_publish_healthy_observer_accel_limit_)*observer_dt;
          if(change.norm()>dv_cap)change*=dv_cap/change.norm();
          healthy_observer_velocity_+=change;
          if(healthy_observer_velocity_.norm()>v_cap)
            healthy_observer_velocity_*=v_cap/healthy_observer_velocity_.norm();
          global_pose.translation()=last_published_global_pose_.translation()+
              observer_dt*healthy_observer_velocity_;
          const double yaw_error=wrap(yawOf(map_from_fast_*T)-yawOf(predicted));
          double desired_yaw_rate=wrap(yawOf(relative))/observer_dt+1.5*yaw_error;
          const double yaw_cap=tracking_publish_healthy_observer_yaw_rate_limit_deg_s_*M_PI/180.0;
          desired_yaw_rate=std::max(-yaw_cap,std::min(yaw_cap,desired_yaw_rate));
          const double dyaw_cap=tracking_publish_healthy_observer_yaw_accel_limit_deg_s2_*
              M_PI/180.0*observer_dt;
          healthy_observer_yaw_rate_+=std::max(-dyaw_cap,std::min(dyaw_cap,
              desired_yaw_rate-healthy_observer_yaw_rate_));
          global_pose.linear()=Eigen::AngleAxisd(healthy_observer_yaw_rate_*observer_dt,
              Eigen::Vector3d::UnitZ()).toRotationMatrix()*last_published_global_pose_.linear();
          ROS_INFO_THROTTLE(1.0,"[STEP3 HEALTHY-OBSERVER] map_age=%.2fs map_vel=%d backlog=%.2fm speed=%.2fm/s",
                            verified_age,map_velocity_valid?1:0,error.norm(),healthy_observer_velocity_.norm());
          }
        }else{
          healthy_observer_velocity_valid_=false;
        }
        if(publish_recovery_pending_){
          // Start continuity compensation on the first pose that is actually
          // published after the expensive global-registration callback. The
          // scan used for GICP can be stale by then, so using its stamp would
          // prematurely consume the smoothing window.
          if(last_published_global_pose_valid_){
            publish_recovery_translation_=last_published_global_pose_.translation()-global_pose.translation();
            publish_recovery_yaw_=wrap(yawOf(last_published_global_pose_)-yawOf(global_pose));
            publish_recovery_start_stamp_=m->header.stamp;
            publish_recovery_active_=true;
            ROS_WARN("[STEP3 GLOBAL-REACQ] starting published-pose continuity ramp offset=%.2fm/%.1fdeg",
                     publish_recovery_translation_.norm(),std::abs(publish_recovery_yaw_)*180.0/M_PI);
          }
          publish_recovery_pending_=false;
        }
        if(publish_recovery_active_){
          const double duration=std::max(0.05,publish_recovery_duration_sec_);
          const double u=(m->header.stamp-publish_recovery_start_stamp_).toSec()/duration;
          const double remaining=1.0-smoothStep01(u);
          if(remaining>1e-6){
            global_pose.translation()+=remaining*publish_recovery_translation_;
            global_pose.linear()=Eigen::AngleAxisd(remaining*publish_recovery_yaw_,Eigen::Vector3d::UnitZ()).toRotationMatrix()*global_pose.linear();
          }
          if(u>=1.0-1e-6){
            publish_recovery_active_=false;
            ROS_INFO("[STEP3 GLOBAL-REACQ] published-pose recovery ramp completed");
          }
        }
        if(depth_map_motion_publishing && last_published_global_pose_valid_){
          const double dt=(m->header.stamp-last_published_global_stamp_).toSec();
          if(dt>0.001 && dt<=0.25){
            Eigen::Isometry3d predicted=last_published_global_pose_;
            {
              std::lock_guard<std::mutex> ml(tracking_motion_mtx_);
              predicted.translation()+=dt*tracking_motion_velocity_;
              predicted.linear()=Eigen::AngleAxisd(dt*tracking_motion_yaw_rate_,Eigen::Vector3d::UnitZ()).toRotationMatrix()*
                  predicted.linear();
            }
            Eigen::Vector3d residual=global_pose.translation()-predicted.translation();
            const double max_extra=std::max(0.1,tracking_publish_depth_map_motion_correction_speed_mps_)*dt;
            if(residual.allFinite()){
              const double distance=residual.norm();
              if(distance>max_extra && distance>1e-9)residual*=max_extra/distance;
              global_pose.translation()=predicted.translation()+residual;
              const double yaw_error=wrap(yawOf(global_pose)-yawOf(predicted));
              const double max_yaw=35.0*M_PI/180.0*dt;
              global_pose.linear()=Eigen::AngleAxisd(std::max(-max_yaw,std::min(max_yaw,yaw_error)),Eigen::Vector3d::UnitZ()).toRotationMatrix()*predicted.linear();
            }
          }
        }
        // Optional physical continuity limiter for the PUBLISHED map pose.
        // Follow actual FAST-LIVO2 local SE(3) displacement since the last
        // publication, then bound ONLY the additional global-map correction.
        // Do not modify the trusted map<-FAST tracking anchor or the ESIKF.
        // This avoids a large correction becoming an instant velocity spike
        // while retaining the map matcher at the correct internal map pose.
        if(!healthy_observer_active && tracking_publish_relative_motion_limit_enable_ &&
           !(tracking_publish_skip_relative_limit_after_critical_recovery_&&
             publish_critical_sensor_recovered_.load()) &&
           publish_full_sensor_mode_.load() &&
           (m->header.stamp.toSec()-publish_full_sensor_mode_since_sec_.load()>=tracking_publish_post_failure_quarantine_sec_) &&
           !fast_local_unstable_.load() &&
           last_published_global_pose_valid_ && publish_previous_local_valid_){
          const double dt=(m->header.stamp-publish_previous_local_stamp_).toSec();
          if(dt>0.001 && dt<=0.25){
            const Eigen::Isometry3d relative=publish_previous_local_pose_.inverse()*T;
            Eigen::Isometry3d predicted=last_published_global_pose_*relative;
            if(map_motion_recovery_publishing){
              // For the same trusted fresh-map interval, the motion used in
              // the continuity limiter must ALSO be map-derived. Otherwise
              // biased FAST Z is reintroduced every 0.1s and can outpace
              // the PCD correction during a turn despite good registrations.
              std::lock_guard<std::mutex> motion_lock(tracking_motion_mtx_);
              predicted.translation()=last_published_global_pose_.translation()+dt*tracking_motion_velocity_;
              predicted.linear()=
                  Eigen::AngleAxisd(dt*tracking_motion_yaw_rate_,Eigen::Vector3d::UnitZ()).toRotationMatrix()*
                  last_published_global_pose_.linear();
            }
            Eigen::Vector3d extra=global_pose.translation()-predicted.translation();
            const double dist=extra.norm();
            // Slow lateral corrections during sharp turns and for a short
            // settling interval afterward. Catch up faster on straight roads
            // to avoid the persistent map error of a fixed low-speed limiter.
            const double local_yaw_rate=std::abs(wrap(yawOf(relative)))*180.0/M_PI/dt;
            if(tracking_publish_relative_turn_sensitive_enable_ &&
               std::isfinite(local_yaw_rate) &&
               local_yaw_rate>=tracking_publish_relative_turn_rate_threshold_deg_s_)
              publish_last_sharp_turn_stamp_=m->header.stamp;
            const bool recent_turn=tracking_publish_relative_turn_sensitive_enable_ &&
                !publish_last_sharp_turn_stamp_.isZero() &&
                (m->header.stamp-publish_last_sharp_turn_stamp_).toSec()<=tracking_publish_relative_turn_hold_sec_;
            const bool recovered=publish_critical_sensor_recovered_.load();
            double correction_speed=recent_turn?
                (recovered?tracking_publish_post_critical_turn_correction_speed_mps_:
                    tracking_publish_relative_turn_correction_speed_mps_):
                tracking_publish_relative_correction_speed_mps_;
            // After a real critical-sensor outage the local FAST trajectory
            // can continue to move vertically/laterally away from the fixed
            // map. A *fresh, verified* LiDAR/PCD registration makes the anchor
            // reliable again. Only then increase correction capacity as a
            // function of actual publisher backlog; never globally loosen
            // turn limits during the healthy small loop or blind outage.
            const double verified_age=m->header.stamp.toSec()-last_verified_map_fix_stamp_sec_.load();
            const bool verified_recovery=tracking_publish_verified_recovery_catchup_enable_ &&
                recovered && verified_age>=0.0 &&
                verified_age<=tracking_publish_verified_recovery_max_age_sec_ &&
                std::isfinite(dist);
            if(verified_recovery && recent_turn && dist>0.50){
              const double adaptive=correction_speed+1.75*(dist-0.50);
              correction_speed=std::min(std::max(correction_speed,tracking_publish_verified_recovery_max_speed_mps_),
                                        std::max(correction_speed,adaptive));
              ROS_INFO_THROTTLE(1.0,"[STEP3 VERIFIED-HANDOFF] accepted map age %.2fs, published backlog %.2fm; bounded correction %.2fm/s",
                                verified_age,dist,correction_speed);
            }
            const double yaw_correction_rate=recent_turn?
                (recovered?tracking_publish_post_critical_turn_yaw_rate_deg_s_:
                    tracking_publish_relative_turn_yaw_correction_deg_s_):
                tracking_publish_relative_yaw_rate_deg_s_;
            const double max_dist=std::max(0.05,correction_speed)*dt;
            if(std::isfinite(dist) && dist>max_dist)
              extra*=max_dist/dist;
            const double yaw_residual=wrap(yawOf(global_pose)-yawOf(predicted));
            const double max_dyaw=std::max(1.0,yaw_correction_rate)*M_PI/180.0*dt;
            const double bounded_yaw=std::max(-max_dyaw,std::min(max_dyaw,yaw_residual));
            if(extra.allFinite() && std::isfinite(yaw_residual)){
              if(dist>max_dist || std::abs(yaw_residual)>max_dyaw)
                ROS_INFO_THROTTLE(0.5,"[STEP3 PUB-LIMIT] map correction %.3fm/%.2fdeg limited to %.3fm/%.2fdeg over %.3fs",
                    dist,yaw_residual*180.0/M_PI,extra.norm(),std::abs(bounded_yaw)*180.0/M_PI,dt);
              global_pose.translation()=predicted.translation()+extra;
              global_pose.linear()=Eigen::AngleAxisd(bounded_yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix()*predicted.linear();
            }
            // The FAST local Z motion can remain biased after a LiDAR or
            // IMU outage, continuously defeating a relative Z correction.
            // Temporarily follow the altitude and vertical motion inferred
            // from accepted PCD/GICP poses instead. Keep XY/yaw on the
            // established turn-aware publisher and bound vertical speed.
            // Disable automatically if verified map fixes become stale.
            if(tracking_publish_recovery_vertical_catchup_enable_ &&
               publish_critical_sensor_recovered_.load() &&
               !fast_local_unstable_.load()){
              double map_z=0.0,map_vz=0.0,map_stamp=0.0;
              bool valid=false;
              {
                std::lock_guard<std::mutex> vertical_lock(verified_vertical_mtx_);
                valid=verified_vertical_valid_;
                map_z=verified_vertical_z_m_;
                map_vz=verified_vertical_v_mps_;
                map_stamp=verified_vertical_stamp_sec_;
              }
              const double age=m->header.stamp.toSec()-map_stamp;
              if(valid&&age>=0.0&&age<=tracking_publish_recovery_vertical_recent_fix_sec_){
                const double target_z=map_z+age*map_vz;
                const double dz=target_z-last_published_global_pose_.translation().z();
                const double max_dz=std::max(0.05,tracking_publish_recovery_vertical_catchup_speed_mps_)*dt;
                const double limited=std::max(-max_dz,std::min(max_dz,dz));
                if(std::isfinite(limited)){
                  global_pose.translation().z()=last_published_global_pose_.translation().z()+limited;
                  if(std::abs(dz)>tracking_publish_recovery_vertical_catchup_threshold_m_)
                    ROS_INFO_THROTTLE(1.0,"[STEP3 RECOVERY-Z] map-follow age %.2fs target %.2fm, backlog %.2fm, applied %.2fm",
                                      age,target_z,dz,limited);
                }
              }
            }
          }
        }
        // A depth-map correction can be geometrically valid but appear at
        // callback time instead of the exposure timestamp.  In LiDAR-outage
        // mode the first such asynchronous fix once caused a -2.58m public Z
        // jump over 0.20s, while XY remained continuous. Bound only the
        // published vertical increment; keep ESIKF and map tracking intact.
        // Catch-up occurs over following frames, without overwriting history.
        if(tracking_publish_lidar_outage_vertical_guard_enable_ &&
           last_published_global_pose_valid_){
          bool lidar_outage=false;
          {
            std::lock_guard<std::mutex> mode_lock(mode_mtx_);
            lidar_outage=(failsafe_mode_=="VI_LIDAR_FAILED" ||
                          failsafe_mode_=="IMU_ONLY_BRIDGE");
          }
          const double dt=(m->header.stamp-last_published_global_stamp_).toSec();
          if(lidar_outage && dt>0.001 && dt<=0.5){
            const double max_dz=std::max(0.2,tracking_publish_lidar_outage_vertical_guard_max_mps_)*dt;
            const double dz=global_pose.translation().z()-last_published_global_pose_.translation().z();
            if(std::isfinite(dz) && std::abs(dz)>max_dz){
              global_pose.translation().z()=last_published_global_pose_.translation().z()+
                  std::copysign(max_dz,dz);
              ROS_WARN_THROTTLE(0.5,"[STEP3 OUTAGE-Z-GUARD] public dz %.3fm/%.3fs bounded to %.3fm; map anchor unchanged",
                                dz,dt,max_dz);
            }
          }
        }
        last_published_global_pose_=global_pose;
        last_published_global_pose_valid_=true;
        last_published_global_stamp_=m->header.stamp;
        publish_previous_local_pose_=T;
        publish_previous_local_stamp_=m->header.stamp;
        publish_previous_local_valid_=true;
      }
      // In MH mode a resumed FAST pose following a multi-second outage may be
      // physically held for one callback. Do NOT stamp that hold as a fresh
      // confident global pose. Fail closed until an independently verified
      // map/FAST re-anchor explicitly clears the untrusted flag.
      if(!mh_enable_||(!mh_pose_untrusted_.load()&&!fast_local_unstable_.load())){
        geometry_msgs::PoseStamped p;p.header.stamp=m->header.stamp;p.header.frame_id="map";p.pose=poseMsg(global_pose);pose_pub_.publish(p);
        if(mh_enable_)mh_last_actual_pose_publication_sec_.store(m->header.stamp.toSec());
      }else ROS_WARN_THROTTLE(1.0,"[STEP3 MH] Global pose withheld: unresolved branch or quarantined FAST local odometry");
    }
  }
  bool odomAt(const ros::Time&t,Eigen::Isometry3d&T){std::lock_guard<std::mutex>l(odom_mtx_);if(odom_.empty())return false;auto it=std::min_element(odom_.begin(),odom_.end(),[&](auto&a,auto&b){return std::abs((a.first-t).toSec())<std::abs((b.first-t).toSec());});if(std::abs((it->first-t).toSec())>.15)return false;T=it->second;return true;}
  bool latestOdom(ros::Time& t,Eigen::Isometry3d& T){std::lock_guard<std::mutex>l(odom_mtx_);if(odom_.empty())return false;t=odom_.back().first;T=odom_.back().second;return true;}
  Cloud::Ptr preprocess(const sensor_msgs::PointCloud2ConstPtr&m){Cloud::Ptr raw(new Cloud);pcl::fromROSMsg(*m,*raw);Cloud::Ptr lidar(new Cloud);lidar->reserve(raw->size());for(const auto&p:*raw){if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;double rr=std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);if(rr<1||rr>80)continue;lidar->push_back(p);}pcl::VoxelGrid<PointT>v;v.setLeafSize(source_voxel_,source_voxel_,source_voxel_);v.setInputCloud(lidar);Cloud::Ptr out(new Cloud);v.filter(*out);return out;}
  struct Match{int id=-1;double sc=1e9,fitness=1e9;Eigen::Isometry3d map_body=Eigen::Isometry3d::Identity();};
  // Bound PCD cache to N submaps (~0.6MB each). Matches along a route
  // repeatedly test neighbouring submaps; re-reading binary PCDs and
  // reallocating each time creates filesystem churn under a 1x ROS replay.
  // Called only by lidarCb under lidar_job_mtx_, so no cross-thread writes.
  Cloud::Ptr submapForId(int id){
    if(id<0 || id>=(int)map_poses_.size())return Cloud::Ptr(new Cloud);
    if(tracking_submap_cache_slots_>0){
      auto it=submap_cache_.find(id);
      if(it!=submap_cache_.end()){
        ++submap_cache_hits_;
        submap_lru_.splice(submap_lru_.begin(),submap_lru_,it->second.lru);
        it->second.lru=submap_lru_.begin();
        return it->second.cloud;
      }
    }
    Cloud::Ptr target(new Cloud);
    if(pcl::io::loadPCDFile(map_db_+"/cache/local_submaps/"+std::to_string(id)+".pcd",*target)!=0)
      return Cloud::Ptr(new Cloud);
    ++submap_cache_misses_;
    if(tracking_submap_cache_slots_>0 && !target->empty()){
      const size_t cap=static_cast<size_t>(std::max(1,std::min(32,tracking_submap_cache_slots_)));
      while(submap_cache_.size()>=cap){
        const int old=submap_lru_.back();
        submap_cache_.erase(old);submap_lru_.pop_back();
      }
      submap_lru_.push_front(id);
      submap_cache_.emplace(id,CachedSubmap{target,submap_lru_.begin()});
      ROS_INFO_THROTTLE(15.0,"[STEP3 PERF] PCD target cache slots=%zu hits=%llu misses=%llu",
        submap_cache_.size(),static_cast<unsigned long long>(submap_cache_hits_),
        static_cast<unsigned long long>(submap_cache_misses_));
    }
    return target;
  }
  Match verify(const Cloud::Ptr&q,int id,int shift,double sc){Match m;m.id=id;m.sc=sc;Cloud::Ptr target=submapForId(id);if(target->empty())return m;Eigen::Isometry3d init=Eigen::Isometry3d::Identity();init.linear()=Eigen::AngleAxisd(-shift*kSectorAngle,Eigen::Vector3d::UnitZ()).toRotationMatrix();pcl::GeneralizedIterativeClosestPoint<PointT,PointT>g;g.setInputSource(q);g.setInputTarget(target);g.setMaximumIterations(30);g.setMaxCorrespondenceDistance(gicp_corr_);g.setTransformationEpsilon(1e-5);Cloud unused;g.align(unused,init.matrix().cast<float>());if(!g.hasConverged())return m;m.fitness=g.getFitnessScore(gicp_corr_);Eigen::Isometry3d corr=Eigen::Isometry3d::Identity();corr.matrix()=g.getFinalTransformation().cast<double>();m.map_body=map_poses_[id]*corr;return m;}
  Match verifyTracking(const Cloud::Ptr&q,int id,const Eigen::Isometry3d& init){
    Match m;m.id=id;m.sc=0.0;Cloud::Ptr target=submapForId(id);
    if(target->empty())return m;
    pcl::GeneralizedIterativeClosestPoint<PointT,PointT>g;g.setInputSource(q);g.setInputTarget(target);g.setMaximumIterations(std::max(8,std::min(35,tracking_gicp_max_iterations_)));g.setMaxCorrespondenceDistance(gicp_corr_);g.setTransformationEpsilon(1e-5);Cloud unused;g.align(unused,init.matrix().cast<float>());if(!g.hasConverged())return m;m.fitness=g.getFitnessScore(gicp_corr_);Eigen::Isometry3d corr=Eigen::Isometry3d::Identity();corr.matrix()=g.getFinalTransformation().cast<double>();m.map_body=map_poses_[id]*corr;return m;
  }

  void mhVisualCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    if(!mh_visual_seed_enable_ || !msg || msg->header.stamp.isZero() ||
       msg->header.frame_id!="map")return;
    const double cx=msg->pose.covariance[0],cy=msg->pose.covariance[7];
    if(!std::isfinite(cx)||!std::isfinite(cy)||cx<=0.0||cy<=0.0||
       cx>mh_visual_seed_max_xy_variance_m2_||cy>mh_visual_seed_max_xy_variance_m2_)return;
    const auto& p=msg->pose.pose;
    Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
    if(!std::isfinite(q.norm())||q.norm()<0.9||q.norm()>1.1)return;
    q.normalize();
    Eigen::Isometry3d camera_candidate=Eigen::Isometry3d::Identity();
    camera_candidate.linear()=q.toRotationMatrix();
    camera_candidate.translation()<<p.position.x,p.position.y,p.position.z;
    if(!camera_candidate.matrix().allFinite())return;
    std::lock_guard<std::mutex> lock(mh_visual_mtx_);
    if(mh_visual_seed_valid_ && msg->header.stamp<=mh_visual_last_stamp_)return;
    mh_visual_last_stamp_=msg->header.stamp;
    mh_visual_last_map_body_=camera_candidate;
    mh_visual_seed_valid_=true;
  }

  // GPS-FREE multi-hypothesis search: propagate each map<-FAST candidate with
  // local odometry, then verify by independent GICP on successive LiDAR scans.
  // The bank is accessed only under lidar_job_mtx_.
  Match mhSelect(const Cloud::Ptr&q,const ros::Time& stamp,const Eigen::Isometry3d& fast_body){
    Match pending;
    if(!mh_last_stamp_.isZero()&&stamp<=mh_last_stamp_)return pending;
    mh_last_stamp_=stamp;
    mh_tracks_.erase(std::remove_if(mh_tracks_.begin(),mh_tracks_.end(),[&](const MhTrack& h){
      return (stamp-h.last_stamp).toSec()>mh_stale_sec_;
    }),mh_tracks_.end());
    Descriptor dq=makeDescriptor(*q);
    std::vector<std::pair<double,int>> ring;
    for(int id=0;id<(int)desc_.size();++id){
      double d=0;
      for(int r=0;r<kRings;++r){double e=dq.ring[r]-desc_[id].ring[r];d+=e*e;}
      ring.push_back({d,id});
    }
    const int nr=std::min<int>(std::max(mh_sc_candidates_*3,20),ring.size());
    if(nr==0)return pending;
    std::partial_sort(ring.begin(),ring.begin()+nr,ring.end());
    std::vector<std::tuple<double,int,int>> candidates;
    for(int k=0;k<nr;++k){
      const int id=ring[k].second;
      const auto d=bestDescriptorDistance(dq,desc_[id]);
      if(d.first<=std::max(0.38,tracking_global_reacquire_descriptor_threshold_))
        candidates.emplace_back(d.first,id,d.second);
    }
    std::sort(candidates.begin(),candidates.end());
    std::vector<Match> observations;
    // Scan Context's top entries often come from adjacent keyframes of ONE
    // road pass. Spend the first GICP budget on spatially distinct map regions
    // so repeated-road hypotheses are actually considered.
    std::vector<int> chosen;
    for(int pass=0;pass<2 && (int)chosen.size()<mh_geometric_candidates_;++pass){
      for(int k=0;k<(int)candidates.size() && (int)chosen.size()<mh_geometric_candidates_;++k){
        if(std::find(chosen.begin(),chosen.end(),k)!=chosen.end())continue;
        const int id=std::get<1>(candidates[k]);
        bool diverse=true;
        for(int prev:chosen){
          const int other=std::get<1>(candidates[prev]);
          if((map_poses_[id].translation()-map_poses_[other].translation()).head<2>().norm()<
             mh_candidate_diversity_m_){diverse=false;break;}
        }
        if(pass==0 && !diverse)continue;
        chosen.push_back(k);
      }
    }
    for(int k:chosen){
      auto [sc,id,shift]=candidates[k];
      Match x=verify(q,id,shift,sc);
      ROS_INFO("[STEP3 MH] global candidate id=%d sc=%.3f gicp=%.4f",id,sc,x.fitness);
      if(x.id>=0&&std::isfinite(x.fitness)&&x.fitness<=mh_max_fitness_&&
         x.map_body.matrix().allFinite())observations.push_back(x);
    }
    // Existing hypotheses remain eligible even if their current descriptor
    // drops out of the global shortlist during turns or weak geometry.
    for(const auto& h:mh_tracks_){
      const Eigen::Isometry3d pred_lidar=h.anchor*fast_body*body_from_lidar_;
      int best_id=-1;double best=1e12;
      for(int i=0;i<(int)map_poses_.size();++i){
        const double xy=(map_poses_[i].translation()-pred_lidar.translation()).head<2>().squaredNorm();
        const double dyaw=wrap(yawOf(map_poses_[i])-yawOf(pred_lidar));
        const double score=xy+9.0*dyaw*dyaw;
        if(score<best){best=score;best_id=i;}
      }
      if(best_id<0)continue;
      Match x=verifyTracking(q,best_id,map_poses_[best_id].inverse()*pred_lidar);
      ROS_INFO("[STEP3 MH] temporal branch=%d candidate=%d gicp=%.4f",h.id,best_id,x.fitness);
      if(x.id>=0&&std::isfinite(x.fitness)&&x.fitness<=mh_max_fitness_&&
         x.map_body.matrix().allFinite())observations.push_back(x);
    }
    // Camera-only PnP is NOT fused as a LiDAR pose. A prior, temporally
    // confirmed ROS image match suggests a map region to TRY with GICP.
    // Correct the image->scan time offset using relative FAST odometry,
    // then require independent LiDAR geometric and temporal verification.
    if(mh_visual_seed_enable_){
      Eigen::Isometry3d visual_pose=Eigen::Isometry3d::Identity();
      ros::Time visual_stamp;bool valid=false;
      {std::lock_guard<std::mutex> lock(mh_visual_mtx_);
        valid=mh_visual_seed_valid_;
        if(valid){visual_pose=mh_visual_last_map_body_;visual_stamp=mh_visual_last_stamp_;}}
      const double age=(stamp-visual_stamp).toSec();
      if(valid && std::isfinite(age) && age>=-0.15 &&
         age<=mh_visual_seed_max_age_sec_){
        Eigen::Isometry3d visual_fast_body=Eigen::Isometry3d::Identity();
        if(odomAt(visual_stamp,visual_fast_body)){
          const Eigen::Isometry3d predicted_lidar=
            visual_pose*visual_fast_body.inverse()*fast_body*body_from_lidar_;
          int visual_keyframe=-1;double nearest_score=1e12;
          for(int i=0;i<(int)map_poses_.size();++i){
            const double xy=(map_poses_[i].translation()-predicted_lidar.translation()).head<2>().squaredNorm();
            const double yaw=wrap(yawOf(map_poses_[i])-yawOf(predicted_lidar));
            const double score=xy+9.0*yaw*yaw;
            if(score<nearest_score){nearest_score=score;visual_keyframe=i;}
          }
          if(visual_keyframe>=0){
            const Eigen::Isometry3d seed=
              map_poses_[visual_keyframe].inverse()*predicted_lidar;
            const Match x=verifyTracking(q,visual_keyframe,seed);
            const bool passed=x.id>=0 && std::isfinite(x.fitness) &&
                              x.fitness<=mh_max_fitness_ &&
                              x.map_body.matrix().allFinite();
            ROS_INFO("[STEP3 MH] VISUAL_SEED_GICP keyframe=%d visual_age=%.2fs fitness=%.4f pass=%d (never pose authority)",
                     visual_keyframe,age,x.fitness,(int)passed);
            if(passed)observations.push_back(x);
          }
        }else ROS_WARN_THROTTLE(1.0,"[STEP3 MH] visual seed lacked time-aligned FAST odometry: skipped");
      }
    }
    std::sort(observations.begin(),observations.end(),[](const Match&a,const Match&b){
      return a.fitness<b.fitness;
    });
    std::vector<bool> updated(mh_tracks_.size(),false);
    for(const auto& x:observations){
      const Eigen::Isometry3d measured=x.map_body*body_from_lidar_.inverse();
      const Eigen::Isometry3d anchor=measured*fast_body.inverse();
      int nearest=-1;double lowest=1e12;
      for(int i=0;i<(int)mh_tracks_.size();++i){
        const Eigen::Isometry3d pred=mh_tracks_[i].anchor*fast_body;
        const double xy=(pred.translation()-measured.translation()).head<2>().norm();
        const double yaw=std::abs(wrap(yawOf(pred)-yawOf(measured)))*180/M_PI;
        if(xy>mh_association_xy_m_||yaw>mh_association_yaw_deg_)continue;
        if(xy+0.05*yaw<lowest){nearest=i;lowest=xy+0.05*yaw;}
      }
      if(nearest>=0){
        if(updated[nearest])continue; // one independent hit per scan
        auto& h=mh_tracks_[nearest];
        h.anchor=anchor;h.map_body=measured;h.last_stamp=stamp;h.id=x.id;
        h.mean_fitness=0.65*h.mean_fitness+0.35*x.fitness;
        ++h.hits;updated[nearest]=true;
      }else if((int)mh_tracks_.size()<mh_max_hypotheses_){
        MhTrack h;h.anchor=anchor;h.map_body=measured;h.last_stamp=stamp;
        h.id=x.id;h.hits=1;h.mean_fitness=x.fitness;
        mh_tracks_.push_back(h);updated.push_back(true);
      }
    }
    std::vector<int> order(mh_tracks_.size());std::iota(order.begin(),order.end(),0);
    std::sort(order.begin(),order.end(),[&](int a,int b){
      const auto& x=mh_tracks_[a];const auto& y=mh_tracks_[b];
      if(x.hits!=y.hits)return x.hits>y.hits;
      return x.mean_fitness<y.mean_fitness;
    });
    // Preserve sub-second ROS sensor timestamps. Default iostream precision
    // emits e.g. 1.79222e+09, destroying ~1000s of timing information and
    // causing the cross-sensor observer to reject genuinely fresh hypotheses.
    std::ostringstream message;
    message<<std::fixed<<std::setprecision(9)
           <<"stamp="<<stamp.toSec()<<" hypotheses="<<order.size();
    for(int i:order){
      const auto& h=mh_tracks_[i];const auto world=h.anchor*fast_body;
      message<<" [id="<<h.id<<" hits="<<h.hits<<" fit="<<h.mean_fitness
             <<" xy="<<world.translation().x()<<","<<world.translation().y()<<"]";
    }
    std_msgs::String msg;msg.data=message.str();mh_status_pub_.publish(msg);
    ROS_INFO_STREAM("[STEP3 MH] "<<msg.data);
    if(order.empty()||mh_tracks_[order[0]].hits<mh_required_hits_)return pending;
    const auto& leader=mh_tracks_[order[0]];
    // A retained hypothesis is only a *prediction* until this scan provides
    // a new geometrically verified update. Previously the MH bank could keep
    // emitting CONFIRMED for an old 20-hit leader with no new GICP support,
    // masking the presence of newly discovered alternative branches.
    if(leader.last_stamp!=stamp){
      ROS_WARN("[STEP3 MH] NO_FRESH_LEADER_EVIDENCE id=%d hits=%d last=%.6f scan=%.6f: candidate withheld",
               leader.id,leader.hits,leader.last_stamp.toSec(),stamp.toSec());
      return pending;
    }
    for(size_t j=1;j<order.size();++j){
      const auto& alt=mh_tracks_[order[j]];
      if(alt.hits<2)continue;
      const auto pos0=leader.anchor*fast_body,pos1=alt.anchor*fast_body;
      const double gap=(pos0.translation()-pos1.translation()).head<2>().norm();
      if(gap>=mh_ambiguity_separation_m_ &&
         (alt.mean_fitness-leader.mean_fitness<mh_min_fitness_advantage_ ||
          leader.hits-alt.hits<2)){
        ROS_WARN("[STEP3 MH] AMBIGUOUS ids=%d,%d separation=%.2fm hits=%d,%d; withholding anchor",
                 leader.id,alt.id,gap,leader.hits,alt.hits);
        publishStatus("RELOCALIZING: MH_AMBIGUOUS");
        return pending;
      }
    }
    Match selected;selected.id=leader.id;selected.fitness=leader.mean_fitness;
    selected.map_body=leader.map_body*body_from_lidar_; // T_map_lidar
    ROS_INFO("[STEP3 MH] CONFIRMED id=%d hits=%d fitness=%.4f xy=[%.2f %.2f]",
             leader.id,leader.hits,leader.mean_fitness,
             leader.map_body.translation().x(),leader.map_body.translation().y());
    return selected;
  }

  Match searchRouteReacquire(const Cloud::Ptr&q,const ros::Time& stamp){
    Match best;
    if(last_tracking_id_<0||desc_.empty())return best;
    const double gap=last_tracking_accept_stamp_.isZero()?0.0:std::max(0.0,(stamp-last_tracking_accept_stamp_).toSec());
    const int dynamic_band=tracking_route_reacquire_id_band_+
      (int)std::ceil(tracking_route_reacquire_id_rate_*std::min(6.0,gap));
    const int lo=std::max(0,last_tracking_id_-dynamic_band);
    const int hi=std::min((int)desc_.size()-1,last_tracking_id_+dynamic_band);
    if(lo>hi)return best;

    Descriptor dq=makeDescriptor(*q);
    std::vector<std::tuple<double,int,int>> cand;
    cand.reserve(std::max(0,hi-lo+1));
    for(int id=lo;id<=hi;++id){
      auto ds=bestDescriptorDistance(dq,desc_[id]);
      if(ds.first<=tracking_route_reacquire_descriptor_threshold_)
        cand.push_back({ds.first,id,ds.second});
    }
    if(cand.empty())return best;
    std::sort(cand.begin(),cand.end());
    const int ng=std::min<int>(tracking_route_reacquire_geometric_candidates_,cand.size());
    for(int k=0;k<ng;++k){
      auto [sc,id,sh]=cand[k];
      Match x=verify(q,id,sh,sc);
      if(x.id<0||!std::isfinite(x.fitness))continue;
      const Eigen::Isometry3d map_body=x.map_body*body_from_lidar_.inverse();
      bool continuity_ok=true;
      double motion=0.0,dyaw=0.0,motion_limit=0.0,yaw_limit=0.0;
      if(last_tracking_map_valid_){
        const double dt=std::max(0.05,(stamp-last_tracking_map_stamp_).toSec());
        const Eigen::Isometry3d d=last_tracking_map_body_.inverse()*map_body;
        motion=d.translation().norm();
        dyaw=std::abs(wrap(yawOf(map_body)-yawOf(last_tracking_map_body_)))*180.0/M_PI;
        motion_limit=3.0+tracking_route_reacquire_max_speed_mps_*dt;
        yaw_limit=12.0+tracking_route_reacquire_max_yaw_rate_deg_s_*dt;
        continuity_ok=motion<=motion_limit&&dyaw<=yaw_limit;
      }
      ROS_INFO("[STEP3 ROUTE-REACQ] id=%d sc=%.3f gicp=%.4f band=[%d,%d] motion=%.2f/%.2f yaw=%.1f/%.1f ok=%d",
               id,sc,x.fitness,lo,hi,motion,motion_limit,dyaw,yaw_limit,continuity_ok?1:0);
      if(continuity_ok&&x.fitness<=tracking_route_reacquire_max_fitness_&&x.fitness<best.fitness)
        best=x;
    }
    return best;
  }

  Match searchGlobalReacquire(const Cloud::Ptr&q,const ros::Time& stamp,const Eigen::Isometry3d& predicted_map_body){
    Descriptor dq=makeDescriptor(*q);
    const auto gps_dist=gpsDistancesAt(stamp);
    std::vector<std::pair<double,int>> ring;ring.reserve(desc_.size());
    for(int i=0;i<(int)desc_.size();++i){double d=0;for(int r=0;r<kRings;++r){double e=dq.ring[r]-desc_[i].ring[r];d+=e*e;}ring.push_back({d,i});}
    const int nr=std::min<int>(tracking_global_reacquire_descriptor_candidates_,ring.size());
    if(nr<=0)return Match();
    std::partial_sort(ring.begin(),ring.begin()+nr,ring.end());
    std::vector<std::tuple<double,int,int>> cand;
    for(int k=0;k<nr;++k){int id=ring[k].second;auto ds=bestDescriptorDistance(dq,desc_[id]);if(ds.first<tracking_global_reacquire_descriptor_threshold_)cand.push_back({ds.first,id,ds.second});}
    std::sort(cand.begin(),cand.end());
    if(!gps_dist.empty()&&!cand.empty()){
      std::stable_sort(cand.begin(),cand.end(),[&](const auto& a,const auto& b){
        const double sa=std::get<0>(a)+gpsPenalty(std::get<1>(a),gps_dist,gps_sc_penalty_per_m_,0.06);
        const double sb=std::get<0>(b)+gpsPenalty(std::get<1>(b),gps_dist,gps_sc_penalty_per_m_,0.06);
        return sa<sb;
      });
      ROS_INFO_THROTTLE(2.0,"[STEP3 GPS] reacquire: soft candidate rank; closest ranked map GPS=%.1fm, LiDAR global candidates retained",gps_dist[std::get<1>(cand.front())]);
    }
    Match best_any,best_cont;
    double best_physical_score=std::numeric_limits<double>::infinity();
    const double gap=last_tracking_accept_stamp_.isZero()?1e9:std::max(0.0,(stamp-last_tracking_accept_stamp_).toSec());
    const bool branch_guard=last_tracking_id_>=0&&gap<tracking_global_reacquire_unrestricted_after_sec_;
    const double id_allow=tracking_id_free_window_+tracking_id_window_per_sec_*std::min(tracking_global_reacquire_unrestricted_after_sec_,gap);
    const int ng=std::min<int>(tracking_global_reacquire_geometric_candidates_,cand.size());
    for(int k=0;k<ng;++k){
      auto [sc,id,sh]=cand[k];Match x=verify(q,id,sh,sc);
      const int id_jump=last_tracking_id_>=0?std::abs(id-last_tracking_id_):0;
      bool branch_ok=!branch_guard||id_jump<=id_allow;
      double map_motion=0.0,map_limit=0.0,map_yaw_deg=0.0,yaw_limit=0.0;
      const Eigen::Isometry3d map_body=x.map_body*body_from_lidar_.inverse();
      const bool valid=x.id>=0&&std::isfinite(x.fitness)&&x.fitness<=tracking_global_reacquire_max_fitness_&&map_body.matrix().allFinite();
      if(tracking_global_reacquire_physical_gate_){
        branch_ok=valid;
        if(branch_ok&&last_tracking_map_valid_){
          const double age=std::max(0.0,(stamp-last_tracking_map_stamp_).toSec());
          map_motion=(map_body.translation()-last_tracking_map_body_.translation()).head<2>().norm();
          map_limit=tracking_global_reacquire_position_slack_m_+tracking_global_reacquire_max_speed_mps_*age;
          map_yaw_deg=std::abs(wrap(yawOf(map_body)-yawOf(last_tracking_map_body_)))*180.0/M_PI;
          yaw_limit=std::min(180.0,tracking_global_reacquire_consistency_yaw_deg_+tracking_global_reacquire_max_yaw_rate_deg_s_*age);
          const double vertical=std::abs(map_body.translation().z()-last_tracking_map_body_.translation().z());
          // The cart may descend/ascend across a 40s LiDAR outage.
          // Do not treat the OLD map height as a contemporaneous measurement,
          // but keep a finite physical envelope and all XY/yaw/GICP gates.
          double vertical_limit=tracking_global_reacquire_vertical_slack_m_;
          if(publish_critical_sensor_recovered_.load()&&
             tracking_global_reacquire_recovery_vertical_rate_mps_>0.0){
            const double cap=std::max(vertical_limit,tracking_global_reacquire_recovery_vertical_max_slack_m_);
            vertical_limit=std::min(cap,vertical_limit+
                tracking_global_reacquire_recovery_vertical_rate_mps_*age);
          }
          branch_ok=map_motion<=map_limit&&map_yaw_deg<=yaw_limit&&vertical<=vertical_limit;
          if(valid&&!branch_ok)
            ROS_INFO_THROTTLE(0.5,"[STEP3 RECOVERY-GATE] age %.1fs dz %.2f/%.2fm xy %.2f/%.2fm yaw %.1f/%.1fdeg",
                age,vertical,vertical_limit,map_motion,map_limit,map_yaw_deg,yaw_limit);
        }
      }
      ROS_INFO("[STEP3 GLOBAL-REACQ] candidate id=%d sc=%.3f gicp=%.4f idjump=%d allow=%.1f physical=%d motion=%.2f/%.2fm yaw=%.1f/%.1fdeg accepted_gate=%d",
               id,sc,x.fitness,id_jump,id_allow,tracking_global_reacquire_physical_gate_?1:0,map_motion,map_limit,map_yaw_deg,yaw_limit,branch_ok?1:0);
      if(x.fitness<best_any.fitness)best_any=x;
      if(branch_ok){
        // Only a soft preference for the FAST-predicted position: after a
        // tracking gap FAST can drift, so it must NOT veto good map geometry.
        const double pred_innov=(map_body.translation()-predicted_map_body.translation()).head<2>().norm();
        const double score=x.fitness+std::max(0.0,tracking_global_reacquire_predicted_pose_weight_)*pred_innov+
          gpsPenalty(id,gps_dist,gps_fitness_penalty_per_m_,0.06);
        if(score<best_physical_score){best_cont=x;best_physical_score=score;}
      }
    }
    if(tracking_global_reacquire_physical_gate_){
      if(best_cont.id>=0)return best_cont;
      ROS_WARN("[STEP3 GLOBAL-REACQ] no physically reachable high-quality map candidate; awaiting further scans");
      return Match();
    }
    if(branch_guard){
      if(best_cont.id>=0)return best_cont;
      ROS_WARN("[STEP3 GLOBAL-REACQ] no branch-continuous candidate after %.1fs gap; refusing wrong traversal until %.1fs",gap,tracking_global_reacquire_unrestricted_after_sec_);
      return Match();
    }
    return best_any;
  }

  void runGlobalReacquire(const Cloud::Ptr&q,const ros::Time& stamp,const Eigen::Isometry3d& fast_body,const Eigen::Isometry3d& anchor_snapshot,bool fast_body_valid){
    Match best=mh_enable_&&fast_body_valid?mhSelect(q,stamp,fast_body):
      (mh_enable_?Match():searchGlobalReacquire(q,stamp,anchor_snapshot*fast_body));
    if(best.id<0||!std::isfinite(best.fitness)||best.fitness>tracking_global_reacquire_max_fitness_){
      if(mh_enable_){publishStatus("RELOCALIZING: MH_UNTRUSTED");return;}
      global_reacquire_consistent_=0;
      global_reacquire_last_map_valid_=false;
      ++tracking_misses_;
      ROS_WARN("[STEP3 GLOBAL-REACQ] no trusted global match; best fitness %.4f misses=%d",best.fitness,tracking_misses_);
      publishStatus("RELOCALIZING");
      return;
    }
    const Eigen::Isometry3d map_body_meas=best.map_body*body_from_lidar_.inverse();
    if(mh_enable_){
      // A freshly verified multi-scan LiDAR map pose may be useful for an
      // external STOPPED-VEHICLE reset review even when Step3 cannot resume
      // its previously published global-pose stream. CANDIDATE ONLY:
      // never latched, not an EKF update and never a /pose publication.
      geometry_msgs::PoseWithCovarianceStamped review;
      review.header.frame_id="map";
      review.header.stamp=stamp;
      review.pose.pose=poseMsg(map_body_meas);
      review.pose.covariance.fill(0.0);
      review.pose.covariance[0]=4.0;
      review.pose.covariance[7]=4.0;
      review.pose.covariance[14]=9.0;
      review.pose.covariance[21]=0.3;
      review.pose.covariance[28]=0.3;
      review.pose.covariance[35]=0.3;
      mh_recovery_candidate_pub_.publish(review);
      // An anchor may be geometrically close to a long-propagated FAST pose,
      // while the last pose ACTUALLY exposed to downstream consumers is tens
      // of metres behind. Resuming output after such an outage produced an
      // 11.94m step in a 0.1s publication interval on localization_3.bag.
      // Never silently reactivate global pose after a long trust gap. An
      // explicit stopped-vehicle pose-reset/authority handshake is required.
      const double last_pub=mh_last_actual_pose_publication_sec_.load();
      const double public_gap=last_pub>0.0?stamp.toSec()-last_pub:0.0;
      if(last_pub>0.0 && (!std::isfinite(public_gap) ||
         public_gap>mh_max_auto_pose_resume_gap_sec_)){
        ROS_ERROR("[STEP3 MH] HANDOFF_REQUIRED after %.2fs WITHOUT published trusted global pose; candidate keyframe=%d GICP=%.4f. NO automatic /pose or map-pose reactivation",
                  public_gap,best.id,best.fitness);
        publishStatus("RELOCALIZING: MH_HANDOFF_REQUIRED");
        return;
      }
      // Do not turn a 10–30m branch change into an apparently smooth vehicle
      // motion. Keep the old anchor unchanged until genuinely safe evidence.
      const Eigen::Isometry3d predicted=anchor_snapshot*fast_body;
      const double error=(map_body_meas.translation()-predicted.translation()).head<2>().norm();
      const double yaw_error=std::abs(wrap(yawOf(map_body_meas)-yawOf(predicted)))*180/M_PI;
      if(error>mh_max_reanchor_m_||yaw_error>mh_max_reanchor_yaw_deg_){
        ROS_ERROR("[STEP3 MH] UNSAFE_REANCHOR id=%d error=%.2fm/%.1fdeg limits=%.1fm/%.1fdeg. Keeping last anchor; global pose withheld.",
                  best.id,error,yaw_error,mh_max_reanchor_m_,mh_max_reanchor_yaw_deg_);
        publishStatus("RELOCALIZING: MH_UNSAFE_REANCHOR");
        return;
      }
    }
    Eigen::Isometry3d candidate_anchor=Eigen::Isometry3d::Identity();
    if(fast_body_valid)candidate_anchor=map_body_meas*fast_body.inverse();
    if(global_reacquire_last_map_valid_){
      const double ds=std::max(0.05,(stamp-global_reacquire_last_map_stamp_).toSec());
      const Eigen::Isometry3d d=global_reacquire_last_map_body_.inverse()*map_body_meas;
      const double motion=d.translation().norm();
      const double dy=std::abs(wrap(yawOf(map_body_meas)-yawOf(global_reacquire_last_map_body_)))*180.0/M_PI;
      const int id_jump=global_reacquire_last_id_>=0?std::abs(best.id-global_reacquire_last_id_):0;
      const double motion_limit=tracking_global_reacquire_consistency_translation_+tracking_global_reacquire_max_speed_mps_*ds;
      const double yaw_limit=tracking_global_reacquire_consistency_yaw_deg_+tracking_global_reacquire_max_yaw_rate_deg_s_*ds;
      const double id_limit=4.0+tracking_global_reacquire_max_id_rate_*ds;
      if(motion>motion_limit||dy>yaw_limit||(!tracking_global_reacquire_physical_gate_&&id_jump>id_limit)){
        ROS_WARN("[STEP3 GLOBAL-REACQ] map-continuity reset motion=%.2f/%.2fm yaw=%.1f/%.1fdeg idjump=%d/%.1f (ID gate %s)",motion,motion_limit,dy,yaw_limit,id_jump,id_limit,tracking_global_reacquire_physical_gate_?"disabled":"legacy");
        global_reacquire_consistent_=0;
      }
    }
    global_reacquire_last_map_body_=map_body_meas;
    global_reacquire_last_map_stamp_=stamp;
    global_reacquire_last_id_=best.id;
    global_reacquire_last_map_valid_=true;
    global_reacquire_candidate_anchor_=candidate_anchor;
    ++global_reacquire_consistent_;
    ROS_INFO("[STEP3 GLOBAL-REACQ] verified id=%d fitness=%.4f consistent=%d/%d map_body=[%.2f %.2f %.2f]",
             best.id,best.fitness,global_reacquire_consistent_,tracking_global_reacquire_required_consistent_,
             map_body_meas.translation().x(),map_body_meas.translation().y(),map_body_meas.translation().z());
    if(!mh_enable_&&global_reacquire_consistent_<tracking_global_reacquire_required_consistent_){publishStatus("RELOCALIZING: verifying");return;}

    geometry_msgs::PoseStamped mp;mp.header.stamp=stamp;mp.header.frame_id="map";mp.pose=poseMsg(map_body_meas);tracking_map_pose_pub_.publish(mp);
    updateTrackingMotionModel(map_body_meas,stamp);
    last_tracking_id_=best.id;
    last_tracking_accept_stamp_=stamp;
    last_verified_map_fix_stamp_sec_.store(stamp.toSec());
    updateVerifiedVerticalReference(stamp,map_body_meas);
    last_tracking_map_body_=map_body_meas;
    last_tracking_map_stamp_=stamp;
    last_tracking_map_valid_=true;
    last_tracking_fast_body_=fast_body;
    last_tracking_fast_valid_=fast_body_valid;
    tracking_misses_=0;

    if(fast_local_unstable_.load()||!fast_body_valid){
      const bool handoff_ready=fast_body_valid&&fast_local_recovery_ready_.load();
      {
        std::lock_guard<std::mutex>l(anchor_mtx_);
        updateMapFixFallbackTargetLocked(map_body_meas,stamp);
        correction_filter_valid_=false;
        correction_ramp_active_=false;
        correction_ramp_last_fraction_=0.0;
        if(handoff_ready){
          map_from_fast_=candidate_anchor;
          map_fix_fallback_active_=false;
          fast_local_unstable_.store(false);
          fast_local_recovery_ready_.store(false);
          fast_local_stable_samples_=0;
          publish_recovery_duration_sec_=tracking_global_reacquire_ramp_sec_;
          publish_recovery_active_=false;
          publish_recovery_pending_=last_published_global_pose_valid_;
        }
      }
      if(handoff_ready){
        global_reacquire_consistent_=0;
        global_reacquire_last_map_valid_=false;
        resetTrajectoryFit();
        addTrajectoryFitPair(stamp,fast_body,map_body_meas);
        geometry_msgs::PoseStamped lock;lock.header.stamp=stamp;lock.header.frame_id="map_from_camera_init";lock.pose=poseMsg(candidate_anchor);lock_pub_.publish(lock);
        publishStatus("FAST_REBASED_FROM_MAP");
        ROS_WARN("[STEP3 MAP-FALLBACK] FAST handoff after trusted map fix id=%d fitness=%.4f",best.id,best.fitness);
      }else{
        global_reacquire_consistent_=tracking_global_reacquire_required_consistent_;
        publishStatus("MAP_FIX_FALLBACK");
        ROS_WARN_THROTTLE(1.0,"[STEP3 MAP-FALLBACK] following trusted map pose id=%d fitness=%.4f map=[%.2f %.2f %.2f]",best.id,best.fitness,map_body_meas.translation().x(),map_body_meas.translation().y(),map_body_meas.translation().z());
      }
      return;
    }

    const Eigen::Isometry3d pred_map_body=anchor_snapshot*fast_body;
    const Eigen::Vector3d residual=map_body_meas.translation()-pred_map_body.translation();
    const double yaw_residual=wrap(yawOf(map_body_meas)-yawOf(pred_map_body));
    const double correction_norm=residual.norm();
    const double yaw_residual_deg=std::abs(yaw_residual)*180.0/M_PI;
    const double publish_ramp_sec=publishSmoothingDuration(correction_norm,yaw_residual);
    {
      std::lock_guard<std::mutex>l(anchor_mtx_);
      // IMPORTANT: the matcher must immediately use the trusted map anchor.
      // Smoothing map_from_fast_ itself makes the next search centre stale and
      // creates a feedback loop (drift -> missed match -> more drift).  Keep
      // the estimator/map anchor exact and smooth only the published output.
      map_from_fast_=candidate_anchor;
      correction_filter_valid_=false;
      correction_ramp_translation_.setZero();
      correction_ramp_yaw_=0.0;
      correction_ramp_start_stamp_=ros::Time(0);
      correction_ramp_duration_sec_=0.0;
      correction_ramp_last_fraction_=0.0;
      correction_ramp_active_=false;
      publish_recovery_duration_sec_=publish_ramp_sec;
      publish_recovery_active_=false;
      publish_recovery_pending_=last_published_global_pose_valid_;
    }
    global_reacquire_consistent_=0;
    global_reacquire_last_map_valid_=false;
    resetTrajectoryFit();
    addTrajectoryFitPair(stamp,fast_body,map_body_meas);
    publishStatus("GLOBAL_REACQUIRED_SMOOTH");
    if(mh_enable_){mh_tracks_.clear();mh_pose_untrusted_.store(false);}
    ROS_INFO("[STEP3 GLOBAL-REACQ] immediate internal re-anchor %.2fm/%.1fdeg; published smoothing %.2fs id=%d fitness=%.4f",
             correction_norm,yaw_residual_deg,publish_ramp_sec,best.id,best.fitness);
  }

  void trackLocked(const sensor_msgs::PointCloud2ConstPtr&m){
    if(!last_tracking_query_.isZero()&&(m->header.stamp-last_tracking_query_).toSec()<1.0/std::max(.1,tracking_hz_))return;
    last_tracking_query_=m->header.stamp;
    const double tracking_gap_s=last_tracking_accept_stamp_.isZero()?0.0:std::max(0.0,(m->header.stamp-last_tracking_accept_stamp_).toSec());
    const bool recovery_bridge=lidarRecoveryBridgeActive(m->header.stamp);
    const bool reacquire_mode=!recovery_bridge&&!last_tracking_accept_stamp_.isZero()&&tracking_gap_s>=tracking_reacquire_after_sec_;
    Eigen::Isometry3d fast_body=Eigen::Isometry3d::Identity();
    const bool fast_body_valid=odomAt(m->header.stamp,fast_body);
    if(fast_body_valid)fast_odom_missing_scans_=0;
    bool global_reacquire_mode=!recovery_bridge&&(fast_local_unstable_.load()||(!last_tracking_accept_stamp_.isZero()&&
      (tracking_gap_s>=tracking_global_reacquire_after_sec_ ||
       tracking_misses_>=tracking_global_reacquire_after_misses_)));
    if(!fast_body_valid){
      // A LiDAR registration callback can run before FAST-LIVO2 has published
      // the odometry sample for the same sensor timestamp.  That is callback
      // ordering / estimator latency, NOT evidence that the local estimator
      // failed.  Never switch to map-only recovery from this condition: doing
      // so produced a false 10-13 m global excursion while the raw FAST
      // trajectory was completely healthy.  Only odomCb's actual local-motion
      // plausibility guard is allowed to mark FAST unstable.
      ++fast_odom_missing_scans_;
      ros::Time latest_fast_stamp;Eigen::Isometry3d latest_fast_pose;
      const bool have_latest_fast=latestOdom(latest_fast_stamp,latest_fast_pose);
      const double odom_age=have_latest_fast?
          (m->header.stamp-latest_fast_stamp).toSec():
          std::numeric_limits<double>::infinity();
      ROS_WARN_THROTTLE(1.0,
          "[STEP3 ALIGN] no time-aligned FAST pose for LiDAR stamp (scan-latest=%.3fs, misses=%d); keeping FAST propagation and skipping map update",
          odom_age,fast_odom_missing_scans_);
      return;
    }
    Cloud::Ptr q=preprocess(m);if(q->size()<150)return;
    Eigen::Isometry3d anchor_snapshot;{std::lock_guard<std::mutex>l(anchor_mtx_);anchor_snapshot=map_from_fast_;}
    if(global_reacquire_mode){
      if(mh_enable_)mh_pose_untrusted_.store(true);
      // Full-map descriptor + GICP search is deliberately much lower-rate than
      // normal tracking. Recovery must never starve FAST-LIVO2 sensor processing.
      const double global_period=1.0/std::max(0.05,tracking_global_reacquire_hz_);
      if(!last_global_reacquire_query_.isZero() &&
         (m->header.stamp-last_global_reacquire_query_).toSec()<global_period) return;
      last_global_reacquire_query_=m->header.stamp;
      // Once the local prediction has been unconstrained for long enough it is
      // not a valid search centre anymore. Re-run true global localization
      // against the whole map instead of rejecting correct matches as "large
      // innovations" relative to a stale anchor.
      runGlobalReacquire(q,m->header.stamp,fast_body,anchor_snapshot,fast_body_valid);
      return;
    }
    if(!fast_body_valid)return;
    Eigen::Isometry3d pred_map_body=anchor_snapshot*fast_body;
    // Propagate the LAST verified map match with relative FAST motion.
    // Unlike the current smart/filtered map anchor, this cannot inherit the
    // previous publishing correction. Only use it over short healthy gaps.
    Eigen::Isometry3d continuity_pred=Eigen::Isometry3d::Identity();
    const bool continuity_pred_valid=tracking_continuity_retry_enable_ &&
        last_tracking_fast_valid_ && last_tracking_map_valid_ &&
        tracking_gap_s>=0.0 && tracking_gap_s<=tracking_continuity_retry_max_age_sec_ &&
        !fast_local_unstable_.load();
    if(continuity_pred_valid)
      continuity_pred=last_tracking_map_body_*(last_tracking_fast_body_.inverse()*fast_body);
    Eigen::Isometry3d gap_pred=Eigen::Isometry3d::Identity();double gap_pred_age=0.0;
    if(tracking_gap_s>=tracking_gap_predict_after_sec_&&trackingMotionPrediction(m->header.stamp,gap_pred,gap_pred_age)){
      pred_map_body=gap_pred;
      ROS_WARN_THROTTLE(0.5,"[STEP3 GAP-PRED] using map-motion search prediction age=%.2fs",gap_pred_age);
    }
    const Eigen::Isometry3d pred_map_lidar=pred_map_body*body_from_lidar_;
    const double id_free_window=tracking_id_free_window_+tracking_id_window_per_sec_*std::min(30.0,tracking_gap_s);
    // At repeated/overlapping parts of the route, nearest XY alone can select
    // a keyframe from the wrong traversal direction. Rank with a soft heading
    // penalty, then lazily evaluate GICP candidates: the common case pays for
    // one registration; candidates 2-3 are only tried when geometry or
    // innovation rejects the first.
    std::vector<std::pair<double,int>> near;near.reserve(map_poses_.size());
    for(int i=0;i<(int)map_poses_.size();++i){
      const Eigen::Vector3d d=map_poses_[i].translation()-pred_map_lidar.translation();
      const double dyaw=std::abs(wrap(yawOf(map_poses_[i])-yawOf(pred_map_lidar)));
      const double hp=tracking_heading_weight_m_per_rad_*dyaw;
      double branch_penalty=0.0;
      if(last_tracking_id_>=0){
        const double id_jump=std::abs(i-last_tracking_id_);
        const double excess=std::max(0.0,id_jump-id_free_window);
        branch_penalty=tracking_id_penalty_m_per_keyframe_*excess;
      }
      near.push_back({d.head<2>().squaredNorm()+hp*hp+branch_penalty*branch_penalty,i});
    }
    const int nk=std::min<int>(tracking_candidates_,near.size());
    std::partial_sort(near.begin(),near.begin()+nk,near.end());
    Match best;double innov=std::numeric_limits<double>::infinity(),yaw_innov=std::numeric_limits<double>::infinity();
    bool accepted=false;
    const double fit_limit=reacquire_mode?tracking_reacquire_max_fitness_:tracking_max_fitness_;
    const double translation_limit=reacquire_mode?tracking_reacquire_max_translation_innovation_:tracking_max_translation_innovation_;
    const double yaw_limit=reacquire_mode?tracking_reacquire_max_yaw_innovation_deg_:tracking_max_yaw_innovation_deg_;
    double fitted_yaw_for_gate=0.0,fitted_fast_path=0.0,fitted_map_path=0.0;
    const bool trajectory_fit_ready=fittedTrajectoryYaw(fitted_yaw_for_gate,fitted_fast_path,fitted_map_path);
    for(int k=0;k<nk;++k){
      int id=near[k].second;
      const double xy_dist=(map_poses_[id].translation()-pred_map_lidar.translation()).head<2>().norm();
      const double heading_diff=std::abs(wrap(yawOf(map_poses_[id])-yawOf(pred_map_lidar)))*180.0/M_PI;
      Eigen::Isometry3d init=map_poses_[id].inverse()*pred_map_lidar;
      Match x=verifyTracking(q,id,init);
      const Eigen::Isometry3d x_body=x.map_body*body_from_lidar_.inverse();
      const double xi=(x_body.translation()-pred_map_body.translation()).norm();
      const double yi=std::abs(wrap(yawOf(x_body)-yawOf(pred_map_body)))*180.0/M_PI;
      const int id_jump=last_tracking_id_>=0?std::abs(id-last_tracking_id_):0;
      ROS_INFO("[STEP3 TRACK] scan=%.6f candidate=%d rank=%d id_jump=%d free=%.1f gap=%.1fs reacq=%d dist=%.2fm heading=%.1fdeg gicp=%.4f innov=%.2fm/%.1fdeg map_lidar=[%.3f %.3f %.3f] map_yaw=%.2fdeg",
               m->header.stamp.toSec(),id,k+1,id_jump,id_free_window,tracking_gap_s,reacquire_mode?1:0,xy_dist,heading_diff,x.fitness,xi,yi,
               x.map_body.translation().x(),x.map_body.translation().y(),x.map_body.translation().z(),
               yawOf(x.map_body)*180.0/M_PI);
      const bool normal_gate=
          x.id>=0&&std::isfinite(x.fitness)&&x.fitness<=fit_limit&&
          std::isfinite(xi)&&xi<=translation_limit&&
          std::isfinite(yi)&&yi<=yaw_limit;
      // Relax yaw only when every other cue says this is the SAME route
      // continuation: strong geometry, small metric innovation, and keyframe
      // progression inside the current ID window. The smart SE(2) fitter then
      // corrects the observed angle gradually; this is not a pose snap.
      const bool relaxed_yaw_gate=
          !reacquire_mode&&x.id>=0&&std::isfinite(x.fitness)&&
          x.fitness<=tracking_relaxed_yaw_max_fitness_&&
          std::isfinite(xi)&&xi<=tracking_relaxed_yaw_max_translation_&&
          std::isfinite(yi)&&yi<=tracking_relaxed_yaw_max_deg_&&
          id_jump<=id_free_window;
      // Some map sections have fewer overlapping VLP-16 structures and a
      // correct local registration can land around 0.20-0.23 fitness. Keep
      // continuity there only when pose innovation and keyframe progression are
      // simultaneously sane. This prevents a brief weak-geometry patch from
      // causing seconds of blind FAST-only drift.
      const bool route_continuous_geometry_gate=
          !reacquire_mode&&x.id>=0&&std::isfinite(x.fitness)&&
          x.fitness<=tracking_route_continuous_max_fitness_&&
          std::isfinite(xi)&&xi<=tracking_route_continuous_max_translation_&&
          std::isfinite(yi)&&yi<=tracking_route_continuous_max_yaw_deg_&&
          id_jump<=std::min(6.0,id_free_window);
      bool motion_consistent=true;
      if(tracking_healthy_motion_consistency_gate_ &&
         x.fitness>tracking_healthy_motion_consistency_strong_fitness_ &&
         !reacquire_mode &&
         last_tracking_fast_valid_ && last_tracking_map_valid_ &&
         !fast_local_unstable_.load()){
        const double gap=(m->header.stamp-last_tracking_map_stamp_).toSec();
        if(gap>0.3 && gap<=tracking_healthy_motion_consistency_max_gap_sec_){
          const double local_distance=(fast_body.translation()-last_tracking_fast_body_.translation()).norm();
          const double map_distance=(x_body.translation()-last_tracking_map_body_.translation()).norm();
          const double allowance=tracking_healthy_motion_consistency_slack_m_+
              tracking_healthy_motion_consistency_rate_mps_*gap;
          // One-sided check: a saved-map fix saying the cart moved LESS than
          // the local filter is essential evidence *against* FAST drift and
          // must remain admissible. Reject only impossible extra PCD travel.
          motion_consistent=std::isfinite(local_distance)&&std::isfinite(map_distance)&&
              map_distance-local_distance<=allowance;
          if(!motion_consistent)
            ROS_WARN("[STEP3 MOTION-GATE] reject GICP id=%d fitness=%.4f map travel=%.2fm FAST travel=%.2fm allowed=%.2fm dt=%.2fs",
                id,x.fitness,map_distance,local_distance,allowance,gap);
        }
      }
      if(motion_consistent && (normal_gate||relaxed_yaw_gate||route_continuous_geometry_gate)){
        if(relaxed_yaw_gate&&!normal_gate)
          ROS_WARN("[STEP3 TRACK] accepting route-continuous heading-drift match id=%d fitness=%.4f innov=%.2fm/%.1fdeg",
                   id,x.fitness,xi,yi);
        else if(route_continuous_geometry_gate&&!normal_gate)
          ROS_WARN("[STEP3 TRACK] accepting route-continuous weak-geometry match id=%d fitness=%.4f innov=%.2fm/%.1fdeg",
                   id,x.fitness,xi,yi);
        best=x;innov=xi;yaw_innov=yi;accepted=true;break;
      }
      if(x.fitness<best.fitness){best=x;innov=xi;yaw_innov=yi;}
    }
    bool route_reacquired=false;
    if(!accepted && continuity_pred_valid && !reacquire_mode){
      // Retry at most two local submaps near the propagated verified pose.
      // Never run a new global descriptor search here, and never relax the
      // geometry or vehicle-motion gates just to fill gaps on the plot.
      const Eigen::Isometry3d seed_lidar=continuity_pred*body_from_lidar_;
      std::vector<std::pair<double,int>> candidates;
      candidates.reserve(map_poses_.size());
      for(int id=0;id<(int)map_poses_.size();++id){
        const Eigen::Vector3d delta=map_poses_[id].translation()-seed_lidar.translation();
        const double heading=std::abs(wrap(yawOf(map_poses_[id])-yawOf(seed_lidar)));
        candidates.push_back({delta.head<2>().squaredNorm()+9.0*heading*heading,id});
      }
      const int retries=std::min<int>(2,candidates.size());
      std::partial_sort(candidates.begin(),candidates.begin()+retries,candidates.end());
      int repeated_seed_attempts=0;
      for(int k=0;k<retries;++k){
        const int id=candidates[k].second;
        bool already_checked=false;
        for(int j=0;j<nk;++j)if(near[j].second==id)already_checked=true;
        // Normal candidates start GICP from the current map<-FAST anchor.
        // After a tight turn that anchor can be ~2m away from the pose
        // propagated from the last independent verified PCD match. The old
        // retry SKIPPED any keyframe already tried, even with this different
        // and often better geometric seed. Retry at most ONE such keyframe.
        if(already_checked){
          if(!tracking_healthy_continuity_reseed_enable_ || repeated_seed_attempts>=1 ||
             !publish_full_sensor_mode_.load() || publish_critical_sensor_recovered_.load())
            continue;
          const double seed_delta=(continuity_pred.translation()-pred_map_body.translation()).norm();
          const double seed_dyaw=std::abs(wrap(yawOf(continuity_pred)-yawOf(pred_map_body)))*180.0/M_PI;
          if(seed_delta<tracking_healthy_continuity_reseed_min_seed_delta_m_ && seed_dyaw<3.0)
            continue;
          ++repeated_seed_attempts;
          ROS_INFO("[STEP3 CONTINUITY-RESEED] retry id=%d with verified FAST-propagated seed; difference=%.2fm/%.1fdeg",
              id,seed_delta,seed_dyaw);
        }
        const Eigen::Isometry3d initial=map_poses_[id].inverse()*seed_lidar;
        const Match x=verifyTracking(q,id,initial);
        const Eigen::Isometry3d body=x.map_body*body_from_lidar_.inverse();
        const double displacement=(body.translation()-continuity_pred.translation()).head<2>().norm();
        const double angle=std::abs(wrap(yawOf(body)-yawOf(continuity_pred)))*180.0/M_PI;
        const double since=std::max(0.1,tracking_gap_s);
        const double motion=(body.translation()-last_tracking_map_body_.translation()).head<2>().norm();
        const double heading_rate=std::abs(wrap(yawOf(body)-yawOf(last_tracking_map_body_)))*180.0/M_PI/since;
        // Keyframe IDs are storage order, not road-network distance. At a
        // loop, a physically adjacent submap can be dozens of IDs away; the
        // old arbitrary ID condition discarded verified ~0.4m/1.6deg matches
        // and forced a multi-second map-matching gap. The opt-in physical
        // mode still requires strong GICP geometry, bounded map distance,
        // local-motion agreement, and heading-rate consistency.
        const bool id_ok=tracking_continuity_retry_physical_only_enable_ ||
            std::abs(id-last_tracking_id_)<=std::max(12.0,id_free_window+8.0);
        const bool safe=x.id>=0&&std::isfinite(x.fitness)&&
          x.fitness<=(already_checked?tracking_healthy_continuity_reseed_max_fitness_:0.16)&&
          std::isfinite(displacement)&&displacement<=2.0&&angle<=8.0&&
          motion<=2.0+8.0*since&&heading_rate<=50.0&&
          id_ok;
        ROS_INFO("[STEP3 CONTINUITY-RETRY] candidate=%d fitness=%.4f innov=%.2fm/%.1fdeg motion=%.1fm heading_rate=%.1fdeg/s accepted=%d",
                 id,x.fitness,displacement,angle,motion,heading_rate,safe?1:0);
        if(safe){
          best=x;
          innov=displacement;
          yaw_innov=angle;
          accepted=true;
          ROS_WARN("[STEP3 CONTINUITY-RETRY] ACCEPT id=%d fitness=%.4f last_good_age=%.2fs",id,x.fitness,tracking_gap_s);
          break;
        }
      }
    }
    if(!accepted&&tracking_route_reacquire_enabled_&&last_tracking_id_>=0){
      Match route=searchRouteReacquire(q,m->header.stamp);
      if(route.id>=0&&std::isfinite(route.fitness)&&route.fitness<=tracking_route_reacquire_max_fitness_){
        const Eigen::Isometry3d route_body=route.map_body*body_from_lidar_.inverse();
        const double route_innov=(route_body.translation()-pred_map_body.translation()).norm();
        const double route_yaw=std::abs(wrap(yawOf(route_body)-yawOf(pred_map_body)))*180.0/M_PI;
        best=route;innov=route_innov;yaw_innov=route_yaw;accepted=true;route_reacquired=true;
        ROS_WARN("[STEP3 ROUTE-REACQ] ACCEPT id=%d fitness=%.4f innov=%.2fm/%.1fdeg after normal candidates failed",
                 best.id,best.fitness,innov,yaw_innov);
      }
    }
    // Apply the same independent motion check to the route/continuity retry.
    // Otherwise an inconsistent GICP candidate rejected above could be
    // reintroduced through a fallback code path and still jerk the anchor.
    if(accepted && tracking_healthy_motion_consistency_gate_ &&
       best.fitness>tracking_healthy_motion_consistency_strong_fitness_ &&
       !reacquire_mode && last_tracking_fast_valid_ && last_tracking_map_valid_ &&
       !fast_local_unstable_.load()){
      const double gap=(m->header.stamp-last_tracking_map_stamp_).toSec();
      if(gap>0.3 && gap<=tracking_healthy_motion_consistency_max_gap_sec_){
        const Eigen::Isometry3d accepted_body=best.map_body*body_from_lidar_.inverse();
        const double map_dist=(accepted_body.translation()-last_tracking_map_body_.translation()).norm();
        const double local_dist=(fast_body.translation()-last_tracking_fast_body_.translation()).norm();
        const double limit=tracking_healthy_motion_consistency_slack_m_+
            tracking_healthy_motion_consistency_rate_mps_*gap;
        if(!std::isfinite(map_dist)||!std::isfinite(local_dist)||
           map_dist-local_dist>limit){
          ROS_WARN("[STEP3 MOTION-GATE] rejecting fallback GICP id=%d map=%.2fm FAST=%.2fm slack=%.2fm over %.2fs",
            best.id,map_dist,local_dist,limit,gap);
          accepted=false;route_reacquired=false;
        }
      }
    }
    if(!accepted){
      ++tracking_misses_;
      if(best.id<0||!std::isfinite(best.fitness)||best.fitness>tracking_max_fitness_)
        ROS_WARN("[STEP3 TRACK] rejected candidates; best fitness %.4f misses=%d reacq=%d",best.fitness,tracking_misses_,reacquire_mode?1:0);
      else
        ROS_WARN("[STEP3 TRACK] rejected candidates; best innovation %.2fm/%.1fdeg fitness=%.4f misses=%d reacq=%d",innov,yaw_innov,best.fitness,tracking_misses_,reacquire_mode?1:0);
      return;
    }
    // best.map_body is T_map_lidar at the source scan time. Convert to body.
    // Raw FAST-LIVO2 remains the local motion authority. The verified map
    // registration updates ONLY map<-camera_init, analogous to a loop-closure
    // correction. This preserves the internally consistent live voxel map.
    // Preserve raw accepted GICP for /step3/gicp_map_pose metrics.  A normal
    // scan matcher can settle ~2m beside the original map traversal on the
    // tight upper-right turn: its nearest structures belong to the parallel
    // pass. If the time-continuous keyframe branch is unambiguous, add a
    // small lateral map-keyframe prior to the INTERNAL anchor only. Crucially,
    // never modify forward (along-track) distance, map registration history,
    // or local FAST-LIVO2 ESIKF. No snapping or use of evaluation reference.
    const Eigen::Isometry3d raw_map_body_meas=best.map_body*body_from_lidar_.inverse();
    Eigen::Isometry3d map_body_meas=raw_map_body_meas;
    const bool route_branch_consistent=last_tracking_id_>=0 &&
        std::abs(best.id-last_tracking_id_)<=8;
    const bool apply_route_lateral_prior=tracking_healthy_route_lateral_prior_enable_ &&
        publish_full_sensor_mode_.load() && !fast_local_unstable_.load() &&
        !reacquire_mode && !route_reacquired && route_branch_consistent &&
        best.id>=1 && best.id+1<(int)map_poses_.size() &&
        std::isfinite(best.fitness) &&
        best.fitness>=tracking_healthy_route_lateral_prior_min_fitness_ &&
        best.fitness<=tracking_healthy_route_lateral_prior_max_fitness_;
    if(apply_route_lateral_prior){
      const Eigen::Vector2d raw_xy=best.map_body.translation().head<2>();
      double nearest_lateral=std::numeric_limits<double>::infinity();
      Eigen::Vector2d nearest_point=raw_xy;
      for(int j=std::max(0,best.id-8);
          j<std::min((int)map_poses_.size()-1,best.id+8);++j){
        const Eigen::Vector2d a=map_poses_[j].translation().head<2>();
        const Eigen::Vector2d b=map_poses_[j+1].translation().head<2>();
        const Eigen::Vector2d ab=b-a;
        if(ab.squaredNorm()<0.25)continue;
        const double u=std::max(0.0,std::min(1.0,(raw_xy-a).dot(ab)/ab.squaredNorm()));
        const Eigen::Vector2d foot=a+u*ab;
        const double distance=(raw_xy-foot).norm();
        if(distance<nearest_lateral){nearest_lateral=distance;nearest_point=foot;}
      }
      if(std::isfinite(nearest_lateral) &&
         nearest_lateral>tracking_healthy_route_lateral_prior_deadband_m_ &&
         nearest_lateral<=tracking_healthy_route_lateral_prior_max_offset_m_){
        const double correction=std::min(tracking_healthy_route_lateral_prior_max_step_m_,
            tracking_healthy_route_lateral_prior_gain_*
            (nearest_lateral-tracking_healthy_route_lateral_prior_deadband_m_));
        const Eigen::Vector2d delta=correction/
            std::max(1.e-8,nearest_lateral)*(nearest_point-raw_xy);
        // body_from_lidar has rotation but zero translation for this dataset;
        // apply correction in map frame, independent of sensor orientation.
        map_body_meas.translation().head<2>()+=delta;
        ROS_INFO("[STEP3 ROUTE-LATERAL] id=%d fitness=%.4f raw_lateral=%.2fm applied=%.2fm raw_xy=[%.2f %.2f] fused_xy=[%.2f %.2f]",
             best.id,best.fitness,nearest_lateral,correction,
             raw_map_body_meas.translation().x(),raw_map_body_meas.translation().y(),
             map_body_meas.translation().x(),map_body_meas.translation().y());
      }
    }

    // Save the original anchor, before map_from_fast_ absorbs this LiDAR fix.
    // Otherwise the ESIKF measurement becomes identically zero by algebra.
    const Eigen::Isometry3d anchor_before_measurement=anchor_snapshot;
    const Eigen::Vector3d map_residual=map_body_meas.translation()-pred_map_body.translation();
    double map_yaw_residual=wrap(yawOf(map_body_meas)-yawOf(pred_map_body));
    if(tracking_healthy_gap_handoff_enable_ && publish_full_sensor_mode_.load() &&
       !publish_critical_sensor_recovered_.load() && !reacquire_mode &&
       tracking_gap_s>=tracking_healthy_gap_handoff_trigger_gap_sec_ &&
       map_residual.head<2>().norm()>=tracking_healthy_gap_handoff_trigger_residual_m_ &&
       std::abs(map_yaw_residual)*180.0/M_PI>=tracking_healthy_gap_handoff_trigger_yaw_deg_ &&
       best.fitness<=0.12 && std::isfinite(best.fitness)){
      healthy_gap_handoff_start_sec_.store(m->header.stamp.toSec());
      ROS_WARN("[STEP3 GAP-HANDOFF] new trusted PCD fix after %.2fs match gap: %.2fm/%.1fdeg correction; bounding published lateral/yaw for %.1fs",
          tracking_gap_s,map_residual.head<2>().norm(),std::abs(map_yaw_residual)*180.0/M_PI,
          tracking_healthy_gap_handoff_duration_sec_);
    }

    updateTrackingMotionModel(map_body_meas,m->header.stamp);
    if(tracking_update_anchor_){
      addTrajectoryFitPair(m->header.stamp,fast_body,map_body_meas);
      const Eigen::Isometry3d raw_candidate_anchor=map_body_meas*fast_body.inverse();
      bool fit_used=false;double applied_yaw=0.0;
      const Eigen::Isometry3d candidate_anchor=smartAnchorAtCurrentPoint(
          anchor_snapshot,fast_body,map_body_meas.translation(),
          yawOf(raw_candidate_anchor),true,fit_used,applied_yaw);
      const double correction_norm=map_residual.norm();
      double publish_ramp_sec=publishSmoothingDuration(correction_norm,applied_yaw);
      if(route_reacquired)
        publish_ramp_sec=std::min(publish_ramp_sec,tracking_route_reacquire_publish_max_sec_);
      {
        std::lock_guard<std::mutex>l(anchor_mtx_);
        // Trusted GICP is the global measurement.  Apply it immediately to the
        // INTERNAL anchor so the next matcher query starts from the correct map
        // location.  Only the externally published orange pose is smoothed.
        map_from_fast_=candidate_anchor;
        correction_filter_valid_=false;
        correction_ramp_active_=false;
        correction_ramp_last_fraction_=0.0;
        if((publish_recovery_active_||publish_recovery_pending_)&&last_published_global_pose_valid_){
          // A large global-recovery ramp may still be in progress when the
          // next ordinary PCD fix arrives.  Do not replace that recovery with
          // a 0.2 s ramp just because this latest residual is tiny: doing so
          // drops the remaining multi-metre continuity offset in one or two
          // odometry samples.  Rebase the smoother from the CURRENT published
          // pose to the newly corrected internal target instead.
          const Eigen::Isometry3d new_target=map_from_fast_*fast_body;
          const double remaining_translation=(last_published_global_pose_.translation()-new_target.translation()).norm();
          const double remaining_yaw=wrap(yawOf(last_published_global_pose_)-yawOf(new_target));
          publish_recovery_duration_sec_=publishSmoothingDuration(remaining_translation,remaining_yaw);
          if(route_reacquired)
            publish_recovery_duration_sec_=std::min(publish_recovery_duration_sec_,tracking_route_reacquire_publish_max_sec_);
          publish_recovery_active_=false;
          publish_recovery_pending_=true;
          ROS_INFO("[STEP3 ANCHOR] preserving active published recovery: remaining %.2fm/%.1fdeg over %.2fs",
                   remaining_translation,std::abs(remaining_yaw)*180.0/M_PI,publish_recovery_duration_sec_);
        }else{
          publish_recovery_duration_sec_=publish_ramp_sec;
          publish_recovery_active_=false;
          publish_recovery_pending_=last_published_global_pose_valid_;
        }
        anchor_snapshot=map_from_fast_;
      }
      ROS_INFO("[STEP3 ANCHOR] smart SE2 update residual=%.2fm raw_yaw=%.2fdeg applied_yaw=%.2fdeg fit=%d; published smoothing %.2fs reacq=%d route_reacq=%d",
               correction_norm,std::abs(map_yaw_residual)*180.0/M_PI,
               applied_yaw*180.0/M_PI,fit_used?1:0,publish_ramp_sec,
               reacquire_mode?1:0,route_reacquired?1:0);
    }

    // ESIKF gets a bounded external-map measurement only when geometry and
    // motion agree strongly. Weak route-continuous matches are deliberately
    // NOT promoted to the estimator, regardless of their accepted map anchor.
    // Development-bag regression: injecting this extra map residual into
    // LIO_CAMERA_FAILED double-corrected an otherwise stable LiDAR+IMU
    // fallback. Continue publishing the verified PCD anchor in that mode,
    // but reserve in-ESIKF feedback for FULL_LIVO healthy operation.
    bool healthy_sensor_mode=false;
    {
      std::lock_guard<std::mutex> mode_lock(mode_mtx_);
      healthy_sensor_mode=(failsafe_mode_=="FULL_LIVO");
    }
    // Two successive direct LiDAR registrations against the fixed PCD give
    // an independent RELATIVE heading observation. Compare this to the local
    // ESIKF's relative heading over exactly the same timestamps to observe a
    // gyro-yaw-rate bias rather than forcibly moving the estimator's pose.
    // This is experimental and OFF unless expressly enabled in the replay.
    if(tracking_publish_gyro_bias_measurement_){
      const bool strong=healthy_sensor_mode&&!reacquire_mode&&!route_reacquired&&
          best.fitness<=0.12&&innov<=1.2&&yaw_innov<=6.0;
      if(!strong){gyro_bias_heading_pair_valid_=false;}
      else{
        if(gyro_bias_heading_pair_valid_){
          const double dt=(m->header.stamp-gyro_bias_previous_stamp_).toSec();
          if(dt>=0.6&&dt<=2.2){
            const double map_dyaw=wrap(yawOf(map_body_meas)-yawOf(gyro_bias_previous_map_body_));
            const double fast_dyaw=wrap(yawOf(fast_body)-yawOf(gyro_bias_previous_fast_body_));
            const double error_rate=wrap(fast_dyaw-map_dyaw)/dt;
            if(std::isfinite(error_rate)&&std::abs(error_rate)<0.06){
              geometry_msgs::PoseWithCovarianceStamped corr;
              corr.header.stamp=m->header.stamp;
              corr.header.frame_id="step3_verified_gyro_bias_error_rate";
              corr.pose.pose.orientation.w=1.0;
              corr.pose.pose.position.z=error_rate;
              corr.pose.covariance.fill(0.0);
              corr.pose.covariance[0]=best.fitness;
              tracking_pub_.publish(corr);
              ROS_INFO("[STEP3 GYRO BIAS OBS] relative-heading rate mismatch=%.3fdeg/s over %.2fs fit=%.4f",
                       error_rate*180.0/M_PI,dt,best.fitness);
            }
          }
        }
        gyro_bias_previous_map_body_=map_body_meas;
        gyro_bias_previous_fast_body_=fast_body;
        gyro_bias_previous_stamp_=m->header.stamp;
        gyro_bias_heading_pair_valid_=true;
      }
    }
    // Compare a geometrically verified global registration with the local
    // filter in a FIXED coordinate frame. The normal map<-FAST publishing
    // anchor is updated at every registration and therefore cannot be used
    // for an ESIKF innovation: that would algebraically cancel the residual.
    // Freeze the initial map<-FAST lock as the measurement reference. This
    // measurement is independent of subsequent output-anchor smoothing.
    if(tracking_publish_anchored_local_measurement_ && healthy_sensor_mode &&
       anchored_local_initial_lock_valid_ && !reacquire_mode &&
       !route_reacquired && !fast_local_unstable_.load() &&
       best.fitness<=0.12 && innov<=1.5 && yaw_innov<=8.0){
      const Eigen::Isometry3d fast_body_measured=
          anchored_local_initial_map_fast_.inverse()*map_body_meas;
      const Eigen::Vector3d local_delta=fast_body_measured.translation()-fast_body.translation();
      const double local_yaw_delta=wrap(yawOf(fast_body_measured)-yawOf(fast_body));
      if(local_delta.allFinite() && std::isfinite(local_yaw_delta) &&
         local_delta.head<2>().norm()<=30.0 && std::abs(local_yaw_delta)<=0.6){
        geometry_msgs::PoseWithCovarianceStamped obs;
        obs.header.stamp=m->header.stamp;
        obs.header.frame_id="step3_verified_fixed_anchor_local_delta";
        obs.pose.pose.position.x=local_delta.x();
        obs.pose.pose.position.y=local_delta.y();
        obs.pose.pose.position.z=0.0;
        obs.pose.pose.orientation.z=std::sin(local_yaw_delta*0.5);
        obs.pose.pose.orientation.w=std::cos(local_yaw_delta*0.5);
        obs.pose.covariance.fill(0.0);
        obs.pose.covariance[0]=best.fitness;
        tracking_pub_.publish(obs);
        ROS_INFO("[STEP3 ANCHORED LOCAL OBS] dx=%.3f dy=%.3f dyaw=%.2fdeg fit=%.4f",
                 local_delta.x(),local_delta.y(),local_yaw_delta*180.0/M_PI,best.fitness);
      }
    }
    if(tracking_publish_esikf_residual_ && healthy_sensor_mode &&
       !reacquire_mode && !route_reacquired &&
       best.fitness <= 0.12 && innov <= 1.2 && yaw_innov <= 6.0){
      const Eigen::Isometry3d fast_body_target_source=
          anchor_before_measurement.inverse()*map_body_meas;
      const Eigen::Vector3d residual_pos=fast_body_target_source.translation()-fast_body.translation();
      double residual_yaw=wrap(yawOf(fast_body_target_source)-yawOf(fast_body));
      Eigen::Isometry3d residual_pose=Eigen::Isometry3d::Identity();residual_pose.translation()=residual_pos;residual_pose.linear()=Eigen::AngleAxisd(residual_yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
      geometry_msgs::PoseWithCovarianceStamped corr;corr.header.stamp=m->header.stamp;corr.header.frame_id="step3_gicp_local_residual";corr.pose.pose=poseMsg(residual_pose);corr.pose.covariance.fill(0.0);corr.pose.covariance[0]=best.fitness;corr.pose.covariance[7]=innov;corr.pose.covariance[35]=yaw_innov*M_PI/180.0;tracking_pub_.publish(corr);
      ROS_INFO("[STEP3 ESIKF MAP MEAS] published *pre-anchor* residual=%.3fm/%.2fdeg fitness=%.4f",
               residual_pos.head<2>().norm(),residual_yaw*180.0/M_PI,best.fitness);
    }
    geometry_msgs::PoseStamped mp;mp.header.stamp=m->header.stamp;mp.header.frame_id="map";
    // Raw GICP is deliberately kept in this topic, so reported registration
    // quality cannot improve merely because the saved keyframe prior moved it.
    mp.pose=poseMsg(raw_map_body_meas);tracking_map_pose_pub_.publish(mp);
    last_tracking_id_=best.id;last_tracking_accept_stamp_=m->header.stamp;tracking_misses_=0;global_reacquire_consistent_=0;
    last_verified_map_fix_stamp_sec_.store(m->header.stamp.toSec());
    updateVerifiedVerticalReference(m->header.stamp,map_body_meas);
    last_tracking_map_body_=map_body_meas;last_tracking_map_stamp_=m->header.stamp;last_tracking_map_valid_=true;
    last_tracking_fast_body_=fast_body;last_tracking_fast_valid_=true;
    ROS_INFO("[STEP3 TRACK] ACCEPT id=%d fitness=%.4f innov=%.2fm/%.2fdeg map_residual=%.2fm/%.2fdeg map_body=[%.2f %.2f %.2f] reacq=%d route_reacq=%d",best.id,best.fitness,innov,yaw_innov,map_residual.norm(),map_yaw_residual*180.0/M_PI,map_body_meas.translation().x(),map_body_meas.translation().y(),map_body_meas.translation().z(),reacquire_mode?1:0,route_reacquired?1:0);
  }
  void lidarCb(const sensor_msgs::PointCloud2ConstPtr&m){
    std::unique_lock<std::mutex> job(lidar_job_mtx_, std::try_to_lock);
    if(!job.owns_lock()) return;  // drop, never backlog stale global-registration scans
    if(locked_.load()){
      if(!tracking_deferred_scan_enable_){trackLocked(m);return;}
      // FAST output can arrive ~0.2-0.8 s behind LiDAR during tight turns.
      // Retain a tiny bounded window of ORIGINAL scans rather than forcing
      // an unverified extrapolated FAST pose into the GICP/map anchor.
      deferred_lidar_.push_back(m);
      const double max_age=std::max(0.25,tracking_deferred_scan_max_age_sec_);
      while(!deferred_lidar_.empty() &&
            (deferred_lidar_.size()>24 ||
             (m->header.stamp-deferred_lidar_.front()->header.stamp).toSec()>max_age)){
        deferred_lidar_.pop_front();++deferred_scans_expired_;
      }
      const double min_period=1.0/std::max(0.1,tracking_hz_);
      int ready_index=-1;
      for(int i=(int)deferred_lidar_.size()-1;i>=0;--i){
        const auto& scan=deferred_lidar_[i];
        if(!last_tracking_query_.isZero() &&
           (scan->header.stamp-last_tracking_query_).toSec()<min_period)continue;
        Eigen::Isometry3d aligned;
        if(odomAt(scan->header.stamp,aligned)){ready_index=i;break;}
      }
      if(ready_index>=0){
        const auto scan=deferred_lidar_[ready_index];
        deferred_lidar_.erase(deferred_lidar_.begin(),deferred_lidar_.begin()+ready_index+1);
        const double delay=(m->header.stamp-scan->header.stamp).toSec();
        if(delay>=0.15){
          ++deferred_scans_processed_;
          ROS_INFO("[STEP3 DEFERRED-LIDAR] aligned real FAST pose for scan %.6f after %.3fs delay; recovered=%zu expired=%zu",
                   scan->header.stamp.toSec(),delay,deferred_scans_processed_,deferred_scans_expired_);
        }
        trackLocked(scan);
      }else if(!deferred_lidar_.empty()){
        ROS_WARN_THROTTLE(1.0,"[STEP3 DEFERRED-LIDAR] awaiting FAST pose for saved LiDAR scan, buffered=%zu latest_delay=%.3fs expired=%zu",
                          deferred_lidar_.size(),(m->header.stamp-deferred_lidar_.front()->header.stamp).toSec(),deferred_scans_expired_);
      }
      return;
    }
    deferred_lidar_.clear();
    if(!last_query_.isZero()&&(m->header.stamp-last_query_).toSec()<1.0/std::max(.1,query_hz_))return;last_query_=m->header.stamp;Eigen::Isometry3d fast_body;if(!odomAt(m->header.stamp,fast_body)){ROS_WARN_THROTTLE(2,"[STEP3] waiting for time-aligned FAST pose");return;}Cloud::Ptr q=preprocess(m);if(q->size()<150){ROS_WARN("[STEP3] sparse query %zu",q->size());return;}
    if(mh_enable_){
      Match accepted=mhSelect(q,m->header.stamp,fast_body);
      if(accepted.id<0){publishStatus("SEARCHING: MH_COLLECTING");return;}
      const Eigen::Isometry3d fast_lidar=fast_body*body_from_lidar_;
      const Eigen::Isometry3d anchor=accepted.map_body*fast_lidar.inverse();
      {std::lock_guard<std::mutex> lock(anchor_mtx_);
        map_from_fast_=anchor;anchored_local_initial_map_fast_=anchor;
        anchored_local_initial_lock_valid_=true;}
      last_tracking_id_=accepted.id;last_tracking_accept_stamp_=m->header.stamp;
      last_tracking_map_body_=accepted.map_body*body_from_lidar_.inverse();
      last_tracking_map_stamp_=m->header.stamp;last_tracking_map_valid_=true;
      last_tracking_fast_body_=fast_body;last_tracking_fast_valid_=true;
      tracking_misses_=0;locked_.store(true);mh_pose_untrusted_.store(false);
      mh_tracks_.clear();
      geometry_msgs::PoseStamped lock;
      lock.header.stamp=m->header.stamp;lock.header.frame_id="map_from_camera_init";
      lock.pose=poseMsg(anchor);lock_pub_.publish(lock);
      publishStatus("GLOBAL_LOCKED");
      ROS_INFO("[STEP3 MH] COLD GLOBAL LOCK keyframe=%d fitness=%.4f",accepted.id,accepted.fitness);
      return;
    }
    Descriptor dq=makeDescriptor(*q);
    std::vector<std::pair<double,int>> ring;ring.reserve(desc_.size());for(int i=0;i<(int)desc_.size();++i){double d=0;for(int r=0;r<kRings;++r){double e=dq.ring[r]-desc_[i].ring[r];d+=e*e;}ring.push_back({d,i});}std::partial_sort(ring.begin(),ring.begin()+std::min<int>(descriptor_candidates_,ring.size()),ring.end());
    std::vector<std::tuple<double,int,int>> cand;for(int k=0;k<std::min<int>(descriptor_candidates_,ring.size());++k){int id=ring[k].second;auto ds=bestDescriptorDistance(dq,desc_[id]);if(ds.first<descriptor_threshold_)cand.push_back({ds.first,id,ds.second});}std::sort(cand.begin(),cand.end());if(cand.empty()){publishStatus("SEARCHING: no descriptor candidate");return;}
    const auto gps_dist=gpsDistancesAt(m->header.stamp);
    if(!gps_dist.empty())std::stable_sort(cand.begin(),cand.end(),[&](const auto&a,const auto&b){
      return std::get<0>(a)+gpsPenalty(std::get<1>(a),gps_dist,gps_sc_penalty_per_m_,0.06) <
             std::get<0>(b)+gpsPenalty(std::get<1>(b),gps_dist,gps_sc_penalty_per_m_,0.06);
    });
    Match best;double best_score=std::numeric_limits<double>::infinity();
    for(int k=0;k<std::min<int>(geometric_candidates_,cand.size());++k){
      auto [sc,id,sh]=cand[k];Match x=verify(q,id,sh,sc);
      const double gd=gps_dist.empty()?-1.0:gps_dist[id];
      ROS_INFO("[STEP3] candidate id=%d sc=%.3f gicp=%.4f gps_distance=%.1fm",id,sc,x.fitness,gd);
      const double score=x.fitness+gpsPenalty(id,gps_dist,gps_fitness_penalty_per_m_,0.06);
      if(x.id>=0&&std::isfinite(x.fitness)&&x.fitness<=gicp_fitness_&&score<best_score){
        best=x;best_score=score;
      }
    }
    if(best.id<0||!std::isfinite(best.fitness)||best.fitness>gicp_fitness_){consistent_=0;publishStatus("SEARCHING: GICP rejected candidates");return;}
    const Eigen::Isometry3d fast_lidar = fast_body * body_from_lidar_;
    Eigen::Isometry3d anchor=best.map_body*fast_lidar.inverse(); if(consistent_>0){Eigen::Isometry3d d=candidate_anchor_.inverse()*anchor;double dt=d.translation().norm(),dy=std::abs(wrap(yawOf(anchor)-yawOf(candidate_anchor_)))*180/M_PI;if(dt>consistency_translation_||dy>consistency_yaw_deg_){ROS_WARN("[STEP3] consistency reset %.2fm %.1fdeg",dt,dy);consistent_=0;}}
    candidate_anchor_=anchor;++consistent_;ROS_INFO("[STEP3] verified id=%d sc=%.3f gicp=%.4f consistent=%d/%d pose=[%.2f %.2f %.2f] yaw=%.1f",best.id,best.sc,best.fitness,consistent_,required_consistent_,best.map_body.translation().x(),best.map_body.translation().y(),best.map_body.translation().z(),yawOf(best.map_body)*180/M_PI);if(consistent_>=required_consistent_){{std::lock_guard<std::mutex>l(anchor_mtx_);map_from_fast_=candidate_anchor_;anchored_local_initial_map_fast_=candidate_anchor_;anchored_local_initial_lock_valid_=true;}last_tracking_id_=best.id;last_tracking_accept_stamp_=m->header.stamp;tracking_misses_=0;global_reacquire_consistent_=0;locked_.store(true);geometry_msgs::PoseStamped lock;lock.header.stamp=m->header.stamp;lock.header.frame_id="map_from_camera_init";lock.pose=poseMsg(map_from_fast_);lock_pub_.publish(lock);publishFailoverSeed(m->header.stamp,map_from_fast_*fast_body);publishStatus("GLOBAL_LOCKED");ROS_INFO("[STEP3] GLOBAL LOCK map<-fast xyz=[%.3f %.3f %.3f] yaw=%.2fdeg keyframe=%d",map_from_fast_.translation().x(),map_from_fast_.translation().y(),map_from_fast_.translation().z(),yawOf(map_from_fast_)*180/M_PI,last_tracking_id_);}
  }
};

int main(int argc,char**argv){
  ros::init(argc,argv,"step3_global_acquirer");
  try{
    Step3GlobalAcquirer n;
    // GICP is CPU-heavy. Keep odometry callbacks alive concurrently while a
    // LiDAR registration job runs, otherwise the tracker compares stale scans
    // against delayed pose state at real-time playback.
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
  }catch(const std::exception&e){ROS_FATAL("Step3 acquirer: %s",e.what());return 1;}
  return 0;
}
