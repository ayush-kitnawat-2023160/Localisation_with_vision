#!/usr/bin/env python3
"""Static isolation checks (unit tests cannot physically certify the cart)."""
import json
from pathlib import Path
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
DEPLOY=ET.parse(str(ROOT/'launch/deployment_gpsfree_shadow_20261021.launch')).getroot()


def find_args():
    # Only top-level definitions: child <include><arg name=... value=...>
    # intentionally reuses names such as map_db and use_sim_time. Flattening
    # both overwrites required/default declarations with their forwarded
    # $(arg ...) expressions and creates false-positive test failures.
    return {x.attrib.get('name'):x.attrib.get('default')
       for x in DEPLOY.findall('arg')}


def forwarded_args():
    return {x.attrib.get('name'):x.attrib.get('value')
       for include in DEPLOY.findall('include') for x in include.findall('arg')}


class Tests(unittest.TestCase):
    def setUp(self):
        self.a=find_args()
    def test_live_clock_by_default(self):
        self.assertEqual(self.a['use_sim_time'],'false')
    def test_lidar_point_time_requires_explicit_driver_unit(self):
        self.assertEqual(self.a['lidar_point_time_unit'],
                         'UNSET_MUST_SPECIFY_SECONDS_OR_MICROSECONDS')
        self.assertEqual(forwarded_args()['lidar_point_time_unit'],
                         '$(arg lidar_point_time_unit)')
        runner=(ROOT/'scripts/run_deployment_gpsfree_shadow_rosbag_20261021.sh').read_text()
        self.assertIn('lidar_point_time_unit:=seconds',runner)
    def test_map_required_no_hardcoded_pose(self):
        self.assertIsNone(self.a['map_db'])
        self.assertIsNone(self.a['calibration_file'])
    def test_injector_disabled(self):
        self.assertEqual(forwarded_args()['failure_injector_enable'],'false')
    def test_legacy_rgbd_disabled(self):
        self.assertEqual(forwarded_args()['rgbd_frontend_enable'],'false')
    def test_no_legacy_nav_pose(self):
        self.assertEqual(forwarded_args()['publish_legacy_pose'],'false')
        auth=(ROOT/'scripts/fresh_pose_authority.py').read_text()
        self.assertIn('if self.allow_pose else None',auth)
    def test_canonical_navigation_channels_remapped_to_shadow(self):
        remap={x.attrib['from']:x.attrib['to'] for x in DEPLOY.findall('remap')}
        for original in ('/aft_mapped_to_init','/path','/tf',
                         '/fast_livo2/step3_global_lock',
                         '/fast_livo2/step3_tracking_correction',
                         '/fast_livo2/map_correction',
                         '/fast_livo2/step3_failover_initialpose',
                         '/fast_livo2/visual_global_candidate',
                         '/fast_livo2/fallback/camera_imu_odom_candidate'):
            self.assertTrue(remap.get(original,'').startswith('/fast_livo2/deployment/'),original)
    def test_step3_cannot_change_fast_livo2_state_in_shadow(self):
        p={x.attrib['name']:x.attrib.get('value') for x in DEPLOY.findall('param')}
        for name in ('step3/tracking_correction_enable','step3/prior_map_enable',
                     'step3/gyro_bias_feedback_enable',
                     'step3/anchored_local_feedback_enable',
                     'failsafe/map_correction_enable'):
            self.assertEqual(p.get(name),'false',name)
        self.assertEqual(forwarded_args()['external_map_correction_enable'],'false')
    def test_gps_disabled(self):
        self.assertEqual(forwarded_args()['lidar_gps_soft_prior_enable'],'false')
        self.assertEqual(forwarded_args()['visual_global_enable'],'false')
    def test_xfeat_enabled_by_default_in_shadow(self):
        self.assertEqual(self.a['learned_visual_enable'],'true')
        self.assertEqual(forwarded_args()['visual_learned_enable'],
                         '$(arg learned_visual_enable)')
    def test_multi_hypothesis_with_three_fresh_hits(self):
        self.assertEqual(forwarded_args()['lidar_mh_enable'],'true')
        self.assertEqual(forwarded_args()['lidar_mh_required_hits'],'3')
    def test_yaw_trajectory_fitting_disabled(self):
        self.assertEqual(forwarded_args()['lidar_trajectory_yaw_fit_enable'],'false')
    def test_fused_map_topic_namespaced_shadow_only(self):
        self.assertEqual(forwarded_args()['fused_map_pose_topic'],
                         '/fast_livo2/shadow_fused_map_pose')
    def test_camera_imu_has_shadow_only(self):
        allparams={p.attrib['name']:p.attrib['value'] for p in DEPLOY.iter('param')}
        self.assertEqual(allparams['shadow_only'],'true')
        self.assertEqual(allparams['require_imu_for_healthy'],'true')
    def test_status_core_never_motion_authorizes(self):
        code=(ROOT/'scripts/deployment_sensor_health_core.py').read_text()
        self.assertIn('autonomous_motion_authorized:bool=False',code)
        self.assertIn('may_auto_reset_map_anchor:bool=False',code)
    def test_new_runner_has_no_precomputed_odom(self):
        code=(ROOT/'scripts/run_deployment_gpsfree_shadow_rosbag_20261021.sh').read_text()
        self.assertNotIn('lidar_only_odom.bag',code)
        self.assertNotIn('/initialpose',code)
        self.assertIn('/mavros/imu/data',code)
    def test_live_launch_refuses_archived_calibration(self):
        proc=subprocess.run([sys.executable,str(ROOT/'scripts/preflight_gpsfree_deployment_20261021.py'),
            '--mode','live'],capture_output=True,text=True)
        self.assertEqual(proc.returncode,2)
        output=json.loads(proc.stdout)
        self.assertIn('LIVE_PROFILE_BAG_OR_INHERITED_MOUNT_REJECTED',output['errors'])
        self.assertFalse(output['eligible_for_navigation'])
    def test_offline_preflight_informs_not_navigation_ready(self):
        proc=subprocess.run([sys.executable,str(ROOT/'scripts/preflight_gpsfree_deployment_20261021.py'),
            '--mode','offline'],capture_output=True,text=True)
        self.assertEqual(proc.returncode,0)
        report=json.loads(proc.stdout)
        self.assertTrue(report['pass_static_preflight'])
        self.assertFalse(report['eligible_for_navigation'])


if __name__=='__main__':unittest.main(verbosity=2)
