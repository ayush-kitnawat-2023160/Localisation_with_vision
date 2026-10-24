#!/usr/bin/env python3
"""Static fail-closed safety regression for the experimental ROS1 launch."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
SAFE=ROOT/'launch/fresh_xfeat_mh_safe_offline.launch'
PARENT=ROOT/'launch/fresh_multimodal_global_rosbag.launch'


def arguments(root):
    return {a.attrib['name']:a.attrib.get('value',a.attrib.get('default'))
            for a in root.findall('arg')}


class SafeOfflineLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.safe=ET.parse(SAFE).getroot()
        cls.parent=ET.parse(PARENT).getroot()
        cls.args=arguments(cls.safe)
        cls.included_args={a.attrib['name']:a.attrib['value']
                           for a in cls.safe.find('include').findall('arg')}
        cls.parent_args=arguments(cls.parent)
    def test_untrusted_pose_never_enabled(self):
        self.assertEqual(self.included_args['publish_legacy_pose'],'false')
        self.assertEqual(self.parent_args['publish_legacy_pose'],'false')
    def test_gps_completely_disabled(self):
        self.assertEqual(self.included_args['lidar_gps_soft_prior_enable'],'false')
        self.assertEqual(self.parent_args['lidar_gps_soft_prior_enable'],'false')
    def test_visual_duplicate_disabled(self):
        self.assertEqual(self.included_args['visual_global_enable'],'false')
        self.assertEqual(self.included_args['visual_learned_enable'],'true')
    def test_multihypotheses_review_enabled(self):
        for name in ('lidar_mh_enable','lidar_mh_visual_seed_enable',
                     'lidar_handoff_review_enable'):
            self.assertEqual(self.included_args[name],'true')
    def test_full_lidar_map_and_reacquire(self):
        self.assertIn('campus_full_lio_226s_20261019',self.args['map_db'])
        self.assertEqual(self.included_args['map_db'],'$(arg map_db)')
        self.assertEqual(self.included_args['lidar_mh_reacquire_hz'],'$(arg mh_reacquire_hz)')
        self.assertEqual(self.args['mh_reacquire_hz'],'1.0')
    def test_authority_requires_hypothesis_trust(self):
        node=next(n for n in self.parent.iter('node') if n.attrib.get('name')=='fresh_pose_authority')
        params={p.attrib['name']:p.attrib['value'] for p in node.iter('param')}
        self.assertEqual(params['require_mh_trusted_status'],'$(arg lidar_mh_enable)')
        self.assertEqual(params['publish_legacy_pose'],'$(arg publish_legacy_pose)')
    def test_no_supervised_reset_executable(self):
        # Offline map review runs, but no component applies an epoch change.
        types={n.attrib.get('type') for n in self.safe.iter('node')}
        types.update(n.attrib.get('type') for n in self.parent.iter('node'))
        self.assertFalse(any('reset' in (x or '').lower() or 'transaction' in (x or '').lower() for x in types))


if __name__=='__main__':unittest.main(verbosity=2)
