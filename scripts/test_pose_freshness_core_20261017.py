#!/usr/bin/env python3
import unittest
from measure_pose_freshness import safe_age
class AgeTests(unittest.TestCase):
 def test_recent(self):self.assertAlmostEqual(safe_age(1700000000.12,1700000000.07),.05,places=5)
 def test_future_small(self):self.assertAlmostEqual(safe_age(1700000000,1700000000.03),-.03,places=5)
 def test_future_bad(self):self.assertIsNone(safe_age(1700000000,1700000001))
 def test_bad_clock(self):self.assertIsNone(safe_age(7,6))
 def test_invalid(self):self.assertIsNone(safe_age(float('nan'),1700000000))
 def test_stale(self):self.assertIsNone(safe_age(1700000020,1700000000))
if __name__=='__main__':unittest.main()
