#!/usr/bin/env python3
"""Use the vetted October-9 corner-fixed September reference, with fresh frozen-case names."""
import summarize_18sep_four_modes as summary
summary.NAMES = {
    'Healthy': 'HEALTHY_HOLDOUT_220S',
    'Camera failed': 'CAMERA_FAIL_HOLDOUT_220S',
    'LiDAR failed': 'LIDAR_FAIL_HOLDOUT_220S',
    'IMU failed': 'IMU_FAIL_HOLDOUT_220S',
}
if __name__=='__main__': summary.main()
