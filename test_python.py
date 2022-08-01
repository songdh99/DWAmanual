#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from std_msgs.msg import Int32, Float32, Bool, String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan


mps = [0.15, 0.13]
mps_n = 2
radps = [0, 0.3, 0.5, 0.6, 0.7, 0.8, 0.9, -0.3, -0.5, -0.6, -0.7, -0.8, -0.9]

step = 0.15 * np.arange(1, 11).reshape(10, 1, 1)
mps_ar = np.array(mps).reshape(mps_n, 1)
zero_dis_step = mps_ar * step
nozero_ar = np.delete(np.array(radps), 0)
radps_ar = np.array(radps)

mps_ar = np.array(mps).reshape(mps_n, 1)
rad_dis_step = 2 * np.sin(step * abs(nozero_ar) / 2) * (mps_ar / abs(nozero_ar)) 
step_distance = np.concatenate((zero_dis_step, rad_dis_step), axis=2) + 0.3
scan_range = np.full((1, ), 0)

print(scan_range)


