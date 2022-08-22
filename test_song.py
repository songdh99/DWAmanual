#!/usr/bin/env python
# -*- coding: utf-8 -*-
from multiprocessing import current_process
import turtle
import numpy as np

dwa_mode = "wait"
sub_mode = "foward_goal"
mps_n = 2
rps_n = 13
mps = [0.15, 0.10]
radps = [0, 0.3, 0.5, 0.6, 0.7, 0.8, 0.9, -0.3, -0.5, -0.6, -0.7, -0.8, -0.9]
goal_stop_x = 1.2225
goal_stop_y = -0.006
goal_distance = 0
current_speed = 0
turn_speed = 0
angle_of_turn = 0
step = 0.15 * np.arange(1, 11).reshape(10, 1, 1)

mps_ar = np.array(mps).reshape(mps_n, 1)
radps_ar = np.array(radps)
nozero_ar = np.delete(np.array(radps), 0)
zero_dis_step = mps_ar * step
rad_dis_step = 2 * np.sin(step * abs(nozero_ar) / 2) * (mps_ar / abs(nozero_ar)) 
step_distance = np.concatenate((zero_dis_step, rad_dis_step), axis=2) + 0.3
#print(step_distance)



angle_160 = np.arange(-80, 80).reshape(4, 40)
#step_angle_160 = np.int32(np.rint(angle_160 + np.degrees(-1 * step * radps_ar / 2) + np.degrees(-1 * radps_ar)))

#print(np.shape(step_angle_160))
#print(step_angle_160)
scan_range = np.full((1, ), 0) 
#scan_distance = scan_range[np.deg2rad(angle_160)]
data0 = np.random.randint(0, 40, size = 20)


data0 = np.random.randint(0, 40, size = 20)
test1 = angle_160[0 ,:][data0]
test2 = test1[data0]
print(test1)
#print(scan_range)
#scan_distance = scan_range[angle_160]
#theta = np.radians(angle_160)
#o2r_dis = np.hypot(step_distance * abs(np.sin(theta)), scan_distance - step_distance * np.cos(theta))
#o2r_dis_min = np.amin(o2r_dis, axis=0)
#obstacle_dis = o2r_dis_min[0]







    