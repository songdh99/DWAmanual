#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math



mps = [0.15, 0.13]
mps_n = 2
radps = [0, 0.3, 0.5, 0.6, 0.7, 0.8, 0.9, -0.3, -0.5, -0.6, -0.7, -0.8, -0.9]

step = 0.15 * np.arange(1, 11).reshape(10, 1, 1)    #(10,1)
mps_ar = np.array(mps).reshape(mps_n, 1)        #(2,1)
zero_dis_step = mps_ar * step   
nozero_ar = np.delete(np.array(radps), 0)
radps_ar = np.array(radps)

mps_ar = np.array(mps).reshape(mps_n, 1)
rad_dis_step = 2 * np.sin(step * abs(nozero_ar) / 2) * (mps_ar / abs(nozero_ar))
step_distance = np.concatenate((zero_dis_step, rad_dis_step), axis=2) + 0.3

angle160 = np.arange(-80, 80).reshape(160, 1, 1, 1)
scan_range = np.full((1, ), 0)
step_angle_160 = np.int32(np.rint(angle160 + np.degrees(-1 * step * radps_ar / 2) + np.degrees(-1 * radps_ar)))

theta = np.radians(angle160)
x = np.sin(theta)
y =np.array([[0],[0]])
z = np.zeros((2, 6))
print(z)



