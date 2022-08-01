#!/usr/bin/env python
# -*- coding: utf-8 -*-
import turtle
import numpy as np
import rospy
from std_msgs.msg import Int32, Float32, Bool, String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
dwa_mode = "wait"
sub_mode = "foward_goal"
mps_n = 2
rps_n = 13
mps = [0.15, 0.10]
radps = [0, 0.3, 0.5, 0.6, 0.7, 0.8, 0.9, -0.3, -0.5, -0.6, -0.7, -0.8, -0.9]

angle_160 = np.array(-80, 80).reshape(160,1,1,1)




class SelfDrive:
    def __init__(self, publisher):
        self.publisher = publisher
        
    def lds_callbasck(self, scan):
        turtle_vel = Twist()
        scan_range = np.array(scan.range)
        global dwa_mode
        global sub_mode
        turtle_vel = Twist()
        scan_range = np.array(scan.range)
        x = turtle_vel.linear.x
        z = turtle_vel.angular.z
        if sub_mode == "stop":
            x = 0.0
            z = 0.0


    

def directory_fun():

                