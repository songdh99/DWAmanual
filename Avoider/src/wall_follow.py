#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int32, Float32, Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SelfDrive:
    def __init__(self, publisher):
        self.publisher = publisher
        self.turtle_vel = Twist()
        self.count = 0

    def init_ranges(self, scan):
        
        scanned = np.array(scan)
        right = np.nan_to_num(np.mean(scanned[190:][np.nonzero(scanned[265:275])]))
        fright = np.nan_to_num(np.mean(scanned[275:340][np.nonzero(scanned[340:360])]))
        bright = np.nan_to_num(np.mean(scanned[245:265][np.nonzero(scanned[245:265])]))
        front = scanned[0]
        point = scanned[70]
        point2 = scanned[80]
        point3 = scanned[90]
        fleft = np.nan_to_num(np.mean(scanned[1:45][np.nonzero(scanned[1:45])]))
        left = np.nan_to_num(np.mean(scanned[45:135][np.nonzero(scanned[45:135])]))
        bleft = np.nan_to_num(np.mean(scanned[135:180][np.nonzero(scanned[135:180])]))
        test1 = scanned[1:45]
        test2 = np.trim_zeros(test1, trim = 'fb')

        return right, fright, front, fleft, left, bleft, bright, point, point2, point3, test1, test2

    def lds_callback(self, scan):
        #right, fright, front, fleft, left, bleft, bright, point, point2, point3, test1, test2 = self.init_ranges(scan.ranges)
        #scan_range = np.empty(shape=360)
        scan_range = np.array(scan.ranges)
        
        step_n = 10
        mps_n = 2
        rps_n = 13

        mps = [0.15, 0.13]
        radps = [0, 0.3, 0.5, 0.6, 0.7, 0.8, 0.9, -0.3, -0.5, -0.6, -0.7, -0.8, -0.9]
        mps_ar = np.array(mps).reshape(mps_n, 1)
        nozero_ar = np.delete(np.array(radps), 0)
        radps_ar = np.array(radps)

        step = 0.15 * np.arange(1, 11).reshape(10, 1, 1)

        zero_dis_step = mps_ar * step
        rad_dis_step = 2 * np.sin(step * abs(nozero_ar) / 2) * (mps_ar / abs(nozero_ar)) 
        #step_distance = np.concatenate((zero_dis_step, rad_dis_step), axis=2) + 0.3 # shape = (10, mps_c, rps_c)
        
        angle_160 = np.arange(-80, 80).reshape(4, 40)
        #angle_160_1 = np.arange(-80,80).reshape(160,1)
        
        #step_angle_160 = np.int32(np.rint(angle_160 + np.degrees(-1 * step * radps_ar / 2) + np.degrees(-1 * radps_ar)))
        
        
        #scan_distance = scan_range[step_angle_160]
        #scan_distance_1 = scan_range[angle_160_1]
        #print(angle_160, end = ' ')
        #print(scan_distance[:,1], end=' ')
        #theta = np.radians(angle_160)

        
        #near_dis_score = np.where(near_dis_score > 0.30, 0.30, near_dis_score)  
        #near_dis_score = np.where(near_dis_score < 0.12, -100, near_dis_score)  

        # print('distance : ', np.round_(obstacle_dis[:, 0], 4))
        scan_distance = scan_range[angle_160]
        # self.theta = np.radians(self.angle_160)
        # self.o2r_dis = np.hypot(self.step_distance * abs(np.sin(self.theta)), self.scan_distance - self.step_distance * np.cos(self.theta))
        # self.o2r_dis_min = np.amin(self.o2r_dis, axis=0)
        # obstacle_dis = self.o2r_dis_min[0]
        # ran_index = np.random.randint(0, 40, size = 20)
        # right = self.scan_distance[0 ,:]            #right
        # front_right = self.scan_distance[1 ,:]      #front_right
        # front_left = self.scan_distance[2 ,:]       #front_left
        # left = self.scan_distance[3 ,:]             #left

        # back = np.logical_and(np.logical_and(right[ran_index] >= 0.2, right[ran_index] < 0.55),
        #                       np.logical_and(front_right[ran_index] >= 0.2, front_right[ran_index] < 0.55),
        #                       np.logical_and(front_left[ran_index] >= 0.2, front_left[ran_index] < 0.55),
        #                       np.logical_and(left[ran_index] >= 0.2, left[ran_index] < 0.55))
        # side_left = np.logical_and(np.logical_and(right[ran_index] >= 0.2, right[ran_index] < 0.55),
        #                            np.logical_and(front_right[ran_index] >= 0.2, front_right[ran_index] < 0.55),
        #                            np.logical_or(front_left[ran_index] > front_right[ran_index]))
        # side_right = np.logical_and(np.logical_and(left[ran_index] >= 0,2, np.left[ran_index] < 0.55),
        #                             np.logical_and(front_left[ran_index] >= 0.2, front_left[ran_index] < 0.55),
        #                             np.logical_or(front_right[ran_index] > front_left[ran_index]))
        

        back = np.logical_and(self.scan_distance[0:3 ,:] >= 0.2, self.scan_distance[0:3 ,:] < 0.4)
        side_left = np.logical_and(np.logical_and(self.scan_distance[0, :] >= 0.2, self.scan_distance[0 ,:] < 0.55),
                                   self.scan_distance[2 ,:] > self.scan_distance[1 ,:])
        side_left2 = np.logical_and(np.logical_and(self.scan_distance[1 ,:] >= 0.2, self.scan_distance[1 ,:] < 0.55),
                                    self.scan_distance[2 ,:] > self.scan_distance[1 ,:])
        side_right = np.logical_and(np.logical_and(self.scan_distance[3 ,:] >= 0,2, self.scan_distance[3 ,:] < 0.55),
                                    self.scan_distance[2 ,:] < self.scan_distance[1 ,:])
        side_right2 = np.logical_and(np.logical_and(self.scan_distance[2 ,:] >= 0,2, self.scan_distance[2 ,:] < 0.55),
                                    self.scan_distance[2 ,:] < self.scan_distance[1 ,:]) 
        
        if True in back:
            rospy.loginfo("back")
            return -0.05, self.radps[self.best_score[1]]
        elif True in side_left and True in side_right:
            rospy.loginfo("side back")
            return -0.05, 0.
        elif True in side_left or True in side_left2: # 우회전
            rospy.loginfo("side left")
            #rospy.loginfo("mps = ", self.mps[self.best_score[0]])
            return self.mps[self.best_score[0]], -0.15
        elif True in side_right or True in side_right2: # 좌회전
            rospy.loginfo("side right")
            #rospy.loginfo("mps = ", self.mps[self.best_score[0]])
            return self.mps[self.best_score[0]], 0.15
        rospy.loginfo("straight")
        #rospy.loginfo("mps = ", self.mps[self.best_score[0]], "radps = ", self.radps[self.best_score[1]])
        return self.mps[self.best_score[0]], self.radps[self.best_score[1]]
        
        

def main():
    rospy.init_node('self_drive')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    subscriber = rospy.Subscriber('scan', LaserScan,
                                  lambda scan: driver.lds_callback(scan))
    rospy.Rate(10000)
    rospy.spin()


if __name__ == "__main__":
    main()

