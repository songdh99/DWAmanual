#!/home/pi/.pyenv/versions/rospy3/bin/python
# -- coding: utf-8 --

import numpy as np
import rospy
#import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Pose2D


def main():
    rospy.init_node('dwa_test')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    driver.calculation()
    while not rospy.is_shutdown():
        rospy.Subscriber('scan', LaserScan, lambda scan: driver.lds_callback(scan)) 
        rospy.spin()


class SelfDrive:
    def __init__(self, publisher):
        rospy.Subscriber('current_pose', Pose, self.current_pose)
        self.publisher = publisher
        self.global_pose = Pose2D()
        self.lidar_array = np.array([])
        self.mps = [0.2, 0.17, 0.15, 0.13, 0.1]
        self.radps = [0, 0.3, 0.5, 0.6, 0.7, 0.8, 0.9, -0.3, -0.5, -0.6, -0.7, -0.8, -0.9]
        self.radps_ar = np.array(self.radps)
        self.step = 0.15 * np.arange(1, 11).reshape(10, 1, 1)
        self.t_array = 0.25 * np.arange(1, 11).reshape(10, 1, 1)
        self.best_score = None
        self.radps_array = None
        self.motion = None
        self.local_pos = None
        self.start_x = -4.115
        self.start_y = 0.409
        self.goal_x = -1.633
        self.goal_y = 0.317
        self.save_x = -1.979
        self.save_y = 0.0
        self.angle_160 = np.arange(-80, 80).reshape(4, 40)
        #self.step_angle_160 = np.int32(np.rint(self.angle_160 + np.degrees(-1 * self.step * self.radps_ar / 2) + np.degrees(-1 * self.radps_ar)))

        self.scan_range = np.full((1, ), 0) 


    def current_pose(self, data):
        self.global_pose.x = data.position.x
        self.global_pose.y = data.position.y
        _, _, self.global_pose.theta = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])

    def lds_callback(self, scan):
        turtle_vel = Twist()
        self.remaining_scoring()
        self.scan_range = np.array(scan.ranges)
        turtle_vel.linear.x, turtle_vel.angular.z = self.obstacle_scoring()
        if np.hypot(self.goal_x - self.global_pose.x, self.goal_y - self.global_pose.y) <= 0.10:
            rospy.loginfo_once("done")
            turtle_vel.linear.x, turtle_vel.angular.z = 0., 0.
        self.publisher.publish(turtle_vel)

    def calculation(self):
        mps_array = np.array(self.mps).reshape(len(self.mps), 1)
        self.radps_array = np.delete(np.array(self.radps), 0)
        line_motion = mps_array * self.t_array      #선운동
        rotational_motion = 2 * (mps_array / self.radps_array ) * np.sin(0.5 * self.radps_array * self.t_array) + 0.05      #회전운동
        self.motion = np.concatenate((line_motion, rotational_motion), axis=2) + 0.4
        local_x_pos = np.concatenate((line_motion, rotational_motion * np.cos(0.5 * self.radps_array * self.t_array)), axis=2)
        local_y_pos = np.concatenate((np.zeros((10, len(self.mps), 1)), rotational_motion * np.sin(0.5 * self.radps_array * self.t_array)), axis=2)
        self.local_pos = np.concatenate((np.reshape((local_x_pos), (10, -1, 1)), np.reshape((local_y_pos), (10, -1, 1))), axis=2)

    def remaining_scoring(self):
        rotation_matrix = np.array(
            [[np.cos(self.global_pose.theta), -np.sin(self.global_pose.theta)],
             [np.sin(self.global_pose.theta), np.cos(self.global_pose.theta)]])
        local_matrix = np.round_((np.dot(self.local_pos, rotation_matrix.T)), 4)
        remain_x = self.goal_x - (self.global_pose.x + np.delete(local_matrix, 1, axis=2))
        remain_y = self.goal_y - (self.global_pose.y + np.delete(local_matrix, 0, axis=2))
        score = np.reshape(np.hypot(remain_x, remain_y), (10, len(self.mps), len(self.radps)))
        self.best_score = np.unravel_index(np.argmin(score[3], axis=None), score[3].shape)

    def obstacle_scoring(self):
        self.scan_distance = self.scan_range[self.angle_160]
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
        
        data = np.mean(self.scan_distance, axis = 1)
        back = 0.55 > np.mean(data) >= 0.1
        side_left = np.logical_and(0.55 > data[0] >= 0.2, data[2] > data[1])
        side_left2 = np.logical_and( 0.55 > data[1] >= 0.2, data[2] > data[1])
        side_right = np.logical_and(0.55 > data[3] >= 0.2, data[2] < data[1])
        side_right2 = np.logical_and(0.55 > data[2] >= 0.2, data[2] < data[1])     
        front = np.logical_and(0.555 > np.mean(data[1:3]) >= 0.2, data[0] < data[3])    #turn left   
        front_1 = np.logical_and(0.555 > np.mean(data[1:3]) >= 0.2, data[0] > data[3])  #turn right          

        #if back == True:           
        #    rospy.loginfo("back")
        #    return -0.2, self.radps[self.best_score[1]]
        if side_left == True and side_right == True:
            rospy.loginfo("side back")
            return -0.2, 0.
        elif side_left == True or side_left2 == True or front_1 == True: # 우회전
            rospy.loginfo("side left")
            return self.mps[self.best_score[0]], -0.15
        elif side_right == True or side_right2 == True or front == True: # 좌회전
            rospy.loginfo("side right")
            return self.mps[self.best_score[0]], 0.15
        rospy.loginfo("straight")
        return self.mps[self.best_score[0]], self.radps[self.best_score[1]]


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
