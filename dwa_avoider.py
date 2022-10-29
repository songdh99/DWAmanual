#!/usr/bin/env python
# -- coding: utf-8 --

import numpy as np
import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Pose2D
from std_msgs.msg import String


def main():
    rospy.init_node('dwa_test')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    driver.calculation()
    while not rospy.is_shutdown():
        rospy.Subscriber('scan', LaserScan, driver.lds_callback)
        rospy.spin()


class SelfDrive:
    def __init__(self, publisher):
        rospy.Subscriber('current_pose', Pose, self.current_pose)
        self.publisher = publisher
        self.global_pose = Pose2D()
        self.lidar_array = np.array([])
        self.mps = [0.2, 0.17, 0.15, 0.13, 0.1]
        self.radps = [0, 0.3, 0.5, 0.6, 0.7, 0.8, 0.9, -0.3, -0.5, -0.6, -0.7, -0.8, -0.9]
        self.t_array = 0.25 * np.arange(1, 11).reshape(10, 1, 1)
        self.best_score = None
        self.radps_array = None
        self.motion = None
        self.local_pos = None
        self.start_x = self.global_pose.x
        self.start_y = self.global_pose.y
        self.goal_x = 2.07
        self.goal_y = -0.709
        self.save_x = -1.979
        self.save_y = 0.0
        self.scan_range = np.full((1,), 0)
        self.angle_160 = np.arange(-80, 80).reshape(160, 1, 1, 1)

    def current_pose(self, data):
        self.global_pose.x = data.position.x
        self.global_pose.y = data.position.y
        _, _, self.global_pose.theta = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])

    def lds_callback(self, scan):
        turtle_vel = Twist()
        self.preprocessing(scan)
        self.remaining_scoring()
        turtle_vel.linear.x, turtle_vel.angular.z = self.obstacle_scoring()
        if np.hypot(self.goal_x - self.global_pose.x, self.goal_y - self.global_pose.y) <= 0.20:
            rospy.loginfo_once("done")
            turtle_vel.linear.x, turtle_vel.angular.z = 0., 0.
        self.publisher.publish(turtle_vel)
    def scan_ranges(self, scan):
        self.data_array = np.asarray(scan.ranges)
        self.data_array[self.data_array == 0.0] = 4.5
        self.angle_160 = np.arange(-80, 80).reshape(160, 1, 1, 1)
        
    def preprocessing(self, scan):
        self.angle_160 = np.arange(-80, 80).reshape(4,40).T
        #print(self.angle_160)
        self.data_array = np.asarray(scan.ranges)
        self.data_array = self.data_array[self.angle_160]
        #self.data_array[self.data_array == 0.0] = None
        print(self.data_array)
        # self.data_array = self.data_array[
        #     np.logical_and(np.logical_or(self.data_array[:, 0] > -0.45, self.data_array[:, 0] < -0.47),
        #                    np.abs(self.data_array[:, 1]) > 0.17)]
       
        # self.data_array = self.data_array[np.logical_and(self.data_array[:, 2] > -0.55, self.data_array[:, 2] < 0.5)]
        

    def calculation(self):
        mps_array = np.array(self.mps).reshape(len(self.mps), 1)
        self.radps_array = np.delete(np.array(self.radps), 0)
        line_motion = mps_array * self.t_array
        rotational_motion = 2 * (mps_array / self.radps_array ) * np.sin(0.5 * self.radps_array * self.t_array) + 0.05
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
        #back = np.logical_and(np.logical_and(self.data_array[:] >= 0.3, self.data_array[:] < 0.55),
        #                       np.logical_and(self.data_array[:] > -0.44, self.data_array[:] < 0.43))
        back = np.logical_and(self.data_array[:]>=0.3, self.data_array[:] < 0.6)
        # slow = np.logical_and(np.logical_and(self.data_array[:, 0] >= 0.55, self.data_array[:, 0] < 0.8),
        #                       np.logical_and(self.data_array[:, 1] > -0.42, self.data_array[:, 1] < 0.41))
        # side_left = np.logical_and(
        #     np.logical_and(
        #         np.logical_and(self.data_array[:, 2] > 0.3, self.data_array[:, 2] < 0.6),
        #         np.logical_and(self.data_array[:, 3] >= 0., self.data_array[:, 3] < 0.68)),
        #     np.logical_and(self.data_array[:, 3] >= -0.55, self.data_array[:, 3] < 0.2))
        # side_right = np.logical_and(
        #     np.logical_and(
        #         np.logical_and(self.data_array[:, 0] > -0.45, self.data_array[:, 0] < 0.),
        #         np.logical_and(self.data_array[:, 1] > -0.69, self.data_array[:, 1] <= -0.4)),
        #     np.logical_and(self.data_array[:, 2] >= -0.55, self.data_array[:, 2] < 0.2))
        side_left = np.logical_and(self.data_array[:, 2] >= 0.2, self.data_array[:, 2] < 0.5)
        side_right = np.logical_and(self.data_array[:, 1]>=0.2, self.data_array[:, 1] < 0.5)
        turn_right = np.logical_and(
            np.logical_and(self.data_array[:, 3] >= 0.2, self.data_array[:, 3] < 0.5),
            np.logical_and(self.data_array[:, 2] >= 0.2, self.data_array[:, 2] < 0.5))
        turn_left = np.logical_and(
            np.logical_and(self.data_array[:, 0] >= 0.2, self.data_array[:, 0] < 0.5),
            np.logical_and(self.data_array[:, 1] >= 0.2, self.data_array[:, 1] < 0.5))
        # if True in back:
        #     rospy.loginfo("back")
        #     return -0.2, self.radps[self.best_score[1]]
        # elif True in slow:
        #     rospy.loginfo("slow")
        #     return 0.15, self.radps[self.best_score[1]]
        if True in turn_left and True in turn_right:
            rospy.loginfo("side back")
            return -0.2, 0.
        elif True in turn_left:
            rospy.loginfo("turn left")
            return 0., -1.5
        elif True in turn_right:
            rospy.loginfo("turn right")
            return 0., -1.5
        elif True in side_left: # 우회전
            rospy.loginfo("side left")
            return self.mps[self.best_score[0]], -0.3
        elif True in side_right: # 좌회전
            rospy.loginfo("side right")
            return self.mps[self.best_score[0]], 0.3
        rospy.loginfo("straight")
        return self.mps[self.best_score[0]], self.radps[self.best_score[1]]


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass