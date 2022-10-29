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
    driver = DWA(publisher)
    rospy.Subscriber('scan', LaserScan, driver.lds_callback)
    rospy.spin()


class DWA:
    def __init__(self, publisher):
        rospy.Subscriber('current_xyz', Pose, self.current_pose)
        self.publisher = publisher  
        self.waffle_pose = Pose2D()
        self.t_array = 0.25 * np.arange(1, 11).reshape(10, 1, 1)
        self.goal_pos = [0.973, 0.756]
        self.home_pos = [self.waffle_pose.x, self.waffle_pose.y]
        #self.home_pos = [0.977, 0.654]
        self.save_pos = self.home_pos
        self.dwa_mode = "patrol"
        self.scan_range = np.full((1,), 0)
        self.angle_160 = np.arange(-80, 80).reshape(160, 1, 1, 1)

    def current_pose(self, data):
        self.waffle_pose.x = data.position.x
        self.waffle_pose.y = data.position.y
        _, _, self.waffle_pose.theta = tf.transformations.euler_from_quaternion(
            [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])

    def lds_callback(self, scan):
        turtle_vel = Twist()
        self.preprocessing(scan)
        mps = [0.2, 0.17, 0.15, 0.13, 0.1]
        radps = [0, 0.3, 0.5, 0.6, 0.7, 0.8, 0.9, -0.3, -0.5, -0.6, -0.7, -0.8, -0.9]
        local_pos = self.calculation(mps, radps)
        best_score = self.remaining_scoring(mps, radps, local_pos)
        if self.dwa_mode == "patrol":
            goal_pos = self.goal_pos
        elif self.dwa_mode == "home":
            goal_pos = self.save_pos
        if np.hypot(goal_pos[0] - self.waffle_pose.x, goal_pos[1] - self.waffle_pose.y) <= 0.10:
            rospy.loginfo_once("fin")
            turtle_vel.linear.x, turtle_vel.angular.z = 0., 0.
        else:
            turtle_vel.linear.x, turtle_vel.angular.z = self.obstacle_scoring(mps, radps, best_score)
            print(self.save_pos)
            rospy.loginfo('{}'.format(self.waffle_pose))
            rospy.loginfo('속도:{} 각속도:{}'.format(turtle_vel.linear.x, turtle_vel.angular.z))
        self.publisher.publish(turtle_vel)
        
    def preprocessing(self, scan):
        self.angle_160 = np.arange(-80, 80).reshape(4,40).T
        self.data_array = np.asarray(scan.ranges)
        self.data_array = self.data_array[self.angle_160]
        #print(self.data_array)
        
    def calculation(self, mps, radps):
        mps_array = np.array(mps).reshape(len(mps), 1)
        radps_array = np.delete(np.array(radps), 0)
        line_motion = mps_array * self.t_array
        rotational_motion = 2 * (mps_array / radps_array ) * np.sin(0.5 * radps_array * self.t_array) + 0.05
        self.motion = np.concatenate((line_motion, rotational_motion), axis=2) + 0.4
        local_x_pos = np.concatenate((line_motion, rotational_motion * np.cos(0.5 * radps_array * self.t_array)), axis=2)
        local_y_pos = np.concatenate((np.zeros((10, len(mps), 1)), rotational_motion * np.sin(0.5 * radps_array * self.t_array)), axis=2)
        local_pos = np.concatenate((np.reshape((local_x_pos), (10, -1, 1)), np.reshape((local_y_pos), (10, -1, 1))), axis=2)
        return local_pos

    def remaining_scoring(self,mps,radps, local_pos):
        rotation_matrix = np.array(
            [[np.cos(self.waffle_pose.theta), -np.sin(self.waffle_pose.theta)],
             [np.sin(self.waffle_pose.theta), np.cos(self.waffle_pose.theta)]])
        local_matrix = np.round_((np.dot(local_pos, rotation_matrix.T)), 4)
        remain_x = self.goal_pos[0] - (self.waffle_pose.x + np.delete(local_matrix, 1, axis=2))
        remain_y = self.goal_pos[1] - (self.waffle_pose.y + np.delete(local_matrix, 0, axis=2))
        score = np.reshape(np.hypot(remain_x, remain_y), (10, len(mps), len(radps)))
        best_score = np.unravel_index(np.argmin(score[3], axis=None), score[3].shape)
        return best_score

    def obstacle_scoring(self, mps, radps, best_score):
        side_left = np.logical_and(self.data_array[:, 2] >= 0.2, self.data_array[:, 2] < 0.6)
        side_right = np.logical_and(self.data_array[:, 1]>=0.2, self.data_array[:, 1] < 0.6)
        turn_right = np.logical_and(
            np.logical_and(self.data_array[:, 3] >= 0.1, self.data_array[:, 3] < 0.5),
            np.logical_and(self.data_array[:, 2] >= 0.1, self.data_array[:, 2] < 0.5))
        turn_left = np.logical_and(
            np.logical_and(self.data_array[:, 0] >= 0.1, self.data_array[:, 0] < 0.5),
            np.logical_and(self.data_array[:, 1] >= 0.1, self.data_array[:, 1] < 0.5))
        if True in turn_left and True in turn_right:
            rospy.loginfo("side back")
            return -0.2, 0.
        elif True in turn_left:
            rospy.loginfo("turn left")
            return 0., -1.3
        elif True in turn_right:
            rospy.loginfo("turn right")
            return 0., -1.3
        elif True in side_left: # 우회전
            rospy.loginfo("side left")
            return mps[best_score[0]], -0.3
        elif True in side_right: # 좌회전
            rospy.loginfo("side right")
            return mps[best_score[0]], 0.3
        rospy.loginfo("straight")
        return mps[best_score[0]], radps[best_score[1]]


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
