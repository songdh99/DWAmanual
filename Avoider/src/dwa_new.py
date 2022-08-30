#!/home/scout/.pyenv/versions/spkm_py2718/bin/python
# -- coding: utf-8 --

import numpy as np
import rospy
import tf
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, Pose, Pose2D


def main():
    rospy.init_node('dwa_test')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = DWA(publisher)
    rospy.Subscriber('scan', LaserScan, lambda scan: driver.lds_callback(scan))
    rospy.spin()


class DWA:
    def __init__(self, publisher):
        rospy.Subscriber('current_pose', Pose, self.current_pose)
        self.publisher = publisher
        self.scout_pose = Pose2D()
        self.dwa_mode = "patrol"
        self.home_pos = 8.14, 2.14
        self.goal_pos = 0.23, 0.11

    def lds_callback(self, scan):
        turtle_vel = Twist()
        mps = [0.1, 0.12, 0.14, 0.16, 0.18, 0.2, 0.22, 0.24, 0.26, 0.28]
        radps = [0, 0.2, 0.4, 0.6, 0.8, -0.2, -0.4, -0.6, -0.8, -0.9]
        scan_range = np.array(scan.ranges)
        data_array = self.preprocessing(scan)
        local_pos = self.make_combination(mps, radps)
        pos_candidates = self.create_pos_candidates(local_pos)
        if self.dwa_mode == "patrol":
            goal_pos = self.goal_pos
        elif self.dwa_mode == "home":
            goal_pos = self.home_pos
        if np.hypot(goal_pos[0] - self.scout_pose.x, goal_pos[1] - self.scout_pose.y) <= 0.6:
            turtle_vel.linear.x, turtle_vel.angular.z = 0., 0.
        else:
            best_score, back_check = self.evaluate_scores(pos_candidates, goal_pos, data_array, local_pos)
            turtle_vel.linear.x, turtle_vel.angular.z = mps[best_score[0]], radps[best_score[1]]
            if back_check:
                turtle_vel.linear.x = mps[best_score[0]] * -1.2
        self.publisher.publish(turtle_vel)

    def current_pose(self, data):
        self.scout_pose.x = data.position.x
        self.scout_pose.y = data.position.y
        _, _, self.scout_pose.theta = tf.transformations.euler_from_quaternion(
            [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])

    @staticmethod
    def preprocessing(data):
        data_array = np.asarray(data.ranges)
        data_array = data_array[np.logical_and(np.logical_or(data_array[:, 0] > -0.45, data_array[:, 0] < -0.47),
                                               np.abs(data_array[:, 1]) > 0.17)]
        data_array = data_array[np.logical_and(data_array[:, 2] > -0.55, data_array[:, 2] < 0.5)]
        return data_array

    @staticmethod
    def make_combination(mps, radps):
        line_motions = np.array(mps).reshape(len(mps), 1)
        radps_array = np.delete(np.array(radps), 0)
        rotational_motions = 2 * (line_motions / radps_array) * np.sin(0.5 * radps_array) + 0.05
        local_x_pos = np.concatenate((line_motions, rotational_motions * np.cos(0.5 * radps_array)), axis=1)
        local_y_pos = np.concatenate(
            (np.zeros((len(mps), 1)), rotational_motions * np.sin(0.5 * radps_array)), axis=1)
        local_pos = np.concatenate((np.reshape(local_x_pos, (-1, 1)), np.reshape(local_y_pos, (-1, 1))), axis=1)
        return local_pos

    def create_pos_candidates(self, local_pos):
        rotation_matrix = np.array(
            [[np.cos(self.scout_pose.theta), -np.sin(self.scout_pose.theta)],
             [np.sin(self.scout_pose.theta), np.cos(self.scout_pose.theta)]])
        rotation_trans = np.round_((np.dot(local_pos, rotation_matrix.T)), 4)
        global_x_pos = self.scout_pose.x + np.delete(rotation_trans, 1, axis=1)
        global_y_pos = self.scout_pose.y + np.delete(rotation_trans, 0, axis=1)
        global_pos = np.concatenate((global_x_pos, global_y_pos), axis=1)
        return global_pos

    def evaluate_scores(self, pos_candidates, goal_pos, data_array, local_pos):
        remaining_scores = self.find_remaining_scores(pos_candidates, goal_pos)
        obstacle_scores = self.find_obstacle_scores(data_array, local_pos)
        clearance_scores = self.find_clearance_scores(data_array, local_pos)
        scores = 0.6 * remaining_scores - 0.27 * obstacle_scores - 0.13 * clearance_scores
        best_score = np.unravel_index(np.argmin(scores), scores.shape)
        return best_score, obstacle_scores[best_score] <= 0.4

    @staticmethod
    def find_remaining_scores(pos_candidates, goal_pos):
        x = goal_pos[0] - np.delete(pos_candidates, 1, axis=1)
        y = goal_pos[1] - np.delete(pos_candidates, 0, axis=1)
        scores = np.reshape(np.hypot(x, y), (10, 10))
        return scores

    @staticmethod
    def find_obstacle_scores(data_array, local_pos):
        x = data_array[:, 0] - np.delete(local_pos, 1, axis=1)
        y = data_array[:, 1] - np.delete(local_pos, 0, axis=1)
        scores = np.reshape(np.amin(np.hypot(x, y), axis=1), (10, 10))
        return scores

    @staticmethod
    def find_clearance_scores(data_array, local_pos):
        x = data_array[:, 0] - np.delete(local_pos, 1, axis=1)
        y = data_array[:, 1] - np.delete(local_pos, 0, axis=1)
        scores = np.reshape(np.amax(np.hypot(x, y), axis=1), (10, 10))
        # norm = np.linalg.norm(scores)
        # scores = scores / norm
        return scores


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
