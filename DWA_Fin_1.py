#!/usr/bin/env python
# -- coding: utf-8 --

from calendar import c
from turtle import back
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
    #rospy.Subscriber('dwa_p', Pose, driver.goal_pose)
    rospy.Subscriber('dwa_m', String, driver.mode)
    rospy.Subscriber('scan', LaserScan, driver.lds_callback)
    rospy.spin()


class DWA:
    def __init__(self, publisher):
        self.waffle_pose = Pose2D()
        #self.goal = Pose()
        rospy.Subscriber('current_xyz', Pose, self.current_pose)
        self.pub_control = rospy.Publisher("DWA", String, queue_size=1)
        self.publisher = publisher
        self.dwa_mode = "patrol"
        self.goal_pos = [2.46, -0.0505]
        #self.home_pos = [0.961, -0.634]
        self.home_pos = [self.waffle_pose.x, self.waffle_pose.y]
        self.rate = rospy.Rate(1)

    def lds_callback(self, scan):
        turtle_vel = Twist()
        scan_range = np.array(scan.ranges)
        scan_range[scan_range == 0.0] = 4.5
        mps = [0.10, 0.11, 0.12, 0.13, 0.15, 0.16, 0.17, 0.18, 0.19, 0.20]
        radps = [ 0, 0.2, 0.4, 0.6, 0.8, -0.2, -0.4, -0.6,-0.8, -1.5]
        local_pos = self.make_combination(mps, radps)
        self.pos_candidates = self.create_pos_candidates(local_pos)
        if self.dwa_mode == "patrol":
            goal_pos = self.goal_pos
        elif self.dwa_mode == "home":
            goal_pos = self.home_pos
        # elif self.dwa_mode == "patrol":
        #     _, _, obstackle_score = self.evaluate_scores(self.pos_candidates, goal_pos, scan_range, local_pos)
        #     best_ob = np.argmax(obstackle_score)
        #     best_ob_x = best_ob//10
        #     best_ob_y = best_ob%10
        #     turtle_vel.linear.x, turtle_vel.angular.z = mps[best_ob_x], radps[best_ob_y]
        else:
            return
        if np.hypot(goal_pos[0] - self.waffle_pose.x, goal_pos[1] - self.waffle_pose.y) <= 0.10:
            rospy.loginfo_once('fin')
            turtle_vel.linear.x, turtle_vel.angular.z = 0., 0.
            self.pub_control.publish("dwa_success")
        else:
            best_score, back_check, obstackle_score = self.evaluate_scores(self.pos_candidates, goal_pos, scan_range, local_pos)
            turtle_vel.linear.x, turtle_vel.angular.z = mps[best_score[0]], radps[best_score[1]]
            rospy.loginfo('{}'.format(self.waffle_pose))
            rospy.loginfo('x:{} z:{}'.format(turtle_vel.linear.x, turtle_vel.angular.z))
            print(back_check)
            print(obstackle_score[best_score])
            # if back_check == False:
            #     turtle_vel.linear.x = -0.07
            #     turtle_vel.angular.z = radps[best_score[1]]
            #     rospy.loginfo('x:{} z:{}'.format(turtle_vel.linear.x, turtle_vel.angular.z))

        self.publisher.publish(turtle_vel)

    def current_pose(self, data):
        self.waffle_pose.x = data.position.x
        self.waffle_pose.y = data.position.y
        _, _, self.waffle_pose.theta = tf.transformations.euler_from_quaternion(
            [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])

    # def goal_pose(self, data): ################################################################
    #     self.goal_pos[0] = data.position.x
    #     self.goal_pos[1] = data.position.y
    #     print(self.goal.position.x)
    #     print(self.goal.position.y)
    
    @staticmethod
    def make_combination(mps, radps):               
        line_motions = np.array(mps).reshape(len(mps), 1)
        radps_array = np.delete(np.array(radps), 0)
        rotational_motions = 2 * (line_motions / radps_array) * np.sin(0.5 * radps_array)
        local_x_pos = np.concatenate((line_motions, rotational_motions * np.cos(0.5 * radps_array)), axis=1)
        local_y_pos = np.concatenate(
            (np.zeros((len(mps), 1)), rotational_motions * np.sin(0.5 * radps_array)), axis=1)
        local_pos = np.concatenate((np.reshape(local_x_pos, (-1, 1)), np.reshape(local_y_pos, (-1, 1))), axis=1)
        return local_pos

    def create_pos_candidates(self, local_pos):
        rotation_matrix = np.array(
            [[np.cos(self.waffle_pose.theta), -np.sin(self.waffle_pose.theta)],
             [np.sin(self.waffle_pose.theta), np.cos(self.waffle_pose.theta)]])
        rotation_trans = np.round_((np.dot(local_pos, rotation_matrix.T)), 4)
        global_x_pos = self.waffle_pose.x + np.delete(rotation_trans, 1, axis=1)
        global_y_pos = self.waffle_pose.y + np.delete(rotation_trans, 0, axis=1)
        global_pos = np.concatenate((global_x_pos, global_y_pos), axis=1)
        return global_pos

        
    def evaluate_scores(self, pos_candidates, goal_pos, data_array, local_pos):         
        remaining_scores = self.find_remaining_scores(pos_candidates, goal_pos)
        obstacle_scores = self.find_others_scores(data_array, local_pos)
        scores = 1.7 * remaining_scores - 1.1* obstacle_scores
        best_score = np.unravel_index(np.argmin(scores), scores.shape)
        return best_score, 0.01 < obstacle_scores[best_score] <= 0.4, obstacle_scores

    @staticmethod
    def find_remaining_scores(pos_candidates, goal_pos):           
        x = goal_pos[0] - np.delete(pos_candidates, 1, axis=1)
        y = goal_pos[1] - np.delete(pos_candidates, 0, axis=1)
        scores = np.reshape(np.hypot(x, y), (10, 10))
        norm = np.linalg.norm(scores)
        scores = scores / norm  
        return scores

    @staticmethod
    def find_others_scores(data_array, local_pos):                
        x = np.delete(local_pos, 1, axis=1)
        y = np.delete(local_pos, 0, axis=1)
        theta = np.int32(np.degrees(np.arctan(y/x)))
        scores = np.reshape(data_array[theta], (10, 10))
        norm = np.linalg.norm(scores)
        scores = scores / norm
        return scores
    
    def mode(self, control_order):
        self.pub_control.publish("dwa_working")
        if control_order.data == "patrol":
            self.dwa_mode = "patrol"
            
        elif control_order.data == "open_door":
            pass
        elif control_order.data == "go_inside":
            pass
        elif control_order.data == "home":
            pass
        self.pub_control.publish("dwa_success")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        