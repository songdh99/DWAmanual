#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from std_msgs.msg import Int32, Float32, Bool, String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan

sub_mode = "wait"
dwa_mode = "toward_goal"
pub_data = "none"
cur_pos_x = 0.0
cur_pos_y = 0.0
cur_ang_z = 0.0
goal_pos_x = 0.0
goal_pos_y = 0.0
goal_stop_x = 1.2225
goal_stop_y = -0.006
goal_ang_z = 0.0
safe_home_x = -0.3357
safe_home_y = -0.1816
safe_stop_x = goal_stop_x
safe_stop_y = goal_stop_y
check_aruco = True
count = 0

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
step_distance = np.concatenate((zero_dis_step, rad_dis_step), axis=2) + 0.3 # shape = (10, mps_c, rps_c)

angle_160 = np.arange(-80, 80).reshape(160, 1, 1, 1)
step_angle_160 = np.int32(np.rint(angle_160 + np.degrees(-1 * step * radps_ar / 2) + np.degrees(-1 * radps_ar)))

scan_range = np.full((1, ), 0)

class SelfDrive:
    def __init__(self, publisher):
        self.publisher = publisher
        self.stop_pub = rospy.Publisher('stop_point', String, queue_size=1)

    def lds_callback(self, scan):
        global dwa_mode
        global pub_data
        global scan_range
        global check_aruco
        global count
        turtle_vel = Twist()
        scan_range = np.array(scan.ranges)
        if sub_mode == "stop":
            turtle_vel.linear.x = 0.0
            turtle_vel.angular.z = 0.0
            pub_data = "none"
            rospy.loginfo("control_mode : {} DWA_mode : {} send_data : {}".format(sub_mode, dwa_mode, pub_data))
        elif sub_mode == "patrol" or sub_mode == "aruco" or sub_mode == "home":
            if dwa_mode == "toward_goal":
                rospy.loginfo_once("control_mode : {} DWA_mode : {} send_data : {}".format(sub_mode, dwa_mode, pub_data))
                mps_x, radps_z = Best_dis_score()
                print(mps_x, radps_z)
                turtle_vel.linear.x = mps_x
                turtle_vel.angular.z = radps_z
                distance = np.hypot(goal_stop_x - cur_pos_x, goal_stop_y - cur_pos_y)
                if -0.05 <= distance <= 0.05:
                    turtle_vel.linear.x = 0.0
                    turtle_vel.angular.z = 0.0
                    dwa_mode = "check_control"
                    rospy.loginfo("control_mode : {} DWA_mode : {} send_data : {}".format(sub_mode, dwa_mode, pub_data))
            
            elif dwa_mode == "check_control":
                if sub_mode == "patrol":
                    dwa_mode = "find_aruco"
                elif sub_mode == "aruco" or sub_mode == "home":
                    dwa_mode = "almost_angle"
                rospy.loginfo("control_mode : {} DWA_mode : {} send_data : {}".format(sub_mode, dwa_mode, pub_data))
            
            elif dwa_mode == "find_aruco":
                turtle_vel.linear.x = 0.0
                count += 1
                if count <= 180:
                    turtle_vel.angular.z = 0.20
                else:
                    count = 0
                    turtle_vel.angular.z = 0.0
                    pub_data = "stop"
                    dwa_mode = "toward_goal"
                    rospy.loginfo("control_mode : {} DWA_mode : {} send_data : {}".format(sub_mode, dwa_mode, pub_data))
            
            elif dwa_mode == "almost_angle":
                turtle_vel.linear.x = 0.0
                angle = np.rad2deg(goal_ang_z - cur_ang_z)
                if -2.5 <= angle <= 2.5:
                    turtle_vel.angular.z = 0.0
                    if dwa_mode == "aruco" and check_aruco:
                        check_aruco = False
                        pub_data = "check_aruco"
                    else:toward_goal
                    rospy.loginfo("control_mode : {} DWA_mode : {} send_data : {}".format(sub_mode, dwa_mode, pub_data))
                elif angle > 2.5:
                    turtle_vel.angular.z = -0.2
                elif angle < -2.5:
                    turtle_vel.angular.z = 0.2
            
            elif dwa_mode == "go_straight":
                turtle_vel.linear.x = 0.1
                turtle_vel.angular.z = 0.0
                distance = np.hypot(goal_pos_x - cur_pos_x, goal_pos_x - cur_pos_y)
                if distance <= 0.18:
                    turtle_vel.linear.x = 0.0
                    dwa_mode = "set_angle"
                    rospy.loginfo("control_mode : {} DWA_mode : {} send_data : {}".format(sub_mode, dwa_mode, pub_data))

            elif dwa_mode == "set_angle":
                turtle_vel.linear.x = 0.0
                angle = np.rad2deg(goal_ang_z - cur_ang_z)
                if -1.0 <= angle <= 1.0:
                    turtle_vel.angular.z = 0.0
                    pub_data = "stop"
                    dwa_mode = "back"
                    rospy.loginfo("control_mode : {} DWA_mode : {} send_data : {}".format(sub_mode, dwa_mode, pub_data))
                elif angle > 1.0:
                    turtle_vel.angular.z = 0.05
                elif angle < -1.0:
                    turtle_vel.angular.z = -0.05

            elif dwa_mode == "back":
                count += 1
                if count > 8:
                    count = 0
                    turtle_vel.linear.x = 0
                    turtle_vel.angular.z = 0
                    dwa_mode = "toward_goal"
                    rospy.loginfo("control_mode : {} DWA_mode : {} send_data : {}".format(sub_mode, dwa_mode, pub_data))
                elif count >= 5:
                    turtle_vel.linear.x = 0
                    turtle_vel.angular.z = -1.2
                else:
                    turtle_vel.linear.x = -0.2
                    turtle_vel.angular.z = 0
        self.publisher.publish(turtle_vel)
        self.stop_pub.publish(pub_data)

def Remaining_dis_score():
    x_move_coordinate = cur_pos_x + step_distance * (np.cos(step * radps_ar / 2) * np.cos(cur_ang_z) - np.sin(-1 * step * radps_ar / 2) * np.sin(cur_ang_z))
    y_move_coordinate = cur_pos_y + step_distance * (np.cos(step * radps_ar / 2) * np.sin(cur_ang_z) + np.sin(-1 * step * radps_ar / 2) * np.cos(cur_ang_z))
    remain_dis = np.hypot(goal_stop_x - x_move_coordinate, goal_stop_y - y_move_coordinate)
    return remain_dis[0]

def Pass_dis_score():
    dis_o15 = np.full((10, 1, rps_n), 0.)
    dps = np.degrees(step * radps_ar)
    dps = np.int32(np.rint(dps))

    for i in range(0, 10):
        for k in range(0, rps_n):
            dis_o15[i][0][k] = scan_range[dps[i][0][k]]
    scan_distance = dis_o15 * np.ones((mps_n, 1))
    check = scan_distance > step_distance

    pass_sec = np.int32(np.zeros((mps_n, rps_n)))
    for i in range(0, 10):
        pass_sec = np.where(check[i], i, pass_sec)
    pass_dis = pass_sec * mps_ar
    return pass_dis

def Obstacle_dis_score():
    scan_distance = scan_range[step_angle_160]
    theta = np.radians(angle_160)

    o2r_dis = np.hypot(step_distance * abs(np.sin(theta)), scan_distance - step_distance * np.cos(theta))
    o2r_dis_min = np.amin(o2r_dis, axis=0)

    obstacle_dis = o2r_dis_min[0]
    obstacle_dis = np.where(obstacle_dis > 0.40, 0.40, obstacle_dis)
    obstacle_dis = np.where(obstacle_dis < 0.12, -1.0, obstacle_dis)
    return obstacle_dis

def Best_dis_score():
    turtle_l_x = 0.0
    turtle_a_z = -0.1
    remaining_score = Remaining_dis_score()
    pass_score = Pass_dis_score()
    obstacle_score = Obstacle_dis_score()

    if np.max(obstacle_score) == -1.0:
        return turtle_l_x, turtle_a_z

    priority_score = pass_score * obstacle_score

    plus_rad_array = np.concatenate((np.array([[0],[0]]) ,priority_score[:, 1:7]), axis = 1)
    plus_rad_array = np.concatenate((plus_rad_array, np.zeros((2, 6))), axis = 1)
    minus_rad_array = np.concatenate((np.zeros((2, 7)), priority_score[:, 7:]), axis = 1)
    zero_rad_array = np.concatenate((priority_score[:, :1], np.zeros((2, 12))), axis = 1)

    plus_row_col = np.unravel_index(np.argmax(plus_rad_array, axis=None), priority_score.shape) 
    minus_row_col = np.unravel_index(np.argmax(minus_rad_array, axis=None), priority_score.shape)
    zero_row_col = np.unravel_index(np.argmax(zero_rad_array, axis=None), priority_score.shape)

    plus_value = np.where(priority_score[plus_row_col[0]][plus_row_col[1]] < 0, 10, 0)
    minus_value = np.where(priority_score[minus_row_col[0]][minus_row_col[1]] < 0, 10, 0)
    zero_value = np.where(priority_score[zero_row_col[0]][zero_row_col[1]] < 0, 10, 0)

    three_pri_row_col = np.array([plus_row_col, minus_row_col, zero_row_col])
    three_pri_score = np.array([plus_value + remaining_score[plus_row_col[0]][plus_row_col[1]], minus_value + remaining_score[minus_row_col[0]][minus_row_col[1]], zero_value + remaining_score[zero_row_col[0]][zero_row_col[1]]])
    best_score_index = np.argmax(three_pri_score, axis=None)
    mps_index, radps_index = three_pri_row_col[best_score_index]

    print(three_pri_score)
    turtle_l_x = mps[mps_index]
    turtle_a_z = radps[radps_index]
    
    return turtle_l_x, turtle_a_z

def DWA_mode_callback(mode):
    global sub_mode
    global goal_pos_x
    global goal_pos_y
    global goal_stop_x
    global goal_stop_y
    global goal_ang_z
    sub_mode = mode.data
    if sub_mode == "patrol":
        goal_stop_x = safe_stop_x
        goal_stop_y = safe_stop_y
    elif sub_mode == "home":
        goal_pos_x = safe_home_x
        goal_pos_y = safe_home_y
        goal_ang_z = 0.0

def cur_pos_callback(pos):
    global cur_pos_x
    global cur_pos_y
    cur_pos_x = pos.position.x
    cur_pos_y = pos.position.y

def cur_ang_callback(angle):
    global cur_ang_z
    cur_ang_z = angle.position.z

def goal_pos_callback(pos):
    global goal_pos_x
    global goal_pos_y
    goal_pos_x = pos.position.x
    goal_pos_y = pos.position.y
    rospy.loginfo("pos.x : {} pos.y : {}".format(goal_pos_x, goal_pos_y))

def goal_ang_callback(angle):
    global goal_stop_x
    global goal_stop_y
    global goal_ang_z
    goal_ang_z = angle.data + np.pi
    if goal_ang_z > 2 * np.pi:
        goal_ang_z = goal_ang_z - 2 * np.pi
    goal_stop_x = goal_pos_x - 0.5 * np.cos(goal_ang_z)
    goal_stop_y = goal_pos_y - 0.5 * np.sin(goal_ang_z)
    rospy.loginfo("stop_x : {} stop_y : {} ang.z : {}".format(goal_stop_x, goal_stop_y, np.rad2deg(goal_ang_z)))

def main():
    rospy.init_node('DWA_made_by_Choi')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    rospy.Subscriber('scan', LaserScan, lambda scan: driver.lds_callback(scan))
    rospy.Subscriber('DWA_pub', String, DWA_mode_callback)
    rospy.Subscriber('current_xyz', Pose, cur_pos_callback)
    rospy.Subscriber('current_angle', Pose, cur_ang_callback)
    rospy.Subscriber('a_about_r_pos', Pose, goal_pos_callback)
    rospy.Subscriber('a_about_r_ang', Pose, goal_ang_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
