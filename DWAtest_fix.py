#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np
import rospy
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import Twist, Pose, Quaternion, Point, PoseStamped
from sensor_msgs.msg import LaserScan

# 속도, 각속도의 개수
mps_c = 2
rps_c = 13

Mps = [0.15, 0.13]
Radps = [0, 0.3, 0.5, 0.6, 0.7, 0.8, 0.9, -0.3, -0.5, -0.6, -0.7, -0.8, -0.9]  # 첫 원소는 무조건 0을 넣어야 함 (계산식이 다르기 때문)

five_Radps_scandistance = np.full((10, 1, rps_c), 0.)  # 10스텝까지의 rps_c개의 각속도에 따른 각도마다 스캔값 저장
# 속도, 각속도에 따라 도달하는 직선거리값을 step 마다 계산
# 한번 계산하고 계속 사용하기 위해 함수 밖에 작성
MpsAr = np.array(Mps).reshape(mps_c, 1)
RadpsAr = np.delete(np.array(Radps), 0)  # 각속도가 0일땐 거리계산식이 달라지므로 제외 후 따로 계산
step = 0.15 * np.arange(1, 11).reshape(10, 1, 1)
zeroRadpsAr = MpsAr * step  # 각속도가 0일때 (10, mps_c, 1)
distancestep = (2 * np.sin(RadpsAr * step / 2) / RadpsAr * MpsAr)  # (10, mps_c, rps_c-1)
fulldistancesteps = np.concatenate((zeroRadpsAr, distancestep), axis=2) + 0.3  # (10, mps_c, rps_c) 로봇의 크기보정을 위해 + 0.2

angle160 = np.arange(-80, 80).reshape(160, 1, 1, 1)
dg_angle160_Radps_step = np.int32(
    np.rint(angle160 + np.degrees(step * np.array(Radps) / 2)))  # (160, 10, 1, rps_c) 반올림 후 정수형으로 변환

# (Local)로봇 기준 이동시 x, y 이동거리 (10, mps_c, rps_c)
x_move_distance = np.concatenate((zeroRadpsAr, (distancestep * np.cos(90 - (180 - step * RadpsAr) / 2))), axis=2)
y_move_distance = np.concatenate((np.zeros((10, mps_c, 1)), (distancestep * np.sin(90 - (180 - step * RadpsAr) / 2))),
                                 axis=2)
# 좌표계 변환을 위해 x,y의 9스텝 까지의 이동거리를 합쳐서 (rps_c, 2)로 만듦
xy_move_distance = np.concatenate(
    (np.reshape((x_move_distance), (10, -1, 1)), np.reshape((y_move_distance), (10, -1, 1))), axis=2)

SCAN_ran = np.full((1, 360), 0)  # 360도 측정 거리값 초기화
current_xyz = Pose()
current_angle = Pose()
stop_point = String()
wherestop = String()
goal_location_x = 2.1064
goal_location_y = -0.01688
g_s_x = 1.6433
g_s_y = -0.00125
aruco_location_x = 2.1064
aruco_location_y = -0.01688
a_s_x = 1.6433
a_s_y = -0.00125
start_location_x = -0.5186
start_location_y = 0.01264
s_s_x = -0.2222
s_s_y = 0.01654
first_location_x = 2.1064
first_location_y = -0.01688
f_s_x = 1.6433
f_s_y = -0.00125
goal_radian = 0.
g_s_radian = 0.
retry = 0
r_g_score = np.full((mps_c, rps_c), 0.)
pass_distance = np.full((mps_c, rps_c), 0.)
near_dis_score = np.full((mps_c, rps_c), 0.)
t = 0
o = 0
n = 0
p = 0
catch_ar = 0
angle = 0
b = 0
bt = 0
DWA_mode = String()
R_G_dis = 0.
R_GS_dis = 0.

fail_first_angle = 0
count_ag = 0
pre_add = 0



class SelfDrive:

    def __init__(self, publisher):
        self.publisher = publisher
        self.stop_point = rospy.Publisher('stop_point', String, queue_size=1)
        rospy.Subscriber('DWA_pub', String, self.check_mode)
        rospy.Subscriber('current_xyz', Pose, self.current_xyz)
        rospy.Subscriber('current_angle', Pose, self.current_angle)
        rospy.Subscriber('a_about_r_pos', Pose, self.a_about_r_pos)

    def current_angle(self, angle):
        global current_angle
        current_angle.position.z = angle.position.z * 180 / math.pi  # angle.position.z가 360도가 1로 나타남
        if current_angle.position.z < 0:
            current_angle.position.z += 360

    def current_xyz(self, xyz):
        global current_xyz
        global retry
        global start_location_x
        global start_location_y
        global wherestop
        current_xyz.position.x = xyz.position.x
        current_xyz.position.y = xyz.position.y
        retry += 1
        if retry == 1:
            wherestop = "goal point"

    def check_mode(self, DWA_pub):
        global o
        global n
        global DWA_mode
        global wherestop
        global goal_location_x
        global goal_location_y
        global g_s_x
        global g_s_y

        DWA_mode = DWA_pub.data  ###########고쳐야됨
        if DWA_mode == "home" and o == 0:
            wherestop = "back"
            stop_point.data = "wait"
            o = 1
            goal_location_x = start_location_x
            goal_location_y = start_location_y
            g_s_x = s_s_x
            g_s_y = s_s_y
        if DWA_mode == "patrol" and o == 1:
            wherestop = "back"
            o = 0
            goal_location_x = first_location_x
            goal_location_y = first_location_y
            g_s_x = f_s_x
            g_s_y = f_s_y
    def a_about_r_pos(self, xyz):
        global goal_location_x
        global goal_location_y
        global g_s_x
        global g_s_y
        global aruco_location_x
        global aruco_location_y
        global a_s_x
        global a_s_y
        global catch_ar
        goal_location_x = xyz.position.x
        goal_location_y = xyz.position.y
        g_s_x = xyz.position.x - 0.5
        g_s_y = xyz.position.y
        aruco_location_y = xyz.position.x
        aruco_location_y = xyz.position.y
        a_s_x = xyz.position.x - 0.5
        a_s_y = xyz.position.y
        catch_ar = 1
        print("{}, {}, {}".format(xyz.position.x, xyz.position.y, xyz.orientation.z))
        


    def lds_callback(self, scan):
        global goal_location_x
        global goal_location_y
        global goal_radian
        global g_s_radian
        global DWA_mode
        global r_g_score
        global stop_point
        global wherestop
        global SCAN_ran
        global t
        global o
        global n
        global p
        global catch_ar
        global angle
        global b
        global bt
        global R_G_dis
        global R_GS_dis
        global near_dis_score
        global pass_distance
        global fail_first_angle
        global count_ag
        global pre_add

        def r_g_scoring():
            global goal_radian
            global g_s_radian
            global wherestop
            global R_G_dis
            global R_GS_dis
            global r_g_score

            x = goal_location_x - current_xyz.position.x
            y = goal_location_y - current_xyz.position.y
            goal_radian = math.atan2(y, x) * 180 / math.pi
            if goal_radian < 0:
                goal_radian += 360
            R_G_dis = np.hypot(goal_location_x - current_xyz.position.x, goal_location_y - current_xyz.position.y)

            gs_x = g_s_x - current_xyz.position.x
            gs_y = g_s_y - current_xyz.position.y
            g_s_radian = math.atan2(gs_y, gs_x) * 180 / math.pi
            if g_s_radian < 0:
                g_s_radian += 360
            R_GS_dis = np.hypot(g_s_x - current_xyz.position.x, g_s_y - current_xyz.position.y)

            # 목표와 로봇사이 거리 스코어
            Rot = np.array(
                [[math.cos(current_angle.position.z * math.pi / 180),
                  -math.sin(current_angle.position.z * math.pi / 180)],
                 [math.sin(current_angle.position.z * math.pi / 180),
                  math.cos(current_angle.position.z * math.pi / 180)]])
            path_len = np.round_((np.dot(xy_move_distance, Rot)), 4)  # 글로벌에서 본 경로거리를 구해서 4째자리까지 반올림
            r_g_path_len_x = goal_location_x - (current_xyz.position.x + np.delete(path_len, 1, axis=2))
            r_g_path_len_y = goal_location_y - (current_xyz.position.y + np.delete(path_len, 0, axis=2))
            r_g_path_dis = np.reshape(np.hypot(r_g_path_len_x, r_g_path_len_y), (10, mps_c, rps_c))
            r_g_score = r_g_path_dis[3]  # (mps_c, rps_c), sqrt(x**2 + y**2)

        def near_dis_scoring():
            global near_dis_score
            global pass_distance
            dfors = np.degrees(step * np.array(Radps))  # degree for scan(10, 1, rps_c)
            dfors = np.int32(np.rint(dfors))  # 반올림 후 int형으로 변경
            # <five_Radps_scandistance>
            for i in range(0, 10):
                for k in range(0, rps_c):
                    five_Radps_scandistance[i][0][k] = SCAN_ran[dfors[i][0][k]]
            t_f_f = five_Radps_scandistance * np.ones((mps_c, 1))
            true_false = t_f_f > fulldistancesteps  # (10, mps_c, rps_c) 계산값이 측정거리보다 낮아 부딪히지 않는다면 True

            # <passsec> (mps_c, rps_c) 해당 가닥이 몇초동안 장애물에 부딪히지 않는지 계산
            passsec = np.int32(np.zeros((mps_c, rps_c)))
            for i in range(0, 10):
                passsec = np.where(true_false[i], i, passsec)  # 부딪히기 바로 전 step을 저장

            # <pass_distance> (mps_c, rps_c) 부딪히지 않고 이동하는 거리
            pass_distance = passsec * np.array(Mps).reshape(mps_c, 1)
            # (160, 10, 1, rps_c) 각도를 스캔한 거리값으로 변경
            a_R_s_scandistance = np.where(True, SCAN_ran[dg_angle160_Radps_step], SCAN_ran[dg_angle160_Radps_step])

            # (160, 10, mps_c, rps_c)
            neardis160 = np.sqrt((a_R_s_scandistance * np.sin(np.radians(dg_angle160_Radps_step))) ** 2 + (
                    fulldistancesteps - a_R_s_scandistance * abs(np.cos(np.radians(dg_angle160_Radps_step)))) ** 2)
            # (10, mps_c, rps_c)
            near_dis_min = np.amin(neardis160, axis=0)

            for i in range(0, mps_c):
                for j in range(0, rps_c):
                    k = (passsec[i][j] - 2) % 1
                    near_dis_score[i][j] = near_dis_min[k][i][j]  # (mps_c, rps_c)
            # (mps_c, rps_c)
            near_dis_score = np.where(near_dis_score > 0.30, 0.30, near_dis_score)  # 30cm가 넘는 것은 30cm로 만듦 ##
            near_dis_score = np.where(near_dis_score < 0.12, -100, near_dis_score)  # 10cm 보다 낮은 것은 -1로 만듦

        turtle_vel = Twist()
        turn = False
        SCAN_ran = np.array(scan.ranges)

        r_g_scoring()
        near_dis_scoring()

        scoremap = near_dis_score * pass_distance ##

        plus_array = np.concatenate((np.array([[0],[0]]) ,scoremap[:, 1:7]), axis = 1)
        plus_array = np.concatenate((plus_array, np.zeros((2, 6))), axis = 1)
        minus_array = np.concatenate((np.zeros((2, 7)), scoremap[:, 7:]), axis = 1)
        advance_array = np.concatenate((scoremap[:, :1], np.zeros((2, 12))), axis = 1)

        plus_row_col = np.unravel_index(np.argmax(plus_array, axis=None), scoremap.shape) 
        minus_row_col = np.unravel_index(np.argmax(minus_array, axis=None), scoremap.shape)
        advance_row_col = np.unravel_index(np.argmax(advance_array, axis=None), scoremap.shape)
        p_m_a = np.array([plus_row_col, minus_row_col, advance_row_col])

        plus = np.where(scoremap[plus_row_col[0]][plus_row_col[1]] < 0, -100, 0)
        minus = np.where(scoremap[minus_row_col[0]][minus_row_col[1]] < 0, -100, 0)
        advance = np.where(scoremap[advance_row_col[0]][advance_row_col[1]] < 0, -100, 0)

        three_matrix = np.array([plus + r_g_score[plus_row_col[0]][plus_row_col[1]], minus + r_g_score[minus_row_col[0]][minus_row_col[1]], advance + r_g_score[advance_row_col[0]][advance_row_col[1]]])
        max_three = np.argmin(three_matrix, axis = None)

        score_row_col = np.unravel_index(np.argmax(scoremap, axis=None), scoremap.shape) 
        turtle_vel.linear.x = Mps[score_row_col[0]] #Mps[score_row_col[0]]
        turtle_vel.angular.z = Radps[score_row_col[1]]#Radps[score_row_col[1]]
        
        # 만약 모든 범위가 10cm 보다 낮다면 turn
        if np.max(near_dis_score) == -100:
            turn = True
        if turn:
            turtle_vel.linear.x = 0
            turtle_vel.angular.z = -1.0
        # if turn == False:
        #    t = 0
        # if turn and t == 0:
        #     if 0 < goal_radian - current_angle.position.z:
        #         t = 1
        #     if 0 > goal_radian - current_angle.position.z:
        #         t = -1
        # if turn and t == 1:
        #     turtle_vel.linear.x = 0
        #     turtle_vel.angular.z = 1.0
        # if turn and t == -1:
        #     turtle_vel.linear.x = 0
        #     turtle_vel.angular.z = -1.0
        gs_m_c = g_s_radian - current_angle.position.z
        if gs_m_c < 0:
            gs_m_c += 360
        
        if R_GS_dis < 0.92 and (wherestop == "goal point" or wherestop == "starting point"):
            if 356 < gs_m_c or gs_m_c < 4:
                turtle_vel.linear.x = 0.12
                turtle_vel.angular.z = 0
            else:
                if 180 < gs_m_c:
                    turtle_vel.linear.x = 0.12
                    turtle_vel.angular.z = -0.5
                if 180 > gs_m_c:
                    turtle_vel.linear.x = 0.12
                    turtle_vel.angular.z = 0.5
                

        if R_GS_dis < 0.05 and wherestop == "goal point":
            wherestop = "stop_rot_goal"

        if R_GS_dis < 0.05 and wherestop == "starting point":
            wherestop = "stop_rot_home"


        g_m_c = goal_radian - current_angle.position.z
        if g_m_c < 0:
            g_m_c += 360
        # 목표에 정면으로 바라보게
        if wherestop == "stop_rot_goal" or wherestop == "stop_rot_home":
            turtle_vel.linear.x = 0
            if catch_ar == 1:
                if g_m_c < 2.5 or 357.5 < g_m_c:
                    turtle_vel.angular.z = 0
                    angle += 1
                else:
                    if 180 < g_m_c:
                        turtle_vel.angular.z = -0.10
                        angle = 0
                    else:
                        turtle_vel.angular.z = 0.10
                        angle = 0
            elif DWA_mode == "home":
                p = 1
                if g_m_c < 2.5 or 357.5 < g_m_c:
                    turtle_vel.angular.z = 0
                    angle += 1
                else:
                    if 180 < g_m_c:
                        turtle_vel.angular.z = -0.20
                        angle = 0
                    else:
                        turtle_vel.angular.z = 0.20
                        angle = 0
            elif wherestop == "stop_rot_goal":
                count_ag += 1
                if count_ag <= 180:
                    turtle_vel.angular.z = 0.20
                else:
                    stop_point.data = "fail_stop"
                    turtle_vel.angular.z = 0.0
            if angle > 2 and p == 0 and DWA_mode != "home":
                angle = 0
                p = 1
                stop_point.data = "semi_stop"
            elif angle > 2 and p == 1:
                wherestop = "stop_rot"
                angle = 0
                p = 0
        # 목표와 20cm 이내로 들게
        if wherestop == "stop_rot":
            turtle_vel.linear.x = 0.1
            turtle_vel.angular.z = 0

        # 12cm 이내로 들게 되면 wherestop = "stop_adv"
        if R_G_dis < 0.18:
            turtle_vel.linear.x = 0
            turtle_vel.angular.z = 0
            wherestop = "STOP" ###
            if g_m_c <= 1.5 or 358.5 <= g_m_c:
                turtle_vel.angular.z = 0
                wherestop = "STOP"
            else:
                if 180 < g_m_c < 358.5:
                    turtle_vel.angular.z = 0.05
                    
                if 1.5 < g_m_c < 180:
                    turtle_vel.angular.z = -0.05
                    
        print("{}".format(g_m_c))
        if wherestop == "STOP":
            turtle_vel.linear.x = 0
            turtle_vel.angular.z = 0
            stop_point.data = "stop"
            

        if wherestop == "back":
            turtle_vel.linear.x = -0.18
            turtle_vel.angular.z = 0
            count_ag = 0
            b += 1
            if b > 5:
                turtle_vel.linear.x = 0
                turtle_vel.angular.z = -1.2
                bt += 1
                if bt > 8:
                    b = 0
                    bt = 0
                    if o ==1:
                        wherestop = "starting point"
                        catch_ar = 0
                    if o == 0:
                        wherestop = "goal point"
                        catch_ar = 0

        if DWA_mode != "patrol" and DWA_mode != "home":
            turtle_vel.linear.x = 0
            turtle_vel.angular.z = 0
            stop_point.data = "wait"

        self.publisher.publish(turtle_vel)
        self.stop_point.publish(stop_point)
        print("g:{}, SP:{}, WH:{}, n:{}".format(goal_location_x, stop_point.data, wherestop, n))




def main():
    rospy.init_node('DWA')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    rospy.Subscriber('scan', LaserScan, lambda scan: driver.lds_callback(scan))

    rospy.spin()


if __name__ == "__main__":
    main()




