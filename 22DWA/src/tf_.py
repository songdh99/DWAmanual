#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped


class Transform():
    def __init__(self):
        self.a_about_m_pos = rospy.Publisher('a_about_m_pos', Pose, queue_size=1)
        self.goal_dist = rospy.Publisher('goal_dist', float, queue_size=1)

        rospy.Subscriber('mani_pos', Pose, self.mani_pos)
        rospy.Subscriber('current_xyz', Pose, self.current_xyz)
        rospy.Subscriber('aruco_xyzw', Pose, self.aruco_xyzw)
        rospy.Rate(10)

    def pub_tf(self):
        a = Pose()
        b = float()
        a.position.x = 1
        a.position.y = 2
        a.position.z = 3

        b = 45
        self.a_about_m_pos.publish(a)
        self.goal_dist.publish(b)



    def mani_pos(self, mani_pos):
        print('mani_pos : {}, {}, {}'.format(mani_pos.position.x, mani_pos.position.y, mani_pos.position.z)  )

    def current_xyz(self, current_xyz):
        print('current_xyz : {}, {}, {}'.format(current_xyz.position.x, current_xyz.position.y, current_xyz.position.z))

    def aruco_xyzw(self, aruco_xyzw):
        print('aruco_xyzw : {}, {}, {}'.format(aruco_xyzw.orientation.x, aruco_xyzw.orientation.y, aruco_xyzw.orientation.z))


def main():
    rospy.init_node("tf")
    tf = Transform()
    while not rospy.is_shutdown():
        tf.pub_tf()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


