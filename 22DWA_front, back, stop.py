#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Bool, String
import numpy as np
import time

class SelfDrive:
    def __init__(self):
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.fin_pub = rospy.Publisher("fin_move_close", Bool, queue_size=1)
        self.turtle_vel = Twist()
        self.bool = Bool()
        self.mode = "stop"
        self.real_mode = 0
        
        
    def init_range(self, scan):
        Scan = np.array(scan)
        front = Scan[0]
        return front
    
    def lds_callback(self, msg, scan):
        self.mode = msg.data
        print(self.mode)
        front = self.init_range(scan.ranges)
        
        def action(self):
            rospy.Subscriber('start_move_closed', String, self.callback)
            rate = rospy.Rate(1)
            rospy.loginfo(self.mode)
            
            if self.mode == "back":
                while 1:
                    self.move(-0.05, 0)
                    rate.sleep()
                    if front < 0.4:
                        self.mode == "stop"
                
            if self.mode == "front":
                while 1:
                    self.move(0.05, 0)
                    rate.sleep()
                    if front < 0.4:
                        self.mode == "stop"
                
            if self.mode == "stop" and 0 < front < 0.4:
                self.move(0,0)
                rate.sleep()
                if not front:
                    self.mode = "front"
    
        def move(self, x, z):
            self.turtle_vel.linear.x = x
            self.turtle_vel.angular.z = z
            self.publisher.publish(self.turtle_vel)
    
    
def main():
    rospy.init_node('DWA')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    rate = rospy.Rate(1)
    rospy.Subscriber('scan', LaserScan, lambda scan: driver.lds_callback(scan))
    while not rospy.is_shutdown():
        driver.act()
        rate.sleep()
            
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass            
        
