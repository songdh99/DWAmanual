#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Pose

class Control():
    def __init__(self):
        self.DWA_pub = rospy.Publisher('DWA', String, queue_size = 10)
        self.mode = "patrol"
        self.stop_check = False
        
    
    def Tower(self):
        if self.mode == "patrol":
            rospy.loginfo("mode : %s", self.mode)
            self.DWA_pub.publish("front")
            self.mode = "back"
            
            
        if self.mode == "back":
            rospy.loginfo("mode : %s", self.mode)
            self.DWA_pub.publish("back")
            self.mode = "stop"
            
            
        if self.mode == "stop":
            rospy.loginfo("mode : %s", self.mode)
            self.DWA_pub.publish("stop")
            self.mode = "patrol"
            
def main():
    rospy.init_node("contol_tower")
    con = Control()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        con.control()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass