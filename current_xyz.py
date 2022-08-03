#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import math
import tf

def main():
	rospy.init_node('send_current_xyz')
	listener = tf.TransformListener()
	pub = rospy.Publisher('current_xyz', Pose, queue_size=1)
	pub2 = rospy.Publisher('current_angle', Pose, queue_size=1)
	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		
		current_xyz = Pose()
		current_xyz.position.x = trans[0]
		current_xyz.position.y = trans[1]
		current_xyz.position.z = trans[2]
		
		current_angle = Pose()
		current_angle.position.z = rot[2]
		

		print("x : {} y : {} z : {} angle.z : {} ".format(current_xyz.position.x, current_xyz.position.y, current_xyz.position.z, current_angle.position.z))
		pub.publish(current_xyz)
		pub2.publish(current_angle)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
		