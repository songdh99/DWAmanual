#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import math
import tf

def main():
	rospy.init_node('send_current_xyz')
	listener = tf.TransformListener()
	pub = rospy.Publisher('current_xyz', Pose, queue_size=1)
	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		
		current_xyz = Pose()
		current_xyz.position.x = trans[0]
		current_xyz.position.y = trans[1]
		current_xyz.position.z = trans[2]

		current_xyz.orientation.x = rot[0]
		current_xyz.orientation.y = rot[1]
		current_xyz.orientation.z = rot[2]
		current_xyz.orientation.w = rot[3]
		
		

		print("x : {} y : {} Quatuernion_z : {} Quatuernion_w : {}".format(current_xyz.position.x, current_xyz.position.y, current_xyz.orientation.z, current_xyz.orientation.w))
		pub.publish(current_xyz)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass