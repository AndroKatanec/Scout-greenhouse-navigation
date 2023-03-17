#!/usr/bin/env python3
#This node removes laser scans in order to ignore obstacles placed on robot that are constantly detected as obstacles

import rospy
from sensor_msgs.msg import LaserScan

def laser_filter_callback(msg):
	msg.ranges = list(msg.ranges)
	for i, ranges in enumerate(msg.ranges):
		if (i<300 or i>1200):
			msg.ranges[i] = 0
			
	msg.ranges = tuple(msg.ranges)
	laser_filter_pub.publish(msg)

rospy.init_node('laser_filter_node')
print('Laser filter node running...')
lase_filter_sub = rospy.Subscriber('/scan', LaserScan, laser_filter_callback)
laser_filter_pub = rospy.Publisher('scan_filtered', LaserScan, queue_size=10)
rospy.spin()
