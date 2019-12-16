#!/usr/bin/env python

import rospy
import numpy as np

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import goal_sequence_driver_server as server

# read goals
def load_goals():

	i = 0
	goals = []
	while(not rospy.is_shutdown()):
		try:

			goals.append([rospy.get_param(server.prefix_goal+str(i+1)+"/X"), \
				rospy.get_param(server.prefix_goal+str(i+1)+"/Y"), \
				rospy.get_param(server.prefix_goal+str(i+1)+"/theta")])
			#goals[i][0] = rospy.get_param(prefix_goal+"i"+"/X")
			#goals[i][1] = rospy.get_param(prefix_goal+"i"+"/Y")
			#goals = np.vstack((goals, np.zeros(2)))
		except:
			print(goals)
			return goals
		i += 1

# pack goals up with visualization_msgs/Marker
def visualize_goals_with_markers(goals):

	rospy.init_node('neo_goal_sequence_driver_visualization')
	rate = rospy.Rate(5)
	pub = rospy.Publisher('/neo_goal_sequence_driver/visualization', MarkerArray, queue_size=1)
	marker_array = MarkerArray()

	i = 0
	for goal in goals:
		# defining a single marker
		marker = Marker()
		i += 1
		marker.header.frame_id = "/map"
		marker.header.seq = i+1
		marker.id = i+1
		marker.type = marker.ARROW
		marker.action = marker.ADD
		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 0.1
		marker.color.a = 1.0
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = goal[0]
		marker.pose.position.y = goal[1]
		quaternion = quaternion_from_euler(0, 0, np.radians(goal[2]))
		marker.pose.orientation.x = quaternion[0]
		marker.pose.orientation.y = quaternion[1] 
		marker.pose.orientation.z = quaternion[2]
		marker.pose.orientation.w = quaternion[3]
		marker_array.markers.append(marker)
	while(not rospy.is_shutdown() and rospy.get_param(server.occupied)):
		pub.publish(marker_array)
		rate.sleep()

# pack goals up with geometry_msgs/PoseStamped
def visualize_goals_with_tf(goals)
	pass

if __name__ == '__main__':

	goals = load_goals()
	#visualize_goals_with_markers(goals)
	visualize_goals_with_tf(goals)