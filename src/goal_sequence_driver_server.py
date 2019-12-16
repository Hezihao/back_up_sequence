#! /usr/bin/env python

import time
import rospy
import actionlib
import numpy as np

from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from neo_goal_sequence_driver.srv import command, commandResponse

# set manually
service_called = True
smallest_step_length = 0.000002

# definition of CMD when not called by service
# default values could be changed here
if(not service_called):
	class CMD:
		switch = "up"
		infi_param = False
		patience = 5
	cmd_default = CMD()

# param list
former_pose = None
time_start = None
dist = 0.0

pkg_name = '/neo_goal_sequence_driver'
prefix_goal = pkg_name+'/goals/goal_'
switch = pkg_name+'/switch'
infi_param = pkg_name+'/infi_param'
print_option = pkg_name+'/print_option'
patience = pkg_name+'/patience'
occupied = pkg_name+'/occupied'

# server of this service
def goal_sequence_driver_server():

	rospy.init_node('goal_sequence_driver_server')
	rospy.loginfo("goal_sequence_driver_server is up.")
	# testing odometer on server
	odom_sub = rospy.Subscriber("odom", Odometry, odom_callback)
	rospy.loginfo("Odometer is up.")
	server = rospy.Service('goal_sequence_driver', command, goal_sequence_driver)
	rospy.spin()

# callback function of odom subscriber
def odom_callback(odometry):

	global dist
	global time_start
	global former_pose
	global smallest_step_length

	pose = odometry.pose.pose.position
	if(former_pose == None):

		former_pose = pose
		time_start = rospy.get_time()
	else:

		d_dist = np.sqrt((pose.x - former_pose.x)**2 + (pose.y - former_pose.y)**2)
		former_pose = pose
		if(d_dist < smallest_step_length):
			d_dist = 0
		dist += d_dist

# transformation of data: euler -> quaternion
def create_quaternion(goal_num):

	# read position & orientation(euler) from param and transfer them
	x = rospy.get_param(prefix_goal+str(goal_num+1)+'/X')
	y = rospy.get_param(prefix_goal+str(goal_num+1)+'/Y')
	theta = np.radians(rospy.get_param(prefix_goal+str(goal_num+1)+'/theta'))
	quaternion = quaternion_from_euler(0, 0, theta)
	return [x, y, quaternion]

# creat goal structure for move_base node
def create_goal(pose):

	# create goal from position & orientation(quaternion)
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = pose[0]
	goal.target_pose.pose.position.y = pose[1]
	goal.target_pose.pose.orientation.z = pose[2][2]
	goal.target_pose.pose.orientation.w = pose[2][3]
	return goal

# the call back function of service
def goal_sequence_driver(cmd):

	global SHOW_CURRENT
	global SHOW_TOTAL

	# need to check if the service is asked to be shut down
	if(cmd.switch == "down"):
		
		rospy.loginfo("goal_sequence_driver down.")
		rospy.set_param(switch, False)

		return "Shut down."
		
	elif(cmd.switch == "up"):

		rospy.loginfo("goal_sequence_driver up.")
		rospy.set_param(switch, True)
	
	if(cmd.infi_param == "True"):

		rospy.set_param(infi_param, True)
	elif(cmd.infi_param == "False"):

		rospy.set_param(infi_param, False)
	rospy.set_param(print_option, cmd.print_option)
	rospy.set_param(patience, cmd.patience)
	try:
	
		rospy.get_param(occupied)
	except:

		rospy.set_param(occupied, False)
	i = 0
	if(not rospy.get_param(occupied)):

		rospy.set_param(occupied, True)
		client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		current_task_start = rospy.get_time()
		if(client.wait_for_server()):

			rospy.loginfo("Client server up.")
		while(not rospy.is_shutdown() and rospy.get_param(switch)):

			try:
				# could cause incomplete visit of the list
				pose = create_quaternion(i)
			except:

				rospy.loginfo("All goals reached, sequence drive task completed.")
				i = 0
				if(not rospy.get_param(infi_param)):
					
					break
			goal = create_goal(pose)
			client.send_goal(goal)
			while(not rospy.is_shutdown()):
				
				if(client.wait_for_result(rospy.Duration.from_sec(rospy.get_param(patience))) == True):

					rospy.loginfo("Goal reached.")
					if(rospy.get_param(print_option) == "all" or rospy.get_param(print_option) == "local"):
						rospy.loginfo("Current task duration:"+str(rospy.get_time() - current_task_start)+"s")
					# just for test
					time.sleep(1)
					break
			i += 1
		rospy.set_param(occupied, False)
		if(rospy.get_param(print_option) == "all" or rospy.get_param(print_option) == "total"):
			rospy.loginfo("Total distance traveled is:"+str(dist)+"m")
			rospy.loginfo("Total time traveled is:"+str(rospy.get_time() - time_start)+"s")
	return "Service done."


if __name__ == '__main__':

	if(service_called):

		goal_sequence_driver_server()
	else:
		# if not called by service, this is supposed to be a very quick
		# demonstration program
		rospy.init_node('goal_sequence_driver')
		rospy.loginfo("goal_sequence_driver is not responding to any request.")
		goal_sequence_driver(cmd_default)