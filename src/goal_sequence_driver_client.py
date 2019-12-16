#!/usr/bin/env python

import sys
import rospy
import rospkg
import rosparam

from neo_goal_sequence_driver.srv import *

# path of goal_list.yaml file
rospack = rospkg.RosPack()
goal_list = rospack.get_path('neo_goal_sequence_driver') + '/config/goal_list.yaml'

# client of the service
def goal_sequence_driver_client(switch, infi_param, print_option, patience):

	rospy.wait_for_service('goal_sequence_driver')
	try:
		gsd = rospy.ServiceProxy('goal_sequence_driver', command)
		resp = gsd(switch, infi_param, print_option, patience)
		return resp.message
	except rospy.ServiceException, e:

		print("Service call failed: %s"%e)

# load goal list from .yaml
'''
def yaml_to_rosparam(file):

	yaml = rosparam.load_file(file)
'''

if __name__ == "__main__":

	if (len(sys.argv) == 5) or ((len(sys.argv) == 7) and sys.argv[5] == '__name:=neo_goal_sequence_driver'):

		switch = str(sys.argv[1])
		infi_param = str(sys.argv[2])
		print_option = str(sys.argv[3])
		patience = int(sys.argv[4])
		try:
		
			goal_sequence_driver_client(switch, infi_param, print_option, patience)
		except:

			print("goal_sequence_driver_client is down.")
			rospy.loginfo("goal_sequence_driver_client is down.")

	else:

			print("Unknown command type, please use e.g. 'up False 5'.")
			print(sys.argv)