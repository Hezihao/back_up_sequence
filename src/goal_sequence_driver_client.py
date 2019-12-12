#!/usr/bin/env python

import sys
import rospy

from pyyaml 
from neo_goal_sequence_driver.srv import *

def goal_sequence_driver_client(switch, infi_param, patience):

	rospy.wait_for_service('goal_sequence_driver')
	try:
		gsd = rospy.ServiceProxy('goal_sequence_driver', command)
		resp = gsd(switch, infi_param, patience)
		return resp.message
	except rospy.ServiceException, e:

		print("Service call failed: %s"%e)

if __name__ == "__main__":

	if(len(sys.argv) == 4):
	
		switch = str(sys.argv[1])
		infi_param = str(sys.argv[2])
		patience = int(sys.argv[3])
		goal_sequence_driver_client(switch, infi_param, patience)
	else:

			print("Unknown command type, please use e.g. 'up False 5'.")