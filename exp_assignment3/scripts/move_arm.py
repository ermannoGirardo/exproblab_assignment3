#!/usr/bin/env python

## @package exp_assignment3
#   \file move_arm.py
#   \brief provide a service to move the robot arm 
#   \author Ermanno Girardo
#   \version 1.0
#
#
#   Services: <BR>
#        /move_arm_service
#          
# Description:    
# 
# Thanks to MoveIt! it is possible to move the arm in an easy way.
# In particualar in this case the motion is guaranteed by FK setting manually the arm's joints values
#

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from exp_assignment3.srv import *

#GLOBAL VARIABLES
arm_group=None

def move_arm_clbk(req):
	""" Callback of the move_arm_service."""
	global arm_group
	#first we have to take the actual joints configuration    
	arm_goal = arm_group.get_current_joint_values()
	#then we can fill the values with the request
	arm_goal[0] = req.joint0
	arm_goal[1] = req.joint1
	arm_goal[2] = req.joint2
	arm_goal[3] = req.joint3
	arm_goal[4] = req.joint4

	#move the arm
	arm_group.set_start_state_to_current_state()
	arm_group.go(arm_goal, wait=True)

    # to ensure residual movements
	arm_group.stop()
  
	return True 
    
def main():
    """main of the move_arm node"""
    global arm_group
    #Init MoveIt commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm',anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    arm_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    
    #Set tolerance since we don't need high precision
    arm_group.set_goal_tolerance(0.05)
    
    # create services
    move_arm_server = rospy.Service('move_arm_service', MoveArm, move_arm_clbk)
    
    rospy.spin()
    
    
if __name__ == '__main__':
    main()
