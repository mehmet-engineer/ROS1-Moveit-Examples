#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
from math import *
import functions

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python", anonymous=True)
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander(group_name)

rospy.sleep(1)

group.set_max_velocity_scaling_factor(0.2)
group.set_max_acceleration_scaling_factor(0.1)

# -------------------------------------------------------------------------------
# MISSIONS

functions.add_table_avoid_below(scene)

functions.print_positions(group)
functions.print_rpy_orientation(group)

functions.go_up_position(group)
#functions.go_to_mission_point(group)
#functions.draw_triangle(group, edge_meter=0.4)

#functions.go_up_position(group)

print("ALL MISSIONS SUCCESSFULLY DONE.")

