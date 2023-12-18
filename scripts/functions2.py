#!/usr/bin/env python3

import sys
import rospkg
import yaml
import rospy
import math
import moveit_commander
import tf.transformations as tf
from geometry_msgs.msg import Pose

class RobotControlMoveit():
    
    def __init__(self, group_name, robot_model_name, eef_link, tcp_link):
        rospy.loginfo("Robot Control Moveit and Gazebo Link Attacher Client initializing...")
    
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        
        self.robot_model_name = robot_model_name
        self.eef_link = eef_link
        self.tcp_link = tcp_link
        self.group_name = group_name
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_end_effector_link(self.tcp_link)
        
        self.init_gazebo_services()
        self.get_gazebo_box_links()
        
        rospy.loginfo("Robot is ready now.")
        rospy.sleep(1)

    def set_vel_and_acc(self, velocity, acc):
        rospy.loginfo("Velocity and acceleration are setting...")
        self.move_group.set_max_velocity_scaling_factor(velocity)
        self.move_group.set_max_acceleration_scaling_factor(acc)

    def get_current_joint_positions(self):
        joint_rad_pos = self.move_group.get_current_joint_values()
        joint_deg_pos = self.get_deg_from_rad(joint_rad_pos)
        return joint_deg_pos
    
    def go_to_joint_position(self, joint_angles):
        success = False
        string = [str(i) for i in joint_angles]
        rospy.loginfo("Going joint target --> [%s]", " ".join(string))
        joint_angles = self.get_rad_from_deg(joint_angles)
        
        plan_exec = self.move_group.go(joint_angles, wait=True)
        if plan_exec != True:
            rospy.logwarn("Plan couldn't be executed")
        else:
            rospy.loginfo("Target positions executed.")
            success = True
        self.move_group.stop()
        
        return success
    
    def go_to_cart_position(self, x, y, z, euler_rotation):
        rospy.loginfo("Going cart target --> %f %f %f", x, y, z)
        pose_goal = Pose()
        
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        quaternion = tf.quaternion_from_euler(euler_rotation[0], euler_rotation[1], euler_rotation[2])
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        self.move_group.set_pose_target(pose_goal)
        
        success = self.move_group.go(wait=True)
        if success != True:
            rospy.logwarn("Plan couldn't be executed")
        else:
            rospy.loginfo("Target positions executed.")
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success
    
    def go_with_relative_linear_cart(self, x, y, z):
        eef_pose = self.move_group.get_current_pose().pose
        
        eef_pose.position.x = eef_pose.position.x + x
        eef_pose.position.y = eef_pose.position.y + y
        eef_pose.position.z = eef_pose.position.z + z
        
        rospy.loginfo("Going relative cart target --> %f %f %f", eef_pose.position.x, eef_pose.position.y, eef_pose.position.z)
        (plan, fraction) = self.move_group.compute_cartesian_path([eef_pose], 0.01, 0.0)
        fraction_thresh = 0.6
        if fraction < fraction_thresh:
            rospy.logwarn("Cartesian fraction is lower than %f ", fraction_thresh)
        success = self.move_group.execute(plan, wait=True)
        if success != True:
            rospy.logwarn("Plan couldn't be executed")
        else:
            rospy.loginfo("Target positions executed.")
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success
    
    def go_home(self):
        rospy.loginfo("Going home ...")
        home_angles = [0, 0, 0, 0, 0, 0]
        joint_angles = self.get_rad_from_deg(home_angles)
        success = self.go_to_joint_position(joint_angles)
        return success
