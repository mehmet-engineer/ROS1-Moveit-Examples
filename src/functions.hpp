#pragma once

double get_rad_from_degree(double degree);
double get_degree_from_rad(double rad);

void get_informations(moveit::planning_interface::MoveGroupInterface& move_group_interface);
void print_positions(moveit::planning_interface::MoveGroupInterface& move_group_interface);
void print_rpy_orientation(moveit::planning_interface::MoveGroupInterface& move_group_interface);

void set_end_effector_orientation(moveit::planning_interface::MoveGroupInterface& move_group_interface, int roll, int pitch, int yaw);
void go_position_randomIK(moveit::planning_interface::MoveGroupInterface& move_group_interface, float x, float y, float z);
void go_joint_angles(moveit::planning_interface::MoveGroupInterface& move_group_interface, float j0, float j1, float j2, float j3, float j4, float j5, float j6);
void add_path_constraints(moveit::planning_interface::MoveGroupInterface& move_group_interface, float orient, float x, float y, float z);
void go_with_constraints(moveit::planning_interface::MoveGroupInterface& move_group_interface, float x, float y, float z);
void draw_square(moveit::planning_interface::MoveGroupInterface& move_group_interface, float edge_meter);

void get_kinematics_info();
std::vector<double> inverse_kinematics(float x, float y, float z, float roll, float pitch, float yaw);

void add_pick_and_place_objects();
std::vector<moveit_msgs::Grasp> init_gripper_config(float x, float y, float z, int roll, int pitch, int yaw);
void set_open_gripper(moveit_msgs::Grasp& grasping);
void set_close_gripper(moveit_msgs::Grasp& grasping);
void gripper_pick(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<moveit_msgs::Grasp> grasp);
void gripper_place(moveit::planning_interface::MoveGroupInterface& move_group, float x, float y, float z, int roll, int pitch, int yaw);

void add_table_avoid_below(std::string link_name, moveit::planning_interface::MoveGroupInterface& move_group_interface);

