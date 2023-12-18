#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "functions.h"

// https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_interface/src/move_group_interface_tutorial.cpp

int main(int argc, char** argv) {

    // SETUP -------------------------------------

    ros::init(argc, argv, "move_group_interface");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    move_group_interface.setMaxVelocityScalingFactor(0.2);
    move_group_interface.setMaxVelocityScalingFactor(0.1);

    // MISSIONS -------------------------------------

    get_informations(move_group_interface);
    print_positions(move_group_interface);
    print_rpy_orientation(move_group_interface);

    //go_joint_angles(move_group_interface, 0, -45, 0, -135, 0, 90, 0);
    //go_position_randomIK(move_group_interface, 0.3, 0.5, 0.6);

    //go_joint_angles(move_group_interface, 0, -30, 0, -145, 0, 120, 50);
    //draw_square(move_group_interface, 0.3);

    //get_kinematics_info();
    //auto joints = inverse_kinematics(0.25, 0.25, 0.25, 30, 120, 90);

    ros::shutdown();
    return 0;
}