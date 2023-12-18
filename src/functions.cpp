#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "functions.h"

// https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_interface/src/move_group_interface_tutorial.cpp

double get_rad_from_degree(double degree)
{
    double pi = 3.14159;
    double rad = (pi * degree) / 180;
    return rad;
}

double get_degree_from_rad(double rad)
{
    double pi = 3.14159;
    double degree = rad * (180 / pi);
    return degree;
}

void get_informations(moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
    // it is not working well
    //std::string planning_frame = move_group_interface.getPlanningFrame().c_str();
    //ROS_INFO("Planning Frame --> %s", planning_frame);

    //std::string eef_link = move_group_interface.getEndEffectorLink().c_str();
    //ROS_INFO("End Effector Link --> %s", eef_link);

    //const std::vector<std::string> active_joints = move_group_interface.getActiveJoints();
    //for (auto i = active_joints.begin(); i != active_joints.end(); ++i)
    //    ROS_INFO("Active Joints --> %s", *i);

    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    // moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

    ROS_INFO("Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
          move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", \n"));

    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
    double pose_x = current_pose.pose.position.x;
    double pose_y = current_pose.pose.position.y;
    double pose_z = current_pose.pose.position.z;
    ROS_INFO("Current Pose position x --> %f", pose_x);
    ROS_INFO("Current Pose position y --> %f", pose_y);
    ROS_INFO("Current Pose position z --> %f", pose_z);
 
    std::vector<double> current_rpy = move_group_interface.getCurrentRPY();
    ROS_INFO("Current Roll --> %f", current_rpy[0]);
    ROS_INFO("Current Pitch --> %f", current_rpy[1]);
    ROS_INFO("Current Yaw --> %f", current_rpy[2]);

    std::vector<double> current_joints = move_group_interface.getCurrentJointValues();
    ROS_INFO("Current Joint Values_0 --> %f", current_joints[0]);
    ROS_INFO("Current Joint Values_1 --> %f", current_joints[1]);
    ROS_INFO("Current Joint Values_2 --> %f", current_joints[2]);
    ROS_INFO("Current Joint Values_3 --> %f", current_joints[3]);
    ROS_INFO("Current Joint Values_4 --> %f", current_joints[4]);
    ROS_INFO("Current Joint Values_5 --> %f", current_joints[5]);
}

void print_positions(moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
    double pose_x = current_pose.pose.position.x;
    double pose_y = current_pose.pose.position.y;
    double pose_z = current_pose.pose.position.z;
    ROS_INFO("Current Pose position x --> %f", pose_x);
    ROS_INFO("Current Pose position y --> %f", pose_y);
    ROS_INFO("Current Pose position z --> %f", pose_z);
}

void print_rpy_orientation(moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
    std::vector<double> current_rpy = move_group_interface.getCurrentRPY();
    double roll_degree = get_degree_from_rad(current_rpy[0]);
    double pitch_degree = get_degree_from_rad(current_rpy[1]);
    double yaw_degree = get_degree_from_rad(current_rpy[2]);
    ROS_INFO("Current Roll degree --> %f", roll_degree);
    ROS_INFO("Current Pitch degree --> %f", pitch_degree);
    ROS_INFO("Current Yaw degree --> %f", yaw_degree);
}

void set_end_effector_orientation(moveit::planning_interface::MoveGroupInterface& move_group_interface, int roll, int pitch, int yaw)
{
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(roll, pitch, yaw);

    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = myQuaternion.getW();
    target_pose.orientation.x = myQuaternion.getX();
    target_pose.orientation.y = myQuaternion.getY();
    target_pose.orientation.z = myQuaternion.getZ();
    ROS_INFO("Target orientation was set");

    //return
}

void go_position_randomIK(moveit::planning_interface::MoveGroupInterface& move_group_interface, float x, float y, float z)
{
    geometry_msgs::Pose target_pose = move_group_interface.getCurrentPose().pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    move_group_interface.setPoseTarget(target_pose);
    ROS_INFO("Target position was set");
    ROS_INFO("plan calculating ...");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success == true) {
        ROS_INFO("Plan was successfully created");
        
        moveit_msgs::RobotTrajectory trajectory = my_plan.trajectory_;
        int len_trajectory = trajectory.joint_trajectory.points.size();
        ROS_INFO("trajectory size: %d", len_trajectory);

        ROS_INFO("going target...");
        move_group_interface.execute(my_plan);

        // CHECK ----------------------------------------------

        geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
        double pose_x = current_pose.pose.position.x;
        double pose_y = current_pose.pose.position.y;
        double pose_z = current_pose.pose.position.z;

        double tolerance_range = 0.2;
        bool x_pass = (x <= pose_x + tolerance_range) & (x >= pose_x - tolerance_range);
        bool y_pass = (x <= pose_y + tolerance_range) & (x >= pose_y - tolerance_range);
        bool z_pass = (x <= pose_z + tolerance_range) & (x >= pose_z - tolerance_range);

        if ( x_pass & y_pass & z_pass == true) {
            ROS_INFO("Reached target position!");
        }
        else {
            ROS_INFO("Couldn't reached target position");
        }
    }
    else {
        ROS_INFO("Plan failed");
    }
    move_group_interface.clearPoseTargets();
}

void go_joint_angles(moveit::planning_interface::MoveGroupInterface& move_group_interface, float j0, float j1, float j2, float j3, float j4, float j5, float j6)
{
    const moveit::core::JointModelGroup* joint_model_group;
    joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("panda_arm");

    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    std::vector<double> joints;
    current_state->copyJointGroupPositions(joint_model_group, joints);

    joints[0] = get_rad_from_degree(j0);
    joints[1] = get_rad_from_degree(j1);
    joints[2] = get_rad_from_degree(j2);
    joints[3] = get_rad_from_degree(j3);
    joints[4] = get_rad_from_degree(j4);
    joints[5] = get_rad_from_degree(j5);
    joints[6] = get_rad_from_degree(j6);
    move_group_interface.setJointValueTarget(joints);
    ROS_INFO("Target joint position was set");
    ROS_INFO("plan calculating ...");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success == true) {
        ROS_INFO("Joint space plan was successfully created");

        moveit_msgs::RobotTrajectory trajectory = my_plan.trajectory_;
        int len_trajectory = trajectory.joint_trajectory.points.size();
        ROS_INFO("trajectory size: %d", len_trajectory);

        ROS_INFO("going target...");
        move_group_interface.move();
        
        ROS_INFO("Reached target joint position!");
    }
    else {
        ROS_INFO("Plan failed");
    }
    move_group_interface.clearPoseTargets();
}

void add_path_constraints(moveit::planning_interface::MoveGroupInterface& move_group_interface, float orient, float x, float y, float z)
{
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "panda_link7";
    ocm.header.frame_id = "panda_link0";

    ocm.orientation.w = orient;
    ocm.absolute_x_axis_tolerance = x;
    ocm.absolute_y_axis_tolerance = y;
    ocm.absolute_z_axis_tolerance = z;
    ocm.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_interface.setPathConstraints(test_constraints);
    ROS_INFO("constraints were set");
}

void go_with_constraints(moveit::planning_interface::MoveGroupInterface& move_group_interface, float x, float y, float z)
{
    const moveit::core::JointModelGroup* joint_model_group;
    joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("panda_arm");

    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
    double pose_x = current_pose.pose.position.x;
    double pose_y = current_pose.pose.position.y;
    double pose_z = current_pose.pose.position.z;
    double orient_w = current_pose.pose.orientation.w;
    double orient_x = current_pose.pose.orientation.x;
    double orient_y = current_pose.pose.orientation.y;
    double orient_z = current_pose.pose.orientation.z;

    moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
    geometry_msgs::Pose start_pose;
    start_pose.orientation.w = orient_w;
    start_pose.orientation.x = orient_x;
    start_pose.orientation.y = orient_y;
    start_pose.orientation.z = orient_z;
    start_pose.position.x = pose_x;
    start_pose.position.y = pose_y;
    start_pose.position.z = pose_z;
    start_state.setFromIK(joint_model_group, start_pose);
    move_group_interface.setStartState(start_state);

    geometry_msgs::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    move_group_interface.setPoseTarget(target_pose);

    move_group_interface.setPlanningTime(10.0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success == true) {
        ROS_INFO("plan was successfully created");
        ROS_INFO("going target...");
        move_group_interface.execute(my_plan);
        ROS_INFO("Reached target position!");
    }
    else {
        ROS_INFO("Plan failed");
    }

    move_group_interface.clearPathConstraints();
    move_group_interface.clearPoseTargets();
}

void draw_square(moveit::planning_interface::MoveGroupInterface& move_group_interface, float edge_meter)
{
    ROS_INFO("drawing %.2f edge meter triangle ...", edge_meter);

    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
    double pose_x = current_pose.pose.position.x;
    double pose_y = current_pose.pose.position.y;
    double pose_z = current_pose.pose.position.z;
    double orient_w = current_pose.pose.orientation.w;
    double orient_x = current_pose.pose.orientation.x;
    double orient_y = current_pose.pose.orientation.y;
    double orient_z = current_pose.pose.orientation.z;

    geometry_msgs::Pose start_pose;
    start_pose.position.x = pose_x;
    start_pose.position.y = pose_y;
    start_pose.position.z = pose_z;
    start_pose.orientation.w = orient_w;
    start_pose.orientation.x = orient_x;
    start_pose.orientation.y = orient_y;
    start_pose.orientation.z = orient_z;

    std::vector<geometry_msgs::Pose> waypoints;
    
    // start point
    waypoints.push_back(start_pose);

    geometry_msgs::Pose target_pose = start_pose;

    // 1. point
    target_pose.position.y -= edge_meter/2;
    waypoints.push_back(target_pose);

    // 2. point
    target_pose.position.x -= edge_meter/2;
    waypoints.push_back(target_pose);

    // 3. point
    target_pose.position.y += edge_meter;
    waypoints.push_back(target_pose);

    // 4. point
    target_pose.position.x += edge_meter;
    waypoints.push_back(target_pose);

    // 5. point
    target_pose.position.y -= edge_meter;
    waypoints.push_back(target_pose);

    // 6. point
    target_pose.position.x -= edge_meter/2;
    waypoints.push_back(target_pose);

    // returning start point
    waypoints.push_back(start_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.8) {
        ROS_INFO("plan calculated %.2f fraction achieved", fraction*100.0);
        ROS_INFO("drawing starting...");
        move_group_interface.execute(trajectory);
        ROS_INFO("square mission successfully done");
    }
    else {
        ROS_INFO("plan failed due to low %.2f fraction", fraction*100.0);
    }

    move_group_interface.clearPoseTargets();
}

void get_kinematics_info()
{
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

    kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");

    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group,
                                kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                reference_point_position, jacobian);
    ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
}

std::vector<double> inverse_kinematics(float x, float y, float z, float roll, float pitch, float yaw)
{
    // https://github.com/felipepolido/EigenExamples
    // https://ros-planning.github.io/moveit_tutorials/doc/robot_model_and_robot_state/robot_model_and_robot_state_tutorial.html

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    int len_joints = joint_names.size();

    // Eigen::Isometry3d desired_pose;
    // desired_pose.translation().x() = x;
    // desired_pose.translation().y() = y;
    // desired_pose.translation().z() = z;

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(roll, pitch, yaw);

    double orient_w = myQuaternion.getW();
    double orient_x = myQuaternion.getX();
    double orient_y = myQuaternion.getY();
    double orient_z = myQuaternion.getZ();

    Eigen::Quaterniond q(orient_w, orient_x, orient_y, orient_z); 
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d T(x, y, z);
    Eigen::Matrix4d desired_matrix;
    desired_matrix.setIdentity();
    desired_matrix.block<3,3>(0,0) = R;
    desired_matrix.block<3,1>(0,3) = T;

    Eigen::Isometry3d matrix;
    matrix = desired_matrix.matrix();

    double timeout = 0.1;
    bool is_ik_found = kinematic_state->setFromIK(joint_model_group, matrix, timeout);

    if (is_ik_found == true) {
        
        ROS_INFO("IK solution found!");

        std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

        //for (std::size_t i = 0; i < len_joints; ++i) {
        //    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        //}

        return joint_values;
    }
    else {
        ROS_INFO("did not find IK solution");
    }
    
}

void add_pick_and_place_objects()
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Table object 1 
    collision_objects[0].id = "table_obj_1";
    collision_objects[0].header.frame_id = "panda_link0";

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[0].operation = collision_objects[0].ADD;

    // Table object 2
    collision_objects[1].id = "table_obj_2";
    collision_objects[1].header.frame_id = "panda_link0";

    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.4;
    collision_objects[1].primitives[0].dimensions[1] = 0.2;
    collision_objects[1].primitives[0].dimensions[2] = 0.4;

    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.5;
    collision_objects[1].primitive_poses[0].position.z = 0.2;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;

    collision_objects[1].operation = collision_objects[1].ADD;

    // Object that will be grasped
    collision_objects[2].id = "object";
    collision_objects[2].header.frame_id = "panda_link0";

    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 0.2;

    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.5;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.5;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;

    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO("planning scene objects are adding...");
    ros::WallDuration(1.0).sleep();
    
}

std::vector<moveit_msgs::Grasp> init_gripper_config(float x, float y, float z, int roll, int pitch, int yaw)
{
    ROS_INFO("gripper configuration initializing...");
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(get_rad_from_degree(roll), get_rad_from_degree(pitch), get_rad_from_degree(yaw));
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = x;
    grasps[0].grasp_pose.pose.position.y = y;
    grasps[0].grasp_pose.pose.position.z = z;

    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    return grasps;
}

void set_open_gripper(moveit_msgs::Grasp& grasping)
{
    trajectory_msgs::JointTrajectory posture = grasping.pre_grasp_posture;
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void set_close_gripper(moveit_msgs::Grasp& grasping)
{
    trajectory_msgs::JointTrajectory posture = grasping.pre_grasp_posture;
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void gripper_pick(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<moveit_msgs::Grasp> grasps)
{
    set_open_gripper(grasps[0]);
    set_close_gripper(grasps[0]);
    ROS_INFO("picking operation starting...");

    move_group.setSupportSurfaceName("table1");
    move_group.pick("object", grasps);
    ROS_INFO("picking operation successfully done.");
    ros::WallDuration(1.0).sleep();
}

void gripper_place(moveit::planning_interface::MoveGroupInterface& move_group, float x, float y, float z, int roll, int pitch, int yaw)
{
    ROS_INFO("placing operation starting...");
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(get_rad_from_degree(roll), get_rad_from_degree(pitch), get_rad_from_degree(yaw));
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    // For place location, we set the value to the exact location of the center of the object
    place_location[0].place_pose.pose.position.x = x;
    place_location[0].place_pose.pose.position.y = y;
    place_location[0].place_pose.pose.position.z = z;

    // Setting pre-place approach
    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Setting posture of eef after placing object
    auto posture = place_location[0].post_place_posture;
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);

    move_group.setSupportSurfaceName("table2");
    move_group.place("object", place_location);
    ROS_INFO("placing operation successfully done.");
    ros::WallDuration(1.0).sleep();
}

void add_table_avoid_below(std::string link_name, moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(link_name);
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface.getPlanningFrame();
    collision_object.id = "box1";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.5;
    primitive.dimensions[primitive.BOX_Z] = 0.2;
    
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);
    visual_tools.trigger();
}