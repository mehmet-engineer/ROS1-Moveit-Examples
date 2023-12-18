import math
import rospy
import copy
import numpy as np
import geometry_msgs


def get_rad_from_degree(angle):
    if angle == 0:
        return 0
    else:
        temp = 180 / angle
        rad = math.pi / temp
        return rad


def get_degree_from_rad(angle):
    if angle == 0:
        return 0
    else:
        degree = math.degrees(angle)
        return degree


def get_degrees(radians, rounding=2):
    degrees = []
    for i in radians:
        deg = get_degree_from_rad(i)
        deg = round(deg, rounding)
        degrees.append(deg)
    return degrees


def get_quaternion_from_euler(roll, pitch, yaw, input_angle="degree"):

    if input_angle == "degree":
        roll = get_rad_from_degree(roll)
        pitch = get_rad_from_degree(pitch)
        yaw = get_rad_from_degree(yaw)

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


def get_euler_from_quaternion(x, y, z, w, output_angle="degree"):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    if output_angle == "degree":
        roll_x = get_degree_from_rad(roll_x)
        pitch_y = get_degree_from_rad(pitch_y)
        yaw_z = get_degree_from_rad(yaw_z)

    return [roll_x, pitch_y, yaw_z]


def print_positions(group):

    pos_1 = group.get_current_pose().pose.position
    print("Robot position State:\n", round(pos_1.x, 2), round(pos_1.y, 2), round(pos_1.z, 2))


def print_rpy_orientation(group):

    r = round(get_degree_from_rad(group.get_current_rpy()[0]))
    p = round(get_degree_from_rad(group.get_current_rpy()[1]))
    y = round(get_degree_from_rad(group.get_current_rpy()[2]))
    print("current RPY:\n", r, p, y)
    print()


def add_table_avoid_below(scene):

    obj_pose = geometry_msgs.msg.PoseStamped()
    obj_pose.header.frame_id = "base"
    box_name = "box"
    scene.add_box(box_name, obj_pose, size=(1.0, 1.0, 0.18))   # a, b, height
    rospy.sleep(1)


def go_up_position(group):
    joint_values = group.get_current_joint_values()
    joint_values[0] = get_rad_from_degree(0)
    joint_values[1] = get_rad_from_degree(-90)
    joint_values[2] = get_rad_from_degree(0)
    joint_values[3] = get_rad_from_degree(-90)
    joint_values[4] = get_rad_from_degree(0)
    joint_values[5] = get_rad_from_degree(0)
    plan_exec = group.go(joint_values, wait=True)
    if plan_exec != True:
        print("going up plan couldn't be executed")
        quit()
    group.stop()
    group.clear_pose_targets()
    print("reached up position \n")


def go_to_mission_point(group):

    joint_values = group.get_current_joint_values()
    joint_values[0] = get_rad_from_degree(0)
    joint_values[1] = get_rad_from_degree(-120)
    joint_values[2] = get_rad_from_degree(95)
    joint_values[3] = get_rad_from_degree(295)
    joint_values[4] = get_rad_from_degree(-90)
    joint_values[5] = get_rad_from_degree(0)
    plan_exec = group.go(joint_values, wait=True)
    if plan_exec != True:
        print("going up plan couldn't be executed")
        quit()
    group.stop()
    group.clear_pose_targets()
    print("reached target point \n")


def draw_triangle(group, edge_meter=0.4):

    print("drawing triangle...")

    way_points = []
    pose = group.get_current_pose().pose

    # 1. point
    pose.position.y = pose.position.y - edge_meter/2
    way_points.append(copy.deepcopy(pose))

    x_value = math.sqrt( edge_meter**2 - (edge_meter/2)**2 )

    # 2. point
    pose.position.x = pose.position.x + x_value
    pose.position.y = pose.position.y + edge_meter/2
    way_points.append(copy.deepcopy(pose))

    # 3. point
    pose.position.x = pose.position.x - x_value
    pose.position.y = pose.position.y + edge_meter/2
    way_points.append(copy.deepcopy(pose))

    # 4. point
    pose.position.y = pose.position.y - edge_meter/2
    way_points.append(copy.deepcopy(pose))

    env_resolution = 0.01        # 1 cm interpolated in cartesian
    fraction_acc_thresh = 0.8

    plan, fraction = group.compute_cartesian_path(way_points, env_resolution, 0.0)

    if fraction > fraction_acc_thresh:
        print("triangle plan executing...")
        plan2_exec = group.execute(plan, wait=True)
        
        if plan2_exec == True:
            print("triangle mission completed. \n")
        else:
            print("execution failed. \n")
            quit()
    else:
        print("fraction accuracy lower than 0.8, not executed.")
        print("mission failed.")
        quit()


def draw_circle(group, radius_meter=0.15):

    print("drawing circle...")

    way_points = []

    fraction_acc_thresh = 0.8

    angle = 0
    circle_resolution = 10
    d_angle = circle_resolution * 3.14/180
    pose_circle = group.get_current_pose().pose

    x_center = pose_circle.position.x
    y_center = pose_circle.position.y

    for i in range(int(360/circle_resolution)):
        angle = angle + d_angle
        pose_circle.position.x = x_center + radius_meter*math.cos(angle)
        pose_circle.position.y = y_center + radius_meter*math.sin(angle)
        way_points.append(copy.deepcopy(pose_circle))

    plan, fraction = group.compute_cartesian_path(way_points, circle_resolution, 0.0)

    if fraction > fraction_acc_thresh:
        print("circle plan executing...")
        plan3_exec = group.execute(plan, wait=True)

        if plan3_exec == True:
            print("circle mission completed. \n")
        else:
            print("execution failed. \n")
            quit()
    else:
        print("fraction accuracy lower than 0.8, not executed.")
        print("mission failed.")
        quit()