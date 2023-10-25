#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import numpy as np
from nav2_simple_commander.robot_navigator import BasicNavigator
import tf_transformations
import csv
import math

def csv_to_array(csv_file):
    targets = []
    with open(csv_file,'r') as file:
        reader = csv.reader(file)
        next(reader) #Skips header
        for row in reader:
            x,y,z,o_x,o_y,o_z,o_w = map(float,row)
            yaw = math.atan2(2.0*(o_z*o_w + o_x*o_y), 1.0 - 2.0*(o_y*o_y + o_z*o_z))
            targets.append((x,y,z,yaw))
    return targets

def create_pose_stamped(navigator:BasicNavigator, position_x, position_y,orientation_z):
    q_x,q_y,q_z,q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def create_goals(targets):
    nav = BasicNavigator()
    goal_poses = []
    index_goals = 0
    for pose in targets:

        if index_goals % 105 == 0:
            x,y,z,yaw = pose
            pose_stamped = create_pose_stamped(nav,x,y,z)
            goal_poses.append(pose_stamped)
            index_goals += 1
        else:
            index_goals += 1 

    return goal_poses

def main():
    #--Init
    rclpy.init()
    nav = BasicNavigator()
    targets = csv_to_array('/home/riser14/Test/odom_true_path.csv')
    print('Waypoint following has started')
    waypoints = create_goals(targets)

    #---Set Intial Pose
    initial_pose = create_pose_stamped(nav,0.0,0.0,0.0)

    #--Wait for Nav2
    nav.waitUntilNav2Active()

    #--Follow Waypoints
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        #print(feedback)

    #--Shutdown
    rclpy.shutdown()


if __name__ == '__main__':
    main()