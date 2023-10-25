import rclpy
import casadi as ca
from rclpy.node import Node

from std_msgs.msg import String

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import csv

from std_msgs.msg import Float64MultiArray
from casadi import *
import tf_transformations as tf
import math

class Pose_Subscriber(Node):

    def __init__(self):
        super().__init__('pid_subscriber')
        self.pose_ = PoseWithCovarianceStamped()
        self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def pose_callback(self,msg: PoseWithCovarianceStamped):
        self.pose_ = msg
        self.x_ = self.pose_.pose.pose.position.x
        self.y_ = self.pose_.pose.pose.position.y
        self.z_ = self.pose_.pose.pose.position.z
        self.get_logger().info(f"MSG: {self.x_},||||{self.y_}|||||{self.z_}")
        self.o_x = self.pose_.pose.pose.orientation.x
        self.o_y = self.pose_.pose.pose.orientation.y
        self.o_z = self.pose_.pose.pose.orientation.z
        self.o_w = self.pose_.pose.pose.orientation.w

def create_pose_stamped(navigator:BasicNavigator, position_x, position_y,orientation_z):
        q_x,q_y,q_z,q_w = tf.quaternion_from_euler(0.0, 0.0, orientation_z)
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
    

def main():

    rclpy.init()
    nav = BasicNavigator()
    #---Set Intial Pose
    initial_pose = create_pose_stamped(nav,0.0,0.0,0.0)
    nav.setInitialPose = initial_pose
    nav.waitUntilNav2Active()
    #--Follow 
    nav.goToPose(create_pose_stamped(nav,0.0,1.0,0.0))
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
    
    pose_subscriber = Pose_Subscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
