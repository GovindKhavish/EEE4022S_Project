import rclpy
import casadi as ca
from rclpy.node import Node

from std_msgs.msg import String

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import geometry_msgs.msg as geo
import csv

from std_msgs.msg import Float64MultiArray
from casadi import *
import tf_transformations as tf
import math

class PID_Control(Node):

    def __init__(self):
        super().__init__('pid_control')
        self.pose_ = Odometry()
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # PID constants for angular velocity (tune as needed)
        self.kp_angular = 0.001
        self.ki_angular = 0.003
        self.kd_angular = 0.001

        # PID constants for linear velocity (tune as needed)
        self.kp_linear = 0.005
        self.ki_linear = 0.005
        self.kd_linear = 0.001

        self.previous_error_angular = 0.0
        self.integral_angular = 0.0

        self.previous_error_linear = 0.0
        self.integral_linear = 0.0

        self.current_waypoint = 0

        #For loop to run through entire path
        self.index, self.targets = self.csv_to_path()

    def pose_callback(self,msg: Odometry):
        self.pose_ = msg
        self.x_ = self.pose_.pose.pose.position.x
        self.y_ = self.pose_.pose.pose.position.y 
        self.z_ = self.pose_.pose.pose.position.z
        self.o_x = self.pose_.pose.pose.orientation.x
        self.o_y = self.pose_.pose.pose.orientation.y
        self.o_z = self.pose_.pose.pose.orientation.z
        self.o_w = self.pose_.pose.pose.orientation.w

        yaw = self.getYaw()

        current_waypoint = self.targets[self.current_waypoint]
        #self.get_logger().info(f"Current waypoint => {current_waypoint}")

        # Calculate error terms
        error_x = current_waypoint[0] - self.x_
        error_y = current_waypoint[1] - self.y_
        error_yaw = math.atan2(error_y, error_x) - np.round(yaw,3)
            # Ensure that error_yaw is in the range -π to π
        if error_yaw > math.pi:
            error_yaw -= 2 * math.pi
        elif error_yaw < -math.pi:
            error_yaw += 2 * math.pi
        #self.get_logger().info(f"Yaw => {yaw}")

        control_angular = (
            self.kp_angular * error_yaw +
            self.ki_angular * self.integral_angular +
            self.kd_angular * (error_yaw - self.previous_error_angular)
        )
        # Calculate error terms for linear control (distance to the waypoint)
        distance_to_waypoint = math.sqrt(error_x **2 + error_y **2)

        # Implement the PID controller for linear control
        control_output_linear = (
            self.kp_linear * distance_to_waypoint +
            self.ki_linear * self.integral_linear +
            self.kd_linear * (distance_to_waypoint - self.previous_error_linear)
        )

        # Publish control commands to move the robot
        #self.get_logger().info(f"Linear: {control_output_linear}")
        #self.get_logger().info(f"Angular: {control_angular}")
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = control_angular  # Angular velocity control
        cmd_vel_msg.linear.x = control_output_linear  # Linear velocity control
        self.publisher_.publish(cmd_vel_msg)

        # Update error and integral terms for angular control
        self.previous_error_angular = error_yaw
        self.integral_angular += error_yaw

        # Update error and integral terms for linear control
        self.previous_error_linear = distance_to_waypoint
        self.integral_linear += distance_to_waypoint

        # Check if the current waypoint is reached
        if distance_to_waypoint < 0.1:
            self.current_waypoint += 1
            self.previous_error_angular = 0.0
            self.integral_angular = 0.0
            self.previous_error_linear = 0.0
            self.integral_linear = 0.0
            #self.get_logger().info("Next Waypoint------------------------------------")

        # Check if all waypoints are reached
        if self.current_waypoint >= self.index:
            self.get_logger().info("All waypoints reached. Stopping the robot.")
            self.destroy_node()
            rclpy.shutdown()

#Helper functions
    def getYaw(self):
        euler_angles = tf.euler_from_quaternion([
        self.o_x,
        self.o_y,
        self.o_z,
        self.o_w
        ])
        yaw = euler_angles[2]
        return yaw

    def csv_to_path(self):
        index_reader_1 = 0
        index = 0
        targets = []
        with open('/home/riser14/Test/odom_true_path.csv','r') as file:
            reader = csv.reader(file)
            next(reader) #Skips header

            for row in reader:
                if index_reader_1 % 105 == 0:
                    x,y,z,o_x,o_y,o_z,o_w = map(float,row)
                    targets.append((np.round(x,2),np.round(y,2)))
                    index_reader_1 +=1
                    index += 1
                else:
                    index_reader_1 +=1

            traj = np.array(targets)
        return index, traj
    
def main(args=None):

    rclpy.init(args=args)
    #nav = BasicNavigator()
    #---Set Intial Pose
    #initial_pose = create_pose_stamped(nav,0.0,0.0,0.0)
    pid_control = PID_Control()
    rclpy.spin(pid_control)
    pid_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



