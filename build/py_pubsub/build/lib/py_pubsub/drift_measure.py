import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import csv

import tf_transformations as tf


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.index = -0.1

        self.csv_file = open('odom_drift_True.csv',mode = 'w',newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['X','Y','Z','x','y','z','w'])

    def odom_callback(self,msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        o_x = msg.pose.pose.orientation.x
        o_y = msg.pose.pose.orientation.y
        o_z = msg.pose.pose.orientation.z
        o_w = msg.pose.pose.orientation.w
        self.index += 0.1

        ori_list = [o_x,o_y,o_z,o_w]
        roll,pitch,yaw = tf.euler_from_quaternion(ori_list)
        self.get_logger().info(f'Odometry Data - Orientation=> Index: {self.index} ,W: {yaw}')

        self.csv_writer.writerow([yaw])


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
