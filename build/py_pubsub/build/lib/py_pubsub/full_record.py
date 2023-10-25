#!/usr/bin/env python
import os
import cv2
import rclpy
import csv
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')
        self.odom_data = []
        self.bridge = CvBridge()
        self.image = Image()
        self.image_count = 0
        self.image_save_path = "/home/riser14/ros_ws/True_path_images"  # Replace with your image save path
        self.odom_csv_path = "/home/riser14/ros_ws/odom_true_path.csv" # Replace with your CSV file path

        # ROS subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.timer = self.create_timer(5.0, self.save_odom_data_to_csv)
        self.timer = self.create_timer(5.0, self.save_image)

    def odom_callback(self, msg: Odometry):
        # Record /odom data
        self.get_logger().info(msg)
        position = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        self.odom_data.append([position.x, position.y,position.z,orient.x,orient.y,orient.z,orient.w])

    def image_callback(self, msg:Image):
        self.image = msg

    def save_image(self):
        # Save camera image
        cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        image_filename = os.path.join(self.image_save_path, f'image_{self.image_count}.png')
        cv2.imwrite(image_filename, cv_image)
        self.image_count += 1

    def save_odom_data_to_csv(self):
        # Save /odom data to a CSV file
        with open(self.odom_csv_path, 'w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(['X', 'Y','Z','x','y','z','w'])
            for entry in self.odom_data:
                csv_writer.writerow([entry[0], entry[1], entry[2],
                                     entry[3],entry[4],entry[5],entry[6]])

def main():
    rclpy.init()
    data_recorder = DataRecorder()

    try:
        rclpy.spin(data_recorder)
    except KeyboardInterrupt:
        pass

    data_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()