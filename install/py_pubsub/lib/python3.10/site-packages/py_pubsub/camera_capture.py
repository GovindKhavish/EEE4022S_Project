#!/usr/bin/env python3

import os
import cv2
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def image_callback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        file_name = "/home/riser14/Tests_of_Turtle/Test_Images/captured_image.jpg"  # Change the file name as needed
        cv2.imwrite(file_name, cv_image)
        print(f"Image saved to {file_name}")
        rclpy.shutdown()  # Shutdown ROS 2 after saving the image
    except Exception as e:
        print(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('image_subscriber')

    image_topic = "/camera/image_raw"  # Change this topic to match your camera topic
    subscription = node.create_subscription(
        Image,
        image_topic,
        image_callback,
        10  # Adjust the queue size as needed
    )
    subscription  # Prevent unused variable warning

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()