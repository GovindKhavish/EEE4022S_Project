#!/usr/bin/env python3

import os
import cv2
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class ImageSubscriber:

    def __init__(self):
        self.node = rclpy.create_node('image_subscriber')
        self.image_topic = "/camera/image_raw"  # Change this topic to match your camera topic
        self.subscription = self.node.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10  # Adjust the queue size as needed
        )
        self.bridge = CvBridge()
        self.reference_image = cv2.imread("reference_image.jpg")  # Load the reference image
        self.error_threshold = 10  # Define your error threshold here

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Calculate the Mean Absolute Error (MAE) between the images
            error = np.abs(cv_image - self.reference_image).mean()

            print(f"MAE Error: {error}")

            # Check if the error exceeds the threshold
            if error > self.error_threshold:
                print("Error exceeds the threshold!")

        except Exception as e:
            print(f"Error processing image: {str(e)}")

    def run(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            print("Shutting down")

        self.node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    image_subscriber.run()

if __name__ == '__main__':
    main()
