#!/usr/bin/env python3

import os
import cv2
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt

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
        #self.reference_image = cv2.imread("reference_image.jpg", cv2.IMREAD_GRAYSCALE)  # Load the reference image in grayscale
        self.reference_image_dir = "/home/riser14/ros_ws/True_path_images"  # Directory containing reference images
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)  # Feature matcher
        self.match_threshold = 50  # Adjust the threshold for matches
        self.window_name = "Image Comparison"
        self.timer_period = 5.0  # Timer period in seconds
        self.timer = self.node.create_timer(self.timer_period, self.timer_callback)
        self.process_image = False  # Flag to control image processing
        self.current_reference_index = 0  # Index of the currently loaded reference image
        self.reference_images = self.load_reference_images()  # Load all reference images
        self.reference_image = self.reference_images[0]  # Initialize reference_image with the first image
        self.correct_matches = []  # List to store the number of correct matches
        self.match_percentage = []  # List to store the match percentage
        self.fig, self.ax = plt.subplots()  # Create a figure and axis for the graph
        self.figure_saved = False  # Flag to track if the figure has been saved


    def load_reference_images(self):
        # List all files in the reference image directory
        reference_image_files = os.listdir(self.reference_image_dir)
        # Filter only image files (you may need to adjust this based on your image file extensions)
        reference_image_files = [f for f in reference_image_files if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
        # Sort the image files to load them in a specific order
        reference_image_files.sort()
        
        reference_images = []
        for file in reference_image_files:
            # Load each reference image and add it to the list
            reference_image_path = os.path.join(self.reference_image_dir, file)
            reference_image = cv2.imread(reference_image_path, cv2.IMREAD_GRAYSCALE)
            reference_images.append(reference_image)

        return reference_images

    def image_callback(self, msg):
        if self.process_image:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

                # Detect ORB features
                orb = cv2.ORB_create()
                kp1, des1 = orb.detectAndCompute(self.reference_image, None)
                kp2, des2 = orb.detectAndCompute(gray_image, None)

                # Match features using the BFMatcher
                matches = self.matcher.match(des1, des2)
                matches = sorted(matches, key=lambda x: x.distance)

                # Filter matches by threshold
                good_matches = [m for m in matches if m.distance < self.match_threshold]

                # Draw matches on the image
                img_matches = cv2.drawMatches(self.reference_image, kp1, gray_image, kp2, good_matches, None)

                # Add headings for reference frame and subscribed frame
                heading_reference = "Reference Frame"
                heading_subscribed = "Subscribed Frame"
                cv2.putText(img_matches, heading_reference, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(img_matches, heading_subscribed, (800, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                # Display the result in a smaller window
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)  # Create a resizable window
                cv2.resizeWindow(self.window_name, 1600, 600)  # Set the size of the window (adjust as needed)
                cv2.imshow(self.window_name, img_matches)
                cv2.waitKey(1)

                # Print the number of features matched out of the total number of features
                total_features = len(kp1)
                matched_features = len(good_matches)
                print(f"Matched Features: {matched_features}/{total_features}")

                 # Calculate match percentage and append to the list
                match_percentage = (matched_features/250)*100
                print(f"Match percentage out of 250: {match_percentage}")
                self.match_percentage.append(match_percentage)

                # Update the correct matches list
                self.correct_matches.append(matched_features)

                if matched_features > 250:
                    self.current_reference_index = (self.current_reference_index + 1) % len(self.reference_images)
                    self.reference_image = self.reference_images[self.current_reference_index]
                    print(f"Loading next reference image: {self.current_reference_index + 1}")

                     # Save the current figure as an image file
                    if not self.figure_saved:
                        self.save_figure()
                        self.figure_saved = True  # Set the flag to indicate the figure has been saved

    
                # Plot the match percentage
                self.ax.clear()
                self.ax.plot(self.match_percentage, label='Match Percentage', marker='o')
                self.ax.set_xlabel('Frame Number')
                self.ax.set_ylabel('Match Percentage (%)')
                self.ax.legend(loc='upper right')
                self.fig.canvas.draw()


            except Exception as e:
                print(f"Error processing image: {str(e)}")
        
    def save_figure(self):
        # Define the file name for saving the figure
        figure_filename = f"match_percentage_reference_{self.current_reference_index + 1}.png"
        
        # Save the figure as an image file
        self.fig.savefig(figure_filename)
        print(f"Saved figure as {figure_filename}")

    def timer_callback(self):
        self.process_image = True

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
    plt.show()

if __name__ == '__main__':
    main()

