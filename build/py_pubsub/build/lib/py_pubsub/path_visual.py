import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion
import csv

class PathVisualizationNode(Node):
    def __init__(self):
        super().__init__('path_visualization_node')
        self.publisher = self.create_publisher(MarkerArray, 'path_markers', 10)
        timer_period = 1  # Adjust as needed (1 Hz in this example)
        self.timer = self.create_timer(timer_period, self.publish_markers)
        
        # Specify the full path to your CSV file
        self.path_data = self.read_path_csv('/home/riser14/ros_ws/odom_true_path.csv')

    def read_path_csv(self, file_path):
        path_data = []
        with open(file_path, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                position = Pose()
                position.position.x = float(row['X'])
                position.position.y = float(row['Y'])
                position.position.z = float(row['Z'])
                position.orientation.x = float(row['x'])
                position.orientation.y = float(row['y'])
                position.orientation.z = float(row['z'])
                position.orientation.w = float(row['w'])
                path_data.append(position)
        return path_data

    def publish_markers(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'base_footprint'  # Adjust to match TurtleBot3's frame
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.pose.orientation.w = 1.0

        for pose in self.path_data:
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = pose.position.z
            marker.points.append(point)

        marker_array.markers.append(marker)
        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()