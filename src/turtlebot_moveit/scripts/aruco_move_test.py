#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist

class ArucoMarkerListener(Node):
    def __init__(self):
        super().__init__('aruco_marker_listener')
        self.subscription = self.create_subscription(
            MarkerArray,
            'detected_markers',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)

    def listener_callback(self, msg):
        target_marker_id = 0  # Change this to the desired marker ID
        for marker in msg.markers:
            if marker.id == target_marker_id:
                self.get_logger().info(f'Marker ID: {marker.id}, PositionZ: {marker.pose.pose.position.z}')
                if marker.pose.pose.position.z > 0.13:
                    self.publish_cmd_vel(0.15)
                else:
                    self.publish_cmd_vel(0.0)
                break
            
    def publish_cmd_vel(self, linear_x, angular_z=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist)      

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()