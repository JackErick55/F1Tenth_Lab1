#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class TalkerNode(Node):
    def __init__(self):
        super().__init__("talker")

        # Declare ROS parameters (v and d)
        self.declare_parameter("v", 1.0)  # Default velocity
        self.declare_parameter("d", 0.0)  # Default steering angle

        # Create a publisher for AckermannDriveStamped messages
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "drive", 10)

        # Publish as fast as possible (no timer)
        self.publish_message()

    def publish_message(self):
        while rclpy.ok():
            # Get parameters
            v = self.get_parameter("v").get_parameter_value().double_value
            d = self.get_parameter("d").get_parameter_value().double_value

            # Create and populate the message
            msg = AckermannDriveStamped()
            msg.drive.speed = v
            msg.drive.steering_angle = d

            # Publish the message
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing: Speed={v}, Steering Angle={d}")

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
