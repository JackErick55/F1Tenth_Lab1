#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class RelayNode(Node):
    def __init__(self):
        super().__init__("relay")

        # Create a subscriber to listen to /drive topic
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            "drive",
            self.listener_callback,
            10
        )

        # Create a publisher for modified messages
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "drive_relay", 10)

    def listener_callback(self, msg):
        # Modify the received message
        new_msg = AckermannDriveStamped()
        new_msg.drive.speed = msg.drive.speed * 3
        new_msg.drive.steering_angle = msg.drive.steering_angle * 3

        # Publish modified message
        self.publisher_.publish(new_msg)
        self.get_logger().info(f"Relayed: Speed={new_msg.drive.speed}, Steering Angle={new_msg.drive.steering_angle}")

def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
