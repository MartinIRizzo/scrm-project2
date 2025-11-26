#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber to odometry to get current position
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Define points (A, B, C, D)
        self.positions = [(0, 0), (2, 0), (2, 2), (0, 2)]
        self.current_index = 0

        self.position = (0, 0)  # Current robot position
        self.yaw = 0.0  # Current orientation (yaw)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.move_to_point)  # 10 Hz

        self.get_logger().info("Robot mover node started")

    def odom_callback(self, msg):
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def move_to_point(self):
        target = self.positions[self.current_index]
        x, y = self.position
        target_x, target_y = target

        # Compute distance and angle to target
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_diff = angle_to_target - self.yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # normalize

        # Control law
        twist = Twist()

        # Linear speed proportional to distance (max 0.5 m/s)
        twist.linear.x = min(0.5, distance)

        # Angular speed proportional to angle difference (max 1 rad/s)
        twist.angular.z = max(-1.0, min(1.0, angle_diff))

        # If close to target, go to next point
        if distance < 0.1:
            self.current_index = (self.current_index + 1) % len(self.positions)
            self.get_logger().info(f"Reached point {self.current_index}, moving to next")

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
