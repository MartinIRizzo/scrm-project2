#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # (x, y, stop_flag)
        self.positions = [
            (0, 0, False),
            (1, 0, True),
            (2, 0, False),
            (1, 0, True),
            (2, 2, False),
            (1, 0, True),
            (0, 2, False),
            (1, 0, True)
        ]
        self.current_index = 0

        self.slowness_radius = 0.6  # radius to start slowing down

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Speed parameters (REALISTIC)
        self.actual_speed = 0.0
        self.max_speed = 0.4      # m/s
        self.min_speed = 0.07     # m/s
        self.accel = 0.05         # acceleration per cycle
        self.decel = 0.05         # deceleration per cycle

        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info("Smooth Robot Mover started")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def update(self):
        target_x, target_y, stop_flag = self.positions[self.current_index]

        dx = target_x - self.x
        dy = target_y - self.y

        distance = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # normalize

        twist = Twist()

        # --------------------------
        # SMOOTH ACCELERATION MODEL
        # --------------------------

        if distance < self.slowness_radius:
            # Smoothly slow down when near the target
            desired_speed = self.min_speed
        else:
            # Accelerate normally
            desired_speed = self.max_speed

        # Stop when target requires stopping
        if stop_flag and distance < self.slowness_radius:
            desired_speed = 0.0

        # Smooth speed approach
        if self.actual_speed < desired_speed:
            self.actual_speed = min(self.actual_speed + self.accel, desired_speed)
        else:
            self.actual_speed = max(self.actual_speed - self.decel, desired_speed)

        twist.linear.x = self.actual_speed

        # Angular speed with smoothing
        twist.angular.z = max(-0.8, min(0.8, angle_diff * 1.2))

        # Check target reached
        if distance < 0.08:
            self.current_index = (self.current_index + 1) % len(self.positions)
            self.get_logger().info(f"Reached point {self.current_index}")

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
