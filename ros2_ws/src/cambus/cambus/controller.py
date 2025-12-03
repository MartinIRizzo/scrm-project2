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
            (-6.7, -27.2, True),
            (-10.47, -27.35, True),
            (-41.45, -27.35, True),
            (-41.45, -14.27, True),
            (-14.16, -14.27, True),
            (6.86, -14.27, True),
            (7.42, -1.86, True),
            (-5.58, -1.86, True),
            (-19.03, -1.86, True),
            (-19.03, 5.69, True),
            (-19.03, 11.071, True),
            (1.30, 11.071, True),
            (8.30, 11.071, True),
            (7.55, .42, True),
            (7.35, -14.16, True),
        ]
        self.current_index = 0

        # Speed zone radii (symmetric profile)
        self.slow_radius = 1.0      # R_SLOW    → min speed zone

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Speed parameters
        self.actual_speed = 0.0
        self.max_speed = 0.8    # MAX
        self.min_speed = 0.1    # MIN
        self.accel = 0.02
        self.decel = 0.02

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
        target_y, target_x, stop_flag = self.positions[self.current_index]

        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        twist = Twist()

        # -----------------------------------------
        # SYMMETRIC SPEED PROFILE
        # -----------------------------------------
        print(f'{distance} <= {self.slow_radius}')

        # A [---]---[---] B

        absolute_dx = self.positions[(self.current_index - 1 + len(self.positions))%len(self.positions)][1] - self.positions[self.current_index][1]
        absolute_dy = self.positions[(self.current_index - 1 + len(self.positions))%len(self.positions)][0] - self.positions[self.current_index][0]
        absolute_distance = math.hypot(absolute_dx, absolute_dy)

        if stop_flag and (distance <= self.slow_radius or absolute_distance - distance <= self.slow_radius):
            # MIN SPEED zone near goal
            desired_speed = self.min_speed

        else:
            # MAX SPEED zone far from goal
            desired_speed = self.max_speed
            # linear acceleration/deceleration zone
            # scale between min_speed → max_speed
            # ratio = (distance - self.slow_radius) / (self.fast_radius - self.slow_radius)
            # desired_speed = self.min_speed + ratio * (self.max_speed - self.min_speed)

        print(f'desired: {desired_speed}')

        # -----------------------------------------
        # Smooth speed change (accel & decel)
        # -----------------------------------------
        if self.actual_speed < desired_speed:
            self.actual_speed = min(self.actual_speed + self.accel, desired_speed)
        else:
            self.actual_speed = max(self.actual_speed - self.decel, desired_speed)

        twist.linear.x = self.actual_speed

        # -----------------------------------------
        # Angular smoothing
        # -----------------------------------------
        twist.angular.z = max(-0.8, min(0.8, angle_diff * 1.2))

        # -----------------------------------------
        # Check target reached
        # -----------------------------------------
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
