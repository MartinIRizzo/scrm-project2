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
        self.positions = [
            (0, 0, False),
            (1, 0, True),
            (2, 0, False),
            (1, 0, True),
            (2, 2, False),
            (1, 0, True),
            (0, 2, False),
            (1, 0, True)]
        self.current_index = 0

        # Radius where speed needs to be reduced [position - radius, position + radius]
        self.slowness_radius = 10

        self.position = (0, 0)  # Current robot position
        self.yaw = 0.0  # Current orientation (yaw)

        # Maximum and minimum robot speed and robot acceleration
        self.actual_speed = 0.0 # m/s
        self.max_speed = 100.0 # m/s
        self.min_speed = 10.0 # m/s
        self.acceleration = 0.5 # m/s

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
        dx = abs(target_x - x)
        dy = abs(target_y - y)
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_diff = angle_to_target - self.yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # normalize

        # Control law
        twist = Twist()

        # We get the previous position
        if self.current_index == 0:
            previous = self.position[-1]
        else:
            previous = self.positions[self.current_index - 1]

        absolute_dx = abs(target_x - previous[0])
        absolute_dy = abs(target_y - previous[1])
        absolute_distance = math.hypot(absolute_dx, absolute_dy)

        # Linear speed proportional to distance (max 0.5 m/s)
        speed = self.actual_speed
        if absolute_distance / 2 > distance:
            if speed < self.max_speed:
                speed = min(speed + self.acceleration, self.max_speed)
        else:
            if speed > self.min_speed:
                speed = min(speed - self.acceleration, self.min_speed)

        twist.linear.x = speed

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
