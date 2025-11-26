#!/usr/bin/env python3
"""
Waypoint Follower Node

Navigates through a predefined set of waypoints using Nav2's FollowWaypoints action.
The robot will visit each waypoint sequentially and return to the start.

Usage:
    ros2 run [package] waypoint_follower.py

Or run as a script:
    python3 waypoint_follower.py

Prerequisites:
    - Simulation running (simulation.launch.py)
    - Nav2 stack active (nav2.launch.py)
    - Robot pose initialized (/initialpose published)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


class WaypointFollower(Node):
    """
    Node that commands the robot to follow a sequence of waypoints.
    """

    def __init__(self):
        super().__init__('waypoint_follower')

        self.get_logger().info('Initializing Waypoint Follower...')

        # Action client for waypoint following
        self._action_client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints'
        )

        # Wait for action server
        self.get_logger().info('Waiting for follow_waypoints action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available!')

        # Define waypoints (x, y, orientation_z, orientation_w)
        # These are example waypoints in the landmark world
        self.waypoints = [
            (-1.0, 1.0, 0.0, 1.0),   # Waypoint 1
            (0.5, 1.5, 0.707, 0.707), # Waypoint 2 (45 degrees)
            (1.0, 0.0, 1.0, 0.0),     # Waypoint 3 (180 degrees)
            (0.0, -1.0, -0.707, 0.707), # Waypoint 4 (-90 degrees)
            (-2.0, 2.0, 0.0, 1.0),    # Return to start
        ]

        self._goal_handle = None
        self._current_waypoint = 0

    def create_waypoint_pose(self, x, y, qz, qw):
        """
        Create a PoseStamped message for a waypoint.

        Args:
            x: X position in meters
            y: Y position in meters
            qz: Quaternion z component for orientation
            qw: Quaternion w component for orientation

        Returns:
            PoseStamped message
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def send_waypoints(self):
        """
        Send all waypoints to the Nav2 waypoint follower.
        """
        goal_msg = FollowWaypoints.Goal()

        # Create list of waypoint poses
        for i, (x, y, qz, qw) in enumerate(self.waypoints):
            pose = self.create_waypoint_pose(x, y, qz, qw)
            goal_msg.poses.append(pose)
            self.get_logger().info(
                f'Waypoint {i+1}: x={x:.2f}, y={y:.2f}, '
                f'orientation=({qz:.2f}, {qw:.2f})'
            )

        self.get_logger().info(
            f'Sending {len(self.waypoints)} waypoints to follow_waypoints action...'
        )

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle response from action server after sending goal.
        """
        self._goal_handle = future.result()

        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was rejected by action server!')
            return

        self.get_logger().info('Goal accepted! Robot will now follow waypoints...')

        # Get result
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback during waypoint following.
        """
        feedback = feedback_msg.feedback
        current_wp = feedback.current_waypoint

        if current_wp != self._current_waypoint:
            self._current_waypoint = current_wp
            self.get_logger().info(
                f'Navigating to waypoint {current_wp + 1}/{len(self.waypoints)}'
            )

    def get_result_callback(self, future):
        """
        Handle final result after all waypoints completed.
        """
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'SUCCESS! Completed all {len(self.waypoints)} waypoints!'
            )
            self.get_logger().info(f'Missed waypoints: {result.missed_waypoints}')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(
                f'Waypoint following ABORTED after {self._current_waypoint} waypoints'
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Waypoint following was CANCELED')
        else:
            self.get_logger().error(f'Waypoint following failed with status: {status}')

        # Shutdown after completion
        self.get_logger().info('Shutting down waypoint follower node...')
        rclpy.shutdown()


def main(args=None):
    """
    Main entry point for waypoint follower node.
    """
    rclpy.init(args=args)

    waypoint_follower = WaypointFollower()

    try:
        # Send waypoints
        waypoint_follower.send_waypoints()

        # Spin to process callbacks
        rclpy.spin(waypoint_follower)

    except KeyboardInterrupt:
        waypoint_follower.get_logger().info('Waypoint following interrupted by user')
    finally:
        waypoint_follower.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
