#!/usr/bin/env python3
"""
Action server that rotates the robot on the spot to a desired angle.

Serves /rotate (my_robot_interfaces/action/Rotate).
Uses wheel odometry (/odom) yaw integration to track angular displacement.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from my_robot_interfaces.action import Rotate

# Helper functions adapted from diffdrive_bot/scripts/drive_distance_server.py


def yaw_from_quaternion(q):
    """Rotation about the vertical z-axis in radians of an orientation quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_difference(target, current):
    """Compute target - current, wrapped to [-pi, pi] to handle angle wrapping cleanly."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


class RotateServer(Node):
    """Action server that rotates the robot to a given angle."""

    def __init__(self):
        """Initialize rotate_server node and register action server."""
        super().__init__("rotate_server")

        self.declare_parameter("turn_speed", 0.5)  # rad/s

        self.yaw = None

        # ReentrantCallbackGroup allows callbacks (odom, action) to execute concurrently
        # on different threads in the MultiThreadedExecutor
        group = ReentrantCallbackGroup()

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10, callback_group=group)

        self.action_server = ActionServer(
            self,
            Rotate,
            "rotate",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=group,
        )

        self.get_logger().info("rotate_server action server ready on /rotate")

    def on_odom(self, msg: Odometry):
        """Update current orientation from odometry message."""
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_goal(self, goal):
        """Accept or reject goal: reject zero turn or if odometry is not yet received."""
        if abs(goal.angle) < 1e-4:
            self.get_logger().warn("Rejecting rotate goal: angle is zero")
            return GoalResponse.REJECT
        if self.yaw is None:
            self.get_logger().warn("Rejecting rotate goal: odometry not yet received")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        """Execute rotation by publishing angular velocity and accumulating yaw delta."""
        target_angle = goal_handle.request.angle
        turn_speed = float(self.get_parameter("turn_speed").value)

        self.get_logger().info(
            f"Executing rotate goal: {target_angle:.3f} rad at {turn_speed:.2f} rad/s"
        )

        last_yaw = self.yaw
        total_turned = 0.0

        direction = 1.0 if target_angle > 0.0 else -1.0
        twist = Twist()
        twist.angular.z = direction * turn_speed

        feedback = Rotate.Feedback()
        result = Rotate.Result()

        rate_sleep = 0.05  # 20 Hz update loop

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.cmd_vel_pub.publish(Twist())
                goal_handle.canceled()
                result.turned = total_turned
                self.get_logger().info(f"Rotate canceled after turning {total_turned:.3f} rad")
                return result

            # Integrate small change in yaw since last iteration
            current_yaw = self.yaw
            delta = angle_difference(current_yaw, last_yaw)
            last_yaw = current_yaw
            total_turned += delta

            # Publish feedback
            feedback.turned = total_turned
            goal_handle.publish_feedback(feedback)

            # Check if target reached
            if direction > 0.0 and total_turned >= target_angle:
                break
            if direction < 0.0 and total_turned <= target_angle:
                break

            self.cmd_vel_pub.publish(twist)
            time.sleep(rate_sleep)

        # Stop robot
        self.cmd_vel_pub.publish(Twist())
        goal_handle.succeed()
        result.turned = total_turned
        self.get_logger().info(
            f"Rotate completed: turned {total_turned:.3f} rad (target was {target_angle:.3f} rad)"
        )
        return result


def main(args=None):
    """Run rotate_server node."""
    rclpy.init(args=args)
    node = RotateServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
