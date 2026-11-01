#!/usr/bin/env python3
"""Action client that drives the robot around a square of a given side length.

Sends 4 sequential pairs of actions:
1. /drive_distance (lab_interfaces/action/DriveDistance) for side_length
2. /rotate (my_robot_interfaces/action/Rotate) for pi/2 rad (quarter turn left)
"""

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):
    """Coordinates driving a square trajectory using action servers."""

    def __init__(self):
        super().__init__("square_driver")

        self.declare_parameter("side_length", 0.5)  # m

        self.drive_client = ActionClient(self, DriveDistance, "drive_distance")
        self.rotate_client = ActionClient(self, Rotate, "rotate")

    def drive_distance_sync(self, distance: float, max_retries: int = 5) -> bool:
        """Send a DriveDistance goal and wait synchronously for result."""
        for attempt in range(max_retries):
            goal = DriveDistance.Goal()
            goal.distance = float(distance)

            send_future = self.drive_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()

            if not goal_handle.accepted:
                self.get_logger().warn(f"DriveDistance goal rejected (attempt {attempt + 1}/{max_retries}). Retrying in 1s...")
                time.sleep(1.0)
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            wrapped_result = result_future.result()

            if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"Drive succeeded: travelled {wrapped_result.result.travelled:.2f} m")
                return True
            else:
                self.get_logger().error(f"Drive ended with status: {wrapped_result.status}")
                return False
        return False

    def rotate_sync(self, angle: float, max_retries: int = 5) -> bool:
        """Send a Rotate goal and wait synchronously for result."""
        for attempt in range(max_retries):
            goal = Rotate.Goal()
            goal.angle = float(angle)

            send_future = self.rotate_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()

            if not goal_handle.accepted:
                self.get_logger().warn(f"Rotate goal rejected (attempt {attempt + 1}/{max_retries}). Retrying in 1s...")
                time.sleep(1.0)
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            wrapped_result = result_future.result()

            if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"Rotate succeeded: turned {wrapped_result.result.turned:.2f} rad")
                return True
            else:
                self.get_logger().error(f"Rotate ended with status: {wrapped_result.status}")
                return False
        return False

    def execute_square(self):
        """Execute the 4-sided square driving pattern."""
        side_length = float(self.get_parameter("side_length").value)
        quarter_turn = math.pi / 2.0  # 1.5708 rad (left turn)

        self.get_logger().info(f"Starting square drive routine with side_length={side_length:.2f} m")
        self.get_logger().info("Waiting for action servers (/drive_distance and /rotate)...")

        if not self.drive_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Action server /drive_distance unavailable. Exiting.")
            return False

        if not self.rotate_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Action server /rotate unavailable. Exiting.")
            return False

        self.get_logger().info("Action servers available. Beginning square sides...")

        for side in range(1, 5):
            self.get_logger().info(f">>> Side {side}/4: Driving forward {side_length:.2f} m <<<")
            if not self.drive_distance_sync(side_length):
                self.get_logger().error(f"Failed while driving forward on side {side}/4. Aborting square.")
                return False

            self.get_logger().info(f">>> Side {side}/4: Turning quarter turn left ({quarter_turn:.3f} rad) <<<")
            if not self.rotate_sync(quarter_turn):
                self.get_logger().error(f"Failed while rotating on side {side}/4. Aborting square.")
                return False

        self.get_logger().info("All 4 sides completed successfully! Robot back at starting pose.")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        node.execute_square()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
