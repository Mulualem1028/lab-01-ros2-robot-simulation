#!/usr/bin/env python3
"""Node that steers the robot away from obstacles using lidar-derived obstacle info.

Subscribes to /obstacle (lab_interfaces/msg/ObstacleInfo) and publishes to
/cmd_vel (geometry_msgs/msg/Twist) and /robot_status (my_robot_interfaces/msg/RobotStatus).
Controlled via services /turn_away_enable and /set_turn_speed.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed


class TurnAway(Node):
    """Reacts to detected obstacles by turning away or driving forward."""

    def __init__(self):
        super().__init__("turn_away")

        # Declare parameters with default values
        self.declare_parameter("forward_speed", 0.2)  # m/s
        self.declare_parameter("turn_speed", 0.5)     # rad/s
        self.declare_parameter("max_turn_speed", 1.5) # rad/s

        # Node starts disabled as required by Step 5
        self.enabled = False

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(RobotStatus, "robot_status", 10)

        # Subscriber
        self.create_subscription(ObstacleInfo, "obstacle", self.on_obstacle, 10)

        # Services
        self.create_service(SetBool, "turn_away_enable", self.on_enable)
        self.create_service(SetTurnSpeed, "set_turn_speed", self.on_set_turn_speed)

        self.get_logger().info("turn_away node initialized (currently disabled)")

    def on_obstacle(self, msg: ObstacleInfo):
        """Callback executed on each incoming ObstacleInfo message."""
        status = RobotStatus()
        status.obstacle_distance = float(msg.distance)

        # While disabled, publish no velocities and set state to 'stopped'
        if not self.enabled:
            status.state = "stopped"
            self.status_pub.publish(status)
            return

        # Dynamically read parameters so changes via 'ros2 param set' take immediate effect
        forward_speed = float(self.get_parameter("forward_speed").value)
        turn_speed = float(self.get_parameter("turn_speed").value)

        twist = Twist()
        if msg.too_close:
            status.state = "turning"
            twist.linear.x = 0.0
            # Obstacle to the left (angle > 0) -> turn right (negative angular.z)
            # Otherwise turn left (positive angular.z)
            if msg.angle > 0.0:
                twist.angular.z = -turn_speed
            else:
                twist.angular.z = turn_speed
        else:
            status.state = "forward"
            twist.linear.x = forward_speed
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)
        self.status_pub.publish(status)

    def on_enable(self, request, response):
        """Enable or disable the turn_away controller."""
        if request.data:
            self.enabled = True
            response.success = True
            response.message = "turn_away enabled"
        else:
            self.stop()
            response.success = True
            response.message = "turn_away disabled"
        self.get_logger().info(response.message)
        return response

    def on_set_turn_speed(self, request, response):
        """Service to set the turn speed within allowable bounds."""
        max_turn_speed = float(self.get_parameter("max_turn_speed").value)
        if request.turn_speed <= 0.0 or request.turn_speed > max_turn_speed:
            response.success = False
            response.message = (
                f"turn_speed must be > 0 and <= {max_turn_speed:.2f} rad/s "
                f"(received {request.turn_speed:.2f})"
            )
        else:
            self.set_parameters([Parameter("turn_speed", value=float(request.turn_speed))])
            response.success = True
            response.message = f"turn_speed set to {request.turn_speed:.2f} rad/s"
        self.get_logger().info(response.message)
        return response

    def stop(self):
        """Disable driving and publish a single zero Twist to stop motors."""
        self.enabled = False
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TurnAway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
