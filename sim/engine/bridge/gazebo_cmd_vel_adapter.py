#!/usr/bin/env python3
"""Convert LingTu TwistStamped commands into Gazebo Twist commands.

LingTu uses `/nav/cmd_vel` as `geometry_msgs/TwistStamped` so command frames
and timestamps remain explicit. ros_gz_bridge commonly exposes Gazebo velocity
controllers as `geometry_msgs/Twist`, so this adapter is the narrow conversion
boundary for Gazebo simulation.
"""

from __future__ import annotations


def main() -> None:
    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = Node("lingtu_gazebo_cmd_vel_adapter")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(Twist, "/lingtu/gazebo/cmd_vel", qos)

    def on_cmd(msg: TwistStamped) -> None:
        twist = Twist()
        twist.linear = msg.twist.linear
        twist.angular = msg.twist.angular
        pub.publish(twist)

    node.create_subscription(TwistStamped, "/nav/cmd_vel", on_cmd, qos)
    node.get_logger().info(
        "Gazebo cmd_vel adapter: /nav/cmd_vel TwistStamped -> "
        "/lingtu/gazebo/cmd_vel Twist"
    )

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
