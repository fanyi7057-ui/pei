#!/usr/bin/env python3
"""Publish only a stationary standing posture; never start race logic."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from stand_only_core import StandCommand


class WheellegStandOnly(Node):
    """Continuously command zero wheel velocity and a safe standing posture."""

    def __init__(self) -> None:
        super().__init__("wheelleg_stand_only")
        self.declare_parameter("stand_height_m", 0.20)
        self.declare_parameter("stand_roll_deg", 0.0)
        self.declare_parameter("stand_pitch_deg", 25.0)
        self.declare_parameter("stand_slide_m", 0.0)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.command = StandCommand.from_posture(
            self.get_parameter("stand_height_m").value,
            self.get_parameter("stand_roll_deg").value,
            self.get_parameter("stand_pitch_deg").value,
            self.get_parameter("stand_slide_m").value,
        )
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.publish_rate_hz = max(5.0, min(50.0, rate_hz))

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.posture_pub = self.create_publisher(JointState, "/cmd_posture", 10)
        self.jump_pub = self.create_publisher(Bool, "/cmd_jump", 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_safe_stand)

        self.publish_safe_stand()
        self.get_logger().warn(
            "仅比赛姿态原地站立：零轮速、无巡线、无跳跃；按 Ctrl+C 返回菜单。"
        )

    def publish_safe_stand(self) -> None:
        """Publish every output required to keep the robot stationary."""
        velocity = Twist()
        velocity.linear.x = self.command.linear_x
        velocity.angular.z = self.command.angular_z
        self.cmd_vel_pub.publish(velocity)

        posture = JointState()
        posture.header.stamp = self.get_clock().now().to_msg()
        posture.name = list(self.command.joint_names)
        posture.position = list(self.command.joint_positions)
        self.posture_pub.publish(posture)

        jump = Bool()
        jump.data = self.command.jump
        self.jump_pub.publish(jump)

    def close(self) -> None:
        """Repeat safe commands briefly while ROS publishers are still alive."""
        for _ in range(10):
            try:
                self.publish_safe_stand()
            except Exception as error:
                try:
                    self.get_logger().warn(
                        f"清理时安全站立消息发送失败: {error}"
                    )
                except Exception:
                    pass
            time.sleep(0.02)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheellegStandOnly()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
