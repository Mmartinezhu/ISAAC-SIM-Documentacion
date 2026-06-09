#!/usr/bin/env python3

import signal
import subprocess

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy


class RobotTeleop(Node):
    def __init__(self):
        super().__init__("robot_teleop")

        self.cmd_pub = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        self.max_speed = 2.0
        self.max_steering_angle = 0.4
        self.max_acceleration = 1.0
        self.max_steering_rate = 0.5

        self.get_logger().info("Teleop node started")

    def joy_callback(self, msg: Joy):
        speed = msg.axes[1] * self.max_speed
        steering_angle = msg.axes[3] * self.max_steering_angle

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.acceleration = self.max_acceleration
        drive_msg.drive.steering_angle_velocity = self.max_steering_rate

        self.cmd_pub.publish(drive_msg)


def main():
    rclpy.init()

    joy_process = subprocess.Popen(
        ["ros2", "run", "joy", "joy_node"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    print("Started joystick driver (joy_node)")

    node = RobotTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down...")
        node.destroy_node()
        rclpy.shutdown()

        joy_process.send_signal(signal.SIGINT)
        joy_process.wait()


if __name__ == "__main__":
    main()
