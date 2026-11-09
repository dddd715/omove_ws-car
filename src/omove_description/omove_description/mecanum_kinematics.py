#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class MecanumKinematics(Node):
    def __init__(self):
        super().__init__('mecanum_kinematics')
        self.r = self.declare_parameter('wheel_radius', 0.076).value
        self.Lx = self.declare_parameter('half_wheelbase', 0.18).value
        self.Ly = self.declare_parameter('half_track', 0.20).value
        self.timeout = self.declare_parameter('cmd_timeout', 0.5).value
        self.max_wheel_speed = self.declare_parameter('max_wheel_speed', 12.0).value
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.r, self.Lx, self.Ly, self.timeout, self.max_wheel_speed)):
            raise ValueError('Geometry, timeout and wheel speed limit must be positive and finite')
        # A paused /clock must not prevent stale commands from being cleared.
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.last_command = None
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.pub_fl = self.create_publisher(Float64MultiArray, '/front_left_velocity_controller/commands', 10)
        self.pub_fr = self.create_publisher(Float64MultiArray, '/front_right_velocity_controller/commands', 10)
        self.pub_rl = self.create_publisher(Float64MultiArray, '/rear_left_velocity_controller/commands', 10)
        self.pub_rr = self.create_publisher(Float64MultiArray, '/rear_right_velocity_controller/commands', 10)

        self.watchdog = self.create_timer(
            0.05, self.watchdog_cb, clock=self.watchdog_clock)

    def publish_wheels(self, values):
        for publisher, value in zip(
                (self.pub_fl, self.pub_fr, self.pub_rl, self.pub_rr), values):
            publisher.publish(Float64MultiArray(data=[value]))

    def watchdog_cb(self):
        now = self.watchdog_clock.now().nanoseconds
        if self.last_command is None or (now - self.last_command) * 1e-9 >= self.timeout:
            self.publish_wheels([0.0] * 4)

    def cmd_cb(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        if not all(math.isfinite(v) for v in (vx, vy, omega)):
            self.last_command = None
            self.publish_wheels([0.0] * 4)
            return
        self.last_command = self.watchdog_clock.now().nanoseconds
        w_fl = (vx - vy - (self.Lx + self.Ly) * omega) / self.r
        w_fr = (vx + vy + (self.Lx + self.Ly) * omega) / self.r
        w_rl = (vx + vy - (self.Lx + self.Ly) * omega) / self.r
        w_rr = (vx - vy + (self.Lx + self.Ly) * omega) / self.r
        values = [w_fl, w_fr, w_rl, w_rr]
        if not all(math.isfinite(v) for v in values):
            self.last_command = None
            self.publish_wheels([0.0] * 4)
            return
        # Scale all four wheels together to preserve the requested twist direction.
        scale = max(1.0, max(abs(v) for v in values) / self.max_wheel_speed)
        self.publish_wheels([v / scale for v in values])


def main():
    rclpy.init()
    node = MecanumKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_wheels([0.0] * 4)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
