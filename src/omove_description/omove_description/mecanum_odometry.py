#!/usr/bin/env python3
"""Wheel-feedback odometry for an X-layout mecanum base."""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class MecanumOdometry(Node):
    JOINTS = tuple(f'{corner}_wheel_joint' for corner in
                   ('front_left', 'front_right', 'rear_left', 'rear_right'))

    def __init__(self):
        super().__init__('mecanum_odometry')
        self.radius = self.declare_parameter('wheel_radius', 0.076).value
        self.lever = (self.declare_parameter('half_wheelbase', 0.18).value +
                      self.declare_parameter('half_track', 0.20).value)
        if self.radius <= 0 or self.lever <= 0:
            raise ValueError('Wheel radius and summed half distances must be positive')
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.x = self.y = self.yaw = 0.0
        self.previous_stamp = None
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)

    def joint_callback(self, msg):
        # Resolve by name: joint_state_broadcaster does not promise wheel order.
        try:
            wheels = [msg.velocity[msg.name.index(name)] for name in self.JOINTS]
        except (ValueError, IndexError):
            return
        if not all(math.isfinite(value) for value in wheels):
            return
        stamp = msg.header.stamp
        seconds = stamp.sec + stamp.nanosec * 1e-9
        fl, fr, rl, rr = wheels
        vx = self.radius * (fl + fr + rl + rr) / 4.0
        vy = self.radius * (-fl + fr + rl - rr) / 4.0
        wz = self.radius * (-fl + fr - rl + rr) / (4.0 * self.lever)
        if self.previous_stamp is not None:
            dt = seconds - self.previous_stamp
            if dt == 0:
                return
            if dt < 0:
                # Gazebo reset: start a new odometry epoch, never integrate backwards.
                self.x = self.y = self.yaw = 0.0
            else:
                self.x, self.y, self.yaw = integrate_pose(
                    self.x, self.y, self.yaw, vx, vy, wz, dt)
        self.previous_stamp = seconds

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        # Provisional wheel-odometry uncertainties; calibrate against measured slip.
        covariance = [0.02, 0.02, 1e6, 1e6, 1e6, 0.05]
        for i, variance in enumerate(covariance):
            odom.pose.covariance[i * 7] = variance
            odom.twist.covariance[i * 7] = variance
        self.publisher.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def integrate_pose(x, y, yaw, vx, vy, wz, dt):
    """Integrate constant body twist exactly, including simultaneous rotation."""
    angle = wz * dt
    if abs(angle) < 1e-8:
        a, b = dt, wz * dt * dt / 2.0
    else:
        a, b = math.sin(angle) / wz, (1.0 - math.cos(angle)) / wz
    dx, dy = a * vx - b * vy, b * vx + a * vy
    x += math.cos(yaw) * dx - math.sin(yaw) * dy
    y += math.sin(yaw) * dx + math.cos(yaw) * dy
    yaw = math.atan2(math.sin(yaw + angle), math.cos(yaw + angle))
    return x, y, yaw


def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
