#!/usr/bin/env python3
"""Project LaserScan to XYZ PointCloud2 using TF at the scan timestamp.

Each output contains one scan. No accumulation, deskew, odometry estimation,
or motion commands are performed here.
"""
from collections import deque
from copy import deepcopy
import math
import time

import numpy as np
from laser_geometry import LaserProjection
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def validate_scan(scan):
    if not scan.header.frame_id:
        raise ValueError('LaserScan header.frame_id is empty')
    if not all(math.isfinite(v) for v in
               (scan.angle_min, scan.angle_max, scan.angle_increment,
                scan.range_min, scan.range_max)):
        raise ValueError('LaserScan angles/range limits must be finite')
    if scan.range_min < 0 or scan.range_max <= scan.range_min:
        raise ValueError('LaserScan range limits are invalid')
    if len(scan.ranges) > 1 and scan.angle_increment == 0:
        raise ValueError('LaserScan angle_increment is zero')


def project_scan(scan, target_frame, transform=None, projector=None):
    """Project and rigidly transform one scan; preserve its acquisition stamp."""
    validate_scan(scan)
    if not target_frame:
        raise ValueError('target_frame is empty')
    projector = projector if projector is not None else LaserProjection()
    # XYZ only: malformed/absent intensity arrays cannot break projection.
    # Infinite/no-return ranges are normal; the projector removes them after
    # its intermediate polar multiplication (which may compute inf * 0).
    with np.errstate(invalid='ignore'):
        cloud = projector.projectLaser(
            scan, channel_options=LaserProjection.ChannelOption.NONE)
    if target_frame == scan.header.frame_id:
        cloud.is_dense = True
        return cloud
    if transform is None:
        raise ValueError('A real TF transform is required; relabeling is invalid')
    if (transform.header.frame_id != target_frame or
            transform.child_frame_id != scan.header.frame_id):
        raise ValueError('TF source/target do not match the requested frames')
    q = transform.transform.rotation
    translation = transform.transform.translation
    values = (q.x, q.y, q.z, q.w, translation.x, translation.y, translation.z)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('TF contains non-finite values')
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if norm < 1e-12:
        raise ValueError('TF quaternion has zero norm')
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    points = point_cloud2.read_points_numpy(cloud, field_names=['x', 'y', 'z'])
    points = points @ rotation.T + np.array(
        [translation.x, translation.y, translation.z])
    header = deepcopy(scan.header)
    header.frame_id = target_frame
    output = point_cloud2.create_cloud_xyz32(header, points.astype(np.float32))
    output.is_dense = True
    return output


class ScanToCloud(Node):
    def __init__(self):
        super().__init__('scan_to_cloud')
        self.target_frame = self.declare_parameter('target_frame', 'odom').value
        input_topic = self.declare_parameter('input_topic', '/scan').value
        output_topic = self.declare_parameter('output_topic', '/scan_cloud').value
        self.wait_seconds = self.declare_parameter('transform_wait', 0.5).value
        self.queue_size = self.declare_parameter('queue_size', 10).value
        if not self.target_frame or not input_topic or not output_topic:
            raise ValueError('Frames and topic names must not be empty')
        if (not math.isfinite(self.wait_seconds) or self.wait_seconds <= 0 or
                self.queue_size <= 0):
            raise ValueError('transform_wait and queue_size must be positive')
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.projector = LaserProjection()
        self.pending = deque()
        self.last_warning = -math.inf
        # Best-effort input accepts the Gazebo sensor publisher. Reliable XYZ
        # output also works with the default RViz PointCloud2 subscriber.
        self.publisher = self.create_publisher(PointCloud2, output_topic, 5)
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.receive_scan, qos_profile_sensor_data)
        self.retry_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.retry_timer = self.create_timer(
            0.02, self.flush_pending, clock=self.retry_clock)
        self.get_logger().info(
            f'{input_topic} -> {output_topic}, XYZ in {self.target_frame}; '
            'single scan, timestamped TF, no accumulation/deskew')

    def warn(self, message):
        now = time.monotonic()
        if now - self.last_warning >= 5.0:
            self.get_logger().warning(message)
            self.last_warning = now

    def receive_scan(self, scan):
        try:
            validate_scan(scan)
        except ValueError as error:
            self.warn(str(error))
            return
        if (scan.header.frame_id != self.target_frame and
                scan.header.stamp.sec == 0 and scan.header.stamp.nanosec == 0):
            self.warn('Dropping zero-stamped scan: TF time=0 means latest, not acquisition time')
            return
        if len(self.pending) >= self.queue_size:
            self.pending.popleft()
            self.warn('TF pending queue full; dropping oldest scan')
        self.pending.append((time.monotonic(), scan))
        self.flush_pending()

    def flush_pending(self):
        # Never block the executor waiting for TF: its subscriber callbacks
        # need the same executor to receive the awaited transform.
        while self.pending:
            received, scan = self.pending[0]
            transform = None
            if scan.header.frame_id != self.target_frame:
                try:
                    transform = self.buffer.lookup_transform(
                        self.target_frame, scan.header.frame_id,
                        Time.from_msg(scan.header.stamp))
                except TransformException as error:
                    if time.monotonic() - received < self.wait_seconds:
                        return
                    self.pending.popleft()
                    self.warn(f'TF unavailable at scan timestamp; dropping scan: {error}')
                    continue
            self.pending.popleft()
            try:
                self.publisher.publish(project_scan(
                    scan, self.target_frame, transform, self.projector))
            except ValueError as error:
                self.warn(f'Dropping invalid scan/transform: {error}')


def main():
    rclpy.init()
    node = ScanToCloud()
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
