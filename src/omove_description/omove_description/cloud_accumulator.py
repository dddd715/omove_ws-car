#!/usr/bin/env python3
"""Bounded odom-frame XYZ cloud assembly; no registration or SLAM."""
from collections import deque
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger


def voxel_filter(points, size):
    """Keep one measured point per voxel; never synthesize vertical layers."""
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        return points
    keys = np.floor(points.astype(np.float64) / size)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


class CloudWindow:
    def __init__(self, window_seconds=30., voxel_size=.03,
                 max_points=150000, max_frames=300):
        if not all(math.isfinite(v) and v > 0 for v in
                   (window_seconds, voxel_size, max_points, max_frames)):
            raise ValueError('Window, voxel size and limits must be positive and finite')
        if int(max_points) != max_points or int(max_frames) != max_frames:
            raise ValueError('Point/frame limits must be integers')
        self.window_ns = int(window_seconds * 1e9)
        self.voxel_size = voxel_size
        self.max_points, self.max_frames = int(max_points), int(max_frames)
        self.clear()

    def clear(self):
        self.frames = deque()
        self.point_count = 0
        self.latest_ns = None

    def add(self, stamp_ns, points):
        reset = self.latest_ns is not None and stamp_ns < self.latest_ns
        if reset:
            self.clear()
        if stamp_ns == self.latest_ns:
            return False  # Ignore a duplicate publication of the same scan.
        points = voxel_filter(points, self.voxel_size)
        if len(points) > self.max_points:
            points = points[np.linspace(0, len(points)-1, self.max_points, dtype=int)]
        self.latest_ns = stamp_ns
        self.frames.append((stamp_ns, points))
        self.point_count += len(points)
        while self.frames and (
                self.frames[0][0] < stamp_ns - self.window_ns or
                len(self.frames) > self.max_frames or self.point_count > self.max_points):
            self.point_count -= len(self.frames.popleft()[1])
        return reset

    def assembled(self):
        if not self.frames:
            return np.empty((0, 3), dtype=np.float32)
        # Prefer the most recent measured point in each voxel.
        return voxel_filter(np.concatenate([p for _, p in reversed(self.frames)]),
                            self.voxel_size)


def save_pcd(path, points, frame, stamp_ns):
    """Write XYZ ASCII PCD, refusing to overwrite an existing file."""
    path = Path(path).expanduser()
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if not len(points):
        raise ValueError('No accumulated points to save')
    created = False
    try:
        with path.open('x', encoding='ascii') as output:
            created = True
            output.write(
                f'# .PCD v0.7\n# frame_id={frame} latest_stamp_ns={stamp_ns}\n'
                'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                f'POINTS {len(points)}\nDATA ascii\n')
            np.savetxt(output, points, fmt='%.6f')
    except Exception:
        if created:
            path.unlink(missing_ok=True)
        raise
    return str(path.resolve())


class CloudAccumulator(Node):
    def __init__(self):
        super().__init__('cloud_accumulator')
        self.frame = self.declare_parameter('fixed_frame', 'odom').value
        if not self.frame or self.frame in ('base_link', 'base_footprint', 'laser_link'):
            raise ValueError('fixed_frame must be a fixed world/odometry frame')
        self.window = CloudWindow(
            self.declare_parameter('window_seconds', 30.).value,
            self.declare_parameter('voxel_size', .03).value,
            self.declare_parameter('max_points', 150000).value,
            self.declare_parameter('max_frames', 300).value)
        rate = self.declare_parameter('publish_rate', 2.).value
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate must be positive and finite')
        self.declare_parameter('save_path', '/tmp/omove_accumulated.pcd')
        self.dirty = False
        self.last_warning = -math.inf
        self.publisher = self.create_publisher(
            PointCloud2, '/scan_cloud_accumulated',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscription = self.create_subscription(
            PointCloud2, '/scan_cloud', self.receive_cloud, qos_profile_sensor_data)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1. / rate, self.publish_cloud, clock=self.clock)
        self.save_service = self.create_service(Trigger, '~/save', self.save)
        self.clear_service = self.create_service(Trigger, '~/clear', self.clear)
        self.get_logger().info(
            '/scan_cloud -> /scan_cloud_accumulated; '
            f'frame={self.frame}, window={self.window.window_ns/1e9}s, '
            f'voxel={self.window.voxel_size}m; odometry assembly, NOT 3D SLAM')

    def warn(self, message):
        now = time.monotonic()
        if now - self.last_warning >= 5.:
            self.get_logger().warning(message)
            self.last_warning = now

    def receive_cloud(self, cloud):
        if cloud.header.frame_id != self.frame:
            self.warn(f'Rejecting {cloud.header.frame_id}: expected already aligned {self.frame} points')
            return
        stamp = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
        if stamp <= 0:
            self.warn('Rejecting zero/negative scan timestamp')
            return
        try:
            points = point_cloud2.read_points_numpy(cloud, field_names=['x', 'y', 'z'])
            if self.window.add(stamp, points):
                self.warn('Input time moved backward; cleared accumulated history')
            self.dirty = True
        except (ValueError, AssertionError, KeyError) as error:
            self.warn(f'Rejecting invalid XYZ cloud: {error}')

    def publish_cloud(self):
        if not self.dirty:
            return
        stamp = self.window.latest_ns
        header = Header(frame_id=self.frame)
        if stamp is not None:
            header.stamp.sec, header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        cloud = point_cloud2.create_cloud_xyz32(header, self.window.assembled())
        cloud.is_dense = True
        self.publisher.publish(cloud)
        self.dirty = False

    def save(self, request, response):
        try:
            path = save_pcd(self.get_parameter('save_path').value,
                            self.window.assembled(), self.frame, self.window.latest_ns)
            response.success, response.message = True, f'Saved XYZ PCD: {path}'
        except (OSError, ValueError) as error:
            response.success, response.message = False, str(error)
        return response

    def clear(self, request, response):
        self.window.clear()
        self.dirty = True
        self.publish_cloud()  # Clear the transient snapshot in late RViz subscribers too.
        response.success, response.message = True, 'Accumulated cloud cleared'
        return response


def main():
    rclpy.init()
    node = CloudAccumulator()
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
