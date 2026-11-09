#!/usr/bin/env python3
"""Opt-in synthetic ROS integration test; never sends motion commands.

Run after sourcing ROS and the workspace. Uses ROS domain 93, synthetic scan/TF,
/tmp logs, and the installed converter. No Gazebo or navigation is started.
"""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

os.environ.update(ROS_DOMAIN_ID='93', ROS_LOG_DIR='/tmp/scan_cloud_test_roslogs')
import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster


def main():
    script = Path(get_package_prefix('omove_description')) / 'lib/omove_description/scan_to_cloud.py'
    report = {'success': False, 'domain': 93, 'motion_commands_sent': False,
              'tests': [], 'transform_wait_seconds': 1.0}
    log = open('/tmp/scan_cloud_transport.log', 'w')
    process = subprocess.Popen(
        [sys.executable, str(script), '--ros-args', '-p', 'use_sim_time:=false',
         '-p', 'target_frame:=cloud_test_odom', '-p', 'transform_wait:=1.0'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('scan_cloud_transport_test')
    received = []
    sub = node.create_subscription(PointCloud2, '/scan_cloud', received.append, 10)
    pub = node.create_publisher(LaserScan, '/scan', 10)
    broadcaster = TransformBroadcaster(node)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            if process.poll() is not None:
                raise RuntimeError('Converter exited; see /tmp/scan_cloud_transport.log')

    def wait(predicate, seconds=5):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            spin(.02)
        assert predicate(), 'Timed out waiting for converter output/discovery'

    def tf(stamp, x):
        result = TransformStamped()
        result.header.stamp.sec = stamp
        result.header.frame_id = 'cloud_test_odom'
        result.child_frame_id = 'cloud_test_laser'
        result.transform.translation.x = float(x)
        result.transform.translation.y = 2.
        result.transform.translation.z = .3
        result.transform.rotation.z = math.sin(math.pi/4)
        result.transform.rotation.w = math.cos(math.pi/4)
        return result

    scan = LaserScan()
    scan.header.frame_id = 'cloud_test_laser'
    scan.header.stamp.sec = 101
    scan.angle_min, scan.angle_max, scan.angle_increment = 0., math.pi, math.pi/4
    scan.range_min, scan.range_max = .15, 8.
    scan.ranges = [2., float('inf'), float('nan'), .1, 8.]

    def check(cloud, stamp, x):
        assert cloud.header.frame_id == 'cloud_test_odom'
        assert cloud.header.stamp.sec == stamp
        assert cloud.width == 1
        points = point_cloud2.read_points_numpy(cloud, field_names=['x', 'y', 'z'])
        np.testing.assert_allclose(points, [[x, 4., .3]], atol=2e-5)

    try:
        wait(lambda: node.count_subscribers('/scan') > 0 and
             node.count_subscribers('/tf') > 0 and node.count_publishers('/scan_cloud') > 0,
             seconds=12)
        spin(.2)
        # Scan precedes TF. TF interpolation at 101 gives x=2, NOT latest x=3.
        pub.publish(scan)
        spin(.15)
        assert not received, 'Published before the required TF arrived'
        broadcaster.sendTransform([tf(100, 1), tf(102, 3)])
        wait(lambda: len(received) >= 1)
        check(received[-1], 101, 2.)
        report['tests'].append('delayed TF + timestamp interpolation + invalid range filtering')

        count = len(received)
        future = copy.deepcopy(scan)
        future.header.stamp.sec = 103
        pub.publish(future)
        spin(.15)
        assert len(received) == count, 'Used latest TF for a future-stamped scan'
        broadcaster.sendTransform(tf(104, 5))
        wait(lambda: len(received) > count)
        check(received[-1], 103, 4.)
        report['tests'].append('future TF waits instead of falling back to latest')

        count = len(received)
        old = copy.deepcopy(scan)
        old.header.stamp.sec = 99
        pub.publish(old)
        spin(1.3)
        assert len(received) == count, 'Published a scan without historical TF'
        pub.publish(scan)
        wait(lambda: len(received) > count)
        check(received[-1], 101, 2.)
        report['tests'].append('missing historical TF expires; valid scans resume')

        count = len(received)
        zero = copy.deepcopy(scan)
        zero.header.stamp.sec = 0
        pub.publish(zero)
        spin(.2)
        assert len(received) == count, 'Zero scan timestamp used latest TF'
        report['tests'].append('zero timestamp rejected for cross-frame projection')
        report['success'] = True
    except Exception as error:
        report['error'] = str(error)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log.close()
        Path('/tmp/scan_cloud_transport.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
