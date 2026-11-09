#!/usr/bin/env python3
"""Opt-in world01 wheel odometry diagnostic, using a separate simulation.

Sends short low-speed forward, lateral and rotation commands. Compares wheel
odometry with Gazebo ground truth; never uses ground truth to drive odometry.
The original world and existing maps are not modified.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def relative(start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return [c * dx + s * dy, -s * dx + c * dy, wrap(end[2] - start[2])]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/tmp/omove_odometry_validation.json')
    parser.add_argument('--domain', type=int, default=96)
    parser.add_argument('--port', type=int, default=11359)
    parser.add_argument('--phase', choices=['all', 'forward', 'lateral', 'rotation'], default='all')
    parser.add_argument('--rotation-speed', type=float, default=.2)
    parser.add_argument('--controller-rate', type=int, default=None,
                        help='Test an alternate controller update rate using a temporary YAML')
    parser.add_argument('--velocity-pid', action='store_true',
                        help='Experiment with native Gazebo velocity PID using a temporary URDF')
    parser.add_argument('--wheel-control-mode', choices=['velocity', 'velocity_pid'],
                        default='velocity_pid', help='Control mode for the production robot model')
    args = parser.parse_args()
    if not math.isfinite(args.rotation_speed) or not 0 < args.rotation_speed <= .6:
        parser.error('--rotation-speed must be finite and in (0, 0.6] rad/s')
    if args.controller_rate is not None and not 1 <= args.controller_rate <= 1000:
        parser.error('--controller-rate must be in [1, 1000] Hz')
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),
                      GAZEBO_MASTER_URI=f'http://localhost:{args.port}',
                      ROS_LOG_DIR=str(output.with_suffix('')) + '_roslogs')

    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import Twist
    from gazebo_msgs.msg import ModelStates
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, JointState
    from std_msgs.msg import Float64MultiArray
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node('wheel_odometry_validation')
    state = {}
    corners = ['front_left', 'front_right', 'rear_left', 'rear_right']
    samples = []
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    def pose(p):
        q = p.orientation
        return [p.position.x, p.position.y,
                math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))]

    def truth(msg):
        if 'omove' in msg.name:
            p = msg.pose[msg.name.index('omove')]
            state['truth'] = pose(p)
            state['height'] = p.position.z
            q = p.orientation
            state['tilt_rad'] = math.acos(max(-1., min(1., 1-2*(q.x*q.x+q.y*q.y))))
            state['truth_received'] = time.monotonic()

    def odom(msg):
        state['odom'] = pose(msg.pose.pose)
        state['sim_time'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        state['odom_received'] = time.monotonic()

    def scan(msg):
        state['clearance'] = min((v for v in msg.ranges if math.isfinite(v)), default=math.inf)

    def joints(msg):
        try:
            state['wheel_feedback'] = [msg.velocity[msg.name.index(c + '_wheel_joint')] for c in corners]
        except (ValueError, IndexError):
            pass

    subscriptions = [
        node.create_subscription(ModelStates, '/diagnostic/model_states', truth, 1),
        node.create_subscription(Odometry, '/odom', odom, 1),
        node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data),
        node.create_subscription(JointState, '/joint_states', joints, 1),
    ]
    for c in corners:
        subscriptions.append(node.create_subscription(
            Float64MultiArray, '/' + c + '_velocity_controller/commands',
            lambda msg, c=c: state.setdefault('wheel_command', {}).update({c: list(msg.data)}), 1))

    def drive(vx, vy, wz, seconds):
        start = state['sim_time']
        deadline = time.monotonic() + 80
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        while state['sim_time'] - start < seconds:
            if time.monotonic() > deadline:
                raise TimeoutError('simulation paused or too slow')
            if any(time.monotonic() - state.get(k, 0) > 2
                   for k in ('truth_received', 'odom_received')):
                raise RuntimeError('truth or wheel odometry became stale')
            if (vx or vy or wz) and state.get('clearance', 0) < 0.45:
                raise RuntimeError(f'obstacle/self-return too close; stopping diagnostic: {state}')
            publisher.publish(msg)
            # Drain the high-rate odom/truth callbacks each control cycle.
            # One spin per sleep can let receive queues skew the comparison.
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.)
            if vx or vy or wz:
                samples.append({k: state.get(k) for k in
                                ['sim_time', 'truth', 'odom', 'wheel_feedback']})
                samples[-1]['wheel_command'] = dict(state.get('wheel_command', {}))
            time.sleep(0.02)

    report = {'phases': [], 'purpose': 'diagnostic, not navigation acceptance',
              'position_limit_m': 0.05, 'heading_limit_rad': 0.05,
              'note': 'Limits are diagnostic thresholds, not calibrated accuracy guarantees.',
              'success': False}
    with tempfile.TemporaryDirectory(prefix='omove_odom_') as temp:
        source = Path(get_package_share_directory('omove_description')) / 'worlds/world01/worlds/playground.world'
        world = Path(temp) / 'diagnostic.world'
        tree = ET.parse(source)
        plugin = ET.SubElement(tree.getroot().find('world'), 'plugin',
                               name='diagnostic_state', filename='libgazebo_ros_state.so')
        ET.SubElement(ET.SubElement(plugin, 'ros'), 'namespace').text = '/diagnostic'
        ET.SubElement(plugin, 'update_rate').text = '50'
        tree.write(world)
        launch_overrides = [f'wheel_control_mode:={args.wheel_control_mode}']
        report['wheel_control_mode'] = args.wheel_control_mode
        if args.controller_rate is not None:
            import yaml
            path = Path(get_package_share_directory('omove_description')) / 'config/mecanum_controllers.yaml'
            config = yaml.safe_load(path.read_text())
            config['controller_manager']['ros__parameters']['update_rate'] = args.controller_rate
            controllers = Path(temp) / 'controllers.yaml'
            controllers.write_text(yaml.safe_dump(config))
            launch_overrides.append(f'controllers_file:={controllers}')
        report['controller_rate_override'] = args.controller_rate
        report['velocity_pid_experiment'] = args.velocity_pid
        if args.velocity_pid:
            import xacro
            model = Path(get_package_share_directory('omove_description')) / 'urdf/omove.urdf.xacro'
            robot = ET.fromstring(xacro.process_file(str(model), mappings={'wheel_control_mode': 'velocity'}).toxml())
            for joint in robot.findall('ros2_control/joint'):
                interface = joint.find('command_interface')
                if interface is not None:
                    interface.set('name', 'velocity_pid')
                    for name, value in [('vel_kp', '.5'), ('vel_ki', '.5'), ('vel_kd', '0'),
                                        ('vel_max_integral_error', '2'), ('vel_min_integral_error', '-2')]:
                        ET.SubElement(joint, 'param', name=name).text = value
            robot_file = Path(temp) / 'velocity_pid.urdf'
            ET.ElementTree(robot).write(robot_file)
            launch_overrides.append(f'robot_file:={robot_file}')
        with output.with_suffix('.log').open('w') as log:
            process = subprocess.Popen([
                'ros2', 'launch', 'omove_description', 'world01_sim.launch.py',
                'slam:=false', 'gui:=false', f'world:={world}'] + launch_overrides,
                stdout=log, stderr=log, start_new_session=True)
            try:
                deadline = time.monotonic() + 70
                while not all(k in state for k in ('truth', 'odom', 'clearance')) or publisher.get_subscription_count() == 0:
                    if process.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('simulation not ready; inspect log')
                    rclpy.spin_once(node, timeout_sec=0.05)
                drive(0., 0., 0., 1.0)
                for name, command, duration in [
                    ('forward', (.1, 0., 0.), 3.),
                    ('lateral', (0., .1, 0.), 3.),
                    ('rotation', (0., 0., args.rotation_speed), 3.),
                ]:
                    if args.phase not in ('all', name):
                        continue
                    samples.clear()
                    before_truth, before_odom = list(state['truth']), list(state['odom'])
                    drive(*command, duration)
                    drive(0., 0., 0., 1.5)
                    t = relative(before_truth, state['truth'])
                    o = relative(before_odom, state['odom'])
                    entry = dict(name=name, command=list(command), sim_seconds=duration,
                                 truth_delta=t, odom_delta=o,
                                 end_truth=list(state['truth']), height=state['height'],
                                 tilt_rad=state['tilt_rad'], scan_min=state['clearance'],
                                 position_error_m=math.hypot(t[0]-o[0], t[1]-o[1]),
                                 heading_error_rad=abs(wrap(t[2]-o[2])))
                    entry['samples'] = list(samples)
                    entry['within_diagnostic_limits'] = (entry['position_error_m'] <= .05
                                                        and entry['heading_error_rad'] <= .05)
                    report['phases'].append(entry)
                    print(json.dumps({k: v for k, v in entry.items() if k != 'samples'}), flush=True)
                report['success'] = all(p['within_diagnostic_limits'] for p in report['phases'])
            except Exception as error:
                report['error'] = f'{type(error).__name__}: {error}'
            finally:
                publisher.publish(Twist())
                output.write_text(json.dumps(report, indent=2))
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait()
                node.destroy_node()
                rclpy.shutdown()
    print('Report:', output, flush=True)
    if not report['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
