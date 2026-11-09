#!/usr/bin/env python3
"""Opt-in Gazebo regression; run after sourcing the built workspace.

Launches an isolated ROS domain and writes metrics/trajectory to JSON.
Does not overwrite project maps or require RViz. Wall-time limits are deliberate.
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


def footprint_clearance(pose):
    """Conservative 0.54 x 0.46 m footprint vs room walls and central box.

    Returns separating-axis margin; negative means footprint overlap, not a
    measured Gazebo contact force. This is a geometry check for this room only.
    """
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    axes = [(1., 0.), (0., 1.), (c, s), (-s, c)]
    box_margin = max(
        abs(x*ax + y*ay) - .25*(abs(ax)+abs(ay))
        - .27*abs(c*ax+s*ay) - .23*abs(-s*ax+c*ay)
        for ax, ay in axes)
    wall_margin = min(2.95-abs(x)-.27*abs(c)-.23*abs(s),
                      2.95-abs(y)-.27*abs(s)-.23*abs(c))
    return min(box_margin, wall_margin)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/tmp/omove_navigation_validation.json')
    parser.add_argument('--domain', type=int, default=97)
    parser.add_argument('--goal-timeout', type=float, default=150.)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(output.parent / (output.stem + '_roslogs'))

    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from gazebo_msgs.msg import ModelStates
    from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
    from rclpy.action import ActionClient

    rclpy.init()
    node = rclpy.create_node('navigation_regression')
    report = {'goals': [], 'trajectory': [], 'checks': {}, 'success': False}
    state = {}

    def pose_values(p):
        q = p.orientation
        return [p.position.x, p.position.y,
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]

    def truth_callback(msg):
        if 'omove' in msg.name:
            state['truth'] = pose_values(msg.pose[msg.name.index('omove')])

    def localized(msg):
        state['amcl'] = pose_values(msg.pose.pose)
        if 'truth' in state:
            truth = state['truth']
            report['trajectory'].append({
                'wall_time': time.monotonic(), 'truth': list(truth),
                'amcl': list(state['amcl']),
                'position_error': math.hypot(truth[0]-state['amcl'][0],
                                             truth[1]-state['amcl'][1])})

    subscriptions = [
        node.create_subscription(ModelStates, '/gazebo/model_states', truth_callback, 10),
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', localized, 10),
        node.create_subscription(Twist, '/cmd_vel', lambda m: state.update(
            cmd=[m.linear.x, m.linear.y, m.angular.z]), 10)]
    initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    spinner = ActionClient(node, Spin, '/spin')
    active = None

    def wait(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not future.done():
            raise TimeoutError(f'operation exceeded {timeout}s wall time')
        return future.result()

    def send(client, goal, timeout):
        nonlocal active
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('action server unavailable')
        active = wait(client.send_goal_async(goal), 5)
        if not active.accepted:
            raise RuntimeError('goal rejected')
        result = wait(active.get_result_async(), timeout)
        active = None
        return result

    def target(x, y):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.
        return goal

    with tempfile.TemporaryDirectory(prefix='omove_navigation_') as temporary:
        world = Path(temporary) / 'validation.world'
        source = Path(get_package_share_directory('omove_description')) / 'worlds/room.world'
        world.write_text(source.read_text().replace('<world name="room">',
            '<world name="room"><plugin name="validation_state" filename="libgazebo_ros_state.so">'
            '<ros><namespace>/gazebo</namespace></ros><update_rate>20</update_rate></plugin>'))
        with output.with_suffix('.log').open('w') as log:
            process = subprocess.Popen([
                'ros2', 'launch', 'omove_description', 'nav_sim.launch.py',
                'gui:=false', f'world:={world}'], stdout=log, stderr=log,
                start_new_session=True)
            started = time.monotonic()
            try:
                deadline = started + 25
                last_initial = 0.
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.05)
                    if time.monotonic()-last_initial > 1:
                        msg = PoseWithCovarianceStamped()
                        msg.header.frame_id = 'map'
                        msg.pose.pose.position.x = 1.5
                        msg.pose.pose.orientation.w = 1.
                        for index in (0, 7, 35):
                            msg.pose.covariance[index] = .02
                        initial.publish(msg)
                        last_initial = time.monotonic()
                if 'truth' not in state or 'amcl' not in state:
                    raise RuntimeError('ground truth or AMCL missing')

                # The direct line crosses the central obstacle: require a detour.
                path_goal = ComputePathToPose.Goal()
                path_goal.goal = target(-1.2, 0).pose
                path_goal.planner_id = 'GridBased'
                path = send(planner, path_goal, 10)
                assert path.status == 4 and path.result.path.poses, 'detour planning failed'
                detour = max(abs(p.pose.position.y) for p in path.result.path.poses)
                assert detour > .5, ('path failed to detour', detour)
                report['checks']['planned_detour_y'] = detour
                print('PASS planned obstacle detour:', detour, flush=True)

                for x, y in [(-1.2, 0), (-1.2, 1), (1.5, 0)]:
                    before = time.monotonic()
                    result = send(navigator, target(x, y), args.goal_timeout)
                    entry = {'target': [x, y], 'status': result.status,
                             'wall_seconds': time.monotonic()-before,
                             'truth': list(state['truth']), 'amcl': list(state['amcl'])}
                    report['goals'].append(entry)
                    print('GOAL', entry, flush=True)
                    assert result.status == 4, 'navigation failed'
                    assert math.hypot(state['truth'][0]-x, state['truth'][1]-y) < .4, 'true endpoint outside tolerance'

                # The box center is occupied; the planner must refuse it.
                path_goal.goal = target(0, 0).pose
                rejected = send(planner, path_goal, 10)
                report['checks']['occupied_goal_status'] = rejected.status
                assert rejected.status == 6, ('occupied goal not aborted', rejected.status)
                report['checks']['occupied_goal_aborted'] = True
                # Exercise the recovery behavior server in clear space.
                spin = Spin.Goal()
                spin.target_yaw = .3
                spin.time_allowance.sec = 15
                result = send(spinner, spin, 40)
                assert result.status == 4, ('spin recovery failed', result.status)
                report['checks']['spin_behavior_succeeded'] = True
                # A valid goal must still work after failed planning and recovery.
                result = send(navigator, target(1.5, .6), args.goal_timeout)
                assert result.status == 4, 'navigation failed after recovery'
                report['checks']['navigation_after_recovery'] = True
                report['success'] = True
            except Exception as error:
                report['error'] = f'{type(error).__name__}: {error}'
                raise
            finally:
                if active is not None and active.accepted:
                    try:
                        wait(active.cancel_goal_async(), 5)
                    except Exception:
                        pass
                report['wall_seconds'] = time.monotonic()-started
                if report['trajectory']:
                    errors = [p['position_error'] for p in report['trajectory']]
                    report['max_position_error'] = max(errors)
                    report['mean_position_error'] = sum(errors)/len(errors)
                    margins = [footprint_clearance(p['truth']) for p in report['trajectory']]
                    report['min_sampled_footprint_clearance'] = min(margins)
                    if min(margins) < 0:
                        report['success'] = False
                        report['geometry_error'] = 'sampled footprint overlaps room obstacle'
                    if max(errors) > .4:
                        report['success'] = False
                        report['localization_error'] = 'position error exceeded 0.4 m'
                output.write_text(json.dumps(report, indent=2))
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                node.destroy_node()
                rclpy.shutdown()
    print('Report:', output, flush=True)
    if not report['success']:
        raise RuntimeError('regression metrics failed; inspect JSON report')


if __name__ == '__main__':
    main()
