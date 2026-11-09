#!/usr/bin/env python3
"""Opt-in live Gazebo navigation measurement; does not launch or tune nodes.

Selects short straight routes whose swept circumscribed footprint is free in /map.
Sends NavigateToPose goals, cancels on timeout, writes per-run samples and metrics.
Run only with a localized robot and no other operator sending goals or cmd_vel.
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


def angle(x):
    return math.atan2(math.sin(x), math.cos(x))


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    p.add_argument('--routes', required=True)
    args = p.parse_args()
    output = Path(args.output)
    if output.exists():
        raise SystemExit('Refusing to overwrite existing report')
    rclpy.init()
    n = rclpy.create_node('dwb_tuning_measurement')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    buf = Buffer()
    listener = TransformListener(buf, n)
    state = {'cmd': [0., 0., 0.]}
    def truth(m):
        if 'omove' in m.name:
            p = m.pose[m.name.index('omove')]
            state['truth'] = [p.position.x, p.position.y, yaw(p.orientation)]
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    subs = [n.create_subscription(OccupancyGrid, '/map', lambda m: state.update(map=m), qos),
            n.create_subscription(ModelStates, '/gazebo/model_states', truth, 10),
            n.create_subscription(Twist, '/cmd_vel', lambda m: state.update(cmd=[m.linear.x,m.linear.y,m.angular.z]), 10),
            n.create_subscription(Odometry, '/odom', lambda m: state.update(odom=[m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)]), 10)]
    client = ActionClient(n, NavigateToPose, '/navigate_to_pose')
    def spin(sec):
        end=time.monotonic()+sec
        while time.monotonic()<end:
            rclpy.spin_once(n, timeout_sec=.02)
    def pose():
        try:
            t=buf.lookup_transform('map','base_footprint',rclpy.time.Time()).transform
            return [t.translation.x,t.translation.y,yaw(t.rotation)]
        except Exception:
            return None
    def wait(f, sec):
        end=time.monotonic()+sec
        while not f.done() and time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.02)
        if not f.done(): raise TimeoutError('action request timeout')
        return f.result()
    report={'runs':[], 'routes':None}
    active=None
    try:
        spin(4)
        if not client.wait_for_server(timeout_sec=8) or pose() is None or 'map' not in state:
            raise RuntimeError('Navigation, TF or map unavailable')
        # Circle radius encloses 0.54 x 0.46 m body, with 4 cm extra clearance.
        def free(x,y):
            m=state['map'];res=m.info.resolution;o=m.info.origin
            a=yaw(o.orientation);dx=x-o.position.x;dy=y-o.position.y
            gx=math.cos(a)*dx+math.sin(a)*dy;gy=-math.sin(a)*dx+math.cos(a)*dy
            cx=int(math.floor(gx/res));cy=int(math.floor(gy/res));r=math.ceil(.395/res)
            for j in range(-r,r+1):
                for i in range(-r,r+1):
                    if math.hypot(i*res,j*res)>.395:continue
                    xx=cx+i;yy=cy+j
                    if not(0<=xx<m.info.width and 0<=yy<m.info.height):return False
                    if m.data[yy*m.info.width+xx]!=0:return False
            return True
        def linefree(a,b):
            steps=max(1,math.ceil(math.dist(a[:2],b[:2])/.04))
            return all(free(a[0]+(b[0]-a[0])*i/steps,a[1]+(b[1]-a[1])*i/steps) for i in range(steps+1))
        rp=Path(args.routes)
        if rp.exists(): routes=json.loads(rp.read_text())
        else:
            start=pose();routes=[{'name':'start','pose':start}]
            for label,off in [('forward',0.),('lateral',math.pi/2)]:
                found=None
                for sign in [1,-1]:
                    for length in [.8,.6]:
                        a=start[2]+off;b=[start[0]+sign*length*math.cos(a),start[1]+sign*length*math.sin(a),start[2]]
                        if linefree(start,b):found=b;break
                    if found:break
                if not found:raise RuntimeError('No clear short '+label+' route from current pose')
                routes += [{'name':label,'pose':found},{'name':'return_'+label,'pose':start}]
            # A stationary 90-degree heading change, then restore, at the same clear center.
            routes += [{'name':'rotate','pose':[start[0],start[1],angle(start[2]+math.pi/2)]}, {'name':'restore','pose':start}]
            rp.write_text(json.dumps(routes,indent=2))
        report['routes']=routes
        for route in routes:
            start=pose();target=route['pose']
            if not linefree(start,target):raise RuntimeError('Route footprint not clear: '+route['name'])
            goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=n.get_clock().now().to_msg()
            goal.pose.pose.position.x,goal.pose.pose.position.y=target[:2]
            goal.pose.pose.orientation.z=math.sin(target[2]/2);goal.pose.pose.orientation.w=math.cos(target[2]/2)
            active=wait(client.send_goal_async(goal),8)
            if not active.accepted:raise RuntimeError('goal rejected')
            f=active.get_result_async();samples=[];begin=time.monotonic();last=-1
            print('Testing',route['name'],target,flush=True)
            while not f.done() and time.monotonic()-begin<80:
                rclpy.spin_once(n,timeout_sec=.02)
                now=n.get_clock().now().nanoseconds/1e9
                if now-last>=.095:
                    samples.append({'time':now,'pose':pose(),'cmd':list(state['cmd']),'truth':state.get('truth'),'odom':state.get('odom')});last=now
            if not f.done():
                wait(active.cancel_goal_async(),5);spin(1)
                raise TimeoutError('goal timeout: '+route['name'])
            status=f.result().status;active=None;spin(1)
            moving=[s for s in samples if math.hypot(*s['cmd'][:2])>.025 and s['pose']]
            headings=[angle(s['pose'][2]-target[2]) for s in moving]
            turns=[s['cmd'][2] for s in samples]
            signs=[1 if x>.07 else -1 for x in turns if abs(x)>.07]
            distances=[]
            dx=target[0]-start[0];dy=target[1]-start[1];length=math.hypot(dx,dy)
            if length>.1:
                distances=[abs(dx*(s['pose'][1]-start[1])-dy*(s['pose'][0]-start[0]))/length for s in moving]
            metrics={'status':status,'wall_seconds':round(time.monotonic()-begin,2),'samples':len(samples),
                     'moving_max_heading_error_deg':round(max(map(abs,headings),default=0)*180/math.pi,2),
                     'moving_heading_span_deg':round((max(headings)-min(headings))*180/math.pi,2) if headings else 0,
                     'max_cross_track_m':round(max(distances,default=0),3),
                     'turn_sign_changes':sum(a!=b for a,b in zip(signs,signs[1:])),
                     'angular_command_total_variation':round(sum(abs(a-b) for a,b in zip(turns,turns[1:])),3),
                     'end_pose':pose()}
            report['runs'].append({'name':route['name'],'metrics':metrics,'samples':samples})
            output.write_text(json.dumps(report,indent=2))
            print(json.dumps(metrics),flush=True)
            if status!=4:raise RuntimeError('Navigation failed, status '+str(status))
    except Exception as e:
        report['error']=str(e);print('ERROR',e,flush=True)
        if active:
            try:wait(active.cancel_goal_async(),5);spin(1)
            except Exception:pass
    finally:
        output.write_text(json.dumps(report,indent=2))
        n.destroy_node();rclpy.shutdown()
    if 'error' in report:
        raise SystemExit(1)


if __name__=='__main__':main()
