#!/usr/bin/env python3
"""Isolated, opt-in reproduction of the 2026-10-31 concave-corner regression.
Run after sourcing ROS/overlay: python3 this_file.py baseline|production.
Uses ROS domain 94, Gazebo port 11361 and /tmp logs; sends a navigation goal
only in its own simulation. Map/spawn coordinates are specific to world01.yaml.
"""
import os,time,json,signal,subprocess,math,sys,tempfile,xml.etree.ElementTree as ET
from pathlib import Path
import yaml
variant='forward_footprint'
os.environ.update(ROS_DOMAIN_ID='94',GAZEBO_MASTER_URI='http://localhost:11361',ROS_LOG_DIR='/tmp/omove_corner_'+variant+'_roslogs')
import rclpy
from rclpy.action import ActionClient
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import PoseWithCovarianceStamped,Twist
from nav_msgs.msg import Odometry,Path as RosPath
from nav2_msgs.action import NavigateToPose
from gazebo_msgs.msg import ModelStates
from dwb_msgs.msg import LocalPlanEvaluation
from rclpy.qos import QoSProfile,DurabilityPolicy

def xy(p):
 q=p.orientation
 return [p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
workspace=Path('/home/deng/omove_ws')
base=workspace/'src/omove_description'
tree=ET.parse(base/'worlds/world01/worlds/playground.world')
plugin=ET.SubElement(tree.getroot().find('world'),'plugin',name='diagnostic_state',filename='libgazebo_ros_state.so')
ET.SubElement(ET.SubElement(plugin,'ros'),'namespace').text='/diagnostic'
ET.SubElement(plugin,'update_rate').text='10'
world='/tmp/omove_corner.world';tree.write(world)
config_path=workspace/'docs/diagnostics/dwb_tuning/corner_candidate_01.yaml'
config=yaml.safe_load(config_path.read_text())
f=config['controller_server']['ros__parameters']['FollowPath'];f['debug_trajectory_details']=variant!='production';f['publish_evaluation']=variant!='production'
if variant in ['holonomic','lookahead']:
 f['critics']=['RotateToGoal','Oscillation','ObstacleFootprint','PathDist','GoalDist']
 f['ObstacleFootprint.scale']=.02
 f['GoalDist.scale']=16.
 f['PathDist.scale']=32.
if variant in ['lookahead','lookahead_only']:
 f['forward_prune_distance']=.8
 f['prune_distance']=1.0
 f['shorten_transformed_plan']=True
params='/tmp/omove_corner_'+variant+'.yaml';Path(params).write_text(yaml.safe_dump(config))
log=open('/tmp/omove_corner_'+variant+'.log','w')
p=subprocess.Popen(['ros2','launch','omove_description','world01_nav.launch.py','gui:=false','map:='+str(workspace/'maps/world01.yaml'),'x:=-1.16','y:=-3.43','world:='+world,'params_file:='+params],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();n=rclpy.create_node('corner_navigation_diagnostic');state={};samples=[];gh=None
n.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
subs=[]
def add(typ,topic,cb,qos=1):subs.append(n.create_subscription(typ,topic,cb,qos))
add(PoseWithCovarianceStamped,'/amcl_pose',lambda m:state.update(amcl=xy(m.pose.pose)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
add(Odometry,'/odom',lambda m:state.update(odom=xy(m.pose.pose),sim=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,twist=[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z]))
def truth(m):
 if 'omove' in m.name:state['truth']=xy(m.pose[m.name.index('omove')])
add(ModelStates,'/diagnostic/model_states',truth)
for topic in ['/cmd_vel','/cmd_vel_nav']:
 add(Twist,topic,lambda m,t=topic:state.update({t:[m.linear.x,m.linear.y,m.angular.z]}))
def ev(m):
 if m.twists:
  b=m.twists[m.best_index];v=b.traj.velocity
  state['best']={'v':[v.x,v.y,v.theta],'total':b.total,'scores':{s.name:[s.raw_score,s.scale] for s in b.scores},'valid':sum(t.total>=0 for t in m.twists),'n':len(m.twists)}
add(LocalPlanEvaluation,'/evaluation',ev)
add(RosPath,'/plan',lambda m:state.update(plan=[[round(p.pose.position.x,2),round(p.pose.position.y,2)] for p in m.poses][::5]))
initial=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
client=n.create_client(GetState,'/bt_navigator/get_state');action=ActionClient(n,NavigateToPose,'/navigate_to_pose')
report={'variant':variant,'success':False,'samples':samples}
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  for _ in range(12):rclpy.spin_once(n,timeout_sec=0.)
  time.sleep(.01)
try:
 deadline=time.monotonic()+80;last=0;active=False;future=None
 while time.monotonic()<deadline and p.poll() is None:
  spin(.1)
  if time.monotonic()-last>1:
   msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=n.get_clock().now().to_msg();msg.pose.pose.position.x=-1.16;msg.pose.pose.position.y=-2.43;msg.pose.pose.orientation.w=1.
   for i in [0,7,35]:msg.pose.covariance[i]=.01
   initial.publish(msg);last=time.monotonic()
  if future is not None and future.done():
   active=future.result().current_state.label=='active';future=None
  if active and action.server_is_ready() and 'truth' in state and 'amcl' in state:break
  if future is None and client.service_is_ready():future=client.call_async(GetState.Request())
 if not active:raise RuntimeError('Nav2 not ready')
 spin(1);report['start']=dict(state)
 goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=n.get_clock().now().to_msg();goal.pose.pose.position.x=.21;goal.pose.pose.position.y=-.11;goal.pose.pose.orientation.w=1.
 future=action.send_goal_async(goal)
 while not future.done():spin(.05)
 gh=future.result();report['accepted']=gh.accepted
 if not gh.accepted:raise RuntimeError('goal rejected')
 result=gh.get_result_async();deadline=time.monotonic()+90;last=0
 while time.monotonic()<deadline and not result.done() and p.poll() is None:
  spin(.1)
  if time.monotonic()-last>2:
   sample=json.loads(json.dumps(state));samples.append(sample);print(json.dumps({k:sample.get(k) for k in ['sim','truth','amcl','/cmd_vel']}),flush=True);last=time.monotonic()
 if result.done():report['status']=result.result().status;report['success']=report['status']==4
 else:report['status']='timeout'
 report['finish']=dict(state)
except Exception as e:report['error']=str(e)
finally:
 if gh is not None:
  try:gh.cancel_goal_async();spin(.3)
  except Exception:pass
 Path('/tmp/omove_corner_'+variant+'.json').write_text(json.dumps(report,indent=2))
 print(json.dumps({k:v for k,v in report.items() if k not in ['samples','start']}),flush=True)
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=15)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 
 if rclpy.ok():rclpy.shutdown()
 log.close()
