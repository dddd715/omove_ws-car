#!/usr/bin/env python3
"""Synthetic ROS-domain-92 assembly/save/clear test; no motion commands."""
import json,os,signal,subprocess,sys,tempfile,time
from pathlib import Path
os.environ.update(ROS_DOMAIN_ID='92',ROS_LOG_DIR='/tmp/cloud_accumulator_test_roslogs')
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_prefix


def main():
    report={'success':False,'domain':92,'motion_commands_sent':False,'tests':[]}
    with tempfile.TemporaryDirectory(prefix='accumulated_cloud_test_') as temp:
        output=Path(temp)/'cloud.pcd';log=open('/tmp/cloud_accumulator_test.log','w')
        script=Path(get_package_prefix('omove_description'))/'lib/omove_description/cloud_accumulator.py'
        process=subprocess.Popen([sys.executable,str(script),'--ros-args','-p','use_sim_time:=false','-p','window_seconds:=1.0','-p','voxel_size:=0.05','-p','publish_rate:=20.0','-p','save_path:='+str(output)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();node=rclpy.create_node('cloud_accumulator_transport_test');received=[];late=[]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub=node.create_subscription(PointCloud2,'/scan_cloud_accumulated',received.append,qos)
        pub=node.create_publisher(PointCloud2,'/scan_cloud',10)
        save=node.create_client(Trigger,'/cloud_accumulator/save');clear=node.create_client(Trigger,'/cloud_accumulator/clear')
        def spin(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.01)
                if process.poll() is not None:raise RuntimeError('Accumulator exited unexpectedly')
        def wait(predicate,seconds=5):
            end=time.monotonic()+seconds
            while not predicate() and time.monotonic()<end:spin(.02)
            assert predicate(),'Timed out waiting for ROS discovery/output'
        def send(stamp,points,frame='odom'):
            h=Header(frame_id=frame);h.stamp.sec=int(stamp);h.stamp.nanosec=round((stamp-int(stamp))*1e9)
            pub.publish(point_cloud2.create_cloud_xyz32(h,points))
        def service(client):
            f=client.call_async(Trigger.Request());wait(f.done);return f.result()
        try:
            wait(lambda:node.count_subscribers('/scan_cloud')>0 and save.service_is_ready() and clear.service_is_ready(),12)
            send(100.,[[0.,0.,.15],[1.,0.,.15]])
            wait(lambda:received and received[-1].width==2)
            send(100.5,[[1.01,0.,.15],[2.,0.,.15]])
            wait(lambda:received[-1].width==3)
            xyz=point_cloud2.read_points_numpy(received[-1],field_names=['x','y','z'])
            assert any(np.allclose(p,[1.01,0,.15]) for p in xyz)
            assert received[-1].header.frame_id=='odom'
            report['tests'].append('multi-frame union and latest point per voxel')
            late_sub=node.create_subscription(PointCloud2,'/scan_cloud_accumulated',late.append,qos)
            wait(lambda:late and late[-1].width==3)
            report['tests'].append('late subscribers receive retained snapshot')
            assert service(save).success
            saved=output.read_bytes();assert b'POINTS 3\n' in saved
            assert not service(save).success and output.read_bytes()==saved
            report['tests'].append('PCD save service and overwrite protection')
            send(101.6,[[3.,0.,.15]])
            wait(lambda:received[-1].width==1)
            np.testing.assert_allclose(point_cloud2.read_points_numpy(received[-1]),[[3,0,.15]],atol=1e-6)
            report['tests'].append('scan-time window expires old frames')
            count=len(received);send(102.,[[9.,0.,.15]],'laser_link');spin(.2)
            assert len(received)==count
            report['tests'].append('moving/wrong input frame is rejected')
            assert service(clear).success
            wait(lambda:received[-1].width==0)
            report['tests'].append('clear service publishes empty retained snapshot')
            send(1.,[[4.,0.,.15]])
            wait(lambda:received[-1].width==1)
            report['success']=True
        except Exception as error:report['error']=str(error)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
            log.close()
    Path('/tmp/cloud_accumulator_test.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2));return 0 if report['success'] else 1

if __name__=='__main__':raise SystemExit(main())
