"""Real ROS/MuJoCo walking check in a private, localhost-only ROS domain.

Checks zero feedback subscriptions, continuous position output, walking,
stop completion, actual displacement and torso tilt. Sends no hardware commands.
"""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--landings',type=int,default=6)
    parser.add_argument('--direction',choices=('forward','backward'),default='forward')
    parser.add_argument('--domain',type=int,default=219)
    parser.add_argument('--log-dir',type=Path)
    args=parser.parse_args()
    if args.landings<2 or not 1<=args.domain<=230:
        parser.error('use at least two landings and a private domain from 1 to 230')
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    os.environ.pop('ROS_LOCALHOST_ONLY',None)
    import rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import String,Float64MultiArray
    from nav_msgs.msg import Odometry

    logs=args.log_dir or Path(tempfile.mkdtemp(prefix='biped-mujoco-check-'))
    logs.mkdir(parents=True,exist_ok=True)
    launch_cmd=[shutil.which('ros2'),'launch','biped_mujoco','mujoco.launch.py','headless:=true','walk:=true']
    rclpy.init(domain_id=args.domain)
    node=rclpy.create_node('gait_mujoco_regression',parameter_overrides=[Parameter('use_sim_time',value=True)])
    latest={};times=[];peak=0.;first_x=last_x=None
    def status(message):
        latest.clear();latest.update(json.loads(message.data))
    def output(message):times.append(time.monotonic())
    def odom(message):
        nonlocal peak,first_x,last_x
        q=message.pose.pose.orientation
        up=1-2*(q.x*q.x+q.y*q.y)
        peak=max(peak,math.degrees(math.acos(max(-1.,min(1.,up)))))
        last_x=message.pose.pose.position.x
        if first_x is None:first_x=last_x
    node.create_subscription(String,'/walk/status',status,10)
    node.create_subscription(Float64MultiArray,'/position_controller/commands',output,10)
    node.create_subscription(Odometry,'/mujoco/odom',odom,10)
    cmd=node.create_publisher(String,'/walk_cmd',10)
    started=time.monotonic()
    with (logs/'launch.log').open('w') as launch_log:
        process=subprocess.Popen(launch_cmd,stdout=launch_log,stderr=subprocess.STDOUT,start_new_session=True)
        def wait(check,timeout):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.01)
                if process.poll() is not None:raise AssertionError('launch exited; see '+str(logs))
                if peak>6.:raise AssertionError(f'torso tilt {peak:.2f} degrees')
                if check():return
            raise AssertionError('timeout: '+json.dumps(latest))
        try:
            wait(lambda:latest.get('mode')=='simulation' and latest.get('state')=='idle' and first_x is not None,20)
            assert not times,'joint commands published before start'
            subscriptions=node.get_subscriber_names_and_types_by_node('quasi_static_walker','/')
            assert all(topic not in ('/imu','/joint_states','/diagnostics') for topic,_ in subscriptions),subscriptions
            assert latest['control']=='timed position loop',latest
            assert latest['foot_lift_m']==.010,latest
            assert latest['step_length_m']==.03,latest
            cmd.publish(String(data=args.direction))
            wait(lambda:latest.get('landings',0)>=args.landings,90+50*args.landings)
            cmd.publish(String(data='stop'))
            wait(lambda:latest.get('state')=='standing',70)
            held_since=node.get_clock().now().nanoseconds*1e-9
            wait(lambda:node.get_clock().now().nanoseconds*1e-9-held_since>=2.,5)
            sign=1 if args.direction=='forward' else -1
            assert sign*(last_x-first_x)>.015*args.landings,(first_x,last_x)
            result=dict(result='PASS',landings=latest['landings'],direction=args.direction,
                feedback_subscriptions=0,peak_tilt_deg=peak,displacement_m=last_x-first_x,
                max_observed_command_gap_s=max(b-a for a,b in zip(times,times[1:])),
                final_state=latest['state'],elapsed_wall_s=time.monotonic()-started,logs=str(logs))
            (logs/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result),flush=True)
        finally:
            node.destroy_node();rclpy.try_shutdown()
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait()


if __name__=='__main__':main()
