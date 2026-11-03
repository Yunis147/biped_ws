"""Repeatable ROS/MuJoCo walking check, isolated from the robot's ROS domain.

Run from a sourced workspace with MuJoCo available. --stress delays sensor
delivery by 20 ms and inserts a 40 ms gyro-only burst after the first landing.
This is a synthetic communication/sensor regression, not a recorded robot bag.
"""
import argparse
from collections import deque
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
    parser.add_argument('--landings',type=int,default=12)
    parser.add_argument('--stress',action='store_true')
    parser.add_argument('--domain',type=int,default=209)
    parser.add_argument('--log-dir',type=Path)
    args=parser.parse_args()
    if args.landings<2 or not 1<=args.domain<=230:
        parser.error('use at least two landings and a private domain from 1 to 230')
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    os.environ.pop('ROS_LOCALHOST_ONLY',None)
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu,JointState
    from std_msgs.msg import String,Float64MultiArray
    from nav_msgs.msg import Odometry

    logs=args.log_dir or Path(tempfile.mkdtemp(prefix='biped-mujoco-check-'))
    logs.mkdir(parents=True,exist_ok=True)
    launch_cmd=[shutil.which('ros2'),'launch','biped_mujoco','mujoco.launch.py','headless:=true','walk:=true']
    if args.stress:
        launch_file=logs/'stress.launch.py'
        launch_file.write_text('''from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
def generate_launch_description():
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('biped_mujoco'))/'launch/mujoco.launch.py')),launch_arguments={'headless':'true','walk':'false'}.items())
    walk=Node(package='biped_gait',executable='quasi_static_walk',output='screen',parameters=[str(Path(share('biped_gait'))/'config/walk.yaml'),{'use_sim_time':True}],remappings=[('/imu','/gait_test/imu'),('/joint_states','/gait_test/joint_states')])
    return LaunchDescription([sim,walk])
''')
        launch_cmd=[shutil.which('ros2'),'launch',str(launch_file)]

    rclpy.init(domain_id=args.domain)
    node=rclpy.create_node('gait_mujoco_regression',parameter_overrides=[Parameter('use_sim_time',value=True)])
    latest={};times=[];peak=0.;first_x=last_x=None
    transfer_start=None;burst_start=None;burst_samples=0
    pending=deque()
    def now():return node.get_clock().now().nanoseconds*1e-9
    def status(message):
        nonlocal transfer_start
        latest.clear();latest.update(json.loads(message.data))
        if latest.get('segment')==['place_right','lean_right'] and transfer_start is None:
            transfer_start=now()
    # Receipt timing can include observer scheduling delays. These messages have
    # no source timestamp, so this does not establish a publisher deadline.
    def output(message):times.append(time.monotonic())
    def odom(message):
        nonlocal peak,first_x,last_x
        q=message.pose.pose.orientation
        roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
        pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
        peak=max(peak,math.degrees(max(abs(roll),abs(pitch))))
        last_x=message.pose.pose.position.x
        if first_x is None:first_x=last_x
    node.create_subscription(String,'/walk/status',status,10)
    node.create_subscription(Float64MultiArray,'/position_controller/commands',output,10)
    node.create_subscription(Odometry,'/mujoco/odom',odom,10)
    cmd=node.create_publisher(String,'/walk_cmd',10)
    if args.stress:
        imu_pub=node.create_publisher(Imu,'/gait_test/imu',qos_profile_sensor_data)
        joint_pub=node.create_publisher(JointState,'/gait_test/joint_states',qos_profile_sensor_data)
        def enqueue(message,publisher):
            nonlocal burst_start,burst_samples
            stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            if isinstance(message,Imu) and transfer_start is not None:
                if burst_start is None and now()-transfer_start>=1.:
                    burst_start=stamp
                if burst_start is not None and 0<=stamp-burst_start<.04:
                    planned=latest.get('planned_rate_rad_s',[0.,0.])
                    message.angular_velocity.x=planned[0]-.0728
                    message.angular_velocity.y=planned[1]-.0700
                    burst_samples+=1
            pending.append((stamp+.02,publisher,message))
        node.create_subscription(Imu,'/imu',lambda m:enqueue(m,imu_pub),qos_profile_sensor_data)
        node.create_subscription(JointState,'/joint_states',lambda m:enqueue(m,joint_pub),qos_profile_sensor_data)
        def deliver():
            while pending and pending[0][0]<=now():
                _,publisher,message=pending.popleft();publisher.publish(message)
        node.create_timer(.005,deliver)

    started=time.monotonic()
    with (logs/'launch.log').open('w') as launch_log:
        process=subprocess.Popen(launch_cmd,stdout=launch_log,stderr=subprocess.STDOUT,start_new_session=True)
        def wait(check,timeout):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.01)
                if process.poll() is not None:raise AssertionError('launch exited; see '+str(logs))
                if latest.get('fault') or latest.get('state') in ('recovering','placement_hold','transfer_hold'):
                    raise AssertionError(json.dumps(latest))
                if peak>6.:raise AssertionError(f'torso tilt {peak:.2f} degrees')
                if check():return
            raise AssertionError('timeout: '+json.dumps(latest))
        try:
            wait(lambda:latest.get('mode')=='simulation' and latest.get('hardware_ready'),20)
            assert not times,'joint commands published before start'
            cmd.publish(String(data='start'))
            wait(lambda:latest.get('landings',0)>=args.landings,90+35*args.landings)
            cmd.publish(String(data='stop'))
            wait(lambda:latest.get('state')=='standing',70)
            assert latest.get('transfer_backoffs')==0,latest
            assert last_x-first_x>.015,(first_x,last_x)
            launch_log.flush()
            pauses=(logs/'launch.log').read_text().count('weight transfer paused for rotation')
            if args.stress:
                assert burst_samples>=2,burst_samples
                assert pauses>0,'gyro burst did not exercise the rate pause'
            result=dict(result='PASS',landings=latest['landings'],stress=args.stress,
                peak_tilt_deg=peak,displacement_m=last_x-first_x,
                max_command_gap_s=max(b-a for a,b in zip(times,times[1:])),
                rate_pauses=pauses,burst_samples=burst_samples,final_state=latest['state'],
                elapsed_wall_s=time.monotonic()-started,logs=str(logs))
            (logs/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result),flush=True)
        finally:
            try:
                node.destroy_node();rclpy.try_shutdown()
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    try:process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL);process.wait()


if __name__=='__main__':main()
