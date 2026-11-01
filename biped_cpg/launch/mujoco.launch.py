"""MuJoCo + small-lean CPG, with the original quasi-static walker disabled."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = Path(get_package_share_directory('robotv2_description'))
    me = Path(get_package_share_directory('biped_cpg'))
    robot = ParameterValue(Command([FindExecutable(name='xacro'), ' ',
                                   str(desc / 'urdf/robot.urdf.xacro'), ' use_sim:=false']), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('kp', default_value='50.0'),
        DeclareLaunchArgument('torque_fraction', default_value='0.5'),
        DeclareLaunchArgument('real_time_factor', default_value='1.0'),
        DeclareLaunchArgument('config', default_value=str(me / 'config/cpg.yaml')),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot, 'use_sim_time': True}]),
        # Use the active python3 environment, so a ROS-compatible MuJoCo virtual
        # environment can be used without changing or installing system packages.
        Node(package='biped_cpg', executable='cpg_mujoco', output='screen',
             prefix=[FindExecutable(name='python3'), ' '], parameters=[{
                 'robot_description': robot, 'package_dir': str(desc),
                 'headless': ParameterValue(LaunchConfiguration('headless'), value_type=bool),
                 'kp': ParameterValue(LaunchConfiguration('kp'), value_type=float),
                 'torque_fraction': ParameterValue(LaunchConfiguration('torque_fraction'), value_type=float),
                 'real_time_factor': ParameterValue(LaunchConfiguration('real_time_factor'), value_type=float)}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(me / 'launch/cpg.launch.py')),
                                 launch_arguments={'config': LaunchConfiguration('config'), 'sim': 'true'}.items()),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time': True}], arguments=['-d', str(desc / 'rviz/display.rviz')]),
    ])
