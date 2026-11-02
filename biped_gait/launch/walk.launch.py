"""Start the walking node.   Simulation:  ros2 launch biped_gait walk.launch.py sim:=true
                            Real robot:  ros2 launch biped_gait walk.launch.py
Then:  ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'start'}"     (and 'stop')"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    config = str(Path(get_package_share_directory("biped_gait"))/"config/walk.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config),
        DeclareLaunchArgument("sim", default_value="false", description="true = follow the Gazebo clock (/clock)"),
        Node(package="biped_gait", executable="quasi_static_walk", output="screen",
             parameters=[LaunchConfiguration("config"), {"use_sim_time": ParameterValue(sim, value_type=bool)}]),
    ])
