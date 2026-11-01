"""Start only the CPG. Run bringup first; use sim:=true for an existing simulator."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    config = str(Path(get_package_share_directory('biped_cpg')) / 'config/cpg.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('sim', default_value='false', description='true = simulator clock; false = hardware with servo feedback checks'),
        Node(package='biped_cpg', executable='cpg_walk', name='biped_cpg', output='screen',
             parameters=[LaunchConfiguration('config'), {'use_sim_time': ParameterValue(LaunchConfiguration('sim'), value_type=bool)}]),
    ])
