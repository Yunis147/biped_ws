import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("robotv2_description")
    xacro_file = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")
    rviz_config = os.path.join(pkg_share, "rviz", "display.rviz")

    # value_type=str stops ROS from trying to parse the URDF XML as YAML.
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]), value_type=str
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
