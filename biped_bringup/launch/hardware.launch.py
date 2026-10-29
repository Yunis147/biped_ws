
"""Real robot bringup.

Torque stays off by default. Walking is disabled by default.
Use mock:=true for simulated hardware.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock = LaunchConfiguration("mock")
    rviz = LaunchConfiguration("rviz")
    walk = LaunchConfiguration("walk")
    imu = LaunchConfiguration("imu")
    port = LaunchConfiguration("port")
    torque_on_start = LaunchConfiguration("torque_on_start")
    startup_position = LaunchConfiguration("startup_position")

    hw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("biped_hardware"),
                "launch",
                "hardware.launch.py",
            ])
        ),
        launch_arguments={
            "mock": mock,
            "rviz": rviz,
            "imu": imu,
            "port": port,
            "torque_on_start": torque_on_start,
            "startup_position": startup_position,
        }.items(),
    )

    walker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("biped_gait"),
                "launch",
                "walk.launch.py",
            ])
        ),
        launch_arguments={"sim": "false"}.items(),
        condition=IfCondition(walk),
    )

    return LaunchDescription([
        DeclareLaunchArgument("mock", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("walk", default_value="false"),
        DeclareLaunchArgument("imu", default_value="true"),
        DeclareLaunchArgument("port", default_value=""),
        DeclareLaunchArgument("torque_on_start", default_value="true"),
        DeclareLaunchArgument("startup_position", default_value="true"),
        hw,
        walker,
    ])
