"""Everything for the REAL ROBOT in one command:  ros2 launch biped_bringup hardware.launch.py
Torque stays OFF (see docs/CALIBRATION.md and the top-level README before enabling it).
Add  walk:=true  to also start the walking node.  mock:=true runs with simulated servos/IMU (no hardware)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock = LaunchConfiguration("mock")
    walk = LaunchConfiguration("walk")
    hw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("biped_hardware"), "launch", "hardware.launch.py"])),
        launch_arguments={"mock": mock, "rviz": LaunchConfiguration("rviz")}.items())
    walker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("biped_gait"), "launch", "walk.launch.py"])),
        launch_arguments={"sim": "false"}.items(), condition=IfCondition(walk))
    return LaunchDescription([
        DeclareLaunchArgument("mock", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("walk", default_value="false"),
        hw, walker,
    ])
