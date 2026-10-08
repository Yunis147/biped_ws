"""Everything for SIMULATION in one command:  ros2 launch biped_bringup sim.launch.py
Add  walk:=true  to also start the walking node (otherwise start it yourself later with
ros2 launch biped_gait walk.launch.py sim:=true — useful when you want to restart just the gait).
Other arguments pass straight through to gazebo.launch.py: world, rviz, spawn_z, x, y, z, yaw."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    walk = LaunchConfiguration("walk")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("robotv2_description"), "launch", "gazebo.launch.py"])))
    walker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("biped_gait"), "launch", "walk.launch.py"])),
        launch_arguments={"sim": "true"}.items(), condition=IfCondition(walk))
    return LaunchDescription([
        DeclareLaunchArgument("walk", default_value="false", description="also start the walking node"),
        gazebo, walker,
    ])
