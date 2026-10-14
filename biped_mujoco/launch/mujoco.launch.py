"""The MuJoCo simulation of the BipedRobot V2.

    ros2 launch biped_mujoco mujoco.launch.py                       # MuJoCo window + physics
    ros2 launch biped_mujoco mujoco.launch.py walk:=true rviz:=true # + the walking node + RViz
    ros2 launch biped_mujoco mujoco.launch.py headless:=true        # no window

Needs the Python package:  pip install --user --break-system-packages mujoco
The simulator publishes /clock, so every other node must run with use_sim_time:=true (walk:=true does that).
Servo / contact settings live in config/mujoco.yaml; to try one value without editing the file, run the node by hand:
    ros2 run biped_mujoco mujoco_biped --ros-args -p kp:=45.0 -p torque_fraction:=0.5   (plus robot_description, see README)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory("robotv2_description")
    me = get_package_share_directory("biped_mujoco")
    xacro_file = os.path.join(desc_share, "urdf", "robot.urdf.xacro")

    headless, rviz, walk = (LaunchConfiguration(k) for k in ("headless", "rviz", "walk"))
    args = [
        DeclareLaunchArgument("headless", default_value="false", description="true = no MuJoCo window"),
        DeclareLaunchArgument("rviz", default_value="false", description="open RViz"),
        DeclareLaunchArgument("walk", default_value="false", description="also start the walking node (sim time)"),
    ]

    # the same xacro output goes to robot_state_publisher (for RViz/TF) and to the simulator (to build the MuJoCo model)
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file, " use_sim:=false"]), value_type=str)

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_description, "use_sim_time": True}])

    sim = Node(package="biped_mujoco", executable="mujoco_biped", output="screen",
               parameters=[os.path.join(me, "config", "mujoco.yaml"),
                           {"robot_description": robot_description, "package_dir": desc_share,
                            "headless": ParameterValue(headless, value_type=bool)}])

    rviz_node = Node(package="rviz2", executable="rviz2", output="screen", parameters=[{"use_sim_time": True}],
                     arguments=["-d", os.path.join(desc_share, "rviz", "display.rviz")], condition=IfCondition(rviz))

    walker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("biped_gait"), "launch", "walk.launch.py")),
        launch_arguments={"sim": "true"}.items(), condition=IfCondition(walk))

    return LaunchDescription(args + [rsp, sim, rviz_node, walker])
