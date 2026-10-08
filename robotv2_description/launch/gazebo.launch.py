import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = "robotv2_description"
    pkg_share = get_package_share_directory(pkg)
    xacro_file = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")
    controllers_yaml = os.path.join(pkg_share, "config", "controllers.yaml")

    world = LaunchConfiguration("world")
    spawn_z = LaunchConfiguration("spawn_z")
    rviz = LaunchConfiguration("rviz")
    rviz_config = os.path.join(pkg_share, "rviz", "display.rviz")

    args = [
        DeclareLaunchArgument(
            "world",
            default_value="flat_world_imu.sdf",
            description="World file in <pkg>/worlds. Default has the IMU system (publishes /imu). "
            "If Gazebo fails to start on your graphics setup use world:=flat_world.sdf (no IMU).",
        ),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.29",
            description="Spawn height of base_link (lowest foot point is 0.2845 m below it).",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Also open RViz (uses sim time).",
        ),
    ]

    # Lets Gazebo resolve package://robotv2_description/meshes/... (the mesh URIs in the URDF).
    # Without this the robot spawns with no visible/colliding geometry.
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        (existing + ":" if existing else "") + os.path.dirname(pkg_share),
    )

    # ParameterValue(..., value_type=str) is REQUIRED: otherwise the XML gets parsed as YAML and
    # robot_state_publisher dies before publishing /robot_description.
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " use_sim:=true",
                " controllers_file:=",
                controllers_yaml,
            ]
        ),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": ["-r ", PathJoinSubstitution([pkg_share, "worlds", world])],
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", "biped_v2", "-topic", "robot_description", "-z", spawn_z],
    )
    # Give Gazebo a few seconds to come up before asking it to spawn.
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_robot])

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
        output="screen",
    )
    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager-timeout", "60"],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
    )

    # spawn robot -> load broadcaster -> load position controller
    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster])
    )
    after_broadcaster = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[position_controller])
    )

    return LaunchDescription(
        args
        + [
            resource_path,
            gz_sim,
            robot_state_publisher,
            bridge,
            rviz_node,
            delayed_spawn,
            after_spawn,
            after_broadcaster,
        ]
    )
