from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------

    mock = LaunchConfiguration("mock")
    port = LaunchConfiguration("port")
    rviz = LaunchConfiguration("rviz")
    imu = LaunchConfiguration("imu")
    torque_on_start = LaunchConfiguration("torque_on_start")
    startup_position = LaunchConfiguration("startup_position")
    calibration_file = LaunchConfiguration("calibration_file")

    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------

    description_share = FindPackageShare("robotv2_description")

    xacro_file = PathJoinSubstitution(
        [
            description_share,
            "urdf",
            "robot.urdf.xacro",
        ]
    )

    rviz_config = PathJoinSubstitution(
        [
            description_share,
            "rviz",
            "robot.rviz",
        ]
    )

    # ------------------------------------------------------------------
    # Robot description
    #
    # Hardware mode:
    #   use_sim:=false
    #
    # This gives robot_state_publisher the same URDF used by the
    # hardware robot, without Gazebo.
    # ------------------------------------------------------------------

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ",
            "use_sim:=false",
        ]
    )

    # ------------------------------------------------------------------
    # robot_state_publisher
    # ------------------------------------------------------------------

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": False,
            }
        ],
    )

    # ------------------------------------------------------------------
    # Real servo bridge
    #
    # servo_bridge:
    #   - discovers the two serial boards
    #   - communicates with all 12 ST3215 servos
    #   - publishes /joint_states
    #   - subscribes to /position_controller/commands
    #
    # torque_on_start:
    #   false -> safest default
    #
    # startup_position:
    #   true -> enable torque and move to calibrated zero/midpoint
    # ------------------------------------------------------------------

    servo_bridge = Node(
        package="biped_hardware",
        executable="servo_bridge",
        name="servo_bridge",
        output="screen",
        parameters=[
            {
                "mock": mock,
                "port": port,
                "torque_on_start": torque_on_start,
                "startup_position": startup_position,
                "calibration_file": calibration_file,
                "use_sim_time": False,
            }
        ],
    )

    # ------------------------------------------------------------------
    # BNO055 IMU
    # ------------------------------------------------------------------

    imu_node = Node(
        package="biped_hardware",
        executable="imu_node",
        name="imu_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
            }
        ],
        condition=IfCondition(imu),
    )

    # ------------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------------

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
        ],
        parameters=[
            {
                "use_sim_time": False,
            }
        ],
        condition=IfCondition(rviz),
    )

    # ------------------------------------------------------------------
    # Launch description
    # ------------------------------------------------------------------

    return LaunchDescription(
        [

            # Arguments
            DeclareLaunchArgument(
                "mock",
                default_value="false",
                description="Use simulated servo/IMU hardware instead of real hardware.",
            ),

            DeclareLaunchArgument(
                "port",
                default_value="",
                description=(
                    "Serial port. Leave empty to use automatic two-board "
                    "servo bus discovery."
                ),
            ),

            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz.",
            ),

            DeclareLaunchArgument(
                "imu",
                default_value="true",
                description="Start the BNO055 IMU node.",
            ),

            DeclareLaunchArgument(
                "torque_on_start",
                default_value="false",
                description=(
                    "Enable servo torque immediately at startup. "
                    "Normally leave this false."
                ),
            ),

            DeclareLaunchArgument(
                "startup_position",
                default_value="false",
                description=(
                    "Enable torque and move the robot smoothly to the "
                    "calibrated zero/midpoint pose."
                ),
            ),

            DeclareLaunchArgument(
                "calibration_file",
                default_value="",
                description=(
                    "Calibration YAML file. Empty uses "
                    "~/.config/biped/calibration.yaml."
                ),
            ),

            # Nodes
            robot_state_publisher,
            servo_bridge,
            imu_node,
            rviz_node,
        ]
    )