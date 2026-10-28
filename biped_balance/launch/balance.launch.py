"""Start only the new balance experiment; existing hardware/gait launch files are unchanged."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([FindPackageShare('biped_balance'), 'config', 'balance.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config, description='New balance package parameter file'),
        DeclareLaunchArgument('actuate', default_value='false', description='Allow actuator tests after an explicit stand command'),
        DeclareLaunchArgument('imu_axes_verified', default_value='false', description='IMU mounting was checked on the supported robot'),
        DeclareLaunchArgument('record_path', default_value='', description='Optional new CSV file; existing files are not overwritten'),
        Node(package='biped_balance', executable='balance_node', output='screen',
             parameters=[LaunchConfiguration('config'), {
                 'actuate': ParameterValue(LaunchConfiguration('actuate'), value_type=bool),
                 'imu_axes_verified': ParameterValue(LaunchConfiguration('imu_axes_verified'), value_type=bool),
                 'record_path': ParameterValue(LaunchConfiguration('record_path'), value_type=str),
             }]),
    ])
