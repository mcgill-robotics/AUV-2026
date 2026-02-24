from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch teleop converter and joy node."""
    
    pkg_share = FindPackageShare('teleop')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'teleop.yaml'])
    
    # Launch argument to override deadman requirement
    deadman_arg = DeclareLaunchArgument(
        'require_deadman', 
        default_value='true',
        description='Require LB held to output force'
    )
    
    # Joy node for gamepad input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )
    
    # Teleop converter - loads config file, can override deadman via launch arg
    teleop_node = Node(
        package='teleop',
        executable='teleop_converter',
        name='teleop_converter',
        parameters=[
            config_file,  # Load YAML config first
            {'require_deadman': LaunchConfiguration('require_deadman')},  # Override
        ],
        output='screen'
    )
    
    return LaunchDescription([
        deadman_arg,
        joy_node,
        teleop_node,
    ])
