from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import UnlessCondition, IfCondition
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
    
    # Launch argument to switch to keyboard input
    keyboard_arg = DeclareLaunchArgument(
        'use_keyboard',
        default_value='false',
        description='Use keyboard instead of gamepad for input'
    )
    
    # Joy node for gamepad input (only if not using keyboard)
    joy_node = Node(
        condition=UnlessCondition(LaunchConfiguration('use_keyboard')),
        package='joy',
        executable='game_controller_node',
        name='game_controller_node',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )
    
    # Note: keyboard_to_joy is NOT launched here because it requires a dedicated TTY.
    # When use_keyboard:=true, we simply omit the joy_node. 
    # The user must manually run: ros2 run teleop keyboard_to_joy
    
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
        keyboard_arg,
        LogInfo(
            condition=IfCondition(LaunchConfiguration('use_keyboard')),
            msg="[Teleop] Launching in KEYBOARD mode."
        ),
        LogInfo(
            condition=UnlessCondition(LaunchConfiguration('use_keyboard')),
            msg="[Teleop] Launching in GAMEPAD (Joystick) mode."
        ),
        joy_node,
        teleop_node,
    ])
