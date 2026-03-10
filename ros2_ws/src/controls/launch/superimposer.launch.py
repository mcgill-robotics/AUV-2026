from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        # No launch configuration variables

        superimposer = Node(
                package='controls',
                executable='superimposer',
                name='superimposer',
                output='screen',
                parameters=[{
                        'effort_bias_force_x': 0.0,
                        'effort_bias_force_y': 0.0,
                        'effort_bias_force_z': 0.0,
                        'effort_bias_torque_x': 0.0,
                        'effort_bias_torque_y': 0.0,
                        'effort_bias_torque_z': 0.0,
                        'publish_hz': 20.0
                }]
        )
        return LaunchDescription([
                superimposer
        ])