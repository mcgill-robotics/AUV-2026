# launch/thrust_mapper.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
        # No Launch configuration variables

        attitude_controller = Node(
                package='controls',
                executable='attitude_controller',
                name='attitude_controller',
                output='screen',
                parameters=[{
                    'control_loop_hz': 10.0, # hz
                    'P_ex': 1.0,
                    'P_ey': 1.0,
                    'P_ez': 1.0,
                    'P_wx': 1.0,
                    'P_wy': 1.0,
                    'P_wz': 1.0,
                    'buoyancy': 278.0, # Newtons
                    'r_bv': [0.0, 0.0, 0.023], # [m] From CAD Model     
                }]
        )
        return LaunchDescription([
                attitude_controller
        ])
        