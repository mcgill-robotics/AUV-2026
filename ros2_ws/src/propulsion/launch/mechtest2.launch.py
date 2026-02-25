# mechtest2.launch.py 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # This finds the 'propulsion' package's share directory and locates the launch file within it.
    propulsion_share_dir = get_package_share_directory('propulsion')
    propulsion_launch_file = os.path.join(propulsion_share_dir, 'launch', 'propulsion.launch.py')

    # --- 2. Launch Arguments ---
    sim_arg     = DeclareLaunchArgument('sim', default_value='false')
    record_arg  = DeclareLaunchArgument('record', default_value='true')
    
   
    gx_offset_arg = DeclareLaunchArgument(
        'gx', default_value=TextSubstitution(text='0.0'),
        description='Center of Mass offset in X-axis (Surge/Sway coupling tuning).'
    )
    gy_offset_arg = DeclareLaunchArgument(
        'gy', default_value=TextSubstitution(text='0.0'),
        description='Center of Mass offset in Y-axis (Sway/Roll coupling tuning).'
    )

    # This includes and runs everything inside propulsion.launch.py
    propulsion_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(propulsion_launch_file),
        launch_arguments={
            # Pass the launch arguments through to the included launch file
            'sim': LaunchConfiguration('sim'),
            'record': LaunchConfiguration('record'),
            
            # The included thrust_mapper node will read these
            'gx_offset': LaunchConfiguration('gx'),
            'gy_offset': LaunchConfiguration('gy'),
        }.items(),
    )
    
    # --- MechTest Node ---
    mechtest_node = Node(
        package='propulsion',
        executable='mechtest_exec', 
        name='allocation_test_node',
        output='screen'
    )

    # --- 5. Return Launch Description ---
    return LaunchDescription([
        sim_arg,
        record_arg,
        gx_offset_arg, 
        gy_offset_arg, 
        
        propulsion_launch_include, 
        mechtest_node,             
    ])