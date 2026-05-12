import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    planner_dir = get_package_share_directory('planner')
    config = os.path.join(planner_dir, 'config', 'behaviour_tree_params.yaml')

    # Load default config to get default values for launch arguments if needed
    with open(config, 'r') as f:
        default_config = yaml.safe_load(f)

    # We keep 'sim' as a launch argument because it's frequently toggled from the CLI
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value=str(default_config["planner_root_tree"]["ros__parameters"]["sim"]),
        description="Whether to use simulation time."
    )

    use_ground_truth_arg = DeclareLaunchArgument(
        "use_ground_truth",
        default_value=str(default_config["planner_root_tree"]["ros__parameters"]["use_ground_truth"]),
        description="Whether to use ground truth pose and twist data (sim only)."
    )

    planner_node = Node(
        package="planner",
        executable="planner_node",
        parameters=[
            config, # Load all parameters from hierarchical YAML
            {
                "sim": LaunchConfiguration("sim"),
                "use_ground_truth": LaunchConfiguration("use_ground_truth"),
            }
        ],
    )

    return LaunchDescription([
        sim_arg,
        use_ground_truth_arg,
        planner_node
    ])