from setuptools import setup
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition,UnlessCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():
        planner_dir = get_package_share_directory('planner')

        config = os.path.join(planner_dir, 'config', 'behaviour_tree_params.yaml')
        with open(config, 'r') as f:
                default_config:dict = yaml.safe_load(f)

        sim_arg = DeclareLaunchArgument(
                "sim",
                default_value=str(default_config["general"]["sim"]),
                description=(
                        "Whether to use simulation time. Should be true when running in simulation and false when running on the real AUV."
                )
        )

        use_ground_truth_arg = DeclareLaunchArgument(
                "use_ground_truth",
                default_value=str(default_config["general"]["use_ground_truth"]),
                description=(
                        "Whether to use ground truth pose and twist data from the simulator. Parameter is only relevant if sim is true."
                )
        )

        tick_rate_arg = DeclareLaunchArgument(
                "tick_rate",
                default_value=str(default_config["general"]["tick_rate"]),
                description=(
                        "The tick rate of the behaviour tree in Hz. Determines how often the tree is ticked and thus how often each behaviour is updated. Higher tick rates can lead to more responsive behaviour but more compute usage."
                )
        )

        planner_node = Node(
                package="planner",
                executable="planner_node",
                parameters=[
                        {
                                "sim": LaunchConfiguration("sim"),
                                "use_ground_truth": LaunchConfiguration("use_ground_truth"),
                                "tick_rate": LaunchConfiguration("tick_rate")
                        }
                ],
        )

        launch_description = LaunchDescription()
        launch_description.add_action(sim_arg)
        launch_description.add_action(use_ground_truth_arg)
        launch_description.add_action(tick_rate_arg)

        launch_description.add_action(planner_node)

        return launch_description