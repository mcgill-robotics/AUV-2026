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

        # ------ Mission Action Parameters  -----------------------------------------------
        pre_qual_yaw_tolerance_arg = DeclareLaunchArgument(
                "pre_qual_yaw_tolerance",
                default_value=str(default_config["general"]["pre_qual_yaw_tolerance"]),
                description=(
                        "The general yaw tolerance for prequal planner missions"
                )
        )

        pre_qual_positional_tolerance_arg = DeclareLaunchArgument(
                "pre_qual_positional_tolerance",
                default_value=str(default_config["general"]["pre_qual_positional_tolerance"]),
                description=(
                        "The general positional tolerance for prequal planner missions"
                )
        )

        pre_qual_hold_time_arg = DeclareLaunchArgument(
                "pre_qual_hold_time",
                default_value=str(default_config["general"]["pre_qual_hold_time"]),
                description=(
                        "The general hold time for prequal planner missions' actions"
                )
        )

        pre_qual_timeout_arg = DeclareLaunchArgument(
                "pre_qual_timeout",
                default_value=str(default_config["general"]["pre_qual_timeout"]),
                description=(
                       "The general timeout for prequal planner missions's actions"
                )
        )

        orbit_pre_qual_yaw_scale_arg = DeclareLaunchArgument(
                "orbit_pre_qual_yaw_scale",
                default_value=str(default_config["general"]["orbit_pre_qual_yaw_scale"]),
                description=(
                        "The yaw tolerance scale for the orbit prequal planner missions" \
                        "This is a scale since the orbit radius may change. As such the tolerances change with" \
                        "it. We only control the tolerance relative to segments of the orbit, hence the scale"
                )
        )

        orbit_pre_qual_positional_scale_arg = DeclareLaunchArgument(
                "orbit_pre_qual_positional_scale",
                default_value=str(default_config["general"]["orbit_pre_qual_positional_scale"]),
                description=(
                        "The positional tolerance scale for the orbit prequal planner missions" \
                        "This is a scale since the orbit radius may change. As such the tolerances change with" \
                        "it. We only control the tolerance relative to segments of the orbit, hence the scale"
                )
        )

        orbit_pre_qual_hold_time_initial_arg = DeclareLaunchArgument(
                "orbit_pre_qual_hold_time_initial",
                default_value=str(default_config["general"]["orbit_pre_qual_hold_time_initial"]),
                description=(
                        "The hold time for the first and last Action setpoints. These are meant to be" \
                        "stricter than the hold times for the individual segments of the circle arc, as we want to anchor" \
                        "Dougie back without momentum."
                )
        )

        orbit_pre_qual_hold_time_segments_arg = DeclareLaunchArgument(
                "orbit_pre_qual_hold_time_segments",
                default_value=str(default_config["general"]["orbit_pre_qual_hold_time_segments"]),
                description=(
                       "The hold time for the first and last Action setpoints. These are meant to be" \
                        "stricter than the hold times for the individual segments of the circle arc, as we want to anchor" \
                        "Dougie back without momentum."
                )
        )

        #------------------ Set up the node ---------------
        planner_node = Node(
                package="planner",
                executable="planner_node",
                parameters=[
                        {
                                "sim": LaunchConfiguration("sim"),
                                "use_ground_truth": LaunchConfiguration("use_ground_truth"),
                                "tick_rate": LaunchConfiguration("tick_rate"),
                                "pre_qual_yaw_tolerance": LaunchConfiguration("pre_qual_yaw_tolerance"),
                                "pre_qual_positional_tolerance": LaunchConfiguration("pre_qual_positional_tolerance"),
                                "pre_qual_hold_time": LaunchConfiguration("pre_qual_hold_time"),
                                "pre_qual_timeout": LaunchConfiguration("pre_qual_timeout"),
                                "orbit_pre_qual_yaw_scale": LaunchConfiguration("orbit_pre_qual_yaw_scale"),
                                "orbit_pre_qual_positional_scale": LaunchConfiguration("orbit_pre_qual_positional_scale"),
                                "orbit_pre_qual_hold_time_initial": LaunchConfiguration("orbit_pre_qual_hold_time_initial"),
                                "orbit_pre_qual_hold_time_segments": LaunchConfiguration("orbit_pre_qual_hold_time_segments"),

                        }
                ],
        )

        launch_description = LaunchDescription()
        launch_description.add_action(sim_arg)
        launch_description.add_action(use_ground_truth_arg)
        launch_description.add_action(tick_rate_arg)
        launch_description.add_action(pre_qual_yaw_tolerance_arg)
        launch_description.add_action(pre_qual_positional_tolerance_arg)
        launch_description.add_action(pre_qual_hold_time_arg)
        launch_description.add_action(pre_qual_timeout_arg)
        launch_description.add_action(orbit_pre_qual_yaw_scale_arg)
        launch_description.add_action(orbit_pre_qual_positional_scale_arg)
        launch_description.add_action(orbit_pre_qual_hold_time_initial_arg)
        launch_description.add_action(orbit_pre_qual_hold_time_segments_arg)
        launch_description.add_action(planner_node)

        return launch_description