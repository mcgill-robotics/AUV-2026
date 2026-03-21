import py_trees
import py_trees_ros
import py_trees_ros.action_clients
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from . import SensorsBehaviour
from . import TemplateBehaviour
from . import CustomCallbackBehaviour
import math
import functools
from auv_msgs.action import AUVNavigate
from controls.goal_helpers import set_depth, set_global_yaw, move_global


# I like my ANSI colours :DDD
green_text = "\033[32m"
default_text = "\033[0m"

# Helper to create a simple navigation leaf using the native PyTrees ROS action client
def nav_leaf(name, goal):
    return py_trees_ros.action_clients.FromConstant(
        name=name,
        action_type=AUVNavigate,
        action_name="/motion/navigate",
        action_goal=goal,
    )

class RootTree(Node):
    """
    This node initializes the root of the behaviour tree and adds the main branches of the tree as children to the root.
    """
    def __init__(self):
        super().__init__("RootTreeNode")
        
        # Set the root of the tree and navigation client instance 
        root = py_trees.composites.Parallel("Root", policy=py_trees.common.ParallelPolicy.SuccessOnAll()) # SUCCESS_ON_ALL means the root will only return success if all children return success
        
        self.blackboard = py_trees.blackboard.Client(name="RootTreeBlackboard")
        
        # Add the sensors behaviour as a child running in parallel to the rest of the tree.
        # This allows the rest of the tree to access the latest sensor data snapshot at each tick.
        sensors_reader = SensorsBehaviour.SensorsBehaviour(node=self, name="Sensors Reader")

        # Add other behaviour here as mission controls node, currently implemented a dummy leaf
        dummy_leaf = TemplateBehaviour.TemplateBehaviour(node=self, name="Dummy Leaf")
        
        # Build the full mission sequence
        mission_sequence = py_trees.composites.Sequence("Mission Sequence", memory=True)
        
        # 1. Dive to -1.5m
        dive_leaf = nav_leaf("Dive", set_depth(z=-1.5, tolerance=0.15, hold_time=2.0))
        
        # 2. Orbit 360 degrees
        callback_leaf = CustomCallbackBehaviour.CustomCallback(node=self, name="Orbit", rotations_segments=8, angle_to_rotate_deg=360, radius_to_rotate_meter=1.5, clockwise=True)
        
        # 3. Turn 180 degrees to look at the gate
        turn_leaf = nav_leaf("Turn 180", set_global_yaw(yaw_rad=math.pi, tolerance=0.1, hold_time=1.0))
        
        # 4. Return to origin (X=0, Y=0) at the current depth
        return_leaf = nav_leaf("Return to Origin", move_global(x=0.0, y=0.0, do_z=False, tolerance=0.5, hold_time=1.0))
        
        # 5. Ascend to surface
        ascend_leaf = nav_leaf("Ascend to Surface", set_depth(z=0.0, tolerance=0.15, hold_time=1.0))
        
        mission_sequence.add_children([dive_leaf, callback_leaf, turn_leaf, return_leaf, ascend_leaf])

        root.add_children([sensors_reader, mission_sequence])

        # Create the behaviour tree with the root and setup the tree and call the setup of the root to initialize and setup all the
        # children behaviours recursively. This will setup all blackboards and ros2 publishers/subscribers
        self.behaviour_tree = py_trees.trees.BehaviourTree(root=root)
        print(py_trees.display.unicode_tree(root=root))
        self.behaviour_tree.setup(timeout=15, node=self)
        self.declare_parameter("tick_rate", 1.0)
        self.tick_rate = self.get_parameter("tick_rate").get_parameter_value().double_value
        self.timer = self.create_timer(self.tick_rate, self.tick_tree)  # tick every tick_rate seconds

        # Debug log to confirm initialization, can remove but I like my colours :D
        self.get_logger().info(f"{green_text}Yaw Behaviour Tree Node Initialized{default_text}")

    def tick_tree(self):
        self.behaviour_tree.tick()
    

def main():
    rclpy.init()
    planner_node = RootTree()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(planner_node)
    while rclpy.ok():
        try:
            executor.spin()
        except KeyboardInterrupt:
            planner_node.get_logger().info("Shutting down yaw BT node")
            executor.shutdown()
            planner_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()