import py_trees
import py_trees_ros
import rclpy
from controls import navigation_client
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from . import SensorsBehaviour
from .TemplateBehaviour import TemplateBehaviour
from .missions.preQual.RectangleQualification import RectangleQualificationMission
from .missions.preQual.OrbitQualification import OrbitQualificationMission

# I like my ANSI colours :DDD
green_text = "\033[32m"
default_text = "\033[0m"

class RootTree(Node):
    """
    This node initializes the root of the behaviour tree and adds the main branches of the tree as children to the root.
    """
    def __init__(self):
        super().__init__("planner_root_tree")
        
        # Set the root of the tree
        root = py_trees.composites.Parallel("Root", policy=py_trees.common.ParallelPolicy.SuccessOnAll()) # SUCCESS_ON_ALL means the root will only return success if all children return success
        
        # Create navigation client instance as a singleton
        # This is to consolidate all information concerning actions into one node client
        self.navigation_client = navigation_client.NavigationClient(name="planner_nav_client")
        self.navigation_client_ongoing_goal = self.navigation_client.current_goal_handle # Store the goal handle of the currently active goal, if any, to allow for cancellation when a new goal is sent.
        
        self.blackboard = py_trees.blackboard.Client(name="RootTreeBlackboard")
        self.blackboard.register_key(key="/navigation_client/client", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/navigation_client/ongoing_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.navigation_client.client = self.navigation_client
        self.blackboard.navigation_client.ongoing_goal = self.navigation_client_ongoing_goal
                
        # Add the sensors behaviour as a child running in parallel to the rest of the tree.
        # This allows the rest of the tree to access the latest sensor data snapshot at each tick.
        sensors_reader = SensorsBehaviour.SensorsBehaviour(node=self, name="Sensors Reader")

        # Add other behaviour here as mission controls node, currently implemented a dummy leaf
        pre_qual = RectangleQualificationMission(self)
        pre_qual_orbit = OrbitQualificationMission(self)
        dummy = TemplateBehaviour(self, name="hi")
        root.add_children([sensors_reader, pre_qual])

        # Create the behaviour tree with the root and setup the tree and call the setup of the root to initialize and setup all the
        # children behaviours recursively. This will setup all blackboards and ros2 publishers/subscribers
        self.behaviour_tree = py_trees.trees.BehaviourTree(root=root)
        self.behaviour_tree.setup(timeout=15)

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
    action_client_node = planner_node.navigation_client # Get the navigation client node from the planner to spin it in parallel since it 
                                                        #has its own executor and needs to be spun to process action results and feedback
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(planner_node)
    executor.add_node(action_client_node)
    while rclpy.ok():
        try:
            executor.spin()
        except KeyboardInterrupt:
            planner_node.get_logger().info("Shutting down yaw BT node")
            executor.shutdown()
            action_client_node.destroy_node()
            planner_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()