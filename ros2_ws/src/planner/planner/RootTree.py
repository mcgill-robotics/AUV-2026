import py_trees
import py_trees_ros
import rclpy
from controls import navigation_client
from rclpy.node import Node
from . import SensorsBehaviour
from . import TemplateBehaviour


# I like my ANSI colours :DDD
green_text = "\033[32m"
default_text = "\033[0m"

class RootTree(Node):
    """
    This node initializes the root of the behaviour tree and adds the main branches of the tree as children to the root.
    """
    def __init__(self):
        super().__init__("RootTreeNode")
        
        # Set the root of the tree and navigation client instance 
        root = py_trees.composites.Parallel("Root", policy=py_trees.common.ParallelPolicy.SuccessOnAll()) # SUCCESS_ON_ALL means the root will only return success if all children return success
        self.navigation_client = navigation_client.NavigationClient(name="NavigationClientNode")
        
        self.blackboard = py_trees.blackboard.Client(name="RootTreeBlackboard")
        self.blackboard.register_key(key="/navigation_client", access=py_trees.common.Access.WRITE)
        self.blackboard.navigation_client = self.navigation_client
                
        # Add the sensors behaviour as a child running in parallel to the rest of the tree.
        # This allows the rest of the tree to access the latest sensor data snapshot at each tick.
        sensors_reader = SensorsBehaviour.SensorsBehaviour(node=self, name="Sensors Reader")

        # Add other behaviour here as mission controls node, currently implemented a dummy leaf
        dummy_leaf = TemplateBehaviour.TemplateBehaviour(node=self, name="Dummy Leaf")
        root.add_children([sensors_reader, dummy_leaf])

        # Create the behaviour tree with the root and setup the tree and call the setup of the root to initialize and setup all the
        # children behaviours recursively. This will setup all blackboards and ros2 publishers/subscribers
        self.behaviour_tree = py_trees.trees.BehaviourTree(root=root)
        print(py_trees.display.unicode_tree(root=root))
        self.behaviour_tree.setup(timeout=15)

        self.declare_parameter("tick_rate", 1.0)
        self.tick_rate = self.get_parameter("tick_rate").get_parameter_value().double_value
        self.timer = self.create_timer(self.tick_rate, self.tick_tree)  # tick every tick_rate seconds

        # Debug log to confirm initialization, can remove but I like my colours :D
        self.get_logger().info(f"{green_text}Yaw Behaviour Tree Node Initialized{default_text}")

    def tick_tree(self):
        self.behaviour_tree.tick()
        # For debug:
        self.get_logger().info("Yaw Behaviour Tree Tickeddd")  

def main():
    rclpy.init()
    node = RootTree()
    while rclpy.ok():
        try:
            rclpy.spin(node)
        
        except KeyboardInterrupt:
            node.get_logger().info("Shutting down yaw BT node")
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()