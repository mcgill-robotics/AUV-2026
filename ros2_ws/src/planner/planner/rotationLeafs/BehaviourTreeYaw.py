import time
import py_trees
import rclpy
from rclpy.node import Node
from .YawMovementBehaviour import YawMovement
from .YawReaderBehaviour import YawReader
from .YawUserInput import userInputYaw

class BehaviourTreeYaw(Node):

    def __init__(self):
        super().__init__("YawNode")
        self.root = py_trees.composites.Sequence("YawControl", memory=False)
        set_yaw = userInputYaw(self)
        check_yaw = YawReader(self)
        move_yaw = YawMovement(self)

        parallel = py_trees.composites.Parallel(
                "RotateUntilReached",
                policy=py_trees.common.ParallelPolicy.SuccessOnOne()
            )
        parallel.add_children([check_yaw, move_yaw])
        self.root.add_children([set_yaw, parallel])
        
        # Wrap in BehaviourTree
        self.tree = py_trees.trees.BehaviourTree(self.root)
        self.tree.setup()

        self.get_logger().info("Yaw Behaviour Tree Node Initialized")
        self.timer = self.create_timer(1.0, self.tick_tree)  # tick every 1s

    def tick_tree(self):
        self.tree.tick()
        self.get_logger().info("Yaw Behaviour Tree Tickeddd")        

def main():
    rclpy.init()
    node = BehaviourTreeYaw()
    while rclpy.ok():
        try:
            rclpy.spin(node)
        
        except KeyboardInterrupt:
            node.get_logger().info("Shutting down yaw BT node")
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()


