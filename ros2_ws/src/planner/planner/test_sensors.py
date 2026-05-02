# Python dependencies
import py_trees

# ROS dependencies
import geometry_msgs
import py_trees_ros
import rclpy
from rclpy.node import Node
import geometry_msgs.msg

# AUV dependencies
import auv_msgs.msg

# Planner dependencies


class SensorsBehaviour(py_trees.composites.Parallel):
    """
    This Behaviour subscribes to the AUV's pose, twist and object_map topics and writes 
    the latest messages to the blackboard for other behaviours to use.
    """    


    def __init__(self, node):
        super().__init__("SensorsBehaviour", \
                         policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL(synchronise=False))
        self.node = node

        use_sim = self.node.get_parameter("sim").get_parameter_value().bool_value
        use_ground_truth = self.node.get_parameter("use_ground_truth").get_parameter_value().bool_value
        
        # If in sim and desire to use absolute pose and twist, subscribe to ground truth topics
        #(since use_ground_truth is only valid if in sim, but in sim you might still wanna use state estimation topics)
        if use_sim and use_ground_truth: 
                topic_pose = "/auv/ground_truth/pose"
                topic_twist = "/auv/ground_truth/twist"

        # Subscriber to pose 
        pose_blackboard_writer = py_trees_ros.subscribers.ToBlackboard(
            name="PoseSubscriber",
            topic_name=topic_pose,
            topic_type=geometry_msgs.msg.PoseStamped,
            blackboard_variables={"sensors": "pose"}
        )

        # Subscriber to twist
        twist_blackboard_writer = py_trees_ros.subscribers.ToBlackboard(
            name="TwistSubscriber",
            topic_name=topic_twist,
            topic_type=geometry_msgs.msg.TwistStamped,
            blackboard_variables={"sensors": "twist"},
        )

        # Subscriber to object map
        object_map_blackboard_writer = py_trees_ros.subscribers.ToBlackboard(
            name="ObjectMapSubscriber",
            topic_name="/vision/object_map",
            topic_type=auv_msgs.msg.VisionObjectArray,
            blackboard_variables={"vision": "object_map"},
        )

        self.add_children([
            pose_blackboard_writer,
            twist_blackboard_writer,
            object_map_blackboard_writer
        ])