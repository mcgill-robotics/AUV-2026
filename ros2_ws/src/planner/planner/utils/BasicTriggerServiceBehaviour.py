import geometry_msgs
import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from py_trees.common import Status, Access
from py_trees.blackboard import Client
from rclpy.action import ActionClient
from auv_msgs.action import AUVNavigate
from std_srvs.srv import Trigger
from controls.goal_helpers import move_robot_centric
from .ActionStatus import ActionStatus

class BasicTriggerServiceBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour represents a template behaviour used to create trigger service behaviours.

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        py_trees.blackboard.Client: blackboard        : the blackboard client for reading/writing sensors data
        """

        def __init__(
            self, 
            node,
            name = "ActionBehaviour",
            service_name = None
            ) -> None:
            """
            Initializes the node and blackboard client for this behaviour.

            Inputs: rclpy.node.Node             : node - the ROS2 node to use for subscribing to topics 
                    str                         : name - the name of the behaviour 
                    str                         : service_name - the name of the service to call

            Outputs: None
            """   
            super().__init__(name)
            self.node = node
            self.name = name
            self.blackboard = py_trees.blackboard.Client(name=self.name)
            self.node.service_client = self.node.create_client(Trigger, service_name)
            self.sent_service_request = False
            self.future = None

        def setup(self) -> None:
            """
            Description: Sets up keys on the blackboard that this behaviour will use.
            """
            # Check if the service is available, if not log an error and raise an exception
            if not self.node.service_client.wait_for_service(timeout_sec=2.0):
                self.node.get_logger().error(f"[{self.name}] Service not available.")
                raise RuntimeError(f"Service {self.node.service_client.srv_name} not available.")


        def initialise(self) -> None:
            """
            Description: Called every time this behavior transitions is not RUNNING. The function is used
            to reset the mission status to NOT_SENT whenever the Behaviour is not RUNNING.

            Inputs: None

            Outputs: None
            """
            # Reset field needed to keep track of goal
            self.sent_service_request = False
            self.future = None

        def update(self) -> py_trees.common.Status:
            """
            Description: This function is called every tick. It should contain the logic of the behaviour, and return a Status based on the result of that logic.

            Ibputs: None

            Outputs: py_trees.common.Status.SUCCESS if the behaviour succeeded, 
                        py_trees.common.Status.FAILURE if it failed, or 
                        py_trees.common.Status.RUNNING if it is still running.
                
            """
            # Set the request and send it asynchronously
            if not self.sent_service_request:
                self.node.get_logger().info(f"[{self.name}] Sending service request.")
                request = Trigger.Request()
                self.future = self.node.service_client.call_async(request)
                self.sent_service_request = True
                return py_trees.common.Status.RUNNING
            
            # Verify if the service response has been received
            if not self.future.done():
                return py_trees.common.Status.RUNNING
            
            # Verify if service is successful or not
            try:
                response = self.future.result()
                if response.success:
                    self.node.get_logger().info(f"[{self.name}] Service succeeded.")
                    return py_trees.common.Status.SUCCESS
                else:
                    self.node.get_logger().info(f"[{self.name}] Service failed.")
                    return py_trees.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(f"[{self.name}] Service call failed with exception: {e}")
                return py_trees.common.Status.FAILURE