# Python dependencies
import math
import py_trees
import time

# ROS dependencies
import py_trees_ros
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import UInt8

# AUV dependencies
from auv_msgs.action import AUVNavigate
from auv_msgs.srv import RosbagControl

# Planner dependencies
from .action_status_enum import ActionStatus
from controls.goal_helpers import (
        _make_goal,
        _DEFAULT_POS_TOL,
        _DEFAULT_ANGULAR_TOL,
        _DEFAULT_HOLD,
        _DEFAULT_TIMEOUT,
        set_global_yaw,
        move_robot_centric,
        translate_field_centric,
)
from controls.utils import quaternion_from_yaw, yaw_from_quaternion
from geometry_msgs.msg import Pose, Point, Quaternion

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class BasicActionBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour represents a template behaviour used to create others.

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        self.attach_blackboard_client: blackboard        : the blackboard client for reading/writing sensors data
        """

        def __init__(
            self, 
            name = "ActionBehaviour",
            goal: AUVNavigate = None,
            ) -> None:
            """
            Initializes the node and blackboard client for this behaviour.

            Inputs: str                         : name - the name of the behaviour 
                    auv_msgs.action.AUVNavigate : goal - the goal to send to the action client

            Outputs: None
            """   
            super().__init__(name)
            self.name = name
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.goal = goal
            
            # Initialize fields needed to keep track of goal
            self.sent_goal = False
            self.action_status = ActionStatus.NOT_SENT # Either succeeded, pending, failed or not sent
            self.is_waiting_for_result = False
            self.result_message = ""
            
        def setup(self, **kwargs) -> None:
            """
            Description: Sets up keys on the blackboard that this behaviour will use.
            """
            self.node = kwargs['node']
            self.navigation_client = kwargs['shared_nav_client']
            
            # Ensure the action server is ready
            self.navigation_client.client_wait_for_server(timeout_sec=5.0) 

            self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        
        def initialise(self) -> None:
            """
            Description: Called every time this behavior transitions is not RUNNING. The function is used
            to reset the mission status to NOT_SENT whenever the Behaviour is not RUNNING.

            Inputs: None

            Outputs: None
            """
            # Reset field needed to keep track of goal
            self.action_status = ActionStatus.NOT_SENT
            self.result_message = ""
                
        def update(self) -> py_trees.common.Status:
            """
            Description: This function is called every tick. It should contain the logic of the behaviour, and return a Status based on the result of that logic.

            Ibputs: None

            Outputs: py_trees.common.Status.SUCCESS if the behaviour succeeded, 
                        py_trees.common.Status.FAILURE if it failed, or 
                        py_trees.common.Status.RUNNING if it is still running.
                
            """
            # Check if pose is properly registered on the blackboard as controls requires
            # a pose value. Block execution if the AUV poses are not published yet
            if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
                self.node.get_logger().info(f"[{self.name}] Waiting for sensor pose data...", throttle_duration_sec=2.0)
                return py_trees.common.Status.RUNNING
                
            # Check for failure condition from the async callbacks (goal response and goal result)
            if self.action_status is ActionStatus.FAILED:
                self.node.get_logger().error(f"[{self.name}] Action failed: {self.result_message}")
                return py_trees.common.Status.FAILURE
                
            # Completion check
            if self.action_status is ActionStatus.SUCCEEDED:
                self.node.get_logger().info(f"[{self.name}] Completed goal. {self.result_message}")
                return py_trees.common.Status.SUCCESS

            # Block loop if currently navigating to a waypoint
            if self.action_status is ActionStatus.PENDING:
                return py_trees.common.Status.RUNNING
                
            # Send the goal if no goals are ongoing and set the mission status to pending
            self.node.get_logger().info(f"[{self.name}] Sent goal.")
            self.navigation_client.send_navigation_goal(self.goal, self.name, self.on_server_goal_response, self.on_server_goal_result)
            self.action_status = ActionStatus.PENDING
            return py_trees.common.Status.RUNNING
        
        def on_server_goal_response(self, goal_response: bool) -> None:
            """
            Description: This function provides customized responses to the 
            action server's decision on accepting the goal or not. In this case,
            the custom implementation updates the status of the mission. Pass this
            function as an input to the navigation_client.send_navigation_goal

            Inputs: goal_response: str, The client will call this function with true upon acceptance, and false upon rejection from the server

            Outputs: None
            """
            if not goal_response:
                self.action_status = ActionStatus.FAILED

        def on_server_goal_result(self, goal_success: bool, message: str) -> None:
            """
            Description: This function provides customized logic to be executed when
            the goal is finished. In this case, the custom implementation updates the status of the mission
            depending on whether or not the goal was successful or failed. Pass this function as an input 
            to the navigation_client.send_navigation_goal

            Inputs: goal_success: bool, The client will call this function with true upon success, and false upon failure
                    message: str, The result message from the action server

            Outputs: None
            """
            self.result_message = message
            if goal_success:
                self.action_status = ActionStatus.SUCCEEDED
            else:
                self.action_status = ActionStatus.FAILED



class BasicTriggerServiceBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour represents a template behaviour used to create trigger service behaviours.

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        self.attach_blackboard_client: blackboard        : the blackboard client for reading/writing sensors data
        """

        def __init__(
            self, 
            name = "ActionBehaviour",
            service_name = None,
            timeout_sec: float = 10.0,
            max_retries: int = 3
            ) -> None:
            """
            Initializes the node and blackboard client for this behaviour.

            Inputs: str                         : name - the name of the behaviour 
                    str                         : service_name - the name of the service to call
                    float                       : timeout_sec - max seconds to wait for a response (default: 10.0)
                    int                         : max_retries - max number of attempts before permanent failure (default: 3)

            Outputs: None
            """   
            super().__init__(name)
            self.name = name
            self.service_name = service_name
            self.timeout_sec = timeout_sec
            self.max_retries = max_retries
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.sent_service_request = False
            self.future = None
            self.request_time = None
            self.attempt_count = 0
            self._exhausted_logged = False

        def setup(self, **kwargs) -> None:
            """
            Description: Sets up keys on the blackboard that this behaviour will use.
            """
            self.node = kwargs['node']
            self.service_client = self.node.create_client(Trigger, self.service_name)
            
            # Check if the service is available at startup
            if not self.service_client.wait_for_service(timeout_sec=2.0):
                self.node.get_logger().warn(f"[{self.name}] Service '{self.service_name}' not yet available. Will retry when triggered.")


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
            self.request_time = None

        def update(self) -> py_trees.common.Status:
            """
            Description: This function is called every tick. It should contain the logic of the behaviour, and return a Status based on the result of that logic.

            Ibputs: None

            Outputs: py_trees.common.Status.SUCCESS if the behaviour succeeded, 
                        py_trees.common.Status.FAILURE if it failed, or 
                        py_trees.common.Status.RUNNING if it is still running.
                
            """
            # Check if max retries exhausted
            if self.attempt_count >= self.max_retries and not self.sent_service_request:
                if not self._exhausted_logged:
                    self.node.get_logger().error(f"[{self.name}] Max retries ({self.max_retries}) exhausted. Giving up.")
                    self._exhausted_logged = True
                return py_trees.common.Status.FAILURE

            # Send the request asynchronously
            if not self.sent_service_request:
                self.attempt_count += 1
                self.node.get_logger().info(f"[{self.name}] Sending service request (attempt {self.attempt_count}/{self.max_retries}).")
                request = Trigger.Request()
                self.future = self.service_client.call_async(request)
                self.sent_service_request = True
                self.request_time = self.node.get_clock().now()
                return py_trees.common.Status.RUNNING
            
            # Still waiting for response
            if not self.future.done():
                # Check for timeout
                elapsed = (self.node.get_clock().now() - self.request_time).nanoseconds / 1e9
                if elapsed >= self.timeout_sec:
                    self.node.get_logger().error(f"[{self.name}] Service call timed out after {self.timeout_sec:.1f}s")
                    self.future.cancel()
                    # Reset to allow retry on the next tick (stays RUNNING)
                    self.sent_service_request = False
                    return py_trees.common.Status.RUNNING
                return py_trees.common.Status.RUNNING
            
            # Check if the service call itself failed (e.g. middleware error)
            if self.future.exception() is not None:
                self.node.get_logger().error(f"[{self.name}] Service call failed: {self.future.exception()}")
                # Reset to allow retry on the next tick (stays RUNNING)
                self.sent_service_request = False
                return py_trees.common.Status.RUNNING
            
            # Verify if service is successful or not
            response = self.future.result()
            if response.success:
                self.node.get_logger().info(f"[{self.name}] Service succeeded.")
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().info(f"[{self.name}] Service failed: {response.message}")
                return py_trees.common.Status.FAILURE



class TimerBehaviour(py_trees.behaviour.Behaviour):
        """
        This behaviour represents a timer to be implemented. The timer is dictated by measuring a time.time()
        interval and checking if it is above a desired threshold upon a new tick's arrival. This not fully accurate  
        method is acceptable since the current scope of this TimerBehaviour is to allow time to untether Douglas

        Fields: 
        rclpy.node.Node: node                         : the ros2 node for subscribing to topics
        self.attach_blackboard_client: blackboard        : the blackboard client for reading/writing sensors data
        """

        def __init__(self, timer_duration: float, name="sensorsLeaf") -> None:
                """
                Initializes the node and blackboard client for this behaviour.

                Inputs: str                : name - the name of the behaviour 
                        float              : timer_duration - the desired duration of the timer

                Outputs: None
                """   
                super().__init__(name)
                self.timer_duration = timer_duration
                self.start_time = 0.0
                self.timer_started = False

        def setup(self, **kwargs) -> None:
                self.node = kwargs['node']
        
        def initialise(self) -> None:
                """
                Whenever the Behaviour is not in RUNNING state, reset the timer to not started

                Inputs: None

                Outputs: None
                """
                self.timer_started = False

        def update(self) -> py_trees.common.Status:
                """
                Description: This function is called every tick. It should contain the logic of the behaviour, and return a Status based on the result of that logic.

                Ibputs: None

                Outputs: py_trees.common.Status.SUCCESS if the behaviour succeeded, 
                         py_trees.common.Status.FAILURE if it failed, or 
                         py_trees.common.Status.RUNNING if it is still running.
                 
                """
                if not self.timer_started:
                        # Set the initial time at start of time counting
                        self.start_time = self.node.get_clock().now().nanoseconds / 1e9
                        self.timer_started = True
                        return py_trees.common.Status.RUNNING
                
                current_time = self.node.get_clock().now().nanoseconds / 1e9
                # Upon each tick, check if interval of time has been passed
                self.node.get_logger().info(f"Time passed: {(current_time - self.start_time)}")
                if (current_time - self.start_time) > float(self.timer_duration):
                        return py_trees.common.Status.SUCCESS
                
                return py_trees.common.Status.RUNNING

class RosbagRecordingDecorator(py_trees.decorators.Decorator):
    """
    A decorator that automatically starts a rosbag recording when the decorated mission
    starts, and stops the recording when the mission ends (whether success, failure, or interrupted).
    """
    def __init__(self, child: py_trees.behaviour.Behaviour, profile: str = "all", bag_name: str = "", service_path: str = "/rosbag_manager/control") -> None:
        super().__init__(name="Rosbag Recording Decorator", child=child)
        self.profile = profile
        self.bag_name = bag_name
        self.service_path = service_path
        self.started_recording = False
        
    def setup(self, **kwargs) -> None:
        self.node = kwargs['node']
        self.service_client = self.node.create_client(RosbagControl, self.service_path)
        
    def initialise(self) -> None:
        if self.service_client.wait_for_service(timeout_sec=1.0):
            req = RosbagControl.Request()
            req.action = RosbagControl.Request.START_RECORD
            req.profile = self.profile
            req.bag_name = self.bag_name
            
            self.node.get_logger().info(f"[{self.name}] Starting auto-recording (profile: {self.profile}, bag: {self.bag_name})...")
            future = self.service_client.call_async(req)
            
            def done_callback(f):
                try:
                    response = f.result()
                    if response.success:
                        self.started_recording = True
                    else:
                        self.node.get_logger().info(f"[{self.name}] Auto-record didn't start: {response.message} (Is a bag already recording?)")
                except Exception as e:
                    self.node.get_logger().error(f"[{self.name}] Service call failed: {e}")
                    
            future.add_done_callback(done_callback)
        else:
            self.node.get_logger().warn(f"[{self.name}] {self.service_path} service not available. Skipping auto-record.")
            
    def update(self) -> py_trees.common.Status:
        return self.decorated.status
        
    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status != py_trees.common.Status.RUNNING and self.started_recording:
            if self.service_client.wait_for_service(timeout_sec=1.0):
                req = RosbagControl.Request()
                req.action = RosbagControl.Request.STOP_RECORD
                
                self.node.get_logger().info(f"[{self.name}] Stopping auto-recording...")
                self.service_client.call_async(req)
            self.started_recording = False

class SetNodeParameterBehaviour(py_trees.behaviour.Behaviour):
        """
        Calls a ROS2 node's ~/set_parameters service to dynamically change a parameter.
        """
        def __init__(self, node_name: str, param_name: str, param_value, name="SetNodeParameter", max_attempts: int = 1):
                super().__init__(name)
                self.target_node = node_name
                self.param_name = param_name
                self.param_value = param_value
                self.max_attempts = max_attempts
                
                self.service_client = None
                self.future = None
                self.request_sent = False
                self.attempt_count = 0

        def setup(self, **kwargs):
                self.node = kwargs['node']
                self.service_client = self.node.create_client(SetParameters, f'{self.target_node}/set_parameters')
                
        def initialise(self):
                self.future = None
                self.request_sent = False
                self.attempt_count = 0
                
        def update(self):
                if not self.request_sent:
                        if not self.service_client.wait_for_service(timeout_sec=1.0):
                                self.attempt_count += 1
                                if self.attempt_count >= self.max_attempts:
                                        self.node.get_logger().warn(f"[{self.name}] Service {self.target_node}/set_parameters not available after {self.max_attempts} attempts (controls offline?). Proceeding without setting parameter.")
                                        return py_trees.common.Status.SUCCESS
                                self.node.get_logger().warn(f"[{self.name}] Service {self.target_node}/set_parameters not available (attempt {self.attempt_count}/{self.max_attempts}).")
                                return py_trees.common.Status.RUNNING
                                
                        request = SetParameters.Request()
                        param = Parameter()
                        param.name = self.param_name
                        val = ParameterValue()
                        
                        if isinstance(self.param_value, bool):
                                val.type = ParameterType.PARAMETER_BOOL
                                val.bool_value = self.param_value
                        elif isinstance(self.param_value, int):
                                val.type = ParameterType.PARAMETER_INTEGER
                                val.integer_value = self.param_value
                        elif isinstance(self.param_value, float):
                                val.type = ParameterType.PARAMETER_DOUBLE
                                val.double_value = self.param_value
                        elif isinstance(self.param_value, str):
                                val.type = ParameterType.PARAMETER_STRING
                                val.string_value = self.param_value
                                
                        param.value = val
                        request.parameters = [param]
                        
                        self.future = self.service_client.call_async(request)
                        self.request_sent = True
                        self.node.get_logger().info(f"[{self.name}] Setting {self.param_name}={self.param_value} on {self.target_node}")
                        return py_trees.common.Status.RUNNING
                        
                if not self.future.done():
                        return py_trees.common.Status.RUNNING
                        
                if self.future.exception() is not None:
                        self.node.get_logger().error(f"[{self.name}] SetParameters failed: {self.future.exception()}")
                        return py_trees.common.Status.FAILURE
                        
                response = self.future.result()
                if not response.results[0].successful:
                        self.node.get_logger().error(f"[{self.name}] Failed to set parameter: {response.results[0].reason}")
                        return py_trees.common.Status.FAILURE
                        
                self.node.get_logger().info(f"[{self.name}] Successfully set parameter.")
                return py_trees.common.Status.SUCCESS


class WaitForTriggerBehaviour(py_trees.behaviour.Behaviour):
        """
        Waits until a ROS 2 service on the planner node is called to trigger the mission.
        """
        def __init__(self, service_name: str = "~/start_mission", name="Wait For Trigger"):
                super().__init__(name)
                self.service_name = service_name
                self.triggered = False
                self.srv = None
                
        def setup(self, **kwargs):
                self.node = kwargs['node']
                self.srv = self.node.create_service(Trigger, self.service_name, self.trigger_callback)
                self.node.get_logger().info(f"[{self.name}] Hosted trigger service at {self.service_name}")
                
        def trigger_callback(self, request, response):
                self.triggered = True
                response.success = True
                response.message = "Mission Triggered!"
                self.node.get_logger().info(f"[{self.name}] Received trigger via service call.")
                return response
                
        def initialise(self):
                # We don't reset triggered here, because once triggered, it should stay triggered
                # unless explicitly reset for a new run.
                pass
                
        def update(self):
                if self.triggered:
                        return py_trees.common.Status.SUCCESS
                return py_trees.common.Status.RUNNING


class SetActuatorBehaviour(py_trees.behaviour.Behaviour):
        """
        Publish a UInt8 command to an actuator topic (e.g. /actuators/grabber or /actuators/torpedo).
        """
        def __init__(self, topic_name: str, command_value: int, name: str = None):
                if name is None:
                        name = f"Set {topic_name} -> {command_value}"
                super().__init__(name)
                self.topic_name = topic_name
                self.command_value = int(command_value)
                self.publisher = None
                self.fallback_publisher = None

        def setup(self, **kwargs):
                self.node = kwargs['node']
                self.publisher = self.node.create_publisher(UInt8, self.topic_name, 10)
                if self.topic_name.startswith("/actuators/"):
                        fallback_topic = self.topic_name.replace("/actuators/", "/actuator/", 1)
                        self.fallback_publisher = self.node.create_publisher(UInt8, fallback_topic, 10)
                elif self.topic_name.startswith("/actuator/"):
                        fallback_topic = self.topic_name.replace("/actuator/", "/actuators/", 1)
                        self.fallback_publisher = self.node.create_publisher(UInt8, fallback_topic, 10)
                self.node.get_logger().info(f"[{self.name}] Created publisher for {self.topic_name}")

        def update(self) -> py_trees.common.Status:
                if self.publisher is None:
                        self.node.get_logger().error(f"[{self.name}] Publisher not initialized for {self.topic_name}")
                        return py_trees.common.Status.FAILURE
                msg = UInt8(data=self.command_value)
                self.publisher.publish(msg)
                if self.fallback_publisher is not None:
                        self.fallback_publisher.publish(msg)
                self.node.get_logger().info(f"[{self.name}] Published {self.command_value} to {self.topic_name}")
                return py_trees.common.Status.SUCCESS


class TaskTransitionBehaviour(py_trees.behaviour.Behaviour):
        """
        Optional transition step between mission tasks.
        If enabled=False (or dx/dy/yaw are unset/zero), returns SUCCESS immediately.
        If enabled=True, rotates to an absolute target_yaw_deg and/or translates horizontally by (dx, dy)
        using goal_helpers (set_global_yaw, move_robot_centric, translate_field_centric).
        """
        def __init__(
                self,
                enabled: bool = False,
                do_yaw: bool = False,
                target_yaw_deg: float = 0.0,
                dx: float = 0.0,
                dy: float = 0.0,
                frame: str = "field",
                position_tolerance: float = _DEFAULT_POS_TOL,
                angular_tolerance: float = _DEFAULT_ANGULAR_TOL,
                hold_time: float = _DEFAULT_HOLD,
                timeout: float = _DEFAULT_TIMEOUT,
                name: str = "Task Transition",
        ) -> None:
                super().__init__(name)
                self.enabled = bool(enabled)
                self.do_yaw = bool(do_yaw)
                self.target_yaw_deg = float(target_yaw_deg) if target_yaw_deg is not None else 0.0
                self.dx = float(dx)
                self.dy = float(dy)
                self.frame = str(frame).lower()
                self.position_tolerance = position_tolerance
                self.angular_tolerance = angular_tolerance
                self.hold_time = hold_time
                self.timeout = timeout
                self.blackboard = self.attach_blackboard_client(name=self.name)
                self.action_status = ActionStatus.NOT_SENT
                self.result_message = ""
                self.phase = 0  # 0: yaw turn, 1: translation

        def setup(self, **kwargs) -> None:
                self.node = kwargs["node"]
                self.navigation_client = kwargs["shared_nav_client"]
                self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)

        def initialise(self) -> None:
                self.action_status = ActionStatus.NOT_SENT
                self.result_message = ""
                self.phase = 0

        def update(self) -> py_trees.common.Status:
                if not self.enabled:
                        return py_trees.common.Status.SUCCESS

                do_yaw = self.do_yaw
                do_pos = (abs(self.dx) > 1e-4 or abs(self.dy) > 1e-4)

                if not do_yaw and not do_pos:
                        return py_trees.common.Status.SUCCESS

                if self.action_status == ActionStatus.SUCCEEDED:
                        if self.phase == 0 and do_yaw and do_pos:
                                # Yaw turn finished; now start horizontal translation phase
                                self.phase = 1
                                self.action_status = ActionStatus.NOT_SENT
                        else:
                                return py_trees.common.Status.SUCCESS

                if self.action_status == ActionStatus.FAILED:
                        return py_trees.common.Status.FAILURE
                if self.action_status == ActionStatus.PENDING:
                        return py_trees.common.Status.RUNNING

                if self.action_status == ActionStatus.NOT_SENT:
                        if self.phase == 0 and do_yaw:
                                target_yaw_rad = math.radians(float(self.target_yaw_deg))
                                goal = set_global_yaw(
                                        yaw_rad=target_yaw_rad,
                                        tolerance=self.angular_tolerance,
                                        hold_time=self.hold_time,
                                        timeout=self.timeout,
                                )
                                self.node.get_logger().info(f"[{self.name}] Phase 0: set_global_yaw({self.target_yaw_deg} deg)")
                        else:
                                self.phase = 1
                                if self.frame == "field":
                                        goal = translate_field_centric(
                                                dx=self.dx,
                                                dy=self.dy,
                                                dz=0.0,
                                                tolerance=self.position_tolerance,
                                                hold_time=self.hold_time,
                                                timeout=self.timeout,
                                        )
                                        self.node.get_logger().info(f"[{self.name}] Phase 1: translate_field_centric({self.dx}, {self.dy})")
                                else:
                                        goal = move_robot_centric(
                                                forward=self.dx,
                                                sway=self.dy,
                                                heave=0.0,
                                                tolerance=self.position_tolerance,
                                                hold_time=self.hold_time,
                                                timeout=self.timeout,
                                        )
                                        self.node.get_logger().info(f"[{self.name}] Phase 1: move_robot_centric({self.dx}, {self.dy})")

                        self.navigation_client.send_navigation_goal(
                                goal, self.name, self._on_goal_response, self._on_goal_result
                        )
                        self.action_status = ActionStatus.PENDING
                        return py_trees.common.Status.RUNNING

                return py_trees.common.Status.RUNNING

        def _on_goal_response(self, goal_response: bool) -> None:
                if not goal_response:
                        self.action_status = ActionStatus.FAILED

        def _on_goal_result(self, goal_success: bool, message: str) -> None:
                self.result_message = message
                self.action_status = ActionStatus.SUCCEEDED if goal_success else ActionStatus.FAILED

