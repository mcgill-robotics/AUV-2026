import math
import py_trees
from auv_msgs.msg import VisionObjectArray
from controls.goal_helpers import rotate_relative
from .action_status_enum import ActionStatus

class SearchSweepBehaviour(py_trees.behaviour.Behaviour):
    """
    Rotates the AUV in a full 360-degree sweep, divided into `num_steps`. 
    After each turn step, it pauses for `step_timeout` seconds to let vision stabilize.
    If it completes `max_attempts` full 360-degree sweeps without finding `target_class`, it fails.
    If the object is seen at any point, it returns SUCCESS immediately.
    """
    def __init__(self, target_class: str, num_steps: int = 5, max_attempts: int = 2, step_timeout: float = 2.0, clockwise: bool = False, name="SearchSweep"):
        super().__init__(name)
        self.target_class = target_class
        self.num_steps = num_steps
        self.max_attempts = max_attempts
        self.step_timeout = step_timeout
        self.clockwise = clockwise
        
        # Calculate how much to turn per step (in radians)
        self.sweep_angle_rad = (2 * math.pi) / float(num_steps)
        if self.clockwise:
            self.sweep_angle_rad = -self.sweep_angle_rad
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        
        # State tracking
        self.current_attempt = 0
        self.current_step = 0
        self.action_status = ActionStatus.NOT_SENT
        self.sent_goal = False
        
        # Pause tracking
        self.is_pausing = False
        self.pause_start_time = 0.0

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0) 
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)

    def initialise(self):
        # Reset state when this behavior starts
        self.current_attempt = 0
        self.current_step = 0
        self.action_status = ActionStatus.NOT_SENT
        self.sent_goal = False
        self.is_pausing = False

    def update(self):
        # 1. LIVE LOGIC: Check the blackboard for the target object right now
        if hasattr(self.blackboard, 'vision') and self.blackboard.vision.object_map is not None:
            for obj in self.blackboard.vision.object_map.array:
                if obj.label == self.target_class:
                    self.node.get_logger().info(f"[{self.name}] Found target '{self.target_class}' in vision!")
                    # Cancel any active turn since we found what we were looking for
                    if self.action_status == ActionStatus.PENDING:
                        self.navigation_client.reset_action_client()
                    return py_trees.common.Status.SUCCESS
        
        # 2. STATE MACHINE: Object not found. Manage the turning sequence.
        
        # If we are currently executing a turn, keep waiting.
        if self.action_status == ActionStatus.PENDING:
            return py_trees.common.Status.RUNNING
            
        # If the turn failed unexpectedly
        if self.action_status == ActionStatus.FAILED:
            self.node.get_logger().error(f"[{self.name}] Turn action failed midway.")
            return py_trees.common.Status.FAILURE
            
        # If we just finished a turn step, we need to pause
        if self.action_status == ActionStatus.SUCCEEDED:
            if not self.is_pausing:
                self.is_pausing = True
                self.pause_start_time = self.node.get_clock().now().nanoseconds / 1e9
                self.node.get_logger().info(f"[{self.name}] Step complete. Pausing for {self.step_timeout}s to stabilize vision.")
            
            # Check if pause time has elapsed
            elapsed = (self.node.get_clock().now().nanoseconds / 1e9) - self.pause_start_time
            if elapsed >= self.step_timeout:
                self.is_pausing = False
                self.action_status = ActionStatus.NOT_SENT
                self.sent_goal = False
                self.current_step += 1
                
                # Check if we just completed a full 360
                if self.current_step >= self.num_steps:
                    self.current_step = 0
                    self.current_attempt += 1
                    self.node.get_logger().info(f"[{self.name}] Completed full 360 sweep {self.current_attempt}/{self.max_attempts}.")
            
            # Return RUNNING so we keep ticking and checking vision while paused
            return py_trees.common.Status.RUNNING
            
        # If we are ready to send a new turn step command
        if self.action_status == ActionStatus.NOT_SENT:
            if self.current_attempt >= self.max_attempts:
                self.node.get_logger().info(f"[{self.name}] Max 360 attempts ({self.max_attempts}) reached. Target not found.")
                return py_trees.common.Status.FAILURE
                
            self.node.get_logger().info(f"[{self.name}] Turning {math.degrees(self.sweep_angle_rad):.1f} deg (Sweep {self.current_attempt+1}/{self.max_attempts}, Step {self.current_step+1}/{self.num_steps})")
            
            # Send the next relative yaw turn (with a large timeout to ensure it has time to physically turn)
            goal = rotate_relative(dyaw_rad=self.sweep_angle_rad, hold_time=self.step_timeout, tolerance=0.175*3, timeout=30.0)
            
            self.navigation_client.send_navigation_goal(
                goal, 
                self.name, 
                self.on_server_goal_response, 
                self.on_server_goal_result
            )
            self.action_status = ActionStatus.PENDING
            self.sent_goal = True
            
            return py_trees.common.Status.RUNNING

    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.action_status = ActionStatus.FAILED

    def on_server_goal_result(self, goal_success: bool):
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.action_status = ActionStatus.FAILED

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            if hasattr(self, 'node') and self.node:
                self.node.get_logger().warn(f"[{self.name}] Aborted. Canceling sweep turn.")
            if hasattr(self, 'navigation_client') and self.navigation_client:
                self.navigation_client.reset_action_client()

