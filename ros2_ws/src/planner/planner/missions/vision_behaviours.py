import math
import py_trees
from controls.goal_helpers import set_global_yaw, look_at
from controls.utils import yaw_from_quaternion, normalize_angle
from .action_status_enum import ActionStatus

class SearchSweepBehaviour(py_trees.behaviour.Behaviour):
    """
    Rotates the AUV in a full 360-degree sweep, divided into `num_steps`. 
    After each turn step, it pauses for `step_timeout` seconds to let vision stabilize.
    If it completes `max_attempts` full 360-degree sweeps without finding `target_class`, it fails.
    If the object is seen at any point, it returns SUCCESS immediately.
    """
    def __init__(self, target_class: str, num_steps: int = 5, max_attempts: int = 2, step_timeout: float = 2.0, clockwise: bool = False, look_at_on_success: bool = True, name="SearchSweep"):
        super().__init__(name)
        self.target_class = target_class
        self.num_steps = num_steps
        self.max_attempts = max_attempts
        self.step_timeout = step_timeout
        self.clockwise = clockwise
        self.look_at_on_success = look_at_on_success
        
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
        
        # Absolute angle tracking
        self.start_yaw = None
        
        # Success alignment tracking
        self.is_looking_at_target = False
        self.target_found_pos = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0) 
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)

    def initialise(self):
        # Reset state when this behavior starts
        self.current_attempt = 0
        self.current_step = 0
        self.action_status = ActionStatus.NOT_SENT
        self.sent_goal = False
        self.is_pausing = False
        self.start_yaw = None
        self.is_looking_at_target = False
        self.target_found_pos = None

    def update(self):
        # 1. LIVE LOGIC: Check the blackboard for the target object right now
        # Skip this if we already found it and are just finishing the "look at" turn
        if not self.is_looking_at_target:
            if hasattr(self.blackboard, 'vision') and self.blackboard.vision.object_map is not None:
                for obj in self.blackboard.vision.object_map.array:
                    if obj.label == self.target_class:
                        self.node.get_logger().info(f"[{self.name}] Found target '{self.target_class}' in vision!")
                        
                        # Cancel any active sweep turn
                        if self.action_status == ActionStatus.PENDING:
                            self.navigation_client.reset_action_client()
                        
                        if self.look_at_on_success:
                            self.node.get_logger().info(f"[{self.name}] Transitioning to final alignment with {self.target_class}.")
                            self.is_looking_at_target = True
                            self.target_found_pos = (obj.pose.position.x, obj.pose.position.y)
                            self.action_status = ActionStatus.NOT_SENT
                            self.sent_goal = False
                            # Continue update() to send the look_at goal immediately
                        else:
                            return py_trees.common.Status.SUCCESS
        
        # 2. STATE MACHINE: Manage turns (either sweep steps or final alignment)
        
        # If we are in the "look at" phase
        if self.is_looking_at_target:
            if self.action_status == ActionStatus.SUCCEEDED:
                self.node.get_logger().info(f"[{self.name}] Final alignment complete. Target centered.")
                return py_trees.common.Status.SUCCESS
            
            if self.action_status == ActionStatus.FAILED:
                self.node.get_logger().error(f"[{self.name}] Final alignment turn failed.")
                return py_trees.common.Status.FAILURE
            
            if self.action_status == ActionStatus.NOT_SENT:
                # Fetch current position to calculate the look_at yaw
                if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
                    return py_trees.common.Status.RUNNING
                
                auv_x = self.blackboard.sensors.pose.pose.position.x
                auv_y = self.blackboard.sensors.pose.pose.position.y
                
                goal = look_at(
                    target_x=self.target_found_pos[0],
                    target_y=self.target_found_pos[1],
                    current_x=auv_x,
                    current_y=auv_y,
                    hold_time=self.step_timeout # Hold a bit to let vision settle on target
                )
                
                self.navigation_client.send_navigation_goal(goal, self.name, self.on_server_goal_response, self.on_server_goal_result)
                self.action_status = ActionStatus.PENDING
                self.sent_goal = True
            
            return py_trees.common.Status.RUNNING

        # 3. SWEEP LOGIC: Object not found yet. Manage the turning sequence.
        
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
                
            # Grab the starting yaw once when the sweep first begins
            if self.start_yaw is None:
                if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
                    self.node.get_logger().warn(f"[{self.name}] Waiting for /sensors/pose to determine starting yaw.")
                    return py_trees.common.Status.RUNNING
                current_quat = self.blackboard.sensors.pose.pose.orientation
                self.start_yaw = yaw_from_quaternion(current_quat)
                
            # Compute absolute target angle for this step
            target_yaw = self.start_yaw + (self.sweep_angle_rad * (self.current_step + 1))
            # Normalize to [-pi, pi]
            target_yaw = normalize_angle(target_yaw)
                
            self.node.get_logger().info(f"[{self.name}] Turning to {math.degrees(target_yaw):.1f} deg (Sweep {self.current_attempt+1}/{self.max_attempts}, Step {self.current_step+1}/{self.num_steps})")
            
            # Send the absolute yaw turn (with a large timeout to ensure it has time to physically turn)
            # We use a tiny hold_time because the behavior itself handles the stabilization pause.
            goal = set_global_yaw(yaw_rad=target_yaw, hold_time=0.1, tolerance=0.175*3, timeout=30.0)
            
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


class ScanBehaviour(py_trees.behaviour.Behaviour):
    """
    Performs a ±scan_angle sweep from the current heading to populate
    the vision object map with nearby objects on both sides.

    Sequence: rotate left → pause → center → pause → right → pause → center → pause → SUCCESS
    """
    def __init__(self, scan_angle_deg: float = 30.0, pause_time: float = 1.0, name="Scan Pipes"):
        super().__init__(name)
        self.scan_angle_rad = math.radians(scan_angle_deg)
        self.pause_time = pause_time

        self.blackboard = self.attach_blackboard_client(name=self.name)

        # State tracking
        self.center_yaw = None
        self.current_phase = 0
        self.action_status = ActionStatus.NOT_SENT
        self.sent_goal = False
        self.is_pausing = False
        self.pause_start_time = 0.0

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)

    def initialise(self):
        self.center_yaw = None
        self.current_phase = 0
        self.action_status = ActionStatus.NOT_SENT
        self.sent_goal = False
        self.is_pausing = False

    def _scan_offsets(self):
        """Returns the sequence of yaw offsets from center to execute."""
        return [
            -self.scan_angle_rad,   # rotate left
            0.0,                     # rotate back to center
            +self.scan_angle_rad,   # rotate right
            0.0,                     # rotate back to center
        ]

    def update(self):
        # Capture center yaw on first tick
        if self.center_yaw is None:
            if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
                self.node.get_logger().warn(f"[{self.name}] Waiting for /sensors/pose.")
                return py_trees.common.Status.RUNNING
            current_quat = self.blackboard.sensors.pose.pose.orientation
            self.center_yaw = yaw_from_quaternion(current_quat)

        offsets = self._scan_offsets()

        # All phases complete
        if self.current_phase >= len(offsets):
            return py_trees.common.Status.SUCCESS

        # Handle pause after a completed rotation
        if self.is_pausing:
            elapsed = (self.node.get_clock().now().nanoseconds / 1e9) - self.pause_start_time
            if elapsed >= self.pause_time:
                self.is_pausing = False
                self.current_phase += 1
                self.action_status = ActionStatus.NOT_SENT
                self.sent_goal = False
            return py_trees.common.Status.RUNNING

        # Rotation completed → start pause
        if self.action_status == ActionStatus.SUCCEEDED:
            self.is_pausing = True
            self.pause_start_time = self.node.get_clock().now().nanoseconds / 1e9
            self.node.get_logger().info(f"[{self.name}] Rotation complete, pausing {self.pause_time}s for vision.")
            return py_trees.common.Status.RUNNING

        if self.action_status == ActionStatus.FAILED:
            self.node.get_logger().error(f"[{self.name}] Rotation failed.")
            return py_trees.common.Status.FAILURE

        if self.action_status == ActionStatus.PENDING:
            return py_trees.common.Status.RUNNING

        # Send rotation goal
        if self.action_status == ActionStatus.NOT_SENT:
            offset = offsets[self.current_phase]
            target_yaw = normalize_angle(self.center_yaw + offset)
            direction = "left" if offset < 0 else ("right" if offset > 0 else "center")
            self.node.get_logger().info(
                f"[{self.name}] Rotating {direction} to {math.degrees(target_yaw):.1f}° "
                f"(phase {self.current_phase + 1}/{len(offsets)})"
            )
            goal = set_global_yaw(yaw_rad=target_yaw, hold_time=0.1, tolerance=0.175 * 3, timeout=30.0)
            self.navigation_client.send_navigation_goal(
                goal, self.name, self._on_goal_response, self._on_goal_result
            )
            self.action_status = ActionStatus.PENDING
            self.sent_goal = True
            return py_trees.common.Status.RUNNING

    def _on_goal_response(self, goal_response: bool):
        if not goal_response:
            self.action_status = ActionStatus.FAILED

    def _on_goal_result(self, goal_success: bool):
        self.action_status = ActionStatus.SUCCEEDED if goal_success else ActionStatus.FAILED

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            if hasattr(self, 'navigation_client') and self.navigation_client:
                self.navigation_client.reset_action_client()




