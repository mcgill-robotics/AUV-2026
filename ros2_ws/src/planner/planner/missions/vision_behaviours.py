import math
from enum import Enum

from typing import Optional, Tuple, List
import py_trees
from controls.goal_helpers import set_global_yaw, look_at, move_and_look
from controls.utils import yaw_from_quaternion, normalize_angle
from .action_status_enum import ActionStatus
from auv_msgs.msg import VisionObject

class SearchSweepBehaviour(py_trees.behaviour.Behaviour):
    """
    Rotates the AUV in a full 360-degree sweep, divided into `num_steps`. 
    After each turn step, it pauses for `step_timeout` seconds to let vision stabilize.
    If it completes `max_attempts` full 360-degree sweeps without finding `target_class`, it fails.
    If the object is seen at any point, it returns SUCCESS immediately.
    """
    def __init__(
        self,
        target_class: str,
        num_steps: int = 5,
        max_attempts: int = 2,
        step_timeout: float = 2.0,
        clockwise: bool = False,
        look_at_on_success: bool = True,
        angular_tolerance_rad: float = math.radians(30.0),  # (rad) yaw convergence threshold per turn step
        turn_hold_time_s: float = 0.1,                   # (s) hold time before turn step SUCCESS
        turn_timeout_s: float = 30.0,                    # (s) timeout before turn step FAILURE
        name="SearchSweep",
    ):
        super().__init__(name)
        self.target_class = target_class
        self.num_steps = num_steps
        self.max_attempts = max_attempts
        self.step_timeout = step_timeout
        self.clockwise = clockwise
        self.look_at_on_success = look_at_on_success
        self.angular_tolerance_rad = angular_tolerance_rad
        self.turn_hold_time_s = turn_hold_time_s
        self.turn_timeout_s = turn_timeout_s
        
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
        self.result_message = ''
        
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
        self.result_message = ''
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
                self.node.get_logger().error(f"[{self.name}] Final alignment turn failed. {self.result_message}")
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
                    tolerance=self.angular_tolerance_rad,   # (rad)
                    hold_time=self.step_timeout,        # (s) hold to let vision settle on target
                    timeout=self.turn_timeout_s,        # (s)
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
            self.node.get_logger().error(f"[{self.name}] Turn action failed midway. {self.result_message}")
            return py_trees.common.Status.FAILURE
            
        # If we just finished a turn step, we need to pause
        if self.action_status == ActionStatus.SUCCEEDED:
            if not self.is_pausing:
                self.is_pausing = True
                self.pause_start_time = self.node.get_clock().now().nanoseconds / 1e9
                self.node.get_logger().info(f"[{self.name}] Step complete. Pausing for {self.step_timeout}s to stabilize vision. {self.result_message}")
            
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
            goal = set_global_yaw(
                yaw_rad=target_yaw,
                tolerance=self.angular_tolerance_rad,  # (rad)
                hold_time=self.turn_hold_time_s,   # (s)
                timeout=self.turn_timeout_s,       # (s)
            )
            
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

    def on_server_goal_result(self, goal_success: bool, message: str):
        self.result_message = message
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.action_status = ActionStatus.FAILED




class ScanBehaviour(py_trees.behaviour.Behaviour):
    """
    Performs a ±scan_angle sweep from the current heading to populate
    the vision object map with nearby objects on both sides.

    Sequence: left -> right -> center -> SUCCESS
    """
    def __init__(
        self,
        scan_angle_deg: float = 30.0,
        pause_time: float = 1.0,
        angular_tolerance_rad: float = math.radians(30.0),  # (rad) yaw convergence threshold per turn
        turn_hold_time_s: float = 0.1,                   # (s) hold time before turn SUCCESS
        turn_timeout_s: float = 30.0,                    # (s) timeout before turn FAILURE
        name="Scan Pipes",
    ):
        super().__init__(name)
        self.scan_angle_rad = math.radians(scan_angle_deg)
        self.pause_time = pause_time
        self.angular_tolerance_rad = angular_tolerance_rad
        self.turn_hold_time_s = turn_hold_time_s
        self.turn_timeout_s = turn_timeout_s

        self.blackboard = self.attach_blackboard_client(name=self.name)

        # State tracking
        self.center_yaw = None
        self.current_phase = 0
        self.action_status = ActionStatus.NOT_SENT
        self.sent_goal = False
        self.result_message = ''
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
        self.result_message = ''
        self.is_pausing = False

    def _scan_offsets(self):
        """Returns the sequence of yaw offsets from center to execute.
        Pattern: left -> right -> center (3 moves covering the full scan range).
        Convention: +yaw = counterclockwise (left), -yaw = clockwise (right).
        """
        return [
            +self.scan_angle_rad,   # rotate left (counterclockwise)
            -self.scan_angle_rad,   # rotate right (sweeps through center)
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
            self.node.get_logger().info(f"[{self.name}] Rotation complete, pausing {self.pause_time}s for vision. {self.result_message}")
            return py_trees.common.Status.RUNNING

        if self.action_status == ActionStatus.FAILED:
            self.node.get_logger().error(f"[{self.name}] Rotation failed. {self.result_message}")
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
            goal = set_global_yaw(
                yaw_rad=target_yaw,
                tolerance=self.angular_tolerance_rad,  # (rad)
                hold_time=self.turn_hold_time_s,   # (s)
                timeout=self.turn_timeout_s,       # (s)
            )
            self.navigation_client.send_navigation_goal(
                goal, self.name, self._on_goal_response, self._on_goal_result
            )
            self.action_status = ActionStatus.PENDING
            self.sent_goal = True
            return py_trees.common.Status.RUNNING

    def _on_goal_response(self, goal_response: bool):
        if not goal_response:
            self.action_status = ActionStatus.FAILED

    def _on_goal_result(self, goal_success: bool, message: str):
        self.result_message = message
        self.action_status = ActionStatus.SUCCEEDED if goal_success else ActionStatus.FAILED


class CircleAroundPhases(Enum):
    """
    Phases of the Circle Around To Find behaviour state machine
    """
    APPROACH = 0
    PAUSING = 1
    CIRCLING = 2


class CircleAroundToFindBehaviour(py_trees.behaviour.Behaviour):
    """
    Circles around a reference point while facing it, attempting at the same time to find a target object in vision.
    This behaviour is mostly intended for the torpedo board, circling around the board to find the icons and thus confirming we are facing the right direction to shoot torpedoes.
    
    reference_class: The vision class of the reference point to circle around (e.g. "torpedo_board").
    target_classes: A dictionary mapping vision classes of interest to integer counts of how many of each class to find (e.g. {"blood": 1, "ambulance": 1}).
    reference_distance: (m) The distance to maintain from the reference point while circling.
    num_steps: The number of discrete steps in the circle (e.g. 8 would be every 45 degrees).
    max_attempts: The number of full circles (360-degree sweeps) to attempt before giving up.
    step_timeout: (s) Seconds to pause after each turn step to let vision stabilize.
    clockwise: Whether to circle in a clockwise direction (True) or counterclockwise (False).
    look_at_on_success: Whether to do a final "look at" turn towards the reference point after finding the target object(s) to better center it in view.
    yaw_tolerance_rad: (rad) Yaw convergence threshold for each turn step.
    turn_hold_time_s: (s) Hold time before declaring turn step SUCCESS.
    turn_timeout_s: (s) Timeout before declaring turn step FAILURE.
    """
    def __init__(
        self,
        reference_class: str,
        target_classes: dict[str,int],
        reference_distance: float = 1.0,
        num_steps: int = 5,
        max_attempts: int = 2,
        step_timeout: float = 2.0,
        clockwise: bool = False,
        look_at_on_success: bool = True,
        yaw_tolerance_rad: float = math.radians(30.0),  # (rad) yaw convergence threshold per turn step
        turn_hold_time_s: float = 0.1,                   # (s) hold time before turn step SUCCESS
        turn_timeout_s: float = 30.0,                    # (s) timeout before turn step FAILURE
        name="CircleAroundToFind",
    ):
        super().__init__(name)
        
        self.reference_class: str = reference_class
        self.target_classes: dict[str, int] = target_classes
        self.reference_distance: float = reference_distance
        self.num_steps: int = num_steps
        self.max_attempts: int = max_attempts
        self.step_timeout: float = step_timeout
        self.clockwise: bool = clockwise
        self.look_at_on_success: bool = look_at_on_success
        self.yaw_tolerance_rad: float = yaw_tolerance_rad
        self.turn_hold_time_s: float = turn_hold_time_s
        self.turn_timeout_s: float = turn_timeout_s

        # State tracking
        self.phase: CircleAroundPhases = CircleAroundPhases.APPROACH
        self.current_attempt: int = 0
        self.current_step: int = 0
        self.action_status: ActionStatus = ActionStatus.NOT_SENT
        self.sent_goal: bool = False
        self.reference_position: Optional[Tuple[float, float, float]] = None

        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        # Pause tracking        
        self.pause_start_time: float = 0.0

        # Absolute angle tracking
        self.start_yaw: Optional[float] = None
        
        # Target found tracking
        self.found_targets: dict[str, int] = {cls: 0 for cls in target_classes.keys()}
        
    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0) 
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        
    def initialise(self):
        # Reset state when this behavior starts
        self.phase = CircleAroundPhases.APPROACH
        self.current_attempt = 0
        self.current_step = 0
        self.action_status = ActionStatus.NOT_SENT
        self.sent_goal = False
        self.is_pausing = False
        self.start_yaw = None
        self.found_targets = {cls: 0 for cls in self.target_classes.keys()}
        self.reference_position = None
        
    def update(self):
        match self.phase:
            case CircleAroundPhases.APPROACH:
                match self.action_status:
                    case ActionStatus.SUCCEEDED:
                        self.node.get_logger().info(f"[{self.name}] Initial face towards reference complete.")
                        self.phase = CircleAroundPhases.PAUSING
                        self.action_status = ActionStatus.NOT_SENT
                        self.sent_goal = False
                        return py_trees.common.Status.RUNNING
                    case ActionStatus.FAILED:
                        self.node.get_logger().error(f"[{self.name}] Initial face towards reference failed.")
                        return py_trees.common.Status.FAILURE
                    case ActionStatus.PENDING:
                        return py_trees.common.Status.RUNNING
                    case ActionStatus.NOT_SENT:
                        pass

                        
                        
                return py_trees.common.Status.RUNNING
            case CircleAroundPhases.APPROACH:
                # find AUV current pose
                if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
                    self.node.get_logger().warn(f"[{self.name}] Waiting for /sensors/pose to determine current position.")
                    return py_trees.common.Status.RUNNING
                auv_x = self.blackboard.sensors.pose.pose.position.x
                auv_y = self.blackboard.sensors.pose.pose.position.y
                # determine vector to reference point
    
    
    
    def _find_object_in_map(self,class_name: str)-> List[VisionObject]:
        """Helper function to find all objects of a given class in the vision object map on the blackboard."""
        
        objects: List[VisionObject] = []
        if hasattr(self.blackboard, 'vision') and self.blackboard.vision.object_map is not None:
            for obj in self.blackboard.vision.object_map.array:
                if obj.label == class_name:
                    objects.append(obj)
        return objects
        
    def _determine_auv_pose(self) -> Optional[Tuple[float, float, float, float]]:
        """Helper function to determine the AUV's current position and yaw from the blackboard. Returns (x, y, z, yaw) or None if pose is not available."""
        
        if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
            return None
        auv_x = self.blackboard.sensors.pose.pose.position.x
        auv_y = self.blackboard.sensors.pose.pose.position.y
        auv_z = self.blackboard.sensors.pose.pose.position.z
        current_quat = self.blackboard.sensors.pose.pose.orientation
        auv_yaw = yaw_from_quaternion(current_quat)
        return (auv_x, auv_y, auv_z, auv_yaw)
    
    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.action_status = ActionStatus.FAILED
            
    def on_server_goal_result(self, goal_success: bool):
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.action_status = ActionStatus.FAILED
                