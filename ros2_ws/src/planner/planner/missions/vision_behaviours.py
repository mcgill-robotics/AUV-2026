import math
import py_trees
from controls.goal_helpers import set_global_yaw, look_at, move_global, _DEFAULT_POS_TOL, _DEFAULT_HOLD#, _DEFAULT_TIMEOUT,_DEFAULT_YAW_TOL, POSITION_EPSILON
from controls.utils import yaw_from_quaternion, normalize_angle
from .action_status_enum import ActionStatus

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
        
        self.is_looking_at_target = False
        self.target_found_pos = None
        self.expected_failures = 0

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
        self.expected_failures = 0

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
                            if self.action_status == ActionStatus.PENDING:
                                self.expected_failures += 1
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
        elif self.expected_failures > 0:
            self.expected_failures -= 1
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
        num_steps_per_side: int = 1                      # Number of steps to scan, avoid big swings
    ):
        super().__init__(name)
        self.scan_angle_rad = math.radians(scan_angle_deg)
        self.pause_time = pause_time
        self.angular_tolerance_rad = angular_tolerance_rad
        self.turn_hold_time_s = turn_hold_time_s
        self.turn_timeout_s = turn_timeout_s
        self.num_steps_per_side = num_steps_per_side

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
        Pattern: left -> right -> center (3 general moves covering the full scan range).
        Convention: +yaw = counterclockwise (left), -yaw = clockwise (right).
        """
        ccw_yaw_targets = []
        cw_yaw_targets = []

        if self.num_steps_per_side <= 0:
            self.node.get_logger().warn(f"Step number per side invalid: {self.num_steps_per_side}")

        for i in range(self.num_steps_per_side + 1, 1, -1):
            ccw_yaw_targets.append(+self.scan_angle_rad/i)
            cw_yaw_targets.append(-self.scan_angle_rad + self.scan_angle_rad/i)

        return [
            *ccw_yaw_targets,   # rotate left (counterclockwise)
            *cw_yaw_targets,   # rotate right (sweeps through center)
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

class GoNearObject(py_trees.behaviour.Behaviour):
    def __init__(self, target_class: str, target_distance: float, height_offset: float | None = None, tolerance_meters: float=_DEFAULT_POS_TOL, hold_time: float=_DEFAULT_HOLD, name: str | None = None):
        super().__init__(name if name else f"GoNear{target_class}")
        self.target_class = target_class
        self.target_planar_distance = target_distance
        self.height_offset = height_offset
        self.tolerance_meters = tolerance_meters
        self.hold_time = hold_time
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/vision/last_goal_pose", access=py_trees.common.Access.WRITE)
        self.action_status = ActionStatus.NOT_SENT

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0) 

    def initialise(self):
        # Find the target in object map
        if hasattr(self.blackboard, 'vision') and self.blackboard.vision.object_map is not None \
           and hasattr(self.blackboard, 'sensors') and self.blackboard.sensors.pose is not None:
            target_obj = None
            auv_pose = self.blackboard.sensors.pose.pose.position
            min_distance = float('inf')
            for obj in self.blackboard.vision.object_map.array:
                if obj.label == self.target_class:
                    distance = math.sqrt((obj.pose.position.x - auv_pose.x) ** 2 + (obj.pose.position.y - auv_pose.y) ** 2)
                    if distance < min_distance:
                        target_obj = obj
                        min_distance = distance
            
            if target_obj is None:
                self.node.get_logger().error(f"[{self.name}] Target '{self.target_class}' not found in vision during setup!")
                return py_trees.common.Status.FAILURE
            
            target_x = target_obj.pose.position.x
            target_y = target_obj.pose.position.y

            current_pose = self.blackboard.sensors.pose.pose.position

            direction_vector = (target_x - current_pose.x, target_y - current_pose.y)
            magnitude = math.sqrt(direction_vector[0]**2 + direction_vector[1]**2)

            # if magnitude <= self.target_planar_distance:
            #     self.node.get_logger().info(f"[{self.name}] Already within target distance of {self.target_class}. No movement needed.")
            #     return py_trees.common.Status.SUCCESS
            
            normalized_direction = (direction_vector[0]/magnitude, direction_vector[1]/magnitude)

            goal_x = target_x - normalized_direction[0] * self.target_planar_distance
            goal_y = target_y - normalized_direction[1] * self.target_planar_distance

            if self.height_offset is None:
                goal = move_global(goal_x, goal_y, do_z=False, tolerance=self.tolerance_meters, hold_time=self.hold_time)
            else:
                goal_z = target_obj.pose.position.z + self.height_offset
                goal = move_global(goal_x, goal_y, goal_z, do_z=True, tolerance=self.tolerance_meters, hold_time=self.hold_time)

            self.blackboard.vision.last_goal_pose = goal
            self.navigation_client.send_navigation_goal(goal, self.name, custom_goal_response=self.on_server_goal_response, custom_goal_result=self.on_server_goal_result)

            self.action_status = ActionStatus.PENDING

            return py_trees.common.Status.RUNNING

        else: return py_trees.common.Status.FAILURE

    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.node.get_logger().error(f"[{self.name}] Failed to send goal to navigation server.")
            self.action_status = ActionStatus.FAILED
    
    def on_server_goal_result(self, goal_success: bool, message: str):
        if goal_success:
            self.node.get_logger().info(f"[{self.name}] Reached target distance from {self.target_class}. {message}")
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to reach target distance from {self.target_class}. {message}")
            self.action_status = ActionStatus.FAILED
        
    def update(self):
        if self.action_status == ActionStatus.SUCCEEDED:
            self.node.get_logger().info(f"[{self.name}] Returning SUCCESS.")
            return py_trees.common.Status.SUCCESS
        elif self.action_status == ActionStatus.FAILED:
            self.node.get_logger().info(f"[{self.name}] Returning FAILURE.")
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING
