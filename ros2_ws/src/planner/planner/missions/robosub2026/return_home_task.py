import math
import py_trees
from controls.goal_helpers import move_global, set_depth
from controls.utils import normalize_angle, find_normal_from_quaternion, Vector2D
from ..action_status_enum import ActionStatus
from ..mission_behaviour_components import BasicActionBehaviour
from ..vision_behaviours import GoNearObject, MoveTowardsTargetByDistance, SearchSweepBehaviour
from .gate_behaviours import create_style_yaw_spin_sequence, create_style_rolling_flip_sequence


class NavigateToReturnPoint(py_trees.behaviour.Behaviour):
    """
    Reads the previously saved gate center and yaw, and computes a waypoint 
    past the gate on the far side to start the return sequence.
    """
    def __init__(
        self,
        return_distance: float,
        position_tolerance: float,
        hold_time: float,
        timeout: float,
        global_yaw_lock: bool = False,
        name: str = "Navigate to Return Point",
    ):
        super().__init__(name)
        self.return_distance = return_distance
        self.position_tolerance = position_tolerance
        self.hold_time = hold_time
        self.timeout = timeout
        self.global_yaw_lock = global_yaw_lock
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.action_status = ActionStatus.NOT_SENT

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.navigation_client = kwargs["shared_nav_client"]
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/center_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/center_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/target_yaw", access=py_trees.common.Access.READ)

    def initialise(self):
        self.action_status = ActionStatus.NOT_SENT

    def update(self):
        if self.action_status == ActionStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        if self.action_status == ActionStatus.FAILED:
            return py_trees.common.Status.FAILURE

        if self.action_status == ActionStatus.PENDING:
            return py_trees.common.Status.RUNNING

        cx = None
        cy = None
        yaw = None

        auv_pose = None
        auv_forward = None
        if hasattr(self.blackboard, "sensors") and self.blackboard.sensors.pose is not None:
            auv_pose = self.blackboard.sensors.pose.pose.position
            auv_forward = find_normal_from_quaternion(self.blackboard.sensors.pose.pose.orientation)

        if hasattr(self.blackboard, "vision") and self.blackboard.vision.object_map is not None:
            survey = None
            rescue = None
            gate_obj = None
            for obj in self.blackboard.vision.object_map.array:
                if auv_pose is not None and auv_forward is not None:
                    to_obj = Vector2D.from_point(obj.pose.position) - Vector2D.from_point(auv_pose)
                    if to_obj.dot(auv_forward) <= 0.0:
                        self.node.get_logger().info(
                            f"[{self.name}] Discarding candidate '{obj.label}' at ({obj.pose.position.x:.2f}, {obj.pose.position.y:.2f}) "
                            f"because it is not in front of AUV (dot={to_obj.dot(auv_forward):.2f} <= 0)."
                        )
                        continue

                if obj.label == "survey_repair":
                    survey = obj
                elif obj.label == "search_rescue":
                    rescue = obj
                elif obj.label == "gate":
                    gate_obj = obj
            
            if survey is not None:
                cx = survey.pose.position.x
                cy = survey.pose.position.y
            elif rescue is not None:
                cx = rescue.pose.position.x
                cy = rescue.pose.position.y
            elif gate_obj is not None:
                cx = gate_obj.pose.position.x
                cy = gate_obj.pose.position.y

        if cx is None or cy is None:
            try:
                if self.blackboard.gate.center_x is not None:
                    cx = self.blackboard.gate.center_x
                    cy = self.blackboard.gate.center_y
            except (AttributeError, KeyError):
                pass
        if cx is None or cy is None:
            self.node.get_logger().error(f"[{self.name}] No return point found in map or blackboard!")
            return py_trees.common.Status.FAILURE

        if self.global_yaw_lock:
            yaw = 0.0
        else:
            try:
                if self.blackboard.gate.target_yaw is not None:
                    yaw = self.blackboard.gate.target_yaw
            except (AttributeError, KeyError):
                pass
        if yaw is None:
            self.node.get_logger().error(f"[{self.name}] Gate target yaw not found on blackboard!")
            return py_trees.common.Status.FAILURE

        # The AUV passed through in the direction of `yaw`. 
        # So the "far" side of the gate is further along that vector.
        target_x = cx + math.cos(yaw) * self.return_distance
        target_y = cy + math.sin(yaw) * self.return_distance
        
        # We want to face backwards (towards the start)
        target_yaw = normalize_angle(yaw + math.pi)

        self.node.get_logger().info(
            f"[{self.name}] Moving to return point ({target_x:.2f}, {target_y:.2f}) "
            f"facing {math.degrees(target_yaw):.1f} deg"
        )

        goal = move_global(
            x=target_x,
            y=target_y,
            yaw=target_yaw,
            do_z=False,
            tolerance=self.position_tolerance,
            hold_time=self.hold_time,
            timeout=self.timeout,
        )
        self.navigation_client.send_navigation_goal(
            goal, self.name, self._on_goal_response, self._on_goal_result
        )
        self.action_status = ActionStatus.PENDING
        return py_trees.common.Status.RUNNING

    def _on_goal_response(self, goal_response: bool):
        if not goal_response:
            self.action_status = ActionStatus.FAILED

    def _on_goal_result(self, goal_success, message):
        self.action_status = ActionStatus.SUCCEEDED if goal_success else ActionStatus.FAILED

class PassBackThroughGate(py_trees.behaviour.Behaviour):
    """
    Computes a point behind the gate (towards the start point) and passes through it.
    """
    def __init__(
        self,
        pass_distance: float,
        position_tolerance: float,
        hold_time: float,
        timeout: float,
        global_yaw_lock: bool = False,
        name: str = "Pass Back Through Gate",
    ):
        super().__init__(name)
        self.pass_distance = pass_distance
        self.position_tolerance = position_tolerance
        self.hold_time = hold_time
        self.timeout = timeout
        self.global_yaw_lock = global_yaw_lock
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.action_status = ActionStatus.NOT_SENT

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.navigation_client = kwargs["shared_nav_client"]
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/center_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/center_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/target_yaw", access=py_trees.common.Access.READ)

    def initialise(self):
        self.action_status = ActionStatus.NOT_SENT

    def update(self):
        if self.action_status == ActionStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        if self.action_status == ActionStatus.FAILED:
            return py_trees.common.Status.FAILURE

        if self.action_status == ActionStatus.PENDING:
            return py_trees.common.Status.RUNNING

        cx = None
        cy = None
        yaw = None

        auv_pose = None
        auv_forward = None
        if hasattr(self.blackboard, "sensors") and self.blackboard.sensors.pose is not None:
            auv_pose = self.blackboard.sensors.pose.pose.position
            auv_forward = find_normal_from_quaternion(self.blackboard.sensors.pose.pose.orientation)

        if hasattr(self.blackboard, "vision") and self.blackboard.vision.object_map is not None:
            survey = None
            rescue = None
            gate_obj = None
            for obj in self.blackboard.vision.object_map.array:
                if auv_pose is not None and auv_forward is not None:
                    to_obj = Vector2D.from_point(obj.pose.position) - Vector2D.from_point(auv_pose)
                    if to_obj.dot(auv_forward) <= 0.0:
                        self.node.get_logger().info(
                            f"[{self.name}] Discarding candidate '{obj.label}' at ({obj.pose.position.x:.2f}, {obj.pose.position.y:.2f}) "
                            f"because it is not in front of AUV (dot={to_obj.dot(auv_forward):.2f} <= 0)."
                        )
                        continue

                if obj.label == "survey_repair":
                    survey = obj
                elif obj.label == "search_rescue":
                    rescue = obj
                elif obj.label == "gate":
                    gate_obj = obj
            
            if gate_obj is not None:
                cx = gate_obj.pose.position.x
                cy = gate_obj.pose.position.y
            elif survey is not None:
                cx = survey.pose.position.x
                cy = survey.pose.position.y
            elif rescue is not None:
                cx = rescue.pose.position.x
                cy = rescue.pose.position.y

        if cx is None or cy is None:
            try:
                if self.blackboard.gate.center_x is not None:
                    cx = self.blackboard.gate.center_x
                    cy = self.blackboard.gate.center_y
            except (AttributeError, KeyError):
                pass
        if cx is None or cy is None:
            self.node.get_logger().error(f"[{self.name}] No return point found in map or blackboard!")
            return py_trees.common.Status.FAILURE

        if self.global_yaw_lock:
            yaw = 0.0
        else:
            try:
                if self.blackboard.gate.target_yaw is not None:
                    yaw = self.blackboard.gate.target_yaw
            except (AttributeError, KeyError):
                pass
        if yaw is None:
            self.node.get_logger().error(f"[{self.name}] Gate target yaw not found on blackboard!")
            return py_trees.common.Status.FAILURE

        # Navigate past the gate in the opposite direction of the original pass
        target_x = cx - math.cos(yaw) * self.pass_distance
        target_y = cy - math.sin(yaw) * self.pass_distance
        target_yaw = normalize_angle(yaw + math.pi)

        self.node.get_logger().info(
            f"[{self.name}] Passing back through gate to ({target_x:.2f}, {target_y:.2f})"
        )

        goal = move_global(
            x=target_x,
            y=target_y,
            yaw=target_yaw,
            do_z=False,
            tolerance=self.position_tolerance,
            hold_time=self.hold_time,
            timeout=self.timeout,
        )
        self.navigation_client.send_navigation_goal(
            goal, self.name, self._on_goal_response, self._on_goal_result
        )
        self.action_status = ActionStatus.PENDING
        return py_trees.common.Status.RUNNING

    def _on_goal_response(self, goal_response: bool):
        if not goal_response:
            self.action_status = ActionStatus.FAILED

    def _on_goal_result(self, goal_success, message):
        self.action_status = ActionStatus.SUCCEEDED if goal_success else ActionStatus.FAILED


class CalculateSlalomAvoidanceWaypoints(py_trees.behaviour.Behaviour):
    """
    Computes an L-shaped (3-step) slalom avoidance path to safely bypass the slalom layer pipes
    on the return journey by shifting to the opposite side of the competition lane where there is more space.
    
    Steps:
      1. Diagonal/lateral move from current pose to `(slalom_mid_x, slalom_shifted_y)`.
      2. Step in X axis only to `(gate_return_x, slalom_shifted_y)`.
      3. Step in Y axis only to `(gate_return_x, gate_return_y)` (the desired gate return approach point).
    """
    def __init__(
        self,
        return_distance: float,
        global_yaw_lock: bool = False,
        lane_y_min: float = -5.0,
        lane_y_max: float = 10.0,
        name: str = "Calculate Slalom Avoidance Waypoints",
    ):
        super().__init__(name)
        self.return_distance = return_distance
        self.global_yaw_lock = global_yaw_lock
        self.lane_y_min = lane_y_min
        self.lane_y_max = lane_y_max
        self.blackboard = self.attach_blackboard_client(name=self.name)

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/center_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/center_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/target_yaw", access=py_trees.common.Access.READ)
        for l_num in (1, 2, 3):
            self.blackboard.register_key(key=f"/slalom/layer_{l_num}_x", access=py_trees.common.Access.READ)
            self.blackboard.register_key(key=f"/slalom/layer_{l_num}_y", access=py_trees.common.Access.READ)
        for s in (1, 2, 3):
            self.blackboard.register_key(key=f"/slalom/avoid_step{s}_x", access=py_trees.common.Access.WRITE)
            self.blackboard.register_key(key=f"/slalom/avoid_step{s}_y", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/slalom/avoid_target_yaw", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/slalom/avoid_valid", access=py_trees.common.Access.WRITE)

    def update(self):
        cx = None
        cy = None
        yaw = None

        if hasattr(self.blackboard, "vision") and self.blackboard.vision.object_map is not None:
            survey = None
            rescue = None
            gate_obj = None
            for obj in self.blackboard.vision.object_map.array:
                if obj.label == "survey_repair":
                    survey = obj
                elif obj.label == "search_rescue":
                    rescue = obj
                elif obj.label == "gate":
                    gate_obj = obj
            
            if survey is not None:
                cx = survey.pose.position.x
                cy = survey.pose.position.y
            elif rescue is not None:
                cx = rescue.pose.position.x
                cy = rescue.pose.position.y
            elif gate_obj is not None:
                cx = gate_obj.pose.position.x
                cy = gate_obj.pose.position.y

        if cx is None or cy is None:
            try:
                if self.blackboard.gate.center_x is not None:
                    cx = self.blackboard.gate.center_x
                    cy = self.blackboard.gate.center_y
            except (AttributeError, KeyError):
                pass
        if cx is None or cy is None:
            self.node.get_logger().error(f"[{self.name}] No return point found in map or blackboard!")
            return py_trees.common.Status.FAILURE

        if self.global_yaw_lock:
            yaw = 0.0
        else:
            try:
                if self.blackboard.gate.target_yaw is not None:
                    yaw = self.blackboard.gate.target_yaw
            except (AttributeError, KeyError):
                pass
        if yaw is None:
            self.node.get_logger().error(f"[{self.name}] Gate target yaw not found on blackboard!")
            return py_trees.common.Status.FAILURE

        gate_return_x = cx + math.cos(yaw) * self.return_distance
        gate_return_y = cy + math.sin(yaw) * self.return_distance
        target_yaw = normalize_angle(yaw + math.pi)

        red_x_vals = []
        red_y_vals = []

        if hasattr(self.blackboard, "vision") and self.blackboard.vision.object_map is not None:
            for obj in self.blackboard.vision.object_map.array:
                if obj.label == "red_pipe":
                    red_x_vals.append(obj.pose.position.x)
                    red_y_vals.append(obj.pose.position.y)

        for l_num in (1, 2, 3):
            try:
                lx = self.blackboard.get(f"/slalom/layer_{l_num}_x")
                ly = self.blackboard.get(f"/slalom/layer_{l_num}_y")
                if lx is not None and ly is not None:
                    red_x_vals.append(lx)
                    red_y_vals.append(ly)
            except (AttributeError, KeyError):
                pass

        if not red_x_vals or not red_y_vals:
            self.node.get_logger().warn(
                f"[{self.name}] No red pipes or saved layers found on blackboard or vision. Falling back to direct navigation to approach point."
            )
            self.blackboard.set("/slalom/avoid_valid", False)
            return py_trees.common.Status.FAILURE

        self.blackboard.set("/slalom/avoid_valid", True)
        slalom_mid_x = sum(red_x_vals) / len(red_x_vals)
        avg_red_y = sum(red_y_vals) / len(red_y_vals)
        self.node.get_logger().info(f"[{self.name}] Average slalom/red pipes pose: ({slalom_mid_x:.2f}, {avg_red_y:.2f})")

        space_right = avg_red_y - self.lane_y_min
        space_left = self.lane_y_max - avg_red_y
        if space_left > space_right:
            slalom_shifted_y = (avg_red_y + self.lane_y_max) / 2.0
        else:
            slalom_shifted_y = (avg_red_y + self.lane_y_min) / 2.0

        self.node.get_logger().info(
            f"[{self.name}] Slalom avoidance Y shifted from red_y={avg_red_y:.2f} to {slalom_shifted_y:.2f} "
            f"(lane bounds: [{self.lane_y_min}, {self.lane_y_max}])"
        )

        self.blackboard.set("/slalom/avoid_step1_x", slalom_mid_x)
        self.blackboard.set("/slalom/avoid_step1_y", slalom_shifted_y)

        self.blackboard.set("/slalom/avoid_step2_x", gate_return_x)
        self.blackboard.set("/slalom/avoid_step2_y", slalom_shifted_y)

        self.blackboard.set("/slalom/avoid_step3_x", gate_return_x)
        self.blackboard.set("/slalom/avoid_step3_y", gate_return_y)
        self.blackboard.set("/slalom/avoid_target_yaw", target_yaw)

        return py_trees.common.Status.SUCCESS


class MoveToSlalomAvoidanceStep(py_trees.behaviour.Behaviour):
    """
    Executes one step of the calculated slalom avoidance path.
    If the step coordinates are not found on the blackboard, completes with SUCCESS.
    If the AUV is already closer along X to the gate than the step target (for steps 1 and 2),
    completes with SUCCESS immediately to prevent backtracking.
    """
    def __init__(
        self,
        step: int,
        position_tolerance: float,
        hold_time: float,
        timeout: float,
        name: str = None,
    ):
        if name is None:
            name = f"Move to Slalom Avoidance Step {step}"
        super().__init__(name)
        self.step = step
        self.position_tolerance = position_tolerance
        self.hold_time = hold_time
        self.timeout = timeout
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.action_status = ActionStatus.NOT_SENT

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.navigation_client = kwargs["shared_nav_client"]
        self.blackboard.register_key(key=f"/slalom/avoid_step{self.step}_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"/slalom/avoid_step{self.step}_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/slalom/avoid_target_yaw", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/slalom/avoid_valid", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/center_x", access=py_trees.common.Access.READ)

    def initialise(self):
        self.action_status = ActionStatus.NOT_SENT

    def update(self):
        if self.action_status == ActionStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        if self.action_status == ActionStatus.FAILED:
            return py_trees.common.Status.FAILURE
        if self.action_status == ActionStatus.PENDING:
            return py_trees.common.Status.RUNNING

        try:
            if self.blackboard.get("/slalom/avoid_valid") is False:
                self.node.get_logger().info(
                    f"[{self.name}] Slalom avoidance skipped due to missing pipe/layer detections. Skipping step {self.step}."
                )
                return py_trees.common.Status.SUCCESS
        except (AttributeError, KeyError):
            pass

        target_x = None
        target_y = None
        target_yaw = None
        try:
            target_x = self.blackboard.get(f"/slalom/avoid_step{self.step}_x")
            target_y = self.blackboard.get(f"/slalom/avoid_step{self.step}_y")
            target_yaw = self.blackboard.get("/slalom/avoid_target_yaw")
        except (AttributeError, KeyError):
            pass

        if target_x is None or target_y is None or target_yaw is None:
            self.node.get_logger().warn(
                f"[{self.name}] Slalom avoidance step {self.step} coordinates missing. Skipping."
            )
            return py_trees.common.Status.SUCCESS

        if self.step in (1, 2) and hasattr(self.blackboard, "sensors") and self.blackboard.sensors.pose is not None:
            curr_x = self.blackboard.sensors.pose.pose.position.x
            gate_x = None
            try:
                gate_x = self.blackboard.gate.center_x
            except (AttributeError, KeyError):
                pass
            if gate_x is not None and abs(curr_x - gate_x) < abs(target_x - gate_x) - 1.0:
                self.node.get_logger().info(
                    f"[{self.name}] AUV is already closer to gate (curr_x={curr_x:.2f}) than target_x={target_x:.2f}. Skipping step {self.step}."
                )
                return py_trees.common.Status.SUCCESS

        self.node.get_logger().info(
            f"[{self.name}] Executing Slalom Avoidance Step {self.step} -> ({target_x:.2f}, {target_y:.2f}) yaw={math.degrees(target_yaw):.1f} deg"
        )

        goal = move_global(
            x=target_x,
            y=target_y,
            yaw=target_yaw,
            do_z=False,
            tolerance=self.position_tolerance,
            hold_time=self.hold_time,
            timeout=self.timeout,
        )
        self.navigation_client.send_navigation_goal(
            goal, self.name, self._on_goal_response, self._on_goal_result
        )
        self.action_status = ActionStatus.PENDING
        return py_trees.common.Status.RUNNING

    def _on_goal_response(self, goal_response: bool):
        if not goal_response:
            self.action_status = ActionStatus.FAILED

    def _on_goal_result(self, goal_success, message):
        self.action_status = ActionStatus.SUCCEEDED if goal_success else ActionStatus.FAILED


class ReturnHomeTask(py_trees.composites.Sequence):
    """
    Sequence for navigating back through the gate and surfacing at the end of the run.
    """
    def __init__(
        self,
        return_distance: float,
        pass_distance: float,
        approach_distance: float,
        surface_depth: float,
        position_tolerance: float,
        hold_time: float,
        timeout: float,
        global_yaw_lock: bool = False,
        do_style_yaw_before: bool = False,
        do_style_yaw_after: bool = False,
        style_yaw_degrees: float = 360.0,
        do_style_roll_before: bool = False,
        do_style_roll_after: bool = False,
        style_roll_degrees: float = 720.0,
        style_roll_torque: float = 15.0,
        style_roll_coast_degrees: float = 180.0,
        travel_depth: float = -0.5,
        gate_pass_depth: float = -1.0,
        exit_octagon_distance: float = 2.0,
        octagon_exit_depth: float = -0.8,
        do_slalom_avoidance: bool = True,
        lane_y_min: float = -5.0,
        lane_y_max: float = 10.0,
        **kwargs,
    ):
        super().__init__("Return Home Task", memory=True)

        return_children = [
            BasicActionBehaviour(
                name=f"Ascend to Octagon Exit Depth ({octagon_exit_depth}m)",
                goal=set_depth(
                    z=octagon_exit_depth,
                    tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                ),
            ),
            MoveTowardsTargetByDistance(
                distance=exit_octagon_distance,
                blackboard_x_key="/gate/center_x",
                blackboard_y_key="/gate/center_y",
                use_3d=False,
                position_tolerance=position_tolerance,
                hold_time=hold_time,
                timeout=timeout,
                name="Exit Octagon Zone Towards Gate",
            ),
            BasicActionBehaviour(
                name=f"Ascend to Travel Depth ({travel_depth}m)",
                goal=set_depth(
                    z=travel_depth,
                    tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                ),
            ),
        ]

        if do_slalom_avoidance:
            slalom_avoidance_path = py_trees.composites.Sequence("Slalom Avoidance Path", memory=True)
            slalom_avoidance_path.add_children([
                CalculateSlalomAvoidanceWaypoints(
                    return_distance=return_distance,
                    global_yaw_lock=global_yaw_lock,
                    lane_y_min=lane_y_min,
                    lane_y_max=lane_y_max,
                    name="Calculate Slalom Avoidance Waypoints",
                ),
                MoveToSlalomAvoidanceStep(
                    step=1,
                    position_tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                    name="Step 1: Shift to Opposite Lane Space beside Slalom Middle",
                ),
                MoveToSlalomAvoidanceStep(
                    step=2,
                    position_tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                    name="Step 2: Step in X Axis Only Past Slalom Field",
                ),
                MoveToSlalomAvoidanceStep(
                    step=3,
                    position_tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                    name="Step 3: Step in Y Axis Only to Gate Approach Position",
                ),
            ])
            slalom_or_direct = py_trees.composites.Selector("Slalom Avoidance or Direct Navigation", memory=True)
            slalom_or_direct.add_children([
                slalom_avoidance_path,
                NavigateToReturnPoint(
                    return_distance=return_distance,
                    position_tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                    global_yaw_lock=global_yaw_lock,
                    name="Directly Navigate to Approach Point (No Slalom Data Fallback)",
                ),
            ])
            return_children.append(slalom_or_direct)
        else:
            return_children.append(
                NavigateToReturnPoint(
                    return_distance=return_distance,
                    position_tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                    global_yaw_lock=global_yaw_lock,
                    name="Directly Navigate to Approach Point",
                )
            )

        return_children.extend([
            BasicActionBehaviour(
                name=f"Descend to Gate Traversal Depth ({gate_pass_depth}m)",
                goal=set_depth(
                    z=gate_pass_depth,
                    tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                ),
            ),
            # Search / visually approach the gate from the back
            GoNearObject(
                target_class="gate",
                target_distance=approach_distance,
                tolerance_meters=position_tolerance,
                hold_time=1.0,
                require_in_front=True,
                name="Visually Approach Back of Gate"
            ),
        ])

        if do_style_yaw_before:
            steps = 6 if style_yaw_degrees >= 720.0 else 3
            return_children.append(
                create_style_yaw_spin_sequence(
                    total_degrees=style_yaw_degrees,
                    num_steps=steps,
                    angular_tolerance_deg=10.0,
                    hold_time_s=0.5,
                    timeout_s=30.0,
                )
            )

        if do_style_roll_before:
            return_children.append(
                create_style_rolling_flip_sequence(
                    roll_torque=style_roll_torque,
                    target_degrees=style_roll_degrees,
                    coast_degrees=style_roll_coast_degrees,
                    timeout_sec=30.0,
                )
            )

        return_children.append(
            PassBackThroughGate(
                pass_distance=pass_distance,
                position_tolerance=position_tolerance,
                hold_time=hold_time,
                timeout=timeout,
                global_yaw_lock=global_yaw_lock,
            )
        )

        if do_style_yaw_after:
            steps = 6 if style_yaw_degrees >= 720.0 else 3
            return_children.append(
                create_style_yaw_spin_sequence(
                    total_degrees=style_yaw_degrees,
                    num_steps=steps,
                    angular_tolerance_deg=10.0,
                    hold_time_s=0.5,
                    timeout_s=30.0,
                )
            )

        if do_style_roll_after:
            return_children.append(
                create_style_rolling_flip_sequence(
                    roll_torque=style_roll_torque,
                    target_degrees=style_roll_degrees,
                    coast_degrees=style_roll_coast_degrees,
                    timeout_sec=30.0,
                )
            )

        return_sequence = py_trees.composites.Sequence("Gate Return Sequence", memory=True)
        return_sequence.add_children(return_children)

        # Fallback 1: Scan for Gate if saved gate center was not found / nominal return failed
        scan_gate_children = [
            BasicActionBehaviour(
                name="Ascend for Gate Scan",
                goal=set_depth(
                    z=gate_pass_depth,
                    tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                ),
            ),
        ]

        if do_slalom_avoidance:
            slalom_avoidance_path_fallback = py_trees.composites.Sequence("Slalom Avoidance Path (Fallback)", memory=True)
            slalom_avoidance_path_fallback.add_children([
                CalculateSlalomAvoidanceWaypoints(
                    return_distance=return_distance,
                    global_yaw_lock=global_yaw_lock,
                    lane_y_min=lane_y_min,
                    lane_y_max=lane_y_max,
                    name="Calculate Slalom Avoidance Waypoints (Fallback)",
                ),
                MoveToSlalomAvoidanceStep(
                    step=1,
                    position_tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                    name="Step 1: Shift beside Slalom Middle (Fallback)",
                ),
                MoveToSlalomAvoidanceStep(
                    step=2,
                    position_tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                    name="Step 2: Step X Past Slalom (Fallback)",
                ),
                MoveToSlalomAvoidanceStep(
                    step=3,
                    position_tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                    name="Step 3: Step Y to Gate Approach (Fallback)",
                ),
            ])
            slalom_or_direct_fallback = py_trees.composites.Selector("Slalom Avoidance or Direct Navigation (Fallback)", memory=True)
            slalom_or_direct_fallback.add_children([
                slalom_avoidance_path_fallback,
                NavigateToReturnPoint(
                    return_distance=return_distance,
                    position_tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                    global_yaw_lock=global_yaw_lock,
                    name="Directly Navigate to Approach Point (Fallback when No Slalom Data)",
                ),
            ])
            scan_gate_children.append(slalom_or_direct_fallback)
        else:
            scan_gate_children.append(
                NavigateToReturnPoint(
                    return_distance=return_distance,
                    position_tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                    global_yaw_lock=global_yaw_lock,
                    name="Directly Navigate to Approach Point (Fallback)",
                )
            )

        scan_gate_children.extend([
            SearchSweepBehaviour(
                target_class="gate",
                num_steps=6,
                max_attempts=2,
                step_timeout=1.0,
                turn_timeout_s=30.0,
                name="Scan For Gate (Fallback)",
            ),
            GoNearObject(
                target_class="gate",
                target_distance=approach_distance,
                tolerance_meters=position_tolerance,
                hold_time=1.0,
                require_in_front=True,
                name="Approach Gate After Scan",
            ),
        ])
        if do_style_yaw_before:
            steps = 6 if style_yaw_degrees >= 720.0 else 3
            scan_gate_children.append(
                create_style_yaw_spin_sequence(
                    total_degrees=style_yaw_degrees,
                    num_steps=steps,
                    angular_tolerance_deg=10.0,
                    hold_time_s=0.5,
                    timeout_s=30.0,
                )
            )
        if do_style_roll_before:
            scan_gate_children.append(
                create_style_rolling_flip_sequence(
                    roll_torque=style_roll_torque,
                    target_degrees=style_roll_degrees,
                    coast_degrees=style_roll_coast_degrees,
                    timeout_sec=30.0,
                )
            )
        scan_gate_children.append(
            PassBackThroughGate(
                pass_distance=pass_distance,
                position_tolerance=position_tolerance,
                hold_time=hold_time,
                timeout=timeout,
                global_yaw_lock=global_yaw_lock,
            )
        )
        if do_style_yaw_after:
            steps = 6 if style_yaw_degrees >= 720.0 else 3
            scan_gate_children.append(
                create_style_yaw_spin_sequence(
                    total_degrees=style_yaw_degrees,
                    num_steps=steps,
                    angular_tolerance_deg=10.0,
                    hold_time_s=0.5,
                    timeout_s=30.0,
                )
            )
        if do_style_roll_after:
            scan_gate_children.append(
                create_style_rolling_flip_sequence(
                    roll_torque=style_roll_torque,
                    target_degrees=style_roll_degrees,
                    coast_degrees=style_roll_coast_degrees,
                    timeout_sec=30.0,
                )
            )
        scan_gate_sequence = py_trees.composites.Sequence("Scan for Gate & Return", memory=True)
        scan_gate_sequence.add_children(scan_gate_children)

        # Fallback 2: Worst Case Scenario — Return to Origin (X=0, Y=0)
        origin_fallback_children = [
            BasicActionBehaviour(
                name="Ascend to Travel Depth for Origin Return",
                goal=set_depth(
                    z=travel_depth,
                    tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                ),
            ),
            BasicActionBehaviour(
                name="Return to Origin (X=0, Y=0)",
                goal=move_global(
                    x=0.0,
                    y=0.0,
                    do_z=False,
                    tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                ),
            ),
        ]
        if do_style_yaw_before:
            steps = 6 if style_yaw_degrees >= 720.0 else 3
            origin_fallback_children.append(
                create_style_yaw_spin_sequence(
                    total_degrees=style_yaw_degrees,
                    num_steps=steps,
                    angular_tolerance_deg=10.0,
                    hold_time_s=0.5,
                    timeout_s=30.0,
                )
            )
        if do_style_roll_before:
            origin_fallback_children.append(
                create_style_rolling_flip_sequence(
                    roll_torque=style_roll_torque,
                    target_degrees=style_roll_degrees,
                    coast_degrees=style_roll_coast_degrees,
                    timeout_sec=30.0,
                )
            )
        if do_style_yaw_after:
            steps = 6 if style_yaw_degrees >= 720.0 else 3
            origin_fallback_children.append(
                create_style_yaw_spin_sequence(
                    total_degrees=style_yaw_degrees,
                    num_steps=steps,
                    angular_tolerance_deg=10.0,
                    hold_time_s=0.5,
                    timeout_s=30.0,
                )
            )
        if do_style_roll_after:
            origin_fallback_children.append(
                create_style_rolling_flip_sequence(
                    roll_torque=style_roll_torque,
                    target_degrees=style_roll_degrees,
                    coast_degrees=style_roll_coast_degrees,
                    timeout_sec=30.0,
                )
            )
        origin_fallback_sequence = py_trees.composites.Sequence("Worst Case Return to Origin", memory=True)
        origin_fallback_sequence.add_children(origin_fallback_children)

        return_strategy = py_trees.composites.Selector("Return Home Strategy", memory=True)
        return_strategy.add_children([
            return_sequence,
            scan_gate_sequence,
            origin_fallback_sequence,
        ])

        # If anything in the return strategy fails, we still want to surface!
        failsafe_return = py_trees.decorators.FailureIsSuccess(
            name="Failsafe Return Wrapper",
            child=return_strategy
        )

        self.add_children([
            failsafe_return,
            BasicActionBehaviour(
                name="Surface (Finish Run)",
                goal=set_depth(
                    z=surface_depth,
                    tolerance=position_tolerance,
                    hold_time=3.0,
                    timeout=timeout,
                )
            )
        ])
