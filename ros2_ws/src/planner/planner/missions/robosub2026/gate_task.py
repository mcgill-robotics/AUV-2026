import math
import random

import py_trees
from controls.goal_helpers import move_global, move_to_pose, set_depth, set_global_yaw
from controls.utils import normalize_angle, yaw_from_quaternion

from ..action_status_enum import ActionStatus
from ..mission_behaviour_components import BasicActionBehaviour
from ..vision_behaviours import ScanBehaviour, SearchSweepBehaviour
from .slalom_behaviours import ForceBlindDriveBehaviour
from .gate_behaviours import create_style_yaw_spin_sequence, create_style_rolling_flip_sequence


class PlanGateTraversalBehaviour(py_trees.behaviour.Behaviour):
    """
    Chooses a gate side, computes a pass-through target, and writes the result
    to the blackboard for the navigation behaviour to consume.

    This first-pass implementation stays intentionally simple:
    - use the tracked `gate` as the center reference
    - use `survey_repair` / `search_rescue` detections to choose the side
    - move through the chosen half of the gate while keeping the current depth
    """

    ROLE_LABELS = ("survey_repair", "search_rescue")

    def __init__(
        self,
        desired_role: str = "survey_repair",
        approach_distance: float = 1.0,
        pass_distance: float = 1.0,
        max_detection_age: int = 10,
        max_wait_time: float = 5.0,
        global_yaw_lock: bool = False,
        name: str = "Plan Gate Traversal",
    ):
        super().__init__(name)
        self.desired_role = desired_role
        self.approach_distance = approach_distance
        self.pass_distance = pass_distance
        self.max_detection_age = max_detection_age
        self.max_wait_time = max_wait_time
        self.global_yaw_lock = global_yaw_lock
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.start_time_sec = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/selected_role", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/gate/selected_side", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/gate/approach_x", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/gate/approach_y", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/gate/pass_x", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/gate/pass_y", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/gate/center_x", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/gate/center_y", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/gate/target_yaw", access=py_trees.common.Access.WRITE)

    def initialise(self):
        self.start_time_sec = self.node.get_clock().now().nanoseconds / 1e9

    def update(self):
        if not hasattr(self.blackboard, "sensors") or self.blackboard.sensors.pose is None:
            self.node.get_logger().warn(f"[{self.name}] Waiting for /sensors/pose.")
            return py_trees.common.Status.RUNNING

        if not hasattr(self.blackboard, "vision") or self.blackboard.vision.object_map is None:
            self.node.get_logger().warn(f"[{self.name}] Waiting for /vision/object_map.")
            return py_trees.common.Status.RUNNING

        object_array = self.blackboard.vision.object_map.array
        if len(object_array) == 0:
            return self._wait_or_fail("Object map is empty.")

        auv_pose = self.blackboard.sensors.pose.pose
        auv_x = auv_pose.position.x
        auv_y = auv_pose.position.y
        auv_yaw = yaw_from_quaternion(auv_pose.orientation)

        gate = self._closest_object(
            [
                obj for obj in object_array
                if obj.label == "gate" and obj.frames_since_last_seen <= self.max_detection_age
            ],
            auv_x,
            auv_y,
        )
        if gate is None:
            return self._wait_or_fail("No recent gate detection available.")

        role_objects = {
            label: self._closest_object(
                [
                    obj for obj in object_array
                    if obj.label == label and obj.frames_since_last_seen <= self.max_detection_age
                ],
                gate.pose.position.x,
                gate.pose.position.y,
            )
            for label in self.ROLE_LABELS
        }

        selected_role = self._select_role(role_objects)
        if selected_role is None:
            return self._wait_or_fail("No recent role panel detections available.")

        selected_role_obj = role_objects[selected_role]
        if selected_role_obj is None:
            return self._wait_or_fail(
                f"Requested role '{selected_role}' is not visible yet."
            )

        gate_x = gate.pose.position.x
        gate_y = gate.pose.position.y
        role_x = selected_role_obj.pose.position.x
        role_y = selected_role_obj.pose.position.y

        # the role panel sits on one half of the opening.
        # we aim directly for the role panel rather than the midpoint
        target_base_x = role_x
        target_base_y = role_y

        if self.global_yaw_lock:
            through_x, through_y = 1.0, 0.0
        else:
            through_x, through_y = self._compute_through_direction(gate, role_objects, auv_x, auv_y)
        approach_x = target_base_x - self.approach_distance * through_x
        approach_y = target_base_y - self.approach_distance * through_y
        pass_x = target_base_x + self.pass_distance * through_x
        pass_y = target_base_y + self.pass_distance * through_y
        target_yaw = math.atan2(through_y, through_x)

        other_role = "search_rescue" if selected_role == "survey_repair" else "survey_repair"
        other_role_obj = role_objects.get(other_role)

        chosen_side = self._compute_side_string(
            gate_x, gate_y, through_x, through_y, role_x, role_y, other_role_obj
        )

        self.blackboard.gate.selected_role = selected_role
        self.blackboard.gate.selected_side = chosen_side
        self.blackboard.gate.approach_x = approach_x
        self.blackboard.gate.approach_y = approach_y
        self.blackboard.gate.pass_x = pass_x
        self.blackboard.gate.pass_y = pass_y
        self.blackboard.gate.center_x = gate_x
        self.blackboard.gate.center_y = gate_y
        self.blackboard.gate.target_yaw = target_yaw

        self.node.get_logger().info(
            f"[{self.name}] Selected role={selected_role}, side={chosen_side}, "
            f"approach=({approach_x:.2f}, {approach_y:.2f}), pass=({pass_x:.2f}, {pass_y:.2f}), "
            f"yaw={math.degrees(target_yaw):.1f} deg"
        )
        return py_trees.common.Status.SUCCESS

    def _select_role(self, role_objects):
        visible_roles = [label for label, obj in role_objects.items() if obj is not None]
        if not visible_roles:
            return None

        if self.desired_role == "random":
            return random.choice(visible_roles)

        if self.desired_role in visible_roles:
            return self.desired_role

        if self.desired_role in self.ROLE_LABELS:
            return self.desired_role

        self.node.get_logger().warn(
            f"[{self.name}] Unknown desired_role='{self.desired_role}', defaulting to "
            f"{visible_roles[0]}."
        )
        return visible_roles[0]

    def _compute_through_direction(self, gate, role_objects, auv_x, auv_y):
        gate_x = gate.pose.position.x
        gate_y = gate.pose.position.y
        to_auv_x = auv_x - gate_x
        to_auv_y = auv_y - gate_y

        if gate.has_orientation:
            facing_yaw = yaw_from_quaternion(gate.pose.orientation)
            facing_x = math.cos(facing_yaw)
            facing_y = math.sin(facing_yaw)
            if facing_x * to_auv_x + facing_y * to_auv_y < 0.0:
                facing_x *= -1.0
                facing_y *= -1.0
            return -facing_x, -facing_y

        survey = role_objects["survey_repair"]
        rescue = role_objects["search_rescue"]
        if survey is not None and rescue is not None:
            span_x = survey.pose.position.x - rescue.pose.position.x
            span_y = survey.pose.position.y - rescue.pose.position.y
            span_norm = math.hypot(span_x, span_y)
            if span_norm > 1e-6:
                span_x /= span_norm
                span_y /= span_norm
                normal_a_x = -span_y
                normal_a_y = span_x
                normal_b_x = -normal_a_x
                normal_b_y = -normal_a_y
                dot_a = normal_a_x * to_auv_x + normal_a_y * to_auv_y
                dot_b = normal_b_x * to_auv_x + normal_b_y * to_auv_y
                facing_x, facing_y = (
                    (normal_a_x, normal_a_y) if dot_a >= dot_b else (normal_b_x, normal_b_y)
                )
                return -facing_x, -facing_y

        gate_to_auv_norm = math.hypot(to_auv_x, to_auv_y)
        if gate_to_auv_norm < 1e-6:
            return 1.0, 0.0
        return -to_auv_x / gate_to_auv_norm, -to_auv_y / gate_to_auv_norm

    def _compute_side_string(self, gate_x, gate_y, through_x, through_y, role_x, role_y, other_role_obj):
        if other_role_obj is not None:
            ref_x = other_role_obj.pose.position.x
            ref_y = other_role_obj.pose.position.y
        else:
            ref_x = gate_x
            ref_y = gate_y
            
        cross = through_x * (role_y - ref_y) - through_y * (role_x - ref_x)
        return "left" if cross >= 0.0 else "right"

    def _closest_object(self, objects, ref_x, ref_y):
        if not objects:
            return None
        return min(
            objects,
            key=lambda obj: math.hypot(obj.pose.position.x - ref_x, obj.pose.position.y - ref_y),
        )

    def _wait_or_fail(self, reason: str):
        elapsed = (self.node.get_clock().now().nanoseconds / 1e9) - self.start_time_sec
        if elapsed < self.max_wait_time:
            self.node.get_logger().info(
                f"[{self.name}] {reason} Waiting for more detections..."
            )
            return py_trees.common.Status.RUNNING

        self.node.get_logger().error(
            f"[{self.name}] {reason} Timed out after {self.max_wait_time:.1f}s."
        )
        return py_trees.common.Status.FAILURE


class NavigateThroughGateBehaviour(py_trees.behaviour.Behaviour):
    """
    Drives the AUV through the previously planned gate target while keeping the
    current depth unchanged.
    """

    def __init__(
        self,
        position_tolerance: float,
        hold_time: float,
        timeout: float,
        name: str = "Navigate Through Gate",
    ):
        super().__init__(name)
        self.position_tolerance = position_tolerance
        self.hold_time = hold_time
        self.timeout = timeout
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.action_status = ActionStatus.NOT_SENT
        self.current_phase = "approach"
        self.result_message = ""

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.navigation_client = kwargs["shared_nav_client"]
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)
        self.blackboard.register_key(key="/gate/approach_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/approach_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/pass_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/pass_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/gate/target_yaw", access=py_trees.common.Access.READ)

    def initialise(self):
        self.action_status = ActionStatus.NOT_SENT
        self.current_phase = "approach"
        self.result_message = ""

    def update(self):
        if self.action_status == ActionStatus.SUCCEEDED:
            if self.current_phase == "approach":
                self.node.get_logger().info(
                    f"[{self.name}] Reached gate approach point. {self.result_message}"
                )
                self.current_phase = "pass"
                self.action_status = ActionStatus.NOT_SENT
                self.result_message = ""
                return py_trees.common.Status.RUNNING

            self.node.get_logger().info(
                f"[{self.name}] Cleared the gate. {self.result_message}"
            )
            return py_trees.common.Status.SUCCESS

        if self.action_status == ActionStatus.FAILED:
            if self.current_phase == "approach":
                self.node.get_logger().error(
                    f"[{self.name}] Failed to reach gate approach point. {self.result_message}"
                )
            else:
                self.node.get_logger().error(
                    f"[{self.name}] Failed to pass through the gate. {self.result_message}"
                )
            return py_trees.common.Status.FAILURE

        if self.action_status == ActionStatus.PENDING:
            return py_trees.common.Status.RUNNING

        if self.current_phase == "approach":
            target_x = self.blackboard.gate.approach_x
            target_y = self.blackboard.gate.approach_y
            phase_label = "approach point"
        else:
            target_x = self.blackboard.gate.pass_x
            target_y = self.blackboard.gate.pass_y
            phase_label = "pass-through point"
            
        target_yaw = normalize_angle(self.blackboard.gate.target_yaw)

        self.node.get_logger().info(
            f"[{self.name}] Moving to {phase_label} ({target_x:.2f}, {target_y:.2f}) "
            f"yaw={math.degrees(target_yaw):.1f} deg"
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
        self.result_message = ""
        return py_trees.common.Status.RUNNING

    def _on_goal_response(self, goal_response: bool):
        if not goal_response:
            self.action_status = ActionStatus.FAILED

    def _on_goal_result(self, goal_success, message):
        self.result_message = message
        self.action_status = ActionStatus.SUCCEEDED if goal_success else ActionStatus.FAILED

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            if hasattr(self, "navigation_client") and self.navigation_client:
                self.navigation_client.reset_action_client()


class GateTask(py_trees.composites.Sequence):
    """
    sequence for the RoboSub 2026 Gate Task (Begin Assessment).

    - dives to working depth
    - sets a forward yaw
    - searches for gate
    - scans to populate the role-panel detections
    - chooses the role side
    - moves to a point in front of that side
    - drives through it
    """

    def __init__(
        self,
        position_tolerance: float,
        hold_time: float,
        timeout: float,
        desired_role: str = "survey_repair",
        search_attempts: int = 2,
        scan_angle_deg: float = 35.0,
        scan_pause_time: float = 1.0,
        approach_distance: float = 1.5,
        pass_distance: float = 2.0,
        global_yaw_lock: bool = False,
        force_blind_forward_dist: float = 0.0,
        scan_angular_tolerance_deg: float = 30.0,
        scan_hold_time: float = 0.1,
        scan_timeout: float = 30.0,
        initial_alignment_tolerance_deg: float = 5.0,
        initial_alignment_hold_time: float = 1.0,
        initial_alignment_timeout: float = 15.0,
        do_style_yaw: bool = False,
        style_yaw_degrees: float = 360.0,
        do_style_roll: bool = False,
        style_roll_degrees: float = 720.0,
        style_roll_torque: float = 15.0,
        style_roll_coast_degrees: float = 180.0,
        initial_depth: float = -1.2,
    ):
        super().__init__("Gate Task", memory=True)

        self.add_child(
            BasicActionBehaviour(
                name=f"Initial Dive ({initial_depth}m)",
                goal=set_depth(
                    z=initial_depth,
                    tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                ),
            )
        )

        import math
        self.add_child(
            BasicActionBehaviour(
                name="Initial Coin Flip Alignment",
                goal=set_global_yaw(
                    yaw_rad=0.0,
                    tolerance=math.radians(initial_alignment_tolerance_deg),
                    hold_time=initial_alignment_hold_time,
                    timeout=initial_alignment_timeout,
                ),
            )
        )

        if force_blind_forward_dist > 0.0:
            self.add_child(
                ForceBlindDriveBehaviour(
                    distance=force_blind_forward_dist,
                    target_yaw=0.0,
                    position_tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                    name=f"Forced Blind Drive ({force_blind_forward_dist}m)",
                )
            )
            return

        vision_sequence = py_trees.composites.Sequence("Gate Vision Sequence", memory=True)
        vision_sequence.add_children(
            [
                SearchSweepBehaviour(
                    target_class="gate",
                    num_steps=8,
                    max_attempts=search_attempts,
                    step_timeout=scan_pause_time,
                    clockwise=False,
                    look_at_on_success=True,
                    angular_tolerance_rad=math.radians(scan_angular_tolerance_deg),
                    turn_hold_time_s=scan_hold_time,
                    turn_timeout_s=scan_timeout,
                    name="Search Gate",
                ),
                ScanBehaviour(
                    scan_angle_deg=scan_angle_deg,
                    pause_time=scan_pause_time,
                    angular_tolerance_rad=math.radians(scan_angular_tolerance_deg),
                    turn_hold_time_s=scan_hold_time,
                    turn_timeout_s=scan_timeout,
                    name="Scan Gate Panels",
                ),
                PlanGateTraversalBehaviour(
                    desired_role=desired_role,
                    approach_distance=approach_distance,
                    pass_distance=pass_distance,
                    global_yaw_lock=global_yaw_lock,
                    name="Plan Gate Traversal",
                ),
                NavigateThroughGateBehaviour(
                    position_tolerance=position_tolerance,
                    hold_time=hold_time,
                    timeout=timeout,
                    name="Pass Through Gate",
                ),
            ]
        )

        if do_style_yaw:
            steps = 6 if style_yaw_degrees >= 720.0 else 3
            vision_sequence.add_child(
                create_style_yaw_spin_sequence(
                    total_degrees=style_yaw_degrees,
                    num_steps=steps,
                    angular_tolerance_deg=scan_angular_tolerance_deg,
                    hold_time_s=scan_hold_time,
                    timeout_s=scan_timeout,
                )
            )

        if do_style_roll:
            vision_sequence.add_child(
                create_style_rolling_flip_sequence(
                    roll_torque=style_roll_torque,
                    target_degrees=style_roll_degrees,
                    coast_degrees=style_roll_coast_degrees,
                    timeout_sec=scan_timeout,
                )
            )

        self.add_child(vision_sequence)
