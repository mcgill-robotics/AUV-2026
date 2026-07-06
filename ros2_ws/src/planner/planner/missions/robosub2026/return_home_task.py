import math
import py_trees
from controls.goal_helpers import move_global, set_depth
from controls.utils import normalize_angle
from ..action_status_enum import ActionStatus
from ..mission_behaviour_components import BasicActionBehaviour
from ..vision_behaviours import GoNearObject
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
            if hasattr(self.blackboard, "gate") and self.blackboard.gate.center_x is not None:
                cx = self.blackboard.gate.center_x
                cy = self.blackboard.gate.center_y
            else:
                self.node.get_logger().error(f"[{self.name}] No return point found in map or blackboard!")
                return py_trees.common.Status.FAILURE

        if self.global_yaw_lock:
            yaw = 0.0
        elif hasattr(self.blackboard, "gate") and hasattr(self.blackboard.gate, "target_yaw") and self.blackboard.gate.target_yaw is not None:
            yaw = self.blackboard.gate.target_yaw
        else:
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
            if hasattr(self.blackboard, "gate") and self.blackboard.gate.center_x is not None:
                cx = self.blackboard.gate.center_x
                cy = self.blackboard.gate.center_y
            else:
                self.node.get_logger().error(f"[{self.name}] No return point found in map or blackboard!")
                return py_trees.common.Status.FAILURE

        if self.global_yaw_lock:
            yaw = 0.0
        elif hasattr(self.blackboard, "gate") and hasattr(self.blackboard.gate, "target_yaw") and self.blackboard.gate.target_yaw is not None:
            yaw = self.blackboard.gate.target_yaw
        else:
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
        **kwargs,
    ):
        super().__init__("Return Home Task", memory=True)

        return_children = [
            NavigateToReturnPoint(
                return_distance=return_distance,
                position_tolerance=position_tolerance,
                hold_time=hold_time,
                timeout=timeout,
                global_yaw_lock=global_yaw_lock,
            ),
            # Search / visually approach the gate from the back
            GoNearObject(
                target_class="gate",
                target_distance=approach_distance,
                tolerance_meters=position_tolerance,
                hold_time=1.0,
                name="Visually Approach Back of Gate"
            ),
        ]

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

        # If anything in the return sequence fails, we still want to surface!
        failsafe_return = py_trees.decorators.FailureIsSuccess(
            name="Failsafe Return Wrapper",
            child=return_sequence
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
