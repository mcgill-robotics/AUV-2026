import math
import py_trees
from controls.goal_helpers import move_global, set_depth
from controls.utils import normalize_angle
from ..action_status_enum import ActionStatus
from ..mission_behaviour_components import BasicActionBehaviour
from ..vision_behaviours import GoNearObject

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
        name: str = "Navigate to Return Point",
    ):
        super().__init__(name)
        self.return_distance = return_distance
        self.position_tolerance = position_tolerance
        self.hold_time = hold_time
        self.timeout = timeout
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.action_status = ActionStatus.NOT_SENT

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.navigation_client = kwargs["shared_nav_client"]
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

        if not hasattr(self.blackboard, "gate") or self.blackboard.gate.center_x is None:
            self.node.get_logger().error(f"[{self.name}] Gate center not found on blackboard!")
            return py_trees.common.Status.FAILURE

        cx = self.blackboard.gate.center_x
        cy = self.blackboard.gate.center_y
        yaw = self.blackboard.gate.target_yaw

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
        name: str = "Pass Back Through Gate",
    ):
        super().__init__(name)
        self.pass_distance = pass_distance
        self.position_tolerance = position_tolerance
        self.hold_time = hold_time
        self.timeout = timeout
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.action_status = ActionStatus.NOT_SENT

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.navigation_client = kwargs["shared_nav_client"]
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

        if not hasattr(self.blackboard, "gate") or self.blackboard.gate.center_x is None:
            self.node.get_logger().error(f"[{self.name}] Gate center not found on blackboard!")
            return py_trees.common.Status.FAILURE

        cx = self.blackboard.gate.center_x
        cy = self.blackboard.gate.center_y
        yaw = self.blackboard.gate.target_yaw

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
    ):
        super().__init__("Return Home Task", memory=True)

        self.add_children([
            NavigateToReturnPoint(
                return_distance=return_distance,
                position_tolerance=position_tolerance,
                hold_time=hold_time,
                timeout=timeout,
            ),
            # Search / visually approach the gate from the back
            GoNearObject(
                target_class="gate",
                target_distance=approach_distance,
                tolerance_meters=position_tolerance,
                hold_time=1.0,
                name="Visually Approach Back of Gate"
            ),
            PassBackThroughGate(
                pass_distance=pass_distance,
                position_tolerance=position_tolerance,
                hold_time=hold_time,
                timeout=timeout,
            ),
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
