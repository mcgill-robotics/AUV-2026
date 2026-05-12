import math
import py_trees
from controls.goal_helpers import move_global
from controls.utils import yaw_from_quaternion, normalize_angle
from ..action_status_enum import ActionStatus


class MatchPipesBehaviour(py_trees.behaviour.Behaviour):
    """
    Reads the object map and performs geometric analysis to find a pipe triplet
    (1 red center + 2 white on either side forming a roughly straight line).

    Writes the computed navigation target (midpoint + perpendicular yaw) to the
    blackboard for the downstream NavigateToGap behaviour to consume.

    This is a one-shot computation that completes in a single tick.
    """
    def __init__(
        self,
        gate_side: str = "right",
        collinearity_threshold: float = 0.5,
        min_forward_dist: float = 0.5,
        name="Match Pipes",
    ):
        super().__init__(name)
        self.gate_side = gate_side
        self.collinearity_threshold = collinearity_threshold
        self.min_forward_dist = min_forward_dist

        self.blackboard = self.attach_blackboard_client(name=self.name)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/slalom/target_x", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/slalom/target_y", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/slalom/target_z", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/slalom/target_yaw", access=py_trees.common.Access.WRITE)

    def update(self):
        # --- Get AUV pose ---
        if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
            self.node.get_logger().warn(f"[{self.name}] Waiting for /sensors/pose.")
            return py_trees.common.Status.RUNNING

        auv_pose = self.blackboard.sensors.pose.pose
        auv_x = auv_pose.position.x
        auv_y = auv_pose.position.y
        auv_yaw = yaw_from_quaternion(auv_pose.orientation)
        # Unit vector in the AUV's forward direction (from its heading angle)
        fwd_x = math.cos(auv_yaw)
        fwd_y = math.sin(auv_yaw)

        # --- Get object map ---
        if not hasattr(self.blackboard, 'vision') or self.blackboard.vision.object_map is None:
            self.node.get_logger().error(f"[{self.name}] No object map available.")
            return py_trees.common.Status.FAILURE

        # --- Filter forward-only pipes ---
        # We use the dot product between the AUV→pipe vector and the AUV's
        # forward unit vector. If dot > 0, the pipe is in front of the AUV.
        # A positive min_forward_dist also rejects pipes that are too close.
        #   dot = (pipe - auv) · forward = |pipe - auv| * cos(angle)
        red_pipes = []
        white_pipes = []
        for obj in self.blackboard.vision.object_map.array:
            dx = obj.pose.position.x - auv_x
            dy = obj.pose.position.y - auv_y
            dot = dx * fwd_x + dy * fwd_y
            if dot < self.min_forward_dist:
                continue
            if obj.label == "red_pipe":
                red_pipes.append(obj)
            elif obj.label == "white_pipe":
                white_pipes.append(obj)

        self.node.get_logger().info(
            f"[{self.name}] Forward pipes: {len(red_pipes)} red, {len(white_pipes)} white"
        )

        if len(red_pipes) == 0:
            self.node.get_logger().error(f"[{self.name}] No red pipes found in front.")
            return py_trees.common.Status.FAILURE
        if len(white_pipes) < 2:
            self.node.get_logger().error(
                f"[{self.name}] Need >= 2 white pipes, found {len(white_pipes)}."
            )
            return py_trees.common.Status.FAILURE

        # --- Closest red pipe ---
        closest_red = min(
            red_pipes,
            key=lambda p: math.hypot(p.pose.position.x - auv_x, p.pose.position.y - auv_y),
        )
        red_x = closest_red.pose.position.x
        red_y = closest_red.pose.position.y
        self.node.get_logger().info(f"[{self.name}] Closest red pipe at ({red_x:.2f}, {red_y:.2f})")

        # --- Find best collinear white pair ---
        # For each pair of white pipes, we check whether the red pipe lies
        # approximately on the line between them (i.e., all 3 are collinear).
        #
        # Two tests are used:
        #
        # 1) PERPENDICULAR DISTANCE (collinearity test)
        #    Given line segment W1->W2, the perpendicular distance of point R
        #    from this line is computed using the 2D cross product:
        #      cross = |(W2 - W1) × (R - W1)| = |dx*(Ry-W1y) - dy*(Rx-W1x)|
        #      perp_dist = cross / |W2 - W1|
        #    If perp_dist < threshold, the 3 points are approximately collinear.
        #    (See: "Point-to-line distance" formula)
        #
        # 2) PROJECTION PARAMETER (between-ness test)
        #    We project the red pipe onto the W1->W2 line segment:
        #      t = ((R - W1) · (W2 - W1)) / |W2 - W1|²
        #    t=0 means R is at W1, t=1 means R is at W2.
        #    If 0.1 < t < 0.9, the red pipe is between the whites (with margin).
        #    (See: "Scalar projection" / "Parameter of closest point on line")
        best_pair = None
        best_perp = float('inf')

        for i in range(len(white_pipes)):
            for j in range(i + 1, len(white_pipes)):
                w1 = white_pipes[i]
                w2 = white_pipes[j]
                w1x, w1y = w1.pose.position.x, w1.pose.position.y
                w2x, w2y = w2.pose.position.x, w2.pose.position.y

                line_dx = w2x - w1x
                line_dy = w2y - w1y
                line_len = math.hypot(line_dx, line_dy)
                if line_len < 0.5:
                    continue

                # Perpendicular distance of red from the w1->w2 line
                cross = abs(line_dx * (red_y - w1y) - line_dy * (red_x - w1x))
                perp_dist = cross / line_len

                # Projection parameter: red must lie between the two whites
                t = ((red_x - w1x) * line_dx + (red_y - w1y) * line_dy) / (line_len ** 2)

                if perp_dist < self.collinearity_threshold and 0.1 < t < 0.9:
                    if perp_dist < best_perp:
                        best_perp = perp_dist
                        best_pair = (w1, w2)

        if best_pair is None:
            self.node.get_logger().error(
                f"[{self.name}] No collinear white pipe pair found "
                f"(threshold={self.collinearity_threshold}m)."
            )
            return py_trees.common.Status.FAILURE

        w1, w2 = best_pair
        w1x, w1y = w1.pose.position.x, w1.pose.position.y
        w2x, w2y = w2.pose.position.x, w2.pose.position.y

        # --- Determine left/right white relative to AUV heading ---
        # We use the 2D cross product of the AUV's forward vector with the
        # AUV->pipe vector to determine which side each pipe is on:
        #   cross = fwd × (pipe - auv) = fwd_x*(py-ay) - fwd_y*(px-ax)
        # Positive cross -> pipe is to the LEFT of the AUV's heading.
        # Negative cross -> pipe is to the RIGHT.
        # (This is the "signed area" / "determinant" side-of-line test.)
        w1_cross = fwd_x * (w1y - auv_y) - fwd_y * (w1x - auv_x)
        w2_cross = fwd_x * (w2y - auv_y) - fwd_y * (w2x - auv_x)

        if w1_cross > w2_cross:
            left_x, left_y, left_z = w1x, w1y, w1.pose.position.z
            right_x, right_y, right_z = w2x, w2y, w2.pose.position.z
        else:
            left_x, left_y, left_z = w2x, w2y, w2.pose.position.z
            right_x, right_y, right_z = w1x, w1y, w1.pose.position.z

        self.node.get_logger().info(
            f"[{self.name}] Matched triplet: "
            f"L-white({left_x:.2f},{left_y:.2f}) "
            f"Red({red_x:.2f},{red_y:.2f}) "
            f"R-white({right_x:.2f},{right_y:.2f})"
        )

        # --- Compute midpoint and depth based on gate side ---
        if self.gate_side == "left":
            target_x = (red_x + left_x) / 2.0
            target_y = (red_y + left_y) / 2.0
            target_z = (closest_red.pose.position.z + left_z) / 2.0
        else:
            target_x = (red_x + right_x) / 2.0
            target_y = (red_y + right_y) / 2.0
            target_z = (closest_red.pose.position.z + right_z) / 2.0

        # --- Compute perpendicular yaw (facing forward through the gap) ---
        # The pipe line direction is the vector from left_white -> right_white.
        # The angle of this line is: line_angle = atan2(dy, dx).
        # A perpendicular to this line is ±90° from line_angle.
        # There are two perpendicular directions (one forward, one backward).
        # We pick the one that is closest to the AUV's current heading,
        # which guarantees the AUV will face forward through the gap.
        pipe_line_dx = right_x - left_x
        pipe_line_dy = right_y - left_y
        line_angle = math.atan2(pipe_line_dy, pipe_line_dx)
        perp1 = normalize_angle(line_angle + math.pi / 2)
        perp2 = normalize_angle(line_angle - math.pi / 2)

        # Pick the perpendicular closer to AUV's current heading
        diff1 = abs(normalize_angle(perp1 - auv_yaw))
        diff2 = abs(normalize_angle(perp2 - auv_yaw))
        target_yaw = perp1 if diff1 < diff2 else perp2

        # --- Write results to blackboard ---
        self.blackboard.slalom.target_x = target_x
        self.blackboard.slalom.target_y = target_y
        self.blackboard.slalom.target_z = target_z
        self.blackboard.slalom.target_yaw = target_yaw

        self.node.get_logger().info(
            f"[{self.name}] Target: ({target_x:.2f}, {target_y:.2f}, z={target_z:.2f}) "
            f"yaw={math.degrees(target_yaw):.1f}° (gate_side={self.gate_side})"
        )
        return py_trees.common.Status.SUCCESS


class NavigateToGapBehaviour(py_trees.behaviour.Behaviour):
    """
    Reads the slalom navigation target from the blackboard and drives the AUV
    to the computed gap midpoint with the perpendicular orientation.
    """
    def __init__(self, adjust_depth: bool = False, name="Navigate To Gap"):
        super().__init__(name)
        self.adjust_depth = adjust_depth
        self.blackboard = self.attach_blackboard_client(name=self.name)

        self.action_status = ActionStatus.NOT_SENT
        self.sent_goal = False

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)
        self.blackboard.register_key(key="/slalom/target_x", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/slalom/target_y", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/slalom/target_z", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/slalom/target_yaw", access=py_trees.common.Access.READ)

    def initialise(self):
        self.action_status = ActionStatus.NOT_SENT
        self.sent_goal = False

    def update(self):
        if self.action_status == ActionStatus.SUCCEEDED:
            self.node.get_logger().info(f"[{self.name}] Reached gap midpoint.")
            return py_trees.common.Status.SUCCESS

        if self.action_status == ActionStatus.FAILED:
            self.node.get_logger().error(f"[{self.name}] Failed to reach gap midpoint.")
            return py_trees.common.Status.FAILURE

        if self.action_status == ActionStatus.PENDING:
            return py_trees.common.Status.RUNNING

        # Send approach goal
        target_x = self.blackboard.slalom.target_x
        target_y = self.blackboard.slalom.target_y
        target_z = self.blackboard.slalom.target_z
        target_yaw = self.blackboard.slalom.target_yaw

        self.node.get_logger().info(
            f"[{self.name}] Approaching gap at ({target_x:.2f}, {target_y:.2f}) "
            f"yaw={math.degrees(target_yaw):.1f}°, do_z={self.adjust_depth}"
        )
        goal = move_global(
            x=target_x, 
            y=target_y, 
            z=target_z if self.adjust_depth else 0.0,
            yaw=target_yaw, 
            do_z=self.adjust_depth, 
            timeout=60.0
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

    def _on_goal_result(self, goal_success: bool):
        self.action_status = ActionStatus.SUCCEEDED if goal_success else ActionStatus.FAILED

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            if hasattr(self, 'navigation_client') and self.navigation_client:
                self.navigation_client.reset_action_client()
