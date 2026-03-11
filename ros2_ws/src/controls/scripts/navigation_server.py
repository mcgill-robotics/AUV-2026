#!/usr/bin/env python3
"""
Navigation Action Server

Hosts the /motion/navigate action server that orchestrates the AUV's PID
controllers (depth, X, Y, attitude) for goal-driven navigation.

Subscribes:
    state/pose (geometry_msgs/PoseStamped) - current AUV pose in pool frame

Publishes setpoints to:
    /controls/depth_setpoint (std_msgs/Float64)
    /controls/x_setpoint     (std_msgs/Float64)
    /controls/y_setpoint     (std_msgs/Float64)
    /controls/quaternion_setpoint (geometry_msgs/Quaternion)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float64
from auv_msgs.action import AUVNavigate

from controls.utils import yaw_from_quaternion, quaternion_from_yaw, normalize_angle


class NavigationServer(Node):
    """Action server that drives the AUV to target poses via PID controllers."""

    def __init__(self):
        super().__init__('navigation_server')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('feedback_rate_hz', 10.0)
        self.declare_parameter('state_topic', '/auv/ground_truth/pose')

        self.feedback_rate_hz = self.get_parameter('feedback_rate_hz').value
        state_topic = self.get_parameter('state_topic').value
        self._feedback_period = 1.0 / self.feedback_rate_hz

        # ── State ────────────────────────────────────────────────────────────
        self.current_pose = None  # Latest PoseStamped from state_aggregator

        # Callback group for concurrent action + subscriptions
        self.cb_group = ReentrantCallbackGroup()

        # QoS for sensor data (latest reading only)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriber: AUV state ────────────────────────────────────────────
        self.pose_sub = self.create_subscription(
            PoseStamped,
            state_topic,
            self._pose_callback,
            sensor_qos,
            callback_group=self.cb_group,
        )

        # ── Publishers: controller setpoints ─────────────────────────────────
        self.pub_depth_sp = self.create_publisher(Float64, '/controls/depth_setpoint', sensor_qos)
        self.pub_x_sp = self.create_publisher(Float64, '/controls/x_setpoint', sensor_qos)
        self.pub_y_sp = self.create_publisher(Float64, '/controls/y_setpoint', sensor_qos)
        self.pub_quat_sp = self.create_publisher(Quaternion, '/controls/quaternion_setpoint', sensor_qos)

        # ── Action server ────────────────────────────────────────────────────
        self.action_server = ActionServer(
            self,
            AUVNavigate,
            '/motion/navigate',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('Navigation server ready on /motion/navigate')

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _pose_callback(self, msg: PoseStamped):
        """Cache the latest AUV pose."""
        self.current_pose = msg

    def _goal_callback(self, goal_request):
        """Accept all incoming goals (preemption handled by action server)."""
        self.get_logger().info('Received navigation goal')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Accept all cancel requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    # ─────────────────────────────────────────────────────────────────────────
    # Goal execution
    # ─────────────────────────────────────────────────────────────────────────

    async def _execute_callback(self, goal_handle):
        """Main execution loop for a navigation goal."""
        goal = goal_handle.request
        result = AUVNavigate.Result()

        # ── Wait for state data ──────────────────────
        if self.current_pose is None:
            self.get_logger().warn('Waiting for state/pose...')
            for _ in range(50):  # Wait up to 5 seconds
                if self.current_pose is not None:
                    break
                await self._sleep(0.1)
            if self.current_pose is None:
                result.success = False
                result.message = 'No state/pose data received'
                goal_handle.abort()
                return result

        # ── Compute absolute target ──────────────────
        target_x, target_y, target_z, target_yaw = self._compute_absolute_target(goal)
        self._log_target(goal, target_x, target_y, target_z, target_yaw)

        # ── Publish setpoints to controllers ─────────
        self._publish_setpoints(goal, target_x, target_y, target_z, target_yaw)

        # ── Feedback loop ────────────────────────────
        feedback_msg = AUVNavigate.Feedback()
        rate_period = self._feedback_period
        time_in_tolerance = 0.0
        start_time = self.get_clock().now()

        while rclpy.ok():
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = 'Goal canceled'
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return result

            # Check for preemption (goal_handle becomes inactive if preempted)
            if not goal_handle.is_active:
                self.get_logger().info('Goal preempted by new goal')
                result.success = False
                result.message = 'Preempted'
                return result

            # Read current state
            if self.current_pose is None:
                await self._sleep(rate_period)
                continue

            current = self.current_pose.pose
            current_yaw = yaw_from_quaternion(current.orientation)

            # Compute errors (only for controlled DOFs)
            pos_error = self._compute_position_error(
                goal, current, target_x, target_y, target_z
            )
            yaw_error = abs(normalize_angle(target_yaw - current_yaw)) if goal.do_yaw else 0.0

            # Check convergence
            pos_converged = pos_error <= goal.position_tolerance
            yaw_converged = yaw_error <= goal.yaw_tolerance

            if pos_converged and yaw_converged:
                time_in_tolerance += rate_period
            else:
                time_in_tolerance = 0.0

            # Publish feedback
            feedback_msg.current_pose = current
            feedback_msg.position_error = pos_error
            feedback_msg.yaw_error = yaw_error
            feedback_msg.time_in_tolerance = time_in_tolerance
            goal_handle.publish_feedback(feedback_msg)

            # Success condition
            if time_in_tolerance >= goal.hold_time:
                result.success = True
                result.message = f'Reached target (held for {goal.hold_time:.1f}s)'
                goal_handle.succeed()
                self.get_logger().info(result.message)
                return result

            # Timeout condition
            if goal.timeout > 0.0:
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed >= goal.timeout:
                    result.success = False
                    result.message = (
                        f'Timed out after {goal.timeout:.1f}s '
                        f'(pos_err={pos_error:.3f}m, yaw_err={math.degrees(yaw_error):.1f}deg)'
                    )
                    goal_handle.abort()
                    self.get_logger().warn(result.message)
                    return result

            await self._sleep(rate_period)

        # Node shutting down
        result.success = False
        result.message = 'Node shutdown'
        goal_handle.abort()
        return result

    # ─────────────────────────────────────────────────────────────────────────
    # Target computation
    # ─────────────────────────────────────────────────────────────────────────

    def _compute_absolute_target(self, goal):
        """Resolve the goal into absolute pool-frame coordinates."""
        current = self.current_pose.pose
        current_yaw = yaw_from_quaternion(current.orientation)
        goal_yaw = yaw_from_quaternion(goal.target_pose.orientation)

        if not goal.is_relative:
            # Absolute target - use directly
            target_x = goal.target_pose.position.x
            target_y = goal.target_pose.position.y
            target_z = goal.target_pose.position.z
            target_yaw = goal_yaw
        else:
            dx = goal.target_pose.position.x
            dy = goal.target_pose.position.y
            dz = goal.target_pose.position.z

            if goal.is_local_frame:
                # Rotate body-frame offsets by current yaw
                cos_yaw = math.cos(current_yaw)
                sin_yaw = math.sin(current_yaw)
                pool_dx = cos_yaw * dx - sin_yaw * dy
                pool_dy = sin_yaw * dx + cos_yaw * dy
                target_x = current.position.x + pool_dx
                target_y = current.position.y + pool_dy
            else:
                # Field-centric: offsets are already in pool frame
                target_x = current.position.x + dx
                target_y = current.position.y + dy

            target_z = current.position.z + dz
            target_yaw = normalize_angle(current_yaw + goal_yaw)

        return target_x, target_y, target_z, target_yaw

    def _compute_position_error(self, goal, current, target_x, target_y, target_z):
        """Compute position error considering only controlled DOFs."""
        err_sq = 0.0
        if goal.do_x:
            err_sq += (target_x - current.position.x) ** 2
        if goal.do_y:
            err_sq += (target_y - current.position.y) ** 2
        if goal.do_z:
            err_sq += (target_z - current.position.z) ** 2
        return math.sqrt(err_sq)

    # ─────────────────────────────────────────────────────────────────────────
    # Setpoint publishing
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_setpoints(self, goal, target_x, target_y, target_z, target_yaw):
        """Publish setpoints to the relevant PID controllers."""
        if goal.do_z:
            msg = Float64()
            msg.data = -target_z  # Convert from pool Z (negative down) to depth (positive down)
            self.pub_depth_sp.publish(msg)

        if goal.do_x:
            msg = Float64()
            msg.data = target_x
            self.pub_x_sp.publish(msg)

        if goal.do_y:
            msg = Float64()
            msg.data = target_y
            self.pub_y_sp.publish(msg)

        if goal.do_yaw:
            quat_msg = quaternion_from_yaw(target_yaw)
            self.pub_quat_sp.publish(quat_msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Utilities
    # ─────────────────────────────────────────────────────────────────────────

    def _log_target(self, goal, x, y, z, yaw):
        """Log a human-readable description of the goal."""
        parts = []
        if goal.do_x:
            parts.append(f'x={x:.2f}')
        if goal.do_y:
            parts.append(f'y={y:.2f}')
        if goal.do_z:
            parts.append(f'z={z:.2f}')
        if goal.do_yaw:
            parts.append(f'yaw={math.degrees(yaw):.1f}deg')

        mode = 'absolute'
        if goal.is_relative and goal.is_local_frame:
            mode = 'local frame relative'
        elif goal.is_relative:
            mode = 'map frame relative'

        self.get_logger().info(
            f'Navigating ({mode}): {", ".join(parts)} '
            f'[tol={goal.position_tolerance:.2f}m, '
            f'yaw_tol={math.degrees(goal.yaw_tolerance):.1f}deg, '
            f'hold={goal.hold_time:.1f}s, '
            f'timeout={goal.timeout:.1f}s]'
        )

    async def _sleep(self, seconds: float):
        """Async sleep using a one-shot ROS timer."""
        future = rclpy.task.Future()

        def _done():
            future.set_result(True)
            timer.cancel()

        timer = self.create_timer(seconds, _done, callback_group=self.cb_group)
        await future


def main(args=None):
    rclpy.init(args=args)
    node = NavigationServer()

    # MultiThreadedExecutor needed for concurrent action execution + subscriptions
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
