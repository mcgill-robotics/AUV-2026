import math
import py_trees
from geometry_msgs.msg import Wrench, Quaternion
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ..action_status_enum import ActionStatus
from ..mission_behaviour_components import (
    BasicActionBehaviour,
    SetNodeParameterBehaviour,
)
from controls.goal_helpers import set_global_yaw, set_attitude
from controls.utils import normalize_angle, yaw_from_quaternion, quaternion_to_euler


class StyleYawStepBehaviour(BasicActionBehaviour):
    """
    Executes a single step of a Style Yaw Spin.
    To prevent error accumulation across multiple spins, each step calculates its target
    heading as an absolute angle relative to the initial starting heading captured on step 1.
    """

    def __init__(
        self,
        step_index: int,
        total_steps: int,
        total_degrees: float,
        angular_tolerance_deg: float,
        hold_time_s: float,
        timeout_s: float,
        name: str = None,
    ):
        super().__init__(name or f"Style Yaw Step {step_index}/{total_steps}")
        self.step_index = step_index
        self.total_steps = total_steps
        self.total_degrees = total_degrees
        self.angular_tol_rad = math.radians(angular_tolerance_deg)
        self.hold_time_s = hold_time_s
        self.timeout_s = timeout_s

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.blackboard.register_key("/style_maneuver_start_yaw", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("/style_maneuver_start_yaw", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        if not hasattr(self.blackboard, "sensors") or self.blackboard.sensors.pose is None:
            self.node.get_logger().info(
                f"[{self.name}] Waiting for sensor pose data...", throttle_duration_sec=2.0
            )
            return py_trees.common.Status.RUNNING

        try:
            start_yaw = self.blackboard.style_maneuver_start_yaw
        except KeyError:
            start_yaw = None

        # On the first step, capture the initial absolute global yaw heading
        if self.step_index == 1 and start_yaw is None:
            current_q = self.blackboard.sensors.pose.pose.orientation
            start_yaw = yaw_from_quaternion(current_q)
            self.blackboard.style_maneuver_start_yaw = start_yaw
            self.node.get_logger().info(
                f"[{self.name}] Locked start yaw for spin: {math.degrees(start_yaw):.1f}°"
            )

        if self.action_status is ActionStatus.NOT_SENT:
            try:
                start_yaw = self.blackboard.style_maneuver_start_yaw
            except KeyError:
                start_yaw = 0.0
            target_deg = (self.step_index / float(self.total_steps)) * self.total_degrees
            target_yaw = normalize_angle(start_yaw + math.radians(target_deg))

            self.goal = set_global_yaw(
                yaw_rad=target_yaw,
                tolerance=self.angular_tol_rad,
                hold_time=self.hold_time_s,
                timeout=self.timeout_s,
            )

        status = super().update()

        # When the final step succeeds, reset the start yaw for future runs
        if status == py_trees.common.Status.SUCCESS and self.step_index == self.total_steps:
            self.blackboard.style_maneuver_start_yaw = None

        return status


class SaveYawBehaviour(py_trees.behaviour.Behaviour):
    """Saves the current yaw from /sensors/pose to the blackboard."""

    def __init__(self, key_name: str = "style_maneuver_roll_start_yaw", name: str = "Save Initial Yaw"):
        super().__init__(name)
        self.key_name = key_name
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"/{self.key_name}", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs.get("node")

    def update(self) -> py_trees.common.Status:
        if not hasattr(self.blackboard, "sensors") or self.blackboard.sensors.pose is None:
            self.node.get_logger().info(
                f"[{self.name}] Waiting for sensor pose data...", throttle_duration_sec=2.0
            )
            return py_trees.common.Status.RUNNING

        q = self.blackboard.sensors.pose.pose.orientation
        yaw = yaw_from_quaternion(q)
        setattr(self.blackboard, self.key_name, yaw)
        self.node.get_logger().info(
            f"[{self.name}] Saved initial yaw: {math.degrees(yaw):.1f}°"
        )
        return py_trees.common.Status.SUCCESS


class RestoreYawBehaviour(BasicActionBehaviour):
    """Restores the saved yaw while commanding roll=0 and pitch=0."""

    def __init__(
        self,
        key_name: str = "style_maneuver_roll_start_yaw",
        angular_tolerance_deg: float = 8.0,
        hold_time_s: float = 0.5,
        timeout_s: float = 10.0,
        name: str = "Restore Upright Attitude (Roll=0, Pitch=0)",
    ):
        super().__init__(name)
        self.key_name = key_name
        self.angular_tol_rad = math.radians(angular_tolerance_deg)
        self.hold_time_s = hold_time_s
        self.timeout_s = timeout_s

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.blackboard.register_key(f"/{self.key_name}", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        if self.action_status is ActionStatus.NOT_SENT:
            try:
                yaw = getattr(self.blackboard, self.key_name)
            except (KeyError, AttributeError):
                yaw = 0.0

            self.goal = set_attitude(
                roll=0.0,
                pitch=0.0,
                yaw=yaw,
                tolerance=self.angular_tol_rad,
                hold_time=self.hold_time_s,
                timeout=self.timeout_s,
            )
            self.node.get_logger().info(
                f"[{self.name}] Commanding level attitude: Roll=0.0°, Pitch=0.0°, Yaw={math.degrees(yaw):.1f}°"
            )

        return super().update()


class ExecuteRollingFlipBehaviour(py_trees.behaviour.Behaviour):
    """
    Performs open-loop barrel rolls (e.g., 720° for 8x max style points in RoboSub Gate task).
    Since attitude controller gains are often tuned for stabilization rather than fast spins,
    applying direct roll torque while summing delta angles between IMU updates is simple and reliable.
    Stops torque early (by coast_degrees, default 180°) to let rotational momentum slow down while stabilizing.
    """

    def __init__(
        self,
        roll_torque: float = 15.0,
        target_degrees: float = 720.0,
        coast_degrees: float = 180.0,
        timeout_sec: float = 12.0,
        name: str = None,
    ):
        super().__init__(name or f"Execute Rolling Flip ({int(target_degrees)} deg)")
        self.roll_torque = roll_torque
        self.target_degrees = target_degrees
        self.coast_degrees = coast_degrees
        self.timeout_sec = timeout_sec

        self.pub_effort = None
        self.sub_imu = None
        self.accumulated_roll_deg = 0.0
        self.last_roll_rad = None
        self.start_time_sec = None
        self._finished = False

    def setup(self, **kwargs):
        self.node = kwargs.get("node")
        # Publish effort directly to attitude effort while controller is disabled
        self.pub_effort = self.node.create_publisher(
            Wrench, "/controls/attitude_effort", 1
        )
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_imu = self.node.create_subscription(
            Imu, "/auv_frame/imu", self._imu_callback, sensor_qos
        )

    def _imu_callback(self, msg: Imu):
        if self._finished or self.start_time_sec is None:
            return

        roll_current, _, _ = quaternion_to_euler(msg.orientation)
        if self.last_roll_rad is not None:
            delta_roll = normalize_angle(roll_current - self.last_roll_rad)
            self.accumulated_roll_deg += math.degrees(abs(delta_roll))
        self.last_roll_rad = roll_current

    def initialise(self):
        self.accumulated_roll_deg = 0.0
        self.last_roll_rad = None
        self.start_time_sec = self.node.get_clock().now().nanoseconds / 1e9
        self._finished = False
        self.node.get_logger().info(
            f"[{self.name}] Starting open-loop rolling flip! Torque = {self.roll_torque} Nm, Target = {self.target_degrees}°, Coasting early by {self.coast_degrees}°"
        )

    def update(self) -> py_trees.common.Status:
        if self._finished:
            return py_trees.common.Status.SUCCESS

        now = self.node.get_clock().now().nanoseconds / 1e9

        # Check timeout / max duration fallback
        if (now - self.start_time_sec) > self.timeout_sec:
            self.node.get_logger().warn(
                f"[{self.name}] Reached time limit ({self.timeout_sec}s)! Accumulated roll: {self.accumulated_roll_deg:.1f}°. Stopping torque."
            )
            self._stop_torque()
            self._finished = True
            # Return SUCCESS even on timeout if time-based rotation was desired or as fallback
            return py_trees.common.Status.SUCCESS

        # Publish torque
        wrench = Wrench()
        wrench.torque.x = float(self.roll_torque)
        self.pub_effort.publish(wrench)

        self.node.get_logger().info(
            f"[{self.name}] Rolling... Completed {self.accumulated_roll_deg:.1f}° / {self.target_degrees:.1f}° (Coast at {self.target_degrees - self.coast_degrees:.1f}°)",
            throttle_duration_sec=0.5,
        )

        # Check if coasting threshold reached (stop early so momentum slows down while stabilizing)
        if self.target_degrees > 0 and self.accumulated_roll_deg >= (self.target_degrees - self.coast_degrees):
            self.node.get_logger().info(
                f"[{self.name}] Reached {self.accumulated_roll_deg:.1f}°! Stopping torque early by {self.coast_degrees}° to let momentum slow down & stabilize."
            )
            self._stop_torque()
            self._finished = True
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def _stop_torque(self):
        if self.pub_effort is not None:
            wrench = Wrench()
            wrench.torque.x = 0.0
            self.pub_effort.publish(wrench)

    def terminate(self, new_status: py_trees.common.Status):
        self._stop_torque()


def create_style_yaw_spin_sequence(
    total_degrees: float = 360.0,
    num_steps: int = 3,
    angular_tolerance_deg: float = 8.0,
    hold_time_s: float = 0.5,
    timeout_s: float = 10.0,
) -> py_trees.composites.Sequence:
    """
    Creates a drift-free style yaw spin sequence (e.g., 360° or 720°).
    Each step targets an absolute global angle relative to the initial starting heading,
    preventing error accumulation across steps.
    """
    seq = py_trees.composites.Sequence(
        f"Style Yaw Spin Sequence ({int(total_degrees)}°)", memory=True
    )
    children = []
    for i in range(1, num_steps + 1):
        children.append(
            StyleYawStepBehaviour(
                step_index=i,
                total_steps=num_steps,
                total_degrees=total_degrees,
                angular_tolerance_deg=angular_tolerance_deg,
                hold_time_s=hold_time_s,
                timeout_s=timeout_s,
                name=f"Yaw Step {i}/{num_steps} ({int(total_degrees * i / num_steps)}°)",
            )
        )
    seq.add_children(children)
    return seq


def create_style_rolling_flip_sequence(
    roll_torque: float = 15.0,
    target_degrees: float = 720.0,
    coast_degrees: float = 180.0,
    timeout_sec: float = 12.0,
    **kwargs,
) -> py_trees.composites.Sequence:
    """
    Creates a sequence that performs an open-loop rolling flip with momentum coasting and upright attitude restore:
    1. Saves initial yaw before the maneuver.
    2. Disables attitude_controller, x_controller, and y_controller to prevent fighting and odometry jump.
    3. Performs open-loop barrel roll while summing delta angles between IMU updates, stopping torque early by coast_degrees (default 180°) so momentum slows down while stabilizing.
    4. Re-enables attitude_controller, x_controller, and y_controller.
    5. Restores the saved yaw while commanding level attitude (Roll=0, Pitch=0) so the attitude controller actively catches the slowing momentum and locks upright.
    """
    seq = py_trees.composites.Sequence(
        f"Style Rolling Flip Sequence ({int(target_degrees)}°)", memory=True
    )
    seq.add_children(
        [
            SaveYawBehaviour(
                key_name="style_maneuver_roll_start_yaw",
                name="Save Initial Yaw",
            ),
            SetNodeParameterBehaviour(
                node_name="/attitude_controller",
                param_name="enabled",
                param_value=False,
                name="Disable Attitude Controller",
            ),
            SetNodeParameterBehaviour(
                node_name="/x_controller",
                param_name="enabled",
                param_value=False,
                name="Disable X Controller",
            ),
            SetNodeParameterBehaviour(
                node_name="/y_controller",
                param_name="enabled",
                param_value=False,
                name="Disable Y Controller",
            ),
            ExecuteRollingFlipBehaviour(
                roll_torque=roll_torque,
                target_degrees=target_degrees,
                coast_degrees=coast_degrees,
                timeout_sec=timeout_sec,
                name=f"Execute {int(target_degrees)}° Barrel Roll (Coast {int(coast_degrees)}° early)",
            ),
            SetNodeParameterBehaviour(
                node_name="/attitude_controller",
                param_name="enabled",
                param_value=True,
                name="Re-enable Attitude Controller (Snap Orientation)",
            ),
            SetNodeParameterBehaviour(
                node_name="/x_controller",
                param_name="enabled",
                param_value=True,
                name="Re-enable X Controller (Snap DVL Position)",
            ),
            SetNodeParameterBehaviour(
                node_name="/y_controller",
                param_name="enabled",
                param_value=True,
                name="Re-enable Y Controller (Snap DVL Position)",
            ),
            RestoreYawBehaviour(
                key_name="style_maneuver_roll_start_yaw",
                name="Restore Upright Attitude (Roll=0, Pitch=0)",
            ),
        ]
    )
    return seq
