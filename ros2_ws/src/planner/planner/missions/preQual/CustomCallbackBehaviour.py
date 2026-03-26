import math
import rclpy
import py_trees
from rclpy.node import Node
from rclpy.action import ActionClient
from auv_msgs.action import AUVNavigate
from controls.goal_helpers import move_global

class CustomCallback(py_trees.behaviour.Behaviour):
    """
    A behavior that executes an orbital trajectory around a target point
    by sending a sequence of discrete absolute navigation goals.
    
    Absolute waypoints are calculated dynamically during the update() tick,
    allowing the path to track a moving center point and preventing the
    accumulation of relative path drift.
    """
    def __init__(
        self, 
        name="CustomCallback", 
        node=None, 
        rotations_segments=5, 
        angle_to_rotate_deg=360, 
        radius_to_rotate_meter=20.0, 
        clockwise=False
    ) -> None:
        super().__init__(name)
        self.node = node
        self.rotations_segments = rotations_segments
        self.clockwise = clockwise
        self.radius_to_rotate_meter = radius_to_rotate_meter
        self.angle_to_rotate_rad = math.radians(angle_to_rotate_deg)
        
        self.action_client = ActionClient(self.node, AUVNavigate, '/motion/navigate')

        # Calculate standard step sizes and tolerances based on the arc
        self.rotate_per_segment = self.angle_to_rotate_rad / self.rotations_segments
        
        # Estimate the straight-line distance of the arc segment to establish a reasonable tolerance
        chord_length = math.sqrt(2 * radius_to_rotate_meter**2 * (1 - math.cos(self.rotate_per_segment)))
        self.initial_translation_tolerance = 0.5 * chord_length
        self.yaw_tolerance = 0.5 * self.rotate_per_segment
        
        # Add 1 to the required segments to include the initial approach vector to the perimeter
        self.remaining_segments = self.rotations_segments + 1
        self.is_waiting_for_result = False
        self.mission_failed = False
        self.current_goal_handle = None

    def setup(self, **kwargs) -> None:
        """Called once when the BT tree is setup."""
        if self.node:
            self.node.get_logger().info(f"[{self.name}] Waiting for Navigation Server...")
        self.action_client.wait_for_server(timeout_sec=5.0)
        
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)

    def initialise(self) -> None:
        """Called every time this behavior transitions into RUNNING."""
        self.remaining_segments = self.rotations_segments + 1
        self.is_waiting_for_result = False
        self.mission_failed = False
        self.current_goal_handle = None
        
        self.target_x = 13.0
        self.target_y = 0.0
        self.target_z = 0.0
        
        self.start_angle = None

    def update(self) -> py_trees.common.Status:
        """Tick function. Calculates the absolute waypoint for the current segment and sends the goal."""
        
        # Block execution if the AUV poses are not published yet
        if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
            self.node.get_logger().info(f"[{self.name}] Waiting for sensor pose data...", throttle_duration_sec=2.0)
            return py_trees.common.Status.RUNNING
            
        # Check for failure condition from the async callbacks
        if self.mission_failed:
            self.node.get_logger().error(f"[{self.name}] Orbit failed midway.")
            return py_trees.common.Status.FAILURE
            
        # Completion check
        if self.remaining_segments <= 0:
            self.node.get_logger().info(f"[{self.name}] Completed all segments.")
            return py_trees.common.Status.SUCCESS

        # Block loop if currently navigating to a waypoint
        if self.is_waiting_for_result:
            return py_trees.common.Status.RUNNING

        # Compute current segment index (0 to N)
        current_segment = (self.rotations_segments + 1) - self.remaining_segments

        # Target center to orbit. Placeholder values; can be read from Vision blackboard keys
        target_x = self.target_x
        target_y = self.target_y
        target_z = self.target_z

        # Calculate the starting phase angle based on the AUV's current position relative to the target.
        # This ensures the circle sequence starts dynamically from the AUV's angle of approach.
        if self.start_angle is None:
            auv_x = self.blackboard.sensors.pose.pose.position.x
            auv_y = self.blackboard.sensors.pose.pose.position.y
            self.start_angle = math.atan2(auv_y - target_y, auv_x - target_x)

        angle_step = self.rotate_per_segment
        if self.clockwise:
            angle_step = -angle_step
            
        current_angle = self.start_angle + (current_segment * angle_step)

        # Calculate absolute Cartesian coordinates on the circle
        waypoint_x = target_x + (self.radius_to_rotate_meter * math.cos(current_angle))
        waypoint_y = target_y + (self.radius_to_rotate_meter * math.sin(current_angle))
        
        # Calculate optimal yaw to face the target coordinates
        dy = target_y - waypoint_y
        dx = target_x - waypoint_x
        look_at_yaw = math.atan2(dy, dx)

        self.node.get_logger().info(f"[{self.name}] Sending absolute goal for segment {current_segment}/{self.rotations_segments}")

        # Adjust tolerance and hold times depending on the stage of the sequence
        tolerance = self.initial_translation_tolerance
        hold_time = 0.2
        
        if current_segment == 0:
            hold_time = 1.0
        elif self.remaining_segments == 1:
            tolerance = 0.3 * self.initial_translation_tolerance
            hold_time = 1.0

        goal_msg = move_global(
            x=waypoint_x, 
            y=waypoint_y, 
            z=target_z, 
            yaw=look_at_yaw, 
            tolerance=tolerance, 
            yaw_tolerance=self.yaw_tolerance, 
            hold_time=hold_time, 
            timeout=30.0,
            do_z=False
        )

        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)
        
        # Flag state to await async completion
        self.is_waiting_for_result = True
        return py_trees.common.Status.RUNNING

    def _goal_response_callback(self, future):
        """Async callback triggered when server accepts or rejects the goal request."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.mission_failed = True
            return

        self.current_goal_handle = goal_handle 
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Async callback triggered when server finishes processing the goal."""
        result = future.result().result
        if result.success:
            self.remaining_segments -= 1
            self.is_waiting_for_result = False
            self.current_goal_handle = None
        else:
            self.mission_failed = True

    def terminate(self, new_status: py_trees.common.Status):
        """Called if the tree aborts this branch or if it naturally finishes."""
        if new_status == py_trees.common.Status.INVALID:
            if self.node:
                self.node.get_logger().warn(f"[{self.name}] Aborted branch. Canceling active goal.")
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None