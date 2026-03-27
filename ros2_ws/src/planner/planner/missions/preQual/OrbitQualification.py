import py_trees
import py_trees_ros
import rclpy
from controls import navigation_client
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from controls.goal_helpers import set_depth, set_global_yaw, move_global
from ...SensorsBehaviour import SensorsBehaviour
from ...utils.BasicActionBehaviour import BasicActionBehaviour
from ...utils.ActionStatus import ActionStatus
from ...utils.MissionChoiceCheckBehaviour import MissionChoiceCheckBehaviour
from ...utils.MissionCompleteBehaviour import MissionCompleteBehaviour
from ...utils.TimerBehaviour import TimerBehaviour
import math

class OrbitQualificationMission(py_trees.composites.Sequence):
    """
    This behaviour is the root of the orbit Pre-qualification mission
    """
    def __init__(self, node):
        super().__init__("OrbitPrequalification", memory=True)

        # 0 Check if user input the desired mission choce
        """
        1: Orbit Prequal
        2: Rectangle Prequal
        3: Basic Move forward
        4: Basic Dive
        5: Basic Yaw
        """
        mission_choice_check = MissionChoiceCheckBehaviour(name="OrbitPrequalUserCheck", choice=1)

        # 1 Wait for 10 seconds before starting the mission
        timer = TimerBehaviour(node=node, timer=10.0, name="Orbit Prequal Timer")

        # Build the full mission sequence
        # 2. Dive to -1.5m
        dive_leaf = BasicActionBehaviour(node, "Dive", set_depth(z=-1.5, tolerance=0.15, hold_time=2.0))
        
        # 3. Go to point where orbit begins 
        go_orbit_start_leaf = BasicActionBehaviour(node, "Move to rectangle start point", move_global(x=11.5, y=0.0, do_z=False, tolerance=0.5, hold_time=1.0))

        # 3.5 (Add a vision check? no rotation unless object in frame)

        # 4. Orbit 360 degrees
        # (Add future check for target with vision  )
        orbit_leaf = OrbitActionBehaviour(node, name="Orbit", rotations_segments=8, angle_to_rotate_deg=360, radius_to_rotate_meter=1.5, clockwise=True, target=(13,0,0))
        
        # 5. Turn 180 degrees to look at the gate
        turn_leaf = BasicActionBehaviour(node, "Turn 180", set_global_yaw(yaw_rad=math.pi, tolerance=0.1, hold_time=1.0))
        
        # 6. Return to origin (X=0, Y=0) at the current depth
        return_leaf = BasicActionBehaviour(node, "Return to Origin", move_global(x=0.0, y=0.0, do_z=False, tolerance=0.5, hold_time=1.0))
        
        # 7. Ascend to surface
        ascend_leaf = BasicActionBehaviour(node, "Ascend to Surface", set_depth(z=0.0, tolerance=0.15, hold_time=1.0))

        # 8. Reset the user mission choice to allow for new mission to be selected
        mission_choice_reset = MissionCompleteBehaviour(node, "Completed Orbit Prequal")
        
        self.add_children([mission_choice_check,
            timer,
            dive_leaf, 
            go_orbit_start_leaf,
            orbit_leaf, 
            turn_leaf, 
            return_leaf, 
            ascend_leaf,
            mission_choice_reset
            ])


class OrbitActionBehaviour(py_trees.behaviour.Behaviour):
    """
    A behavior that executes an orbital trajectory around a target point
    by sending a sequence of discrete absolute navigation goals.
    
    Absolute waypoints are calculated dynamically during the update() tick,
    allowing the path to track a moving center point and preventing the
    accumulation of relative path drift.
    """
    def __init__(
        self, 
        node=None,
        name="CustomCallback",  
        rotations_segments=5, 
        angle_to_rotate_deg=360, 
        radius_to_rotate_meter=20.0, 
        clockwise=False,
        target: tuple[float, float, float] = None
    ) -> None:
        super().__init__(name)
        self.node = node
        self.rotations_segments = rotations_segments
        self.clockwise = clockwise
        self.radius_to_rotate_meter = radius_to_rotate_meter
        self.angle_to_rotate_rad = math.radians(angle_to_rotate_deg) 
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.target = target
        self.start_angle = None

        # Set the action status to be used to keep track of state of Behaviour
        self.action_status = ActionStatus.NOT_SENT

        # Calculate standard step sizes and tolerances based on the arc
        self.rotate_per_segment = self.angle_to_rotate_rad / self.rotations_segments
        
        # Estimate the straight-line distance of the arc segment to establish a reasonable tolerance
        linear_translation = math.sqrt(2 * radius_to_rotate_meter**2 * (1 - math.cos(self.rotate_per_segment)))
        self.initial_translation_tolerance = 0.5 * linear_translation
        self.yaw_tolerance = 0.5 * self.rotate_per_segment
        
        # Add 1 to the required segments to include the initial approach vector to the perimeter
        self.remaining_segments = self.rotations_segments

        # Fields for hold time, tune as needed
        self.hold_time_final_segment = 1.0
        self.hold_time_regular_segment = 0.2

    def setup(self, **kwargs) -> None:
        """Called once when the BT tree is setup."""
        self.blackboard.register_key(key="/navigation_client/client", access=py_trees.common.Access.READ)
        self.navigation_client = self.blackboard.navigation_client.client 
        self.navigation_client.client_wait_for_server(timeout_sec=5.0) # Ensure the action server is ready
        
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)

    def initialise(self) -> None:
        """Called every time this behavior transitions is not RUNNING."""
        self.remaining_segments = self.rotations_segments + 1
        self.action_status = ActionStatus.NOT_SENT
        self.start_angle = None

    def update(self) -> py_trees.common.Status:
        """Tick function. Calculates the absolute waypoint for the current segment and sends the goal."""
        
        # Block execution if the AUV poses are not published yet
        if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
            self.node.get_logger().info(f"[{self.name}] Waiting for sensor pose data...", throttle_duration_sec=2.0)
            return py_trees.common.Status.RUNNING
            
        # Check for failure condition from the async callbacks
        if self.action_status is ActionStatus.FAILED:
            self.node.get_logger().error(f"[{self.name}] Orbit failed midway.")
            return py_trees.common.Status.FAILURE
            
        # Completion check
        if self.remaining_segments <= 0 and self.action_status is ActionStatus.SUCCEEDED:
            self.node.get_logger().info(f"[{self.name}] Completed all segments.")
            return py_trees.common.Status.SUCCESS

        # Block loop if currently navigating to a waypoint
        if self.action_status is ActionStatus.PENDING:
            return py_trees.common.Status.RUNNING

        # Compute current segment index (0 to N)
        current_segment = (self.rotations_segments + 1) - self.remaining_segments

        # Target center to orbit. Placeholder values; can be read from Vision blackboard keys
        target_x = self.target[0]
        target_y = self.target[1]
        target_z = self.target[2]

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
        hold_time = self.hold_time_regular_segment
        
        if current_segment == 0:
            hold_time = self.hold_time_final_segment
        elif self.remaining_segments == 1:
            tolerance = 0.3 * self.initial_translation_tolerance
            hold_time = self.hold_time_final_segment

        goal_msg = move_global(
            x=waypoint_x, 
            y=waypoint_y, 
            do_z=False, 
            yaw=look_at_yaw, 
            tolerance=tolerance, 
            yaw_tolerance=self.yaw_tolerance, 
            hold_time=hold_time, 
            timeout=30.0
        )

        # Send the goal if no goals are ongoing and set the mission status to pending
        self.navigation_client.send_navigation_goal(goal_msg, self.name, self.on_server_goal_response, self.on_server_goal_result)
        self.action_status = ActionStatus.PENDING
        return py_trees.common.Status.RUNNING
        
    def on_server_goal_response(self, goal_response: bool) -> None:
        """
        Description: This function provides customized responses to the 
        action server's decision on accepting the goal or not. In this case,
        the custom implementation updates the status of the mission. Pass this
        function as an input to the navigation_client.send_navigation_goal

        Inputs: goal_response: str, The client will call this function with true upon acceptance, and false upon rejection from the server

        Outputs: None
        """
        if not goal_response:
            self.action_status = ActionStatus.FAILED

    def on_server_goal_result(self, goal_success) -> None:
        """
        Description: This function provides customized logic to be executed when
        the goal is finished. In this case, the custom implementation updates the status of the mission
        depending on whether or not the goal was successful or failed. Pass this function as an input 
        to the navigation_client.send_navigation_goal

        Inputs: goal_success: str, The client will call this function with true upon success, and false upon failure

        Outputs: None
        """
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
            self.remaining_segments -= 1
        else:
            self.action_status = ActionStatus.FAILED
    
    def terminate(self, new_status: py_trees.common.Status):
        """Called if the tree aborts this branch or if it naturally finishes."""
        if new_status == py_trees.common.Status.INVALID:
            if self.node:
                self.node.get_logger().warn(f"[{self.name}] Aborted branch. Canceling active goal.")
            self.navigation_client.reset_action_client()
