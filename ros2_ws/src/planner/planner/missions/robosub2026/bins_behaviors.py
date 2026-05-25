import py_trees
import math
import planner.missions.vision_behaviours as vision_behaviours
from controls.goal_helpers import move_global, set_depth, move_robot_centric
from planner.missions.action_status_enum import ActionStatus

DOWNCAM_FOV_HORIZONTAL = 69
DOWNCAM_FOV_VERTICAL = 50
CAMERA_WIDTH = 672
CAMERA_HEIGHT = 376
BIN_DOWNCAM_MOVING_AVERAGE_WEIGHT = 0.2

class GoAboveClosestBin(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("GoAboveClosestBin")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.action_status = ActionStatus.NOT_SENT

        self.consecutive_bin_downcam_detections = 0
        self.following_downcam_bin = False
        self.down_cam_bin_position = None
        self.down_cam_new_goal_timer = 0
        # When you send a new goal, the old one fails, account for this
        self.expected_failures = 0
    
    def setup(self, **kwargs):
        self.navigation_client = kwargs['shared_nav_client']
        self.node = kwargs['node']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)

        self.blackboard.register_key('/vision/down_cam/detections', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/sensors/pose', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/vision/object_map', access=py_trees.common.Access.READ)

        self.node.get_logger().info("Starting GoAboveClosestBin behavior")
        
    def initialise(self):
        if hasattr(self.blackboard, 'vision') and self.blackboard.vision.object_map is not None \
            and hasattr(self.blackboard, 'sensors') and self.blackboard.sensors.pose is not None:
            self.closest_bin = None

            min_bin_distance = float('inf')
            for obj in self.blackboard.vision.object_map.array:
                if obj.label == "bin":
                    # Calculate distance to the bin
                    auv_position = self.blackboard.sensors.pose.pose.position
                    bin_position = obj.pose.position
                    distance = math.sqrt((auv_position.x - bin_position.x) ** 2 +
                                            (auv_position.y - bin_position.y) ** 2 +
                                            (auv_position.z - bin_position.z) ** 2)
                    if distance < min_bin_distance:
                        min_bin_distance = distance
                        self.closest_bin = obj
            
            if self.closest_bin is None:
                return py_trees.common.Status.FAILURE
            
            target_position = self.closest_bin.pose.position

            self.goal = move_global(target_position.x, target_position.y, target_position.z + 0.8)
        else:
            return py_trees.common.Status.FAILURE
        
    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.node.get_logger().error(f"[{self.name}] Goal rejected by server.")
            self.action_status = ActionStatus.FAILED

    def on_server_goal_result(self, goal_success: bool) -> None:
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        elif self.expected_failures > 0:
            self.expected_failures -= 1
        else:
            self.action_status = ActionStatus.FAILED

    def update(self) -> py_trees.common.Status:
        # Check if pose is properly registered on the blackboard as controls requires
        # a pose value. Block execution if the AUV poses are not published yet
        if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
            self.node.get_logger().info(f"[{self.name}] Waiting for sensor pose data...", throttle_duration_sec=2.0)
            return py_trees.common.Status.RUNNING
            
        # Check for failure condition from the async callbacks (goal response and goal result)
        if self.action_status is ActionStatus.FAILED:
            self.node.get_logger().error(f"[{self.name}] Action failed midway.")
            return py_trees.common.Status.FAILURE
            
        # Completion check
        if self.action_status is ActionStatus.SUCCEEDED:
            # If succeeded, it means we got to the downcam goal before it was updated, meaning we're very close so this behavior is complete
            if self.following_downcam_bin:
                self.node.get_logger().info(f"[{self.name}] Completed goal.")
                return py_trees.common.Status.SUCCESS
            # If we succeeded but we're not following the downcam bin, it means we got to the original goal but now we should start following the downcam detections since we're close enough
            self.node.get_logger().info(f"[{self.name}] Reached initial goal, now following downcam detections.")
            self.following_downcam_bin = True
            # Don't return so the rest of the update loop handles downcam following
        
        # If not sent, send it
        if self.action_status is ActionStatus.NOT_SENT:
            self.navigation_client.send_navigation_goal(self.goal, self.name, self.on_server_goal_response, self.on_server_goal_result)
            self.action_status = ActionStatus.PENDING
            return py_trees.common.Status.RUNNING
        
        # Currently navigating to the goal
        # Check down cam
        if self.following_downcam_bin and hasattr(self.blackboard, 'vision') and self.blackboard.vision.down_cam.detections is not None:
            detected_bin = False

            for detection in self.blackboard.vision.down_cam.detections.detections:
                hypothesis = detection.results[0].hypothesis
                if hypothesis.class_id == "bin":
                    self.consecutive_bin_downcam_detections += 1
                    detected_bin = True
                    
                    coordinates = (detection.bbox.center.position.x, detection.bbox.center.position.y)
                    # Coordinates are in pixel coordinates, add half camera dimensions to get coordiantes from center
                    # Y is also inverted (+y is down in pixel coordinates)
                    current_bin_position = (CAMERA_WIDTH / 2 + coordinates[0], CAMERA_HEIGHT / 2 - coordinates[1])
                    # Moving average
                    PREV_WEIGHT = 1 - BIN_DOWNCAM_MOVING_AVERAGE_WEIGHT
                    self.down_cam_bin_position = (
                        (self.down_cam_bin_position[0] * PREV_WEIGHT + current_bin_position[0] * BIN_DOWNCAM_MOVING_AVERAGE_WEIGHT),
                        (self.down_cam_bin_position[1] * PREV_WEIGHT + current_bin_position[1] * BIN_DOWNCAM_MOVING_AVERAGE_WEIGHT)
                    ) if self.down_cam_bin_position is not None else current_bin_position
                    break
            
            self.node.get_logger().info(f"Detected bin on downcam at {self.down_cam_bin_position} with consecutive detections: {self.consecutive_bin_downcam_detections}")
            
            if not detected_bin:
                self.consecutive_bin_downcam_detections = 0
                # No bin detected, so just skip this tick
                return py_trees.common.Status.RUNNING
        
            if self.down_cam_new_goal_timer > 0:
                self.down_cam_new_goal_timer -= 1
                return py_trees.common.Status.RUNNING
            
            # Send new goal every 6 ticks (2 seconds) to avoid spamming controls and messing with the PIDs
            self.down_cam_new_goal_timer = 6
            # Get the angle based on fov and stuff
            x_angle = math.radians(self.down_cam_bin_position[1] / CAMERA_WIDTH * DOWNCAM_FOV_HORIZONTAL)
            y_angle = math.radians(-self.down_cam_bin_position[0] / CAMERA_HEIGHT * DOWNCAM_FOV_VERTICAL)

            self.node.get_logger().info(f"Sending new goal: x_angle: {x_angle}, y_angle: {y_angle}")

            self.expected_failures += 1  # current goal will fail once, ignore that failure

            # We know the bin is 0.5m below us, so calculate the bin position
            self.goal = move_robot_centric(forward=math.cos(y_angle) * 0.5, sway=math.sin(x_angle) * 0.5)
            self.action_status = ActionStatus.NOT_SENT  # Next tick, new goal will be sent automatically

        return py_trees.common.Status.RUNNING

class GoNearBinStructure(py_trees.composites.Sequence):
    def __init__(self):
        super().__init__("GoNearBinStructure", memory=True)
        self.blackboard = self.attach_blackboard_client(name=self.name)

        go_4m_away_node = vision_behaviours.GoDistanceFromObject(
            target_class="bin_structure",
            target_distance=4.0)
        
        go_to_height_node = GoToObjectHeight(target_class="bin_structure")
        
        go_2m_away_node = vision_behaviours.GoDistanceFromObject(
            target_class="bin_structure",
            target_distance=2.0
        )

        self.add_children([
            go_4m_away_node,
            go_to_height_node,
            go_2m_away_node,
        ])

class GoToObjectHeight(py_trees.behaviour.Behaviour):
    def __init__(self, target_class: str):
        super().__init__("GoToObjectHeight")
        self.target_class = target_class
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key('/vision/object_map', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/sensors/pose', access=py_trees.common.Access.READ)

        self.action_status = ActionStatus.NOT_SENT

    def setup(self, **kwargs):
        self.navigation_client = kwargs['shared_nav_client']
        self.node = kwargs['node']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)

        self.node.get_logger().info(f"Starting GoToObjectHeight behavior for target class {self.target_class}")
        
    def initialise(self):
        if hasattr(self.blackboard, 'vision') and self.blackboard.vision.object_map is not None:
            target_object = None
            for obj in self.blackboard.vision.object_map.array:
                if obj.label == self.target_class:
                    target_object = obj
                    break
            
            if target_object is None:
                return py_trees.common.Status.FAILURE
            
            target_height = target_object.pose.position.z
            self.goal = set_depth(target_height + 0.6)

        else:
            return py_trees.common.Status.FAILURE
        
    def update(self) -> py_trees.common.Status:
        # Check if pose is properly registered on the blackboard as controls requires
        # a pose value. Block execution if the AUV poses are not published yet
        if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
            self.node.get_logger().info(f"[{self.name}] Waiting for sensor pose data...", throttle_duration_sec=2.0)
            return py_trees.common.Status.RUNNING
            
        # Check for failure condition from the async callbacks (goal response and goal result)
        if self.action_status is ActionStatus.FAILED:
            self.node.get_logger().error(f"[{self.name}] Action failed midway.")
            return py_trees.common.Status.FAILURE
            
        # Completion check
        if self.action_status is ActionStatus.SUCCEEDED:
            self.node.get_logger().info(f"[{self.name}] Completed goal.")
            return py_trees.common.Status.SUCCESS

        # Block loop if currently navigating to a waypoint
        if self.action_status is ActionStatus.PENDING:
            return py_trees.common.Status.RUNNING
            
        # Send the goal if no goals are ongoing and set the mission status to pending
        self.navigation_client.send_navigation_goal(self.goal, self.name, self.on_server_goal_response, self.on_server_goal_result)
        self.action_status = ActionStatus.PENDING
        return py_trees.common.Status.RUNNING
        
    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.action_status = ActionStatus.FAILED

    def on_server_goal_result(self, goal_success: bool) -> None:
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.action_status = ActionStatus.FAILED
