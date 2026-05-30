from dataclasses import dataclass
import py_trees
import math
import planner.missions.vision_behaviours as vision_behaviours
from controls.goal_helpers import move_global, move_robot_centric
from planner.missions.action_status_enum import ActionStatus
import geometry_msgs.msg._pose

DOWNCAM_FOV_HORIZONTAL = 59.7
DOWNCAM_FOV_VERTICAL = 46.3
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
BIN_DOWNCAM_MOVING_AVERAGE_WEIGHT = 1.0
GO_ABOVE_BIN_HEIGHT = 0.7

@dataclass
class BinInfo:
    pose: geometry_msgs.msg._pose
    visited: bool = False

class FollowDowncamBin(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("FollowDowncamBin")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.action_status = ActionStatus.PENDING

        self.down_cam_bin_position = None
        self.down_cam_new_goal_timer = 0
        # When you send a new goal, the old one fails, account for this
        self.expected_failures = 0
    
    def setup(self, **kwargs):
        self.navigation_client = kwargs['shared_nav_client']
        self.node = kwargs['node']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)

        self.blackboard.register_key('/vision/down_cam/detections', access=py_trees.common.Access.READ)

        self.node.get_logger().info("Starting FollowDowncamBin behavior")
        
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
        # Check for failure condition from the async callbacks (goal response and goal result)
        if self.action_status is ActionStatus.FAILED:
            self.node.get_logger().error(f"[{self.name}] Action failed midway.")
            return py_trees.common.Status.FAILURE
            
        # Completion check
        if self.action_status is ActionStatus.SUCCEEDED:
            # If succeeded, it means we got to the downcam goal before it was updated, meaning we're very close so this behavior is complete
            # return py_trees.common.Status.SUCCESS
            pass
        
        # If not sent, send it
        if self.action_status is ActionStatus.NOT_SENT:
            self.navigation_client.send_navigation_goal(self.goal, self.name, self.on_server_goal_response, self.on_server_goal_result)
            self.action_status = ActionStatus.PENDING
            return py_trees.common.Status.RUNNING
        
        if hasattr(self.blackboard, 'vision') and self.blackboard.vision.down_cam.detections is not None:
            downcam_bin_positions = []

            for detection in self.blackboard.vision.down_cam.detections.detections:
                hypothesis = detection.results[0].hypothesis
                if hypothesis.class_id == "bin" or hypothesis.class_id == "blood" or hypothesis.class_id == "fire":
                    
                    coordinates = (detection.bbox.center.position.x, detection.bbox.center.position.y)
                    downcam_bin_positions.append(coordinates)
                    # Coordinates are in pixel coordinates, add half camera dimensions to get coordiantes from center
                    # Y is also inverted (+y is down in pixel coordinates)

            if len(downcam_bin_positions) == 0:
                return py_trees.common.Status.RUNNING

            # find bin closest to camera center
            closest_downcam_bin = None
            closest_bin_distance = float('inf')
            for downcam_bin in downcam_bin_positions:
                distance_squared = float((downcam_bin[0] - CAMERA_WIDTH / 2) ** 2 + (downcam_bin[1] - CAMERA_HEIGHT / 2) ** 2)
                if distance_squared < closest_bin_distance:
                    closest_downcam_bin = (downcam_bin[0] - CAMERA_WIDTH / 2, downcam_bin[1] - CAMERA_HEIGHT / 2)
                    closest_bin_distance = distance_squared

            self.node.get_logger().info(f"Bin position: {closest_downcam_bin}")

            current_bin_position = closest_downcam_bin
            # Moving average
            PREV_WEIGHT = 1 - BIN_DOWNCAM_MOVING_AVERAGE_WEIGHT
            self.down_cam_bin_position = (
                (self.down_cam_bin_position[0] * PREV_WEIGHT + current_bin_position[0] * BIN_DOWNCAM_MOVING_AVERAGE_WEIGHT),
                (self.down_cam_bin_position[1] * PREV_WEIGHT + current_bin_position[1] * BIN_DOWNCAM_MOVING_AVERAGE_WEIGHT)
            ) if self.down_cam_bin_position is not None else current_bin_position
                        
            # Get the angle based on fov
            x_angle = math.radians(self.down_cam_bin_position[0] / CAMERA_WIDTH * DOWNCAM_FOV_HORIZONTAL)
            y_angle = math.radians(self.down_cam_bin_position[1] / CAMERA_HEIGHT * DOWNCAM_FOV_VERTICAL)

            #self.node.get_logger().info(f"Sending new goal: x_angle: {x_angle}, y_angle: {y_angle}")

            self.expected_failures += 1  # current goal will fail once, ignore that failure

            # We know the bin is 1.0m below us, so calculate the bin position
            self.goal = move_robot_centric(forward=-math.tan(y_angle) * (GO_ABOVE_BIN_HEIGHT - 0.1), sway=-math.tan(x_angle) * (GO_ABOVE_BIN_HEIGHT - 0.1))
            # self.goal = move_robot_centric(forward=-self.down_cam_bin_position[1] / CAMERA_HEIGHT, sway=-self.down_cam_bin_position[0] / CAMERA_WIDTH)
            self.action_status = ActionStatus.NOT_SENT  # Next tick, new goal will be sent automatically

        return py_trees.common.Status.RUNNING

class AlignClosestBin(py_trees.composites.Sequence):
    def __init__(self):
        super().__init__("AlignClosestBin", memory=True)
        #go_above_closest_bin = vision_behaviours.GoNearObject(target_class="bin", target_distance=0.0, height_offset=GO_ABOVE_BIN_HEIGHT)
        go_above_closest_bin = GoAboveClosestBin()
        follow_downcam_bin = FollowDowncamBin()

        self.add_children([go_above_closest_bin, follow_downcam_bin])

class GoAboveClosestBin(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("GoAboveClosestBin")
        self.blackboard = self.attach_blackboard_client(name=self.name)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)
        self.blackboard.register_key(key="/bins_task/closest_bins", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        if not hasattr(self.blackboard.bins_task, 'closest_bins') or self.blackboard.bins_task.closest_bins is None:
            return py_trees.common.Status.FAILURE
        
        bins = self.blackboard.bins_task.closest_bins
        bin_position = bins[0].pose if bins[0].visited else bins[1].pose

        goal = move_global(bin_position.x, bin_position.y, bin_position.z + GO_ABOVE_BIN_HEIGHT)
        self.navigation_client.send_navigation_goal(goal, self.name)
        return py_trees.common.Status.SUCCESS
    
class GoNearBinStructure(py_trees.composites.Sequence): 
    def __init__(self):
        super().__init__("GoNearBinStructure", memory=True)

        go_4m_away_node = vision_behaviours.GoNearObject(
            target_class="bin_structure",
            target_distance=4.0)

        look_at_bin_structure = vision_behaviours.SearchSweepBehaviour(target_class="bin_structure")
        
        go_2m_away_node = vision_behaviours.GoNearObject(
            target_class="bin_structure",
            target_distance=2.0,
            height_offset=0.5
        )

        self.add_children([
            go_4m_away_node,
            look_at_bin_structure,
            go_2m_away_node,
        ])

# TODO: don't fail if it only finds one bin
class GetClosestBins(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("GoAboveClosestBin")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.success = False

    def setup(self, **kwargs):
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/bins_task/closest_bins", access=py_trees.common.Access.WRITE)

    def initialise(self):
        if not hasattr(self.blackboard, 'vision') or self.blackboard.vision.object_map is None:
            self.success = False
            return py_trees.common.Status.FAILURE
        
        bin_1_distance = float('inf')
        bin_1 = None
        bin_2_distance = float('inf')
        bin_2 = None

        auv_pos = self.blackboard.sensors.pose.pose.position

        for object in self.blackboard.vision.object_map.array:
            if object.label == "bin":
                bin_pos = object.pose.position
                distance: float = (bin_pos.x - auv_pos.x) ** 2 + (bin_pos.y - auv_pos.y) ** 2
                if distance < bin_1_distance:
                    bin_2 = bin_1
                    bin_2_distance = bin_1_distance
                    bin_1 = object
                    bin_1_distance = distance
                
                elif distance < bin_2_distance:
                    bin_2 = object
                    bin_2_distance = distance
        
        if bin_1 is not None and bin_2 is not None:
            self.blackboard.bins_task.closest_bins = [BinInfo(pose=bin_1.pose.position), BinInfo(pose=bin_2.pose.position)]
            self.success = True

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.SUCCESS if self.success else py_trees.common.Status.FAILURE
    
