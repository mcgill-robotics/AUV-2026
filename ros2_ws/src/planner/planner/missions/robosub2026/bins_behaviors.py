from dataclasses import dataclass
from planner.missions.mission_behaviour_components import BasicActionBehaviour
import py_trees
import math
import planner.missions.vision_behaviours as vision_behaviours
from controls.goal_helpers import move_global, move_robot_centric
from planner.missions.action_status_enum import ActionStatus
import geometry_msgs.msg._pose
import transforms3d

class ApproachObject(py_trees.composites.Sequence): 
    def __init__(self, target_class: str, target_distance: float, height_offset: float):
        super().__init__("Approach" + target_class.capitalize(), memory=True)

        go_4m_away_node = vision_behaviours.GoNearObject(
            target_class=target_class,
            target_distance=4.0, 
            tolerance_meters=0.5,
            hold_time=0.0)

        look_at_bin_structure = vision_behaviours.SearchSweepBehaviour(target_class=target_class)
        
        go_2m_away_node = vision_behaviours.GoNearObject(
            target_class=target_class,
            target_distance=target_distance,
            height_offset=height_offset,
            tolerance_meters=0.5,
            hold_time=0.0
        )

        self.add_children([
            go_4m_away_node,
            look_at_bin_structure,
            go_2m_away_node,
        ])

class FindBinStructure(py_trees.composites.Selector):
    def __init__(self, target_distance: float, height_offset: float):
        super().__init__("FindBinStructure", memory=True)

        try_bin_structure = ApproachObject(target_class="bin_structure", target_distance=target_distance, height_offset=height_offset)
        try_bins = ApproachObject(target_class="bin", target_distance=target_distance, height_offset=height_offset)

        self.add_children([
            try_bin_structure,
            try_bins,
        ])

@dataclass
class BinInfo:
    pose: geometry_msgs.msg._pose
    visited: bool = False

# TODO: don't fail if it only finds one bin
class GetClosestBins(py_trees.behaviour.Behaviour):
    def __init__(self, num_bins: int = 2):
        super().__init__("GetClosestBins")
        self.num_bins = num_bins
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
        if self.num_bins >= 2:
            if bin_1 is not None and bin_2 is not None:
                self.blackboard.bins_task.closest_bins = [BinInfo(pose=bin_1.pose.position), BinInfo(pose=bin_2.pose.position)]
                self.success = True
        elif self.num_bins == 1:
            if bin_1 is not None:
                self.blackboard.bins_task.closest_bins = [BinInfo(pose=bin_1.pose.position)]
                self.success = True

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.SUCCESS if self.success else py_trees.common.Status.FAILURE
    
class AlignClosestBin(py_trees.composites.Sequence):
    def __init__(self, bins_params: dict = None):
        super().__init__("AlignClosestBin", memory=True)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.bins_params = bins_params or {}
        # initialize config-backed fields from yaml (fail if missing)
        self.downcam_fov_horizontal = self.bins_params['downcam_fov_horizontal']
        self.downcam_fov_vertical = self.bins_params['downcam_fov_vertical']
        self.camera_width = self.bins_params['downcam_image_width']
        self.camera_height = self.bins_params['downcam_image_height']
        self.bin_moving_average_weight = self.bins_params['bin_moving_average_weight']
        self.go_above_bin_height = self.bins_params['go_above_bin_height']
        self.wrong_task_type_threshold = self.bins_params['wrong_task_type_threshold']
        self.bin_lined_up_threshold = self.bins_params['bin_lined_up_threshold']
        go_above_closest_bin = GoAboveClosestBin(self.bins_params)
        follow_downcam_bin = FollowDowncamBin(self.bins_params)
        drop_marker = DropMarker(self.bins_params)

        self.add_children([go_above_closest_bin, follow_downcam_bin, drop_marker])

class AlignBinsAttempt(py_trees.composites.Sequence):
    def __init__(self, bins_params: dict = None):
        super().__init__("AlignBinsAttempt", memory=True)
        self.bins_params = bins_params or {}
        # just run it twice, works with visited logic
        get_bins = GetClosestBins(self.bins_params['num_bins'])
        if self.bins_params['num_bins'] >= 2:
            align_bins = TryAlignBothBins(self.bins_params)
        else:
            align_bins = AlignClosestBin(self.bins_params)

        self.add_children([get_bins, align_bins])

class TryAlignBothBins(py_trees.composites.Selector):
    def __init__(self, bins_params: dict = None):
        super().__init__("TryAlignBothBins", memory=True)
        self.bins_params = bins_params or {}
        try_align_first_bin = AlignClosestBin(self.bins_params)
        try_align_second_bin = AlignClosestBin(self.bins_params)

        self.add_children([try_align_first_bin, try_align_second_bin])

class AlignOtherSideBin(py_trees.composites.Sequence):
    def __init__(self, bins_params: dict = None):
        super().__init__("AlignOtherSideBin", memory=True)
        self.bins_params = bins_params or {}
        go_to_other_side = GoToOtherSide(self.bins_params)
        bins_params_other_side = bins_params.copy()
        bins_params_other_side['num_bins'] -= 2
        align_bins_attempt = AlignBinsAttempt(bins_params_other_side)

        self.add_children([go_to_other_side, align_bins_attempt])
    
class AlignCorrectBin(py_trees.composites.Selector):
    def __init__(self, bins_params: dict = None):
        super().__init__("AlignCorrectBin", memory=True)
        self.bins_params = bins_params or {}
        align_bin_first_side = AlignBinsAttempt(self.bins_params)
        if self.bins_params['num_bins'] > 2:
            align_other_bin = AlignOtherSideBin(self.bins_params)
        else:
            align_other_bin = py_trees.behaviours.Failure(name="SkipOtherBinAlignment")

        self.blackboard = self.attach_blackboard_client(name=self.name)

        self.add_children([align_bin_first_side, align_other_bin])

    def setup(self, **kwargs):
        self.blackboard.register_key('/bins_task/number_markers', access=py_trees.common.Access.WRITE)
    
    def initialise(self):
        self.blackboard.bins_task.number_markers = 0

class SwitchSides(py_trees.behaviour.Behaviour):
    def __init__(self, bins_params: dict = None):
        super().__init__("SwitchSides")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.action_status = ActionStatus.NOT_SENT
        self.bins_params = bins_params or {}
        # cache parameter values from yaml (fail if missing)
        self.bin_structure_distance = self.bins_params['bin_structure_distance']
        self.bins_to_bin_structure_distance = self.bins_params['bins_to_bin_structure']
        self.go_above_bin_structure_height = self.bins_params['go_above_bin_structure_height']
        self.switch_sides_safety_height = self.bins_params['switch_sides_height']
        self.use_fallback = self.bins_params['force_fallback_alignment']

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)
        self.blackboard.register_key(key="/bins_task/closest_bins", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/bins_task/structure_pos", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)

    def initialise(self):
        if not hasattr(self.blackboard.bins_task, 'closest_bins') or self.blackboard.bins_task.closest_bins is None:
            self.action_status = ActionStatus.FAILED
            self.node.get_logger().error("No bins found on blackboard for SwitchSides behavior.")
        
        if not hasattr(self.blackboard, 'bins_task') or self.blackboard.bins_task.structure_pos is None:
            self.action_status = ActionStatus.FAILED
            self.node.get_logger().error("No bin structure position found on blackboard for SwitchSides behavior.")
        
        structure_pos = self.blackboard.bins_task.structure_pos

        bins = self.blackboard.bins_task.closest_bins
        bins_midpoint = (bins[0].pose.x + bins[1].pose.x) / 2, (bins[0].pose.y + bins[1].pose.y) / 2

        structure_to_bins_vector = bins_midpoint[0] - structure_pos.x, bins_midpoint[1] - structure_pos.y
        structure_to_bins_mag = math.sqrt(structure_to_bins_vector[0] ** 2 + structure_to_bins_vector[1] ** 2)
        structure_to_bins_unit_vector = structure_to_bins_vector[0] / structure_to_bins_mag, structure_to_bins_vector[1] / structure_to_bins_mag
        goal_position = structure_pos.x - structure_to_bins_unit_vector[0] * self.bin_structure_distance, structure_pos.y - structure_to_bins_unit_vector[1] * self.bin_structure_distance
        
        yaw_goal = math.atan2(structure_pos.y - goal_position[1], structure_pos.x - goal_position[0])
        
        goal = move_global(goal_position[0], goal_position[1], structure_pos.z + self.go_above_bin_structure_height, yaw=yaw_goal, hold_time=0.0)

        self.navigation_client.send_navigation_goal(goal, self.name, custom_goal_response=self.on_server_goal_response, custom_goal_result=self.on_server_goal_result)
        self.action_status = ActionStatus.PENDING

    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.node.get_logger().error(f"[{self.name}] Goal rejected by server.")
            self.action_status = ActionStatus.FAILED

    def on_server_goal_result(self, goal_success: bool, message: str) -> None:
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to switch sides. {message}")
            self.action_status = ActionStatus.FAILED

    def update(self) -> py_trees.common.Status:
        if self.action_status == ActionStatus.FAILED:
            return py_trees.common.Status.FAILURE
        
        if self.action_status == ActionStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING
    
class GetBinStructurePos(py_trees.behaviour.Behaviour):
    def __init__(self, bins_params: dict = None):
        super().__init__("GetBinStructurePos")
        self.bins_params = bins_params
        self.bins_to_bin_structure_distance = bins_params['bins_to_bin_structure']
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.failed = False

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/bins_task/structure_pos", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/bins_task/closest_bins", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)

    def initialise(self):
        if not hasattr(self.blackboard.vision, 'object_map') or self.blackboard.vision.object_map is None:
            self.failed = True
            return
        
        structure_pos = None
        if not self.bins_params["force_fallback_alignment"] and hasattr(self.blackboard.vision, 'object_map') and self.blackboard.vision.object_map is not None:
            for object in self.blackboard.vision.object_map.array:
                if object.label == "bin_structure":
                    structure_pos = object.pose.position
                    break

        # Either didn't find bin structure, or force fallback is enabled
        if structure_pos is None:
            self.node.get_logger().warn("Using bins for fallback alignment.")
            if not hasattr(self.blackboard, "bins_task") or self.blackboard.bins_task.closest_bins is None:
                self.failed = True

            bins = self.blackboard.bins_task.closest_bins
            bins_avg_height = (bins[0].pose.z + bins[1].pose.z) / 2
            bins_midpoint = (bins[0].pose.x + bins[1].pose.x) / 2, (bins[0].pose.y + bins[1].pose.y) / 2

            auv_orientation = self.blackboard.sensors.pose.pose.orientation
            q = [auv_orientation.w, auv_orientation.x, auv_orientation.y, auv_orientation.z]
            _, _, yaw = transforms3d.euler.quat2euler(q)
            auv_forward_vector = (math.cos(yaw), math.sin(yaw))

            bins_vector = (bins[1].pose.x - bins[0].pose.x, bins[1].pose.y - bins[0].pose.y)
            bins_perpendicular_vector = (-bins_vector[1], bins_vector[0])
            magnitude = math.sqrt(bins_perpendicular_vector[0] ** 2 + bins_perpendicular_vector[1] ** 2)
            if magnitude == 0:
                magnitude = 1.0 # prevent division by zero
            bins_perpendicular_unit_vector = bins_perpendicular_vector[0] / magnitude, bins_perpendicular_vector[1] / magnitude

            dot_product = auv_forward_vector[0] * bins_perpendicular_unit_vector[0] + auv_forward_vector[1] * bins_perpendicular_unit_vector[1]
            if dot_product < 0:
                bins_perpendicular_unit_vector = (-bins_perpendicular_unit_vector[0], -bins_perpendicular_unit_vector[1])
            
            structure_pos = geometry_msgs.msg.Point(x=bins_midpoint[0] + bins_perpendicular_unit_vector[0] * self.bins_to_bin_structure_distance, y=bins_midpoint[1] + bins_perpendicular_unit_vector[1] * self.bins_to_bin_structure_distance, z=bins_avg_height)

        if structure_pos is not None:
            self.blackboard.bins_task.structure_pos = structure_pos
        else:
            self.failed = True

    def update(self) -> py_trees.common.Status:
        if self.failed:
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS

class GoToBlackboardBinStructure(py_trees.behaviour.Behaviour):
    def __init__(self, height_offset: float, bins_params: dict = None):
        super().__init__("GoToBlackboardBinStructure")
        self.bins_params = bins_params
        self.height_offset = height_offset
        self.blackboard = self.attach_blackboard_client(self.name)
        self.action_status = ActionStatus.NOT_SENT

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.blackboard.register_key("/bins_task/structure_pos", py_trees.common.Access.READ)
        self.blackboard.register_key("/bins_task/closest_bins", py_trees.common.Access.READ)

    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.node.get_logger().error(f"[{self.name}] Goal rejected by server.")
            self.action_status = ActionStatus.FAILED

    def on_server_goal_result(self, goal_success: bool, message: str) -> None:
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.node.get_logger().error(f"[{self.name}] Goal failed during execution. {message}")
            self.action_status = ActionStatus.FAILED

    def initialise(self):
        if not hasattr(self.blackboard.bins_task, 'structure_pos') or self.blackboard.bins_task.structure_pos is None:
            return py_trees.common.Status.FAILURE
        
        structure_pos = self.blackboard.bins_task.structure_pos

        bins = self.blackboard.bins_task.closest_bins

        bins_midpoint = (bins[0].pose.x + bins[1].pose.x) / 2, (bins[0].pose.y + bins[1].pose.y) / 2

        yaw = math.atan2(structure_pos.y - bins_midpoint[1], structure_pos.x - bins_midpoint[0])
        
        self.goal = move_global(structure_pos.x, structure_pos.y, structure_pos.z + self.height_offset, yaw=yaw, hold_time=0.0)

    def update(self):
        if self.action_status is ActionStatus.NOT_SENT:
            self.action_status = ActionStatus.PENDING
            self.navigation_client.send_navigation_goal(self.goal, self.name, self.on_server_goal_response, self.on_server_goal_result)
        
        if self.action_status is ActionStatus.FAILED:
            return py_trees.common.Status.FAILURE
        if self.action_status is ActionStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING

class GoToOtherSide(py_trees.composites.Sequence):
    def __init__(self, bins_params: dict = None):
        super().__init__("GoToOtherSide", memory=True)
        self.bins_params = bins_params or {}
        switch_height = self.bins_params['switch_sides_height']
        get_bin_structure_pos = GetBinStructurePos(bins_params)
        go_up = GoToBlackboardBinStructure(height_offset=switch_height, bins_params=bins_params)
        go_forward = BasicActionBehaviour(name="GoForwardForSwitchSides", goal=move_robot_centric(forward=2.0, hold_time=0.0))
        switch_sides = SwitchSides(self.bins_params)

        self.add_children([get_bin_structure_pos, go_up, go_forward, switch_sides])

class FollowDowncamBin(py_trees.behaviour.Behaviour):
    def __init__(self, bins_params: dict = None):
        super().__init__("FollowDowncamBin")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.action_status = ActionStatus.PENDING
        self.bins_params = bins_params or {}
        # initialize config-backed fields from yaml (fail if missing)
        self.downcam_fov_horizontal = self.bins_params['downcam_fov_horizontal']
        self.downcam_fov_vertical = self.bins_params['downcam_fov_vertical']
        self.camera_width = self.bins_params['downcam_image_width']
        self.camera_height = self.bins_params['downcam_image_height']
        self.bin_moving_average_weight = self.bins_params['bin_moving_average_weight']
        self.go_above_bin_height = self.bins_params['go_above_bin_height']
        self.wrong_task_type_threshold = self.bins_params['wrong_task_type_threshold']
        self.bin_lined_up_threshold = self.bins_params['bin_lined_up_threshold']
        self.no_detection_timeout = self.bins_params.get('no_detection_timeout', 10.0)

        self.down_cam_bin_position = None
        self.down_cam_new_goal_timer = 0
        # When you send a new goal, the old one fails, account for this
        self.expected_failures = 0
        self.fire_detections = 0
        self.blood_detections = 0
        
        self.bin_lined_up_frames = 0
    
    def setup(self, **kwargs):
        self.navigation_client = kwargs['shared_nav_client']
        self.node = kwargs['node']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)

        self.blackboard.register_key('/vision/down_cam/detections', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/gate/selected_role', access=py_trees.common.Access.READ)

    def initialise(self):
        # if hasattr(self.blackboard, 'gate') and self.blackboard.gate.selected_role is not None:
        #     self.role = self.blackboard.gate.selected_role
        # else:
        #     self.role = "search_rescue" # default to search and rescue if for some reason gate role isn't available, better to try to align with any bin than fail outright
        try:
            self.role = self.blackboard.gate.selected_role
        except Exception as e:
            self.node.get_logger().error("Accessing /gate/selected_role failed, using search_rescue")
            self.role = "search_rescue"
        self.down_cam_bin_position = None
        self.expected_failures = 0
        self.fire_detections = 0
        self.blood_detections = 0
        self.bin_lined_up_frames = 0
        self.action_status = ActionStatus.PENDING
        self.last_detection_time = self.node.get_clock().now()

    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.node.get_logger().error(f"[{self.name}] Goal rejected by server.")
            self.action_status = ActionStatus.FAILED

    def on_server_goal_result(self, goal_success: bool, message: str) -> None:
        self.node.get_logger().info(f"[{self.name}] Goal result received. Success: {goal_success}, Expected failures: {self.expected_failures}, Message: {message}")
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        elif self.expected_failures > 0:
            self.expected_failures -= 1
        else:
            self.node.get_logger().error(f"[{self.name}] Goal failed during execution. {message}")
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
            downcam_bins = []

            for detection in self.blackboard.vision.down_cam.detections.detections:
                if detection.label == "blood" or detection.label == "fire": # or detection.label == "bin": # for testing, accept any bin detection as valid
                    
                    #coordinates = (detection.bbox.center.position.x, detection.bbox.center.position.y)
                    downcam_bins.append(detection)
                    # if hypothesis.class_id == "fire":
                    #     self.fire_detections += 1
                    # elif hypothesis.class_id == "blood":
                    #     self.blood_detections += 1

            if len(downcam_bins) == 0:
                time_since_last_detection = (self.node.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
                if time_since_last_detection > self.no_detection_timeout:
                    self.node.get_logger().error(f"[{self.name}] No valid bin detected for {self.no_detection_timeout}s. Timing out.")
                    return py_trees.common.Status.FAILURE
                self.node.get_logger().info(f"[{self.name}] No 'blood' or 'fire' detected. Waiting...", throttle_duration_sec=2.0)
                return py_trees.common.Status.RUNNING
            
            self.last_detection_time = self.node.get_clock().now()
            
            # check if its the wrong label
            if (self.role == "survey_repair" and self.blood_detections > self.fire_detections + self.wrong_task_type_threshold
                or self.role == "search_rescue" and self.fire_detections > self.blood_detections + self.wrong_task_type_threshold):
                self.node.get_logger().info("Detected wrong task type, going to other bin.")
                return py_trees.common.Status.FAILURE
        
            elif self.bin_lined_up_frames >= self.bin_lined_up_threshold:
                self.node.get_logger().info("Bin has been lined up for multiple frames, assuming aligned and succeeding.")
                return py_trees.common.Status.SUCCESS

            # find bin closest to camera center
            closest_downcam_bin = None
            closest_bin_detection = None
            closest_bin_distance = float('inf')
            for downcam_bin_detection in downcam_bins:
                downcam_bin_position = (downcam_bin_detection.bbox_center_x, downcam_bin_detection.bbox_center_y)
                distance_squared = float((downcam_bin_position[0] - self.camera_width / 2) ** 2 + (downcam_bin_position[1] - self.camera_height / 2) ** 2)
                if distance_squared < closest_bin_distance:
                    closest_bin_detection = downcam_bin_detection
                    closest_downcam_bin = (downcam_bin_position[0] - self.camera_width / 2, downcam_bin_position[1] - self.camera_height / 2)
                    closest_bin_distance = distance_squared

            if closest_bin_detection is not None:
                if closest_bin_detection.label == "fire":
                    self.fire_detections += 1
                elif closest_bin_detection.label == "blood":
                    self.blood_detections += 1

            if closest_bin_distance < self.bin_lined_up_threshold**2: # because closest bin distance is squared as well
                self.bin_lined_up_frames += 1
            else:
                self.bin_lined_up_frames = 0
                
            self.node.get_logger().info(f"[{self.name}] Pixel Dist^2: {closest_bin_distance:.1f} (Threshold: {self.bin_lined_up_threshold**2}). Lined up frames: {self.bin_lined_up_frames}/{self.bin_lined_up_threshold}")


            
            current_bin_position = closest_downcam_bin
            # Moving average
            PREV_WEIGHT = 1 - self.bin_moving_average_weight
            self.down_cam_bin_position = (
                (self.down_cam_bin_position[0] * PREV_WEIGHT + current_bin_position[0] * self.bin_moving_average_weight),
                (self.down_cam_bin_position[1] * PREV_WEIGHT + current_bin_position[1] * self.bin_moving_average_weight)
            ) if self.down_cam_bin_position is not None else current_bin_position
                        
            # Get the angle based on fov
            x_angle = math.radians(self.down_cam_bin_position[0] / self.camera_width * self.downcam_fov_horizontal)
            y_angle = math.radians(self.down_cam_bin_position[1] / self.camera_height * self.downcam_fov_vertical)

            forward_goal = -math.tan(y_angle) * (self.go_above_bin_height - 0.1)
            sway_goal = -math.tan(x_angle) * (self.go_above_bin_height - 0.1)
            self.node.get_logger().info(f"[{self.name}] Calculated physical goal -> Forward: {forward_goal:.3f}m, Sway: {sway_goal:.3f}m")

            self.expected_failures += 1  # current goal will fail once, ignore that failure

            # We know the bin is 1.0m below us, so calculate the bin position
            self.goal = move_robot_centric(forward=forward_goal, sway=sway_goal)
            # self.goal = move_robot_centric(forward=-self.down_cam_bin_position[1] / CAMERA_HEIGHT, sway=-self.down_cam_bin_position[0] / CAMERA_WIDTH)
            self.action_status = ActionStatus.NOT_SENT  # Next tick, new goal will be sent automatically

        return py_trees.common.Status.RUNNING

class GoAboveClosestBin(py_trees.behaviour.Behaviour):
    def __init__(self, bins_params: dict = None):
        super().__init__("GoAboveClosestBin")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.action_status = ActionStatus.NOT_SENT
        self.bins_params = bins_params or {}
        # cache configured go height from yaml (fail if missing)
        self.go_height = self.bins_params['go_above_bin_height']

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)
        self.blackboard.register_key(key="/bins_task/closest_bins", access=py_trees.common.Access.WRITE)

    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.node.get_logger().error(f"[{self.name}] Goal rejected by server.")
            self.action_status = ActionStatus.FAILED
    
    def on_server_goal_result(self, goal_success: bool, message: str) -> None:
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to go above closest bin. {message}")
            self.action_status = ActionStatus.FAILED

    def update(self) -> py_trees.common.Status:
        if self.action_status == ActionStatus.FAILED:
            return py_trees.common.Status.FAILURE
        elif self.action_status == ActionStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        elif self.action_status == ActionStatus.NOT_SENT:
            if not hasattr(self.blackboard.bins_task, 'closest_bins') or self.blackboard.bins_task.closest_bins is None:
                return py_trees.common.Status.FAILURE
            
            bins = self.blackboard.bins_task.closest_bins
            if bins[0] == None or (bins[0].visited and bins[1] == None):
                return py_trees.common.Status.FAILURE
            
            if bins[0].visited:
                bin_position = bins[1].pose
                bins[1].visited = True
            else:
                bin_position = bins[0].pose
                bins[0].visited = True

            goal = move_global(bin_position.x, bin_position.y, bin_position.z + self.go_height)
            self.navigation_client.send_navigation_goal(goal, self.name, custom_goal_response=self.on_server_goal_response, custom_goal_result=self.on_server_goal_result)

            self.action_status = ActionStatus.PENDING

            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.RUNNING

# Placeholder for drop marker
class DropMarker(py_trees.behaviour.Behaviour):
    def __init__(self, bins_params: dict = None):
        super().__init__("DropMarker")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.bins_params = bins_params or {}
        self.required_markers = self.bins_params['num_required_markers']
    
    def setup(self, **kwargs):
        self.blackboard.register_key(key="/bins_task/number_markers", access=py_trees.common.Access.WRITE)
    
    def update(self) -> py_trees.common.Status:
        if not hasattr(self.blackboard.bins_task, 'number_markers'):
            return py_trees.common.Status.FAILURE
        
        self.blackboard.bins_task.number_markers += 1
        if self.blackboard.bins_task.number_markers >= self.required_markers:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
