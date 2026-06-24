import py_trees
import math 
import planner.missions.vision_behaviours as vision_behaviours
from controls.goal_helpers import set_depth, move_global, move_robot_centric
from planner.missions.action_status_enum import ActionStatus
from ..mission_behaviour_components import BasicActionBehaviour

# The following classes are defined in a pre-order fashion, meaning that if a class is used in another class, it is defined before the class that uses it.
#  For example, CheckAboveTable is defined before GoAboveTable, since GoAboveTable uses CheckAboveTable as a child behavior.

class GoAboveTable(py_trees.composites.Sequence):
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float, **kwargs):
        super().__init__("Go Above Table", memory=True)

        self.blackboard = self.attach_blackboard_client(name="GoAboveTable Blackboard")
        self.blackboard.register_key('/missions/octagon/expected_table_items', access=py_trees.common.Access.WRITE)

        # Initialize the expected table items, this list will get truncated as objects are placed
        # in their respective bins
        self.blackboard.missions.octagon.expected_table_items = ["table", "pill", "nutbolt", "electric", "bandaid", "warning", "redcross_helmet"]

        shallow_approach_depth = kwargs.get("shallow_approach_depth", -0.4)
        shallow_approach_tolerance = kwargs.get("shallow_approach_tolerance", position_tolerance)
        shallow_approach_hold_time = kwargs.get("shallow_approach_hold_time", hold_time)
        
        # 1. Go to a set depth to view the table better. The table is best seen at a shallow depth, since we get to
        # see the top of the table, whereas at a lower depth there is a lot of empty space
        go_shallow_depth = BasicActionBehaviour(
            name="Octagon: Go Shallow Depth",
            goal=set_depth(z=shallow_approach_depth, tolerance=shallow_approach_tolerance, hold_time=shallow_approach_hold_time, timeout=timeout)
        )

        # 2. Navigate to Octagon by looking for the table 
        search_for_table = vision_behaviours.SearchSweepBehaviour(
            target_class="table",
            num_steps=5,
            step_timeout=0.5)
        
        # 3. Navigate 4 meters away from the table to get a better view of the table
        # With the better view, we get more reliable depth readings of the table, which
        # are needed to go right above it
        go_to_table_1m_away = vision_behaviours.GoNearObject(
            target_class="table",
            target_distance=4.0,
            tolerance_meters=shallow_approach_tolerance,
            height_offset=None,
            hold_time=hold_time
        ) 

        # 4. Navigate 2 meters away from the table to get a better view of the table
        # With the better view, we get more reliable depth readings of the table, which
        # are needed to go right above it
        go_to_table2 = vision_behaviours.GoNearObject(
            target_class="table",
            target_distance=2.0,
            tolerance_meters=shallow_approach_tolerance,
            height_offset=None,
            hold_time=hold_time
        )

        go_table = vision_behaviours.GoNearObject(
            name="Go To Table (Front Cam)",
            target_class="table",
            target_distance=0.0,
            tolerance_meters=shallow_approach_tolerance,
            height_offset=None,
            hold_time=hold_time
        )

        go_table_refined = vision_behaviours.GoNearObject(
            name="Go To Table Refined (Down Cam Update)",
            target_class="table",
            target_distance=0.0,
            tolerance_meters=shallow_approach_tolerance,
            height_offset=None,
            hold_time=hold_time
        )

        #5 Center around that the table is underneath Dougie
        align_table = AlignWithTable(**kwargs)

        surface_depth = kwargs.get("surface_depth", -0.1)
        surface_tolerance = kwargs.get("surface_tolerance", 0.1)
        surface_hold_time = kwargs.get("surface_hold_time", 3.0)
        
        #6 Surface octagon
        surface_octagon = BasicActionBehaviour(
            name="Surface in Octagon", 
            goal=set_depth(z=surface_depth, tolerance=surface_tolerance, hold_time=surface_hold_time, timeout=timeout)
        )


        #7 Scan octagon for images
        scan_images = ScanOctagonImages(num_steps_per_side=5)

        navigation_only = kwargs.get("navigation_only", True)

        children = [
            go_shallow_depth,
            search_for_table,
            go_to_table_1m_away,
            go_to_table2,
            go_table,
            go_table_refined,
        ]

        if not navigation_only:
            children.append(align_table)
            
        children.append(surface_octagon)

        if not navigation_only:
            children.append(scan_images)

        self.add_children(children)


class GoTable(py_trees.composites.Sequence):
    def __init__(self, target_distance: float, tolerance_meters: float, height_offset: float, hold_time: float):
        super().__init__("Go table", memory=True)

        #1 Go until table is seen
        go_till_table_seen = GoTillTableSeen(target_distance,
                                tolerance_meters,
                                height_offset,
                                hold_time)
        
        # 2 If overly prematurely seen (not close to goal position), go to pose when table first seen
        go_premature_table_position_if_seen = GoTablePrematurelySeen()

        self.add_children([go_till_table_seen,
                           go_premature_table_position_if_seen])


class GoTillTableSeen(py_trees.composites.Parallel):
    def __init__(self, target_distance: float, tolerance_meters: float, height_offset: float, hold_time: float):
        super().__init__("Navigate towards table until it is seen under", policy=py_trees.common.ParallelPolicy.SuccessOnOne())

        # 1 Stop if we can detect the table underneath us
        stop_if_see_table = StopTableBelow() # Is buggy rn, sends in eternal loop of preemption and sending goal, need to add another behaviour check, im lazy

        # 2 Send an object-map based goal to go to the table
        go_to_table = vision_behaviours.GoNearObject(
            target_class="table",
            target_distance=0.0,
            tolerance_meters=tolerance_meters,
            height_offset=None,
            hold_time=hold_time
        )

        # 3 TODO: Fallback for when no table after, think of naive moving again in straight line and hope #1 gets it

        self.add_children([stop_if_see_table,
                          go_to_table])

class StopTableBelow(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("StopTableBelow")
        self.blackboard = self.attach_blackboard_client(name="Stop if Table below")
        self.initial_pose = None
        self.has_stopped = False
    
    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)

        self.blackboard.register_key('/vision/down_cam/detections', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/vision/last_goal_pose', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/sensors/pose', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/sensors/twist', access=py_trees.common.Access.READ)
        
        self.blackboard.register_key('/missions/octagon/pose_at_premature_seen_table', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key('/missions/octagon/twist_at_premature_seen_table', access=py_trees.common.Access.WRITE)

    def initialise(self):
        # Reset initial goal pose and distance upon beginning of behaviour
        if hasattr(self.blackboard, 'sensors') and self.blackboard.sensors.pose is not None:
            self.initial_pose = self.blackboard.sensors.pose.pose.position
        else:
            self.node.get_logger().warn("Stop table below: No blackboard sensor pose")

        self.blackboard.missions.octagon.pose_at_premature_seen_table = None

    def update(self):
        # Verify if we are close to the goal, if we are already close to table, it is natural to 
        # see it in the down cam view
        current_position = self.blackboard.sensors.pose.pose.position 
        goal_position = self.blackboard.vision.last_goal_pose.target_pose.position
        if math.sqrt( (current_position.x - goal_position.x)**2 + ((current_position.y - goal_position.y)**2)) < 1.0:
            return py_trees.common.Status.RUNNING 

        # Look if we can see a table in down cam detections
        for detection in self.blackboard.vision.down_cam.detections.detections:
                if detection.label == "table":
                    self.navigation_client.cancel_navigation_goal() # TODO: Add perhaps a callback to wait till this is done
                    self.blackboard.missions.octagon.pose_at_premature_seen_table = self.blackboard.sensors.pose.pose.position
                    return py_trees.common.Status.SUCCESS

class GoTablePrematurelySeen(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("GoTablePrematurelySeen")
        self.blackboard = self.attach_blackboard_client(name="Stop if Table below")
        self.action_status = None
        self.tolerance_meters = 1.0
        self.hold_time = 1.0
    
    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)

        self.blackboard.register_key('/missions/octagon/pose_at_premature_seen_table', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/missions/octagon/twist_at_premature_seen_table', access=py_trees.common.Access.READ)

    def initialise(self):
        self.action_status = ActionStatus.NOT_SENT
        self.expected_failures = 1
    
    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.node.get_logger().error(f"[{self.name}] Goal rejected by server.")
            self.action_status = ActionStatus.FAILED
    
    def on_server_goal_result(self, goal_success: bool, message: str) -> None:
        if goal_success:
            # Naive simple controller to get table in the middle 
            self.action_status = ActionStatus.SUCCEEDED
        elif self.expected_failures > 0:
            self.expected_failures -= 1
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to go to table. {message}")
            self.action_status = ActionStatus.FAILED

    def update(self):
        if self.action_status == ActionStatus.SUCCEEDED:
            self.node.get_logger().info(f"[{self.name}] Returning SUCCESS.")
            return py_trees.common.Status.SUCCESS
        elif self.action_status == ActionStatus.FAILED:
            self.node.get_logger().info(f"[{self.name}] Returning FAILURE.")
            return py_trees.common.Status.FAILURE
        elif self.action_status == ActionStatus.PENDING:
            return py_trees.common.Status.RUNNING
        

        if self.action_status == ActionStatus.NOT_SENT:
            pose_at_premature_seen_table = self.blackboard.missions.octagon.pose_at_premature_seen_table
            if pose_at_premature_seen_table is not None:
                position_at_premature_seen_table = pose_at_premature_seen_table
                print(position_at_premature_seen_table)
                goal = move_global(x=position_at_premature_seen_table.x, y=position_at_premature_seen_table.y, do_z=False, tolerance=self.tolerance_meters, hold_time=self.hold_time)
                self.navigation_client.send_navigation_goal(goal, self.name, custom_goal_response=self.on_server_goal_response, 
                    custom_goal_result=self.on_server_goal_result)
                self.action_status = ActionStatus.PENDING
                return py_trees.common.Status.RUNNING
            else:
                return py_trees.common.Status.SUCCESS


class AlignWithTable(py_trees.composites.Selector):
    def __init__(self, position_tolerance: float = 0.2, hold_time: float = 1.0, timeout: float = 30.0, number_of_items_to_check: int = 2, **kwargs):
        super().__init__("Align with Table", memory=False)

        self.blackboard = self.attach_blackboard_client(name="AlignWithTable Blackboard")

        # 1. Check that we are above the table by verifying that we can see at least 2 items on the table (including the table itself
        check_above_table = CheckAboveTable(name="Octagon: Check Above Table", number_of_items_to_check=number_of_items_to_check, **kwargs)

        # 2. If we are not properly above the table, align yourself again 
        # upon success, 
        loop_align_check = LoopTableAlignCheck(**kwargs)

        retry_loop_align_check = py_trees.decorators.Retry(
            name="Retry",
            child=loop_align_check,
            num_failures=5
        )

        self.add_children([
            check_above_table,
            retry_loop_align_check
        ])

class CheckAboveTable(py_trees.behaviour.Behaviour):
    def __init__(self, name="Octagon: Check Above Table", number_of_items_to_check: int = 1, target_consecutive_ticks_full_set_of_items: int = 10, **kwargs):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name="CheckAboveTable Blackboard")
        self.number_of_items_to_check = number_of_items_to_check
        self.table = False
        self.pool_depth = kwargs.get("pool_depth", 2.1)
        self.known_height_to_pill = kwargs.get("known_height_to_pill", 0.745625)
        self.known_pill_area = kwargs.get("known_pill_area", 16900)
        self.consecutive_ticks_full_set_of_items = 0
        self.target_consecutive_ticks_full_set_of_items = 10

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)
        
        self.blackboard.register_key('/vision/down_cam/detections', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/sensors/pose', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/missions/octagon/expected_table_items', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/mission/octagon_task/view_table', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key('/mission/octagon_task/height_table', access=py_trees.common.Access.WRITE)

    def initialise(self) -> None:
        self.table = None
        
            
    def update(self) -> py_trees.common.Status:
        if not hasattr(self.blackboard, 'vision') or self.blackboard.vision.down_cam.detections is None:
            self.node.get_logger().error("Blackboard down cam detection key is not set up or detections is None")
            return py_trees.common.Status.FAILURE

        self.seen_items = []
        potential_pill_detection = None
        # Verify in detections if there are at least number_of_items_to_check items in the down cam detections 
        # to certify we are above the table (not neccesarily centered)
        for detection in self.blackboard.vision.down_cam.detections.detections:
            if detection.label in self.blackboard.missions.octagon.expected_table_items:
                if detection.label == "table":
                    self.blackboard.mission.octagon_task.view_table = detection # Extract the table since needed for alignment
                elif detection.label == "pill":
                    potential_pill_detection = detection # Extract the pill since its BB can help us determine height
                self.seen_items.append(detection.label)

        seen_items_str = ""
        for i in range(len(self.seen_items)):
            seen_items_str += self.seen_items[i]
        if len(self.seen_items) > 3:
            self.node.get_logger().info(f"Seen: {seen_items_str}")

        if set(self.blackboard.missions.octagon.expected_table_items).issubset(self.seen_items):
            area_pill = potential_pill_detection.bbox_size_x * potential_pill_detection.bbox_size_y
            
            # From inverse proportionality with area and distance / TODO: Replace with DVL transducer
            # beams for altitude because turns out this inverse prop. thing stinks some hot cheeks
            height_to_pill = self.known_height_to_pill * math.sqrt(self.known_pill_area/area_pill)
            self.blackboard.mission.octagon_task.height_table = self.pool_depth - height_to_pill - (-1 * self.blackboard.sensors.pose.pose.position.z)
            
            self.consecutive_ticks_full_set_of_items += 1
            if self.consecutive_ticks_full_set_of_items >= self.target_consecutive_ticks_full_set_of_items:  
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
        else:
            self.consecutive_ticks_full_set_of_items = 0
            return py_trees.common.Status.FAILURE

class LoopTableAlignCheck(py_trees.composites.Sequence):
    def __init__(self, position_tolerance: float = 0.2, hold_time: float = 1.0, timeout: float = 30.0, **kwargs):
        super().__init__("Align with Table", memory=True)

        self.blackboard = self.attach_blackboard_client(name="AlignWithTable Blackboard")


        # 1. If we are not properly above the table, align yourself again 
        # upon success, 
        center_table = CenterTable(**kwargs)

        # 2. Check that we are above the table by verifying that we can see at least 2 items on the table (including the table itself
        check_above_table = CheckAboveTable(number_of_items_to_check=2, **kwargs)

        self.add_children([
            center_table,
            check_above_table
        ])


class CenterTable(py_trees.behaviour.Behaviour):
    def __init__(self, name="Center table",  discovery_distance: float = 0.3, **kwargs):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name="CheckAboveTable Blackboard")
        self.acceptable_table_items = ["table", "pill", "nut", "electric", "bandaid"]
        self.number_of_items_to_check = 2
        self.discovery_distance = kwargs.get("discovery_distance", discovery_distance)
        self.camera_width = kwargs.get("downcam_image_width", 640)
        self.camera_height = kwargs.get("downcam_image_height", 480)
        self.downcam_fov_horizontal = kwargs.get("downcam_fov_horizontal", 59.7)
        self.downcam_fov_vertical = kwargs.get("downcam_fov_vertical", 47.6)
        self.table_avg_height = kwargs.get("table_avg_height", 0.75)
        self.action_status = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)
        
        self.blackboard.register_key('/vision/down_cam/detections', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/sensors/pose', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/mission/octagon_task/view_table', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/mission/octagon_task/height_table', access=py_trees.common.Access.READ)
        
    def initialise(self) -> None:
        self.action_status = ActionStatus.NOT_SENT
        self.seen_items = []

    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.node.get_logger().error(f"[{self.name}] Goal rejected by server.")
            self.action_status = ActionStatus.FAILED
    
    def on_server_goal_result(self, goal_success: bool, message: str) -> None:
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to go center table. {message}")
            self.action_status = ActionStatus.FAILED
            
    def update(self) -> py_trees.common.Status:
        if not hasattr(self.blackboard, 'vision') or self.blackboard.vision.down_cam.detections is None:
            self.node.get_logger().error("Blackboard down cam detection key is not set up or detections is None")
            return py_trees.common.Status.FAILURE
        
        if self.action_status == ActionStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
    
        if self.action_status == ActionStatus.PENDING:
            self.node.get_logger().info("PASS1.5")
            return py_trees.common.Status.RUNNING
        
        table = self.blackboard.mission.octagon_task.view_table
        # Find the general direction of the table by looking at which corner the table BB is in.
        # Move in that general direction to try to see more of it.
        cx = table.bbox_center_x
        cy = table.bbox_center_y
        w = table.bbox_size_x
        h = table.bbox_size_y
        x_min = cx - w / 2.0
        x_max = cx + w / 2.0
        y_min = cy - h / 2.0
        y_max = cy + h / 2.0

        x_angle = math.radians( (cx - self.camera_width/2) / self.camera_width * self.downcam_fov_horizontal)
        y_angle = math.radians( (cy - self.camera_height/2) / self.camera_height * self.downcam_fov_vertical)

        height_to_table = 2.1 - (-1 * self.blackboard.sensors.pose.pose.position.z + self.table_avg_height)
        self.node.get_logger().info("sup")
        
        x = math.tan(x_angle) * height_to_table
        y = math.tan(y_angle) * height_to_table
        self.node.get_logger().info(f"forward: {-y}, sway: {-x}")
        goal = move_robot_centric(forward=-y, sway=-x, heave=-1 * self.blackboard.sensors.pose.pose.position.z - 0.1, tolerance=0.1, hold_time=3.0)

        self.navigation_client.send_navigation_goal(goal, self.name, custom_goal_response=self.on_server_goal_response, 
                custom_goal_result=self.on_server_goal_result)
        self.action_status = ActionStatus.PENDING

        return py_trees.common.Status.RUNNING

class ScanOctagonImages(py_trees.composites.Sequence):
    def __init__(self, num_steps_per_side=5):
        super().__init__("Scan Octagon Images", memory=True)

        # 1. Scan a 360 deg yaw to find images
        scan_for_objects = vision_behaviours.ScanBehaviour(turn_hold_time_s=2.0, scan_angle_deg=180, num_steps_per_side=num_steps_per_side)

        # 2. Verify all images are present and log angles to BB
        verify_images = VerifyOctagonImages()

        self.add_children([
            scan_for_objects,
            verify_images
        ])
    
class VerifyOctagonImages(py_trees.behaviour.Behaviour):
    def __init__(self, name="Verify octagon images", octagon_images=["compass", "tools", "lifebuoy", "sos"]):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name="CheckAboveTable Blackboard")
        self.octagon_images = octagon_images

    def setup(self, **kwargs):
        self.node = kwargs['node']

        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/mission/octagon_task/octagon_images", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/mission/octagon_task/pose_at_scan_image", access=py_trees.common.Access.WRITE)

    def update(self):
        if hasattr(self.blackboard, 'vision') and self.blackboard.vision.object_map is None:
            return py_trees.common.Status.FAILURE
        
        auv_pose = self.blackboard.sensors.pose.pose.position
        self.blackboard.mission.octagon_task.pose_at_scan_image = auv_pose
        self.blackboard.mission.octagon_task.octagon_images = {}

        count_object = len(self.octagon_images)
        for obj in self.blackboard.vision.object_map.array:
            self.node.get_logger().info(f"{obj.label}")
            if obj.label in self.octagon_images:
                yaw_angle_rad = math.atan2(obj.pose.position.y - auv_pose.y, obj.pose.position.x - auv_pose.x)
                self.blackboard.mission.octagon_task.octagon_images[obj.label] = yaw_angle_rad
                count_object -= 1
        self.node.get_logger().info(f"{self.blackboard.mission.octagon_task.octagon_images}")

        if count_object != 0:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

class DropItemInBasket(py_trees.composites.Sequence):
    def __init__(self, item_to_grab="pill", **kwargs):
        super().__init__(f"Drop Item {item_to_grab}", memory=True)

        # 1. Go to object 
        go_object_vicinity = vision_behaviours.GoNearObject(
            target_class=item_to_grab,
            target_distance=4.0,
            tolerance_meters=0.1,
            height_offset=0.1,
            hold_time=3.0
        )

        # 2. Align properly on object Center (make this a 2 parter, one for rotation, useful for rectangle items, one for translation)
        go_object = GoObject(item_to_grab, **kwargs)

        # 3. Grab object and surface (verify that the object is still in grabber)

        # 4. Go to respective bin 

        # 5. Drop the item

        # 6. Go back to center table and surface


class GoObject(py_trees.behaviour.Behaviour):
    def __init__(self, item_to_grab, **kwargs):
        super().__init__(f"Go Item Object: {item_to_grab}")
        self.blackboard = self.attach_blackboard_client(name="CheckAboveTable Blackboard")
        self.action_status = None 
        self.item_to_grab = item_to_grab
        self.rotate_for_specific_items = True
        self.camera_width = kwargs.get("downcam_image_width", 640)
        self.camera_height = kwargs.get("downcam_image_height", 480)
        self.downcam_fov_horizontal = kwargs.get("downcam_fov_horizontal", 59.7)
        self.downcam_fov_vertical = kwargs.get("downcam_fov_vertical", 47.6)
        self.table_avg_height = kwargs.get("table_avg_height", 0.75)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']

        self.blackboard.register_key('/vision/down_cam/detections', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/sensors/pose', access=py_trees.common.Access.READ)

    def initialise(self) -> None:
        self.action_status = ActionStatus.NOT_SENT

    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.node.get_logger().error(f"[{self.name}] Goal rejected by server.")
            self.action_status = ActionStatus.FAILED
    
    def on_server_goal_result(self, goal_success: bool, message: str) -> None:
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to go to object. {message}")
            self.action_status = ActionStatus.FAILED

    def update(self):
        if self.action_status == ActionStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
    
        if self.action_status == ActionStatus.PENDING:
            return py_trees.common.Status.RUNNING
        
        if self.action_status == ActionStatus.FAILED:
            return py_trees.common.Status.FAILURE
        
        need_to_rotate = False
        desired_detection = None
        if self.item_to_grab in ["bandaid", "electric"] and self.rotate_for_specific_items:
            need_to_rotate = True

        # Get the detection and go to it 
        for detection in self.blackboard.vision.down_cam.detections.detections:
            if detection.label == self.item_to_grab:
                desired_detection = detection
                break 
                
        # Find the general direction of the table by looking at which corner the table BB is in.
        # Move in that general direction to try to see more of it.
        cx = desired_detection.bbox_center_x
        cy = desired_detection.bbox_center_y
        w = desired_detection.bbox_size_x
        h = desired_detection.bbox_size_y

        # TODO: Make a check (maybe other behaviour) to move until center of BB is fully in frame
        x_min = cx - w / 2.0
        x_max = cx + w / 2.0
        y_min = cy - h / 2.0
        y_max = cy + h / 2.0

        x_angle = math.radians( (cx - self.camera_width/2) / self.camera_width * self.downcam_fov_horizontal)
        y_angle = math.radians( (cy - self.camera_height/2) / self.camera_height * self.downcam_fov_vertical)

        height_to_table = self.blackboard.mission.octagon_task.height_table
        
        x = math.tan(x_angle) * height_to_table/2
        y = math.tan(y_angle) * height_to_table/2

        goal = move_robot_centric(forward=-y, sway=-x, heave=-height_to_table + 0.1, hold_time=3.0)

        self.navigation_client.send_navigation_goal(goal, self.name, custom_goal_response=self.on_server_goal_response, 
                custom_goal_result=self.on_server_goal_result)
        self.action_status = ActionStatus.PENDING
        return py_trees.common.Status.RUNNING
    


