from typing import Any

import py_trees
import math 
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster, TransformStamped
from tf_transformations import quaternion_from_euler
import planner.missions.vision_behaviours as vision_behaviours
from controls.goal_helpers import set_depth, move_global, move_robot_centric, move_rigid_component_global, set_global_yaw
from planner.missions.action_status_enum import ActionStatus
from scipy.spatial.transform import Rotation
from ..mission_behaviour_components import BasicActionBehaviour, TimerBehaviour
from std_msgs.msg import UInt8
from auv_msgs.srv import SetDownCamProjectionHeights

# The following classes are defined in a pre-order fashion, meaning that if a class is used in another class, it is defined before the class that uses it.
#  For example, CheckAboveTable is defined before GoAboveTable, since GoAboveTable uses CheckAboveTable as a child behavior.

class GoAboveTable(py_trees.composites.Sequence):
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float, **kwargs):
        super().__init__("Go Above Table", memory=True)

        self.blackboard = self.attach_blackboard_client(name="GoAboveTable Blackboard")
        self.blackboard.register_key('/missions/octagon/expected_table_items', access=py_trees.common.Access.WRITE)

        # Initialize the expected table items, this list will get truncated as objects are placed
        # in their respective bins
        expected_table_items = kwargs.get("expected_table_items", ["table", "pill", "nutbolt", "electric", "bandaid", "warning", "redcross_helmet"])
        self.blackboard.missions.octagon.expected_table_items = expected_table_items

        # 1. Go to a set depth to view the table better. The table is best seen at a shallow depth, since we get to
        # see the top of the table, whereas at a lower depth there is a lot of empty space
        shallow_approach_depth = kwargs.get("shallow_approach_depth", -0.4)
        shallow_approach_tolerance = kwargs.get("shallow_approach_tolerance", position_tolerance)
        shallow_approach_hold_time = kwargs.get("shallow_approach_hold_time", hold_time)

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
        
        # 5 Go back to shallow_depth
        go_shallow_depth_2 = BasicActionBehaviour(
            name="Octagon: Go Shallow Depth",
            goal=set_depth(z=shallow_approach_depth, tolerance=shallow_approach_tolerance, hold_time=shallow_approach_hold_time, timeout=timeout)
        )

        # 6 Center around that the table is underneath Dougie
        align_table = AlignWithTable(**kwargs)
        
        # 7 Get the depth of the table using DVL, this is more reliable than using the down cam 
        # since the down cam BB can be noisy and not give a good estimate
        get_table_depth = AcquireTableDepthDVL(**kwargs)
        
        #8 Surface octagon
        surface_depth = kwargs.get("surface_depth", -0.1)
        surface_tolerance = kwargs.get("surface_tolerance", 0.1)
        surface_hold_time = kwargs.get("surface_hold_time", 3.0)

        surface_octagon = BasicActionBehaviour(
            name="Surface in Octagon", 
            goal=set_depth(z=surface_depth, tolerance=surface_tolerance, hold_time=surface_hold_time, timeout=timeout)
        )


        # 9 Scan octagon for images
        scan_images = ScanOctagonImages(**kwargs)

        navigation_only = kwargs.get("navigation_only", False)
        children = []
        
        # TODO: make it fuse when get internets back :D
        children.append(go_shallow_depth)
        children.append(search_for_table)
        children.append(go_to_table_1m_away)
        children.append(go_to_table2)
        children.append(go_table)
        children.append(go_table_refined)
        children.append(go_shallow_depth_2)

        if not navigation_only:
            children.append(align_table)
            children.append(get_table_depth)
            
        children.append(surface_octagon)
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
        self.blackboard.mission.octagon_task.view_table = None
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
            #area_pill = potential_pill_detection.bbox_size_x * potential_pill_detection.bbox_size_y
            
            # From inverse proportionality with area and distance / TODO: Replace with DVL transducer
            # beams for altitude because turns out this inverse prop. thing stinks some hot cheeks
            #height_to_pill = self.known_height_to_pill * math.sqrt(self.known_pill_area/area_pill)
            #self.blackboard.mission.octagon_task.height_table = self.pool_depth - height_to_pill - (-1 * self.blackboard.sensors.pose.pose.position.z)
            
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
        self.pool_depth = kwargs.get('pool_depth', 2.1)
        self.discovery_distance = kwargs.get("discovery_distance", discovery_distance)
        self.camera_width = kwargs.get("downcam_image_width", 640)
        self.camera_height = kwargs.get("downcam_image_height", 480)
        self.downcam_fov_horizontal = kwargs.get("downcam_fov_horizontal", 59.7)
        self.downcam_fov_vertical = kwargs.get("downcam_fov_vertical", 47.6)
        self.table_avg_height = kwargs.get("table_avg_height", 0.75)

        self.down_cam_positional_tolerance = kwargs.get('down_cam_nav_position_tolerance', '0.1')
        self.down_cam_yaw_tolerance = kwargs.get('down_cam_yaw_tolerance', 10.0)
        self.down_cam_hold_time = kwargs.get('down_cam_hold_time', 3.0)
        self.down_cam_timeout = kwargs.get('down_cam_timeout', 30.0)

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
        
        if not hasattr(self.blackboard, 'mission') or self.blackboard.mission is None:
            self.node.get_logger().error("Blackboard mission key is not set up or mission is None")
            return py_trees.common.Status.FAILURE

        if not hasattr(self.blackboard.mission, 'octagon_task') or self.blackboard.mission.octagon_task is None:
            self.node.get_logger().error("Blackboard mission octagon_task key is not set up or octagon_task is None")
            return py_trees.common.Status.FAILURE
        
        if not hasattr(self.blackboard.mission.octagon_task, 'view_table') or self.blackboard.mission.octagon_task.view_table is None:
            self.node.get_logger().error("Blackboard mission octagon_task view_table key is not set up or view_table is None")
            return py_trees.common.Status.FAILURE

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

        # Use avg possible height here for estimation since we do not yet have measurement of true height
        # with help from DVL
        height_to_table = self.pool_depth - (-1 * self.blackboard.sensors.pose.pose.position.z + self.table_avg_height)

        x = math.tan(x_angle) * height_to_table
        y = math.tan(y_angle) * height_to_table
        self.node.get_logger().info(f"forward: {-y}, sway: {-x}")
        # Add the negatives because of image axis being X (right is positive) and Y (down is positive)
        goal = move_robot_centric(forward=-y, sway=-x, heave=0.0, tolerance=self.down_cam_positional_tolerance, 
                                  hold_time=self.down_cam_hold_time, timeout=self.down_cam_timeout)

        self.navigation_client.send_navigation_goal(goal, self.name, custom_goal_response=self.on_server_goal_response, 
                custom_goal_result=self.on_server_goal_result)
        self.action_status = ActionStatus.PENDING

        return py_trees.common.Status.RUNNING

class AcquireTableDepthDVL(py_trees.composites.Sequence):
    def __init__(self, name="Acquire Table Depth DVL", **kwargs):
        super().__init__(name, memory=True)
        self.blackboard = self.attach_blackboard_client(name="AcquireTableDepthDVL Blackboard")
        self.depth_to_measure_table_height = kwargs.get("depth_to_measure_table_height", -0.4)

        get_slightly_above_table = BasicActionBehaviour(
            goal=set_depth(z=self.depth_to_measure_table_height, tolerance=.15, hold_time=3.0, timeout=30.0)
        )

        measure_table_depth_dvl = MeasureTableDepthDVL(**kwargs)

        self.add_children([get_slightly_above_table,
            measure_table_depth_dvl
        ])

class MeasureTableDepthDVL(py_trees.behaviour.Behaviour):
    def __init__(self, name="Measure Table Depth DVL", **kwargs):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name="MeasureTableDepthDVL Blackboard")
        self.com_to_dvl = kwargs.get('COM_to_dvl_height', 0.0)
        self.table_max_height = kwargs.get('table_max_height', 0.9)
        self.table_min_height = kwargs.get('table_min_height', 0.6)
        self.table_avg_height = kwargs.get('table_avg_height', 0.75)
        self.pool_depth = kwargs.get('pool_depth', 2.1)
        self.service_sent = False
        self.future = None
        self.timeout_sec = 10.0
    
    def setup(self, **kwargs):
        self.node = kwargs['node']
        
        self.service_client = self.node.create_client(SetDownCamProjectionHeights, '/vision/down_cam/set_projection_heights')
        
        self.blackboard.register_key('/sensors/dvl/velocity', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/sensors/pose', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/mission/octagon_task/height_table', access=py_trees.common.Access.WRITE)

    def initialise(self):
        self.service_sent = False

    def update(self) -> py_trees.common.Status:
        if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.dvl.velocity is None:
            self.node.get_logger().error("Blackboard sensors dvl key is not set up or dvl is None")
            return py_trees.common.Status.FAILURE

        if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
            self.node.get_logger().error("Blackboard sensors pose key is not set up or pose is None")
            return py_trees.common.Status.FAILURE
        
        # Send the service
        if not self.service_sent:
            # Get the dvl altitude and acquire the table height from it
            
            # Set the DVL's unit vector for each transducer beams is 22.5 deg 
            # from DVL Z axis. 4 arrays in form of [x,y,z].
            # Each beam is configured at 45, -45, -135, 135 deg on the DVL XY plane
            x_transducer_proj_magnitude = math.sin(math.radians(22.5)) * math.sin(math.radians(45)) # spherical coords math
            y_transducer_proj_magnitude = math.cos(math.radians(22.5)) * math.sin(math.radians(45))
            z_transducer_proj_magnitude = math.cos(math.radians(45.0))


            # From the DVL protocol, beams are ordered in CW starting from top right. Axis is FLU convention
            unit_vectors_dvl = [[x_transducer_proj_magnitude, -y_transducer_proj_magnitude, -z_transducer_proj_magnitude], 
                                [-x_transducer_proj_magnitude, -y_transducer_proj_magnitude, -z_transducer_proj_magnitude], 
                                [-x_transducer_proj_magnitude, y_transducer_proj_magnitude, -z_transducer_proj_magnitude],
                                [x_transducer_proj_magnitude, y_transducer_proj_magnitude, -z_transducer_proj_magnitude]]
            
            candidate_table_heights = []

            # Acquire the AUV's roll and pitch
            quaternion = self.blackboard.sensors.pose.orientation
            r = Rotation.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            auv_rotation_in_euler_rpy = r.as_euler('ZYX', degrees=False) # In radians
            roll = auv_rotation_in_euler_rpy[2]
            pitch = auv_rotation_in_euler_rpy[1]

            
            for i in range(0,3):
                # Acquire DVL transducer beams in order 
                altitude = self.blackboard.sensors.dvl.velocity.beams[i].distance
                if altitude == -1:
                    # Not valid altitude
                    continue

                # Project altitude onto the Pool Z axis frame
                unit_vector_for_beam = [component * altitude for component in unit_vectors_dvl[i]]
                dvl_z_frame_beam_altitude = -math.sin(roll) * unit_vector_for_beam[0] \
                                            + math.cos(roll) * math.sin(pitch) * unit_vector_for_beam[1] \
                                            + math.cos(roll) * math.sin(pitch) * unit_vector_for_beam[2]
                
                # Flip the sign for it to be positive altitude 
                dvl_z_frame_beam_altitude *= -1

                # Find the table height from this given altitude
                table_height_from_floor = self.pool_depth - (-1 * self.blackboard.sensors.pose.pose.position.z + self.com_to_dvl + dvl_z_frame_beam_altitude)
                candidate_table_heights.append(table_height_from_floor)

            # Filter out altitude values outside expected table height ranges
            for height in candidate_table_heights:
                if not (self.table_min_height < height < self.table_max_height):
                    candidate_table_heights.pop(height)

            # Fuse the remaining measurements
            if len(candidate_table_heights) == 0:
                self.node.get_logger().info(f"Transducer altitudes not valid. Using average table height")
                self.blackboard.mission.octagon_task.height_table = self.table_avg_height
            else:
                fused_height = sum(candidate_table_heights)/len(candidate_table_heights)
                self.blackboard.mission.octagon_task.height_table = fused_height
            
            # Send the service to get down cam detection node to calculate pose with new table height as the plane for projections
            self.node.get_logger().info(f"[{self.name}] Acquired table height from DVL: {self.blackboard.mission.octagon_task.height_table:.3f} m")
            request = SetDownCamProjectionHeights.Request()
            request.all_labels = True
            request.projection_height = self.blackboard.mission.octagon_task.height_table
            request.specific_label = ""
            self.request_time = self.node.get_clock().now()
            self.future = self.service_client.call_async(request)
            self.service_sent = True
        
        # Still waiting for response
        if not self.future.done():
            # Check for timeout
            elapsed = (self.node.get_clock().now() - self.request_time).nanoseconds / 1e9
            if elapsed >= self.timeout_sec:
                self.node.get_logger().error(f"[{self.name}] Service call timed out after {self.timeout_sec:.1f}s")
                self.future.cancel()
                # Reset to allow retry on the next tick (stays RUNNING)
                self.service_sent = False
                return py_trees.common.Status.RUNNING
            return py_trees.common.Status.RUNNING
        
        # Check if the service call itself failed (e.g. middleware error)
        if self.future.exception() is not None:
            self.node.get_logger().error(f"[{self.name}] Service call failed: {self.future.exception()}")
            # Reset to allow retry on the next tick (stays RUNNING)
            self.service_sent = False
            return py_trees.common.Status.RUNNING
        
        # Verify if service is successful or not
        response = self.future.result()
        if response.success:
            self.node.get_logger().info(f"[{self.name}] Service succeeded.")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info(f"[{self.name}] Service failed: {response.message}")
            return py_trees.common.Status.FAILURE

class ScanOctagonImages(py_trees.composites.Sequence):
    def __init__(self, **kwargs):
        super().__init__("Scan Octagon Images", memory=True)
        
        self.scan_octagon_num_steps_per_side = kwargs.get('scan_octagon_num_steps_per_side', 3)
        self.scan_octagon_yaw_hold_time = kwargs.get('scan_octagon_yaw_hold_time', 2.0)
        self.scan_octagon_yaw_tolerance = kwargs.get('scan_octagon_yaw_tolerance', 180.0)
        self.navigation_only = kwargs.get('navigation_only', True)
        self.role = kwargs.get('role', "survey_repair")
        self.octagon_images_survey_repair = kwargs.get('octagon_images.survey_repair', [])
        self.octagon_images_search_rescue = kwargs.get('octagon_images.search_rescue', [])


        # 1. Scan a 360 deg yaw to find images
        scan_for_objects = vision_behaviours.ScanBehaviour(turn_hold_time_s=self.scan_octagon_yaw_hold_time, scan_angle_deg=180, 
                                                           angular_tolerance_rad=math.radians(self.scan_octagon_yaw_tolerance),
                                                           num_steps_per_side=self.scan_octagon_num_steps_per_side)

        # 2. Verify all images are present and log angles to BB
        verify_images = VerifyOctagonImages()

        # If navigation only, do search sweep and find image depending on role
        desired_image = self.octagon_images_survey_repair[0] if self.role == "survey_repair" else self.octagon_images_search_rescue[0]
        find_role_image = vision_behaviours.SearchSweepBehaviour(
            target_class=desired_image,
            num_steps=7,
            step_timeout=0.5
        )

        look_at_image_for_5s = TimerBehaviour(timer_duration=5.0)

        if self.navigation_only:
            self.add_children([
                find_role_image,
                look_at_image_for_5s
            ])
        else:
            self.add_children([scan_for_objects,
                verify_images])
    
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
# -------------------------------------TABLE CLEANUP TIME BAAAAAAAAAAAAAAABY ------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

class TableCleaning(py_trees.composites.Parallel):
    def __init__(self, **kwargs):
        super().__init__("Table Cleaning", policy=py_trees.common.ParallelPolicy.SuccessOnOne())

        self.task_cleaning_timeout = kwargs.get('time_before_abort', 200.0)

        # Time allocated before to clean up moving on to next task
        time_allocated_to_task = TimerBehaviour(timer_duration=self.task_cleaning_timeout)
        
        # Sequence of a clean up task per item to grab and drop
        sequence_of_clean_up_tasks = SequenceOfCleanUpTasks(**kwargs)

        self.add_children([
            time_allocated_to_task,
            sequence_of_clean_up_tasks
        ])


class SequenceOfCleanUpTasks(py_trees.composites.Sequence):
    def __init__(self, **kwargs):
        super().__init__("Sequence of item clean Up Tasks", memory=True)
        self.grab_only_for_role = kwargs.get('grab_only_for_role', True)
        self.number_of_items_to_grab = kwargs.get('number_of_items_to_grab', 1)
        self.role = kwargs.get('role', 'survey_repair')
        self.survey_repair_items_item_labels = kwargs.get('survey_repair_items.items_labels', [])
        self.search_rescue_items_item_labels = kwargs.get('search_rescue_items.items_labels', [])
        self.survey_repair_items_bin_label = kwargs.get('survey_repair_items.bin_label', [])
        self.search_rescue_items_bin_label = kwargs.get('search_rescue_items.bin_label', [])
        self.items_to_grab = ["bandaid", "nutbolt", "electric", "pill"]

        # Initialize amount of items dropped in good bin on blackboard
        self.blackboard = self.attach_blackboard_client(name="CheckAboveTable Blackboard")
        self.blackboard.register_key(key="/mission/octagon_task/items_in_good_bin", access=py_trees.common.Access.WRITE)
        self.blackboard.mission.octagon_task.items_in_good_bin = 0
      
        if self.grab_only_for_role:
            self.number_of_items_to_grab = min(self.number_of_items_to_grab, 2)
            
            if self.role == 'survey_repair':
                self.items_to_grab = ["nutbolt", "electric"]
            elif self.role == 'search_rescue':
                self.items_to_grab = ["bandaid", "pill"]
            else:
                self.items_to_grab = ["bandaid", "pill"]        

        for i in range(self.number_of_items_to_grab):
            bin_label = None
            item_to_grab = self.items_to_grab[i]
            if item_to_grab in self.survey_repair_items_item_labels:
                bin_label = self.survey_repair_items_bin_label
            elif item_to_grab in self.search_rescue_items_item_labels:
                bin_label = self.search_rescue_items_bin_label
            else:
                # Default label for bin
                bin_label = "Warning"

            self.add_children([
                DropItemInBasket(item_to_grab="electric", bin_to_drop_in=bin_label,**kwargs)])


class DropItemInBasket(py_trees.composites.Sequence):
    def __init__(self, item_to_grab="pill", bin_to_drop_in="redcross_helmet", **kwargs):
        super().__init__(f"Drop Item {item_to_grab}", memory=True)

        self.item_drop_height_relative_bin = kwargs.get('item_drop_height_relative_bin', 0.2)
        self.grab_only_no_drop = kwargs.get('grab_only_no_drop', False)
        self.chosen_role = kwargs.get('role', "survey_repair")      
        self.table_item_target_distance = kwargs.get('table_item_target_distance', 0.3)
        self.table_item_tolerance_meters = kwargs.get('table_item_tolerance_meters', 0.1)
        self.table_item_height_offset = kwargs.get('table_item_height_offset', 0.1)    
        self.table_item_hold_time = kwargs.get('table_item_hold_time', 3.0)              
        self.bin_target_distance = kwargs.get('bin_target_distance', 0.3)                 
        self.bin_tolerance_meters = kwargs.get('bin_tolerance_meters', 0.1)               
        self.bin_item_drop_height_relative_bin = kwargs.get('bin_item_drop_height_relative_bin', 0.1) 
        self.bin_hold_time = kwargs.get('bin_hold_time', 3.0)

        # 1. Go to object 
        go_object_vicinity = vision_behaviours.GoNearObject(
            target_class=item_to_grab,
            target_distance=self.table_item_target_distance,
            tolerance_meters=self.table_item_tolerance_meters,
            height_offset=self.table_item_height_offset,
            hold_time=self.table_item_hold_time
        )

        # 2. Align properly on object Center with downcam pixels
        go_object = AlignCameraWithObject(item_to_grab, **kwargs)

        # 3. Move such that the object is aligned on top of the grabber (verify that the object is still in grabber)
        align_grabber = AlignGrabberWithObject(item_to_grab, **kwargs)

        # 4. Lower depth to have object inside grabber
        lower_depth = LowerDepthToGrabber(item_to_grab, **kwargs)
        
        # 5. Grab object (TODO: verify that the object is still in grabber)
        close_actuator = ActuateGrabber(open=False, **kwargs)

        #6.  Surface to a depth where we can resee the full table, align to table to remeasure bins
        #  and go to the respective bin
        surface_depth = kwargs.get("surface_depth", -0.1)
        surface_tolerance = kwargs.get("surface_tolerance", 0.1)
        surface_hold_time = kwargs.get("surface_hold_time", 3.0)

        surface_octagon_1 = BasicActionBehaviour(
            name="Surface in Octagon", 
            goal=set_depth(z=surface_depth/2, tolerance=surface_tolerance, hold_time=surface_hold_time, timeout=30.0)
        )

        surface_octagon_2 = BasicActionBehaviour(
            name="Surface in Octagon", 
            goal=set_depth(z=surface_depth, tolerance=surface_tolerance, hold_time=surface_hold_time, timeout=30.0)
        )

        # 7. Go to respective bin (Need to IRL check if can stil see downcam with object grabbed)
        go_bin_vicinity = vision_behaviours.GoNearObject(
            target_class=bin_to_drop_in,
            target_distance=self.bin_target_distance,
            tolerance_meters=self.bin_tolerance_meters,
            height_offset=self.bin_item_drop_height_relative_bin,
            hold_time=self.bin_hold_time
        )

        # 8. Drop the item
        open_actuator = ActuateGrabber(open=False, **kwargs)

        # 9. Update the expected items on table now that we dropped one by checking
        # item is indeed in bin
        check_item_dropped_good = CheckItemNotDroppedInBin(object=item_to_grab, bin=bin_to_drop_in, chosen_role=self.chosen_role, **kwargs)
         
        # 10. Go back to where table was
        shallow_approach_tolerance = kwargs.get("shallow_approach_tolerance", 0.3)
        shallow_approach_hold_time = kwargs.get("shallow_approach_hold_time", 30.0)
        go_table = vision_behaviours.GoNearObject(
            name="Go To Table Refined (Down Cam Update)",
            target_class="table",
            target_distance=0.0,
            tolerance_meters=shallow_approach_tolerance,
            height_offset=0.3,
            hold_time=shallow_approach_hold_time
        )

        # 11. Surface 
        surface_depth = kwargs.get("surface_depth", -0.1)
        surface_tolerance = kwargs.get("surface_tolerance", 0.1)
        surface_hold_time = kwargs.get("surface_hold_time", 3.0)

        surface_octagon = BasicActionBehaviour(
            name="Surface in Octagon", 
            goal=set_depth(z=surface_depth, tolerance=surface_tolerance, hold_time=surface_hold_time, timeout=30.0)
        )

        # 12. Go back to center table
        align_table = AlignWithTable(**kwargs)

        children = [go_object_vicinity,
            go_object,
            align_grabber,
            lower_depth,
            close_actuator,
            surface_octagon_1,
            surface_octagon_2]
           
        if not self.grab_only_no_drop:
            children.append(go_bin_vicinity)
            children.append(open_actuator)
            children.append(check_item_dropped_good)
        
        children.append(go_table)
        children.append(surface_octagon)
        children.append(align_table)
            
        self.add_children(children)


class AlignCameraWithObject(py_trees.behaviour.Behaviour):
    def __init__(self, item_to_grab, **kwargs):
        super().__init__(f"Go Object Align Camera: {item_to_grab}")
        self.blackboard = self.attach_blackboard_client(name="CheckAboveTable Blackboard")
        self.action_status = None 
        self.item_to_grab = item_to_grab
        self.rotate_for_specific_items = True
        self.camera_width = kwargs.get("downcam_image_width", 640)
        self.camera_height = kwargs.get("downcam_image_height", 480)
        self.downcam_fov_horizontal = kwargs.get("downcam_fov_horizontal", 59.7)
        self.downcam_fov_vertical = kwargs.get("downcam_fov_vertical", 47.6)
        self.table_avg_height = kwargs.get("table_avg_height", 0.75)

        self.down_cam_positional_tolerance = kwargs.get('down_cam_nav_position_tolerance', '0.1')
        self.down_cam_yaw_tolerance = kwargs.get('down_cam_yaw_tolerance', 10.0)
        self.down_cam_hold_time = kwargs.get('down_cam_hold_time', 3.0)
        self.down_cam_timeout = kwargs.get('down_cam_timeout', 30.0)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']

        self.blackboard.register_key('/vision/down_cam/detections', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/sensors/pose', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/mission/octagon_task/height_table', access=py_trees.common.Access.READ)

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
        
        if desired_detection is None:
            self.node.get_logger().warn("Desired detection not found when aligning camera")
            return py_trees.common.Status.FAILURE
        
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
        #height_to_table = 0.7
        
        x = math.tan(x_angle) * height_to_table/2
        y = math.tan(y_angle) * height_to_table/2

        goal = move_robot_centric(forward=-y, sway=-x, heave=0.0, tolerance=self.down_cam_positional_tolerance, hold_time=self.down_cam_hold_time)

        self.navigation_client.send_navigation_goal(goal, self.name, custom_goal_response=self.on_server_goal_response, 
                custom_goal_result=self.on_server_goal_result)
        self.action_status = ActionStatus.PENDING
        return py_trees.common.Status.RUNNING

class AlignGrabberWithObject(py_trees.behaviour.Behaviour):
    def __init__(self, item_to_grab, **kwargs):
        super().__init__(f"Align Grabber with Object: {item_to_grab}")
        self.blackboard = self.attach_blackboard_client(name="AlignGrabberWithObject Blackboard")
        self.action_status = None 
        self.grabber_tf_xyz = kwargs.get("tf_auv_to_grabber.xyz", [0.0, 0.0, 0.0])
        self.grabber_tf_rpy = kwargs.get("tf_auv_to_grabber.rpy", [0.0, 0.0, 0.0])
        self.down_cam_tf_xyz = kwargs.get("tf_auv_to_down_cam.xyz", [0.0, 0.0, 0.0])
        self.grabber_position_tolerance = kwargs.get("grabber_position_tolerance", 1.0)
        self.grabber_yaw_tolerance = kwargs.get("grabber_yaw_tolerance", 10.0)
        self.grabber_hold_time = kwargs.get("grabber_hold_time", 3.0)
        self.grabber_timeout = kwargs.get("grabber_timeout", 30.0)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']

        self.blackboard.register_key('/sensors/pose', access=py_trees.common.Access.READ)

        # Set up the TF broadcaster for foxglove visualization if there is a transform parameter
        if self.grabber_tf_xyz is not None and self.grabber_tf_rpy is not None:
            self.static_tf_broadcaster = StaticTransformBroadcaster(self.node)

            t = TransformStamped()
            t.header.frame_id = "auv_link"
            t.child_frame_id = "grabber_link"
            xyz = self.grabber_tf_xyz
            rpy = self.grabber_tf_rpy
            t.transform.translation.x = xyz[0]
            t.transform.translation.y = xyz[1]
            t.transform.translation.z = xyz[2]

            q = quaternion_from_euler(rpy[0], rpy[1], rpy[2], axes='szyx')
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.static_tf_broadcaster.sendTransform(t)

    def initialise(self):
        self.action_status = None

    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.node.get_logger().error(f"[{self.name}] Goal rejected by server.")
            self.action_status = ActionStatus.FAILED
    
    def on_server_goal_result(self, goal_success: bool, message: str) -> None:
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to align grabber with object. {message}")
            self.action_status = ActionStatus.FAILED

    def update(self):
        if self.action_status == ActionStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
    
        if self.action_status == ActionStatus.PENDING:
            return py_trees.common.Status.RUNNING
        
        if self.action_status == ActionStatus.FAILED:
            return py_trees.common.Status.FAILURE
        
        # Reset initial goal pose and distance upon beginning of behaviour
        desired_grabber_pose = None
        if hasattr(self.blackboard, 'sensors') and self.blackboard.sensors.pose is not None:
            # Make the desired grabber pose where the camera currently is looking
            desired_pose_x = self.blackboard.sensors.pose.pose.position.x + self.down_cam_tf_xyz[0]
            desired_pose_y = self.blackboard.sensors.pose.pose.position.y + self.down_cam_tf_xyz[1]
            desired_pose_z = self.blackboard.sensors.pose.pose.position.z + self.down_cam_tf_xyz[2]
        else:
            self.node.get_logger().warn("Stop table below: No blackboard sensor pose")
            return py_trees.common.Status.FAILURE
        
        # No interest in rotation for grabber alignment
        goal = move_rigid_component_global(
            x=desired_pose_x,
            y=desired_pose_y,
            z=desired_pose_z,
            tolerance=self.grabber_position_tolerance,
            angular_tolerance=math.radians(self.grabber_yaw_tolerance),
            hold_time=self.grabber_hold_time,
            timeout=self.grabber_timeout
        )
        self.navigation_client.send_navigation_goal(goal, self.name, custom_goal_response=self.on_server_goal_response, 
                custom_goal_result=self.on_server_goal_result)
        self.action_status = ActionStatus.PENDING

        return py_trees.common.Status.RUNNING

class LowerDepthToGrabber(py_trees.behaviour.Behaviour):
    def __init__(self, name="Lower Depth to Grabber", **kwargs):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name="LowerDepthToGrabber Blackboard")
        self.grabber_tf_xyz = kwargs.get("tf_auv_to_grabber.xyz", [[0.0, 0.0, 0.0]])
        self.pool_depth = kwargs.get("pool_depth", 2.1)
        self.grabber_clearance_margin = kwargs.get("grabber_clearance_margin")
        self.grabber_desired_depth_tolerance = kwargs.get('grabber_desired_depth_tolerance', 0.1)
        self.grabber_desired_depth_hold_time = kwargs.get('grabber_desired_depth_hold_time', 3.0)
        self.grabber_desired_depth_timeout = kwargs.get('grabber_desired_depth_timeout', 30.0)

        self.action_status = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']

        self.blackboard.register_key('/sensors/pose', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/mission/octagon_task/height_table', access=py_trees.common.Access.READ)

    def initialise(self):
        self.action_status = ActionStatus.NOT_SENT

    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.node.get_logger().error(f"[{self.name}] Goal rejected by server.")
            self.action_status = ActionStatus.FAILED
    
    def on_server_goal_result(self, goal_success: bool, message: str) -> None:
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to lower depth to grabber. {message}")
            self.action_status = ActionStatus.FAILED

    def update(self):
        if self.action_status == ActionStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
    
        if self.action_status == ActionStatus.PENDING:
            return py_trees.common.Status.RUNNING
        
        if self.action_status == ActionStatus.FAILED:
            return py_trees.common.Status.FAILURE
        
        # Lower depth to grabber height
        offset_dougie_grabber_z = self.grabber_tf_xyz[2] if self.grabber_tf_xyz is not None else 0.0
        desired_depth = self.pool_depth -offset_dougie_grabber_z - \
            (self.blackboard.mission.octagon_task.height_table + self.grabber_clearance_margin)  # Add some offset for clearance
        
        #desired_depth = self.pool_depth - offset_dougie_grabber_z - (0.70 + 0.024)  # Add some offset for clearance
        self.node.get_logger().info(f"[{self.name}] Lowering depth to grabber: {desired_depth:.3f} m")
        goal = set_depth(z=-desired_depth, tolerance=self.grabber_desired_depth_tolerance, hold_time=self.grabber_desired_depth_hold_time,
                         timeout=self.grabber_desired_depth_timeout)
        
        self.navigation_client.send_navigation_goal(goal, self.name, custom_goal_response=self.on_server_goal_response, 
                custom_goal_result=self.on_server_goal_result)
        self.action_status = ActionStatus.PENDING

        return py_trees.common.Status.RUNNING

class ActuateGrabber(py_trees.composites.Sequence):
    def __init__(self, open=True, **kwargs):
        super().__init__("Actuate Grabber", memory=True)
        self.blackboard = self.attach_blackboard_client(name="ActuateGrabber Blackboard")

        actuate_grabber = SendActuateGrabberCommand(open=open, **kwargs)

        # Wait a second to ensure actuator has time to actuate before moving away
        wait_1s_for_actuator = TimerBehaviour(timer_duration=1.0)

        self.add_children([actuate_grabber, wait_1s_for_actuator])

class SendActuateGrabberCommand(py_trees.behaviour.Behaviour):
    def __init__(self, name="Actuate Grabber", open=True, **kwargs):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name="ActuateGrabber Blackboard")
        self.open = open

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.node.create_publisher(UInt8, "/actuator/grabber", 10)

    def update(self):
        # Send the command to actuate the grabber
        msg = UInt8()
        msg.data = 255 if self.open else 0
        self.node.publish("/actuator/grabber", msg)

        return py_trees.common.Status.SUCCESS

class CheckItemNotDroppedInBin(py_trees.behaviour.Behaviour):
    # We can't reliably check if the item was dropped inside the 
    # bin because it may fall sideways or land on top of one another.
    # Instead, we double check by making sure we do not detect outside of the box

    # Checking item inside the bin is important for the final rotation count at end of task
    def __init__(self, object="pill", bin="redcross_helmet", chosen_role="survey_repair", **kwargs):
        super().__init__("Check item dropped in bin")
        self.blackboard = self.attach_blackboard_client(name="LowerDepthToGrabber Blackboard")
        self.item_to_check = object
        self.desired_bin = bin
        self.role = chosen_role
        self.survey_repair_items = kwargs.get('survey_repair_items', {})
        self.search_rescue_items = kwargs.get('search_rescue_items', {})

    def setup(self, **kwargs):
        self.node = kwargs['node']

        self.blackboard.register_key('/vision/down_cam/detections', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/missions/octagon.expected_table_items', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key('/mission/octagon_task/items_in_good_bin', access=py_trees.common.Access.WRITE)
        
    def update(self):
        # No matter what, for simplicity, remove it from items we are expecting to be on the table
        if self.item_to_check in self.blackboard.missions.octagon.expected_table_items:
            self.blackboard.missions.octagon.expected_table_items.pop(self.item_to_check)
        
        # Check if there are overlaps between bounding boxes
        bin_detection = None
        item_detection = None

        for detection in self.blackboard.vision.down_cam.detections.detections:
            if detection.label == self.item_to_check:
                item_detection = detection
            elif detection.label == self.desired_bin:
                bin_detection = detection
        
        if item_detection is None or bin_detection is None:
            self.node.get_logger().info("Detections not found. Assume nothing was dropped in bin (we dont even see bin)")
            return py_trees.common.Status.FAILURE
        
        # Item boundaries
        item_cx = item_detection.bbox_center_x
        item_cy = item_detection.bbox_center_y
        item_w = item_detection.bbox_size_x
        item_h = item_detection.bbox_size_y
        item_x_min = item_cx - item_w / 2.0
        item_x_max = item_cx + item_w / 2.0
        item_y_min = item_cy - item_h / 2.0
        item_y_max = item_cy + item_h / 2.0

        # Bins boundaries
        bin_cx = bin_detection.bbox_center_x
        bin_cy = bin_detection.bbox_center_y
        bin_w = bin_detection.bbox_size_x
        bin_h = bin_detection.bbox_size_y
        bin_x_min = bin_cx - bin_w / 2.0
        bin_x_max = bin_cx + bin_w / 2.0
        bin_y_min = bin_cy - bin_h / 2.0
        bin_y_max = bin_cy + bin_h / 2.0

        # Look for overlap
        is_horizontal_seperate = bin_x_max < item_x_min or item_x_max < bin_x_min
        is_vertical_seperate = bin_y_max < item_y_min or item_y_max < bin_y_min

        if not (is_horizontal_seperate and is_vertical_seperate):
            # We do not detect the 2 items being seperated
            self.blackboard.mission.octagon_task.items_in_good_bin += 1

        return py_trees.common.Status.SUCCESS

# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ------------------------------------- CLEANUP FINISHED GO LOOK AT AND ROLL BAAAAAAAAAAAAAAABY ------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------

class LookAndSpin(py_trees.composites.Sequence):
    def __init__(self, **kwargs):
        super().__init__("LookAndSpin", memory=True)

        self.blackboard = self.attach_blackboard_client(name="Look and Spin Blackboard")
        def setup(self, **kwargs):
            # Children here are added in setup because we need to read from the blackboard to determine which image to look at
            self.node = kwargs['node']
            self.navigation_client = kwargs['shared_nav_client']

            self.blackboard.register_key(key="/mission/octagon_task/items_in_good_bin", access=py_trees.common.Access.READ)
            self.blackboard.register_key(key="/mission/octagon_task/octagon_images", access=py_trees.common.Access.READ)
            self.blackboard.register_key(key="/mission/octagon_task/octagon_images", access=py_trees.common.Access.READ)
            
            # 1. Look at image
            face_object_related_to_role = FaceObjectRelatedToRole(**kwargs)

            # 2. Do yaws depending on the items droppede in good bin
            spin_1st_if_needed = SpinIfNeeded(number_item_in_bin_to_check=1, **kwargs)

            pass_on_fail_1 = py_trees.decorators.FailureIsSuccess(
                name="spin_1st_if_needed fail_is_success",
                child=spin_1st_if_needed,
            )
            spin_2nd_if_needed = SpinIfNeeded(number_item_in_bin_to_check=2, **kwargs)

            pass_on_fail_2 = py_trees.decorators.FailureIsSuccess(
                name="spin_2nd_if_needed fail_is_success",
                child=spin_2nd_if_needed,
            )


            self.add_children([face_object_related_to_role,
                               pass_on_fail_1,
                               pass_on_fail_2])

class FaceObjectRelatedToRole(py_trees.behaviour.Behaviour):
    def __init__(self, name="Face Object Related to Role", **kwargs):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name="FaceObjectRelatedToRole Blackboard")
        self.role = kwargs.get('role', 'survey_repair')
        self.survey_repair_items_item_labels = kwargs.get('survey_repair_items.items_labels', [])
        self.search_rescue_items_item_labels = kwargs.get('search_rescue_items.items_labels', [])
        self.survey_repair_items_bin_label = kwargs.get('survey_repair_items.bin_label', [])
        self.search_rescue_items_bin_label = kwargs.get('search_rescue_items.bin_label', [])
        self.look_at_image_yaw_tolerance = kwargs.get('look_at_image_yaw_tolerance', 10.0)
        self.look_at_image_yaw_hold_time = kwargs.get('look_at_image_yaw_hold_time', 3.0)
        self.look_at_image_yaw_timeout = kwargs.get('look_at_image_yaw_timeout', 30.0)

        self.scan_octagon_yaw_tolerance = kwargs.get('scan_octagon_yaw_tolerance', 10.0)
        self.look_at_image_hold_time_per_step = kwargs.get('look_at_image_hold_time_per_step', 10.0)
        self.scan_octagon_num_steps_per_side = kwargs.get('scan_octagon_num_steps_per_side', 5)
        self.look_at_image_yaw_tolerance = kwargs.get('look_at_image_yaw_tolerance', 10.0)
        self.look_at_image_yaw_hold_time = kwargs.get('look_at_image_yaw_hold_time', 2.0)
        self.look_at_image_yaw_timeout = kwargs.get('look_at_image_yaw_timeout', 30.0)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']

        self.blackboard.register_key(key="/mission/octagon_task/items_in_good_bin", access=py_trees.common.Access.READ)
    
    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.node.get_logger().error(f"[{self.name}] Goal rejected by server.")
            self.action_status = ActionStatus.FAILED
    
    def on_server_goal_result(self, goal_success: bool, message: str) -> None:
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to align grabber with object. {message}")
            self.action_status = ActionStatus.FAILED
        
    def update(self):
        global_yaw_image = 0.0 # Fallback yaw
        desired_image = None

        if 0 < self.blackboard.mission.octagon_task.items_in_good_bin <= 2:
            if self.role == 'survey_repair':
                desired_image = self.survey_repair_items_item_labels[self.blackboard.mission.octagon_task.items_in_good_bin - 1]
            elif self.role == 'search_rescue':
                desired_image = self.search_rescue_items_item_labels[self.blackboard.mission.octagon_task.items_in_good_bin - 1]
            else:
                desired_image = self.survey_repair_items_item_labels[0]

            if desired_image in self.blackboard.mission.octagon_task.octagon_images:
                global_yaw_image = self.blackboard.mission.octagon_task.octagon_images[desired_image]
            
            goal = set_global_yaw(
                    yaw_rad=global_yaw_image,
                    tolerance=math.radians(self.look_at_image_yaw_tolerance),
                    hold_time=self.look_at_image_yaw_hold_time,
                    timeout=self.look_at_image_yaw_timeout
                )
            self.navigation_client.send_navigation_goal(goal, self.name, self.on_server_goal_response, self.on_server_goal_result)
        return py_trees.common.Status.SUCCESS

class SpinIfNeeded(py_trees.composites.Sequence):
    def __init__(self, number_item_in_bin_to_check=1, **kwargs):
        super().__init__("SpinIfNeeded", memory=True)
        self.look_at_image_hold_time_per_step = kwargs.get('look_at_image_hold_time_per_step', 2.0)
        self.scan_octagon_num_steps_per_side = kwargs.get('scan_octagon_num_steps_per_side', 5)
        self.blackboard = self.attach_blackboard_client(name="Look and Spin Blackboard")
        
        # Children here are added in setup because we need to read from the blackboard to determine which image to look at
        self.blackboard.register_key(key="/mission/octagon_task/items_in_good_bin", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/mission/octagon_task/octagon_images", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/mission/octagon_task/octagon_images", access=py_trees.common.Access.READ)
        
        # 1. Check if we are good to spin depending on object to check
        check_if_can_spin = CheckIfSpin(number_item_in_bin_to_check=number_item_in_bin_to_check, **kwargs)
        
        # 2. 
        # Number of yaw needed to be done
        do_yaw = vision_behaviours.ScanBehaviour(turn_hold_time_s=self.look_at_image_hold_time_per_step, \
                                                             scan_angle_deg=180, num_steps_per_side=self.scan_octagon_num_steps_per_side)
        
        self.add_children([check_if_can_spin,
                           do_yaw])


class CheckIfSpin(py_trees.composites.behaviour.Behaviour):
    def __init__(self, number_item_in_bin_to_check=1, name="Face Object Related to Role", **kwargs):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name="Check for Spin Blackboard")
        self.number_item_in_bin_to_check = number_item_in_bin_to_check

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.blackboard.register_key(key="/mission/octagon_task/items_in_good_bin", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.mission.octagon_task.items_in_good_bin >= self.number_item_in_bin_to_check:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE