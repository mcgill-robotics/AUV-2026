import py_trees
import math 
import planner.missions.vision_behaviours as vision_behaviours
from controls.goal_helpers import set_depth, move_global, move_robot_centric
from planner.missions.action_status_enum import ActionStatus
from ..mission_behaviour_components import BasicActionBehaviour

# The following classes are defined in a pre-order fashion, meaning that if a class is used in another class, it is defined before the class that uses it.
#  For example, CheckAboveTable is defined before GoAboveTable, since GoAboveTable uses CheckAboveTable as a child behavior.

class GoAboveTable(py_trees.composites.Sequence):
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("Go Above Table", memory=True)

        self.blackboard = self.attach_blackboard_client(name="GoAboveTable Blackboard")

        # 1. Go to a set depth to view the table better. The table is best seen at a shallow depth, since we get to
        # see the top of the table, whereas at a lower depth there is a lot of empty space
        go_shallow_depth = BasicActionBehaviour(
            name="Octagon: Go Shallow Depth",
            goal=set_depth(z=-0.3, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)
        )

        # 2. Navigate to Octagon by looking for the table 
        search_for_table = vision_behaviours.SearchSweepBehaviour(
            target_class="table",
            num_steps=5,
            step_timeout=0.5)
        
        # 3. Navigate a meter away from the table to get a better view of the table
        # With the better view, we get more reliable depth readings of the table, which
        # are needed to go right above it
        go_to_table_1m_away = vision_behaviours.GoNearObject(
            target_class="table",
            target_distance=4.0,
            tolerance_meters=0.2,
            height_offset=None,
            hold_time=3.0
        ) 

        # 3. Navigate a meter away from the table to get a better view of the table
        # With the better view, we get more reliable depth readings of the table, which
        # are needed to go right above it
        go_to_table2 = vision_behaviours.GoNearObject(
            target_class="table",
            target_distance=2.0,
            tolerance_meters=0.2,
            height_offset=None,
            hold_time=3.0
        )

        go_table = GoTable(
            target_distance=0.0,
            tolerance_meters=0.2,
            height_offset=None,
            hold_time=3.0
        )  

        #4 Center around that the table is underneath Dougie
        alignTable = AlignWithTable()

        self.add_children([
            go_shallow_depth,
            search_for_table,
            go_to_table_1m_away,
            go_to_table2,
            go_table,
            alignTable
        ])


class GoTable(py_trees.composites.Sequence):
    def __init__(self, target_distance: float, tolerance_meters: float, height_offset: float, hold_time: float):
        super().__init__("Go table", memory=True)

        #1 Go until table is seen
        go_till_table_seen = GoTillTableSeen(target_distance,
                                tolerance_meters,
                                height_offset,
                                hold_time)
        
        # 2 If overly prematurely seen (not close to goal position), go to pose when table first seen
        self.add_children([go_till_table_seen])




class GoTillTableSeen(py_trees.composites.Parallel):
    def __init__(self, target_distance: float, tolerance_meters: float, height_offset: float, hold_time: float):
        super().__init__("Navigate towards table until it is seen under", policy=py_trees.common.ParallelPolicy.SuccessOnOne())

        # 1 Stop if we can detect the table underneath us
        stop_if_see_table = StopTableBelow()

        # 2 Send an object-map based goal to go to the table
        go_to_table = vision_behaviours.GoNearObject(
            target_class="table",
            target_distance=0.0,
            tolerance_meters=0.1,
            height_offset=None,
            hold_time=3.0
        )

        self.add_children([stop_if_see_table,
                          go_to_table])

class StopTableBelow(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("StopTableBelow")
        self.blackboard = self.attach_blackboard_client(name="Stop if Table below")
        self.initial_pose = None
    
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
        if math.sqrt( (current_position.x - goal_position.x)**2 + ((current_position.y - goal_position.y)**2)) < 0.5:
            py_trees.common.Status.RUNNING 

        # Look if we can see a table in down cam detections
        for detection in self.blackboard.vision.down_cam.detections.detections:
                hypothesis = detection.results[0].hypothesis
                if hypothesis.class_id == "table":
                    self.navigation_client.cancel_navigation_goal() # TODO: Add perhaps a callback to wait till this is done
                    self.blackboard.missions.octagon.pose_at_premature_seen_table = self.blackboard.sensors.pose.pose.position
                    return py_trees.common.Status.SUCCESS

class GoTablePrematurelySeen(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("GoTablePrematurelySeen")
        self.blackboard = self.attach_blackboard_client(name="Stop if Table below")
    
    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)

        self.blackboard.register_key('/missions/octagon/pose_at_premature_seen_table', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/missions/octagon/twist_at_premature_seen_table', access=py_trees.common.Access.READ)

    def initialise(self):
        pass
    
    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.node.get_logger().error(f"[{self.name}] Goal rejected by server.")
            self.action_status = ActionStatus.FAILED
    
    def on_server_goal_result(self, goal_success: bool, message: str) -> None:
        if goal_success:
            # Naive simple controller to get table in the middle 
            self.action_status = ActionStatus.SUCCEEDED
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
        
        # Verify if we are close to the goal, if we are already close to table, it is natural to 
        # see it in the down cam view
        pose_at_premature_seen_table = self.blackboard.missions.octagon.pose_at_premature_seen_table
        if pose_at_premature_seen_table is not None:
            position_at_premature_seen_table = pose_at_premature_seen_table.pose.position
            print(position_at_premature_seen_table)
            goal = move_global(x=position_at_premature_seen_table.x, y=position_at_premature_seen_table.y, do_z=False, tolerance=self.tolerance_meters, hold_time=self.hold_time)
            self.navigation_client.send_navigation_goal(goal, self.name)
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS


class AlignWithTable(py_trees.composites.Selector):
    def __init__(self, position_tolerance: float = 0.2, hold_time: float = 1.0, timeout: float = 30.0, number_of_items_to_check: int = 2):
        super().__init__("Align with Table", memory=False)

        self.blackboard = self.attach_blackboard_client(name="AlignWithTable Blackboard")

        # 1. Check that we are above the table by verifying that we can see at least 2 items on the table (including the table itself
        check_above_table = CheckAboveTable(name="Octagon: Check Above Table", number_of_items_to_check=number_of_items_to_check)

        # 2. If we are not properly above the table, align yourself again 
        # upon success, 
        loop_align_check = LoopTableAlignCheck()

        self.add_children([
            check_above_table,
            loop_align_check
        ])

class CheckAboveTable(py_trees.behaviour.Behaviour):
    def __init__(self, name="Octagon: Check Above Table", number_of_items_to_check: int = 1, discovery_distance: float = 0.3):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name="CheckAboveTable Blackboard")
        self.number_of_items_to_check = number_of_items_to_check
        self.discovery_distance = discovery_distance
        self.acceptable_table_items = ["table", "pill", "nut", "electric", "bandaid"]
        self.seen_items = []
        self.table = False

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)
        
        self.blackboard.register_key('/vision/down_cam/detections', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/mission/octagon_task/view_table', access=py_trees.common.Access.WRITE)
        
    def initialise(self) -> None:
        self.table = None
        self.seen_items = []
            
    def update(self) -> py_trees.common.Status:
        if not hasattr(self.blackboard, 'vision') or self.blackboard.vision.down_cam.detections is None:
            self.node.get_logger().error("Blackboard down cam detection key is not set up or detections is None")
            return py_trees.common.Status.FAILURE
        
        # Verify in detections if there are at least number_of_items_to_check items in the down cam detections 
        # to certify we are above the table (not neccesarily centered)
        for detection in self.blackboard.vision.down_cam.detections.detections:
            if detection.class_id in self.acceptable_table_items:
                if detection.class_id == "table":
                    self.blackboard.mission.octagon_task.view_table = detection
                self.seen_items.append(detection)
                
        if self.seen_items == self.acceptable_table_items:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class LoopTableAlignCheck(py_trees.composites.Sequence):
    def __init__(self, position_tolerance: float = 0.2, hold_time: float = 1.0, timeout: float = 30.0):
        super().__init__("Align with Table", memory=False)

        self.blackboard = self.attach_blackboard_client(name="AlignWithTable Blackboard")

        # 1. Check that we are above the table by verifying that we can see at least 2 items on the table (including the table itself
        check_above_table = CheckAboveTable(number_of_items_to_check=2)

        # 2. If we are not properly above the table, align yourself again 
        # upon success, 
        center_table = CenterTable()

        self.add_children([
            check_above_table,
            center_table
        ])


class CenterTable(py_trees.behaviour.Behaviour):
    def __init__(self, name="Octagon: Check Above Table",  discovery_distance: float = 0.3):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name="CheckAboveTable Blackboard")
        self.acceptable_table_items = ["table", "pill", "nut", "electric", "bandaid"]
        self.number_of_items_to_check = 2
        self.discovery_distance = discovery_distance
        self.camera_width = 640
        self.camera_height = 480

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.navigation_client.client_wait_for_server(timeout_sec=5.0)
        
        self.blackboard.register_key('/vision/down_cam/detections', access=py_trees.common.Access.READ)
        self.blackboard.register_key('/mission/octagon_task/view_table', access=py_trees.common.Access.WRITE)
        
    def initialise(self) -> None:
        self.table = None
        self.seen_items = []

    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.node.get_logger().error(f"[{self.name}] Goal rejected by server.")
            self.action_status = ActionStatus.FAILED
    
    def on_server_goal_result(self, goal_success: bool, message: str) -> None:
        if goal_success:
            # Naive simple controller to get table in the middle 
            self.discovery_distance = 0.8 * self.discovery_distance
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to go above closest bin. {message}")
            self.action_status = ActionStatus.FAILED
            
    def update(self) -> py_trees.common.Status:
        if not hasattr(self.blackboard, 'vision') or self.blackboard.vision.down_cam.detections is None:
            self.node.get_logger().error("Blackboard down cam detection key is not set up or detections is None")
            return py_trees.common.Status.FAILURE
        
        # Verify in detections if there are at least number_of_items_to_check items in the down cam detections 
        # to certify we are above the table (not neccesarily centered)
        for detection in self.blackboard.vision.down_cam.detections.detections:
            hypothesis = detection.results[0].hypothesis
            if hypothesis.class_id in self.acceptable_table_items:
                if hypothesis.class_id == "table":
                    self.table = detection
                self.seen_items.append(detection)
                

        if self.number_of_items_to_check <= 0:
            return py_trees.common.Status.SUCCESS

        if self.table and self.seen_items and len(self.seen_items) < self.number_of_items_to_check:
            self.node.get_logger().warning("Seen table and at least one other item, but not \
            enough items to be sure we are above the table. Continuing to check.")
            
            # Find the general direction of the table by looking at which corner the table BB is in.
            # Move in that general direction to try to see more of it.
            cx = self.table.bbox.center.position.x
            cy = self.table.bbox.center.position.y
            w = self.table.bbox.size_x
            h = self.table.bbox.size_y
            x_min = cx - w / 2.0
            x_max = cx + w / 2.0
            y_min = cy - h / 2.0
            y_max = cy + h / 2.0

            distance = self.discovery_distance
            if x_max >= self.camera_width and y_max >= self.camera_height:
                self.node.get_logger().info("Table is in bottom right corner, moving down and right")
                # move top right
                x = distance * math.cos(math.radians(45))
                y = distance * math.sin(math.radians(45))
            elif x_max >= self.camera_width and y_min <= 0:
                self.node.get_logger().info("Table is in top right corner, moving up and right")
                # move bottom right
                x = distance * math.cos(math.radians(315))
                y = distance * math.sin(math.radians(315))

            elif x_min <= 0 and y_max >= self.camera_height:
                self.node.get_logger().info("Table is in bottom left corner, moving down and left")
                # move top left
                x = distance * math.cos(math.radians(135))
                y = distance * math.sin(math.radians(135))
            elif x_min <= 0 and y_min <= 0:
                self.node.get_logger().info("Table is in top left corner, moving up and left")
                # move bottom left
                x = distance * math.cos(math.radians(225))
                y = distance * math.sin(math.radians(225))

            goal = move_robot_centric(forward=x, sway=y, hold_time=1.0)

            self.navigation_client.send_navigation_goal(goal, self.name)
            self.action_status = ActionStatus.PENDING

            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.RUNNING

class SurfaceAndAlignWithTable(py_trees.composites.Sequence):
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("Surface and Align with Table", memory=True)

        self.blackboard = self.attach_blackboard_client(name="SurfaceAndAlignWithTable Blackboard")

        # 1. Surface inside Octagon
        surface_action = BasicActionBehaviour(
            name="Surface in Octagon", 
            goal=set_depth(z=0.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)
        )

        # 2. Now that we are at the surface, align on one of the table items. 
        align_with_table = AlignWithTable()

