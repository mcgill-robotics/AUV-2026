import py_trees
import math 
import planner.missions.vision_behaviours as vision_behaviours
from controls.goal_helpers import set_depth
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
            goal=set_depth(z=-0.5, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)
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
            target_distance=1.0,
            tolerance_meters=0.2,
            height_offset=0.5,
            hold_time=3.0
        ) 

        #4 Check that the table is underneath Dougie
        check_above_table = CheckAboveTable(number_of_items_to_check=2)

        self.add_children([
            go_shallow_depth,
            search_for_table,
            go_to_table_1m_away,
            check_above_table
        ])

class CheckAboveTable(py_trees.behaviour.Behaviour):
    def __init__(self, name="Octagon: Check Above Table", number_of_items_to_check: int = 1):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name="CheckAboveTable Blackboard")
        self.number_of_items_to_check = number_of_items_to_check
        self.acceptable_table_items = ["table", "pill", "nut", "electric", "bandaid"]

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.blackboard.register_key('/vision/down_cam/detections', access=py_trees.common.Access.READ)
        
    def initialise(self) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        if not hasattr(self.blackboard, 'vision') and self.blackboard.vision.down_cam.detections is None:
            self.node.get_logger().error("Blackboard down cam detection key is not set up or detections is None")
            return py_trees.common.Status.FAILURE
        
        # Verify in detections if there are at least number_of_items_to_check items in the down cam detections 
        # to certify we are above the table (not neccesarily centered)
        for detection in self.blackboard.vision.down_cam.detections:
            if detection.class_id in self.acceptable_table_items:
                self.number_of_items_to_check -= 1

        if self.number_of_items_to_check <= 0:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        

class SurfaceAndAlignWithTable(py_trees.composites.Sequence):
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("Surface and Align with Table", memory=True)

        self.blackboard = self.attach_blackboard_client(name="SurfaceAndAlignWithTable Blackboard")

        # 1. Surface inside Octagon
        surface_action = BasicActionBehaviour(
            name="Surface in Octagon", 
            goal=set_depth(z=0.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)
        )

        # 2. Now that we are at the surface, align Dougie to have the table centered in the camera view. 
        align_with_table = AlignWithTable()


class AlignWithTable(py_trees.behaviour.Behaviour):
    def __init__(self, name="Align with Table", params: dict = None):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name="AlignWithTable Blackboard")
        self.params = params or {}
        # initialize config-backed fields from yaml (fail if missing)
        self.downcam_fov_horizontal = self.params['downcam_fov_horizontal']
        self.downcam_fov_vertical = self.params['downcam_fov_vertical']
        self.camera_width = self.params['downcam_image_width']
        self.camera_height = self.params['downcam_image_height']


    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.blackboard.register_key('/vision/front_cam/detections', access=py_trees.common.Access.READ)

    def initialise(self) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        if not hasattr(self.blackboard, 'vision') and self.blackboard.vision.front_cam.detections is None:
            self.node.get_logger().error("Blackboard front cam detection key is not set up or detections is None")
            return py_trees.common.Status.FAILURE
        
        for detection in self.blackboard.vision.front_cam.detections:
            if detection.class_id == "table":
                # Get the dimensions of the bounding box of table
                bbox_width = detection.bbox.width
                bbox_height = detection.bbox.height

                # The ratio should be 1:1 or around 1:1 since table is square 
                if bbox_width == 0 or bbox_height == 0:
                    self.node.get_logger().error("Bounding box width or height is zero, cannot compute ratio")
                    return py_trees.common.Status.FAILURE
                ratio = bbox_width / bbox_height

                # If the ratio is close to 1, we just need to check that 
                if ratio > 0.8 and ratio < 1.2:
                    return py_trees.common.Status.SUCCESS
                else:
                    self.node.get_logger().error("Table is not properly aligned")
                    return py_trees.common.Status.FAILURE   
