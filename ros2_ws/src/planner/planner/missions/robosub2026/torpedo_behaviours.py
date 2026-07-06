import math
import py_trees
import numpy as np
from typing import Optional, Tuple, Callable
from ..mission_behaviour_components import BasicActionBehaviour
import controls.utils as geometry
from controls.utils import Vector2D
from controls.goal_helpers import set_attitude_quaternion,move_global, move_to_pose
from enum import Enum
from geometry_msgs.msg import Point, Quaternion, Pose

TORPEDO_COUNT = 2

"""
For the sake of standardization, we define board types based on their icons, going clockwise from the top left:
Board Type 1: Fire Firetruck Blood Ambulance
Board Type 2: Blood Ambulance Fire Firetruck
"""
class BoardType(Enum):
    FIRE_TOP_LEFT = 1
    BLOOD_TOP_LEFT = 2
    
class TorpedoSide(Enum):
    RIGHT = 1
    LEFT = 2

class Action(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Action, self).__init__(name)

    def setup(self, **kwargs):
        self.logger.debug(f"Action::setup {self.name}")
        self.blackboard = self.attach_blackboard_client(name=self.name)

    def initialise(self):
        self.logger.debug(f"Action::initialize {self.name}")

    def terminate(self, new_status):
        self.logger.debug(f"Action::terminate {self.name} to {new_status}")

class Condition(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Condition, self).__init__(name)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.logger.debug(f"Condition::setup {self.name}")
        self.blackboard = self.attach_blackboard_client(name=self.name)

    def initialise(self):
        self.logger.debug(f"Condition::initialize {self.name}")

    def update(self):
        self.logger.debug(f"Condition::update {self.name}")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Condition::terminate {self.name} to {new_status}")

def compute_target_in_front_of_point_on_board(point: Vector2D, board_orientation: Quaternion, distance_from_point: float) -> Vector2D:
    """
    Torpedo-specific utility function:
    Compute a target point in front of a point based on the board orientation and desired distance from the object.
    point: position of the vision object in the XY plane
    board_orientation: orientation of the board as a quaternion
    distance_from_point: desired 2D distance from the point to the target point along the normal direction
    """
    # orientation computed by object map is always normal pointing towards the AUV, so we simply take a point at the desired distance in front of the icon along the orientation normal as our target point
    # recover normal from yaw of board orientation
    board_normal = geometry.find_normal_from_quaternion(board_orientation)
    # compute target point at desired distance from point along normal
    return point + board_normal.normalized() * distance_from_point


### Actions:
class DetermineBoardOrientation(Action):
    """
    Action to find the orientation of the board and write it to the blackboard. This will be used for downstream alignment to the board when we are close to the board and can no longer rely on vision to find the orientation of the board.
    This is necessary for orientation in particular since position averaging is already handled by a Kalman Filter in the vision pipeline, whereas orientation is inferred based on the icons on the board, and is more susceptible to noise and outliers.
    n_samples: number of orientation samples to collect before computing mean orientation
    sample_every_n_ticks: how often to sample the board orientation in ticks. For example, if the tree is ticking at 10Hz and sample_every_n_ticks is 10, then we will sample the board orientation every 1 second.
    rejection_threshold: threshold for rejecting outlier orientation samples.
    compare_measurement_with_blackboard: if True, instead of writing to the blackboard, compare the computed mean orientation with the orientation on the blackboard and return SUCCESS if they are within the rejection threshold, and FAILURE if they are not. This can be used as a consistency check if our stable estimate of the board orientation is good enough
    """
    def __init__(
            self,
            n_samples: int = 5,
            sample_every_n_ticks: int = 1,
            rejection_threshold: float = 0.2,
            compare_measurement_with_blackboard: bool = False
        ):
        super().__init__("Determine Board Orientation sampling" + (" once" if n_samples == 1 else f" {n_samples} times") + (f" for consistency validation" if compare_measurement_with_blackboard else ""))
        self.orientation_samples_number = n_samples
        self.sample_rate = sample_every_n_ticks
        self.rejection_threshold = rejection_threshold
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.sample_count = 0
        self.tick_counter = 0
        self.compare_measurement_with_blackboard = compare_measurement_with_blackboard
        self.orientation_samples = []
        
    def setup(self, **kwargs):
        self.node = kwargs['node']
        # self.node.get_logger().info(f"[{self.name}] Setting up with sample_every_n_ticks={self.sample_rate}")
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/board/orientation", access=py_trees.common.Access.WRITE)
        
    def initialise(self):
        self.orientation_samples = []
        self.sample_count = 0
        self.tick_counter = 0
            
    def update(self):
        self.tick_counter += 1
        # self.node.get_logger().info(f"[{self.name}] Tick {self.tick_counter}/{self.sample_rate}. Sample count: {self.sample_count}/{self.orientation_samples_number}")
        if self.tick_counter % self.sample_rate != 0:
            return py_trees.common.Status.RUNNING
        if not self.blackboard.exists("/vision/object_map") or self.blackboard.vision.object_map is None:
            self.node.get_logger().error(f"[{self.name}] No object map available.")
            return py_trees.common.Status.FAILURE
        
        board_vo = None
        for vision_object in self.blackboard.vision.object_map.array:
            if vision_object.label == "board":
                board_vo = vision_object
                break

        if board_vo is None:
            self.node.get_logger().error(f"[{self.name}] Board not found in vision.")
            return py_trees.common.Status.FAILURE
        
        orientation = board_vo.pose.orientation
        if len(self.orientation_samples) > 0:
            if geometry.is_quaternion_outlier(orientation, self.orientation_samples, self.rejection_threshold):
                self.node.get_logger().warning(f"[{self.name}] Rejected outlier orientation sample: {orientation}")
                return py_trees.common.Status.RUNNING
        self.orientation_samples.append(orientation)
        self.sample_count += 1
        self.node.get_logger().debug(f"[{self.name}] Collected orientation sample {self.sample_count}/{self.orientation_samples_number}: {orientation}")
        
        if self.sample_count >= self.orientation_samples_number:
            # Compute mean orientation and write to blackboard
            mean_orientation = geometry.compute_mean_orientation(self.orientation_samples)
            if self.compare_measurement_with_blackboard and self.blackboard.exists("/board/orientation") and self.blackboard.board.orientation is not None:
                if geometry.quaternion_distance(mean_orientation, self.blackboard.board.orientation) > self.rejection_threshold:
                    self.node.get_logger().warning(f"[{self.name}] Computed mean orientation is too different from blackboard orientation.")
                    return py_trees.common.Status.FAILURE
                else:
                    self.node.get_logger().info(f"[{self.name}] Computed mean orientation is consistent with blackboard orientation.")
                    return py_trees.common.Status.SUCCESS
            
            self.blackboard.board.orientation = mean_orientation
            euler_angles_degrees = [math.degrees(angle) for angle in geometry.quaternion_to_euler(mean_orientation)]
            self.node.get_logger().info(f"[{self.name}] Finalized board orientation: {[f'{angle:.2f}' for angle in euler_angles_degrees]} (Euler angles)")
        
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
     
class MoveToFrontOfBoard(BasicActionBehaviour):
    """
    Navigation Action to move to the front of the board based on the orientation found in the FindBoardOrientation action.
    align_to_board: if True, will align to the board orientation, if False, will maintain current orientation
    """
    def __init__(
            self,
            align_to_board: bool,
            distance_from_board: float,
            z_reference: float,
            position_tolerance: float = 0.1,
            orientation_tolerance_rad: float = 0.1,
            timeout: float = 30.0,
            hold_time: float = 0.5,
        ):
        super(MoveToFrontOfBoard, self).__init__("Move to Front of Board and Align")
        self.distance_from_board = distance_from_board
        self.z_reference = z_reference
        self.align_to_board = align_to_board
        self.position_tolerance = position_tolerance
        self.orientation_tolerance_rad = orientation_tolerance_rad
        self.timeout = timeout
        self.hold_time = hold_time
        self.has_failed = False
                
    def setup(self, **kwargs):
        super(MoveToFrontOfBoard, self).setup(**kwargs)
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/board/orientation", access=py_trees.common.Access.READ)

    def initialise(self) -> None:
        self.has_failed = False
        super().initialise()
        if not self.blackboard.exists("/board/orientation") or self.blackboard.board.orientation is None:
            self.node.get_logger().error(f"[{self.name}] No board orientation available on blackboard.")
            self.has_failed = True
            return
        if not self.blackboard.exists("/vision/object_map") or self.blackboard.vision.object_map is None:
            self.node.get_logger().error(f"[{self.name}] No object map available to determine board position.")
            self.has_failed = True
            return
        # get board position from vision
        board_vo = None
        for vision_object in self.blackboard.vision.object_map.array:
            if vision_object.label == "board":
                board_vo = vision_object
                break
        if board_vo is None:
            self.node.get_logger().error(f"[{self.name}] Board not found in vision.")
            self.has_failed = True
            return
        board_center_xy:Vector2D = Vector2D (
            x = board_vo.pose.position.x,
            y = board_vo.pose.position.y
        )
        
        target_xy = compute_target_in_front_of_point_on_board(board_center_xy, self.blackboard.board.orientation, self.distance_from_board)
        
        if self.align_to_board:
            if not self.blackboard.exists("/board/orientation") or self.blackboard.board.orientation is None:
                self.node.get_logger().error(f"[{self.name}] No board orientation available on blackboard.")
                self.has_failed = True
                return
            self.node.get_logger().info(f"[{self.name}] Moving to front of board at {target_xy} and aligning to board orientation.")
            board_orientation = self.blackboard.board.orientation
            # we want the AUV to face the board, since the board faces us, we want to flip the board orientation by 180 degrees in yaw
            target_orientation = geometry.rotate_quaternion(board_orientation, 0, 0, math.pi)
            target_pose:Pose = Pose(
                position=Point(
                    x=target_xy.x,
                    y=target_xy.y,
                    z=self.z_reference
                ),
                orientation=target_orientation
            )
            self.goal = move_to_pose(
                pose=target_pose,
                tolerance=self.position_tolerance,
                angular_tolerance=self.orientation_tolerance_rad,
                hold_time=self.hold_time,
                timeout=self.timeout,
            )
        else:
            self.node.get_logger().info(f"[{self.name}] Moving to front of board at {target_xy} and maintaining current orientation.")

            self.goal = move_global(
                x=target_xy.x,
                y=target_xy.y,
                z=self.z_reference,
                tolerance=self.position_tolerance,
                angular_tolerance=self.orientation_tolerance_rad,
                hold_time=self.hold_time,
                timeout=self.timeout,
            )

    def update(self) -> py_trees.common.Status:
            if self.has_failed:
                return py_trees.common.Status.FAILURE
            return super().update()
    
    def on_server_goal_response(self, goal_response: bool):
        super().on_server_goal_response(goal_response)
        if goal_response:
            self.node.get_logger().info(f"[{self.name}] Moving to front of board.")
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to send goal to move to front of board.")
    def on_server_goal_result(self, goal_success: bool, message: str = "Server result callback received with no message."):
        super().on_server_goal_result(goal_success, message)
        if goal_success:
            self.node.get_logger().info(f"[{self.name}] Successfully moved to front of board. {self.result_message}")
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to move to front of board. {self.result_message}")
        


class AlignToBoard(BasicActionBehaviour):
    """
    Navigation Action to align to the board based on the orientation found in the FindBoardOrientation action.
    """
    
    def __init__(self,
        position_tolerance: float = 0.1,
        orientation_tolerance_rad: float = 0.1,
        timeout: float = 30.0,
        hold_time: float = 0.5,
    ):
        super(AlignToBoard, self).__init__("Align to Board")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.orientation_tolerance_rad = orientation_tolerance_rad
        self.position_tolerance = position_tolerance
        self.timeout = timeout
        self.hold_time = hold_time
        self.has_failed = False
        
    def setup(self, **kwargs):
        super(AlignToBoard, self).setup(**kwargs)
        self.blackboard.register_key(key="/board/orientation", access=py_trees.common.Access.READ)
        
    def initialise(self) -> None:
        self.has_failed = False
        super().initialise()
        if not self.blackboard.exists("/board/orientation") or self.blackboard.board.orientation is None:
            self.node.get_logger().error(f"[{self.name}] No board orientation available on blackboard.")
            self.has_failed = True
            return
        board_orientation = self.blackboard.board.orientation
        # we want the AUV to face the board, since the board faces us, we want to flip the board orientation by 180 degrees in yaw
        target_orientation = geometry.rotate_quaternion(board_orientation, 0, 0, math.pi)
        
        self.goal = set_attitude_quaternion(
            orientation=target_orientation,
            tolerance=self.orientation_tolerance_rad,
            hold_time=self.hold_time,
            timeout=self.timeout,
        )
    
    def update(self) -> py_trees.common.Status:
        if self.has_failed:
            return py_trees.common.Status.FAILURE
        return super().update()
    
    def on_server_goal_response(self, goal_response: bool):
        super().on_server_goal_response(goal_response)
        if goal_response:
            self.node.get_logger().info(f"[{self.name}] Aligning to board.")
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to send goal to align to board.")
    def on_server_goal_result(self, goal_success: bool, message: str = "Server result callback received with no message."):
        super().on_server_goal_result(goal_success, message)
        if goal_success:
            self.node.get_logger().info(f"[{self.name}] Successfully aligned to board. {self.result_message}")
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to align to board. {self.result_message}")

class DetermineBoardType(Action):
    """
    Action to determine board type, either type 1 with fire on top left, or type 2 with blood on top left, based on the relative positions of the icons on the board.
    """
    def __init__(self):
        super().__init__("Determine Board Type")

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.node = kwargs['node']
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/board/type", access=py_trees.common.Access.WRITE)
        
    def update(self):
        # Get board and icon position from blackboard
        if not self.blackboard.exists("/vision/object_map") or self.blackboard.vision.object_map is None:
            self.node.get_logger().error(f"[{self.name}] No object map available.")
            return py_trees.common.Status.FAILURE
        board_vo = None
        blood_vo = None
        fire_vo = None
        ambulance_vo = None
        firetruck_vo = None
        for vision_object in self.blackboard.vision.object_map.array:
            match vision_object.label:
                case "board":
                    board_vo = vision_object
                case "blood":
                    blood_vo = vision_object
                case "fire":
                    fire_vo = vision_object
                case "ambulance":
                    ambulance_vo = vision_object
                case "firetruck":
                    firetruck_vo = vision_object
                case _: # we don't care about other vision objects
                    continue

        if board_vo is None:
            self.node.get_logger().error(f"[{self.name}] Board not found in vision.")
            return py_trees.common.Status.FAILURE
        
        hazard_pair_found = (blood_vo is not None and fire_vo is not None)
        vehicle_pair_found = (ambulance_vo is not None and firetruck_vo is not None)
        
        if not hazard_pair_found and not vehicle_pair_found:
            self.node.get_logger().error(f"[{self.name}] Missing hazard and vehicle vision object pairs in board. Unable to determine board type.")
            return py_trees.common.Status.FAILURE
        
        hazard_board_type: Optional[BoardType]  = None
        vehicle_board_type: Optional[BoardType] = None
        if hazard_pair_found:
            # just to satisfy the type checker, we know these are not None since hazard_pair_found is true
            assert blood_vo is not None and fire_vo is not None
            # fire is above blood for board type 1, blood is above fire for board type 2
            if fire_vo.pose.position.z > blood_vo.pose.position.z:
                hazard_board_type = BoardType.FIRE_TOP_LEFT
            else:
                hazard_board_type = BoardType.BLOOD_TOP_LEFT
                
        if vehicle_pair_found:
            # just to satisfy the type checker, we know these are not None since vehicle_pair_found is true
            assert ambulance_vo is not None and firetruck_vo is not None
            # firetruck is above ambulance for board type 1, ambulance is above firetruck for board type 2
            if firetruck_vo.pose.position.z > ambulance_vo.pose.position.z:
                vehicle_board_type = BoardType.FIRE_TOP_LEFT
            else:
                vehicle_board_type = BoardType.BLOOD_TOP_LEFT

        if hazard_pair_found and vehicle_pair_found and hazard_board_type != vehicle_board_type:
            self.node.get_logger().error(f"[{self.name}] Inconsistent board type between hazard and vehicle vision objects.")
            return py_trees.common.Status.FAILURE
        else:
            assert hazard_board_type is not None or vehicle_board_type is not None
            board_type: Optional[BoardType] = hazard_board_type if hazard_pair_found else vehicle_board_type
            # just to satisfy the type checker, we know board_type is not None since at least one of the pairs is found
            assert board_type is not None
            self.blackboard.board.type = board_type.value
            self.node.get_logger().info(f"[{self.name}] Detected board type: {board_type.name}")

        return py_trees.common.Status.SUCCESS
    
class CheckBoardType(Condition):
    """
    Condition to check if the board type on the blackboard matches the expected board type.
    """
    def __init__(self, board_type: BoardType):
        super().__init__(f"Check Board Type {board_type.name}")
        self.expected_board_type = board_type
        
    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.blackboard.register_key(key="/board/type", access=py_trees.common.Access.READ)
        
    def update(self):
        if not self.blackboard.exists("/board/type") or self.blackboard.board.type is None:
            self.node.get_logger().error(f"[{self.name}] No board type available on blackboard.")
            return py_trees.common.Status.FAILURE
        if self.blackboard.board.type == self.expected_board_type.value:
            self.node.get_logger().info(f"[{self.name}] Board type {self.blackboard.board.type} matches expected {self.expected_board_type.name}.")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info(f"[{self.name}] Board type {self.blackboard.board.type} does not match expected {self.expected_board_type.name}.")
            return py_trees.common.Status.FAILURE

class BoardIcon(Enum):
    FIRE = "fire"
    BLOOD = "blood"
    AMBULANCE = "ambulance"
    FIRETRUCK = "firetruck"
    
class MoveToFrontOfIcon(BasicActionBehaviour):
    """
    Navigation Action to move to the front of a specific icon on the board based on the vision information.
    Alignment based on the orientation found in the FindBoardOrientation action.
    use_icon_z: if True, use the z position of the icon as the z reference for the navigation goal, if False, use a fixed z reference provided by z_reference parameter.
    distance_from_icon: 2D distance from the icon to the target point in front of the icon, if None, will maintain current distance from the icon and just move in front of it
    """
    def __init__(
            self,
            target_icon: BoardIcon,
            distance_from_icon: Optional[float] = None,
            use_icon_z: bool = True,
            z_reference: Optional[float] = None,
            position_tolerance: float = 0.1,
            orientation_tolerance_rad: float = 0.1,
            timeout: float = 30.0,
            hold_time: float = 0.5,
        ):
        super(MoveToFrontOfIcon, self).__init__(f"Move in Front of {target_icon.value.capitalize()} " + (f"at {distance_from_icon}m away" if distance_from_icon is not None else "maintaining current distance"))
        self.target_icon = target_icon
        self.distance_from_icon = distance_from_icon
        if not use_icon_z and z_reference is None:
            raise ValueError("z_reference must be provided when use_icon_z is False")
        self.use_icon_z = use_icon_z
        self.z_reference = z_reference
        self.compute_distance_from_icon = distance_from_icon is None
        self.position_tolerance = position_tolerance
        self.orientation_tolerance_rad = orientation_tolerance_rad
        self.timeout = timeout
        self.hold_time = hold_time
        self.has_failed = False
        
    def setup(self, **kwargs):
        super(MoveToFrontOfIcon, self).setup(**kwargs)
        if self.compute_distance_from_icon:
            self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/board/orientation", access=py_trees.common.Access.READ)
        
    def initialise(self) -> None:
        self.has_failed = False
        super().initialise()
        if not self.blackboard.exists("/board/orientation") or self.blackboard.board.orientation is None:
            self.node.get_logger().error(f"[{self.name}] No board orientation available on blackboard.")
            self.has_failed = True
            return
        if not self.blackboard.exists("/vision/object_map") or self.blackboard.vision.object_map is None:
            self.node.get_logger().error(f"[{self.name}] No object map available to determine icon position.")
            self.has_failed = True
            return
        
        # get target icon position from vision
        target_icon_vo = None
        for vision_object in self.blackboard.vision.object_map.array:
            if vision_object.label == self.target_icon.value:
                target_icon_vo = vision_object
                break
        if target_icon_vo is None:
            self.node.get_logger().error(f"[{self.name}] Target icon {self.target_icon.value} not found in vision.")
            self.has_failed = True
            return
        
        target_2d_position = Vector2D.from_point(target_icon_vo.pose.position)
        
        if self.compute_distance_from_icon:
            if not self.blackboard.exists("/sensors/pose") or self.blackboard.sensors.pose is None:
                self.node.get_logger().warn(f"[{self.name}] AUV pose not available to determine distance from icon.")
                self.has_failed = True
                return
            auv_2d_position = Vector2D.from_point(self.blackboard.sensors.pose.pose.position)
            self.distance_from_icon = geometry.plane_point_distance(
                point=target_2d_position,
                plane_point=auv_2d_position,
                plane_normal=geometry.find_normal_from_quaternion(self.blackboard.board.orientation)
            )
        # self.compute_distance_from_icon should guarantee that self.distance_from_icon is not None, but we add an assertion here to satisfy the type checker and catch any potential bugs
        assert self.distance_from_icon is not None
        self.node.get_logger().info(f"[{self.name}] Distance from AUV to icon: {self.distance_from_icon:.2f}m")
        target_xy = compute_target_in_front_of_point_on_board(target_2d_position, self.blackboard.board.orientation, self.distance_from_icon)
        
        target_z = target_icon_vo.pose.position.z if self.use_icon_z else self.z_reference
        
        self.goal = move_global(
            x=target_xy.x,
            y=target_xy.y,
            z=target_z,
            tolerance=self.position_tolerance,
            angular_tolerance=self.orientation_tolerance_rad,
            hold_time=self.hold_time,
            timeout=self.timeout,
        )
        
    def update(self) -> py_trees.common.Status:
        if self.has_failed:
            return py_trees.common.Status.FAILURE
        return super().update()
    
    def on_server_goal_response(self, goal_response: bool):
        super().on_server_goal_response(goal_response)
        if goal_response:
            self.node.get_logger().info(f"[{self.name}] Moving to front of icon {self.target_icon.value}.")
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to send goal to move to front of icon {self.target_icon.value}.")
    
    def on_server_goal_result(self, goal_success: bool, message: str = "Server result callback received with no message."):
        super().on_server_goal_result(goal_success, message)
        if goal_success:
            self.node.get_logger().info(f"[{self.name}] Successfully moved to front of icon {self.target_icon.value}. {self.result_message}")
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to move to front of icon {self.target_icon.value}. {self.result_message}")

class DetermineIconPosition(Action):
    """
    Action to determine the position of a specific icon on the board and save it to the blackboard.
    minimum_ticks: 
    """
    def __init__(self, target_icon: BoardIcon, attempts : int = 1):
        super().__init__("Determine Icon Position: " + target_icon.value)
        self.target_icon = target_icon
        self.attempts = attempts
        self.current_attempt = 0

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.node = kwargs['node']
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/torpedo/icon_position", access=py_trees.common.Access.WRITE)
        
    def initialise(self):
        super().initialise()
        self.current_attempt = 0

    def update(self):
        if not self.blackboard.exists("/vision/object_map") or self.blackboard.vision.object_map is None:
            self.node.get_logger().error(f"[{self.name}] No object map available.")
            return py_trees.common.Status.FAILURE
        
        target_icon_vo = None
        for vision_object in self.blackboard.vision.object_map.array:
            if vision_object.label == self.target_icon.value:
                target_icon_vo = vision_object
                break
        
        if target_icon_vo is None:
            self.current_attempt += 1
            if self.current_attempt < self.attempts:
                self.node.get_logger().warn(f"[{self.name}] Target icon {self.target_icon.value} not found in vision. Retrying...")
                return py_trees.common.Status.RUNNING
            else:
                self.node.get_logger().error(f"[{self.name}] Target icon {self.target_icon.value} not found in vision.")
                return py_trees.common.Status.FAILURE
        
        self.blackboard.torpedo.icon_position = target_icon_vo.pose.position
        self.node.get_logger().info(f"[{self.name}] Saved position of {self.target_icon.value} icon: {self.blackboard.torpedo.icon_position}")
        return py_trees.common.Status.SUCCESS
    
    
class DetermineOffsetToHole(Action):
    """
    Action to determine the offset from the icon to the hole and save it to the blackboard, based on board time and icon.
    """

    def __init__(self, 
            icon: BoardIcon, 
            distance_from_board: float,
            icon_to_nearest_hole: dict[BoardType, dict[BoardIcon, Tuple[float,float,float]]]
        ):
        super().__init__(name=f"Resolve Offset for {icon.name} @ {distance_from_board}m")
        self.icon = icon
        self.distance_from_board = distance_from_board
        self.icon_to_nearest_hole = icon_to_nearest_hole
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.node = kwargs['node']
        self.blackboard.register_key("/board/type", access=py_trees.common.Access.READ)
        self.blackboard.register_key("/torpedo/offset_to_hole", access=py_trees.common.Access.WRITE)
    
    def update(self):
        if not self.blackboard.exists("/board/type") or self.blackboard.board.type is None:
            self.node.get_logger().error(f"[{self.name}] No board type available on blackboard.")
            return py_trees.common.Status.FAILURE
        board_type = BoardType(self.blackboard.get("/board/type"))
        self.node.get_logger().info(f"[{self.name}] Board type: {board_type}, Icon: {self.icon.value}, Distance from board: {self.distance_from_board}")
        offset_to_hole = self.icon_to_nearest_hole[board_type][self.icon]
        if offset_to_hole is None:
            self.node.get_logger().error(f"[{self.name}] Offset to hole not found for icon {self.icon.value} and board type {board_type.value}")
            return py_trees.common.Status.FAILURE
        self.blackboard.set("/torpedo/offset_to_hole", (
            offset_to_hole[0] - self.distance_from_board,
            offset_to_hole[1],
            offset_to_hole[2],
        ))
        return py_trees.common.Status.SUCCESS
    
class AlignTorpedoToHole(BasicActionBehaviour):
    """
    Navigation Action to align the torpedo to the hole.
    """
    def __init__(
            self,
            auv_to_torpedos : dict[TorpedoSide, Tuple[float,float,float]],
            forward_trajectory : np.polynomial.Polynomial,
            lateral_trajectory : np.polynomial.Polynomial,
            vertical_trajectory : np.polynomial.Polynomial,
            position_tolerance: float = 0.1,
            orientation_tolerance_rad: float = 0.1,
            timeout: float = 30.0,
            hold_time: float = 0.5
        ):
        super(AlignTorpedoToHole, self).__init__(f"Align Torpedo to Hole")
        self.lateral_trajectory = lateral_trajectory.copy()
        self.vertical_trajectory = vertical_trajectory.copy()
        self.forward_trajectory = forward_trajectory.copy()
        self.auv_to_torpedos = auv_to_torpedos
        
        self.position_tolerance = position_tolerance
        self.orientation_tolerance_rad = orientation_tolerance_rad
        self.timeout = timeout
        self.hold_time = hold_time
        self.has_failed = False
        
        
    def setup(self, **kwargs):
        super(AlignTorpedoToHole, self).setup(**kwargs)
        self.blackboard.register_key(key="/board/orientation", access=py_trees.common.Access.READ)
        # whether to orient left or right torpedo
        self.blackboard.register_key(key="/torpedo/count", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/torpedo/icon_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/torpedo/trajectory_time", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/torpedo/offset_to_hole", access=py_trees.common.Access.READ)
        
    def initialise(self) -> None:
        self.has_failed = False
        super().initialise()
        
        if not self.blackboard.exists("/board/orientation") or self.blackboard.board.orientation is None:
            self.node.get_logger().error(f"[{self.name}] No board orientation available on blackboard.")
            self.has_failed = True
            return 
        if not self.blackboard.exists("/torpedo/icon_position") or self.blackboard.torpedo.icon_position is None:
            self.node.get_logger().error(f"[{self.name}] No torpedo icon position available on blackboard.")
            self.has_failed = True
            return 
        if not self.blackboard.exists("/torpedo/count") or self.blackboard.torpedo.count is None:
            self.node.get_logger().error(f"[{self.name}] No torpedo count available on blackboard.")
            self.has_failed = True
            return
        
        match self.blackboard.torpedo.count:
            case 0:
                self.node.get_logger().error(f"[{self.name}] No torpedo detected. Alignment is redundant with absence of torpedo payload.")
                self.has_failed = True
                return
            case 1:
                side = TorpedoSide.LEFT
            case 2:
                side = TorpedoSide.RIGHT
            case _:
                self.node.get_logger().error(f"[{self.name}] Invalid torpedo count {self.blackboard.torpedo.count}. Expected 1 or 2.")
                self.has_failed = True
                return
        self.auv_to_torpedo = self.auv_to_torpedos[side]
                        
        self.node.get_logger().info(f"[{self.name}] Trajectory coeffs: Forward: {self.forward_trajectory.coef}, Lateral: {self.lateral_trajectory.coef}, Vertical: {self.vertical_trajectory.coef}")
        # after alignment, torpedo launch should be aligned with the hole
        # to know how much to shift back by, we need to know the trajectory time such that forward trajectory is equal to the hole offset, and then we can use that time to find the lateral and vertical offsets
        if not self.blackboard.exists("/torpedo/offset_to_hole") or self.blackboard.torpedo.offset_to_hole is None:
            self.node.get_logger().error(f"[{self.name}] No torpedo offset to hole available on blackboard. Unable to align torpedo to hole.")
            self.has_failed = True
            return
        to_hole_offset = self.blackboard.torpedo.offset_to_hole
        self.forward_trajectory.coef[0] += to_hole_offset[0]
        self.node.get_logger().info(f"[{self.name}] Adjusted trajectories coeffs for hole offset: F {self.forward_trajectory.coef}, L {self.lateral_trajectory.coef}, V {self.vertical_trajectory.coef}")
        forward_roots = sorted(r.real for r in self.forward_trajectory.roots() if np.isreal(r))
        
        # 0 is always solution since all trajectories start at 0
        if len(forward_roots) < 1:
            self.node.get_logger().error(f"[{self.name}] Torpedo forward trajectory does not have roots. Forward roots: {forward_roots}")
            self.has_failed = True
            return
        
        self.node.get_logger().info(f"[{self.name}] Torpedo forward trajectory intersects with hole at roots: {forward_roots}. F values: {[self.forward_trajectory(root) for root in forward_roots]}. L values: {[self.lateral_trajectory(root) for root in forward_roots]} V values: {[self.vertical_trajectory(root) for root in forward_roots]}")

        trajectory_time = forward_roots[0]
        backward_offset = self.forward_trajectory(trajectory_time) 
        rightward_offset = self.lateral_trajectory(trajectory_time)
        downward_offset = self.vertical_trajectory(trajectory_time) 
        
        self.node.get_logger().info(f"[{self.name}] Torpedo trajectory time to reach hole: {trajectory_time:.2f}s. Corresponding offsets: Backward: {backward_offset:.2f}m, Rightward: {rightward_offset:.2f}m, Downward: {downward_offset:.2f}m")
        
        self.blackboard.torpedo.trajectory_time = trajectory_time
        target_orientation = geometry.rotate_quaternion(self.blackboard.board.orientation, 0, 0, math.pi)
        
        offset_vector = Point(
            x= to_hole_offset[0] - self.auv_to_torpedo[0] + backward_offset,
            y= to_hole_offset[1] - self.auv_to_torpedo[1] - rightward_offset,
            z= to_hole_offset[2] - self.auv_to_torpedo[2] - downward_offset
        )
        
        self.node.get_logger().info(f"[{self.name}] Aligning torpedo to hole offset from icon position: {offset_vector}")
        # move from current position to hole position, then move such that torpedo launch is at the AUV CoM, then move back by the forward offset to align the torpedo with the hole
        goal_offset = geometry.rotate_3d_vector(
            q=target_orientation,
            vector=offset_vector
        )
        
        self.node.get_logger().info(f"[{self.name}] Rotated offset vector to hole in board frame: {goal_offset}")
        
        goal_position_x = self.blackboard.torpedo.icon_position.x + goal_offset.x
        goal_position_y = self.blackboard.torpedo.icon_position.y + goal_offset.y
        goal_position_z = self.blackboard.torpedo.icon_position.z + goal_offset.z
        
        self.node.get_logger().info(f"[{self.name}] Moving to position in front of hole: ({goal_position_x:.2f}, {goal_position_y:.2f}, {goal_position_z:.2f}) with backward offset: {backward_offset:.2f}m")
        
        self.goal = move_global(
            x=goal_position_x,
            y=goal_position_y,
            z=goal_position_z,
            tolerance=self.position_tolerance,
            angular_tolerance=self.orientation_tolerance_rad,
            hold_time=self.hold_time,
            timeout=self.timeout,
        )
        
    def update(self) -> py_trees.common.Status:
        if self.has_failed:
            return py_trees.common.Status.FAILURE
        return super().update()
    
    def on_server_goal_response(self, goal_response: bool):
        super().on_server_goal_response(goal_response)
        if goal_response:
            self.node.get_logger().info(f"[{self.name}] Aligning torpedo to hole.")
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to send goal to align torpedo to hole.")
    def on_server_goal_result(self, goal_success: bool, message: str = "Server result callback received with no message."):
        super().on_server_goal_result(goal_success, message)
        if goal_success:
            self.node.get_logger().info(f"[{self.name}] Successfully aligned torpedo to hole. {self.result_message}")
        else:
            self.node.get_logger().error(f"[{self.name}] Failed to align torpedo to hole. {self.result_message}")

class FireState(Enum):
    PAUSING = 0
    FIRING = 1
   
class FireTorpedo(Action):
    """
    Action to fire a torpedo.
    launch_function: callaback that takes a TorpedoSide and fires the corresponding torpedo
    firing_buffer_time: time in seconds to wait before and after firing to ensure the torpedo is launched properly and the AUV is stable
    """
    def __init__(self, launch_function: Callable[[TorpedoSide], None], firing_buffer_time: float):
        super().__init__("Fire Torpedo")
        self.launch_function = launch_function
        self.firing_buffer_time = firing_buffer_time
        self.has_fired: bool = False
        self.fire_state: FireState = FireState.PAUSING
        # Pause tracking        
        self.pause_start_time: float = 0.0
        self.is_pausing: bool = False
        self.pause_time_total: float = self.firing_buffer_time

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.blackboard.register_key(key="/torpedo/count", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/torpedo/trajectory_time", access=py_trees.common.Access.READ)
        self.node = kwargs['node']
        
    def initialise(self):
        super().initialise()
        self.has_fired = False
        self.pause_start_time = 0.0
        self.is_pausing = False
        self.pause_time_total = self.firing_buffer_time
        self.fire_state = FireState.PAUSING

    def update(self):
        match self.fire_state:
            case FireState.PAUSING:
                # pause before firing to ensure AUV is stable
                elapsed = (self.node.get_clock().now().nanoseconds / 1e9) - self.pause_start_time
                self.pause_time_total = self.firing_buffer_time
                if self.has_fired:
                    if not self.blackboard.exists("/torpedo/trajectory_time") or self.blackboard.torpedo.trajectory_time is None:
                        self.node.get_logger().error(f"[{self.name}] No torpedo trajectory time found on blackboard. Assuming no trajectory delay after firing.")
                    else:
                        self.pause_time_total += self.blackboard.torpedo.trajectory_time
                if not self.is_pausing:
                    self.node.get_logger().info(f"[{self.name}] Pausing for {self.firing_buffer_time:.2f}s.")
                    self.pause_start_time = self.node.get_clock().now().nanoseconds / 1e9
                    self.is_pausing = True
                    return py_trees.common.Status.RUNNING
                elif elapsed < self.pause_time_total:
                    return py_trees.common.Status.RUNNING
                else:
                    if not self.has_fired:
                        self.node.get_logger().info(f"[{self.name}] Firing torpedo after {elapsed:.2f}s pause.")
                        self.fire_state = FireState.FIRING
                        return py_trees.common.Status.RUNNING
                    else:
                        self.node.get_logger().info(f"[{self.name}] Finished firing torpedo after {elapsed:.2f}s pause.")
                        return py_trees.common.Status.SUCCESS                        
            case FireState.FIRING:
                if not self.blackboard.exists("/torpedo/count") or self.blackboard.torpedo.count is None:
                    self.node.get_logger().error(f"[{self.name}] No torpedo count available on blackboard.")
                    return py_trees.common.Status.FAILURE
                if self.blackboard.torpedo.count <= 0:
                    self.node.get_logger().error(f"[{self.name}] No torpedos left. Cannot fire.")
                    return py_trees.common.Status.FAILURE
                match self.blackboard.torpedo.count:
                    case 1:
                        side = TorpedoSide.LEFT
                    case 2:
                        side = TorpedoSide.RIGHT
                    case _:
                        self.node.get_logger().error(f"[{self.name}] Invalid torpedo count: {self.blackboard.torpedo.count}. Cannot determine which torpedo to fire.")
                        return py_trees.common.Status.FAILURE
                self.launch_function(side)
                self.blackboard.torpedo.count -= 1
                self.node.get_logger().info(f"[{self.name}] Fired torpedo. New count: {self.blackboard.torpedo.count}")
                self.has_fired = True
                self.fire_state = FireState.PAUSING
                return py_trees.common.Status.RUNNING
