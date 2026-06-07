import math
import py_trees
import py_trees_ros
from time import sleep
from ..action_status_enum import ActionStatus
import controls.utils as geometry
from controls.utils import Vector2D
from controls.goal_helpers import move_to_pose
from enum import Enum
from geometry_msgs.msg import Pose, Quaternion

class Season(Enum):
    SPRING = 1
    SUMMER = 2
    AUTUMN = 3
    WINTER = 4
TORPEDO_COUNT = 2

"""
For the sake of standardization, we define board types based on their icons, going clockwise from the top left:
Board Type 1: Fire Firetruck Blood Ambulance
Board Type 2: Blood Ambulance Fire Firetruck
"""
class BoardType(Enum):
    FIRE_TOP_LEFT = 1
    BLOOD_TOP_LEFT = 2

class Action(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Action, self).__init__(name)

    def setup(self, **kwargs):
        self.logger.debug(f"Action::setup {self.name}")
        self.blackboard = self.attach_blackboard_client(name=self.name)

    def initialise(self):
        self.logger.debug(f"Action::initialize {self.name}")

    def update(self):
        self.logger.debug(f"Action::update {self.name}")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Action::terminate {self.name} to {new_status}")

class Condition(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Condition, self).__init__(name)

    def setup(self, **kwargs):
        self.logger.debug(f"Condition::setup {self.name}")
        self.blackboard = self.attach_blackboard_client(name=self.name)

    def initialise(self):
        self.logger.debug(f"Condition::initialize {self.name}")

    def update(self):
        self.logger.debug(f"Condition::update {self.name}")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Condition::terminate {self.name} to {new_status}")

class TorpedoNodeFactory:
    """
    Handles reusable subtrees for Torpedo Behaviour Tree 
    """
    
    def make_node_align_board(self, suffix="")->py_trees.composites.Sequence:
        """
        Node to orient to board, match z point, and move to xy point. 
        Sequence: ensure all actions are done in a specific order.
        This is a sub tree that will be called in the check align board selector node.
        
        returns: py_trees.composites.Sequence
        """
        node_align_board = py_trees.composites.Sequence(f"align_with_board_{suffix}", memory=True)
        orient_with_middle = Action(f"orient_with_middle_{suffix}")
        match_z_point = Action(f"match_z_point_{suffix}")
        move_to_xy = Action(f"move_to_xy_{suffix}")
        node_align_board.add_children(
            [
                orient_with_middle, 
                match_z_point, 
                move_to_xy
            ]
        )
        return node_align_board
    
    def make_node_check_align_board(self, suffix="")->py_trees.composites.Selector:
        """
        Node to check if the board is in view.
        Selector: if the board is in view, we are done, if not, we need to orient to the board (call the align board sequence node)
        
        returns: py_trees.composites.Selector
        """
        node_check_align_board = py_trees.composites.Selector(f"check_align_board_{suffix}", memory=False)
        check_board_in_view = Condition(f"check_board_in_view_{suffix}")
        node_check_align_board.add_children(
            [
                check_board_in_view, 
                self.make_node_align_board(suffix)
            ]
        )
        return node_check_align_board
    

    def make_node_go_to_animal(self, suffix="")->py_trees.composites.Sequence:
        """
        Node to make Dougie go to the animal.
        Sequence: ensure board is in frame, then align xfeat.

        retruns: py_trees.composites.Sequence
        """
        node_go_to_animal = py_trees.composites.Sequence(f"go_to_animal_{suffix}", memory=True)
        align_xfeat = Action(f"align_xfeat_{suffix}")
        node_go_to_animal.add_children(
            [
                align_xfeat,
                self.make_node_check_align_board(suffix)
            ]
        )
        return node_go_to_animal


### Actions:
class FindBoardOrientation(Action):
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
        super().__init__("Find Board Orientation sampling" + (" once" if n_samples == 1 else f" {n_samples} times") + (f" for consistency validation" if compare_measurement_with_blackboard else ""))
        self.orientation_samples_number = n_samples
        self.sample_rate = sample_every_n_ticks
        self.rejection_threshold = rejection_threshold
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.sample_count = 0
        self.tick_counter = 0
        self.compare_measurement_with_blackboard = compare_measurement_with_blackboard
        self.orientation_samples = []
        
    def setup(self, **kwargs):
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/board/orientation", access=py_trees.common.Access.WRITE)
        
    def initialise(self):
        self.orientation_samples = []
        self.sample_count = 0
            
    def update(self):
        self.tick_counter += 1
        if self.tick_counter % self.sample_rate != 0:
            return py_trees.common.Status.RUNNING
        if not hasattr(self.blackboard, 'vision') or self.blackboard.vision.object_map is None:
            self.logger.error(f"[{self.name}] No object map available.")
            return py_trees.common.Status.FAILURE
        
        board_vo = None
        for vision_object in self.blackboard.vision.object_map.array:
            if vision_object.label == "board":
                board_vo = vision_object
                break

        if board_vo is None:
            self.logger.error(f"[{self.name}] Board not found in vision.")
            return py_trees.common.Status.FAILURE
        
        orientation = board_vo.pose.orientation
        if len(self.orientation_samples) > 0:
            if geometry.is_quaternion_outlier(orientation, self.orientation_samples, self.rejection_threshold):
                self.logger.warning(f"[{self.name}] Rejected outlier orientation sample: {orientation}")
                return py_trees.common.Status.RUNNING
        self.orientation_samples.append(orientation)
        self.sample_count += 1
        self.logger.info(f"[{self.name}] Collected orientation sample {self.sample_count}/{self.orientation_samples_number}: {orientation}")
        
        if self.sample_count >= self.orientation_samples_number:
            # Compute mean orientation and write to blackboard
            mean_orientation = geometry.compute_mean_orientation(self.orientation_samples)
            if self.compare_measurement_with_blackboard and hasattr(self.blackboard, 'board') and self.blackboard.board.orientation is not None:
                if geometry.quaternion_distance(mean_orientation, self.blackboard.board.orientation) > self.rejection_threshold:
                    self.logger.warning(f"[{self.name}] Computed mean orientation is too different from blackboard orientation.")
                    return py_trees.common.Status.FAILURE
                else:
                    self.logger.info(f"[{self.name}] Computed mean orientation is consistent with blackboard orientation.")
                    return py_trees.common.Status.SUCCESS
            
            self.blackboard.board.orientation = mean_orientation
            self.logger.info(f"[{self.name}] Finalized board orientation: {mean_orientation}")
        
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
class MoveToFrontOfBoardAndAlign(Action):
    """
    Action to align to the board based on the orientation found in the FindBoardOrientation action.
    """
    def __init__(
            self,
            distance_from_board: float,
            z_reference: float,
            position_tolerance: float = 0.1,
            orientation_tolerance_rad: float = 0.1,
            timeout: float = 30.0,
            hold_time: float = 0.5,
        ):
        super().__init__("Move to Front of Board and Align")
        self.distance_from_board = distance_from_board
        self.z_reference = z_reference
        self.position_tolerance = position_tolerance
        self.orientation_tolerance_rad = orientation_tolerance_rad
        self.timeout = timeout
        self.hold_time = hold_time
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.result_message: str = ""
        
        self.action_status:ActionStatus = ActionStatus.NOT_SENT
        self.sent_goal = False
        
    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['shared_nav_client']
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/board/orientation", access=py_trees.common.Access.READ)
        self.result_message: str = ""

    
    def update(self):
        match self.action_status:
            case ActionStatus.SUCCEEDED:
                self.node.get_logger().info(f"[{self.name}] Successfully moved to front of board and aligned. {self.result_message}")
                return py_trees.common.Status.SUCCESS
            case ActionStatus.FAILED:
                self.node.get_logger().error(f"[{self.name}] Failed to move to front of board and align. {self.result_message}")
                return py_trees.common.Status.FAILURE
            case ActionStatus.PENDING:
                return py_trees.common.Status.RUNNING
            case ActionStatus.NOT_SENT:  
                if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
                    self.node.get_logger().warn(f"[{self.name}] Waiting for /sensors/pose to determine AUV yaw.")
                    return py_trees.common.Status.RUNNING
                if not hasattr(self.blackboard, 'board') or self.blackboard.board.orientation is None:
                    self.node.get_logger().error(f"[{self.name}] No board orientation available on blackboard.")
                    return py_trees.common.Status.FAILURE
                if not hasattr(self.blackboard, 'vision') or self.blackboard.vision.object_map is None:
                    self.node.get_logger().error(f"[{self.name}] No object map available to determine board position.")
                    return py_trees.common.Status.FAILURE
                # get board position from vision
                board_vo = None
                for vision_object in self.blackboard.vision.object_map.array:
                    if vision_object.label == "board":
                        board_vo = vision_object
                        break
                if board_vo is None:
                    self.node.get_logger().error(f"[{self.name}] Board not found in vision.")
                    return py_trees.common.Status.FAILURE
                
                # get board orientation from blackboard
                board_orientation = self.blackboard.board.orientation
                
                # shift by half of size to get board center
                board_center_xy:Vector2D = Vector2D (
                    x = board_vo.pose.position.x + 0.5 * board_vo.size.x,
                    y = board_vo.pose.position.y + 0.5 * board_vo.size.y
                )
                
                ### orientation computed by object map is always normal pointing towards the AUV, so we simply take a point at the desired distance in front of the board along the orientation normal as our target point
                # recover normal from yaw of board orientation
                board_normal = geometry.find_normal_to_quaternion(board_orientation)
                
                # compute target point at desired distance from board along normal
                target_xy = board_center_xy + board_normal.normalized() * self.distance_from_board
                
                target_pose = Pose()
                target_pose.position.x = target_xy.x
                target_pose.position.y = target_xy.y
                target_pose.position.z = self.z_reference
                # we want the AUV to face the board, since the board faces us, we want to flip the board orientation by 180 degrees in yaw
                board_yaw = geometry.rotate_quaternion(board_orientation, 0, 0, math.pi)
                target_pose.orientation = board_yaw

                goal = move_to_pose(
                    pose=target_pose,
                    tolerance=self.position_tolerance,
                    angular_tolerance=self.orientation_tolerance_rad,
                    timeout=self.timeout,
                    hold_time=self.hold_time
                )
                
                self.navigation_client.send_navigation_goal(goal, self.name, self.on_server_goal_response, self.on_server_goal_result)
                self.action_status = ActionStatus.PENDING
                return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS
    
    def on_server_goal_response(self, goal_response: bool):
        if not goal_response:
            self.action_status = ActionStatus.FAILED

    def on_server_goal_result(self, goal_success: bool, message: str):
        self.result_message = message
        if goal_success:
            self.action_status = ActionStatus.SUCCEEDED
        else:
            self.action_status = ActionStatus.FAILED
                
class DetermineBoardType(Action):
    """
    
    """
    def __init__(self, distance_from_board):
        super().__init__("Move and Align to Board")
        self.distance_from_board = distance_from_board

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.node = kwargs['node']
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/board/type", access=py_trees.common.Access.WRITE)
        
    def update(self):
        # Get board and icon position from blackboard
        if not hasattr(self.blackboard, 'vision') or self.blackboard.vision.object_map is None:
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
        

        if hazard_pair_found:
            # fire is above blood for board type 1, blood is above fire for board type 2
            if fire_vo.pose.position.z > blood_vo.pose.position.z:
                hazard_board_type: BoardType = BoardType.FIRE_TOP_LEFT
            else:
                hazard_board_type: BoardType = BoardType.BLOOD_TOP_LEFT
                
        if vehicle_pair_found:
            # firetruck is above ambulance for board type 1, ambulance is above firetruck for board type 2
            if firetruck_vo.pose.position.z > ambulance_vo.pose.position.z:
                vehicle_board_type: BoardType = BoardType.FIRE_TOP_LEFT
            else:
                vehicle_board_type: BoardType = BoardType.BLOOD_TOP_LEFT
                
        if hazard_pair_found and vehicle_pair_found and hazard_board_type != vehicle_board_type:
            self.node.get_logger().error(f"[{self.name}] Inconsistent board type between hazard and vehicle vision objects.")
            return py_trees.common.Status.FAILURE
        else:
            board_type = hazard_board_type if hazard_pair_found else vehicle_board_type
            self.blackboard.board.type = board_type.value
            self.node.get_logger().info(f"[{self.name}] Detected board type: {board_type.name}")
        

        # # 2. Move to distance from board while aligning to board
        return py_trees.common.Status.SUCCESS
