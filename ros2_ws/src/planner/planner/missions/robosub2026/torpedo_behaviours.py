import py_trees
import py_trees_ros
from time import sleep
from ..vision_behaviours import SearchSweepBehaviour, CircleAroundToFindBehaviour
from ..action_status_enum import ActionStatus
from controls.utils import is_quaternion_outlier, compute_mean_orientation, find_normal_to_quaternion, Vector2D
from controls.goal_helpers import move_to_pose
from enum import Enum
from geometry_msgs.msg import Pose

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

class TorpedoStrategySelector(py_trees.composites.Selector):
    """
    High Level Strategy Selector for Torpedo Task. Depending on if we find the board
    1. Try to get highest points by finding board, aligning with board, and firing torpedos through the correct holes
    2. If we fail to find the board or find a path to the front of the board such that we are aligned, try to get partial points by just firing torpedos at random
    """
    
    def __init__(
            self,
            initial_distance_from_board: float = 3.0,
            z_reference: float = -1.0,
            scan_pause_time: float = 1.0,
            yaw_tolerance_rad: float = 0.3,
            hold_time: float = 0.5,
            timeout: float = 45.0,
        ):
        super().__init__("Torpedo Strategy", memory=True)
        self.factory = TorpedoNodeFactory()
        self.ss_time = scan_pause_time
        self.yaw_tolerance_rad = yaw_tolerance_rad
        self.hold_time = hold_time
        self.timeout = timeout
        self.add_children(
            [
                self.board_strategy(),
                self.node_base_case()
            ]
        )
        self.initial_distance_from_board = initial_distance_from_board
        self.z_reference = z_reference
        # convert initial distance from board to 2D distance from board for circling behaviour
        self.initial_distance_from_board_2d = (initial_distance_from_board**2 - z_reference**2)**0.5 

    def tick_tree(self):
        pass

    # This is the first node in the torpedo BT
    def board_strategy(self)->py_trees.composites.Sequence:
        """
        Subtree to align to board and fire into board
        1. Search Sweep to find board
        2. Determine Distance Strategy

        returns py_trees.composites.Sequence
        """
        board_align:py_trees.composites.Sequence = py_trees.composites.Sequence("Board Strategy", memory=True)
        ss_board = SearchSweepBehaviour(
            target_class="board",
            num_steps=5,
            max_attempts=2,
            step_timeout=self.ss_time,
            clockwise=False,
            look_at_on_success=True,
            turn_hold_time_s=self.hold_time,
            turn_timeout_s=self.timeout,
            name="Search Sweep for Board"
        )
        
        circle_board = CircleAroundToFindBehaviour(
            reference_class="board",
            z_reference=self.z_reference,
            target_classes={"blood":1, "fire":1, "ambulance":1, "firetruck":1},
            reference_distance=self.initial_distance_from_board_2d,
            num_circle_steps=6,
            max_circling_attempts=2,
            step_timeout=self.ss_time,
            clockwise=False,
            yaw_tolerance_rad=self.yaw_tolerance_rad,
        ) 
        board_align.add_children(
            [
                ss_board,
                circle_board,
                self.distance_strategy_selector()
            ]
        )

        return board_align
    
    def distance_strategy_selector(self)->py_trees.composites.Selector:
        """
        Distance from board
        1. Farther: 0.46m away
        2. Far: 0.3m away
        3. Close: <0.3m away, just stick torpedo up to board and hope for the best
        """
        distance_strategy_selector = py_trees.composites.Selector("Distance Strategy Selector", memory=True)
        distance_strategy_selector.add_children(
            [
                self.build_distance_strategy(0.46),
                self.build_distance_strategy(0.3),
                self.build_distance_strategy(0.1)
            ]
         )
        return distance_strategy_selector


    def build_distance_strategy(self, distance_from_board: float)->py_trees.composites.Sequence:
        """
        Build a distance strategy based on the distance from the board
        """
        distance_strategy = py_trees.composites.Sequence(f"Distance Strategy {distance_from_board}m", memory=True)

        move_and_align = MoveAndAlignToBoard(distance_from_board)
        # TODO add navigation to board position here
        distance_strategy.add_children(
            [
                move_and_align,
            ]
        )
        return distance_strategy
    
    def build_find_board_orientation_sequence(self)->py_trees.composites.Sequence:
        """
        Build a sequence to find the board orientation and align to it. Used to save a stable version of the board orientatio for downstream alignments when approaching the board

        returns py_trees.composites.Sequence
        """
        find_board_orientation_sequence = py_trees.composites.Sequence("Find Board Orientation Sequence", memory=True)
        # find_board_orientation_and_align = FindBoardOrientationAndAlign()
        find_board_orientation_sequence.add_children(
            [
                find_board_orientation_and_align,
            ]
        )
        return find_board_orientation_sequence

    def node_base_case(self)->py_trees.composites.Sequence:
        """
        Base case subtree that will just try to fire the torpedos without finding or aligning to board
        
        returns py_trees.composites.Sequence
        """

        node_base_case = py_trees.composites.Sequence("base_case", memory=True)
        check_torpedo_count1 = Action("check_torpedo_count1")
        check_torpedo_count2 = Action("check_torpedo_count2")
        fire_torpedo1 = Action("fire_torpedo1")
        fire_torpedo2 = Action("fire_torpedo2") 
        node_base_case.add_children(
            [
                check_torpedo_count1,
                fire_torpedo1,
                check_torpedo_count2,
                fire_torpedo2
            ]
        )
        return node_base_case


### Actions:
class FindBoardOrientation(Action):
    """
    Action to find the orientation of the board and write it to the blackboard. This will be used for downstream alignment to the board when we are close to the board and can no longer rely on vision to find the orientation of the board.
    This is necessary for orientation in particular since position averaging is already handled by a Kalman Filter in the vision pipeline, whereas orientation is inferred based on the icons on the board, and is more susceptible to noise and outliers.
    """
    def __init__(
            self,
            n_samples: int = 5,
            sample_every_n_ticks: int = 1,
            rejection_threshold: float = 0.2,
        ):
        super().__init__("Find Board Orientation" + (f" sampling {n_samples} times"))
        self.orientation_samples_number = n_samples
        self.sample_rate = sample_every_n_ticks
        self.rejection_threshold = rejection_threshold
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.sample_count = 0
        self.tick_counter = 0
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
            if is_quaternion_outlier(orientation, self.orientation_samples, self.rejection_threshold):
                self.logger.warning(f"[{self.name}] Rejected outlier orientation sample: {orientation}")
                return py_trees.common.Status.RUNNING
        self.orientation_samples.append(orientation)
        self.sample_count += 1
        self.logger.info(f"[{self.name}] Collected orientation sample {self.sample_count}/{self.orientation_samples_number}: {orientation}")
        
        if self.sample_count >= self.orientation_samples_number:
            # Compute mean orientation and write to blackboard
            mean_orientation = compute_mean_orientation(self.orientation_samples)
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
        self.result_message = ""
        
        self.action_status:ActionStatus = ActionStatus.NOT_SENT
        self.sent_goal = False
        
    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.navigation_client = kwargs['navigation_client']
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/vision/object_map", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/board/orientation", access=py_trees.common.Access.READ)
        self.result_message = ""

    
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
                
                # orientation computed by object map is always normal pointing towards the AUV, so we simply take a point at the desired distance in front of the board along the orientation normal as our target point
                
                # recover normal from yaw of board orientation
                board_normal = find_normal_to_quaternion(board_orientation)
                
                # compute target point at desired distance from board along normal
                target_xy = Vector2D.from_point(board_vo.pose.position) + board_normal.normalized() * self.distance_from_board
                
                target_pose = Pose()
                target_pose.position.x = target_xy.x
                target_pose.position.y = target_xy.y
                target_pose.position.z = self.z_reference
                target_pose.orientation = board_orientation # we want to match the orientation of the board 
                
                goal = move_to_pose(
                    pose=target_pose,
                    tolerance=self.position_tolerance,
                    angular_tolerance=self.orientation_tolerance_rad,
                    timeout=self.timeout,
                    hold_time=self.hold_time
                )
                
                self.navigation_client.send_goal(goal, self.on_server_goal_response, self.on_server_goal_result)
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
