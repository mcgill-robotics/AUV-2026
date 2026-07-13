import py_trees
from numpy.polynomial.polynomial import Polynomial
from .torpedo_behaviours import *
from ..vision_behaviours import SearchSweepBehaviour, CircleAroundToFindBehaviour, GoNearObject
import operator
from typing import List, cast, Tuple, Callable
class HoleType(Enum):
    LARGE = 1
    SMALL = 2
class TorpedoTask(py_trees.composites.Sequence):
    """
    Torpedo Task Tree
    """
    def __init__(
            self,
            initial_distance_from_board: float = 3.0,
            z_reference: float = -1.0,
            scan_pause_time: float = 1.0,
            position_tolerance: float = 0.5,
            yaw_tolerance_rad: float = 0.3,
            hold_time: float = 0.5,
            timeout: float = 45.0,
            orientation_refinement_rejection_threshold_rad: float = 0.1,
            position_refinement_rejection_threshold: float = 0.1,
            refinement_attempts: int = 3,
            alignments_per_attempt: int = 3,
            orientation_samples_per_alignment: int = 5,
            position_samples_per_alignment: int = 5,
            refinement_sample_every_n_ticks: int = 3,
            # TODO use tuple instead
            auv_to_torpedos: dict[str, list[float]] = {
                "left": [-0.1204,-0.0167, -0.1624],
                "right": [-0.1174,0.0167, -0.1624]
            },
            torpedo_trajectory_coefficients: dict[str, list[float]] = {
                "x": [0.0, 1.35, -1.44, 0.701, -0.117],
                "y": [0.0, 0.0, 0.0, 0.0, 0.0],
                "z": [0.0, -0.0181, 0.241, -0.172, 0.0332]
            },
            icon_to_nearest_hole: dict[str, dict[str, list[float]]] = {
                "board_1": {
                    "blood": [-0.045, -0.045, 0.0],
                    "fire": [0.045, 0.045, 0.0],
                    "ambulance": [-0.045, 0.045, 0.0],
                    "firetruck": [0.045, -0.045, 0.0]
                },
                "board_2": {
                    "blood": [0.045, 0.045, 0.0],
                    "fire": [-0.045, -0.045, 0.0],
                    "ambulance": [0.045, -0.045, 0.0],
                    "firetruck": [-0.045, 0.045, 0.0]
                }
            },
            launch_function: Callable[[TorpedoSide], None] = lambda side: print(f"Launching {side.name} torpedo"),
            torpedo_firing_buffer_time: float = 1.0
        ):
        super().__init__("Torpedo Task", memory=True)
        
        self.launch_function = launch_function
        self.pause_time = scan_pause_time
        self.position_tolerance = position_tolerance
        self.yaw_tolerance_rad = yaw_tolerance_rad
        self.hold_time = hold_time
        self.timeout = timeout
        self.initial_distance_from_board = initial_distance_from_board
        self.z_reference = z_reference

        # refinement parameters
        self.orientation_refinement_rejection_threshold_rad:float = orientation_refinement_rejection_threshold_rad
        self.position_refinement_rejection_threshold:float = position_refinement_rejection_threshold
        self.refinement_attempts:int = refinement_attempts
        self.alignments_per_attempt:int = alignments_per_attempt
        self.orientation_samples_per_alignment:int = orientation_samples_per_alignment
        self.position_samples_per_alignment:int = position_samples_per_alignment
        self.refinement_sample_every_n_ticks:int = refinement_sample_every_n_ticks

        # distance thresholds from torpedo board
        self.farther_distance_threshold = 0.46
        self.far_distance_threshold = 0.3

        # trajectory parameters
        self.auv_to_torpedos : dict[TorpedoSide, Tuple[float,float,float]] = {
            TorpedoSide.LEFT: cast(Tuple[float,float,float], auv_to_torpedos["left"]),
            TorpedoSide.RIGHT: cast(Tuple[float,float,float], auv_to_torpedos["right"])
        }
        self.forward_trajectory:Polynomial = Polynomial(torpedo_trajectory_coefficients["x"])
        self.lateral_trajectory:Polynomial = Polynomial(torpedo_trajectory_coefficients["y"])
        self.vertical_trajectory:Polynomial = Polynomial(torpedo_trajectory_coefficients["z"])
        
        self.icon_to_nearest_hole : dict[BoardType, dict[BoardIcon, Tuple[float, float, float]]] = {
            BoardType.FIRE_TOP_LEFT: {
                # type cast to satisfy linter
                BoardIcon.BLOOD: cast(Tuple[float, float, float], icon_to_nearest_hole["board_1"]["blood"]), 
                BoardIcon.FIRE: cast(Tuple[float, float, float], icon_to_nearest_hole["board_1"]["fire"]),
                BoardIcon.AMBULANCE: cast(Tuple[float, float, float], icon_to_nearest_hole["board_1"]["ambulance"]),
                BoardIcon.FIRETRUCK: cast(Tuple[float, float, float], icon_to_nearest_hole["board_1"]["firetruck"])
            },
            BoardType.BLOOD_TOP_LEFT: {
                BoardIcon.BLOOD: cast(Tuple[float, float, float], icon_to_nearest_hole["board_2"]["blood"]),
                BoardIcon.FIRE: cast(Tuple[float, float, float], icon_to_nearest_hole["board_2"]["fire"]),
                BoardIcon.AMBULANCE: cast(Tuple[float, float, float], icon_to_nearest_hole["board_2"]["ambulance"]),
                BoardIcon.FIRETRUCK: cast(Tuple[float, float, float], icon_to_nearest_hole["board_2"]["firetruck"])
            }
        }
        
        self.torpedo_firing_buffer_time = torpedo_firing_buffer_time
        
        self.add_children([
            py_trees.behaviours.SetBlackboardVariable(
                name="Set Torpedo Count to 2",
                variable_name="/torpedo/count",
                variable_value=2,
                overwrite=True
            ),
            self.strategy_selector()
        ])

    def strategy_selector(self)->py_trees.composites.Selector:
        """
        Selector for torpedo strategy. If we can find and align to the board, we will try to fire torpedos through the openings. If we fail to find or align to the board, we can fallback to just firing torpedos at random in front of us for partial points.

        returns py_trees.composites.Selector
        """
        strategy_selector = py_trees.composites.Selector("Torpedo Strategy Selector", memory=True)
        strategy_selector.add_children(
            [
                self.board_sequence(),
                self.node_base_case()
            ]
         )
        return strategy_selector

    def board_sequence(self)->py_trees.composites.Sequence:
        """
        Strategy to find and align to board, then fire torpedos through openings. If we fail to find or align to the board, we can fallback to just firing torpedos at random in front of us for partial points.

        returns py_trees.composites.Sequence
        """
        board_strategy = py_trees.composites.Sequence("Board Strategy", memory=True)
        board_strategy.add_children(
            [
                self.board_rough_position_sequence(),
                self.board_orientation_refinement_selector(),
                DetermineBoardType(),
                self.icon_strategy_selector(self.farther_distance_threshold)
            ]
         )
        return board_strategy

    def board_rough_position_sequence(self)->py_trees.composites.Sequence:
        """
        Sequence for initial position in front of board

        returns py_trees.composites.Sequence
        """
        board_rough_position:py_trees.composites.Sequence = py_trees.composites.Sequence("Board Rough Positioning", memory=True)
        ss_board = SearchSweepBehaviour(
            target_class="board",
            num_steps=5,
            max_attempts=2,
            step_timeout=self.pause_time,
            clockwise=False,
            look_at_on_success=True,
            turn_timeout_s=self.timeout,
            turn_hold_time_s=self.hold_time,
            name="Search Sweep for Board"
        )
        
        approach_board_far = GoNearObject(
            target_class="board",
            target_distance=self.initial_distance_from_board*4,
            height_offset=None,
            tolerance_meters=self.position_tolerance,
            hold_time=self.hold_time / 2,
            name="Approach Board Far"
        )
        
        approach_board_near = GoNearObject(
            target_class="board",
            target_distance=self.initial_distance_from_board*2,
            height_offset=None,
            tolerance_meters=self.position_tolerance,
            hold_time=self.hold_time / 2,
            name="Approach Board Near"
        )

        circle_board = CircleAroundToFindBehaviour(
            reference_class="board",
            z_reference=self.z_reference,
            target_classes={"blood":1, "fire":1, "ambulance":1, "firetruck":1},
            reference_distance=self.initial_distance_from_board,
            num_circle_steps=6,
            max_circling_attempts=2,
            step_timeout=self.pause_time,
            clockwise=False,
            position_tolerance=self.position_tolerance,
            yaw_tolerance_rad=self.yaw_tolerance_rad,
            name="Circle Around Board to Find Icons"
        )
        
        find_board_orientation_one_sample = DetermineBoardPose (
            n_orientation_samples=1,
            n_position_samples=1,
            sample_every_n_ticks=1,
            orientation_rejection_threshold=self.orientation_refinement_rejection_threshold_rad,
            position_rejection_threshold=self.position_refinement_rejection_threshold,
            compare_measurement_with_blackboard=False
        )
        
        move_to_front_of_board = MoveToFrontOfBoard(
            align_to_board=True,
            distance_from_board=self.initial_distance_from_board,
            z_reference=self.z_reference,
            position_tolerance=self.position_tolerance,
            orientation_tolerance_rad=self.yaw_tolerance_rad,
            hold_time=self.hold_time,
            timeout=self.timeout
        )
        
        board_rough_position.add_children(
            [
                ss_board,
                approach_board_far,
                approach_board_near,
                circle_board,
                find_board_orientation_one_sample,
                move_to_front_of_board
            ]
        )
        return board_rough_position

    def board_orientation_refinement_selector(self)->py_trees.composites.Selector:
        """
        Subtree containing "<attempts>" sequences to find the board orientation. Every sequence will try to find the board orientation and align to it, with an extra check at the end to see if orientation has stabilized. This is to save a stable version of the board orientatio for downstream alignments when approaching the board. 

        returns py_trees.composites.Selector
        """
        find_board_orientation_selector = py_trees.composites.Selector("Find Board Orientation Refinement Attempt Selector", memory=True)
        find_board_orientation_selector.add_child(
            py_trees.decorators.Retry(
                child=self.board_orientation_refinement_sequence(),
                name=f"Refinement Attempt",
                num_failures=self.refinement_attempts
            )
        )
        
        
        return find_board_orientation_selector
    
    def board_orientation_refinement_sequence(self)->py_trees.composites.Sequence:
        """
        Subtree to find the board orientation and align to it.
        1. Find board orientation with multiple samples rejecting outliers, to get a more stable estimate of the board orientation
        2. Align to board again based on stable orientation estimate
        3. Find orientation again and check if it's close to stable orientation estimate, if not, fail entire sequence

        returns py_trees.composites.Sequence
        """
        find_board_orientation_sequence = py_trees.composites.Sequence(f"Refinement Board Orientation Sequence", memory=True)
        
        find_and_align_sequence: py_trees.composites.Sequence = py_trees.composites.Sequence("Find and Align", memory=True)
        find_and_align_sequence.add_children([
            DetermineBoardPose (
                n_orientation_samples=self.orientation_samples_per_alignment,
                n_position_samples=self.position_samples_per_alignment,
                sample_every_n_ticks=self.refinement_sample_every_n_ticks,
                orientation_rejection_threshold=self.orientation_refinement_rejection_threshold_rad,
                position_rejection_threshold=self.position_refinement_rejection_threshold,
                compare_measurement_with_blackboard=False
            ),
            AlignToBoard(
                position_tolerance=0.5,
                orientation_tolerance_rad=self.yaw_tolerance_rad,
                hold_time=self.hold_time,
                timeout=self.timeout
            )
        ])
        children:List[py_trees.behaviour.Behaviour] = [py_trees.decorators.Repeat(
            child=find_and_align_sequence, 
            name=f"Find and Align",
            num_success=self.alignments_per_attempt)]
        consistency_check = DetermineBoardPose (
                n_orientation_samples=1,
                n_position_samples=1,
                sample_every_n_ticks=1,
                orientation_rejection_threshold=self.orientation_refinement_rejection_threshold_rad,
                position_rejection_threshold=self.position_refinement_rejection_threshold,
                compare_measurement_with_blackboard=True
            )
        children.append(consistency_check)
        find_board_orientation_sequence.add_children(children)
        return find_board_orientation_sequence


    def icon_strategy_selector(self, distance_from_board:float)->py_trees.composites.Selector:
        """
        Distance from board
        1. Farther: 0.46m away
        2. Far: 0.3m away
        3. Close: <0.3m away, just stick torpedo up to board and hope for the best
        """
        distance_strategy_selector = py_trees.composites.Selector("Icon Strategy Selector", memory=True)
        distance_strategy_selector.add_children(
            [
                self.icon_strategy(distance_from_board,BoardIcon.FIRE),
                # self.icon_strategy(distance_from_board,BoardIcon.BLOOD),
                # self.icon_strategy(distance_from_board,BoardIcon.FIRETRUCK),
                # self.icon_strategy(distance_from_board,BoardIcon.AMBULANCE)
            ]
         )
        return distance_strategy_selector


    def icon_strategy(self, distance_from_board:  float, icon: BoardIcon)->py_trees.composites.Sequence:
        """
        Build an icon strategy based on the distance from the board
        """
        # include distance from board as negative x
        distance_strategy = py_trees.composites.Sequence(f"Icon Strategy {icon.name}", memory=True)

        
        distance_strategy.add_children(
            [
                MoveToFrontOfBoard(
                    align_to_board=True,
                    distance_from_board=self.initial_distance_from_board,
                    z_reference=self.z_reference,
                    position_tolerance=self.position_tolerance,
                    orientation_tolerance_rad=self.yaw_tolerance_rad,
                    hold_time=self.hold_time,
                    timeout=self.timeout
                ),
                self.torpedo_firing_sequence(icon, distance_from_board)
            ]
        )
        return distance_strategy

    def torpedo_firing_sequence(self, icon:BoardIcon, distance_from_board: float)->py_trees.composites.Sequence:
        
        torpedo_firing_sequence = py_trees.composites.Sequence(f"Torpedo Firing Sequence", memory=True)
        
        torpedo_firing_sequence.add_children([
                AlignTorpedoToHole(
                        target_icon=icon,
                        attempts_to_find_icon=3,
                        auv_to_front = 0,
                        distance_from_board = distance_from_board,
                        icon_to_nearest_hole = self.icon_to_nearest_hole,
                        auv_to_torpedos = self.auv_to_torpedos,
                        forward_trajectory = self.forward_trajectory,
                        lateral_trajectory = self.lateral_trajectory,
                        vertical_trajectory = self.vertical_trajectory,
                        position_tolerance = self.position_tolerance,
                        orientation_tolerance_rad = self.yaw_tolerance_rad,
                        timeout = self.timeout,
                        hold_time = self.hold_time
                    ),
                FireTorpedo(launch_function=self.launch_function, firing_buffer_time=self.torpedo_firing_buffer_time),
            ]
        )
        return torpedo_firing_sequence
    
    def node_base_case(self)->py_trees.composites.Sequence:
        """
        Base case subtree that will just try to fire the torpedos without finding or aligning to board
        
        returns py_trees.composites.Sequence
        """

        node_base_case = py_trees.composites.Sequence("Base Case (No Board Alignment)", memory=True)
        node_base_case.add_children(
            [
                py_trees.behaviours.CheckBlackboardVariableValue(
                    name="Check torpedo count is at least 1",
                    check=py_trees.common.ComparisonExpression(
                        variable="/torpedo/count",
                        value=1,
                        operator=operator.ge,
                    )
                ),
                FireTorpedo(launch_function=self.launch_function, firing_buffer_time=self.torpedo_firing_buffer_time)
            ]
        )
        return node_base_case