import py_trees
from numpy.polynomial.polynomial import Polynomial
from .torpedo_behaviours import *
from ..vision_behaviours import SearchSweepBehaviour, CircleAroundToFindBehaviour
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
            refinement_rejection_threshold_rad: float = 0.1,
            refinement_attempts: int = 3,
            alignments_per_attempt: int = 3,
            samples_per_alignment: int = 5,
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
            }
        ):
        super().__init__("Torpedo Task", memory=True)
        # TODO: Implement Torpedo Task
        # 1. Locate Torpedo board via acoustic localization or vision
        # 2. Identify large and small openings matching role
        # 3. Align with Large opening and fire torpedo
        # 4. Align with Small opening and fire torpedo
        # 5. Optional: Maintain distance (1ft or 1.5ft) for extra points
        
        self.pause_time = scan_pause_time
        self.position_tolerance = position_tolerance
        self.yaw_tolerance_rad = yaw_tolerance_rad
        self.hold_time = hold_time
        self.timeout = timeout
        self.initial_distance_from_board = initial_distance_from_board
        self.z_reference = z_reference

        # refinement parameters
        self.refinement_rejection_threshold_rad:float = refinement_rejection_threshold_rad
        self.refinement_attempts:int = refinement_attempts
        self.alignments_per_attempt:int = alignments_per_attempt
        self.samples_per_alignment:int = samples_per_alignment
        self.refinement_sample_every_n_ticks:int = refinement_sample_every_n_ticks

        # distance thresholds from torpedo board
        self.farther_distance_threshold = 0.46
        self.far_distance_threshold = 0.3
        
        self.add_children([
            self.board_sequence(),
            self.node_base_case(),
            py_trees.behaviours.Success(name="Placeholder Torpedo Success")
        ])
        
        # trajectory parameters
        self.auv_to_torpedos = {
            TorpedoSide.LEFT: auv_to_torpedos["left"],
            TorpedoSide.RIGHT: auv_to_torpedos["right"]
        }
        self.forward_trajectory:Polynomial = Polynomial(torpedo_trajectory_coefficients["x"])
        self.lateral_trajectory:Polynomial = Polynomial(torpedo_trajectory_coefficients["y"])
        self.vertical_trajectory:Polynomial = Polynomial(torpedo_trajectory_coefficients["z"])

    def tick_tree(self):
        pass

    def board_sequence(self)->py_trees.composites.Sequence:
        """
        Strategy to find and align to board, then fire torpedos through openings. If we fail to find or align to the board, we can fallback to just firing torpedos at random in front of us for partial points.

        returns py_trees.composites.Sequence
        """
        board_strategy = py_trees.composites.Sequence("Board Strategy", memory=True)
        board_type = DetermineBoardType()
        board_strategy.add_children(
            [
                self.board_rough_position_sequence(),
                self.board_orientation_refinement_selector(),
                board_type,
                self.firing_order_strategy_selector()
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
        
        find_board_orientation_one_sample = DetermineBoardOrientation (
            n_samples=1,
            sample_every_n_ticks=1,
            rejection_threshold=self.refinement_rejection_threshold_rad,
            compare_measurement_with_blackboard=False
        )
        
        move_to_front_of_board = MoveToFrontOfBoard(
            distance_from_board=self.initial_distance_from_board,
            z_reference=self.z_reference,
            position_tolerance=self.position_tolerance,
            orientation_tolerance_rad=self.yaw_tolerance_rad,
            hold_time=self.hold_time,
            timeout=self.timeout
        )
        
        align_to_board = AlignToBoard(
            position_tolerance=self.position_tolerance,
            orientation_tolerance_rad=self.yaw_tolerance_rad,
            hold_time=self.hold_time,
            timeout=self.timeout
        )
        
        board_rough_position.add_children(
            [
                ss_board,
                circle_board,
                find_board_orientation_one_sample,
                move_to_front_of_board,
                align_to_board
            ]
        )
        return board_rough_position

    def board_orientation_refinement_selector(self)->py_trees.composites.Selector:
        """
        Subtree containing "<attempts>" sequences to find the board orientation. Every sequence will try to find the board orientation and align to it, with an extra check at the end to see if orientation has stabilized. This is to save a stable version of the board orientatio for downstream alignments when approaching the board. 

        returns py_trees.composites.Selector
        """
        find_board_orientation_selector = py_trees.composites.Selector("Find Board Orientation Refinement Attempt Selector", memory=True)
        
        find_board_orientation_seqs = []
        for i in range(self.refinement_attempts):
            find_board_orientation_and_align = self.board_orientation_refinement_sequence(i)
            find_board_orientation_seqs.append(find_board_orientation_and_align)
        find_board_orientation_selector.add_children(find_board_orientation_seqs)
        return find_board_orientation_selector
    
    def board_orientation_refinement_sequence(self, count:int)->py_trees.composites.Sequence:
        """
        Subtree to find the board orientation and align to it.
        1. Find board orientation with multiple samples rejecting outliers, to get a more stable estimate of the board orientation
        2. Align to board again based on stable orientation estimate
        3. Find orientation again and check if it's close to stable orientation estimate, if not, fail entire sequence

        returns py_trees.composites.Sequence
        """
        find_board_orientation_sequence = py_trees.composites.Sequence(f"Refinement Board Orientation Sequence {count+1}/{self.alignments_per_attempt+1}", memory=True)
        
        find_and_aligns = []
        for _ in range(self.alignments_per_attempt):
            find_board_orientation = DetermineBoardOrientation (
                n_samples=self.samples_per_alignment,
                sample_every_n_ticks=self.refinement_sample_every_n_ticks,
                rejection_threshold=self.refinement_rejection_threshold_rad,
                compare_measurement_with_blackboard=False
            )
            find_and_aligns.append(find_board_orientation)
            align_to_board = AlignToBoard(
                position_tolerance=0.5,
                orientation_tolerance_rad=self.yaw_tolerance_rad,
                hold_time=self.hold_time,
                timeout=self.timeout
            )
            find_and_aligns.append(align_to_board)
        consistency_check = DetermineBoardOrientation (
                n_samples=1,
                sample_every_n_ticks=1,
                rejection_threshold=self.refinement_rejection_threshold_rad,
                compare_measurement_with_blackboard=True
            )
        find_and_aligns.append(consistency_check)
        find_board_orientation_sequence.add_children(find_and_aligns)
        return find_board_orientation_sequence

    def firing_order_strategy_selector(self)->py_trees.composites.Selector:
        """
        Selector for firing order strategy. In order of points acquired, where large holes are near fire/blood icons, small holes are near ambulance/firetruck icons:
        
        - large then small
        - large
        - small

        returns py_trees.composites.Selector
        """
        firing_order_selector = py_trees.composites.Selector("Firing Order Strategy Selector", memory=True)
        
        large_then_small = py_trees.composites.Sequence("Large then Small Strategy", memory=True)
        large_then_small.add_children(
            [
                CheckTorpedoCount(expected_count=2),
                self.board_type_selector(HoleType.LARGE),
                self.board_type_selector(HoleType.SMALL)
            ]
        )
        large_only = py_trees.composites.Sequence("Large Only Strategy", memory=True)
        small_only = py_trees.composites.Sequence("Small Only Strategy", memory=True)
        small_only.add_children(
            [
                CheckTorpedoCount(expected_count=1),
                self.board_type_selector(HoleType.SMALL)
            ]
        )
        large_only.add_children(
            [
                CheckTorpedoCount(expected_count=1),
                self.board_type_selector(HoleType.LARGE)
            ]
        )
        
        firing_order_selector.add_children(
            [
                large_then_small,
                large_only,
                small_only
            ]
         )
        return firing_order_selector


    def board_type_selector(self, hole_type: HoleType)->py_trees.composites.Selector:
        """
        Here is where torpedo strategy diverges based on board type
        """
        board_type_selector = py_trees.composites.Selector("Board Type Selector", memory=True)
        
        board_type_selector.add_children(
            [
                self.board_type_strategy(BoardType.FIRE_TOP_LEFT, hole_type),
                self.board_type_strategy(BoardType.BLOOD_TOP_LEFT, hole_type)
            ]
         )
        return board_type_selector
        
    def board_type_strategy(self, board_type: BoardType, hole_type: HoleType) -> py_trees.composites.Sequence:
        """
        Check board type, and immediately fail if board type does not match. Otherwise try firing strategies based on board type.
        TODO: We can make this dynamic to tree is pruned based on which board type we have, see MissionSpawner
        """
        board_type_strategy = py_trees.composites.Sequence(f"Board type {board_type.value} Strategy for {hole_type.name} Hole", memory=True)
        board_type_strategy.add_children(
            [
                CheckBoardType(board_type=board_type),
                self.hole_selector(board_type, hole_type)
            ]
         )
        return board_type_strategy
        
    
    
    def hole_selector(self, board_type: BoardType, hole_type: HoleType)->py_trees.composites.Selector:
        """
        Selector for which hole to fire into
        """
        hole_selector = py_trees.composites.Selector(f"{hole_type.name} Hole Selector for Board type {board_type.value}", memory=True)
        match hole_type:
            case HoleType.LARGE: # hazard icons
                match board_type:
                    case BoardType.FIRE_TOP_LEFT:
                        # top left
                        icon1 = BoardIcon.FIRE
                        # bottom right
                        icon2 = BoardIcon.BLOOD
                    case BoardType.BLOOD_TOP_LEFT:
                        # top left
                        icon1 = BoardIcon.BLOOD
                        # bottom right
                        icon2 = BoardIcon.FIRE
            case HoleType.SMALL: # vehicle icons
                match board_type:
                    case BoardType.FIRE_TOP_LEFT:
                        icon1 = BoardIcon.AMBULANCE
                        icon2 = BoardIcon.FIRETRUCK
                    case BoardType.BLOOD_TOP_LEFT:
                        icon1 = BoardIcon.FIRETRUCK
                        icon2 = BoardIcon.AMBULANCE
        hole_selector.add_children(
            [
                self.distance_strategy_selector(icon1),
                self.distance_strategy_selector(icon2)
            ]
        )                                          
        return hole_selector

    def distance_strategy_selector(self, icon: BoardIcon)->py_trees.composites.Selector:
        """
        Distance from board
        1. Farther: 0.46m away
        2. Far: 0.3m away
        3. Close: <0.3m away, just stick torpedo up to board and hope for the best
        """
        distance_strategy_selector = py_trees.composites.Selector("Distance Strategy Selector", memory=True)
        find_icon = DetermineIconPosition(icon, attempts=3)
        distance_strategy_selector.add_children(
            [
                find_icon,
                self.distance_strategy(self.farther_distance_threshold,icon),
                self.distance_strategy(self.far_distance_threshold,icon),
            ]
         )
        return distance_strategy_selector


    def distance_strategy(self, distance_from_board: float, icon: BoardIcon)->py_trees.composites.Sequence:
        """
        Build a distance strategy based on the distance from the board
        """
        distance_strategy = py_trees.composites.Sequence(f"Distance Strategy {distance_from_board}m from {icon.name}", memory=True)
        # since we always aim for same Z, assume Z distance is 0 and convert distance from board to 2D distance
        distance_from_board_2d = distance_from_board

        distance_strategy.add_children(
            [
                MoveToFrontOfBoard(
                    distance_from_board=self.initial_distance_from_board,
                    z_reference=self.z_reference,
                    position_tolerance=self.position_tolerance,
                    orientation_tolerance_rad=self.yaw_tolerance_rad,
                    hold_time=self.hold_time,
                    timeout=self.timeout
                ),
                AlignToBoard(
                    position_tolerance=self.position_tolerance,
                    orientation_tolerance_rad=self.yaw_tolerance_rad,
                    hold_time=self.hold_time,
                    timeout=self.timeout
                ),
                MoveToFrontOfIcon(
                            target_icon=icon,
                            use_icon_z=True,
                            distance_from_icon=distance_from_board_2d,
                            position_tolerance=self.position_tolerance,
                            orientation_tolerance_rad=self.yaw_tolerance_rad,
                            hold_time=self.hold_time,
                            timeout=self.timeout
                )
            ]
        )
        return distance_strategy

    def node_base_case(self)->py_trees.composites.Sequence:
        """
        Base case subtree that will just try to fire the torpedos without finding or aligning to board
        
        returns py_trees.composites.Sequence
        """

        node_base_case = py_trees.composites.Sequence("base_case", memory=True)
        node_base_case.add_children(
            [
                # TODO add behaviours to just fire torpedos at random in front of us for partial points
                py_trees.behaviours.Success(name="Placeholder Torpedo Success")
            ]
        )
        return node_base_case