import py_trees
from .torpedo_behaviours import MoveToFrontOfBoardAndAlign, FindBoardOrientation
from ..vision_behaviours import SearchSweepBehaviour, CircleAroundToFindBehaviour

class TorpedoTask(py_trees.composites.Sequence):
    """
    Sequence for the RoboSub 2026 Torpedo Task (Deploy).
    AUV fires torpedoes through openings in a board.
    Targets match role (Fire/Firetruck vs Blood/Ambulance).
    Sequence: Large opening then Small opening.
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
        
        # convert initial distance from board to 2D distance from board for circling behaviour
        self.initial_distance_from_board_2d = (initial_distance_from_board**2 - z_reference**2)**0.5 
        self.add_children([
            self.board_sequence(),
            self.node_base_case(),
            py_trees.behaviours.Success(name="Placeholder Torpedo Success")
        ])

    def tick_tree(self):
        pass

    def board_sequence(self)->py_trees.composites.Sequence:
        """
        Strategy to find and align to board, then fire torpedos through openings. If we fail to find or align to the board, we can fallback to just firing torpedos at random in front of us for partial points.

        returns py_trees.composites.Sequence
        """
        board_strategy = py_trees.composites.Sequence("Board Strategy", memory=True)
        board_strategy.add_children(
            [
                self.board_rough_position_sequence(),
                self.board_orientation_refinment_selector(attempts=3, alignments_per_attempt=2),
                # TODO add firing torpedo behaviours here
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
            reference_distance=self.initial_distance_from_board_2d,
            num_circle_steps=6,
            max_circling_attempts=2,
            step_timeout=self.pause_time,
            clockwise=False,
            position_tolerance=self.position_tolerance,
            yaw_tolerance_rad=self.yaw_tolerance_rad,
            name="Circle Around Board to Find Icons"
        )
        
        find_board_orientation_one_sample = FindBoardOrientation (
            n_samples=1,
            sample_every_n_ticks=1,
            rejection_threshold=self.yaw_tolerance_rad,
            compare_measurement_with_blackboard=False
        )
        
        align_to_board = MoveToFrontOfBoardAndAlign(
            distance_from_board=self.initial_distance_from_board,
            z_reference=self.z_reference,
            position_tolerance=0.5,
            orientation_tolerance_rad=self.yaw_tolerance_rad,
            hold_time=self.hold_time,
            timeout=self.timeout
        )
        
        board_rough_position.add_children(
            [
                ss_board,
                circle_board,
                find_board_orientation_one_sample,
                align_to_board
            ]
        )
        return board_rough_position
    
    
    
    def board_orientation_refinment_selector(self, attempts:int, alignments_per_attempt:int)->py_trees.composites.Selector:
        """
        Subtree containing "<attempts>" sequences to find the board orientation. Every sequence will try to find the board orientation and align to it, with an extra check at the end to see if orientation has stabilized. This is to save a stable version of the board orientatio for downstream alignments when approaching the board. 

        returns py_trees.composites.Selector
        """
        find_board_orientation_selector = py_trees.composites.Selector("Find Board Orientation Strategy Selector", memory=True)
        
        find_board_orientations = []
        for _ in range(attempts):
            find_board_orientation_and_align = self.board_orientation_refinement_sequence(alignments_per_attempt)
            find_board_orientations.append(find_board_orientation_and_align)
        find_board_orientation_selector.add_children(find_board_orientations)
        return find_board_orientation_selector
    
    def board_orientation_refinement_sequence(self, alignments: int)->py_trees.composites.Sequence:
        """
        Subtree to find the board orientation and align to it.
        1. Find board orientation with multiple samples rejecting outliers, to get a more stable estimate of the board orientation
        2. Align to board again based on stable orientation estimate
        3. Find orientation again and check if it's close to stable orientation estimate, if not, fail entire sequence

        returns py_trees.composites.Sequence
        """
        find_board_orientation_sequence = py_trees.composites.Sequence("Find Board Orientation Sequence", memory=False)
        
        
        # find_board_orientation_sequence.add_children(
        # )
        return find_board_orientation_sequence
    
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
        node_base_case.add_children(
            [
                # TODO add behaviours to just fire torpedos at random in front of us for partial points
                py_trees.behaviours.Success(name="Placeholder Torpedo Success")
            ]
        )
        return node_base_case