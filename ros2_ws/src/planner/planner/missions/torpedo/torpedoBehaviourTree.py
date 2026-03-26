import py_trees
import py_trees_ros
from time import sleep

class Action(pytrees.behaviour.Behaviour):
    def __init__(self, name):
        super(Action, self).__init__(name)

    def setup(self):
        self.logger.debug(f"Action::setup {self.name}")

    def initialise(self):
        self.logger.debug(f"Action::initialize {self.name}")

    def update(self):
        self.logger.debug(f"Action::update {self.name}")
        sleep(1)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Action::terminate {self.name} to {new_status}")

class Condition(pytrees.behaviour.Behaviour):
    def __init__(self, name):
        super(Condition, self).__init__(name)

    def setup(self):
        self.logger.debug(f"Condition::setup {self.name}")

    def initialise(self):
        self.logger.debug(f"Condition::initialize {self.name}")

    def update(self):
        self.logger.debug(f"Condition::update {self.name}")
        sleep(1)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Condition::terminate {self.name} to {new_status}")

class TorpedoNodeFactory:
    """
    Handles reusable subtrees for Torpedo Behaviour Tree 
    """
    
    def make_node_alignment_sequence(self, suffix="")->py_trees.composites.Sequence:
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

class TorpedoBehaviourTree:
    def __init__(self):
        self.factory = TorpedoNodeFactory()
        self.root = py_trees.composites.Selector("torpedo_mission_root", memory=False)
        self.root.add_children(
            [
                self.node_highest_pts(),
                self.node_partial_points(),
                self.node_base_case()
            ]
        )

    # This is the first node in the torpedo BT
    def node_highest_pts(self)->py_trees.composites.Sequence: # rename this to something better
        """
        First subtree to get the hightest amount of points.
        Sequence: get the chosen animal, go to chosen animal, fire torpedo 1, go to other animal, fire torpedo 2.

        returns py_trees.composites.Sequence
        """
        node_highest_pts = py_trees.composites.Sequence("highest_pts", memory=True)
        get_chosen_animal = Action("get_chosen_animal")
        go_to_animal1 = self.factory.make_node_go_to_animal("Animal1") # TODO: replace this with animal name
        go_to_animal2 = self.factory.make_node_go_to_animal("Animal2") # TODO: replace with other animal name
        fire_torpedo1 = Action("fire_torpedo1")
        fire_torpedo2 = Action("fire_torpedo2")

        node_highest_pts.add_children(
            [
                get_chosen_animal,
                go_to_animal1,
                fire_torpedo1,
                go_to_animal2, 
                fire_torpedo2,
            ]
        )

        return node_highest_pts

    def node_partial_points(self)->py_trees.composites.Sequence: # TODO: rename this to something less ambiguous
        """
        Subtree to get partial points if we can't get the highest points (Ex. failure during highest points subtree)
        This backup plan plan does not try to align with board but still tries to go to the a
        
        returns pytree.composites.Sequence
        """
        node_partial_points = py_trees.composites.Sequence("partial_pts", memory=True) #TODO: rename this this as well
        generic_align_board = self.factory.make_node_check_align_board("generic") # TODO: rename this
        check_torpedo_count1 = Action("check_torpedo_count1")
        check_torpedo_count2 = Action("check_torpedo_count2")
        fire_torpedo1 = Action("fire_torpedo1")
        fire_torpedo2 = Action("fire_torpedo2")

        node_partial_points.add_children(
            [
            generic_align_board,
            check_torpedo_count1,
            fire_torpedo1,
            check_torpedo_count2,
            fire_torpedo2,
            ]
        )
        
        return node_partial_points

    def node_base_case(self)->py_trees.composites.Sequence:
        """
        Base case subtree that will just try to fire the torpedos without aligning with the board or going to the animals
        
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
