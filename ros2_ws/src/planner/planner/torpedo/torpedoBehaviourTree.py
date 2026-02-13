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

def make_node_align_board()->py_trees.composites.Sequence:
    """
    Node to orient to board, match z point, and move to xy point. 
    Sequence: ensure all actions are done in a specific order.
    This is a sub tree that will be called in the check align board selector node.
    
    returns: py_trees.composites.Sequence
    """
    node_align_board = py_trees.composites.Sequence("sequence", memory=False)
    orient_with_middle = Action("orient_with_middle")
    match_z_point = Action("match_z_point")
    move_to_xy = Action("move_to_xy")
    node_align_board.add_children(
        [
            orient_with_middle, 
            match_z_point, 
            move_to_xy
        ]
    )
    return node_align_board

def make_node_check_align_board()->py_trees.composites.Selector:
    """
    Node to check if the board is in view.
    Selector: if the board is in view, we are done, if not, we need to orient to the board (call the align board sequence node)
    
    returns: py_trees.composites.Selector
    """
    node_check_align_board = py_trees.composites.Selector("check_align_board", memory=False)
    check_board_in_view = Condition("check_board_in_view")
    node_check_align_board.add_children(
        [
            check_board_in_view, 
            make_node_align_board()
        ]
    )
    return node_check_align_board

def make_node_go_to_animal()->py_trees.composites.Sequence:
    """
    Node to make Dougie go to the animal.
    Sequence: ensure board is in frame, then align xfeat.

    retruns: py_trees.composites.Sequence
    """
    node_go_to_animal = py_trees.composites.Sequence("go_to_animal", memory=False)
    align_xfeat = Action("align_xfeat")
    node_go_to_animal.add_children(
        [
            align_xfeat,
            make_node_check_align_board()
        ]
    )
    return node_go_to_animal

def make_node_highest_pts()->py_trees.composites.Sequence: # rename this to something better
    """
    First subtree to get the hightest amount of points.
    Sequence: get the chosen animal, go to chosen animal, fire torpedo 1, go to other animal, fire torpedo 2.

    returns py_trees.composites.Sequence
    """
    node_highest_pts = py_trees.composites.Sequence("highest_pts", memory=False)
    get_chosen_animal = Action("get_chosen_animal")
    go_to_animal1 = make_node_go_to_animal()
    go_to_animal2 = make_node_go_to_animal()
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