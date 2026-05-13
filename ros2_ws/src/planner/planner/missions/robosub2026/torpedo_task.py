import py_trees


class TorpedoTask(py_trees.composites.Sequence):
    """
    Sequence for the RoboSub 2026 Torpedo Task (Deploy).
    AUV fires torpedoes through openings in a board.
    Targets match role (Fire/Firetruck vs Blood/Ambulance).
    Sequence: Large opening then Small opening.
    """
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("Torpedo Task", memory=True)
        # TODO: Implement Torpedo Task
        # 1. Locate Torpedo board via acoustic localization or vision
        # 2. Identify large and small openings matching role
        # 3. Align with Large opening and fire torpedo
        # 4. Align with Small opening and fire torpedo
        # 5. Optional: Maintain distance (1ft or 1.5ft) for extra points
        self.add_children([
            py_trees.behaviours.Success(name="Placeholder Torpedo Success")
        ])
