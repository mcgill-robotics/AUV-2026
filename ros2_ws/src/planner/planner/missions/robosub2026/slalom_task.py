import py_trees


class SlalomTask(py_trees.composites.Sequence):
    """
    Sequence for the RoboSub 2026 Slalom Task (Avoid Debris).
    AUV must navigate 3 sets of vertical pipes: White (left), Red (middle), White (right).
    Must stay on the same side of the Red pipe as the gate divider side.
    """
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("Slalom Task", memory=True)
        # TODO: Implement Slalom Task
        # 1. Detect next set of Slalom pipes
        # 2. Identify Red pipe vs White pipes
        # 3. Navigate through the set on the correct side of Red
        # 4. Repeat for all 3 sets
        # 5. Maintain correct depth (stay within plane of pipes)
        self.add_children([
            py_trees.behaviours.Success(name="Placeholder Slalom Success")
        ])
