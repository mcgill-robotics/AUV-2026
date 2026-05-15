import py_trees


class BinsTask(py_trees.composites.Sequence):
    """
    Sequence for the RoboSub 2026 Bins Task (Recon).
    AUV must locate bins on a 3D pipeline and drop markers into role-matching bins.
    Targets: Fire (Survey & Repair) vs Blood (Search & Rescue).
    """
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("Bins Task", memory=True)
        # TODO: Implement Bins Task
        # 1. Search for and locate 3D pipeline/bins structure
        # 2. Identify bins matching current role (Fire vs Blood)
        # 3. Align over correct bin
        # 4. Drop marker(s)
        # 5. Optional: Detect light and turn off via magnetic detector
        self.add_children([
            py_trees.behaviours.Success(name="Placeholder Bins Success")
        ])
