from ..vision_behaviours import SearchSweepBehaviour
import py_trees

class GateTask(py_trees.composites.Sequence):
    """
    Sequence for the RoboSub 2026 Gate Task (Begin Assessment).
    The AUV must pass through the gate on a specific side to choose a role:
    - Survey & Repair (Compass/Hammer)
    - Search & Rescue (Lifebuoy/SOS)
    Extra points for 'style' (90 degree orientation changes).
    """
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("Gate Task", memory=True)
        # TODO: Implement Gate Task
        # 1. Detect Gate and role images (Survey & Repair vs Search & Rescue)
        # 2. Choose side/role (e.g. via mission parameter or coin flip)
        # 3. Align and pass through chosen side
        # 4. Optional: Perform style maneuvers (roll/pitch/yaw)
        # 5. Record chosen role to blackboard for subsequent tasks
        self.add_children([
            SearchSweepBehaviour(target_class="gate", num_steps=5, max_attempts=2, step_timeout=0.5, clockwise=False),
            py_trees.behaviours.Success(name="Placeholder Gate Success")
        ])
