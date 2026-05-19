import py_trees
from std_msgs.msg import Int32

# --- OLD MISSIONS (Pre-Quals & Tests) ---
from .preQual.orbit_qualification import OrbitQualificationMission
from .preQual.rectangle_with_yaw_qualification import RectangleQualificationMission
from .preQual.rectangle_no_yaw_qualification import TranslationRectangleMission
from .debugNavigation.test_move_forward_behaviour import TestMoveForwardBehaviour
from .debugNavigation.test_yaw_behaviour import TestYawBehaviour
from .debugNavigation.test_dive_behaviour import TestDiveBehaviour
from .debugNavigation.test_serivce_call_behaviour import TestServiceCallBehaviour

# --- NEW ROBOSUB 2026 TASKS ---
from .robosub2026.gate_task import GateTask
from .robosub2026.slalom_task import SlalomTask
from .robosub2026.bins_task import BinsTask
from .robosub2026.torpedo_task import TorpedoTask
from .robosub2026.table_octagon_task import TableOctagonTask

class MissionSpawner(py_trees.behaviour.Behaviour):
    def __init__(self, placeholder: py_trees.composites.Sequence, **kwargs):
        super().__init__("Mission Spawner")
        self.placeholder = placeholder
        self.params = kwargs
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.mission_loaded = False
        self.message_shown = False

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.nav_client = kwargs['shared_nav_client']
        
        self.blackboard.register_key("/mission_choice", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("/mission_choice", access=py_trees.common.Access.READ)
        
        # Subscribe to the topic natively here
        self.subscriber = self.node.create_subscription(
            Int32, "/mission_selector", self.choice_callback, 10
        )

    def choice_callback(self, msg: Int32):
        self.blackboard.mission_choice = msg.data

    def initialise(self):
        # This is called whenever the sequence restarts (i.e., a mission finishes or fails)
        self.blackboard.mission_choice = None
        self.mission_loaded = False
        self.message_shown = False
        self.placeholder.remove_all_children() # Dynamically shrink the tree!

    def update(self):
        if self.blackboard.mission_choice is None:
            if not self.message_shown:
                menu_text = (
                    "Waiting for mission choice. Publish an Int32 to /mission_selector:\n"
                    "--- Pre-Quals & Tests ---\n"
                    "  1: Orbit Prequal\n"
                    "  2: Rectangle Prequal\n"
                    "  3: Basic Move forward (1.0m relative)\n"
                    "  4: Basic Dive (Down 1.5m)\n"
                    "  5: Basic Yaw (180 deg)\n"
                    "  6: Translation Rectangle (no yaw)\n"
                    "  7: Test Service Call (reset dead reckoning)\n"
                    "--- RoboSub 2026 Tasks ---\n"
                    "  8: Gate Task\n"
                    "  9: Slalom Task\n"
                    " 10: Bins Task\n"
                    " 11: Torpedo Task\n"
                    " 12: Table & Octagon Task\n"
                    "--- Full Run ---\n"
                    " 13: FULL COMPETITION RUN"
                )
                self.node.get_logger().info(menu_text)
                self.message_shown = True
            return py_trees.common.Status.RUNNING

        if not self.mission_loaded:
            choice = self.blackboard.mission_choice
            mission_root = self._build_mission(choice)

            if mission_root is None:
                self.node.get_logger().warn(f"[{self.name}] Invalid Choice: {choice}")
                self.blackboard.mission_choice = None
                self.message_shown = False
                return py_trees.common.Status.RUNNING

            # Setup the new dynamically created subtree and attach it
            py_trees.trees.setup(root=mission_root, node=self.node, shared_nav_client=self.nav_client)
            self.placeholder.add_child(mission_root)
            self.mission_loaded = True
            self.node.get_logger().info(f"[{self.name}] Successfully loaded and started mission {choice}!")

        # Once loaded, we return SUCCESS so the Sequence moves on to execute the placeholder
        return py_trees.common.Status.SUCCESS

    def _build_mission(self, choice: int):
        p = self.params
        slalom = p.get('slalom_params', {})
        if choice == 1:
            return OrbitQualificationMission(p['yaw_tolerance'], p['position_tolerance'], p['hold_time'], p['timeout'], p['orbit_pre_qual_yaw_tolerance_scale'], p['orbit_pre_qual_positional_tolerance_scale'], p['orbit_pre_qual_hold_time_initial'], p['orbit_pre_qual_hold_time_segments'])
        elif choice == 2:
            return RectangleQualificationMission(p['yaw_tolerance'], p['position_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 3:
            return TestMoveForwardBehaviour(p['position_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 4:
            return TestDiveBehaviour(p['position_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 5:
            return TestYawBehaviour(p['yaw_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 6:
            return TranslationRectangleMission(p['position_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 7:
            return TestServiceCallBehaviour()
        elif choice == 8:
            return GateTask(p['position_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 9:
            return SlalomTask(**slalom)
        elif choice == 10:
            return BinsTask(p['position_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 11:
            return TorpedoTask(p['position_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 12:
            return TableOctagonTask(p['position_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 13:
            full_run = py_trees.composites.Sequence("FULL COMPETITION RUN", memory=True)
            full_run.add_children([
                GateTask(p['position_tolerance'], p['hold_time'], p['timeout']),
                SlalomTask(**slalom),
                BinsTask(p['position_tolerance'], p['hold_time'], p['timeout']),
                TorpedoTask(p['position_tolerance'], p['hold_time'], p['timeout']),
                TableOctagonTask(p['position_tolerance'], p['hold_time'], p['timeout']),
            ])
            return full_run
        return None

class DynamicMissionSequence(py_trees.composites.Sequence):
    def __init__(self, **kwargs):
        super().__init__("Mission Runner", memory=True)
        
        # The placeholder where missions will be injected and removed dynamically
        self.active_mission_placeholder = py_trees.composites.Sequence("Active Mission", memory=True)
        
        # The spawner that routes the choice to the placeholder
        self.spawner = MissionSpawner(self.active_mission_placeholder, **kwargs)

        self.add_children([self.spawner, self.active_mission_placeholder])