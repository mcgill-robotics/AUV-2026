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
from .debugNavigation.comprehensive_test_mission import ComprehensiveTestMission
from .debugNavigation.test_style_yaw_spin import TestStyleYawSpin
from .debugNavigation.test_style_rolling_flip import TestStyleRollingFlip
from .debugNavigation.test_grabber_behaviour import TestGrabberBehaviour
from .debugNavigation.test_torpedo_behaviour import TestTorpedoBehaviour

# --- NEW ROBOSUB 2026 TASKS ---
from .robosub2026.gate_task import GateTask
from .robosub2026.slalom_task import SlalomTask
from .robosub2026.bins_task import BinsTask
from .robosub2026.torpedo_task import TorpedoTask
from .robosub2026.table_octagon_task import TableOctagonTask
from .robosub2026.return_home_task import ReturnHomeTask

from .mission_behaviour_components import RosbagRecordingDecorator, TimerBehaviour, SetNodeParameterBehaviour, TaskTransitionBehaviour

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
                    "  8: Comprehensive Lane Test\n"
                    "--- Style Maneuver Tests ---\n"
                    "  9: Test Style Yaw Spin (360 deg)\n"
                    " 10: Test Style Rolling Flip (720 deg)\n"
                    "--- Actuator Tests ---\n"
                    " 11: Test Grabber (open 1s, close 1s, open 1s)\n"
                    " 12: Test Torpedo (close 0, open 1, open 2, close 0)\n"
                    "--- RoboSub 2026 Tasks ---\n"
                    " 13: Gate Task\n"
                    " 14: Slalom Task\n"
                    " 15: Bins Task\n"
                    " 16: Torpedo Task\n"
                    " 17: Table & Octagon Task\n"
                    " 18: Return Home Task\n"
                    "--- Full Run ---\n"
                    " 19: FULL COMPETITION RUN"
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

            # Apply auto-recording decorator if enabled
            auto_record_params = self.params.get('auto_record_params', {})
            if auto_record_params.get('enabled', True):
                profile = auto_record_params.get('profile', 'all')
                prefix = auto_record_params.get('bag_prefix', 'mission_')
                service_path = auto_record_params.get('service_path', '/rosbag_manager/control')
                
                # Format the mission's name (e.g. "Slalom Task" -> "slalom_task")
                mission_name_formatted = mission_root.name.lower().replace(' ', '_')
                bag_name = f"{prefix}{mission_name_formatted}"
                
                mission_root = RosbagRecordingDecorator(mission_root, profile=profile, bag_name=bag_name, service_path=service_path)

            # Create the startup sequence to allow untethering and enabling controllers
            startup_delay = self.params.get('startup_delay', 15.0)
            startup_sequence = py_trees.composites.Sequence("Startup Sequence", memory=True)
            if startup_delay > 0.0:
                startup_sequence.add_child(TimerBehaviour(timer_duration=startup_delay, name=f"Startup Delay ({startup_delay}s)"))
            
            startup_sequence.add_children([
                SetNodeParameterBehaviour("/attitude_controller", "enabled", True, name="Enable Attitude Controller"),
                SetNodeParameterBehaviour("/depth_controller", "enabled", True, name="Enable Depth Controller"),
                SetNodeParameterBehaviour("/x_controller", "enabled", True, name="Enable X Controller"),
                SetNodeParameterBehaviour("/y_controller", "enabled", True, name="Enable Y Controller")
            ])
            py_trees.trees.setup(root=startup_sequence, node=self.node, shared_nav_client=self.nav_client)
            self.placeholder.add_child(startup_sequence)

            # Setup the new dynamically created subtree and attach it after the startup sequence
            py_trees.trees.setup(root=mission_root, node=self.node, shared_nav_client=self.nav_client)
            self.placeholder.add_child(mission_root)
            self.mission_loaded = True
            self.node.get_logger().info(f"[{self.name}] Successfully loaded and started mission {choice}!")

        # Once loaded, we return SUCCESS so the Sequence moves on to execute the placeholder
        return py_trees.common.Status.SUCCESS

    def _build_mission(self, choice: int):
        p = self.params
        slalom = p.get('slalom_params', {})
        torpedo = p.get('torpedo_params', {})
        gate = p.get('gate_params', {})
        bins = p.get('bins_params', {})
        octagon = p.get('octagon_params', {})
        return_home = p.get('return_home_params', {})
        transitions = p.get('transitions_params', {})
        if choice == 1:
            return OrbitQualificationMission(p['angular_tolerance'], p['position_tolerance'], p['hold_time'], p['timeout'], p['orbit_pre_qual_angular_tolerance_scale'], p['orbit_pre_qual_positional_tolerance_scale'], p['orbit_pre_qual_hold_time_initial'], p['orbit_pre_qual_hold_time_segments'])
        elif choice == 2:
            return RectangleQualificationMission(p['angular_tolerance'], p['position_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 3:
            return TestMoveForwardBehaviour(p['position_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 4:
            return TestDiveBehaviour(p['position_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 5:
            return TestYawBehaviour(p['angular_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 6:
            return TranslationRectangleMission(p['position_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 7:
            return TestServiceCallBehaviour()
        elif choice == 8:
            return ComprehensiveTestMission(p['position_tolerance'], p['hold_time'], p['timeout'])
        elif choice == 9:
            return TestStyleYawSpin()
        elif choice == 10:
            return TestStyleRollingFlip(roll_torque=gate.get('style_roll_torque', 15.0))
        elif choice == 11:
            return TestGrabberBehaviour()
        elif choice == 12:
            return TestTorpedoBehaviour()
        elif choice == 13:
            return GateTask(**gate)
        elif choice == 14:
            return SlalomTask(**slalom)
        elif choice == 15:
            return BinsTask(**bins)
        elif choice == 16:
            return TorpedoTask(**torpedo)
        elif choice == 17:
            return TableOctagonTask(**octagon)
        elif choice == 18:
            return ReturnHomeTask(**return_home)
        elif choice == 19:
            full_run = py_trees.composites.Sequence("FULL COMPETITION RUN", memory=True)
            full_run.add_children([
                GateTask(**gate),
                TaskTransitionBehaviour(name="Gate -> Slalom Transition", **transitions.get('after_gate', {})),
                SlalomTask(**slalom),
                TaskTransitionBehaviour(name="Slalom -> Bins Transition", **transitions.get('after_slalom', {})),
                BinsTask(**bins),
                TaskTransitionBehaviour(name="Bins -> Torpedo Transition", **transitions.get('after_bins', {})),
                TorpedoTask(**torpedo),
                TaskTransitionBehaviour(name="Torpedo -> Octagon Transition", **transitions.get('after_torpedo', {})),
                TableOctagonTask(**octagon),
                TaskTransitionBehaviour(name="Octagon -> Return Home Transition", **transitions.get('after_octagon', {})),
                ReturnHomeTask(**return_home)
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