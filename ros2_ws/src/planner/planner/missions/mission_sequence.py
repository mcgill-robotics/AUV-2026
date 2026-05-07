# Python dependencies
import py_trees
from std_msgs.msg import Int32

# Planner dependencies
from .mission_behaviour_components import MissionChoiceCheckBehaviour, MissionCompleteBehaviour

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


class MissionSequence(py_trees.composites.Sequence):
    """
    The master sequence that holds the mission selector and all available missions.
    """
    def __init__(self, 
                 position_tolerance: float, 
                 yaw_tolerance: float, 
                 hold_time: float, 
                 timeout: float, 
                 orbit_pre_qual_yaw_tolerance_scale: float, 
                 orbit_pre_qual_positional_tolerance_scale: float, 
                 orbit_pre_qual_hold_time_initial: float, 
                 orbit_pre_qual_hold_time_segments: float):
        super().__init__("Mission Master Sequence", memory=True)

        # ---------------------------------------------------------
        # 1. INITIALIZE OLD MISSIONS (Choices 1-7)
        # ---------------------------------------------------------
        pre_qual_orbit = OrbitQualificationMission(yaw_tolerance=yaw_tolerance, position_tolerance=position_tolerance, hold_time=hold_time, timeout=timeout, orbit_pre_qual_yaw_tolerance_scale=orbit_pre_qual_yaw_tolerance_scale, orbit_pre_qual_positional_tolerance_scale=orbit_pre_qual_positional_tolerance_scale, orbit_pre_qual_hold_time_initial=orbit_pre_qual_hold_time_initial, orbit_pre_qual_hold_time_segments=orbit_pre_qual_hold_time_segments)
        pre_qual_rectangle = RectangleQualificationMission(yaw_tolerance=yaw_tolerance, position_tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)
        test_move_forward = TestMoveForwardBehaviour(position_tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)
        test_dive = TestDiveBehaviour(position_tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)
        test_yaw = TestYawBehaviour(yaw_tolerance=yaw_tolerance, hold_time=hold_time, timeout=timeout)
        translation_rectangle = TranslationRectangleMission(position_tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)
        test_service_call = TestServiceCallBehaviour()

        # ---------------------------------------------------------
        # 2. INITIALIZE STANDALONE ROBOSUB 2026 TASKS (Choices 8-12)
        # ---------------------------------------------------------
        def make_standalone(task_node, choice_id, name_prefix):
            """Wraps a pure task so it can be run individually from the menu"""
            seq = py_trees.composites.Sequence(name=f"Standalone: {name_prefix}", memory=True)
            seq.add_children([
                MissionChoiceCheckBehaviour(choice=choice_id, name=f"Check Choice {choice_id}"),
                task_node,
                MissionCompleteBehaviour(name=f"Complete {name_prefix}")
            ])
            return seq

        standalone_gate    = make_standalone(GateTask(position_tolerance, hold_time, timeout), choice_id=8, name_prefix="Gate")
        standalone_slalom  = make_standalone(SlalomTask(position_tolerance, hold_time, timeout), choice_id=9, name_prefix="Slalom")
        standalone_bins    = make_standalone(BinsTask(position_tolerance, hold_time, timeout), choice_id=10, name_prefix="Bins")
        standalone_torpedo = make_standalone(TorpedoTask(position_tolerance, hold_time, timeout), choice_id=11, name_prefix="Torpedo")
        standalone_tab_oct = make_standalone(TableOctagonTask(position_tolerance, hold_time, timeout), choice_id=12, name_prefix="Table & Octagon")

        # ---------------------------------------------------------
        # 3. INITIALIZE FULL COMPETITION RUN (Choice 13)
        # ---------------------------------------------------------
        full_competition_run = py_trees.composites.Sequence(name="FULL COMPETITION RUN", memory=True)
        full_competition_run.add_children([
            MissionChoiceCheckBehaviour(choice=13, name="Check Choice 13 (Full Run)"),
            
            # Instantiate fresh copies of the tasks for the chain
            GateTask(position_tolerance, hold_time, timeout),
            SlalomTask(position_tolerance, hold_time, timeout),
            BinsTask(position_tolerance, hold_time, timeout),
            TorpedoTask(position_tolerance, hold_time, timeout),
            TableOctagonTask(position_tolerance, hold_time, timeout),

            MissionCompleteBehaviour(name="Complete Full Run")
        ])

        # ---------------------------------------------------------
        # 4. COMPILE MISSION LIST & ADD TO TREE
        # ---------------------------------------------------------
        mission_list = [
            pre_qual_orbit, 
            pre_qual_rectangle, 
            test_move_forward, 
            test_dive, 
            test_yaw, 
            translation_rectangle, 
            test_service_call,
            standalone_gate,
            standalone_slalom,
            standalone_bins,
            standalone_torpedo,
            standalone_tab_oct,
            full_competition_run
        ]

        all_missions_selector = py_trees.composites.Selector("Available Missions", memory=True)
        all_missions_selector.add_children(mission_list)

        mission_choice_listener = MissionChoiceBehaviour(name="Wait For User Input", mission_count=len(mission_list))

        self.add_children([
            mission_choice_listener, 
            all_missions_selector
        ])

class MissionChoiceBehaviour(py_trees.behaviour.Behaviour):
    """ 
    This behaviour gets user input for mission choice
    and writes it to the blackboard for other mission behaviours to check.

    Fields: rclpy.Node: node         : the ROS2 node for logging and debugging purposes
    py_trees.blackboard blackboard   : the blackboard client
    mission_count                    : the number of missions to put a boundary on user input
    message_shown                    : boolean to track if the message prompting user input has been shown 
                                       to avoid spamming the console
    """
    def __init__(self, name="MissionChoiceUser", mission_count=0):
        """
		Initializes the MissionChoiceUser behaviour. 

		Inputs:
			str: name        -- The name of the behaviour (default: "userInputYaw")
		"""
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.message_shown = False
        self.mission_count = mission_count
	
    def setup(self, **kwargs):
        """
        Description: Sets up keys on the blackboard that this behaviour will use.
        """
        self.node = kwargs['node']
        # Behaviour Tree bb setup in case of hardware setup or ros2 node setup
        self.blackboard.register_key(key="/mission_choice", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="/mission_choice", access=py_trees.common.Access.READ)
        self.blackboard.mission_choice = None
        
        # Create a ros2 subsriber to listen to user choices
        self.mission_choice_subscriber = self.node.create_subscription(Int32, 
                                                                       "/mission_selector", 
                                                                       self.mission_choice_callback, 
                                                                       10)
    
    def initialise(self):
        """Called every time this behavior transitions is not RUNNING."""
        self.message_shown = False
        self.blackboard.mission_choice = None
    
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
           
        return py_trees.common.Status.SUCCESS
    
    def mission_choice_callback(self, msg: Int32):
        """
        This callback sets the mission_choice key on the blackboard to match 
        the topic's choice

        Input: 
            std_msgs.msg.Int32 : msg - The message sent on the topic /mission_choice representing the user's mission choice
        
        Outputs: None
        """
        if not (0 < msg.data <= self.mission_count):
                self.node.get_logger().warn("Input must be an integer between 1 and {} inclusively!".format(self.mission_count))
                return
        
        self.blackboard.mission_choice = msg.data
        