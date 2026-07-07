# Python dependencies
import py_trees

# Planner dependencies
from ..mission_behaviour_components import SetActuatorBehaviour, TimerBehaviour


class TestTorpedoBehaviour(py_trees.composites.Sequence):
    """
    Test mission for the torpedo actuator:
    1. Close torpedo (0)
    2. Open first torpedo (1)
    3. Open second torpedo (2)
    4. Close torpedo (0)
    Each step waits 1 sec.
    """

    def __init__(self):
        super().__init__("TestTorpedoBehaviour", memory=True)

        close_torpedo_1 = SetActuatorBehaviour(
            topic_name="/actuators/torpedo",
            command_value=0,
            name="Close Torpedo (0)",
        )
        wait_1s_1 = TimerBehaviour(timer_duration=1.0, name="Wait 1s (Torpedo Close 1)")

        open_first = SetActuatorBehaviour(
            topic_name="/actuators/torpedo",
            command_value=1,
            name="Open First Torpedo (1)",
        )
        wait_1s_2 = TimerBehaviour(timer_duration=1.0, name="Wait 1s (Torpedo Open First)")

        open_second = SetActuatorBehaviour(
            topic_name="/actuators/torpedo",
            command_value=2,
            name="Open Second Torpedo (2)",
        )
        wait_1s_3 = TimerBehaviour(timer_duration=1.0, name="Wait 1s (Torpedo Open Second)")

        close_torpedo_2 = SetActuatorBehaviour(
            topic_name="/actuators/torpedo",
            command_value=0,
            name="Close Torpedo Back (0)",
        )
        wait_1s_4 = TimerBehaviour(timer_duration=1.0, name="Wait 1s (Torpedo Close Final)")

        self.add_children([
            close_torpedo_1,
            wait_1s_1,
            open_first,
            wait_1s_2,
            open_second,
            wait_1s_3,
            close_torpedo_2,
            wait_1s_4,
        ])
