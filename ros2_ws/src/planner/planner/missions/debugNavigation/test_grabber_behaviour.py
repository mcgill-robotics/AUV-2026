# Python dependencies
import py_trees

# Planner dependencies
from ..mission_behaviour_components import SetActuatorBehaviour, TimerBehaviour


class TestGrabberBehaviour(py_trees.composites.Sequence):
    """
    Test mission for the grabber actuator:
    1. Open grabber (0) for 1 sec
    2. Close grabber (1) for 1 sec
    3. Open grabber back (0) for 1 sec
    """

    def __init__(self):
        super().__init__("TestGrabberBehaviour", memory=True)

        open_grabber_1 = SetActuatorBehaviour(
            topic_name="/actuators/grabber",
            command_value=0,
            name="Open Grabber (0)",
        )
        wait_1s_1 = TimerBehaviour(timer_duration=1.0, name="Wait 1s (Grabber Open)")

        close_grabber = SetActuatorBehaviour(
            topic_name="/actuators/grabber",
            command_value=1,
            name="Close Grabber (1)",
        )
        wait_1s_2 = TimerBehaviour(timer_duration=1.0, name="Wait 1s (Grabber Close)")

        open_grabber_2 = SetActuatorBehaviour(
            topic_name="/actuators/grabber",
            command_value=0,
            name="Open Grabber Back (0)",
        )
        wait_1s_3 = TimerBehaviour(timer_duration=1.0, name="Wait 1s (Grabber Open Back)")

        self.add_children([
            open_grabber_1,
            wait_1s_1,
            close_grabber,
            wait_1s_2,
            open_grabber_2,
            wait_1s_3,
        ])
