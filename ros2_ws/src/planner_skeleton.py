import typing
import py_trees

# this is only skeleton for the correct yaw node where 
#     SUCCESS  -> yaw needs correction
#     FAILURE  -> yaw is within acceptable bounds

class YawErrorAboveThreshold(py_trees.behaviour.Behaviour):

    def __init__(self, name: str, threshold: float) -> None:
        super().__init__(name)
        self.threshold = threshold # we need to set this to whatever we decide is acceptable for yaw error. Maybe this could depend on the task too. 
        
        # not to be confused with setup function.
        # from documentation: 
        # Minimal one-time initialisation.

        # A good rule of thumb is to only include the initialisation relevant
        # for being able to insert this behaviour in a tree for offline rendering to dot graphs.

        # Other one-time initialisation requirements should be met via
        # the setup() method.

    def setup(self, **kwargs: typing.Any) -> None:
        # Placeholder for sensor or state interface setup for example. 
        pass

    def initialise(self) -> None:
        # Reset or cache values if needed
        self.feedback_message = ""

    def update(self) -> py_trees.common.Status:
        # TODO: replace with actual yaw error computation
        yaw_error = 0.0  # mock value

        if abs(yaw_error) > self.threshold:
            self.feedback_message = "Yaw error above threshold"
            return py_trees.common.Status.SUCCESS # here the way it is designed is that SUCCESS means that yaw error is above threshold. so you need to fix.
        else:
            self.feedback_message = "Yaw within tolerance" # FAILURE actually means the yaw error is 0 or below threshold. so you don't need to fix.
            return py_trees.common.Status.FAILURE

# next is the action node from the above condition skeleton 
#     Action node.
#     RUNNING -> actively rotating
#     SUCCESS -> yaw corrected


class RotateYaw(py_trees.behaviour.Behaviour):

    def __init__(self, name: str) -> None:
        super().__init__(name)

    def setup(self, **kwargs: typing.Any) -> None:
        # Placeholder for thruster/controller setup
        pass

    def initialise(self) -> None:
        # Kick off yaw rotation command
        self.feedback_message = "Starting yaw correction"

    def update(self) -> py_trees.common.Status:
        # TODO: monitor rotation progress because parallel task 
        yaw_corrected = False  # this is an example of a condition that you need to monitor. you need to monitor the rotation progress.

        if not yaw_corrected:
            self.feedback_message = "Rotating to correct yaw"
            return py_trees.common.Status.RUNNING
        else:
            self.feedback_message = "Yaw corrected"
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        # Stop rotation if interrupted or finished
        pass


# note that feedback messages aren't mandatory for usage but recommended in documentation: https://py-trees.readthedocs.io/en/devel/behaviours.html

# below here we have the subtree for if we get SUCCESS (yaw error is above threshold), and the tree dictates we then need to correct the yaw by rotating 
def create_correct_yaw_subtree() -> py_trees.behaviour.Behaviour:

    yaw_check = YawErrorAboveThreshold(
        name="Yaw Error > Threshold?",
        threshold=0.1,  # radians (example)
    )

    rotate_yaw = RotateYaw(
        name="Rotate Yaw"
    )

    correct_yaw_sequence = py_trees.composites.Sequence(
        name="Correct Yaw",
        memory=False,
        children=[yaw_check, rotate_yaw],
    )

    return correct_yaw_sequence


# there was debate on whether or not this is a sequence or a parallel. 
# on one hand parallel makes sense because you want continuous feedback from the yaw check 
# however im not sure anymore because you might want to wait for the rotation to complete before you can check the yaw error again.
# and sequence allows you to do that, and makes sense because you tick the right child if the left child returns SUCCESS (over error threshold)

