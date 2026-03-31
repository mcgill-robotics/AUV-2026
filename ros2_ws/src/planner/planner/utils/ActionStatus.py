from enum import Enum

class ActionStatus(Enum):
    """
    Description: An enumeration used to keep track of state of Action Behaviours
    """
    SUCCEEDED = 0
    FAILED = 1
    PENDING = 2
    NOT_SENT = 3