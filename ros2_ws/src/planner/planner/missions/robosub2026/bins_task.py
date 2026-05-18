import math
import py_trees
import planner.missions.vision_behaviours as vision_behaviours
import planner.missions.mission_behaviour_components as basic_behaviours
from controls.goal_helpers import move_global

class BinsTask(py_trees.composites.Sequence):
    """
    Sequence for the RoboSub 2026 Bins Task (Recon).
    AUV must locate bins on a 3D pipeline and drop markers into role-matching bins.
    Targets: Fire (Survey & Repair) vs Blood (Search & Rescue).
    """
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("Bins Task", memory=True)

        self.blackboard = self.attach_blackboard_client(name="BinsTaskBlackboard")
        # TODO: Implement Bins Task
        # 1. Search for and locate 3D pipeline/bins structure
        search_for_bin_structure = vision_behaviours.SearchSweepBehaviour(
            target_class="bin_structure",
            num_steps=8,
            max_attempts=2,
            step_timeout=0.5)
        
        # 2. Go to bin structure
        go_near_bin_structure

        # 2. Find the closest bin
        search_for_bins = vision_behaviours.SearchSweepBehaviour(
            target_class="bin",
            num_steps=8,
            max_attempts=2,
            step_timeout=0.5)
        # 3. Find the closest bin
        search_closest_bin = SearchClosestObject(target_class="bin")
        
        # 4. Go over that bin
        go_to_closest_bin = GoAboveClosestBin()
        # 4. Drop marker(s)
        # 5. Optional: Detect light and turn off via magnetic detector
        self.add_children([
            py_trees.behaviours.Success(name="Placeholder Bins Success")
        ])

class SearchClosestObject(py_trees.behaviour.Behaviour):
    """
    Searches the blackboard's vision object map for the closest object of a specified class.
    If found, it sets the blackboard variable '/bins_task/object_name' to that object and returns SUCCESS.
    If no object of the target class is found, it returns FAILURE.
    """
    def __init__(self, target_class: str):
        super().__init__("SearchClosestObject")
        self.target_class = target_class
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="/bins_task/closest_bin", access=py_trees.common.Access.WRITE)

    def update(self):

        
        return py_trees.common.Status.FAILURE

class GoAboveClosestBin(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("GoAboveClosestBin")
        self.blackboard = self.attach_blackboard_client(name=self.name)

        # search through all the bins and find the closest one
        if hasattr(self.blackboard, 'vision') and self.blackboard.vision.object_map is not None:
            closest_bin = None
            closest_distance = float('inf')
            pose = self.blackboard.sensors.pose.pose.position
            for obj in self.blackboard.vision.object_map.array:
                if obj.label == "bin":
                    # Calculate distance from AUV to object (assuming AUV pose is at origin)
                    distance_squared = (obj.position.x - pose.x)**2 + (obj.position.y - pose.y)**2 + (obj.position.z - pose.z)**2
                    if distance_squared < closest_distance:
                        closest_distance = distance_squared
                        closest_bin = obj
            
            if closest_bin is None:
                return py_trees.common.Status.FAILURE
            
            self.closest_bin = closest_bin
    
        target_position = self.closest_bin.position
        self.goal_node = move_global(target_position.x, target_position.y, target_position.z + 0.5, 0.0)
        
    def update(self):
        
        
        return py_trees.common.Status.FAILURE