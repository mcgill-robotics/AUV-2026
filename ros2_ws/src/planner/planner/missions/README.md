# Missions

A mission is a set of actions and behaviour accomplished by lovely Dougie in the desire to accomplish a task. The missions of the planner are each developped in their own subdirectory. These missions are each a **py_trees Sequence** node, meaning that to succeed all its children nodes must succeed. 

All missions are encapsulted within the MissionSequence Behaviour, which serves as a mission control. This allows the user to repeatedly send out desired missions when one is completed and control what behaviours to test easily.

To write a new mission, create a folder corresponding to it. If many missions fall under the same umbrella, put them under the same folder. For example, for pre-qualification, there is a PreQual folder holding 3 different pre-qualification attempt methods. 

Making a Mission consists of adding checks, navigation, and actuator movements. A basic mission, testing forward movement defined inside the debugNavigation folder, goes as follows:

```python
class TestMoveForwardBehaviour(py_trees.composites.Sequence):
    """
    This PyTrees Sequence is the root of the test translation mission
    """
    def __init__(self, node):
        super().__init__("TestMoveForwardBehaviour", memory=True)

        # Get the general parameters from the configs that were declared in root of Behaviour Tree
        position_tolerance = node.pre_qual_positional_tolerance
        hold_time = node.pre_qual_hold_time
        timeout = node.pre_qual_timeout

        # 0 Check if user input the desired mission choce
        """
        1: Orbit Prequal
        2: Rectangle Prequal
        3: Basic Move forward
        4: Basic Dive
        5: Basic Yaw
        """
        mission_choice_check = MissionChoiceCheckBehaviour(name="Test Move Forward", choice=3)

        # Build the full mission sequence
        # 1. Move Forward
        forward_move_leaf = BasicActionBehaviour(node, "Move to forward", move_robot_centric(forward=1.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        # 2. Reset the user mission choice to allow for new mission to be selected
        mission_choice_reset = MissionCompleteBehaviour(node, "Completed Test Move Forward")

        self.add_children([mission_choice_check, 
            forward_move_leaf, 
            mission_choice_reset
            ])
```


Breaking down the code, we make the mission a class who inherits from the py_trees **Sequence**.
```python
class TestMoveForwardBehaviour(py_trees.composites.Sequence)
```


The next section of code gets the parameters described in the *planner/config* responsible for actions. Factors such as tolerances and hold times for navigation are initialized locally to reflect the config parameters. If your own mission requires an extra set of parameters, make sure to first declare the parameters in the RootTree.py before accessing them here.

```python
position_tolerance = node.pre_qual_positional_tolrance
hold_time = node.pre_qual_hold_time
timeout = node.pre_qual_timeout
```

Now, onto the structure of the actual mission. All missions start with the **MissionChoiceBehaviour**. This class serves to only run the mission when the user input matches the integer corresponding to your mission. This ensures the tree does not go hay wire and execute many missions at once.

```python
mission_choice_check = MissionChoiceBehaviour(name="Test Move Forward", choice=3)
```

Next up is the heart of your mission, here you add all the implentations required to accomplish your task. In this example, the task is simple: it is to move forward. To make any movements with the robot, use the **BasicActionBehaviour** class. As third argument, the class needs a navigation goal. Thankfully, this is abscrated with the functions provided in the controls package in *controls/controls/goal_helpers.py*. In this case the ***move_robot_centric*** function from goal_helpers makes a goal in the perspective of the robot.

```python
forward_move_leaf = BasicActionBehaviour(node, "Move to forward", move_robot_centric(forward=1.0, tolrance=position_tolerance, hold_time=hold_time, timeout=timeout))
```

To finish your mission, reset the choice of which mission to run with the following Behaviour Tree node. 

```python
mission_choice_reset = MissionCompleteBehaviour(node, "Completed Test Move Forward")
```

Add all parts of the mission to the **py_trees Sequence**. The add_children method is provided by the **py_trees Sequence** class.

```python
self.add_children([mission_choice_check, 
            forward_move_leaf, 
            mission_choice_reset
            ])
```

Congrats! You've made a mission, to run it do not forget to add the mission to the **MissionsSequence.py** file. To add it, go to the sequence file and add your mission in these lines

```python
def __init__(self, node):
        super().__init__("MissionSequence", memory=False)

        # Build the full mission sequence
        mission_choice = MissionChoiceBehaviour(node, name="Mission_Choice")
        all_missions_selector = py_trees.composites.Selector("All missions", memory=True)

        pre_qual_rectangle = RectangleQualificationMission(node)
        pre_qual_orbit = OrbitQualificationMission(node)
        test_move_forward = TestMoveForwardBehaviour(node)
        test_dive = TestDiveBehaviour(node)
        test_yaw = TestYawBehaviour(node)
        translation_rectangle = TranslationRectangleMission(node)
        test_service_call = TestServiceCallBehaviour(node)
        YOUR_MISSION = YOUR_MISSION_CLASS(node)
        
        mission_list = [pre_qual_orbit, 
            pre_qual_rectangle, 
            test_move_forward, 
            test_dive, 
            test_yaw, 
            translation_rectangle, 
            test_service_call,
            YOUR_MISSION]

        # Count the number of missions to put a boundary on user input
        self.mission_count = len(mission_list)

        all_missions_selector.add_children(mission_list)
        self.add_children([mission_choice, 
            all_missions_selector])
```

Finally, to make documentation consistent, update the README.MD of the package with the new available missions and MissionSequence.py self.update method 

```python
# Note that after a mission is completed, the respective mission will set this key back to None
        if self.blackboard.mission_choice is None:
            if not self.message_shown:
                self.node.get_logger().info("Waiting for mission choice. Publish once an Int32 to topic /mission_selector ...\n \
                            1: Orbit Prequal\n \
                            2: Rectangle Prequal\n \
                            3: Basic Move forward (1.0m relative to Dougie)\n \
                            4: Basic Dive (Down 1.5m)\n \
                            5: Basic Yaw (180 deg)\n \
                            6: Translation Rectangle (no yaw)\n \
                            7: Test Service Call (reset dead reckoning)\n \
                            8: YOUR MISSION")
                self.message_shown = True
            return py_trees.common.Status.RUNNING
```


