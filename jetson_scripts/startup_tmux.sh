#!/bin/bash

# tmux attach -t auv_boot

SESSION_NAME="auv_boot"

echo "Waiting 60 seconds before starting the boot sequence..."
for i in {60..1}; do
    echo -ne "Starting in $i seconds...\r"
    sleep 1
done
echo -e "\nStarting boot sequence now."

# Create a new tmux session in detached mode
tmux new-session -d -s $SESSION_NAME

# Helper function for sending ROS 2 commands with workspace setup
run_ros_cmd() {
    local cmd=$1
    echo "source ~/.bashrc && $cmd"
}

# Terminal 1: Launch DVL driver
tmux rename-window -t $SESSION_NAME:0 'dvl'
tmux send-keys -t $SESSION_NAME:0 "$(run_ros_cmd 'ros2 run dvl_a50 dvl_a50_node --ros-args -p ip_address:="192.168.194.95" -p enable_on_activate:="true"')" C-m

# Terminal 2: Lifecycle operations and reset
tmux new-window -t $SESSION_NAME -n 'lifecycle'
# We wait slightly to ensure the DVL node is spun up before configuring
tmux send-keys -t $SESSION_NAME:1 "$(run_ros_cmd 'sleep 5 && ros2 lifecycle set /dvl_a50 configure && ros2 lifecycle set /dvl_a50 activate && ros2 service call /reset_dead_reckoning std_srvs/Trigger {}')" C-m

# Terminal 3: Bringup
tmux new-window -t $SESSION_NAME -n 'bringup'
# Wait 15 seconds so reset_dead_reckoning (at ~sec 9) finishes before bringup
tmux send-keys -t $SESSION_NAME:2 "$(run_ros_cmd 'sleep 15 && ros2 launch bringup bringup.launch.py')" C-m

# Terminal 4: Planner
tmux new-window -t $SESSION_NAME -n 'planner'
# We wait 25 seconds to allow bringup to initialize properly before launching the planner
tmux send-keys -t $SESSION_NAME:3 "$(run_ros_cmd 'sleep 25 && ros2 launch planner planner.launch.py sim:=false')" C-m

# Terminal 5: Mission Configuration
tmux new-window -t $SESSION_NAME -n 'mission'
# We wait 30 seconds for planner and bringup to register their topics
tmux send-keys -t $SESSION_NAME:4 "$(run_ros_cmd 'sleep 30 && ros2 topic pub --once /mission_selector std_msgs/msg/Int32 "{data: 6}"')" C-m

echo "Tmux session '$SESSION_NAME' has been started with the boot sequence."
echo "To view it, run:"
echo "  tmux attach -t $SESSION_NAME"
