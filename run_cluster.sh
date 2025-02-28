#!/bin/bash

# Check for gnome-terminal
if ! command -v gnome-terminal &> /dev/null; then
    echo "installing gnome-terminal"
    sudo apt update
    sudo apt install -y gnome-terminal
    if ! command -v gnome-terminal &> /dev/null; then
        echo "gnome-terminal install failed"
        USE_ALTERNATIVE=true
    else
        echo "gnome-terminal installed"
    fi
else
    echo "detected gnome-terminal"
fi

# Get the current working directory
CURRENT_DIR=$(pwd)

# Remove build, install, and log directories
echo "Cleaning build, install, and log directories..."
rm -rf build/ install/ log/

# Build the workspace
echo "Building with colcon..."
colcon build

# Check if build was successful
if [ $? -ne 0 ]; then
    echo "Colcon build failed! Exiting..."
    exit 1
fi

# Ensure setup file exists
if [ ! -f install/setup.bash ]; then
    echo "Error: install/setup.bash not found! Exiting..."
    exit 1
fi

pkill -f "ros2 topic echo"

# Read robot IDs from CSV using Python
echo "Determining required robot nodes..."
ROBOT_LAUNCH_COMMANDS=$(python cluster_robot_id.py)

# Launch robot manager and wait for it to initialize
echo "Launching robot manager..."
gnome-terminal --working-directory="$CURRENT_DIR" -- bash -c "echo Robot Manager; source install/setup.bash; ros2 launch obj_robot_manager obj_robot_manager.launch.py; exec bash"

# Wait for manager to initialize
echo "Waiting for manager to initialize (5 seconds)..."
sleep 2

nohup bash -c "source install/setup.bash && ros2 topic echo /manager/global_start > global_start.log 2>&1" &
echo "Started monitoring /manager/global_start "

# Launch local robot nodes (will register with manager)
echo "Launching local robot nodes..."
eval "$ROBOT_LAUNCH_COMMANDS"

# Wait for robots to register and clusters to be created
echo "Waiting for robots to register and cluster nodes to initialize (5 seconds)..."
sleep 2

# # Launch robot visualizer
# echo "Launching visualization node..."
# gnome-terminal --working-directory="$CURRENT_DIR" -- bash -c "echo Robot Visualizer; source install/setup.bash; ros2 launch obj_robot_visualizer robot_visualizer.launch.py; exec bash"

sleep 2

echo "Launching listening and logging..."

nohup bash -c "source install/setup.bash && ros2 topic echo /manager/robot_states > robot_state.log 2>&1" &
echo "Started monitoring /manager/robot_states"


nohup bash -c "source install/setup.bash && ros2 topic echo /robot_0/state > robot_0_state.log 2>&1" &
echo "Started monitoring /robot_0/state"

nohup bash -c "source install/setup.bash && ros2 topic echo /robot_1/state > robot_1_state.log 2>&1" &
echo "Started monitoring /robot_1/state"

