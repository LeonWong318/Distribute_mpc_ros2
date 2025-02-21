#!/bin/bash

# Get the current working directory
CURRENT_DIR=$(pwd)

# Remove build, install, and log directories
echo "Cleaning build, install, and log directories..."
rm -rf build/ install/ log/

# Build the workspace
echo "Building with colcon..."
colcon build

# Check if the build was successful before proceeding
if [ $? -ne 0 ]; then
    echo "Colcon build failed! Exiting..."
    exit 1
fi

# Ensure setup file exists
if [ ! -f install/setup.bash ]; then
    echo "Error: install/setup.bash not found! Exiting..."
    exit 1
fi

# Open 3 terminals and run ROS2 launch commands
echo "Launching ROS2 nodes in 3 terminals..."

gnome-terminal --working-directory="$CURRENT_DIR" -- bash -c "source install/setup.bash; ros2 launch manager manager.launch.py; exec bash"
sleep 1  # Small delay to ensure proper launch order

gnome-terminal --working-directory="$CURRENT_DIR" -- bash -c "source install/setup.bash; ros2 launch robot robot.launch.py robot_id:=1; exec bash"
sleep 1  # Delay for stability

gnome-terminal --working-directory="$CURRENT_DIR" -- bash -c "source install/setup.bash; ros2 launch robot robot.launch.py robot_id:=0; exec bash"

# Echo topic in current terminal and save output to file
echo "Echoing /manager/robot_states and saving to robot_state.log..."
source install/setup.bash
ros2 topic echo /manager/robot_states > robot_state.log
