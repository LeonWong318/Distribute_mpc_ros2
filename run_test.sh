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

# Check for xdotool
if ! command -v xdotool &> /dev/null; then
    echo "installing xdotool"
    sudo apt update
    sudo apt install -y xdotool
    if ! command -v xdotool &> /dev/null; then
        echo "xdotool install failed"
        USE_ALTERNATIVE=true
    else
        echo "xdotool installed"
    fi
else
    echo "detected xdotool"
fi

# Check for imagemagick
if ! ( command -v import &> /dev/null && command -v convert &> /dev/null ); then
    echo "installing imagemagick"
    sudo apt update
    sudo apt install -y imagemagick
    if ! ( command -v import &> /dev/null && command -v convert &> /dev/null ); then
        echo "imagemagick install failed"
        USE_ALTERNATIVE=true
    else
        echo "imagemagick installed"
    fi
else
    echo "detected imagemagick"
fi

# Get the current working directory
CURRENT_DIR=$(pwd)

# restart network status
echo "Restarting Network"
sudo NetworkManager restart

# Remove build, install, and log directories
echo "Cleaning build, install, and log directories..."
rm -rf build/ install/ log/ *.log *.out

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

# Launch robot visualizer
echo "Launching auto test node..."
gnome-terminal --working-directory="$CURRENT_DIR" -- bash -c "echo auto test; source install/setup.bash; ros2 launch obj_auto_test obj_auto_test.launch.py; exec bash"
