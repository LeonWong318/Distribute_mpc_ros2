import pandas as pd
import sys
import yaml
import os

sys_config_path = "./config/sys_config.yaml"

if not os.path.exists(sys_config_path):
    print(f"Error: Configuration file {sys_config_path} not found.", file=sys.stderr)
    sys.exit(1)
try:
    with open(sys_config_path, 'r') as f:
        config = yaml.safe_load(f)
        if config is None:
            print(f"Error: Configuration file {sys_config_path} is empty or invalid.", file=sys.stderr)
            sys.exit(1)
except Exception as e:
    print(f"Error reading configuration file {sys_config_path}: {str(e)}", file=sys.stderr)
    sys.exit(1)

schedule_path = os.path.join('./', str(config.get('robot_spec_path','data/test_data/schedule.csv'))) 

# Read CSV file
df = pd.read_csv(schedule_path)

# Get unique robot IDs
robot_ids = df["robot_id"].unique()

# Print the number of robots
print(f"Detected {len(robot_ids)} robots: {robot_ids}")

# Generate ROS2 launch commands
commands = []
for robot_id in robot_ids:
    cmd = (
        f'gnome-terminal --working-directory="$CURRENT_DIR" -- bash -c "echo Robot ID: {robot_id}; '
        f'source install/setup.bash; ros2 launch obj_local_robot obj_local_robot.launch.py robot_id:={robot_id}; exec bash"'
    )
    commands.append(cmd)

# Print launch commands
for cmd in commands:
    print(cmd)
