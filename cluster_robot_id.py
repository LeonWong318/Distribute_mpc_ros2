import pandas as pd

# Path to the CSV file
csv_path = "./data/test_data/schedule.csv"

# Read CSV file
df = pd.read_csv(csv_path)

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
