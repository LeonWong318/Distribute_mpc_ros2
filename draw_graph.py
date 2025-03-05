import re
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path


def parse_robot_log(file_path):
    """
    Parse robot state log file
    Returns lists of timestamps, x positions, and y positions
    """
    with open(file_path, 'r') as f:
        content = f.read()
    
    # Split records using "---" delimiter
    entries = content.split("---")
    
    timestamps = []
    x_positions = []
    y_positions = []
    
    for entry in entries:
        if not entry.strip():
            continue
        
        # Extract positions
        x_match = re.search(r'x: ([-\d\.]+)', entry)
        y_match = re.search(r'y: ([-\d\.]+)', entry)
        
        # Extract timestamp
        sec_match = re.search(r'sec: (\d+)', entry)
        nanosec_match = re.search(r'nanosec: (\d+)', entry)
        
        if x_match and y_match and sec_match and nanosec_match:
            x = float(x_match.group(1))
            y = float(y_match.group(1))
            sec = int(sec_match.group(1))
            nanosec = int(nanosec_match.group(1))
            
            # Calculate complete timestamp (seconds)
            timestamp = sec + nanosec / 1e9
            
            timestamps.append(timestamp)
            x_positions.append(x)
            y_positions.append(y)
    
    return timestamps, x_positions, y_positions


def calculate_euclidean_velocities(timestamps, x_positions, y_positions):
    """
    Calculate euclidean velocities based on timestamps and positions
    """
    euclidean_velocities = []
    
    # Need at least two points to calculate velocity
    if len(timestamps) < 2:
        return [], []
    
    # Calculate velocity between adjacent points
    for i in range(1, len(timestamps)):
        dt = timestamps[i] - timestamps[i-1]
        dx = x_positions[i] - x_positions[i-1]
        dy = y_positions[i] - y_positions[i-1]
        
        # Avoid division by zero
        if dt > 0:
            # Euclidean velocity (linear speed)
            euclidean_distance = np.sqrt(dx**2 + dy**2)
            euclidean_velocity = euclidean_distance / dt
            euclidean_velocities.append(euclidean_velocity)
        else:
            euclidean_velocities.append(0)
    
    # Velocity sequence has one less element than the original sequence
    vel_timestamps = timestamps[1:]
    
    return vel_timestamps, euclidean_velocities


def main():
    robot_files = ["robot_0_state.log", "robot_1_state.log"]
    colors = ["blue", "red"]
    labels = ["Robot 0", "Robot 1"]
    
    # Create figure
    plt.figure(figsize=(10, 6))
    
    for i, file_name in enumerate(robot_files):
        try:
            # Check if file exists
            if not Path(file_name).exists():
                print(f"Warning: File {file_name} does not exist, skipped")
                continue
            
            # Parse log file
            timestamps, x_positions, y_positions = parse_robot_log(file_name)
            
            # Calculate velocities
            vel_timestamps, euclidean_velocities = calculate_euclidean_velocities(
                timestamps, x_positions, y_positions
            )
            
            # Normalize timestamps to make the first timestamp 0
            if vel_timestamps:
                start_time = vel_timestamps[0]
                normalized_timestamps = [t - start_time for t in vel_timestamps]
                
                # Plot euclidean velocity
                plt.plot(normalized_timestamps, euclidean_velocities, 
                         color=colors[i], label=f"{labels[i]}")
        
        except Exception as e:
            print(f"Error processing file {file_name}: {e}")
    
    # Configure plot
    plt.title('Euclidean (Linear) Velocity over Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Euclidean Velocity (units/second)')
    plt.grid(True)
    plt.legend()
    
    # Display chart
    plt.show()


if __name__ == "__main__":
    main()