import matplotlib.pyplot as plt
import re
import numpy as np
from matplotlib.patches import Arrow

def parse_real_state_log(filename):
    """Parse robot_0_real_state.log file."""
    with open(filename, 'r') as f:
        content = f.read()
    
    entries = content.split('---')
    x_coords = []
    y_coords = []
    thetas = []
    timestamps = []
    
    for entry in entries:
        if not entry.strip():
            continue
        
        x_match = re.search(r'x: ([-\d\.]+)', entry)
        y_match = re.search(r'y: ([-\d\.]+)', entry)
        theta_match = re.search(r'theta: ([-\d\.]+)', entry)
        sec_match = re.search(r'sec: (\d+)', entry)
        nanosec_match = re.search(r'nanosec: (\d+)', entry)
        
        if x_match and y_match and theta_match and sec_match and nanosec_match:
            x_coords.append(float(x_match.group(1)))
            y_coords.append(float(y_match.group(1)))
            thetas.append(float(theta_match.group(1)))
            # Convert timestamp to seconds
            sec = int(sec_match.group(1))
            nanosec = int(nanosec_match.group(1))
            timestamps.append(sec + nanosec/1e9)
    
    return np.array(x_coords), np.array(y_coords), np.array(thetas), np.array(timestamps)

def parse_state_log(filename):
    """Parse robot_0_state.log file."""
    with open(filename, 'r') as f:
        content = f.read()
    
    entries = content.split('---')
    x_coords = []
    y_coords = []
    thetas = []
    timestamps = []
    
    for entry in entries:
        if not entry.strip():
            continue
        
        x_match = re.search(r'x: ([-\d\.]+)', entry)
        y_match = re.search(r'y: ([-\d\.]+)', entry)
        theta_match = re.search(r'theta: ([-\d\.]+)', entry)
        sec_match = re.search(r'sec: (\d+)', entry)
        nanosec_match = re.search(r'nanosec: (\d+)', entry)
        
        if x_match and y_match and theta_match and sec_match and nanosec_match:
            x_coords.append(float(x_match.group(1)))
            y_coords.append(float(y_match.group(1)))
            thetas.append(float(theta_match.group(1)))
            # Convert timestamp to seconds
            sec = int(sec_match.group(1))
            nanosec = int(nanosec_match.group(1))
            timestamps.append(sec + nanosec/1e9)
    
    return np.array(x_coords), np.array(y_coords), np.array(thetas), np.array(timestamps)

def plot_paths(real_state_file, state_file, output_file='robot_paths.png'):
    """Plot the robot paths from both log files."""
    # Parse log files
    real_x, real_y, real_theta, real_timestamps = parse_real_state_log(real_state_file)
    pred_x, pred_y, pred_theta, pred_timestamps = parse_state_log(state_file)
    
    # Create figure
    plt.figure(figsize=(10, 8))
    
    # Plot real path
    plt.plot(real_x, real_y, 'b-', linewidth=2, label='Real Path')
    
    # Plot predicted path
    plt.plot(pred_x, pred_y, 'r--', linewidth=2, label='Predicted Path')
    
    # Mark start points
    plt.plot(real_x[0], real_y[0], 'bs', markersize=8, label='Real Start')
    plt.plot(pred_x[0], pred_y[0], 'rs', markersize=8, label='Predicted Start')
    
    # Mark end points
    plt.plot(real_x[-1], real_y[-1], 'bo', markersize=8, label='Real End')
    plt.plot(pred_x[-1], pred_y[-1], 'ro', markersize=8, label='Predicted End')
    
    # Add direction arrows at regular intervals
    arrow_indices_real = np.linspace(0, len(real_x)-1, min(10, len(real_x))).astype(int)
    arrow_indices_pred = np.linspace(0, len(pred_x)-1, min(10, len(pred_x))).astype(int)
    
    arrow_length = 0.1
    for i in arrow_indices_real:
        dx = arrow_length * np.cos(real_theta[i])
        dy = arrow_length * np.sin(real_theta[i])
        plt.arrow(real_x[i], real_y[i], dx, dy, head_width=0.05, head_length=0.05, fc='b', ec='b')
    
    for i in arrow_indices_pred:
        dx = arrow_length * np.cos(pred_theta[i])
        dy = arrow_length * np.sin(pred_theta[i])
        plt.arrow(pred_x[i], pred_y[i], dx, dy, head_width=0.05, head_length=0.05, fc='r', ec='r')
    
    # Set plot properties
    plt.grid(True)
    plt.axis('equal')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Robot Paths Comparison')
    plt.legend()
    
    # Add timestamp information
    real_duration = real_timestamps[-1] - real_timestamps[0]
    pred_duration = pred_timestamps[-1] - pred_timestamps[0]
    plt.figtext(0.5, 0.01, f'Real path duration: {real_duration:.2f}s | Predicted path duration: {pred_duration:.2f}s', 
                ha='center', fontsize=10, bbox={"facecolor":"lightgray", "alpha":0.5, "pad":5})
    
    # Save the plot
    plt.tight_layout()
    plt.savefig(output_file, dpi=300)
    plt.close()
    
    print(f"Plot saved to {output_file}")

def main():
    real_state_file = 'robot_0_real_state.log'
    state_file = 'robot_0_state.log'
    
    plot_paths(real_state_file, state_file)
    
    print("Path plotting completed successfully!")

if __name__ == "__main__":
    main()