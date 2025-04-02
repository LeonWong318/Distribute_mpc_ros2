import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def parse_ros_log(filename):
    """Parse ROS2 topic echo log file"""
    data = []
    current_entry = {}
    
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            
            # Detect message separator
            if line == '---':
                if current_entry:
                    data.append(current_entry)
                    current_entry = {}
                continue
                
            # Parse fields
            if ':' in line:
                key, value = [part.strip() for part in line.split(':', 1)]
                
                # Handle timestamp
                if key == 'sec':
                    current_entry['sec'] = int(value)
                elif key == 'nanosec':
                    current_entry['nanosec'] = int(value)
                elif key in ['x', 'y', 'theta']:
                    current_entry[key] = float(value)
                elif key == 'robot_id':
                    current_entry[key] = int(value)
    
    # Add the last entry
    if current_entry:
        data.append(current_entry)
    
    # Convert to DataFrame and calculate unified timestamp
    df = pd.DataFrame(data)
    if 'sec' in df.columns and 'nanosec' in df.columns:
        df['timestamp'] = df['sec'] + df['nanosec'] * 1e-9
    
    return df

def interpolate_states(df, time_points):
    """Interpolate states based on time points"""
    # Ensure data is sorted by time
    df = df.sort_values('timestamp')
    
    # Create interpolation functions for x and y
    x_interp = interp1d(df['timestamp'], df['x'], kind='linear', bounds_error=False, fill_value='extrapolate')
    y_interp = interp1d(df['timestamp'], df['y'], kind='linear', bounds_error=False, fill_value='extrapolate')
    
    # Interpolate for each time point
    interp_df = pd.DataFrame({
        'timestamp': time_points,
        'x': x_interp(time_points),
        'y': y_interp(time_points)
    })
    
    return interp_df

def calculate_euclidean_distance(df1, df2):
    """Calculate the Euclidean distance between two datasets"""
    return np.sqrt((df1['x'] - df2['x'])**2 + (df1['y'] - df2['y'])**2)

def main():
    # 1. Read the three log files
    print("Parsing log files...")
    before_df = parse_ros_log('robot_0_before_fusion.log')
    after_df = parse_ros_log('robot_0_after_fusion.log')
    real_df = parse_ros_log('robot_0_real_state.log')
    
    print(f"Parsing complete. Records: before={len(before_df)}, after={len(after_df)}, real={len(real_df)}")
    
    # 2. Determine time range
    start_time = max(before_df['timestamp'].min(), after_df['timestamp'].min(), real_df['timestamp'].min())
    end_time = min(before_df['timestamp'].max(), after_df['timestamp'].max(), real_df['timestamp'].max())
    
    print(f"Common time range: {start_time} to {end_time}")
    
    # 3. Create common time points (100ms interval)
    time_points = np.arange(start_time, end_time, 0.1)
    
    # 4. Interpolate each dataset
    print("Performing time interpolation...")
    before_interp = interpolate_states(before_df, time_points)
    after_interp = interpolate_states(after_df, time_points)
    real_interp = interpolate_states(real_df, time_points)
    
    # 5. Calculate errors (Euclidean distance)
    print("Calculating Euclidean distances...")
    before_errors = calculate_euclidean_distance(before_interp, real_interp)
    after_errors = calculate_euclidean_distance(after_interp, real_interp)
    
    # 6. Calculate average errors
    avg_before_error = before_errors.mean()
    avg_after_error = after_errors.mean()
    
    print(f"Before fusion average error: {avg_before_error:.6f} meters")
    print(f"After fusion average error: {avg_after_error:.6f} meters")
    print(f"Error improvement: {100 * (avg_before_error - avg_after_error) / avg_before_error:.2f}%")
    
    # 7. Visualize results
    plt.figure(figsize=(12, 8))
    
    # Error over time curve
    plt.subplot(2, 1, 1)
    plt.plot(time_points - time_points[0], before_errors, label='Before Fusion Error')
    plt.plot(time_points - time_points[0], after_errors, label='After Fusion Error')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Euclidean Distance (meters)')
    plt.title('Position Error Before and After State Fusion')
    plt.legend()
    plt.grid(True)
    
    # Error boxplot
    plt.subplot(2, 1, 2)
    plt.boxplot([before_errors, after_errors], labels=['Before Fusion', 'After Fusion'])
    plt.ylabel('Euclidean Distance (meters)')
    plt.title('Error Distribution Comparison')
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('fusion_error_analysis.png', dpi=300)
    plt.show()
    
    # 8. Save results to CSV
    result_df = pd.DataFrame({
        'timestamp': time_points,
        'before_fusion_error': before_errors,
        'after_fusion_error': after_errors
    })
    result_df.to_csv('fusion_error_results.csv', index=False)
    print("Analysis results saved to fusion_error_results.csv and fusion_error_analysis.png")

if __name__ == "__main__":
    main()