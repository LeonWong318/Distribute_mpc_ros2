import json
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import matplotlib.colors as mcolors
from matplotlib.collections import LineCollection
import math

def load_json_file(file_path):
    """Load JSON data from file"""
    with open(file_path, 'r') as f:
        return json.load(f)

def draw_map(ax, map_data):
    """Draw the map boundaries and obstacles"""
    # Draw boundary
    boundary = Polygon(map_data['boundary_coords'], fill=False, 
                       edgecolor='black', linewidth=1.5, zorder=1)
    ax.add_patch(boundary)
    
    # Draw obstacles
    for obstacle in map_data['obstacle_list']:
        obs_patch = Polygon(obstacle, 
                          facecolor='lightgray', 
                          edgecolor='dimgray',
                          linewidth=0.8,
                          alpha=0.7,
                          zorder=2)
        ax.add_patch(obs_patch)

def draw_graph(ax, graph_data, robot_data, schedule_path=None):
    """Draw the graph nodes and edges with paths colored according to robots"""
    node_dict = graph_data['node_dict']
    edge_list = graph_data['edge_list']
    
    # Extended color palette for up to 10 robots
    robot_colors = [
        '#1f77b4',  # blue
        '#ff7f0e',  # orange
        '#2ca02c',  # green
        '#d62728',  # red
        '#9467bd',  # purple
        '#8c564b',  # brown
        '#e377c2',  # pink
        '#7f7f7f',  # gray
        '#bcbd22',  # olive
        '#17becf'   # teal
    ]
    
    robot_color_map = {str(i): robot_colors[i % len(robot_colors)] for i in range(len(robot_data))}
    
    # Load schedule from CSV
    robot_paths = {}
    try:
        import csv
        with open(schedule_path, 'r') as f:
            reader = csv.DictReader(f)
            schedule_data = list(reader)
            
            # Group schedule by robot_id
            schedule_by_robot = {}
            for row in schedule_data:
                robot_id = row['robot_id']
                if robot_id not in schedule_by_robot:
                    schedule_by_robot[robot_id] = []
                schedule_by_robot[robot_id].append({
                    'node_id': row['node_id'],
                    'ETA': int(row['ETA'])
                })
            
            # Sort each robot's path by ETA
            for robot_id, path in schedule_by_robot.items():
                sorted_path = sorted(path, key=lambda x: x['ETA'])
                robot_paths[robot_id] = [point['node_id'] for point in sorted_path]
    except Exception as e:
        raise Exception(f"Error reading schedule file: {e}")
    
    # Draw all nodes as simple dots first (we'll overlay special ones later)
    all_node_x = [pos[0] for pos in node_dict.values()]
    all_node_y = [pos[1] for pos in node_dict.values()]
    ax.scatter(all_node_x, all_node_y, c='lightgray', s=15, zorder=3, alpha=0.5)
    
    # Draw all edges as light gray
    normal_lines = []
    for edge in edge_list:
        start_node, end_node = edge
        if start_node in node_dict and end_node in node_dict:
            start_pos = node_dict[start_node]
            end_pos = node_dict[end_node]
            normal_lines.append([start_pos, end_pos])
    
    lc = LineCollection(normal_lines, colors='lightgray', linewidths=0.8, alpha=0.5, zorder=2)
    ax.add_collection(lc)
    
    # Create an edge lookup dictionary for faster access
    edge_lookup = {}
    for edge in edge_list:
        start, end = edge
        if start not in edge_lookup:
            edge_lookup[start] = []
        if end not in edge_lookup:
            edge_lookup[end] = []
        edge_lookup[start].append(end)
        edge_lookup[end].append(start)  # Assuming undirected graph
    
    # Now draw robot-specific paths
    for robot_id, path_nodes in robot_paths.items():
        if robot_id in robot_color_map:
            color = robot_color_map[robot_id]
            
            # Draw path lines
            path_lines = []
            
            # Function to find shortest path between two nodes using BFS
            def find_connecting_path(start_node, end_node):
                if start_node not in node_dict or end_node not in node_dict:
                    return []
                
                # BFS to find shortest path
                queue = [[start_node]]
                visited = set([start_node])
                
                while queue:
                    path = queue.pop(0)
                    node = path[-1]
                    
                    if node == end_node:
                        return path
                    
                    # Check neighbors
                    if node in edge_lookup:
                        for neighbor in edge_lookup[node]:
                            if neighbor not in visited:
                                new_path = list(path)
                                new_path.append(neighbor)
                                queue.append(new_path)
                                visited.add(neighbor)
                
                return []  # No path found
            
            # Connect all nodes in the path
            for i in range(len(path_nodes) - 1):
                current_node = path_nodes[i]
                next_node = path_nodes[i + 1]
                
                # Find connecting path
                connection = find_connecting_path(current_node, next_node)
                
                # Add all segments of the connecting path
                if len(connection) > 1:
                    for j in range(len(connection) - 1):
                        node1 = connection[j]
                        node2 = connection[j + 1]
                        if node1 in node_dict and node2 in node_dict:
                            start_pos = node_dict[node1]
                            end_pos = node_dict[node2]
                            path_lines.append([start_pos, end_pos])
            
            # Draw the colored path for this robot
            if path_lines:
                lc = LineCollection(path_lines, colors=color, linewidths=1.5, alpha=0.8, zorder=4)
                ax.add_collection(lc)
            
            # Highlight nodes in the path
            for i, node_id in enumerate(path_nodes):
                if node_id in node_dict:
                    node_pos = node_dict[node_id]
                    
                    # Start node (first in path)
                    if i == 0:
                        ax.scatter(node_pos[0], node_pos[1], c=color, s=50, marker='o', 
                                 edgecolors='black', linewidth=1.0, zorder=5)
                    
                    # End node (last in path)
                    elif i == len(path_nodes) - 1:
                        ax.scatter(node_pos[0], node_pos[1], c=color, s=70, marker='D', 
                                 edgecolors='black', linewidth=1.0, zorder=5)
                    
                    # Intermediate nodes
                    else:
                        ax.scatter(node_pos[0], node_pos[1], c=color, s=25, marker='o', 
                                 edgecolors='black', linewidth=0.5, alpha=0.7, zorder=5)

def draw_robots(ax, robot_data):
    """Draw robots as rectangles with orientation"""
    robot_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
    robot_width = 0.6  # 0.6m wide
    robot_length = 0.9  # 0.9m long
    
    for i, (robot_id, robot_info) in enumerate(robot_data.items()):
        x, y, theta = robot_info
        
        # Calculate center point for rectangle
        color = robot_colors[i % len(robot_colors)]
        

        bl_x, bl_y = -robot_length/2, -robot_width/2
        
        corners = [
            (bl_x, bl_y),  # bottom-left
            (bl_x + robot_length, bl_y),  # bottom-right
            (bl_x + robot_length, bl_y + robot_width),  # top-right
            (bl_x, bl_y + robot_width)  # top-left
        ]
        
        # Rotate the corners around the origin by theta
        rotated_corners = []
        for corner_x, corner_y in corners:
            # Apply rotation matrix
            rotated_x = corner_x * np.cos(theta) - corner_y * np.sin(theta)
            rotated_y = corner_x * np.sin(theta) + corner_y * np.cos(theta)
            # Translate to robot position
            rotated_corners.append((rotated_x + x, rotated_y + y))
        
        # Create and add the rotated rectangle as a polygon
        rect = Polygon(rotated_corners, 
                     facecolor=color, 
                     edgecolor='black', 
                     alpha=0.7, 
                     zorder=5)
        ax.add_patch(rect)
        
        # Add robot ID with better visibility (text with outline)
        label_offset_y = 1.5  # Offset in y direction (positive = above)
        
        # Create text background for better visibility
        ax.text(x, y + label_offset_y, f"Robot {robot_id}", fontsize=6,
               ha='center', va='center', color='black',
               bbox=dict(facecolor='white', alpha=0.7, edgecolor='black', pad=1.5),
               weight='bold', zorder=6)

def visualize_environment(map_path, graph_path, robot_path, output_path=None, schedule_path=None):
    """Visualize the complete environment with map, graph, and robots"""
    # Load data
    map_data = load_json_file(map_path)
    graph_data = load_json_file(graph_path)
    robot_data = load_json_file(robot_path)
    
    # Create figure with academic paper style
    plt.style.use('default')
    fig, ax = plt.subplots(figsize=(10, 8), dpi=300)
    
    # Draw components
    draw_map(ax, map_data)
    draw_graph(ax, graph_data, robot_data, schedule_path)  # Pass schedule_path to draw_graph
    draw_robots(ax, robot_data)
    
    # Remove title and axis labels
    ax.set_xlabel('')
    ax.set_ylabel('')
    ax.set_title('')
    
    # Set axis limits based on boundary
    boundary = map_data['boundary_coords']
    x_min = min(point[0] for point in boundary) - 1
    x_max = max(point[0] for point in boundary) + 1
    y_min = min(point[1] for point in boundary) - 1
    y_max = max(point[1] for point in boundary) + 1
    
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    
    # Add grid and legend
    ax.grid(True, linestyle='--', alpha=0.3)
    
    
    # Force equal aspect ratio to ensure correct scaling
    ax.set_aspect('equal', 'box')
    
    # Add additional details for academic paper
    plt.tight_layout()
    
    # Save the figure if output path is provided
    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Figure saved to {output_path}")
    
    # Display the figure
    plt.show()

if __name__ == "__main__":
    # Example usage with the provided JSON files
    file_path = "data/test_data_cross/"
    map_file = "map.json"
    graph_file = "graph.json"
    robot_file = "robot_start.json"
    schedule_file = "schedule.csv"
    
    # Visualize the environment
    visualize_environment(
        file_path + map_file, 
        file_path + graph_file, 
        file_path + robot_file, 
        file_path + "environment_visualization.png",
        file_path + schedule_file
    )