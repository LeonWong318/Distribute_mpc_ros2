import numpy as np
import math
from datetime import datetime
from collections import defaultdict

class PathEvaluator:
    
    def __init__(self, logger):
        self.logger = logger
        self.robot_start_times = {}
        self.robot_end_times = {}
        self.previous_status = {}
        
        self.STATUS_RUNNING = 2
        self.STATUS_TARGET_REACHED = 4
    
    def update_robot_status(self, robot_id, status):
        if robot_id not in self.previous_status:
            self.previous_status[robot_id] = status
            return
            
        if status == self.STATUS_RUNNING and self.previous_status[robot_id] != self.STATUS_RUNNING:
            self.robot_start_times[robot_id] = datetime.now()
            self.logger.info(f"Robot {robot_id} started running at {self.robot_start_times[robot_id]}")
            
        elif status == self.STATUS_TARGET_REACHED and self.previous_status[robot_id] == self.STATUS_RUNNING:
            self.robot_end_times[robot_id] = datetime.now()
            self.logger.info(f"Robot {robot_id} reached target at {self.robot_end_times[robot_id]}")
        
        self.previous_status[robot_id] = status
    
    def _point_to_line_segment_distance(self, point, line_start, line_end):
        line_vec = (line_end[0] - line_start[0], line_end[1] - line_start[1])
        point_vec = (point[0] - line_start[0], point[1] - line_start[1])
        line_length_squared = line_vec[0]**2 + line_vec[1]**2
        
        if line_length_squared == 0:
            return math.sqrt((point[0] - line_start[0])**2 + (point[1] - line_start[1])**2)
        
        t = max(0, min(1, (point_vec[0] * line_vec[0] + point_vec[1] * line_vec[1]) / line_length_squared))
        
        proj_x = line_start[0] + t * line_vec[0]
        proj_y = line_start[1] + t * line_vec[1]
        
        return math.sqrt((point[0] - proj_x)**2 + (point[1] - proj_y)**2)
    
    def _find_min_distance_to_path(self, point, path):
        if len(path) < 2:
            return float('inf')
            
        min_distance = float('inf')
        
        for i in range(len(path) - 1):
            line_start = path[i]
            line_end = path[i + 1]
            
            distance = self._point_to_line_segment_distance(point, line_start, line_end)
            
            min_distance = min(min_distance, distance)
            
        return min_distance
    
    def _calculate_path_length(self, path):
        length = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            length += math.sqrt(dx*dx + dy*dy)
        return length
    
    def _calculate_path_deviation_area(self, actual_path, planned_path):
        if not actual_path or not planned_path:
            return float('inf')
            
        distances = []
        for point in actual_path:
            distance = self._find_min_distance_to_path(point, planned_path)
            distances.append(distance)

        area = 0.0
        for i in range(len(actual_path) - 1):
            d1 = distances[i]
            d2 = distances[i+1]
            dx = actual_path[i+1][0] - actual_path[i][0]
            dy = actual_path[i+1][1] - actual_path[i][1]
            segment_length = math.sqrt(dx*dx + dy*dy)
            area += 0.5 * (d1 + d2) * segment_length
            
        return area
        
    def evaluate_robot_paths(self, robot_ids, robot_paths, shortest_paths):
        results = {}
        
        for robot_id in robot_ids:
            if robot_id not in robot_paths or robot_id not in shortest_paths:
                self.logger.warn(f"Skipping evaluation for robot {robot_id}: paths not available")
                continue
                
            actual_path = robot_paths.get(robot_id, [])
            planned_path = shortest_paths.get(robot_id, [])
            
            if len(actual_path) < 2 or len(planned_path) < 2:
                self.logger.warn(f"Skipping evaluation for robot {robot_id}: insufficient path points")
                continue
                
            planned_length = self._calculate_path_length(planned_path)
            deviation_area = self._calculate_path_deviation_area(actual_path, planned_path)
            normalized_deviation = deviation_area / planned_length if planned_length > 0 else float('inf')
            
            execution_time = None
            if robot_id in self.robot_start_times and robot_id in self.robot_end_times:
                time_diff = self.robot_end_times[robot_id] - self.robot_start_times[robot_id]
                execution_time = time_diff.total_seconds()
            
            results[robot_id] = {
                'deviation_area': deviation_area,
                'normalized_deviation': normalized_deviation,
                'execution_time': execution_time,
                'path_length': planned_length
            }
            
            self.logger.info(f"Evaluation for robot {robot_id}:")
            self.logger.info(f"  Path length: {planned_length:.2f} units")
            self.logger.info(f"  Deviation area: {deviation_area:.2f} square units")
            self.logger.info(f"  Normalized deviation: {normalized_deviation:.4f}")
            if execution_time is not None:
                self.logger.info(f"  Execution time: {execution_time:.2f} seconds")
            else:
                self.logger.info("  Execution time: Not available")
                
        return results
    
    def all_robots_reached_target(self, robot_ids):
        for robot_id in robot_ids:
            if robot_id not in self.robot_end_times:
                return False
        return True