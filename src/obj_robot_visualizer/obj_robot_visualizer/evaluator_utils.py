import numpy as np
import math
from datetime import datetime
from collections import defaultdict

class PathEvaluator:
    def __init__(self, logger):
        self.logger = logger
        self.robot_start_times = {}
        self.robot_end_times = {}
        self.current_status = {}
        self.robot_states = defaultdict(list)

        self.start_recorded = {}
        self.finish_recorded = {}
        
        self.STATUS_INITIALIZING = 0
        self.STATUS_IDLE = 1
        self.STATUS_RUNNING = 2
        self.STATUS_EMERGENCY_STOP = 3
        self.STATUS_TARGET_REACHED = 4
        self.STATUS_DISCONNECT_STOP = 5
        self.STATUS_COLLISION = 6
        self.STATUS_SAFETY_STOP = 7
    
    def update_robot_status(self, robot_id, status):
        if robot_id not in self.start_recorded:
            self.start_recorded[robot_id] = False
        if robot_id not in self.finish_recorded:
            self.finish_recorded[robot_id] = False
        
        if status == self.STATUS_INITIALIZING and not self.start_recorded[robot_id]:
            self.robot_start_times[robot_id] = datetime.now()
            self.robot_states[robot_id] = []
            self.start_recorded[robot_id] = True
            self.logger.info(f"Robot {robot_id} started running at {self.robot_start_times[robot_id]}")
            
        elif status == self.STATUS_TARGET_REACHED and not self.finish_recorded[robot_id]:
            self.robot_end_times[robot_id] = datetime.now()
            self.finish_recorded[robot_id] = True
            self.logger.info(f"Robot {robot_id} reached target at {self.robot_end_times[robot_id]}")
        
        self.current_status[robot_id] = status
    
    def reset(self):
        self.robot_start_times = {}
        self.robot_end_times = {}
        self.current_status = {}
        self.robot_states = defaultdict(list)
        self.start_recorded = {}
        self.finish_recorded = {}
    
    def update_robot_state(self, robot_id, state_msg):
        state_time = datetime.now()
        self.robot_states[robot_id].append({
            'x': state_msg.x,
            'y': state_msg.y,
            'theta': state_msg.theta,
            'time': state_time
        })
    
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
    
    def _calculate_smoothness(self, robot_id):
        states = self.robot_states.get(robot_id, [])
        if len(states) < 3:
            self.logger.warn(f"Insufficient acceleration data to calculate smoothness for robot {robot_id}")
            return {
                'linear_smoothness': float('inf'),
                'angular_smoothness': float('inf')
            }
        
        velocities = []
        angular_velocities = []
        
        for i in range(1, len(states)):
            dx = states[i]['x'] - states[i-1]['x']
            dy = states[i]['y'] - states[i-1]['y']
            dtheta = states[i]['theta'] - states[i-1]['theta']
            
            dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
            
            dt = (states[i]['time'] - states[i-1]['time']).total_seconds()
            
            if dt > 0:
                v = math.sqrt(dx*dx + dy*dy) / dt
                omega = dtheta / dt
                
                velocities.append(v)
                angular_velocities.append(omega)
            else:
                self.logger.warn(f"Zero time difference detected for robot {robot_id}")
        
        if len(velocities) < 2:
            self.logger.warn(f"Insufficient acceleration data to calculate smoothness for robot {robot_id}")
            return {
                'linear_smoothness': float('inf'),
                'angular_smoothness': float('inf')
            }
        
        linear_accelerations = []
        angular_accelerations = []
        
        for i in range(1, len(velocities)):
            dv = velocities[i] - velocities[i-1]
            domega = angular_velocities[i] - angular_velocities[i-1]
            
            dt = (states[i+1]['time'] - states[i]['time']).total_seconds()
            
            if dt > 0:
                a = dv / dt
                alpha = domega / dt
                
                linear_accelerations.append(abs(a))
                angular_accelerations.append(abs(alpha))
            else:
                self.logger.warn(f"Zero time difference detected for robot {robot_id}")
        
        if not linear_accelerations or not angular_accelerations:
            self.logger.warn(f"Insufficient acceleration data to calculate smoothness for robot {robot_id}")
            return {
                'linear_smoothness': float('inf'),
                'angular_smoothness': float('inf')
            }
        
        linear_smoothness = sum(linear_accelerations) / len(linear_accelerations)
        angular_smoothness = sum(angular_accelerations) / len(angular_accelerations)
        
        return {
            'linear_smoothness': linear_smoothness,
            'angular_smoothness': angular_smoothness
        }
        
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
            smoothness_metrics = self._calculate_smoothness(robot_id)
            
            execution_time = None
            if robot_id in self.robot_start_times and robot_id in self.robot_end_times:
                time_diff = self.robot_end_times[robot_id] - self.robot_start_times[robot_id]
                execution_time = time_diff.total_seconds()
            
            results[robot_id] = {
                'deviation_area': deviation_area,
                'normalized_deviation': normalized_deviation,
                'execution_time': execution_time,
                'path_length': planned_length,
                'linear_smoothness': smoothness_metrics['linear_smoothness'],
                'angular_smoothness': smoothness_metrics['angular_smoothness']
            }
            
            self.logger.info(f"Evaluation for robot {robot_id}:")
            self.logger.info(f"  Path length: {planned_length:.2f} units")
            self.logger.info(f"  Deviation area: {deviation_area:.2f} square units")
            self.logger.info(f"  Normalized deviation: {normalized_deviation:.4f}")
            self.logger.info(f"  Linear smoothness (avg |accel|): {smoothness_metrics['linear_smoothness']:.4f} m/s²")
            self.logger.info(f"  Angular smoothness (avg |ang_accel|): {smoothness_metrics['angular_smoothness']:.4f} rad/s²")
            if execution_time is not None:
                self.logger.info(f"  Execution time: {execution_time:.2f} seconds")
            else:
                self.logger.info("  Execution time: Not available")
                
        return results
    
    def all_robots_reached_target(self, robot_ids):
        for robot_id in robot_ids:
            if robot_id not in self.robot_end_times or robot_id not in self.robot_start_times:
                return False
        return len(robot_ids) > 0 and len(self.robot_end_times) > 0