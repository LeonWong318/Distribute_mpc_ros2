import numpy as np
import random
import json
from shapely.geometry import Point, Polygon, LineString

class Node:
    def __init__(self, state, parent=None):
        self.state = np.array(state)  # [x, y, theta]
        self.parent = parent

class RRTPlanner:
    def __init__(self, map_json_path, dt = 0.2, step_size=0.5, max_iter=1000, goal_sample_rate=0.1, robot_radius=0.4):
        """
        Initialize the RRT planner.
        - map_json_path: path to the JSON file
        - step_size: distance to move toward a sampled point
        - max_iter: maximum number of samples
        - goal_sample_rate: probability to sample the goal
        - robot_radius: safety buffer radius
        """
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        self.robot_radius = robot_radius
        self.dt  = dt

        # Load map JSON from file
        with open(map_json_path, 'r') as f:
            map_data = json.load(f)

        # Load map data
        self.boundary_coords = map_data['boundary_coords']
        self.obstacle_list = map_data['obstacle_list']
        self.obstacle_dict = map_data['obstacle_dict']  # optional, stored for metadata

        # Compute world bounds
        x_coords = [pt[0] for pt in self.boundary_coords]
        y_coords = [pt[1] for pt in self.boundary_coords]
        self.world_bounds = (min(x_coords), max(x_coords), min(y_coords), max(y_coords))

        # Prepare polygons
        self.static_obstacles = [Polygon(obs) for obs in self.obstacle_list]
        self.boundary_polygon = Polygon(self.boundary_coords)

        # Initialize dynamic components
        self.start = None
        self.goal = None
        self.nodes = []
        self.other_robots = []

    def update_environment(self, start, goal, other_robots):
        """
        Update the start, goal, and dynamic other robots.
        - start, goal: [x, y, theta]
        - other_robots: list of [x, y, theta]
        """
        self.start = Node(start)
        self.goal = Node(goal)
        self.nodes = [self.start]
        self.other_robots = [Point(r[:2]) for r in other_robots]

    def plan(self):
        for _ in range(self.max_iter):
            rand_state = self.sample()
            nearest_node = self.get_nearest_node(rand_state)
            traj = self.steer(nearest_node.state, rand_state)
            if traj is None:
                continue

            if self.collision(traj):
                # collision along the trajectory → reject
                continue

            # now traj is valid and non-colliding
            new_node = Node(traj[-1], nearest_node)
            new_node.traj_from_parent = traj
            self.nodes.append(new_node)

            # check goal reachability
            if np.linalg.norm(new_node.state[:2] - self.goal.state[:2]) < self.step_size:
                final_traj = self.steer(new_node.state, self.goal.state)
                if final_traj is not None and not self.collision(final_traj):
                    goal_node = Node(self.goal.state, new_node)
                    goal_node.traj_from_parent = final_traj
                    
                    return self.shortcut_path(np.array(self.extract_path(goal_node)))
        return None


    def sample(self):
        if random.random() < self.goal_sample_rate:
            return self.goal.state
        else:
            x_min, x_max, y_min, y_max = self.world_bounds
            if random.random()<0.9:
                angle = random.uniform(-np.pi/3,0)  #prefer turn right
            else: 
                angle = random.uniform(-np.pi/3, np.pi/3)
            distance = random.uniform(0.5, 5.0)
            base_angle = np.arctan2(self.goal.state[1]-self.start.state[1], self.goal.state[0]-self.start.state[0])
            x = self.start.state[0] + distance * np.cos(base_angle + angle)
            y = self.start.state[1] + distance * np.sin(base_angle + angle)
            x = np.clip(x, x_min, x_max)
            y = np.clip(y, y_min, y_max)
            return np.array([x, y, random.uniform(-np.pi, np.pi)])


    def get_nearest_node(self, sample_state):
        return min(self.nodes, key=lambda node: np.linalg.norm(node.state[:2] - sample_state[:2]))

    def steer(self, from_state, to_state, num_steps=10, max_linear_vel=1.0, max_angular_vel=2):
        """
        Steer from from_state towards to_state using unicycle dynamics,
        with a preference for right turns.
        Simulates multiple steps and returns the trajectory.

        Args:
            from_state: [x, y, theta]
            to_state: [x, y, theta]
            num_steps: how many steps to simulate
            max_linear_vel: maximum forward speed
            max_angular_vel: maximum turn rate (rad/s)

        Returns:
            np.ndarray trajectory of shape (num_steps, 3) or None if left-turn preference rejected
        """
        x, y, theta = from_state
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        target_x, target_y, _ = to_state

        traj = []
        dt = self.dt

        # Compute global direction from current to target
        global_dx = target_x - x
        global_dy = target_y - y
        global_target_theta = np.arctan2(global_dy, global_dx)
        # delta_theta = global_target_theta - theta
        # delta_theta = np.arctan2(np.sin(delta_theta), np.cos(delta_theta))  # normalize to [-pi, pi]

        # Right turn preference: reject if delta_theta > 0 with some probability
        # if delta_theta > 0 and random.random() < 0.8:  # 80% chance to reject left turns
        # if delta_theta >0:
        #     return None

        # Start forward simulation
        for _ in range(num_steps):
            # Recompute local control (dx, dy change at each step)
            dx = target_x - x
            dy = target_y - y
            target_theta = np.arctan2(dy, dx)
            heading_error = target_theta - theta
            heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

            distance = np.hypot(dx, dy)

            # Proportional controller
            v = max_linear_vel * np.clip(distance, 0.0, 1.0)
            omega = max_angular_vel * np.clip(heading_error, -1.0, 1.0)

            # Apply unicycle motion
            x += v * np.cos(theta) * dt
            y += v * np.sin(theta) * dt
            theta += omega * dt
            theta = np.arctan2(np.sin(theta), np.cos(theta))

            traj.append([x, y, theta])

        return np.array(traj)

    def shortcut_path(self, path, num_attempts=100):
        path = path.copy().tolist()
        for _ in range(num_attempts):
            if len(path) <= 2:
                break
            i = random.randint(0, len(path) - 2)
            j = random.randint(i + 1, len(path) - 1)
            from_state = path[i]
            to_state = path[j]

            # Use your kinodynamic steer, not a straight line
            traj = self.steer(from_state, to_state)
            if traj is not None and len(traj) > 0 and not self.collision(traj):
                # Replace middle points with new traj
                path = path[:i+1] + traj.tolist() + path[j+1:]
        return np.array(path)


    def collision(self, traj):
        for state in traj:
            point = Point(state[:2])
            if not self.boundary_polygon.contains(point):
                return True
            for poly in self.static_obstacles:
                if point.buffer(self.robot_radius).intersects(poly):
                    return True
            for robot in self.other_robots:
                if point.distance(robot) < 2 * self.robot_radius:
                    return True
        return False


    def extract_path(self, node):
        path = []
        while node.parent:
            path.extend(reversed(node.traj_from_parent.tolist()))
            node = node.parent
        path.append(node.state.tolist())  # start node
        return path[::-1]
    
    def create_fused_initial_guess(self, rrt_result, previous_solution, horizon_length, distance_threshold=0.3):
        """
        Create a fused initial guess by combining RRT path with previous MPC solution.

        Args:
            rrt_result: Flattened array of [x1, y1, x2, y2, ...]
            previous_solution: Previous MPC solution as list of [x, y, theta] states
            horizon_length: The horizon length of MPC (N_hor)
            distance_threshold: Threshold to determine when to use more RRT influence

        Returns:
            Flattened array of [x1, y1, x2, y2, ...] for use as initial guess
        """
        # # Reshape inputs for easier processing
        # rrt_points = np.array([[state[0],state[1]] for state in rrt_result])
        # print(f"Type of rrt_result: {type(rrt_result)}")
        # print(f"Type of previous_solution: {type(previous_solution)}")

        # Safely convert RRT result to numpy points array
        # try:
        #     if isinstance(rrt_result, np.ndarray):
        #         if rrt_result.ndim == 1:  # Flattened array [x1,y1,x2,y2,...]
        rrt_points = rrt_result.reshape(-1, 2)
        #         else:  # Already in format [[x1,y1], [x2,y2], ...]
        #             rrt_points = rrt_result[:, :2]  # Take only x,y components if needed
        #     else:
        #         # Convert from list or other format
        #         rrt_points = np.array(rrt_result).reshape(-1, 2)
        # except Exception as e:
        #     print(f"Error converting RRT result: {e}")
        # Extract only x, y from previous solution (discard theta)
        if previous_solution is not None:
            prev_xy = np.array([[state[0], state[1]] for state in previous_solution])
        else:
            prev_xy = rrt_points

        # Ensure we have enough points for the full horizon
        if len(rrt_points) < horizon_length:
            # Pad RRT points with last position if needed
            last_point = rrt_points[-1]
            padding = np.tile(last_point, (horizon_length - len(rrt_points), 1))
            rrt_points = np.vstack([rrt_points, padding])
        else:
            # Truncate if we have too many points
            rrt_points = rrt_points[:horizon_length]

        # Similarly for previous solution
        if len(prev_xy) < horizon_length:
            last_point = prev_xy[-1]
            padding = np.tile(last_point, (horizon_length - len(prev_xy), 1))
            prev_xy = np.vstack([prev_xy, padding])
        else:
            prev_xy = prev_xy[:horizon_length]

        # Calculate weights based on distance between solutions
        weights = []
        for i in range(horizon_length):
            # Calculate distance between paths at this point
            distance = np.linalg.norm(rrt_points[i] - prev_xy[i])

            # More weight to RRT when it differs significantly from previous solution
            # Normalize weight between 0.2 and 0.8
            weight = 0.2 + min(0.6, distance / distance_threshold * 0.6)
            weights.append(weight)

        # Time-based adjustment: more RRT influence at beginning, more previous at end
        time_factor = np.linspace(1.0, 0.5, horizon_length)
        weights = np.array(weights) * time_factor

        # Create weighted combination
        fused_points = np.zeros_like(rrt_points)
        for i in range(horizon_length):
            w = weights[i]
            fused_points[i] = w * rrt_points[i] + (1 - w) * prev_xy[i]

        expected_dim = horizon_length * 2
        fused_flat = fused_points.flatten()

        if len(fused_flat) != expected_dim:
            print(f"Warning: Fused guess has wrong dimension: {len(fused_flat)} vs expected {expected_dim}")
            # Ensure exact dimension
            if len(fused_flat) > expected_dim:
                fused_flat = fused_flat[:expected_dim]
            else:
                # Pad with zeros or last values
                padding = np.zeros(expected_dim - len(fused_flat))
                fused_flat = np.concatenate([fused_flat, padding])

        # Flatten the result for OpEn format
        return fused_flat


