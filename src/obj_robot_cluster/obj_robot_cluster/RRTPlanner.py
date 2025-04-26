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

            if not self.collision(traj):
                new_node = Node(traj[-1], nearest_node)
                new_node.traj_from_parent = traj  # save the trajectory
                self.nodes.append(new_node)

                if np.linalg.norm(new_node.state[:2] - self.goal.state[:2]) < self.step_size:
                    final_traj = self.steer(new_node.state, self.goal.state)
                    if not self.collision(final_traj):
                        goal_node = Node(self.goal.state, new_node)
                        goal_node.traj_from_parent = final_traj
                        return np.array(self.extract_path(goal_node))
        return None


    def sample(self):
        if random.random() < self.goal_sample_rate:
            return self.goal.state
        else:
            x_min, x_max, y_min, y_max = self.world_bounds
            return np.array([
                random.uniform(x_min, x_max),
                random.uniform(y_min, y_max),
                random.uniform(-np.pi, np.pi)
            ])

    def get_nearest_node(self, sample_state):
        return min(self.nodes, key=lambda node: np.linalg.norm(node.state[:2] - sample_state[:2]))

    def steer(self, from_state, to_state, num_steps=10, max_linear_vel=1.0, max_angular_vel=np.pi/4):
        """
        Steer from from_state towards to_state using unicycle dynamics.
        Returns: final state after simulating motion.
        """
        x, y, theta = from_state
        target_x, target_y, _ = to_state

        traj = []
        dt = self.dt
        for _ in range(num_steps):
            # Calculate control
            dx = target_x - x
            dy = target_y - y
            target_theta = np.arctan2(dy, dx)
            heading_error = target_theta - theta
            heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))  # normalize

            distance = np.hypot(dx, dy)

            # Simple proportional controller
            v = max_linear_vel * np.clip(distance, 0.0, 1.0)
            omega = max_angular_vel * np.clip(heading_error, -1.0, 1.0)

            # Apply unicycle model
            x += v * np.cos(theta) * dt
            y += v * np.sin(theta) * dt
            theta += omega * dt
            theta = np.arctan2(np.sin(theta), np.cos(theta))  # keep theta bounded

            traj.append([x, y, theta])

        return np.array(traj)


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
