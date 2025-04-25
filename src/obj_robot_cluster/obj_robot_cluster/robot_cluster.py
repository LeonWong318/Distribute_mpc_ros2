import sys
sys.path.append('src')

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np
import json
from shapely.geometry import Polygon, Point, LineString
import math

from pkg_configs.configs import MpcConfiguration, CircularRobotSpecification
from basic_motion_model.motion_model import UnicycleModel
from pkg_motion_plan.local_traj_plan import LocalTrajPlanner
from pkg_tracker_mpc.trajectory_tracker import TrajectoryTracker
from pkg_motion_plan.global_path_coordinate import GlobalPathCoordinator
from .new_state import NewState

import threading

from msg_interfaces.msg import (
    ClusterToManagerState, 
    ManagerToClusterStateSet, 
    ClusterToRobotTrajectory, 
    RobotToClusterState,
    ManagerToClusterStart,
    ClusterBetweenRobotHeartBeat,
    ClusterToRvizShortestPath,
    ClusterToRvizConvergeSignal
)

class ClusterNode(Node):
    def __init__(self):
        super().__init__('cluster_node')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', 0),
                ('lookahead_time', 0.5),
                ('close_to_target_rate', 0.5),
                ('control_frequency', 10.0),
                ('mpc_config_path', ''),
                ('robot_config_path', ''),
                ('map_path', ''),
                ('graph_path', ''),
                ('schedule_path', ''),
                ('robot_start_path', ''),
                ('mpc_method','')
            ]
        )
        
        self.robot_id = self.get_parameter('robot_id').value
        self.lookahead_time = self.get_parameter('lookahead_time').value
        self.close_to_target_rate = self.get_parameter('close_to_target_rate').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.mpc_config_path = self.get_parameter('mpc_config_path').value
        self.robot_config_path = self.get_parameter('robot_config_path').value
        self.map_path = self.get_parameter('map_path').value
        self.graph_path = self.get_parameter('graph_path').value
        self.schedule_path = self.get_parameter('schedule_path').value
        self.robot_start_path = self.get_parameter('robot_start_path').value
        self.mpc_method = self.get_parameter('mpc_method').value
        self.converge_flag = False

        self.get_logger().info(f'Initializing cluster node for robot {self.robot_id}')
                
        self.load_config_files()
        
        self.motion_model = UnicycleModel(sampling_time=self.config_mpc.ts)
        self.planner = LocalTrajPlanner(
            self.config_mpc.ts,
            self.config_mpc.N_hor,
            self.config_robot.lin_vel_max
        )
        
        self.heart_beat_send_period = 0.1
        self.heart_beat_check_period = 0.1
        self.waiting_for_robot = True
        self.robot_ready = False
        self.received_first_heartbeat = False
        self.last_heartbeat_time = None
        
        self.robot_state = None
        self.other_robot_states = {}
        self.old_traj = None
        self.predicted_trajectory = None
        self.pred_states = None
        self.idle = True
        self.ref_path = None
        self.use_ref_path = False
        
        self._state_lock = threading.Lock()
        self._last_state_update_time = None
        
        self.current_state_update = NewState(self.lookahead_time, self.config_mpc.ts, self.close_to_target_rate)
        
        self.create_pub_and_sub()
    
    def create_pub_and_sub(self):
        self.callback_group = ReentrantCallbackGroup()
        
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, #using Transient_local to prevent message loss
            depth=10
        )
        
        self.best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.from_robot_state_sub = self.create_subscription(
            RobotToClusterState,
            f'/robot_{self.robot_id}/state_delayed',
            self.from_robot_state_callback,
            self.reliable_qos, 
            callback_group=self.callback_group
        )
        
        self.from_manager_states_sub = self.create_subscription(
            ManagerToClusterStateSet,
            '/manager/robot_states',
            self.from_manager_states_callback,
            self.reliable_qos,
            callback_group=self.callback_group
        )
        
        self.global_start_sub = self.create_subscription(
            ManagerToClusterStart,
            '/manager/global_start',
            self.global_start_callback,
            self.reliable_qos,
            callback_group=self.callback_group
        )
        
        self.to_manager_state_pub = self.create_publisher(
            ClusterToManagerState,
            f'/cluster_{self.robot_id}/state',
            self.reliable_qos
        )
        
        
        self.to_robot_trajectory_pub = self.create_publisher(
            ClusterToRobotTrajectory,
            f'/cluster_{self.robot_id}/trajectory',
            self.reliable_qos
        )
        
        self.shortest_path_pub = self.create_publisher(
            ClusterToRvizShortestPath,
            f'/robot_{self.robot_id}/shortest_path',
            self.reliable_qos
        )
        
        self.converge_signal_pub = self.create_publisher(
            ClusterToRvizConvergeSignal,
            f'/robot_{self.robot_id}/converge_signal',
            self.reliable_qos
        )
    
    def heartbeat_callback(self, msg: ClusterBetweenRobotHeartBeat):
        try:
            self.last_heartbeat_time = self.get_clock().now()

            if not self.received_first_heartbeat:
                self.received_first_heartbeat = True
                self.get_logger().info(f'Received first heartbeat from robot {self.robot_id}')

            self.get_logger().debug(f'Received heartbeat from robot {self.robot_id}')
        except Exception as e:
            self.get_logger().error(f'Error in heartbeat_callback: {str(e)}')


    def send_heartbeat(self):
        try:
            heartbeat_msg = ClusterBetweenRobotHeartBeat()
            heartbeat_msg.stamp = self.get_clock().now().to_msg()

            self.heartbeat_pub.publish(heartbeat_msg)
            self.get_logger().debug(f'Sent heartbeat to robot {self.robot_id}')
        except Exception as e:
            self.get_logger().error(f'Error sending heartbeat: {str(e)}')

    def check_heartbeat(self):
        try:
            if not self.received_first_heartbeat:
                return
            
            current_time = self.get_clock().now()
            time_diff = (current_time - self.last_heartbeat_time).nanoseconds / 1e9
            self.get_logger().debug(f'Last heartbeat from robot {self.robot_id} at {current_time.nanoseconds / 1e9:.1f} seconds')
            
            if time_diff > self.heart_beat_check_period * 60.0:
                self.get_logger().warn(f'No heartbeat from robot {self.robot_id} for {time_diff:.1f} seconds')
                self.handle_robot_offline()
        except Exception as e:
            self.get_logger().error(f'Error checking heartbeat: {str(e)}')

    def handle_robot_offline(self):
        if not self.idle:
            self.get_logger().error(f'Robot {self.robot_id} appears to be offline, entering idle state')
            self.idle = True
    
    def load_config_files(self):
        try:
            self.config_mpc = MpcConfiguration.from_yaml(self.mpc_config_path)
            self.config_robot = CircularRobotSpecification.from_yaml(self.robot_config_path)
            
            with open(self.map_path, 'r') as f:
                self.map_json = f.read()
            with open(self.graph_path, 'r') as f:
                self.graph_json = f.read()
            with open(self.schedule_path, 'r') as f:
                self.schedule_json = f.read()
            with open(self.robot_start_path, 'r') as f:
                robot_start_data = f.read()
                self.robot_start = json.loads(robot_start_data)

            self.get_logger().info('Config files loaded successfully')
            
        except Exception as e:
            self.get_logger().error(f'Error loading config files: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            raise
    
    def setup_control_components(self):        
        self.planner.load_map(self.gpc.inflated_map.boundary_coords, self.gpc.inflated_map.obstacle_coords_list)
        
        self.controller = TrajectoryTracker(
            self.config_mpc,
            self.config_robot,
            robot_id=self.robot_id
        )
        self.controller.load_motion_model(self.motion_model)
    
    def add_schedule(self):
        self.planner.load_path(self.path_coords,self.path_times,nomial_speed=self.config_robot.lin_vel_max, method="linear")
        self.get_logger().info(f"path_coords at scheduling: {self.path_coords}")
        goal_coord = self.path_coords[-1]
        goal_coord_prev = self.path_coords[-2]
        goal_heading = np.arctan2(goal_coord[1]-goal_coord_prev[1], goal_coord[0]-goal_coord_prev[0])
        self.goal_state = np.array([*goal_coord, goal_heading])

        self.idle = False
        self.controller.load_init_states(self._state, self.goal_state)
    
    def global_start_callback(self, msg: ManagerToClusterStart):
        try:
            self.get_logger().info(f'Robot {self.robot_id} started with global signal')
            self.initialize_cluster_components()
        
        except Exception as e:
            self.get_logger().error(f'Error in global start callback: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def update_robot_state(self, x, y, theta, timestamp, source="unknown"):
        current_time = self.get_clock().now()
        current_stamp = current_time.nanoseconds / 1e9

        with self._state_lock:
            if self._last_state_update_time is None or current_stamp > self._last_state_update_time:
                self._state = np.array([x, y, theta])
                self._last_state_update_time = current_stamp
                self.get_logger().debug(f'Updated state from {source}')
                return True
            else:
                self.get_logger().warn(f'Unusual: Current time {current_stamp} not greater than last update time {self._last_state_update_time}')
                return False
    
    def from_manager_states_callback(self, msg: ManagerToClusterStateSet):
        try:
            self.other_robot_states.clear()
            for state in msg.robot_states:
                if state.robot_id != self.robot_id:
                    self.other_robot_states[state.robot_id] = state
                else:
                    self.old_traj = state
                    if self.update_robot_state(state.x, state.y, state.theta, state.stamp, "manager"):
                        self.get_logger().debug(f'Updated state from manager: x={state.x}, y={state.y}, theta={state.theta}')
        except Exception as e:
            self.get_logger().error(f'Error processing robot states: {str(e)}')
    
    def from_robot_state_callback(self, msg: RobotToClusterState):
        try:
            if self.waiting_for_robot:
                self.waiting_for_robot = False
                self.robot_ready = True
                self.get_logger().info(f'Robot {self.robot_id} is ready, starting heartbeat')
                self.start_heart_beat()

            self.robot_state = msg
            self.idle = msg.idle
            if self.update_robot_state(msg.x, msg.y, msg.theta, msg.stamp, "local robot"):
                self.get_logger().debug(f'Updated robot state from local: x={msg.x}, y={msg.y}, theta={msg.theta}')
            self.publish_state_to_manager(msg.stamp)

        except Exception as e:
            self.get_logger().error(f'Error in robot_state_callback: {str(e)}')

    def check_robot_ready(self):
        if self.waiting_for_robot:
            self.get_logger().debug(f'Waiting for robot {self.robot_id} to connect...')
        else:
            self.robot_check_timer.cancel()

    def start_heart_beat(self):
        self.heartbeat_callback_group = MutuallyExclusiveCallbackGroup()
        self.heartbeat_pub = self.create_publisher(
            ClusterBetweenRobotHeartBeat,
            f'/cluster_{self.robot_id}/heartbeat',
            self.best_effort_qos
        )

        self.heartbeat_sub = self.create_subscription(
            ClusterBetweenRobotHeartBeat,
            f'/robot_{self.robot_id}/heartbeat',
            self.heartbeat_callback,
            self.best_effort_qos,
            callback_group=self.heartbeat_callback_group
        )

        self.heartbeat_timer = self.create_timer(
            self.heart_beat_send_period,
            self.send_heartbeat,
            callback_group=self.heartbeat_callback_group
        )

        self.last_heartbeat_time = self.get_clock().now()
        self.received_first_heartbeat = False

        self.heartbeat_check_timer = self.create_timer(
            self.heart_beat_check_period,
            self.check_heartbeat,
            callback_group=self.heartbeat_callback_group
        )
    
    def initialize_cluster_components(self):
        try:
            # Initialize GPC
            self.gpc = GlobalPathCoordinator.from_csv_string(self.schedule_json)

            self.gpc.load_graph_from_json_string(self.graph_json)

            self.gpc.load_map_from_json_string(
                self.map_json,
                inflation_margin=self.config_robot.vehicle_width+0.2
            )
            self.static_obstacles = self.gpc.inflated_map.obstacle_coords_list
            self.path_coords, self.path_times = self.gpc.get_robot_schedule(self.robot_id)
            self.expected_robots = set(int(robot_id) for robot_id in self.robot_start.keys())
            self.get_logger().debug(f'static obstacles{self.static_obstacles}')
            
            # setup control
            self.setup_control_components()
                        
            # self.set_state(np.asarray(self.robot_start[str(self.robot_id)]))
            self._state = np.asarray(self.robot_start[str(self.robot_id)])

            # add schedule
            self.add_schedule()
                        
            # Timer for control loop
            self.create_timer(
                1.0/self.control_frequency,
                self.control_loop
            )
            
            self.waiting_for_robot = True
            self.robot_ready = False

            self.robot_check_timer = self.create_timer(
                0.5,
                self.check_robot_ready
            )
            
            self.publish_shortest_path()
            
            self.get_logger().info('Initialization completed successfully')

        except Exception as e:
            self.get_logger().error(f'Error in initialize cluster components: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            return False
        
    def get_current_state(self,current_time, current_pos, traj_time):
        if self.mpc_method == 'state_fusion':
            self.get_logger().debug('state_fusion')
            fused_state = self.current_state_update.get_new_current_state(self.old_traj.pred_states, current_time, current_pos, traj_time)
            return fused_state
        elif self.mpc_method == 'state_origin':
            self.get_logger().debug('state_origin')
            return current_pos
        else:
            return current_pos
        
    def check_static_obstacles_on_the_way(self, ref_states, step_size=0.1):
        """
        Check if there are any static obstacles from current position to start of ref_states,
        and along the entire reference trajectory.

        Parameters:
            ref_states (np.ndarray): (N, 3) array of reference states
            step_size (float): step size for interpolation from current state to ref start

        Returns:
            
        """
        if ref_states.shape[0] == 0:
            return False, 'end' # Nothing to check

        # Convert obstacles to polygons
        obstacle_polygons = [Polygon(ob) for ob in self.static_obstacles]

        # Helper function to check a point against all obstacles
        def is_colliding(x, y):
            point = Point(x, y)
            return any(poly.contains(point) for poly in obstacle_polygons)
        
        connecting_end_check = True
        connecting_first_check = True
        ref_check = True
        # 1. Check path from current state to end of ref_states
        x0, y0 = self._state[:2]
        x1, y1 = ref_states[-1, :2]
        line = LineString([(x0, y0), (x1, y1)])
        length = line.length
        num_steps = max(2, int(length / step_size))
        for i in range(num_steps):
            pt = line.interpolate(i / (num_steps - 1), normalized=True)
            if is_colliding(pt.x, pt.y):
                connecting_end_check = False
        # 2. check path from current state to first of ref_states
        x2, y2 = ref_states[0,:2]
        line_1 = LineString([(x0, y0), (x2, y2)])
        length_1 = line_1.length
        num_steps_1 = max(2, int(length_1 / step_size))
        for i in range(num_steps_1):
            pt = line_1.interpolate(i / (num_steps_1 - 1), normalized=True)
            if is_colliding(pt.x, pt.y):
                connecting_first_check = False
        # 3. Check each point in ref_states
        for x, y, _ in ref_states:
            if is_colliding(x, y):
                ref_check = False

        if connecting_end_check:
            return False, 'end'
        elif connecting_first_check and ref_check:
            return False, 'first'
        else:
            return True, 'collision'
    
    def check_dynamic_obstacles(self, ref_states, robot_states_for_control, num_others, state_dim, horizon, radius=3):
        """
        Check dynamic obstacles: front zone presence and predicted trajectory cross.

        Parameters:
            ref_states (np.ndarray): (N, 3) reference path
            robot_states_for_control (List[float]): Flat list of other robot states and predictions
            num_others (int): number of other robots
            state_dim (int): usually 3 (x, y, theta)
            horizon (int): prediction horizon for each robot
            radius (float): semi-circular front detection range

        Returns:
            bool: True if there's a dynamic obstacle concern
        """
        x0, y0, theta0 = self._state
        my_path = np.vstack((self._state[:2], ref_states[:, :2]))
        my_path_line = LineString(my_path)

        # Loop over each robot
        for i in range(num_others):
            base_idx = i * state_dim
            x, y, theta = robot_states_for_control[base_idx : base_idx + 3]

            # --- Check semi-circular front zone ---
            dx = x - x0
            dy = y - y0
            dist = np.hypot(dx, dy)
            if dist <= radius:
                angle_to_robot = math.atan2(dy, dx)
                angle_diff = self._normalize_angle(angle_to_robot - theta0)
                if -np.pi/2 <= angle_diff <= np.pi/2:
                    return True  # Robot in front region

            # --- Check predicted trajectory cross ---
            pred_idx = (state_dim * num_others) + i * state_dim * horizon
            pred_traj = robot_states_for_control[pred_idx : pred_idx + state_dim * horizon]
            pred_points = [
                (pred_traj[j], pred_traj[j+1])
                for j in range(0, len(pred_traj) - state_dim + 1, state_dim)
            ]
            if len(pred_points) >= 2:
                other_traj_line = LineString(pred_points)
                if my_path_line.intersects(other_traj_line):
                    return True

        return False
    def _normalize_angle(self, angle):
        """
        Normalize angle to the range (-pi, pi].
    
        Parameters:
            angle (float): angle in radians
    
        Returns:
            float: normalized angle
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    
    @staticmethod
    def generate_connecting_path(start, end, gap):
        """
        Generate points from `start` to `end` using fixed step size `gap`.

        Args:
            start (np.ndarray): shape (3,), current state [x, y, theta]
            end (np.ndarray): shape (3,), target state [x, y, theta]
            gap (float): distance between sampled points

        Returns:
            np.ndarray: (N, 3), each row is [x, y, theta]
        """
        start = np.array(start)
        end = np.array(end)

        vec = end[:2] - start[:2]
        dist = np.linalg.norm(vec)
        if dist < 1e-4:
            return np.empty((0, 3))  # Already at the point

        direction = vec / dist
        num_steps = int(dist // gap)

        path = [start]
        for i in range(1, num_steps + 1):
            x = start[0] + i * gap * direction[0]
            y = start[1] + i * gap * direction[1]
            theta = start[2] + (end[2] - start[2]) * (i / num_steps)
            path.append([x, y, theta])

        return np.array(path)
    
    def control_loop(self):
        try:
            if self.idle:
                return

            # Get current time
            current_time = self.get_clock().now().seconds_nanoseconds()
            current_time = current_time[0] + current_time[1] * 1e-9
            traj_time = self.old_traj.stamp
            traj_time = traj_time.sec + traj_time.nanosec * 1e-9

            # Get local ref and set ref states
            current_pos = (self._state[0], self._state[1])
            current_pos = self.get_current_state(current_time, current_pos, traj_time)
            ref_states, ref_speed, done = self.planner.get_local_ref(
                current_time=current_time,
                current_pos=current_pos,
                idx_check_range=10
            )
            connecting_end_path = self.generate_connecting_path(
                start=self._state,
                end=ref_states[-1],
                gap=0.2  # set your preferred step gap
            )
            connecting_first_path = self.generate_connecting_path(
                start=self._state,
                end=ref_states[0],
                gap=0.2
            )
            # Ensure connecting_path has at least 10 points
            if len(connecting_end_path) < 10:
                num_to_add = 10 - len(connecting_end_path)
                last_point = ref_states[-1]
                padding = np.tile(last_point, (num_to_add, 1))
                self.get_logger().info(f'Connecting path is:{connecting_end_path}')
                connecting_end_path = np.vstack((connecting_end_path, padding))
                    
                                    
            
            self.get_logger().debug(f'Local ref_states:{ref_states}')
            
            self.controller.set_ref_states(ref_states, ref_speed=ref_speed)
            
            # get other robot states
            received_robot_states = [state for rid, state in self.other_robot_states.items()]
            if len(received_robot_states) == len(self.expected_robots) - 1:
                state_dim = 3  # x, y, theta
                horizon = self.config_mpc.N_hor
                num_others = self.config_mpc.Nother
                robot_states_for_control = [-10.0] * state_dim * (horizon + 1) * num_others
                
                idx = 0
                for state in received_robot_states:
                    robot_states_for_control[idx:idx+state_dim] = [state.x, state.y, state.theta]
                    idx += state_dim
                
                idx_pred = state_dim * num_others
                for state in received_robot_states:
                    if hasattr(state, 'pred_states') and len(state.pred_states) >= state_dim * horizon:
                        robot_states_for_control[idx_pred:idx_pred+state_dim*horizon] = state.pred_states[:state_dim*horizon]
                    idx_pred += state_dim * horizon
                
                start_time = self.get_clock().now()
                check_static, path_type = self.check_static_obstacles_on_the_way(ref_states=ref_states)
                
                if  check_static is False and \
                    self.check_dynamic_obstacles(ref_states=ref_states, robot_states_for_control=robot_states_for_control,
                                                 num_others=num_others,state_dim=state_dim,horizon=horizon)==False:
                    
                    if path_type == 'first':
                        self.ref_path = np.vstack((connecting_first_path, ref_states))
                    else:
                        self.ref_path = np.vstack((connecting_end_path, ref_states[-1]))
                    self.use_ref_path = True
                    self.converge_flag = True
                    self.pred_states = self.ref_path
                    self.publish_trajectory_to_robot()
                    self.get_logger().debug('Using ref path')
                else:
                    # run controller
                    self.use_ref_path = False
                    self.last_actions, self.pred_states, self.current_refs, self.debug_info, exist_status= self.controller.run_step(
                        static_obstacles=self.static_obstacles,
                        other_robot_states=robot_states_for_control
                    )

                    end_time = self.get_clock().now()
                    duration_ms = (end_time.nanoseconds - start_time.nanoseconds) / 1e9
                    self.get_logger().debug(f'Controller run_step() took {duration_ms:.3f} s')

                    # run step
                    self.controller.set_current_state(self._state)
                    if exist_status == 'Converged':
                        # publish traj to robot after calculating
                        self.converge_flag = True
                        self.get_logger().info('Converged')
                        self.publish_trajectory_to_robot()
                        self.publish_converge_signal(self.converge_flag)
                    else:
                        self.converge_flag = False
                        # self.publish_trajectory_to_robot()
                        self.get_logger().info(f'Not converge reason: {exist_status}')
                        self.publish_converge_signal(self.converge_flag)
                
            else:
                self.get_logger().debug('Not enough other robot states, skip this control loop')
            
            if self.controller.check_termination_condition(external_check=self.planner.idle):
                self.get_logger().info('Arrived goal and entered idle state')
                self.idle = True

        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')

    def publish_trajectory_to_robot(self):
        try:
            if self.pred_states is None:
                return
            if self.use_ref_path:
                traj_msg = ClusterToRobotTrajectory()
                traj_msg.stamp = self.get_clock().now().to_msg()

                traj_msg.x = []
                traj_msg.y = []
                traj_msg.theta = []
                traj_msg.traj_type = 'ref'
                for state in self.ref_path:
                    if isinstance(state, np.ndarray):
                        traj_msg.x.append(float(state[0]))
                        traj_msg.y.append(float(state[1]))
                        traj_msg.theta.append(float(state[2]))

                self.to_robot_trajectory_pub.publish(traj_msg)
                return
            
            traj_msg = ClusterToRobotTrajectory()
            traj_msg.stamp = self.get_clock().now().to_msg()

            traj_msg.x = []
            traj_msg.y = []
            traj_msg.theta = []
            traj_msg.traj_type = 'mpc'
            for state in self.pred_states:
                if isinstance(state, np.ndarray):
                    traj_msg.x.append(float(state[0]))
                    traj_msg.y.append(float(state[1]))
                    traj_msg.theta.append(float(state[2]))

            self.to_robot_trajectory_pub.publish(traj_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing trajectory: {str(e)}')
        
    def publish_state_to_manager(self, stamp):
        try:
            
            state_msg = ClusterToManagerState()
            state_msg.robot_id = self.robot_id
            state_msg.x = self._state[0]
            state_msg.y = self._state[1]
            state_msg.theta = self._state[2]
            state_msg.idle = self.idle
            state_msg.stamp = stamp

            if self.pred_states is not None and self.converge_flag:
                flattened_states = []
                for state in self.pred_states:
                    if isinstance(state, np.ndarray):
                        flattened_states.extend(state.tolist())
                    elif isinstance(state, (list, tuple)):
                        flattened_states.extend(state)
                    else:
                        flattened_states.append(state)
                state_msg.pred_states = flattened_states
            else:
                state_msg.pred_states = []

            self.to_manager_state_pub.publish(state_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing state to manager: {str(e)}')

    def publish_shortest_path(self):
        try:
            path_msg = ClusterToRvizShortestPath()
            path_msg.robot_id = self.robot_id

            path_msg.x = [float(coord[0]) for coord in self.path_coords]
            path_msg.y = [float(coord[1]) for coord in self.path_coords]

            self.shortest_path_pub.publish(path_msg)
            self.get_logger().debug(f'Published shortest path for robot {self.robot_id} with {len(self.path_coords)} points')

        except Exception as e:
            self.get_logger().error(f'Error publishing shortest path: {str(e)}')

    def publish_converge_signal(self,is_converge):
        try:
            converge_msg = ClusterToRvizConvergeSignal()
            converge_msg.robot_id = self.robot_id
            converge_msg.is_converge = is_converge
            converge_msg.stamp = self.get_clock().now().to_msg()
            
            self.converge_signal_pub.publish(converge_msg)
            self.get_logger().info(f'Published converge signal for robot {self.robot_id} with converge signal is {is_converge}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing converge signal: {str(e)}')
    
def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    
    node = ClusterNode()
    
    executor.add_node(node)
        
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # Keep the main thread running
        executor_thread.join()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
