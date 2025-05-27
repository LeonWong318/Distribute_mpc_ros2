import sys
sys.path.append('src')
import rclpy
from rclpy.node import Node

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import json

import numpy as np
from shapely.geometry import Polygon, Point, LineString
import math
from rclpy.executors import MultiThreadedExecutor
import subprocess
import os
import traceback
from threading import Lock, Thread, Event
from pkg_configs.configs import MpcConfiguration, CircularRobotSpecification
from basic_motion_model.motion_model import UnicycleModel
from pkg_motion_plan.local_traj_plan import LocalTrajPlanner
from pkg_tracker_mpc.trajectory_tracker import TrajectoryTracker
from pkg_motion_plan.global_path_coordinate import GlobalPathCoordinator
from msg_interfaces.msg import (
    ManagerToClusterStateSet, 
    GazeboToManagerState,
    ClusterToRobotTrajectory, 
    RobotToClusterState,
    ClusterBetweenRobotHeartBeat,
    ClusterToRvizShortestPath,
    ClusterToRvizConvergeSignal,
    ClusterToManagerState
    )
from msg_interfaces.srv import RegisterRobot


class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_path', ''),
                ('graph_path', ''),
                ('schedule_path', ''),
                ('robot_start_path', ''),
                ('cluster_package', ''),
                ('mpc_config_path', ''),
                ('robot_config_path', ''),
                ('publish_frequency', 10.0),
                ('enable_traj_sharing', True)
            ]
        )
        
        # Get Parameter
        self.map_path = self.get_parameter('map_path').value
        self.graph_path = self.get_parameter('graph_path').value
        self.schedule_path = self.get_parameter('schedule_path').value
        self.robot_start_path = self.get_parameter('robot_start_path').value
        self.cluster_package = self.get_parameter('cluster_package').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.mpc_config_path = self.get_parameter('mpc_config_path').value
        self.robot_config_path = self.get_parameter('robot_config_path').value
        self.enable_traj_sharing = self.get_parameter('enable_traj_sharing').value

        # Load configuration files
        self.load_config_files()
        
        # Load MPC and robot configurations
        self.config_mpc = MpcConfiguration.from_yaml(self.mpc_config_path)
        self.config_robot = CircularRobotSpecification.from_yaml(self.robot_config_path)
        
        # Class state variables
        self.received_first_heartbeat = {}
        self.active_robots = [] 
        self.robot_states = {}  # Will store the latest RobotToClusterState messages
        self.processed_states = {}  # Will store processed (x, y, theta, stamp, pred_states, idle) tuples
        self.idle = {}
        self.converge_flag = {}
        self.control_thread = None
        self.control_stop_event = Event()
        self.robot_planners = {}
        self.robot_controllers = {}
        
        # Timing parameters
        self.heart_beat_send_period = 0.1
        self.heart_beat_check_period = 0.1
        self.control_loop_period = 0.1
        
        # Locks
        self._converter_lock = Lock()
        self._lock = Lock()
        self._registration_lock = Lock()
        self._state_lock = Lock()
        self._control_lock = Lock()
        self._processState_lock = Lock()
        
        # State trackers
        self.converter_states = {}
        self.converter_subscribers = {}
        self.expected_robots = set()  # Set of expected robots(from config)
        self.registered_robots = set()  # Set of registered robots
        self.last_heartbeat_time = {}
        self.cluster_processes = {}
        
        # Global path coordinator setup
        self.gpc = GlobalPathCoordinator.from_csv(self.schedule_path)
        self.gpc.load_graph_from_json(self.graph_path)
        self.gpc.load_map_from_json(self.map_path, inflation_margin=self.config_robot.vehicle_width+0.2)
        self.static_obstacles = self.gpc.inflated_map.obstacle_coords_list
        
        # Parse robot starting positions
        self.parse_robot_start()
        
        # Publishers and subscribers
        self.heartbeat_sub = {}
        self.heartbeat_pub = {}
        self.heartbeat_timer = {}
        self.state_subscribers = {}
        self.traj_pub = {}
        self.shortest_path_pub = {}
        self.converge_signal_pub = {}
        
        # Flag to track if global start signal has been sent
        self._global_start_sent = False
        
        # Create publishers and subscribers
        self.create_pub_and_sub()
        
        # Create service for robot registration
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.register_service = self.create_service(
            RegisterRobot,
            '/register_robot',
            self.handle_register_robot,
            callback_group=self.service_group
        )
        
        # State publisher
        self.states_publisher = self.create_publisher(
            ManagerToClusterStateSet,
            '/manager/robot_states',
            self.reliable_qos
        )
        
        self.get_logger().info('Robot manager initialized successfully')

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

        for robot_id in self.expected_robots:
            # Initialize tracking dictionaries for this robot
            self.idle[robot_id] = False
            self.converge_flag[robot_id] = False
            
            # Subscriber: robot's state
            sub_topic = f'/robot_{robot_id}/state_delayed'
            sub = self.create_subscription(
                RobotToClusterState,
                sub_topic,
                lambda msg, rid=robot_id: self.from_robot_state_callback(msg, rid),
                self.reliable_qos,
                callback_group=self.callback_group
            )
            self.state_subscribers[robot_id] = sub

            # Publisher: trajectory command to each robot
            traj_pub_topic = f'/cluster_{robot_id}/trajectory'
            traj_pub = self.create_publisher(
                ClusterToRobotTrajectory,
                traj_pub_topic,
                self.reliable_qos
            )
            self.traj_pub[robot_id] = traj_pub

            # Publisher: RViz shortest path for each robot
            shortest_path_topic = f'/robot_{robot_id}/shortest_path'
            shortest_path_pub = self.create_publisher(
                ClusterToRvizShortestPath,
                shortest_path_topic,
                self.reliable_qos
            )
            self.shortest_path_pub[robot_id] = shortest_path_pub

            # Publisher: RViz convergence signal for each robot
            converge_signal_topic = f'/robot_{robot_id}/converge_signal'
            converge_signal_pub = self.create_publisher(
                ClusterToRvizConvergeSignal,
                converge_signal_topic,
                self.reliable_qos
            )
            self.converge_signal_pub[robot_id] = converge_signal_pub
            
            # Setup heartbeat
            self.start_heart_beat(robot_id)
    def publish_shortest_path(self, rid, path_coords):
        try:
            path_msg = ClusterToRvizShortestPath()
            path_msg.robot_id = rid

            path_msg.x = [float(coord[0]) for coord in path_coords]
            path_msg.y = [float(coord[1]) for coord in path_coords]

            self.shortest_path_pub[rid].publish(path_msg)
            self.get_logger().debug(f'Published shortest path for robot {rid} with {len(path_coords)} points')

        except Exception as e:
            self.get_logger().error(f'Error publishing shortest path: {str(e)}')     
    def publish_robot_states(self):
        try:
            with self._processState_lock:
                if self.processed_states is not None:
                    msg = ManagerToClusterStateSet()
                    msg.stamp = self.get_clock().now().to_msg()
                    for rid in self.expected_robots:
                        new_msg = ClusterToManagerState()
                        new_msg.robot_id = rid
                        new_msg.x = self.processed_states[rid][0]
                        new_msg.y = self.processed_states[rid][1]
                        new_msg.theta = self.processed_states[rid][2]
                        new_msg.stamp = self.processed_states[rid][3]
                        new_msg.pred_states = self.processed_states[rid][4]
                        new_msg.idle = self.processed_states[rid][5]
                        msg.robot_states.append(new_msg)
                    # Publish the merged states
                if msg.robot_states:
                    self.states_publisher.publish(msg)
                    self.get_logger().debug(f'Published merged states for {len(msg.robot_states)} robots')
                else:
                    self.get_logger().warn('No robot states to publish')

        except Exception as e:
            self.get_logger().error(f'Error publishing robot states: {str(e)}')
            self.get_logger().error(traceback.format_exc())

    def from_robot_state_callback(self, msg, robot_id):
        """Callback for receiving robot state messages"""
        try:
            with self._state_lock:
                if robot_id in self.robot_states:
                    # Check if this is a newer message
                    existing_stamp = self.robot_states[robot_id][3]
                    new_stamp = msg.stamp

                    if (new_stamp.sec > existing_stamp.sec or 
                       (new_stamp.sec == existing_stamp.sec and new_stamp.nanosec > existing_stamp.nanosec)):
                        self.robot_states[robot_id][0] = msg.x
                        self.robot_states[robot_id][1] = msg.y
                        self.robot_states[robot_id][2] = msg.theta
                        self.robot_states[robot_id][3] = msg.stamp
                        
                        self.robot_states[robot_id][5] = msg.idle
                        # Also update the processed state
                        self.processed_states[robot_id] = (
                            msg.x, 
                            msg.y, 
                            msg.theta, 
                            msg.stamp, 
                            self.robot_states[robot_id][4],
                            msg.idle if hasattr(msg, 'idle') else False
                        )
                        self.get_logger().debug(f'Updated state for robot {robot_id}: x={msg.x}, y={msg.y}, theta={msg.theta}')
                else:
                    self.robot_states[robot_id][0] = msg.x
                    self.robot_states[robot_id][1] = msg.y
                    self.robot_states[robot_id][2] = msg.theta
                    self.robot_states[robot_id][3] = msg.stamp
                        
                    self.robot_states[robot_id][5] = msg.idle
                    # Initialize the processed state
                    self.processed_states[robot_id] = (
                        msg.x, 
                        msg.y, 
                        msg.theta, 
                        msg.stamp, 
                        self.robot_states[robot_id][4], 
                        msg.idle if hasattr(msg, 'idle') else False
                    )
                    self.get_logger().debug(f'Received first state for robot {robot_id}: x={msg.x}, y={msg.y}, theta={msg.theta}')
        except Exception as e:
            self.get_logger().error(f'Error in from_robot_state_callback: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            
    def start_heart_beat(self, robot_id):
        """Initialize heartbeat communication for a specific robot"""
        self.heartbeat_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create publisher
        heartbeat_pub = self.create_publisher(
            ClusterBetweenRobotHeartBeat,
            f'/cluster_{robot_id}/heartbeat',
            self.best_effort_qos
        )
        self.heartbeat_pub[robot_id] = heartbeat_pub
        
        # Create subscriber
        heartbeat_sub = self.create_subscription(
            ClusterBetweenRobotHeartBeat,
            f'/robot_{robot_id}/heartbeat',
            lambda msg, rid=robot_id: self.heartbeat_callback(msg, rid),
            self.best_effort_qos,
            callback_group=self.heartbeat_callback_group
        )
        self.heartbeat_sub[robot_id] = heartbeat_sub
        
        # Create timer for sending heartbeats
        heartbeat_timer = self.create_timer(
            self.heart_beat_send_period,
            lambda rid=robot_id: self.send_heartbeat(rid),
            callback_group=self.heartbeat_callback_group
        )
        self.heartbeat_timer[robot_id] = heartbeat_timer
        
        # Initialize tracker for this robot
        self.last_heartbeat_time[robot_id] = self.get_clock().now()
        self.received_first_heartbeat[robot_id] = False

        # Create one global heartbeat check timer if not already created
        if not hasattr(self, 'heartbeat_check_timer'):
            self.heartbeat_check_timer = self.create_timer(
                self.heart_beat_check_period,
                self.check_heartbeats,
                callback_group=self.heartbeat_callback_group
            )

    def heartbeat_callback(self, msg, rid):
        """Process heartbeat messages from a specific robot"""
        try:
            self.last_heartbeat_time[rid] = self.get_clock().now()

            if not self.received_first_heartbeat[rid]:
                self.received_first_heartbeat[rid] = True
                self.get_logger().info(f'Received first heartbeat from robot {rid}')

            self.get_logger().debug(f'Received heartbeat from robot {rid}')
        except Exception as e:
            self.get_logger().error(f'Error in heartbeat_callback: {str(e)}')

    def send_heartbeat(self, rid):
        """Send heartbeat to a specific robot"""
        try:
            heartbeat_msg = ClusterBetweenRobotHeartBeat()
            heartbeat_msg.stamp = self.get_clock().now().to_msg()

            self.heartbeat_pub[rid].publish(heartbeat_msg)
            self.get_logger().debug(f'Sent heartbeat to robot {rid}')
        except Exception as e:
            self.get_logger().error(f'Error sending heartbeat to robot {rid}: {str(e)}')

    def check_heartbeats(self):
        """Check heartbeats for all robots"""
        try:
            current_time = self.get_clock().now()
            
            for rid in list(self.registered_robots):
                if not self.received_first_heartbeat.get(rid, False):
                    continue
                
                if rid not in self.last_heartbeat_time:
                    continue
                    
                time_diff = (current_time - self.last_heartbeat_time[rid]).nanoseconds / 1e9
                self.get_logger().debug(f'Last heartbeat from robot {rid} at {time_diff:.1f} seconds ago')
                
                if time_diff > self.heart_beat_check_period * 60.0:
                    self.get_logger().warn(f'No heartbeat from robot {rid} for {time_diff:.1f} seconds')
                    self.handle_robot_offline(rid)
        except Exception as e:
            self.get_logger().error(f'Error checking heartbeats: {str(e)}')

    def handle_robot_offline(self, rid):
        """Handle when a robot goes offline"""
        if rid in self.idle and not self.idle[rid]:
            self.get_logger().error(f'Robot {rid} appears to be offline, entering idle state')
            self.idle[rid] = True
    
    def load_config_files(self):
        """Load configuration files from disk"""
        try:
            for path_name, path in [
                ('map_path', self.map_path),
                ('graph_path', self.graph_path),
                ('schedule_path', self.schedule_path),
                ('robot_start_path', self.robot_start_path),
                ('mpc_config_path', self.mpc_config_path),
                ('robot_config_path', self.robot_config_path)
            ]:
                if not os.path.exists(path):
                    self.get_logger().error(f'File not found: {path_name} = {path}')
                    raise FileNotFoundError(f'{path_name} file not found: {path}')
                    
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
            raise
        
    def parse_robot_start(self):
        """Parse robot start configurations"""
        try:
            robot_start_data = self.robot_start
            self.expected_robots = set(int(robot_id) for robot_id in robot_start_data.keys())
            self.get_logger().info(f'Expected robots: {self.expected_robots}')
        except Exception as e:
            self.get_logger().error(f'Error parsing robot_start: {str(e)}')
            raise
    
    def check_all_robots_registered(self):
        """Check if all expected robots are registered and start control loop if yes"""
        if len(self.registered_robots) == len(self.expected_robots):
            self.get_logger().info('All robots registered, starting control loop')
            
            self.start_control_loop()
    
    def start_control_loop(self):
        """Start the continuous control loop in a separate thread"""
        if self.control_thread is not None and self.control_thread.is_alive():
            self.get_logger().warn('Control loop already running')
            return
            
        # Reset stop event
        self.control_stop_event.clear()
        
        # Initialize planners and controllers for each robot
        self.init_planners_and_controllers()
        
        # Start the control loop in a separate thread
        self.control_thread = Thread(target=self.continuous_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        self.get_logger().info('Control loop started')
        self.create_timer(
            1.0 / self.publish_frequency,
            self.publish_robot_states,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
    def stop_control_loop(self):
        """Stop the continuous control loop"""
        if self.control_thread is not None and self.control_thread.is_alive():
            self.control_stop_event.set()
            self.control_thread.join(timeout=5.0)
            self.get_logger().info('Control loop stopped')
    
    def init_planners_and_controllers(self):
        """Initialize planners and controllers for all robots"""
        for rid in self.expected_robots:
            # Create motion model
            motion_model = UnicycleModel(sampling_time=self.config_mpc.ts)
            
            # Create local trajectory planner
            planner = LocalTrajPlanner(
                self.config_mpc.ts, 
                self.config_mpc.N_hor, 
                self.config_robot.lin_vel_max, 
                verbose=False
            )
            planner.load_map(
                self.gpc.inflated_map.boundary_coords, 
                self.gpc.inflated_map.obstacle_coords_list
            )
            
            # Load path info for this robot
            path_coords, path_times = self.gpc.get_robot_schedule(rid)
            planner.load_path(
                path_coords,
                path_times,
                nomial_speed=self.config_robot.lin_vel_max, 
                method="linear"
            )
            goal_coord = path_coords[-1]
            goal_coord_prev = path_coords[-2]
            goal_heading = np.arctan2(goal_coord[1]-goal_coord_prev[1], goal_coord[0]-goal_coord_prev[0])
            goal_state = np.array([*goal_coord, goal_heading])

            
            
            # Create trajectory tracker (controller)
            controller = TrajectoryTracker(
                self.config_mpc, 
                self.config_robot, 
                robot_id=rid, 
                verbose=False
            )
            controller.load_motion_model(motion_model)
            init_state = np.asarray(self.robot_start[str(rid)])
            self.get_logger().info(f'Robot {rid} init state is {init_state}')
            controller.load_init_states(init_state, goal_state)
            self.robot_states[rid] = [
                init_state[0],
                init_state[1],
                init_state[2],
                self.get_clock().now().to_msg(),  # ros2 builtin_interfaces.msg.Time
                [],  # list of predicted states
                False     # boolean flag
            ]
            # Store planner and controller
            self.robot_planners[rid] = planner
            self.robot_controllers[rid] = controller
            self.publish_shortest_path(rid,path_coords=path_coords)
            self.get_logger().info(f'Initialized planner and controller for robot {rid}')
    
    def continuous_control_loop(self):
        """Continuous control loop running in a separate thread"""
        self.get_logger().info('Continuous control loop started')
        
        try:
            # Keep running until stop event is set or all robots are idle
            while not self.control_stop_event.is_set():
                all_idle = True
                
                # Guard with lock to prevent race conditions
                with self._control_lock:
                    # Run trajectory planning and control for each active robot
                    for rid in list(self.registered_robots):
                        if rid in self.idle and not self.idle[rid]:
                            all_idle = False
                            self.traj_plan(rid)
                
                # If all robots are idle, we're done
                if all_idle and all(self.idle.get(rid, False) for rid in self.registered_robots):
                    self.get_logger().info('All robots are idle, control loop terminating')
                    break
                    
                # Sleep to avoid consuming too many resources
                # Using the event with a timeout allows for clean shutdowns
                self.control_stop_event.wait(self.control_loop_period)
                
        except Exception as e:
            self.get_logger().error(f'Error in continuous_control_loop: {str(e)}')
            self.get_logger().error(traceback.format_exc())
        
        self.get_logger().info('Continuous control loop ended')
    
      
    def handle_register_robot(self, request, response):
        robot_id = request.robot_id

        try:
            self.get_logger().info(f'Received registration request from robot {robot_id}')

            # check if matches the expected list
            if robot_id not in self.expected_robots:
                self.get_logger().warn(f'Robot {robot_id} not in expected robots list, registration rejected')
                response.success = False
                response.message = f"Robot {robot_id} not in expected robot list"
                return response

            # check if already registered
            if robot_id in self.registered_robots:
                self.get_logger().warn(f'Robot {robot_id} already registered')
                response.success = True
                response.message = f"Robot {robot_id} already registered"
                return response

            self.create_converter_subscriber(robot_id)

            with self._registration_lock:
                self.registered_robots.add(robot_id)
                self.active_robots.append(robot_id)
                with self._state_lock:
                    self.robot_states[robot_id] = None
                    self.processed_states[robot_id] = None

            response.success = True
            response.message = f"Robot {robot_id} registered successfully"
            self.get_logger().info(f'Robot {robot_id} registered successfully')
            self.check_all_robots_registered()
            return response

        except Exception as e:
            self.get_logger().error(f'Error handling registration request: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Registration error: {str(e)}"
            return response
    
    
    
    def create_converter_subscriber(self, robot_id):
        subscriber = self.create_subscription(
            GazeboToManagerState,
            f'/robot_{robot_id}/sim_state_delayed',
            lambda msg: self.converter_state_callback(msg, robot_id),
            self.reliable_qos,
            callback_group=self.service_group
        )

        self.converter_subscribers[robot_id] = subscriber
        self.get_logger().info(f'Created subscriber for converter {robot_id} state')
    
    
    def converter_state_callback(self, msg, robot_id):
        with self._converter_lock:
            if msg.robot_id != robot_id:
                self.get_logger().warn(f'Received converter state with mismatched robot_id: expected {robot_id}, got {msg.robot_id}')
                return

            if robot_id in self.converter_states:
                # ignore outdated msg due to latency in system
                existing_stamp = self.converter_states[robot_id].stamp
                new_stamp = msg.stamp

                # compare the time stamp
                if (new_stamp.sec > existing_stamp.sec or 
                   (new_stamp.sec == existing_stamp.sec and new_stamp.nanosec > existing_stamp.nanosec)):
                    self.converter_states[robot_id] = msg

                    # Also update processed states if this is newer than what we have
                    if robot_id in self.processed_states and self.processed_states[robot_id] is not None:
                        current_stamp = self.processed_states[robot_id][3]
                        if (new_stamp.sec > current_stamp.sec or 
                           (new_stamp.sec == current_stamp.sec and new_stamp.nanosec > current_stamp.nanosec)):
                            self.processed_states[robot_id] = (
                                msg.x, 
                                msg.y, 
                                msg.theta, 
                                msg.stamp, 
                                self.processed_states[robot_id][4],  # Keep existing pred_states
                                self.processed_states[robot_id][5]   # Keep existing idle state
                            )

                    self.get_logger().debug(f'Updated converter state for robot {robot_id}: x={msg.x}, y={msg.y}, theta={msg.theta}')
            else:
                self.converter_states[robot_id] = msg

                # Initialize processed state if not yet available
                if robot_id not in self.processed_states or self.processed_states[robot_id] is None:
                    self.processed_states[robot_id] = (
                        msg.x,
                        msg.y,
                        msg.theta,
                        msg.stamp,
                        [],  # No predicted states from converter
                        False  # Not idle by default
                    )

                self.get_logger().debug(f'Received first converter state for robot {robot_id}: x={msg.x}, y={msg.y}, theta={msg.theta}')
    
    
    def update_robot_states(self):
        try:
            # get locks of cluster and converter
            with self._state_lock, self._converter_lock, self._processState_lock:
                for robot_id in self.registered_robots:
                    cluster_state = self.robot_states[robot_id]
                    converter_state = self.converter_states.get(robot_id)
                    
                    if cluster_state is not None and converter_state is not None:
                        self.get_logger().debug(f'Robot {robot_id}: Both cluster and converter states available')

                        # Compare timestamps
                        cluster_stamp = cluster_state[3]
                        converter_stamp = converter_state.stamp

                        self.get_logger().debug(
                            f'Robot {robot_id}: Cluster stamp: {cluster_stamp.sec}.{cluster_stamp.nanosec}, '
                            f'Converter stamp: {converter_stamp.sec}.{converter_stamp.nanosec}'
                        )
                        if cluster_state[5]:
                            x = cluster_state[0]
                            y = cluster_state[1]
                            theta = cluster_state[2]
                            stamp = cluster_stamp
                            pred_states = []
                            idle = cluster_state[5]
                        elif (converter_stamp.sec > cluster_stamp.sec or
                            (converter_stamp.sec == cluster_stamp.sec and converter_stamp.nanosec > cluster_stamp.nanosec)):
                            self.get_logger().debug(f'Robot {robot_id}: Using CONVERTER state (newer timestamp)')
                            x = converter_state.x
                            y = converter_state.y
                            theta = converter_state.theta
                            stamp = converter_stamp
                            pred_states = cluster_state[4]
                            idle = cluster_state[5]
                        else:
                            self.get_logger().debug(f'Robot {robot_id}: Using CLUSTER state (newer or equal timestamp)')
                            x = cluster_state[0]
                            y = cluster_state[1]
                            theta = cluster_state[2]
                            stamp = cluster_stamp
                            pred_states = cluster_state[4]
                            idle = cluster_state[5]

                        self.processed_states[robot_id] = (x, y, theta, stamp, pred_states, idle)

                    elif cluster_state is not None:
                        self.get_logger().info(f'Robot {robot_id}: Only CLUSTER state available, using it directly')
                        self.processed_states[robot_id] = cluster_state
                        if cluster_state[5]:
                            self.processed_states[robot_id][4] = []

                    elif converter_state is not None:
                        self.get_logger().info(f'Robot {robot_id}: Only CONVERTER state available, using it directly')
                        self.processed_states[robot_id][0] =  converter_state.x
                        self.processed_states[robot_id][1] = converter_state.y
                        self.processed_states[robot_id][2] = converter_state.theta
                        self.processed_states[robot_id][3] = converter_state.stamp
                            

                    else:
                        self.get_logger().warn(f'Robot {robot_id}: Neither cluster nor converter state available')

        except Exception as e:
            self.get_logger().error(f'Error updating robot states: {str(e)}')
            self.get_logger().error(traceback.format_exc())

    def traj_plan(self, rid):
        try:
            # Skip if robot state isn't available
            if rid not in self.processed_states or self.processed_states[rid] is None:
                self.get_logger().warn(f"Skipping trajectory planning for robot {rid} - state not available")
                return
            self.update_robot_states()
            
            # Get the planner and controller previously initialized
            planner = self.robot_planners[rid]
            controller = self.robot_controllers[rid]

            # Get current robot state from processed_states
            current_x, current_y, current_theta = self.processed_states[rid][0:3]

            # Store current state for use in checking obstacles
            current_state = np.array([current_x, current_y, current_theta])

            # Get current time and position
            current_pos = (current_x, current_y)
            current_time = self.get_clock().now().seconds_nanoseconds()
            current_time = current_time[0] + current_time[1] * 1e-9

            # Get reference trajectory
            ref_states, ref_speed, done = planner.get_local_ref(
                current_time=current_time,
                current_pos=current_pos,
                idx_check_range=10
            )

            # Generate connecting path
            connecting_end_path = self.generate_connecting_path(
                start=current_state,
                end=ref_states[5],
                gap=0.2  # set your preferred step gap
            )

            # Set reference states for controller
            controller.set_ref_states(ref_states, ref_speed=ref_speed)
            controller.set_current_state(current_state)
            # Get other robot states
            other_robot_states = []
            for robot_id, state in self.processed_states.items():
                if robot_id != rid and state is not None:
                    other_robot_states.append(state)
            self.get_logger().debug(f'Robot {rid} other_robot_states:{other_robot_states}')
            # Check if we have enough information about other robots
            if len(other_robot_states) == len(self.expected_robots) - 1:
                state_dim = 3  # x, y, theta
                horizon = self.config_mpc.N_hor
                num_others = self.config_mpc.Nother
                robot_states_for_control = [-10.0] * state_dim * (horizon + 1) * num_others
                robot_states_for_switch = [-10.0] * state_dim * (horizon + 1) * num_others
                idx = 0
                for state in other_robot_states:
                    robot_states_for_control[idx:idx+state_dim] = [state[0], state[1], state[2]]
                    robot_states_for_switch[idx:idx+state_dim] = [state[0], state[1], state[2]]
                    idx += state_dim

                idx_pred = state_dim * num_others
                if self.enable_traj_sharing:
                    for state in other_robot_states:
                        if state[4]!=[] and len(state[4]) >= state_dim*horizon:
                            self.get_logger().debug(f'Robot {rid} other robot states is {state[4]}')
                            robot_states_for_control[idx_pred:idx_pred+state_dim*horizon] = state[4][:state_dim*horizon]
                        idx_pred += state_dim * horizon
                    robot_states_for_switch = robot_states_for_control
                else:
                    for __rid in self.expected_robots:
                        if __rid != rid:
                            cur_states = next(state for _rid, state in self.processed_states.items() if _rid == __rid)
                            cur_pos = (cur_states[0], cur_states[1])
                            traj, _,_ = self.robot_planners[__rid].get_local_ref(
                                current_time=current_time,
                                current_pos=cur_pos,
                                idx_check_range=10
                            )
                            flat_traj = traj[:horizon].flatten().tolist()
                            robot_states_for_switch[idx_pred:idx_pred+state_dim*horizon] = flat_traj
                            idx_pred += state_dim*horizon
                start_time = self.get_clock().now()
                
                check_static, path_type = self.check_static_obstacles_on_the_way(ref_states=ref_states, current_state=current_state)

                if check_static is False and \
                   self.check_dynamic_obstacles(ref_states=ref_states, robot_states_for_control=robot_states_for_switch,
                                             num_others=num_others, state_dim=state_dim, horizon=horizon, current_state = current_state,
                                             robot_width=self.config_robot.vehicle_width) is False:
                    remaining_needed = horizon - len(connecting_end_path)
                    remaining_ref = ref_states[5:]
                    remaining_ref = remaining_ref[:remaining_needed]
                    ref_path = np.vstack((connecting_end_path, remaining_ref))
                    use_ref_path = True
                    
                    pred_states = ref_path
                    
                    self.publish_trajectory_to_robot(rid, pred_states, use_ref_path)
                    self.update_pred_states(rid,pred_states)
                    self.get_logger().info(f'Robot {rid} using ref path')
                else:
                    # run controller
                    use_ref_path = False
                    
                    last_actions, pred_states, current_refs, debug_info, exist_status, monitored_cost = controller.run_step(
                        static_obstacles=self.static_obstacles,
                        other_robot_states=robot_states_for_control,
                        inital_guess=None
                    )
                    end_time = self.get_clock().now()
                    duration_ms = (end_time.nanoseconds - start_time.nanoseconds) / 1e9
                    self.get_logger().debug(f'Controller run_step() took {duration_ms:.3f} s')
                    total_cost = monitored_cost["total_cost"]
                    current_state[2] = np.arctan2(np.sin(current_state[2]), np.cos(current_state[2]))
                    
                    # exist_status = 'Converged'
                    if exist_status == 'Converged':
                        # publish traj to robot after calculating
                        self.converge_flag[rid] = True
                        # self.pred_states = init_guess
                        self.get_logger().info(f'Robot {rid} Converged')
                        self.get_logger().info(f'Robot {rid} Cost:{total_cost}')
                        self.publish_trajectory_to_robot(rid, pred_states, use_ref_path)
                        self.publish_converge_signal(rid, self.converge_flag[rid], duration_ms)
                        self.update_pred_states(rid,pred_states)
                    else:
                        self.converge_flag[rid] = False
                        # self.publish_trajectory_to_robot()
                        self.get_logger().info(f'Robot {rid} Not converge reason: {exist_status}')
                        self.get_logger().info(f'Robot {rid} Cost:{total_cost}')
                        #self.publish_trajectory_to_robot(rid, pred_states, use_ref_path)
                        self.publish_converge_signal(rid, self.converge_flag[rid], duration_ms)
                # self.robot_states[rid][4] = pred_states
                # self.get_logger().info(f'Robot {rid} pred_states:{self.robot_states[rid][4]}')
                
                
            else:
                self.get_logger().debug('Not enough other robot states, skip this control loop')

            if controller.check_termination_condition(external_check=planner.idle):
                self.get_logger().info(f'Robot {rid} arrived at goal and entered idle state')
                self.idle[rid] = True

        except Exception as e:
            self.get_logger().error(f'Error in trajectory planning for robot {rid}: {str(e)}')
            self.get_logger().error(traceback.format_exc())

    def update_pred_states(self,rid, pred_states):
        flat_list = [value for row in pred_states for value in row]
        self.robot_states[rid][4]=flat_list

    def publish_trajectory_to_robot(self, rid, pred_states, use_ref_path):
        try:
            if pred_states is None:
                return
            if use_ref_path:
                traj_msg = ClusterToRobotTrajectory()
                traj_msg.stamp = self.get_clock().now().to_msg()

                traj_msg.x = []
                traj_msg.y = []
                traj_msg.theta = []
                traj_msg.traj_type = 'ref'
                for state in pred_states:
                    if isinstance(state, np.ndarray):
                        traj_msg.x.append(float(state[0]))
                        traj_msg.y.append(float(state[1]))
                        traj_msg.theta.append(float(state[2]))

                self.traj_pub[rid].publish(traj_msg)
                return
            
            traj_msg = ClusterToRobotTrajectory()
            traj_msg.stamp = self.get_clock().now().to_msg()

            traj_msg.x = []
            traj_msg.y = []
            traj_msg.theta = []
            traj_msg.traj_type = 'mpc'
            for state in pred_states:
                if isinstance(state, np.ndarray):
                    traj_msg.x.append(float(state[0]))
                    traj_msg.y.append(float(state[1]))
                    traj_msg.theta.append(float(state[2]))

            self.traj_pub[rid].publish(traj_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing Robot {rid} trajectory: {str(e)}')
    def publish_trajectory_to_RViz(self,):
        return
    def unregister_robot(self, robot_id):
        if robot_id not in self.registered_robots:
            return

        if robot_id in self.converter_subscribers:
            self.destroy_subscription(self.converter_subscribers[robot_id])
            del self.converter_subscribers[robot_id]

        # update registration status
        with self._registration_lock:
            self.registered_robots.remove(robot_id)
            if robot_id in self.active_robots:
                self.active_robots.remove(robot_id)
                
            with self._state_lock, self._converter_lock:
                if robot_id in self.robot_states:
                    del self.robot_states[robot_id]
                if robot_id in self.converter_states:
                    del self.converter_states[robot_id]

        self.get_logger().info(f'Robot {robot_id} unregistered')
    def publish_converge_signal(self, rid, converged, computing_time):
        try:
            msg = ClusterToRvizConvergeSignal()
            msg.stamp = self.get_clock().now().to_msg()
            msg.is_converge = converged
            msg.computing_time = computing_time
            self.converge_signal_pub[rid].publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing converge signal for robot {rid}: {str(e)}')
    def check_static_obstacles_on_the_way(self, ref_states, current_state, step_size=0.1):
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
        x0, y0 = current_state[:2]
        x1, y1 = ref_states[5, :2]
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
        ref_states_5 = ref_states[5:]
        for x, y, _ in ref_states_5:
            if is_colliding(x, y):
                connecting_end_check = False
        if connecting_end_check:
            return False, 'end'
        elif connecting_first_check and ref_check:
            return False, 'first'
        else:
            return True, 'collision'
    
    def check_dynamic_obstacles(
        self,
        ref_states,
        robot_states_for_control,
        num_others,
        state_dim,
        horizon,
        current_state,
        robot_width,
        radius: float = 2.0
    ) -> bool:
        """
        Check dynamic obstacles using:
        - Front-zone (semi-circular area in front of robot)
        - Per-timestep trajectory proximity

        Parameters:
            ref_states (np.ndarray): shape (H, 3), reference trajectory
            robot_states_for_control (List[float]): flattened list of all other robot states and predictions
            num_others (int): number of other robots
            state_dim (int): typically 3 (x, y, theta)
            horizon (int): prediction horizon
            robot_width (float): width of a robot
            radius (float): radius for semi-circular front detection

        Returns:
            bool: True if interaction detected
        """
        x0, y0, theta0 = current_state
        my_traj = np.vstack((current_state[:2], ref_states[:, :2]))  # (H+1, 2)

        for i in range(num_others):
            base_idx = i * state_dim
            x, y, theta = robot_states_for_control[base_idx : base_idx + 3]

            # --- 1. Semi-circular front zone check ---
            dx, dy = x - x0, y - y0
            dist = np.hypot(dx, dy)
            if dist <= radius:
                angle_to_robot = math.atan2(dy, dx)
                angle_diff = self._normalize_angle(angle_to_robot - theta0)
                if -np.pi / 2 <= angle_diff <= np.pi / 2:
                    return True  # robot in front zone

            # --- 2. Per-timestep proximity check ---
            pred_idx = (state_dim * num_others) + i * state_dim * horizon
            pred_traj_flat = robot_states_for_control[pred_idx : pred_idx + state_dim * horizon]
            other_traj = np.array([
                pred_traj_flat[j:j+2]
                for j in range(0, len(pred_traj_flat), state_dim)
            ])  # (horizon, 2)

            for t in range(min(horizon, len(my_traj) - 1)):
                my_pos = my_traj[t + 1]  # skip current state
                other_pos = other_traj[t]
                if np.linalg.norm(my_pos - other_pos) <= 2 * robot_width:
                    return True  # interaction at timestep

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

def main(args=None):
    rclpy.init(args=args)
    
    manager = RobotManager()
    
    executor = MultiThreadedExecutor()
    executor.add_node(manager)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for robot_id, process in list(manager.cluster_processes.items()):
            try:
                process.terminate()
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    process.kill()
            except:
                pass
                
        executor.shutdown()
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()