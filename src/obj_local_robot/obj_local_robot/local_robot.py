import sys
sys.path.append('src')

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from basic_motion_model.motion_model import UnicycleModel
from pkg_configs.configs import CircularRobotSpecification, CBFconfig

import numpy as np
import time
import asyncio
from pkg_local_control.pure_pursuit import PurePursuit
from pkg_local_control.lqr import LQRController
from pkg_local_control.lqr_update import LQR_Update_Controller
from pkg_local_control.cbf_lqr import CBF_LQR_Controller
from pkg_local_control.obt_processer import ObstacleProcessor

from msg_interfaces.msg import ClusterToRobotTrajectory, RobotToClusterState, RobotToRvizStatus, ClusterBetweenRobotHeartBeat, RobotToRvizTargetPoint
from msg_interfaces.srv import RegisterRobot, ExecuteCommand
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import LaserScan

class RobotNode(Node):
    async def initialize(self):
        try:
            self.get_logger().info('Starting initialization...')
            
            self.update_robot_status(self.STATUS_INITIALIZING)
            
            # Wait for register service
            while not self.register_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Register service not available, waiting...')
            
            # Register with manager
            self.get_logger().info('Registering with manager...')
            registration_result = await self.register_with_manager()
            
            if not registration_result:
                self.get_logger().error('Registration failed')
                return False
                
            self.get_logger().info('Registration successful')
            
            # Set initial state and publish init state after initialization is complete
            self._state, self.target_point = self.load_init_state_and_target()
            self.publish_state_to_cluster()
            
            # Wait for cluster node to be ready (by detecting trajectory messages)
            self.get_logger().info('Waiting for cluster node to start publishing...')
            cluster_ready = await self.wait_for_cluster()

            if not cluster_ready:
                self.get_logger().error('Timeout waiting for cluster node')
                return False
                
            self.get_logger().info('Starting heartbeat mechanism...')
            self.start_heart_beat()
            self.get_logger().info('Heartbeat mechanism started')
            
            self.idle = False
            self.update_robot_status(self.STATUS_IDLE)
        
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error during initialization: {str(e)}')
            return False

    def __init__(self):
        super().__init__('robot_node')

        # Declare node parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', 0),
                ('max_velocity', 1.0),
                ('max_angular_velocity', 1.0),
                ('control_frequency', 10.0),
                ('mpc_ts', 0.3),
                ('lookahead_distance', 0.5),
                ('lookahead_time', 0.2),
                ('lookahead_style', 'dist'),
                ('robot_config_path', ''),
                ('robot_start_path', ''),
                ('robot_graph_path',''),
                ('robot_schedule_path',''),
                ('cluster_wait_timeout', 30.0),  # Timeout for waiting for cluster node (seconds)
                ('alpha', 1.0),  # Tuning parameter for velocity reduction at high curvature
                ('ts', 0.2),  # Sampling time for controllers
                ('controller_type', 'pure_pursuit'),  # 'pure_pursuit', 'lqr', or 'cbf'
                ('lqr_q_pos', 1.0),
                ('lqr_q_theta', 0.5),
                ('lqr_r_v', 0.1),
                ('lqr_r_omega', 0.1),
                ('lqr_update_q_pos', 1.0),
                ('lqr_update_q_theta', 0.5),
                ('lqr_update_r_v', 0.1),
                ('lqr_update_r_omega', 0.1),
                ('lqr_lookahead_dist',1),
                ('lqr_lookahead_time', .2),
                ('lqr_lookahead_style', 'time'),
                ('cbf_config_path',''),
                ('safety_margin', 0.2),
                ('max_obstacle_distance', 3.0),
            ]
        )
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.robot_config_path = self.get_parameter('robot_config_path').value
        self.robot_start_path = self.get_parameter('robot_start_path').value
        self.robot_graph_path = self.get_parameter('robot_graph_path').value
        self.robot_schedule_path = self.get_parameter('robot_schedule_path').value
        self.cluster_wait_timeout = self.get_parameter('cluster_wait_timeout').value
        
        # Get type of controller
        self.controller_type = self.get_parameter('controller_type').value
        self.mpc_ts = self.get_parameter('mpc_ts').value
        
        # Get purepursuit parameters
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.lookahead_time = self.get_parameter('lookahead_time').value
        self.lookahead_style = self.get_parameter('lookahead_style').value
        self.alpha = self.get_parameter('alpha').value
        self.ts = 1.0 / self.control_frequency
        
        # Get lqr parameters
        self.lqr_q_pos = self.get_parameter('lqr_q_pos').value
        self.lqr_q_theta = self.get_parameter('lqr_q_theta').value
        self.lqr_r_v = self.get_parameter('lqr_r_v').value
        self.lqr_r_omega = self.get_parameter('lqr_r_omega').value

        # Get lqr update parameters
        self.lqr_update_q_pos = self.get_parameter('lqr_update_q_pos').value
        self.lqr_update_q_theta = self.get_parameter('lqr_update_q_theta').value
        self.lqr_update_r_v = self.get_parameter('lqr_update_r_v').value
        self.lqr_update_r_omega = self.get_parameter('lqr_update_r_omega').value
        self.lqr_lookahead_dist = self.get_parameter('lqr_lookahead_dist').value
        self.lqr_lookahead_time = self.get_parameter('lqr_lookahead_time').value
        self.lqr_lookahead_style = self.get_parameter('lqr_lookahead_style').value
        
        # Get CBF config
        self.cbf_config_path = self.get_parameter('cbf_config_path').value
        self.cbf_config = CBFconfig.from_yaml(self.cbf_config_path)
        self.safety_margin = self.get_parameter('safety_margin').value
        self.max_obstacle_distance = self.get_parameter('max_obstacle_distance').value
        self.laser_processor = ObstacleProcessor(self.safety_margin, self.max_obstacle_distance)

        # Load robot configuration
        self.config_robot = CircularRobotSpecification.from_yaml(self.robot_config_path)
        self.motion_model = UnicycleModel(sampling_time=self.ts)
        
        # Status Definition
        self.STATUS_INITIALIZING = 0
        self.STATUS_IDLE = 1
        self.STATUS_RUNNING = 2
        self.STATUS_EMERGENCY_STOP = 3
        self.STATUS_TARGET_REACHED = 4
        self.STATUS_DISCONNECT_STOP = 5
        self.STATUS_COLLISION = 6 
        self.STATUS_SAFETY_STOP = 7
        
        self.current_status = self.STATUS_INITIALIZING
        self.status_descriptions = {
            self.STATUS_INITIALIZING: "Initializing",
            self.STATUS_IDLE: "Idle",
            self.STATUS_RUNNING: "Running",
            self.STATUS_EMERGENCY_STOP: "Emergency Stop",
            self.STATUS_TARGET_REACHED: "Target Reached",
            self.STATUS_DISCONNECT_STOP: "DISCONNECT Stop",
            self.STATUS_COLLISION: "Collision Detected",
            self.STATUS_SAFETY_STOP: "Safety stop for collision avoidance"
        }
        
        # Initialize controllers
        self.setup_controllers()

        # Initialize state variables
        self.idle = True
        self._state = None  # Current state
        self.current_trajectory = None  # Current trajectory
        self.trajectory_received = False  # Flag for trajectory reception
        self.last_received_state_from_gazebo_time = self.get_clock().now().to_msg()
        self.collision_detected = False
        self.front_obstacles = None
        self.back_obstacles = None
        self.back_distances = None
        
        # Create callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Set up QoS profiles
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create registration service client
        self.register_client = self.create_client(
            RegisterRobot,
            '/register_robot',
            callback_group=self.callback_group
        )
        
        # Create subscriber for trajectory from cluster
        self.trajectory_sub = self.create_subscription(
            ClusterToRobotTrajectory,
            f'/cluster_{self.robot_id}/trajectory_delayed',
            self.trajectory_callback,
            self.reliable_qos,
            callback_group=self.callback_group
        )

        # Create subscriber for laser scan info from gazebo
        self.front_laser = self.create_subscription(
            LaserScan,
            f'/robot_{self.robot_id}/f_scan',
            self.front_laserscan_callback,
            self.best_effort_qos,
            callback_group=self.callback_group
        )

        self.back_laser = self.create_subscription(
            LaserScan,
            f'/robot_{self.robot_id}/b_scan',
            self.back_laserscan_callback,
            self.best_effort_qos,
            callback_group=self.callback_group
        )
        
        # create subscription for collision
        self.collision_sub = self.create_subscription(
            ContactsState,
            f'/robot_{self.robot_id}/robot_collision',
            self.collision_callback,
            self.best_effort_qos,
            callback_group=self.callback_group
        )
    
        
        # Create publisher for robot state to cluster
        self.to_cluster_state_pub = self.create_publisher(
            RobotToClusterState,
            f'/robot_{self.robot_id}/state',
            self.reliable_qos
        )
        
        self.send_command_client = self.create_client(
            ExecuteCommand,
            f'/robot_{self.robot_id}/command',
            callback_group=self.callback_group
        )
        
        self.status_pub = self.create_publisher(
            RobotToRvizStatus,
            f'/robot_{self.robot_id}/status',
            self.reliable_qos
        )
        
        self.target_point_pub = self.create_publisher(
            RobotToRvizTargetPoint,
            f'robot_{self.robot_id}/target_point',
            self.reliable_qos
        )
        
        # Create control timer
        timer_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(
            timer_period, 
            self.control_loop_callback,
            callback_group=self.callback_group
        )
        
        # Set heart beat period
        self.heart_beat_send_period = 0.1
        self.heart_beat_check_period = 0.2
        
        self.get_logger().info(f'Robot node initialized with controller: {self.controller_type}')

    def setup_controllers(self):
        """Initialize the selected controller"""
        # Initialize Pure Pursuit controller
        self.pure_pursuit = PurePursuit(
            self.lookahead_distance, 
            self.ts, 
            self.max_velocity, 
            self.alpha,
            self.lookahead_style,
            self.lookahead_time,
            self.mpc_ts
        )
        
        # Initialize LQR controller
        Q = np.diag([self.lqr_q_pos, self.lqr_q_pos, self.lqr_q_theta])
        R = np.diag([self.lqr_r_v, self.lqr_r_omega])
        self.lqr_controller = LQRController(self.ts, Q, R, self.max_velocity)

        # Initialize LQR update controller
        Q_update = np.diag([self.lqr_update_q_pos, self.lqr_update_q_pos, self.lqr_update_q_theta])
        R_update = np.diag([self.lqr_update_r_v, self.lqr_update_r_omega])
        self.lqr_update_controller = LQR_Update_Controller(
            self.ts,Q_update,R_update, 
            self.max_velocity, 
            self.lqr_lookahead_dist,
            self.lqr_lookahead_style,
            self.lqr_lookahead_time,
            self.mpc_ts
            )
        
        # Initialize CBF controller
        self.cbf_controller = CBF_LQR_Controller(self.cbf_config, self.max_velocity, self.ts)
        
        self.get_logger().info(f'Controllers initialized: {self.controller_type}')

    async def register_with_manager(self):
        """Register with manager node"""
        try:
            # Create request
            request = RegisterRobot.Request()
            request.robot_id = self.robot_id
            
            # Send request
            future = self.register_client.call_async(request)
            
            # Wait for response
            response = await future
            
            if not response.success:
                self.get_logger().error(f'Registration failed: {response.message}')
                return False
                
            self.get_logger().info(f'Registration successful: {response.message}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error during registration: {str(e)}')
            return False
    
    async def wait_for_cluster(self):
        """Wait for cluster node to be ready (wait until trajectory is received)"""
        try:
            start_time = time.time()
            
            # Wait until trajectory is received or timeout
            while not self.trajectory_received:
                # Check for timeout
                if time.time() - start_time > self.cluster_wait_timeout:
                    self.get_logger().warn('Timeout waiting for trajectory from cluster node')
                    return False
                    
                # Wait for a short period
                await asyncio.sleep(0.1)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error waiting for cluster: {str(e)}')
            return False
    
    def start_heart_beat(self):
        self.heartbeat_pub = self.create_publisher(
            ClusterBetweenRobotHeartBeat,
            f'/robot_{self.robot_id}/heartbeat',
            self.best_effort_qos
        )

        self.heartbeat_sub = self.create_subscription(
            ClusterBetweenRobotHeartBeat,
            f'/cluster_{self.robot_id}/heartbeat',
            self.heartbeat_callback,
            self.best_effort_qos,
            callback_group=self.callback_group
        )

        self.heartbeat_timer = self.create_timer(
            self.heart_beat_send_period,
            self.send_heartbeat,
            callback_group=self.callback_group
        )

        self.last_heartbeat_time = self.get_clock().now()
        self.received_first_heartbeat = False  

        self.heartbeat_check_timer = self.create_timer(
            self.heart_beat_check_period,
            self.check_heartbeat,
            callback_group=self.callback_group
        )

        self.cluster_connected = False
    
    def heartbeat_callback(self, msg: ClusterBetweenRobotHeartBeat):
        try:
            self.last_heartbeat_time = self.get_clock().now()

            if not self.received_first_heartbeat:
                self.received_first_heartbeat = True
                self.get_logger().info(f'Received first heartbeat from cluster {self.robot_id}')

            if not self.cluster_connected:
                self.cluster_connected = True
                self.get_logger().info(f'Cluster node {self.robot_id} is now connected')
                
                if self.current_status == self.STATUS_DISCONNECT_STOP:
                    self.idle = False 
                    self.update_robot_status(self.STATUS_IDLE)
                
            self.get_logger().debug(f'Received heartbeat from cluster {self.robot_id}')
        except Exception as e:
            self.get_logger().error(f'Error in heartbeat_callback: {str(e)}')

    def send_heartbeat(self):
        try:
            heartbeat_msg = ClusterBetweenRobotHeartBeat()
            heartbeat_msg.stamp = self.get_clock().now().to_msg()

            self.heartbeat_pub.publish(heartbeat_msg)
            self.get_logger().debug(f'Sent heartbeat to cluster {self.robot_id}')
        except Exception as e:
            self.get_logger().error(f'Error sending heartbeat: {str(e)}')

    def check_heartbeat(self):
        try:
            if not self.received_first_heartbeat:
                return
            
            current_time = self.get_clock().now()
            time_diff = (current_time - self.last_heartbeat_time).nanoseconds / 1e9

            if time_diff > self.heart_beat_check_period * 10.0:
                if self.cluster_connected:
                    self.cluster_connected = False
                    self.get_logger().warn(f'Cluster node {self.robot_id} appears to be offline')
                    self.handle_cluster_offline()
        except Exception as e:
            self.get_logger().error(f'Error checking heartbeat: {str(e)}')

    def gazebo_state_callback(self, msg):
        try:
            self._state = [msg.x, msg.y, msg.theta]
            self.last_received_state_from_gazebo_time = msg.stamp
            self.get_logger().debug(f'Updated from Gazebo: x={msg.x:.2f}, y={msg.y:.2f}, θ={msg.theta:.2f}')

        except Exception as e:
            self.get_logger().error(f'Error in gazebo_state_callback: {str(e)}')

    
    def handle_cluster_offline(self):
        if not self.idle:
            self.get_logger().error(f'Cluster {self.robot_id} appears to be offline, entering DISCONNECT stop state')
            self.idle = True
            self.update_robot_status(self.STATUS_DISCONNECT_STOP)
            self.execute_stop()

    def execute_stop(self):
        try:
            self.send_command_to_gazebo(0, 0)
            self.get_logger().debug(f'Robot {self.robot_id} stop executed')
        except Exception as e:
            self.get_logger().error(f'Error during stop: {str(e)}')

    def trajectory_callback(self, msg: ClusterToRobotTrajectory):
        """Process trajectory message from cluster"""
        try:    
            # Mark trajectory as received
            self.trajectory_received = True
            
            # Check if state is initialized
            if self._state is None:
                self.get_logger().warn('Received trajectory but state is not initialized yet')
                self.current_trajectory = msg
                return
            
            # Calculate minimum distance between current state and trajectory points
            min_distance = float('inf')
            current_position = np.array([self._state[0], self._state[1]])

            for i in range(len(msg.x)):
                traj_point = np.array([msg.x[i], msg.y[i]])
                distance = np.linalg.norm(current_position - traj_point)
                min_distance = min(min_distance, distance)

            # Define threshold for acceptable distance (you can make this a parameter)
            distance_threshold = 1000000.0  # meters

            if min_distance > distance_threshold:
                self.get_logger().warn(f'Current position too far from trajectory (min distance: {min_distance:.2f}m). Stopping robot until next update.')
                self.execute_stop()
                self.current_trajectory = None
                self.update_robot_status(self.STATUS_EMERGENCY_STOP)
            else:
                # Store current trajectory if distance is acceptable
                self.current_trajectory = msg
                self.get_logger().debug(f'Received valid trajectory with {len(msg.x)} points (min distance: {min_distance:.2f}m)')

            
        except Exception as e:
            self.get_logger().error(f'Error in trajectory_callback: {str(e)}')
    
    def front_laserscan_callback(self, msg: LaserScan):
        """
        Callback function to process incoming LaserScan messages

        :param msg: LaserScan message from ROS
        """
        # Use the imported class to process the scan data
        if self._state is None:
            return
        
        closest_obstacles, _ = self.laser_processor.get_closest_front_obstacles(msg, self._state)
        
        self.front_obstacles = closest_obstacles

        self.get_logger().debug(f'Closest front obstacles: {closest_obstacles}')
        if closest_obstacles.shape[0] != 0:
            self.safety_stop_handling()
            self.get_logger().info('SAFETY STOP')
        elif closest_obstacles.shape[0] == 0 and self.current_status == self.STATUS_SAFETY_STOP:
            self.get_logger().info('return to running')
            self.send_command_to_gazebo(0,0)
            self.update_robot_status(self.STATUS_RUNNING)
        

    def back_laserscan_callback(self, msg: LaserScan):
        """
        Callback function to process incoming LaserScan messages

        :param msg: LaserScan message from ROS
        """
        # Use the imported class to process the scan data
        if self._state is None:
            return
        
        closest_obstacles, distances = self.laser_processor.get_closest_back_obstacles(msg, self._state)
        
        self.back_obstacles = closest_obstacles
        self.back_distances = distances

        self.get_logger().debug(f'Closest back obstacles: {closest_obstacles}')

    def safety_stop_handling(self):
        self.send_command_to_gazebo(0, 0)
        self.update_robot_status(self.STATUS_SAFETY_STOP)
        if self.laser_processor.is_back_clear(self.back_obstacles, self.back_distances, 1.5):
            self.send_command_to_gazebo(-0.25, 0)

    def collision_callback(self, msg: ContactsState):
        try:
            if not msg.states:
                return

            if self.collision_detected:
                return

            self.get_logger().error(f'Collision detected for robot {self.robot_id}!')

            self.collision_detected = True
            self.update_robot_status(self.STATUS_COLLISION)
            self.execute_stop()
            self.idle = True

            collision_info = []
            for state in msg.states:
                collision_with = "environment"
                other_robot_id = None

                for name in [state.collision1_name, state.collision2_name]:
                    if "robot_" in name and f"robot_{self.robot_id}" not in name:
                        collision_with = "robot"
                        parts = name.split('robot_')
                        if len(parts) > 1:
                            other_id = parts[1].split('/')[0]
                            other_robot_id = other_id

                collision_data = {
                    "collision_with": collision_with,
                    "other_robot_id": other_robot_id
                }

                if state.contact_positions:
                    position = [
                        state.contact_positions[0].x,
                        state.contact_positions[0].y,
                        state.contact_positions[0].z
                    ]
                    collision_data["position"] = position

                collision_info.append(collision_data)

                if collision_with == 'robot':
                    self.get_logger().error(f'Collision with robot {other_robot_id}')
                else:
                    self.get_logger().error(f'Collision with environment')

                
        except Exception as e:
            self.get_logger().error(f'Error in collision_callback: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def control_loop_callback(self):
        """Control loop: compute and apply control commands"""
        try:
            if self.idle:
                return
            
            if self._state is None:
                return
            
            # Check if trajectory and state are available
            if self.current_trajectory is None:
                # traj is None when first initialization or state/traj separate
                self.send_command_to_gazebo(0, 0) 
                self.update_robot_status(self.STATUS_EMERGENCY_STOP)
                return
            # if self.obstacles is None:
            #     self.obstacles = None

            if self.current_status == self.STATUS_SAFETY_STOP:
                self.get_logger().info('Control check: SAFETY STOP')
                return
            
            self.update_robot_status(self.STATUS_RUNNING)
            
            # Get current position and heading
            current_position = (self._state[0], self._state[1])
            current_heading = self._state[2]
            
            # Build trajectory list
            trajectory_list = []
            for i in range(len(self.current_trajectory.x)):
                point = (
                    self.current_trajectory.x[i],
                    self.current_trajectory.y[i],
                    self.current_trajectory.theta[i]
                )
                trajectory_list.append(point)

            traj_time = self.current_trajectory.stamp.sec + self.current_trajectory.stamp.nanosec * 1e-9
            current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec  * 1e-9

            # Compute control commands based on selected controller type
            if self.controller_type == 'pure_pursuit':
                v, omega = self.pure_pursuit.compute_control_commands(
                    current_position,
                    current_heading,
                    trajectory_list,
                    traj_time,
                    current_time
                )
            elif self.controller_type == 'lqr':
                v, omega = self.lqr_controller.compute_control_commands(
                    current_position,
                    current_heading,
                    trajectory_list
                )
            elif self.controller_type == 'lqr_update':
                v, omega, target = self.lqr_update_controller.compute_control_commands(
                    current_position,
                    current_heading,
                    trajectory_list,
                    traj_time,
                    current_time
                )
                self.publish_target_point(target)
                
            elif self.controller_type == 'cbf':

                # if self.obstacles is None:
                #     v, omega = self.lqr_update_controller.compute_control_commands(
                #         current_position,
                #         current_heading,
                #         trajectory_list,
                #         traj_time,
                #         current_time
                #     )
                #     self.get_logger().info('LQR')
                # else:
                #     v, omega = self.cbf_controller.compute_control_commands(
                #         current_position,
                #         current_heading,
                #         trajectory_list,
                #         self.obstacles
                #     )
                #     self.get_logger().debug('CBF')
                self.get_logger().warn_once(f'Unknown controller type: {self.controller_type}')
                return
            
            # Limit control commands
            v = np.clip(v, -self.max_velocity, self.max_velocity)
            omega = np.clip(omega, -self.max_angular_velocity, self.max_angular_velocity)
            
            self.get_logger().info(f'Sending Control commands ({self.controller_type}): v={v:.2f}, omega={omega:.2f}')
            
            # Send command and wait for state update
            self.send_command_to_gazebo(v, omega)
            
        except Exception as e:
            self.get_logger().error(f'Error in control_loop_callback: {str(e)}')

    def send_command_to_gazebo(self, v, omega):
        try:
            if not self.send_command_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Send command service not available')
                return

            # Create request
            request = ExecuteCommand.Request()
            request.v = float(v)
            request.omega = float(omega)

            # end request and wait to get response
            future = self.send_command_client.call_async(request)
            future.add_done_callback(self.command_response_callback)

        except Exception as e:
            self.get_logger().error(f'Error sending command to Gazebo: {str(e)}')
    
    def command_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                new_state = np.array([response.x, response.y, response.theta])
                self._state = new_state
                self.publish_state_to_cluster()
            else:
                self.get_logger().warn('Command sent but state update failed')
        except Exception as e:
            self.get_logger().error(f'Error in command response callback: {str(e)}')

    def publish_state_to_cluster(self):
        """Publish robot state to cluster"""
        try:
            # Skip if state is not initialized
            if self._state is None:
                return
                
            # Create state message
            state_msg = RobotToClusterState()
            state_msg.x = self._state[0]
            state_msg.y = self._state[1]
            state_msg.theta = self._state[2]
            state_msg.idle = self.check_termination_condition()
            state_msg.stamp = self.last_received_state_from_gazebo_time
            
            # Publish state
            self.to_cluster_state_pub.publish(state_msg)
            
            self.get_logger().debug(f'publish state to cluster x={self._state[0]:.2f} y={self._state[1]:.2f} theta={self._state[2]:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing state: {str(e)}')

    def publish_robot_status(self):
        try:
            status_msg = RobotToRvizStatus()
            status_msg.state = self.current_status
            status_msg.state_desc = self.status_descriptions[self.current_status]
            status_msg.stamp = self.get_clock().now().to_msg()

            self.status_pub.publish(status_msg)

            self.get_logger().debug(f'Published robot status: {status_msg.state_desc}')
        except Exception as e:
            self.get_logger().error(f'Error publishing robot status: {str(e)}')

    def publish_target_point(self, target_point):
        try:
            target_point_msg = RobotToRvizTargetPoint()
            target_point_msg.x = target_point[0]
            target_point_msg.y = target_point[1]
            target_point_msg.stamp = self.get_clock().now().to_msg()

            self.target_point_pub.publish(target_point_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing target point: {str(e)}')
    
    def update_robot_status(self, new_status):
        if self.current_status != new_status:
            old_state_desc = self.status_descriptions[self.current_status]
            new_state_desc = self.status_descriptions[new_status]
            self.get_logger().info(f'Robot status changed: {old_state_desc} -> {new_state_desc}')
            self.current_status = new_status

            # publish status change
            self.publish_robot_status()
    
    def check_termination_condition(self):
        current_position = self._state[:2]
        target_position = self.target_point

        distance = np.linalg.norm(current_position - target_position)

        if distance < 0.3:
            self.idle = True
            self.execute_stop()
            self.update_robot_status(self.STATUS_TARGET_REACHED)
            self.get_logger().debug(f'Robot {self.robot_id} reached target. Distance: {distance:.3f}')
            return True
        else:
            self.get_logger().debug(f'Robot {self.robot_id} have not reached target. Distance: {distance:.3f}')
            return False
    
    def load_init_state_and_target(self):
        try:
            import json

            with open(self.robot_start_path, 'r') as f:
                robot_start_data = json.load(f)

            robot_id_str = str(self.robot_id)

            if robot_id_str not in robot_start_data:
                self.get_logger().error(f'Robot ID {self.robot_id} not found in start configuration')
                return np.zeros(3), np.zeros(2)

            initial_state = np.array(robot_start_data[robot_id_str])

            self.get_logger().info(f'Loaded initial state for robot {self.robot_id}: {initial_state}')

            with open(self.robot_graph_path, 'r') as f:
                graph_data = json.load(f)

            node_dict = graph_data["node_dict"]

            import csv
            final_node_id = None

            with open(self.robot_schedule_path, 'r') as f:
                csv_reader = csv.DictReader(f)
                robot_tasks = [row for row in csv_reader if int(row['robot_id']) == self.robot_id]

                if not robot_tasks:
                    self.get_logger().error(f'No schedule found for robot {self.robot_id}')
                    return initial_state, np.zeros(2)

                final_task = max(robot_tasks, key=lambda x: float(x['ETA']))
                final_node_id = final_task['node_id']

            if final_node_id not in node_dict:
                self.get_logger().error(f'Node ID {final_node_id} not found in graph')
                return initial_state, np.zeros(2)

            target_position = np.array(node_dict[final_node_id])
            self.get_logger().info(f'Loaded target position for robot {self.robot_id}: {target_position}')

            return initial_state, target_position

        except Exception as e:
            self.get_logger().error(f'Error loading loading data: {str(e)}')
            return np.zeros(3), np.zeros(2)

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    # Create executor
    executor = rclpy.executors.MultiThreadedExecutor()
    
    # Create node
    node = RobotNode()
    executor.add_node(node)
    
    # Run executor in separate thread
    import threading
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Run async initialization
    import asyncio
    loop = asyncio.get_event_loop()
    init_success = loop.run_until_complete(node.initialize())
    
    # Shutdown if initialization fails
    if not init_success:
        node.get_logger().error('Initialization failed, shutting down')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()