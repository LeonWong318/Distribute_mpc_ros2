import sys
sys.path.append('src')

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np
import json

from pkg_configs.configs import MpcConfiguration, CircularRobotSpecification
from basic_motion_model.motion_model import UnicycleModel
from pkg_motion_plan.local_traj_plan import LocalTrajPlanner
from pkg_tracker_mpc.trajectory_tracker import TrajectoryTracker
from pkg_motion_plan.global_path_coordinate import GlobalPathCoordinator

import threading

from msg_interfaces.msg import (
    ClusterToManagerState, 
    ManagerToClusterStateSet, 
    ClusterToRobotTrajectory, 
    RobotToClusterState,
    ManagerToClusterStart,
    ClusterBetweenRobotHeartBeat
)

class ClusterNode(Node):
    def __init__(self):
        super().__init__('cluster_node')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', 0),
                ('control_frequency', 10.0),
                ('mpc_config_path', ''),
                ('robot_config_path', ''),
                ('map_path', ''),
                ('graph_path', ''),
                ('schedule_path', ''),
                ('robot_start_path', '')
            ]
        )
        
        self.robot_id = self.get_parameter('robot_id').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.mpc_config_path = self.get_parameter('mpc_config_path').value
        self.robot_config_path = self.get_parameter('robot_config_path').value
        self.map_path = self.get_parameter('map_path').value
        self.graph_path = self.get_parameter('graph_path').value
        self.schedule_path = self.get_parameter('schedule_path').value
        self.robot_start_path = self.get_parameter('robot_start_path').value
        
        self.get_logger().info(f'Initializing cluster node for robot {self.robot_id}')
        
        self.load_config_files()
        
        self.motion_model = UnicycleModel(sampling_time=self.config_mpc.ts)
        self.planner = LocalTrajPlanner(
            self.config_mpc.ts,
            self.config_mpc.N_hor,
            self.config_robot.lin_vel_max
        )
        
        self.heart_beat_send_period = 0.1
        self.heart_beat_check_period = 0.2
        self.waiting_for_robot = True
        self.robot_ready = False
        self.received_first_heartbeat = False
        self.last_heartbeat_time = None
        
        self.robot_state = None
        self.other_robot_states = {}
        self.predicted_trajectory = None
        self.pred_states = None
        self.idle = True
        
        self._state_lock = threading.Lock()
        self._last_state_update_time = None
        
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
            
            if time_diff > self.heart_beat_check_period * 10.0:
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
                self.get_logger().info(f'Updated state from {source}')
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
                    if self.update_robot_state(state.x, state.y, state.theta, msg.stamp, "manager"):
                        self.get_logger().info(f'Updated state from manager: x={state.x}, y={state.y}, theta={state.theta}')
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
                self.get_logger().info(f'Updated robot state: x={msg.x}, y={msg.y}, theta={msg.theta}')
            self.publish_state_to_manager(msg.stamp)

        except Exception as e:
            self.get_logger().error(f'Error in robot_state_callback: {str(e)}')

    def check_robot_ready(self):
        if self.waiting_for_robot:
            self.get_logger().debug(f'Waiting for robot {self.robot_id} to connect...')
        else:
            self.robot_check_timer.cancel()

    def start_heart_beat(self):
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
            
            self.get_logger().info('Initialization completed successfully')

        except Exception as e:
            self.get_logger().error(f'Error in initialize cluster components: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            return False
    
    def control_loop(self):
        try:
            if self.idle:
                return

            # Get current time
            current_time = self.get_clock().now().seconds_nanoseconds()
            current_time = current_time[0] + current_time[1] * 1e-9


            # Get local ref and set ref states
            current_pos = (self._state[0], self._state[1])
            ref_states, ref_speed, done = self.planner.get_local_ref(
                current_time=current_time,
                current_pos=current_pos,
                idx_check_range=10
            )

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
                
                # run controller
                self.last_actions, self.pred_states, self.current_refs, self.debug_info = self.controller.run_step(
                    static_obstacles=self.static_obstacles,
                    other_robot_states=robot_states_for_control
                )
                
                end_time = self.get_clock().now()
                duration_ms = (end_time.nanoseconds - start_time.nanoseconds) / 1e9
                self.get_logger().warning(f'Controller run_step() took {duration_ms:.3f} s')
                
                # run step
                self.controller.set_current_state(self._state)
                
                # publish traj to robot after calculating
                self.publish_trajectory_to_robot()
                
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

            traj_msg = ClusterToRobotTrajectory()
            traj_msg.stamp = self.get_clock().now().to_msg()

            traj_msg.x = []
            traj_msg.y = []
            traj_msg.theta = []

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

            if self.pred_states is not None:
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

def main(args=None):
    rclpy.init(args=args)
    
    node = ClusterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
