import sys
sys.path.append('src')

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np
import json

from pkg_configs.configs import MpcConfiguration, CircularRobotSpecification
# from pkg_moving_object.moving_object import RobotObject
from basic_motion_model.motion_model import UnicycleModel
from pkg_motion_plan.local_traj_plan import LocalTrajPlanner
from pkg_tracker_mpc.trajectory_tracker import TrajectoryTracker
from pkg_motion_plan.global_path_coordinate import GlobalPathCoordinator

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
            f'/robot_{self.robot_id}/state',
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
    
    def from_manager_states_callback(self, msg: ManagerToClusterStateSet):
        try:
            self.other_robot_states.clear()
            for state in msg.robot_states:
                if state.robot_id != self.robot_id:
                    self.other_robot_states[state.robot_id] = state
                        
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
            self._state = np.array([msg.x, msg.y, msg.theta])
            self.publish_state_to_manager()

            self.get_logger().debug(f'Updated robot state: x={msg.x}, y={msg.y}, theta={msg.theta}')

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
    
    # def pre_warm_controller(self):
    #     self.get_logger().info(f"Pre-warming controller for Robot-{self.robot_id}...")
    #     dummy_obstacles = self.static_obstacles 
    #     dummy_robot_states = [-10] * (3 * (self.config_mpc.N_hor + 1) * self.config_mpc.Nother)  # 创建假的机器人状态
    #     current_time = self.get_clock().now().seconds_nanoseconds()
    #     current_time = current_time[0] + current_time[1] * 1e-9
    #     current_pos = (self._state[0], self._state[1])
    #     ref_states, ref_speed, done = self.planner.get_local_ref(
    #         current_time=current_time,
    #         current_pos=current_pos,
    #         idx_check_range=10
    #     )
    #     self.controller.set_ref_states(ref_states, ref_speed=ref_speed)
    #     import time
        
    #     for i in range(3):
    #         start_time = time.time()
    #         self.controller.run_step(
    #             static_obstacles=dummy_obstacles,
    #             other_robot_states=dummy_robot_states
    #         )
    #         end_time = time.time()
    #         self.get_logger().info(f"Pre-warm iteration {i+1} took: {end_time-start_time:.4f}s")

    #     self.get_logger().info(f"Controller pre-warming complete for Robot-{self.robot_id}")
    
    
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
            
            # self.pre_warm_controller()
            
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

            # # get other robot states
            # received_robot_states = [state for rid, state in self.other_robot_states.items()]

            # if len(received_robot_states) == len(self.expected_robots) - 1:
            #     robot_states_for_control = []
            #     for state in received_robot_states:
            #         for _ in range(self.config_mpc.N_hor + 1):
            #             robot_states_for_control.extend([state.x, state.y, state.theta])

            #     # check if state length is right
            #     required_length = 3 * (self.config_mpc.N_hor + 1) * self.config_mpc.Nother
            #     if len(robot_states_for_control) < required_length:
            #         remaining_length = required_length - len(robot_states_for_control)
            #         robot_states_for_control.extend([-10] * remaining_length)

            #     # run controller
            #     self.last_actions, self.pred_states, self.current_refs, self.debug_info = self.controller.run_step(
            #         static_obstacles=self.static_obstacles,
            #         other_robot_states=robot_states_for_control
            #     )
            
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
                
                # run controller
                self.last_actions, self.pred_states, self.current_refs, self.debug_info = self.controller.run_step(
                    static_obstacles=self.static_obstacles,
                    other_robot_states=robot_states_for_control
                )
                
                # run step
                # self.step(self.last_actions[-1])
                self.controller.set_current_state = self._state
                
                # publish traj to robot after calculating
                self.publish_trajectory_to_robot()
                
            else:
                self.get_logger().debug('Not enough other robot states, skip this control loop')
            
            if self.controller.check_termination_condition(external_check=self.planner.idle):
                self.get_logger().info('Arrived goal and entered idle state')
                self.idle = True

        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
    
    # def control_loop_with_timestamp(self):
    #     try:
    #         if self.idle:
    #             return

    #         loop_start = self.get_clock().now().seconds_nanoseconds()
    #         loop_start_time = loop_start[0] + loop_start[1] * 1e-9
    #         self.get_logger().warn(f"[TIMESTAMP][Robot-{self.robot_id}] control_loop start: {loop_start_time:.6f}s")

    #         # Get current time
    #         current_time = self.get_clock().now().seconds_nanoseconds()
    #         current_time = current_time[0] + current_time[1] * 1e-9

    #         # 记录获取本地参考轨迹开始时间
    #         t1 = self.get_clock().now().seconds_nanoseconds()
    #         t1_time = t1[0] + t1[1] * 1e-9

    #         # Get local ref and set ref states
    #         current_pos = (self._state[0], self._state[1])
    #         ref_states, ref_speed, done = self.planner.get_local_ref(
    #             current_time=current_time,
    #             current_pos=current_pos,
    #             idx_check_range=10
    #         )

    #         # 记录获取本地参考轨迹结束时间
    #         t2 = self.get_clock().now().seconds_nanoseconds()
    #         t2_time = t2[0] + t2[1] * 1e-9
    #         duration1 = t2_time - t1_time
    #         self.get_logger().warn(f"[TIMESTAMP][Robot-{self.robot_id}] get_local_ref took: {duration1:.6f}s")

    #         self.controller.set_ref_states(ref_states, ref_speed=ref_speed)

    #         # 记录控制器运行开始时间
    #         t3 = self.get_clock().now().seconds_nanoseconds()
    #         t3_time = t3[0] + t3[1] * 1e-9

    #         # get other robot states
    #         received_robot_states = [state for rid, state in self.other_robot_states.items()]

    #         if len(received_robot_states) == len(self.expected_robots) - 1:
    #             robot_states_for_control = []
    #             for state in received_robot_states:
    #                 for _ in range(self.config_mpc.N_hor + 1):
    #                     robot_states_for_control.extend([state.x, state.y, state.theta])

    #             # check if state length is right
    #             required_length = 3 * (self.config_mpc.N_hor + 1) * self.config_mpc.Nother
    #             if len(robot_states_for_control) < required_length:
    #                 remaining_length = required_length - len(robot_states_for_control)
    #                 robot_states_for_control.extend([-10] * remaining_length)

    #             # 记录run_step开始时间
    #             t4 = self.get_clock().now().seconds_nanoseconds()
    #             t4_time = t4[0] + t4[1] * 1e-9

    #             # run controller
    #             self.last_actions, self.pred_states, self.current_refs, self.debug_info = self.controller.run_step(
    #                 static_obstacles=self.static_obstacles,
    #                 other_robot_states=robot_states_for_control
    #             )

    #             # 记录run_step结束时间
    #             t5 = self.get_clock().now().seconds_nanoseconds()
    #             t5_time = t5[0] + t5[1] * 1e-9
    #             duration2 = t5_time - t4_time
    #             self.get_logger().warn(f"[TIMESTAMP][Robot-{self.robot_id}] controller.run_step took: {duration2:.6f}s")

    #             # 记录障碍物数量
    #             self.get_logger().warn(f"[INFO][Robot-{self.robot_id}] static_obstacles: {len(self.static_obstacles)}, other_robots: {len(received_robot_states)}")

    #             # run step
    #             self.step(self.last_actions[-1])

    #             # publish traj to robot after calculating
    #             t_pub_start = self.get_clock().now().seconds_nanoseconds()
    #             t_pub_start_time = t_pub_start[0] + t_pub_start[1] * 1e-9

    #             self.publish_trajectory_to_robot()

    #             t_pub_end = self.get_clock().now().seconds_nanoseconds()
    #             t_pub_end_time = t_pub_end[0] + t_pub_end[1] * 1e-9
    #             pub_duration = t_pub_end_time - t_pub_start_time
    #             self.get_logger().warn(f"[TIMESTAMP][Robot-{self.robot_id}] publish_trajectory took: {pub_duration:.6f}s")
    #         else:
    #             self.get_logger().debug('Not enough other robot states, skip this control loop')

    #         # 记录整个循环的耗时
    #         loop_end = self.get_clock().now().seconds_nanoseconds()
    #         loop_end_time = loop_end[0] + loop_end[1] * 1e-9
    #         total_duration = loop_end_time - loop_start_time
    #         self.get_logger().warn(f"[TIMESTAMP][Robot-{self.robot_id}] control_loop total duration: {total_duration:.6f}s")

    #         # 检查是否是延迟原因
    #         if total_duration > 0.5:  # 如果总时间超过0.5秒，记录额外信息
    #             self.get_logger().warn(f"[SLOWDOWN][Robot-{self.robot_id}] Slow control loop detected! Check details above.")

    #         if self.controller.check_termination_condition(external_check=self.planner.idle):
    #             self.get_logger().info('Arrived goal and entered idle state')
    #             self.idle = True

    #     except Exception as e:
    #         self.get_logger().error(f'Error in control loop: {str(e)}')

    # def set_state(self, state: np.ndarray) -> None:
    #     self._state = state
    #     self.robot_object = RobotObject(state=state, ts=self.config_robot.ts, radius=self.config_robot.vehicle_width/2)
    #     self.robot_object.motion_model = self.motion_model
        
    # def step(self, action: np.ndarray) -> None:
    #     self.robot_object.one_step(action)
    #     self._state = self.robot_object.state
    #     self.controller.set_current_state = self._state

    def publish_trajectory_to_robot(self):
        try:
            if self.pred_states is None:
                return

            traj_msg = ClusterToRobotTrajectory()
            traj_msg.stamp = self.get_clock().now().to_msg()

            traj_msg.x = []
            traj_msg.y = []
            traj_msg.theta = []

            traj_msg.x.append(float(self._state[0]))
            traj_msg.y.append(float(self._state[1]))
            traj_msg.theta.append(float(self._state[2]))

            for state in self.pred_states:
                if isinstance(state, np.ndarray):
                    traj_msg.x.append(float(state[0]))
                    traj_msg.y.append(float(state[1]))
                    traj_msg.theta.append(float(state[2]))

            self.to_robot_trajectory_pub.publish(traj_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing trajectory: {str(e)}')
        
    def publish_state_to_manager(self):
        try:
            state_msg = ClusterToManagerState()
            state_msg.robot_id = self.robot_id
            state_msg.x = self._state[0]
            state_msg.y = self._state[1]
            state_msg.theta = self._state[2]
            state_msg.idle = self.idle
            state_msg.stamp = self.get_clock().now().to_msg()

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
