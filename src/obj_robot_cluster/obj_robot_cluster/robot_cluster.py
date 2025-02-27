import sys
sys.path.append('src')

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import json

from pkg_configs.configs import MpcConfiguration, CircularRobotSpecification
from pkg_moving_object.moving_object import RobotObject
from basic_motion_model.motion_model import UnicycleModel
from pkg_motion_plan.local_traj_plan import LocalTrajPlanner
from pkg_tracker_mpc.trajectory_tracker import TrajectoryTracker
from pkg_motion_plan.global_path_coordinate import GlobalPathCoordinator

from msg_interfaces.msg import ClusterToManagerState, ManagerToClusterStateSet, ClusterToRobotTrajectory, RobotToClusterState

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
        self.publish_frequency = 10.0
        self.mpc_config_path = self.get_parameter('mpc_config_path').value
        self.robot_config_path = self.get_parameter('robot_config_path').value
        self.map_path = self.get_parameter('map_path').value
        self.graph_path = self.get_parameter('graph_path').value
        self.schedule_path = self.get_parameter('schedule_path').value
        self.robot_start_path = self.get_parameter('robot_start_path').value
        
        self.get_logger().info(f'Initializing cluster node for robot {self.robot_id}')
        
        self.load_config_files()
        
        self.create_pub_and_sub()

        self.robot_state = None
        self.other_robot_states = {}
        self.predicted_trajectory = None
        self.idle = True
        
        self.initialize_cluster_components()
        
        self.get_logger().info(f'Cluster node initialized for robot {self.robot_id}')
    
    def create_pub_and_sub(self):
        self.callback_group = ReentrantCallbackGroup()
        
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.robot_state_sub = self.create_subscription(
            RobotToClusterState,
            f'/robot_{self.robot_id}/state',
            self.robot_state_callback,
            self.reliable_qos,
            callback_group=self.callback_group
        )
        
        self.manager_states_sub = self.create_subscription(
            ManagerToClusterStateSet,
            '/manager/robot_states',
            self.manager_states_callback,
            self.reliable_qos,
            callback_group=self.callback_group
        )
        
        self.manager_state_pub = self.create_publisher(
            ClusterToManagerState,
            f'/cluster_{self.robot_id}/state',
            self.reliable_qos
        )
        
        self.robot_trajectory_pub = self.create_publisher(
            ClusterToRobotTrajectory,
            f'/cluster_{self.robot_id}/trajectory',
            self.reliable_qos
        )
    
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
        self.motion_model = UnicycleModel(sampling_time=self.config_mpc.ts)
        self.planner = LocalTrajPlanner(
            self.config_mpc.ts,
            self.config_mpc.N_hor,
            self.config_robot.lin_vel_max
        )
        
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
    
    def robot_states_callback(self, msg: ManagerToClusterStateSet):
        try:
            self.other_robot_states.clear()
            for state in msg.robot_states:
                if state.robot_id != self.robot_id:
                    self.other_robot_states[state.robot_id] = state
                        
        except Exception as e:
            self.get_logger().error(f'Error processing robot states: {str(e)}')
            rclpy.shutdown()
    
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
            
            self.set_state(np.asarray(self.robot_start[str(self.robot_id)]))

            # add schedule
            self.add_schedule()
            
            # Timer for control loop
            self.create_timer(
                1.0/self.control_frequency,
                self.control_loop
            )

            # Timer for publish states
            self.create_timer(
                1.0 / self.publish_frequency,
                self.publish_state_to_manager,
                callback_group=self.callback_group
            )
            
            self.get_logger().info('Initialization completed successfully')

        except Exception as e:
            self.get_logger().error(f'Error in initialize_after_map_data: {str(e)}')
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
                robot_states_for_control = []
                for state in received_robot_states:
                    for _ in range(self.config_mpc.N_hor + 1):
                        robot_states_for_control.extend([state.x, state.y, state.theta])

                # check if state length is right
                required_length = 3 * (self.config_mpc.N_hor + 1) * self.config_mpc.Nother
                if len(robot_states_for_control) < required_length:
                    remaining_length = required_length - len(robot_states_for_control)
                    robot_states_for_control.extend([-10] * remaining_length)

                # run controller
                self.last_actions, self.pred_states, self.current_refs, self.debug_info = self.controller.run_step(
                    static_obstacles=self.static_obstacles,
                    other_robot_states=robot_states_for_control
                )
                
                # run step
                self.step(self.last_actions[-1])
                
            else:
                self.get_logger().info('Not enough other robot states, skip this control loop')

            self.publish_state()
            
            if self.controller.check_termination_condition(external_check=self.planner.idle):
                self.get_logger().info('Arrived goal and entered idle state')
                self.idle = True

        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
    

    def set_state(self, state: np.ndarray) -> None:
        self._state = state
        self.robot_object = RobotObject(state=state, ts=self.config_robot.ts, radius=self.config_robot.vehicle_width/2)
        self.robot_object.motion_model = self.motion_model
        
    def step(self, action: np.ndarray) -> None:
        self.robot_object.one_step(action)
        self._state = self.robot_object.state
        self.controller.set_current_state = self._state
    
    def publish_state(self):
        """发布机器人状态"""
        try:

            state_msg = RobotState()
            state_msg.robot_id = self.robot_id
            state_msg.x = self._state[0]
            state_msg.y = self._state[1]
            state_msg.theta = self._state[2]
            state_msg.idle = self.idle
            state_msg.stamp = self.get_clock().now().to_msg()
            traj_msg = Trajectory()
            traj_msg.robot_id = state_msg.robot_id
            traj_msg.x = state_msg.x
            traj_msg.y = state_msg.y 
            traj_msg.theta = state_msg.theta

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
                traj_msg.pred_states = state_msg.pred_states

            self.state_publisher.publish(state_msg)
            self.traj_publisher.publish(traj_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in update_and_publish_state: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    executor = rclpy.executors.MultiThreadedExecutor()
    
    node = RobotNode()
    executor.add_node(node)
    
    import threading
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    import asyncio
    loop = asyncio.get_event_loop()
    loop.run_until_complete(node.initialize())
    
    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

