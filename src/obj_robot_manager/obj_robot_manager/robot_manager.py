import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from threading import Lock
import subprocess
import os
import json
import threading

from msg_interfaces.msg import ManagerToClusterStateSet, ClusterToManagerState, ManagerToClusterStart, GazeboToManagerState
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
                ('publish_frequency', 10.0)
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

        
        self.active_robots = [] 
        self.cluster_processes = {}  
        self.cluster_subscribers = {}
        self.robot_states = {}
        
        self.converter_states = {}
        self._converter_lock = Lock()
        self.converter_subscribers = {}
        
        self._lock = Lock()
        self._registration_lock = Lock()
        self._state_lock = Lock()
        
        self.expected_robots = set()  # Set of expected robots(from config)
        self.registered_robots = set()  # Set of registrated robots
        
        # Flag to track if global start signal has been sent
        self._global_start_sent = False
        
        self.load_config_files()
        self.parse_robot_start()
        
        # Allow parallel execution
        self.service_group = ReentrantCallbackGroup()
        
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # create registration service
        self.register_service = self.create_service(
            RegisterRobot,
            '/register_robot',
            self.handle_register_robot,
            callback_group=self.service_group
        )
        
        self.states_publisher = self.create_publisher(
            ManagerToClusterStateSet,
            '/manager/robot_states',
            self.qos_profile
        )
        
        self.start_signal_publisher = self.create_publisher(
            ManagerToClusterStart,
            '/manager/global_start',
            self.qos_profile
        )
        
        self.create_timer(
            1.0 / self.publish_frequency,
            self.publish_robot_states,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        self.get_logger().info('Robot manager initialized successfully')
    
    def load_config_files(self):
        try:
            for path_name, path in [
                ('map_path', self.map_path),
                ('graph_path', self.graph_path),
                ('schedule_path', self.schedule_path),
                ('robot_start_path', self.robot_start_path)
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
                self.robot_start = f.read()
                
            self.get_logger().info('Config files loaded successfully')
        
        except Exception as e:
            self.get_logger().error(f'Error loading config files: {str(e)}')
            raise
        
    def parse_robot_start(self):
        try:
            robot_start_data = json.loads(self.robot_start)
            self.expected_robots = set(int(robot_id) for robot_id in robot_start_data.keys())
            self.get_logger().info(f'Expected robots: {self.expected_robots}')
        except Exception as e:
            self.get_logger().error(f'Error parsing robot_start: {str(e)}')
            raise
    
    def check_all_robots_registered(self):
        if len(self.registered_robots) == len(self.expected_robots):
            self.send_global_start_signal()
    
    def send_global_start_signal(self):
        # Prevent sending multiple start signals
        if self._global_start_sent:
            return
        
        try:
            # Prepare the start signal message
            start_msg = ManagerToClusterStart()
            start_msg.stamp = self.get_clock().now().to_msg()
            
            # Publish the start signal
            self.start_signal_publisher.publish(start_msg)
            
            # Mark that start signal has been sent
            self._global_start_sent = True
            
            self.get_logger().info(
                f'Global start signal sent. '
                f'Total registered robots: {len(self.registered_robots)}'
            )
        
        except Exception as e:
            self.get_logger().error(f'Error sending global start signal: {str(e)}')
    
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
            
            # create cluster node
            # success = self.create_cluster_node(robot_id)
            success = self.create_cluster_node_with_terminal(robot_id)
            if not success:
                response.success = False
                response.message = f"Failed to create cluster node for robot {robot_id}"
                return response
            
            # Create Subscriber for cluster status
            self.create_cluster_subscriber(robot_id)
            
            self.create_converter_subscriber(robot_id)
            
            with self._registration_lock:
                self.registered_robots.add(robot_id)
                self.active_robots.append(robot_id)
                with self._state_lock:
                    self.robot_states[robot_id] = None
            
            response.success = True
            response.message = f"Robot {robot_id} registered successfully"
            self.get_logger().info(f'Robot {robot_id} registered successfully')
            self.check_all_robots_registered()
            return response
                
        except Exception as e:
            self.get_logger().error(f'Error handling registration request: {str(e)}')
            response.success = False
            response.message = f"Registration error: {str(e)}"
            return response
    
    def create_cluster_subscriber(self, robot_id):
        subscriber = self.create_subscription(
            ClusterToManagerState,
            f'/cluster_{robot_id}/state',
            lambda msg: self.cluster_state_callback(msg, robot_id),
            self.qos_profile,
            callback_group=self.service_group
        )
        
        self.cluster_subscribers[robot_id] = subscriber
        self.get_logger().info(f'Created subscriber for cluster {robot_id} state')
    
    def create_converter_subscriber(self, robot_id):
        subscriber = self.create_subscription(
            GazeboToManagerState,
            f'/robot_{robot_id}/sim_state_delayed',
            lambda msg: self.converter_state_callback(msg, robot_id),
            self.qos_profile,
            callback_group=self.service_group
        )

        self.converter_subscribers[robot_id] = subscriber
        self.get_logger().info(f'Created subscriber for converter {robot_id} state')
    
    def cluster_state_callback(self, msg, robot_id):
        with self._state_lock:
            if msg.robot_id != robot_id:
                self.get_logger().warn(f'Received state with mismatched robot_id: expected {robot_id}, got {msg.robot_id}')
                return
            
            self.robot_states[robot_id] = msg
            self.get_logger().debug(f'Updated state for robot {robot_id}: x={msg.x}, y={msg.y}, theta={msg.theta}, idle={msg.idle}')
    
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
                    self.get_logger().debug(f'Updated converter state for robot {robot_id}: x={msg.x}, y={msg.y}, theta={msg.theta}')
            else:
                self.converter_states[robot_id] = msg
                self.get_logger().debug(f'Received first converter state for robot {robot_id}: x={msg.x}, y={msg.y}, theta={msg.theta}')
    
    
    def publish_robot_states(self):
        try:
            # get locks of cluster and converter
            with self._state_lock, self._converter_lock:
                msg = ManagerToClusterStateSet()
                msg.stamp = self.get_clock().now().to_msg()
                for robot_id in self.registered_robots:
                    cluster_state = self.robot_states.get(robot_id)
                    converter_state = self.converter_states.get(robot_id)

                    if cluster_state is not None and converter_state is not None:
                        self.get_logger().debug(f'Robot {robot_id}: Both cluster and converter states available')

                        # get the latest state
                        cluster_stamp = cluster_state.stamp
                        converter_stamp = converter_state.stamp
                        merged_state = ClusterToManagerState()
                        merged_state.robot_id = robot_id

                        # Log timestamp information for debugging
                        self.get_logger().debug(f'Robot {robot_id}: Cluster stamp: {cluster_stamp.sec}.{cluster_stamp.nanosec}, Converter stamp: {converter_stamp.sec}.{converter_stamp.nanosec}')

                        if (converter_stamp.sec > cluster_stamp.sec or
                            (converter_stamp.sec == cluster_stamp.sec and converter_stamp.nanosec > cluster_stamp.nanosec)):
                            # using state from converter
                            self.get_logger().debug(f'Robot {robot_id}: Using CONVERTER state (newer timestamp)')
                            merged_state.x = converter_state.x
                            merged_state.y = converter_state.y
                            merged_state.theta = converter_state.theta
                            merged_state.stamp = converter_stamp
                        else:
                            # using state from cluster
                            self.get_logger().debug(f'Robot {robot_id}: Using CLUSTER state (newer or equal timestamp)')
                            merged_state.x = cluster_state.x
                            merged_state.y = cluster_state.y
                            merged_state.theta = cluster_state.theta
                            merged_state.stamp = cluster_stamp

                        merged_state.pred_states = cluster_state.pred_states
                        merged_state.idle = cluster_state.idle

                        msg.robot_states.append(merged_state)

                    elif cluster_state is not None:
                        # Only cluster state is available
                        self.get_logger().debug(f'Robot {robot_id}: Only CLUSTER state available, using it directly')
                        msg.robot_states.append(cluster_state)

                    elif converter_state is not None:
                        # Only converter state is available
                        self.get_logger().debug(f'Robot {robot_id}: Only CONVERTER state available, creating new state from it')
                        new_state = ClusterToManagerState()
                        new_state.robot_id = robot_id
                        new_state.x = converter_state.x
                        new_state.y = converter_state.y
                        new_state.theta = converter_state.theta
                        new_state.stamp = converter_state.stamp
                        new_state.pred_states = []
                        new_state.idle = False
                        msg.robot_states.append(new_state)

                    else:
                        # Neither state is available
                        self.get_logger().warn(f'Robot {robot_id}: Neither cluster nor converter state available')

                # Publish the merged states
                if msg.robot_states:
                    self.states_publisher.publish(msg)
                    self.get_logger().debug(f'Published merged states for {len(msg.robot_states)} robots')
                else:
                    self.get_logger().warn('No robot states to publish')

        except Exception as e:
            self.get_logger().error(f'Error publishing robot states: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def create_cluster_node(self, robot_id):
        try:
            cmd = [
                'ros2', 'launch',
                self.cluster_package,
                'obj_robot_cluster.launch.py',
                f'robot_id:={robot_id}'
            ]

            env = os.environ.copy()
            conda_prefix = os.environ.get('CONDA_PREFIX', '')
            if conda_prefix:
                pythonpath = os.environ.get('PYTHONPATH', '')
                env['PYTHONPATH'] = f"{conda_prefix}/lib/python3.8/site-packages:{pythonpath}"

            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=env
            )

            self.cluster_processes[robot_id] = process

            def monitor_output(process, robot_id):
                for line in process.stdout:
                    self.get_logger().info(f'Cluster[{robot_id}]: {line.strip()}')
                for line in process.stderr:
                    self.get_logger().error(f'Cluster[{robot_id}]: {line.strip()}')

            log_thread = threading.Thread(
                target=monitor_output,
                args=(process, robot_id),
                daemon=True
            )
            log_thread.start()

            returncode = process.poll()
            if returncode is not None and returncode != 0:
                stderr = process.stderr.read()
                self.get_logger().error(f'Failed to start cluster for robot {robot_id}: {stderr}')
                return False

            self.get_logger().info(f'Cluster node for robot {robot_id} started using launch file')
            return True

        except Exception as e:
            self.get_logger().error(f'Error creating cluster node: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
        
    def create_cluster_node_with_terminal(self, robot_id):
        try:
            cmd = [
                'ros2', 'launch',
                self.cluster_package,
                'obj_robot_cluster.launch.py',
                f'robot_id:={robot_id}'
            ]

            env = os.environ.copy()
            conda_prefix = os.environ.get('CONDA_PREFIX', '')
            if conda_prefix:
                pythonpath = os.environ.get('PYTHONPATH', '')
                env['PYTHONPATH'] = f"{conda_prefix}/lib/python3.8/site-packages:{pythonpath}"

            terminal_cmd = [
                'gnome-terminal',
                '--',
                'bash', '-c',
                f'{" ".join(cmd)}; exec bash'
            ]

            process = subprocess.Popen(
                terminal_cmd,
                env=env,
                shell=False
            )

            self.cluster_processes[robot_id] = process
            self.get_logger().info(f'Cluster node for robot {robot_id} started in new terminal using launch file')
            return True

        except Exception as e:
            self.get_logger().error(f'Error creating cluster node: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
    
    def unregister_robot(self, robot_id):
        if robot_id not in self.registered_robots:
            return

        # close cluster process
        if robot_id in self.cluster_processes:
            try:
                process = self.cluster_processes[robot_id]
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    
                del self.cluster_processes[robot_id]
                self.get_logger().info(f'Cluster process for robot {robot_id} terminated')
            except Exception as e:
                self.get_logger().error(f'Error terminating cluster process: {str(e)}')

        if robot_id in self.cluster_subscribers:
            self.destroy_subscription(self.cluster_subscribers[robot_id])
            del self.cluster_subscribers[robot_id]
            
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
    
    def __del__(self):
        for robot_id, process in list(self.cluster_processes.items()):
            try:
                self.get_logger().info(f'Terminating cluster process for robot {robot_id}')
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
            except Exception as e:
                self.get_logger().error(f'Error terminating process: {str(e)}')

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