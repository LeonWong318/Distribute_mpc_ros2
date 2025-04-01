import os
import rclpy
import yaml
import csv
import time
import signal
import math
import json
import threading
import subprocess
from datetime import datetime
from rclpy.node import Node
from threading import Event
import math
from msg_interfaces.msg import PerformanceMetricsArray, RobotToRvizStatus

class AutoTest(Node):
    def __init__(self):
        super().__init__('obj_auto_test')
        
        self.load_test_config()
        
        self.processes = {}
        self.robot_statuses = {}
        self.robot_ids = []
        self.test_completed = Event()
        self.current_metrics = None
        self.iteration_success = False
        self.current_latency = 0.0
        self.workspace_root = os.getcwd()
        self.failure_type = None
        
        self.initialize_log_files()
        
        self.metrics_subscription = self.create_subscription(
            PerformanceMetricsArray,
            '/performance_metrics',
            self.metrics_callback,
            10
        )
        
        self.get_logger().info('AutoTest node initialized successfully')
    
    def load_test_config(self):
        """Load test configuration file"""
        # First try to get the configuration file path from parameters
        self.declare_parameter('test_config_path', 'config/test_config.yaml')
        self.declare_parameter('robot_start_path', 'data/test_data/robot_start.json')
        self.declare_parameter('log_dir', 'auto_test_logs')
        self.declare_parameter('workspace_root', os.getcwd())
        self.declare_parameter('show_terminals', True)
        
        # Read from parameters
        config_path = self.get_parameter('test_config_path').value
        self.robot_start_path = self.get_parameter('robot_start_path').value
        self.log_dir_base = self.get_parameter('log_dir').value
        self.workspace_root = self.get_parameter('workspace_root').value
        self.show_terminals = self.get_parameter('show_terminals').value
        
        # Get the latency value list from parameters (if specified)
        self.declare_parameter('latency_values', [0.0, 0.1, 0.3, 0.5, 1.0])
        self.declare_parameter('iterations_per_latency', 20)
        self.declare_parameter('timeout_seconds', 300.0)
        
        latency_param = self.get_parameter('latency_values')
        iterations_param = self.get_parameter('iterations_per_latency')
        timeout_param = self.get_parameter('timeout_seconds')
        
        # Default configuration
        self.latency_values = latency_param.value
        self.iterations_per_latency = iterations_param.value
        self.timeout_seconds = timeout_param.value
        
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                
                if config:
                    self.latency_values = config.get('latency_values', self.latency_values)
                    self.iterations_per_latency = config.get('iterations_per_latency', self.iterations_per_latency)
                    self.timeout_seconds = config.get('timeout_seconds', self.timeout_seconds)
                    self.robot_start_path = config.get('robot_start_path', self.robot_start_path)
                
                self.get_logger().info(f'Test configuration loaded from {config_path}')
            except Exception as e:
                self.get_logger().error(f'Error loading configuration: {str(e)}')
        else:
            self.get_logger().info(f'Configuration file not found: {config_path}, using default values')
        
        self.get_logger().info(f'Using latency values: {self.latency_values}')
        self.get_logger().info(f'Using iterations per latency: {self.iterations_per_latency}')
        self.get_logger().info(f'Using timeout: {self.timeout_seconds} seconds')
    
    def initialize_log_files(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_dir = os.path.join(self.workspace_root, f'{self.log_dir_base}_{timestamp}')

        os.makedirs(self.log_dir, exist_ok=True)

        self.summary_log_path = os.path.join(self.log_dir, 'summary_results.csv')
        with open(self.summary_log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'Latency', 'Success Rate', 'Timeout Rate', 'Collision Rate',
                'Avg Execution Time', 'Avg Deviation', 'Avg Path Length', 
                'Avg Linear Smoothness', 'Avg Angular Smoothness'
            ])

        self.get_logger().info(f'Log files initialized in {self.log_dir}')

    def start_simulation(self):
        try:
            # Ensure event state is reset
            self.test_completed.clear()
            self.iteration_success = None
            self.robot_statuses = {}

            # Save a list of subscription objects (to prevent garbage collection)
            self._status_subscriptions = []

            # Start the visualization node
            self.processes['visualizer'] = self.start_process_with_terminal(
                'Robot Visualizer',
                ['ros2', 'launch', 'obj_robot_visualizer', 'robot_visualizer.launch.py']
            )
            self.get_logger().info('Started visualizer node')
            time.sleep(2)

            # Start the message buffer node (set delay)
            self.processes['msg_buffer'] = self.start_process_with_terminal(
                'Message Buffer',
                ['ros2', 'launch', 'obj_msg_buffer', 'msg_buffer.launch.py', f'mean_delay:={self.current_latency}']
            )
            self.get_logger().info(f'Started message buffer node with latency {self.current_latency}')
            time.sleep(2)

            # Start the robot manager
            self.processes['manager'] = self.start_process_with_terminal(
                'Robot Manager',
                ['ros2', 'launch', 'obj_robot_manager', 'obj_robot_manager.launch.py']
            )
            self.get_logger().info('Started robot manager node')
            time.sleep(2)

            # Start Gazebo simulation
            self.processes['gazebo'] = self.start_process_with_terminal(
                'Gazebo Simulation',
                ['ros2', 'launch', 'obj_gazebo_simulation', 'gazebo_simulation.launch.py']
            )
            self.get_logger().info('Started Gazebo simulation')

            # Wait for Gazebo to initialize
            self.get_logger().info('Waiting for Gazebo to initialize (5 seconds)...')
            time.sleep(5)

            # Get the list of robot IDs and start robot nodes
            self.robot_ids = self.get_robot_ids()
            if not self.robot_ids:
                self.get_logger().error('Failed to get robot IDs')
                return False

            # Start robot nodes
            for robot_id in self.robot_ids:
                self.processes[f'robot_{robot_id}'] = self.start_process_with_terminal(
                    f'Robot {robot_id}',
                    ['ros2', 'launch', 'obj_local_robot', 'obj_local_robot.launch.py', f'robot_id:={robot_id}']
                )
                time.sleep(1)  # Increase waiting time

            self.get_logger().info(f'Started {len(self.robot_ids)} robot nodes')

            # Wait for all robot nodes to fully initialize
            time.sleep(5)

            # Create robot status subscriptions
            for robot_id in self.robot_ids:
                def make_callback(rid):
                    return lambda msg: self.robot_status_callback(msg, rid)

                subscription = self.create_subscription(
                    RobotToRvizStatus,
                    f'/robot_{robot_id}/status',
                    make_callback(robot_id),
                    10
                )
                self._status_subscriptions.append(subscription)
                self.get_logger().info(f'Created status subscription for robot {robot_id}')

            # Set initial status
            self.robot_statuses = {rid: -1 for rid in self.robot_ids}
            self.get_logger().info(f'Initialized robot statuses: {self.robot_statuses}')

            self.received_status_updates = False

            return True

        except Exception as e:
            self.get_logger().error(f'Error starting simulation: {str(e)}')
            self.cleanup_processes()
            return False
    
    def start_process_with_terminal(self, title, command):
        try:
            full_command = [
                'gnome-terminal',
                '--title', title,
                '--',
                'bash', '-c',
                f"cd {self.workspace_root} && source install/setup.bash && {' '.join(command)}; exec bash"
            ]
            
            process = subprocess.Popen(
                full_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            self.get_logger().info(f'Started terminal for {title}: {process.pid}')
            return process
            
        except Exception as e:
            self.get_logger().error(f'Error starting terminal for {title}: {str(e)}')
            raise
    
    def get_robot_ids(self):
        try:
            robot_start_path = os.path.join(self.workspace_root, self.robot_start_path)
            
            if not os.path.exists(robot_start_path):
                self.get_logger().error(f'Robot start file not found: {robot_start_path}')
                return []
            
            with open(robot_start_path, 'r') as f:
                robot_data = json.load(f)
            
            robot_ids = [int(rid) for rid in robot_data.keys()]
            self.get_logger().info(f'Found {len(robot_ids)} robots: {robot_ids}')
            return robot_ids
            
        except Exception as e:
            self.get_logger().error(f'Error getting robot IDs: {str(e)}')
            return []
    
    def robot_status_callback(self, msg, robot_id):
        old_status = self.robot_statuses.get(robot_id, -1)
        self.robot_statuses[robot_id] = msg.state

        STATUS_RUNNING = 2
        STATUS_TARGET_REACHED = 4
        STATUS_COLLISION = 6

        if msg.state == STATUS_COLLISION and self.iteration_success is None:
            self.get_logger().info(f'Robot {robot_id} collision detected, marking iteration as failed')
            self.iteration_success = False
            self.failure_type = 'collision'
            self.test_completed.set()

        if all(status == STATUS_TARGET_REACHED for status in self.robot_statuses.values()) and self.iteration_success is None:
            self.get_logger().info('All robots reached target, marking iteration as successful')
            self.iteration_success = True
            self.failure_type = None
            self.test_completed.set()
    
    def metrics_callback(self, msg):
        if self.test_completed.is_set():
            self.current_metrics = msg
            self.get_logger().info('Received performance metrics')
    
    def reset_simulation(self):
        try:
            self.test_completed.clear()
            self.current_metrics = None
            self.iteration_success = None
            self.failure_type = None 
            self.robot_statuses = {}
            self.received_status_updates = False 

            self.get_logger().info('Reset simulation state variables for next iteration')
            return True
        except Exception as e:
            self.get_logger().error(f'Error resetting simulation state: {str(e)}')
            return False
    
    def cleanup_processes(self):
        """Clean up all ROS2 related processes"""
        try:
            self_pid = os.getpid()
            self.get_logger().info(f'Self process ID: {self_pid}, preserving this process')

            result = subprocess.run(
                ['pgrep', '-f', 'ros2'], 
                capture_output=True, 
                text=True
            )

            if result.returncode == 0:
                for pid_str in result.stdout.strip().split('\n'):
                    try:
                        pid = int(pid_str)
                        if pid != self_pid and os.getpgid(pid) != os.getpgid(self_pid):
                            self.get_logger().info(f'Killing ROS2 process: {pid}')
                            os.kill(pid, signal.SIGTERM)
                    except (ValueError, ProcessLookupError) as e:
                        self.get_logger().debug(f'Error processing PID {pid_str}: {str(e)}')
        except Exception as e:
            self.get_logger().warn(f'Error during process cleanup: {str(e)}')

        self.processes = {}
        self.get_logger().info('All processes cleaned up')
    
    def wait_for_completion(self, timeout=None):
        if timeout is None:
            timeout = self.timeout_seconds

        self.get_logger().info(f'Waiting for simulation to complete (timeout: {timeout}s)')
        completed = self.test_completed.wait(timeout=timeout)

        if not completed:
            self.get_logger().warn('Simulation timed out')
            self.iteration_success = False
            self.failure_type = 'timeout'
            return False

        time.sleep(2)
        return self.iteration_success
    
    def log_iteration_results(self, latency, iteration, success, metrics=None):
       log_path = os.path.join(self.log_dir, f'latency_{latency}_iteration_{iteration}.csv')
       
       with open(log_path, 'w', newline='') as f:
           writer = csv.writer(f)
           
           writer.writerow(['Latency', latency])
           writer.writerow(['Iteration', iteration])
           writer.writerow(['Success', success])
           writer.writerow(['Failure Type', self.failure_type if not success else 'None'])
           writer.writerow([])
           
           if metrics and metrics.metrics:
               writer.writerow([
                   'Robot ID', 'Deviation Area', 'Normalized Deviation',
                   'Execution Time', 'Path Length', 'Linear Smoothness', 'Angular Smoothness'
               ])
               
               for metric in metrics.metrics:
                   writer.writerow([
                       metric.robot_id,
                       metric.deviation_area,
                       metric.normalized_deviation,
                       metric.execution_time,
                       metric.path_length,
                       metric.linear_smoothness,
                       metric.angular_smoothness
                   ])
       
       self.get_logger().info(f'Iteration results logged to {log_path}')

    def log_latency_summary(self, latency, success_rate, timeout_rate, collision_rate, avg_metrics):
        with open(self.summary_log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                latency,
                success_rate,
                timeout_rate,
                collision_rate,
                avg_metrics.get('execution_time', 'N/A'),
                avg_metrics.get('normalized_deviation', 'N/A'),
                avg_metrics.get('path_length', 'N/A'),
                avg_metrics.get('linear_smoothness', 'N/A'),
                avg_metrics.get('angular_smoothness', 'N/A')
            ])
    
    def calculate_average_metrics(self, metrics_list):
        """Calculate average metrics from multiple iterations (success only)"""
        if not metrics_list:
            return {}

        all_metrics = {
            'execution_time': [],
            'normalized_deviation': [],
            'path_length': [],
            'linear_smoothness': [],
            'angular_smoothness': []
        }

        for metrics in metrics_list:
            if not metrics or not metrics.metrics:
                continue

            for metric in metrics.metrics:
                if not math.isnan(metric.execution_time):
                    all_metrics['execution_time'].append(metric.execution_time)
                if not math.isnan(metric.normalized_deviation):
                    all_metrics['normalized_deviation'].append(metric.normalized_deviation)
                if not math.isnan(metric.path_length):
                    all_metrics['path_length'].append(metric.path_length)
                if not math.isnan(metric.linear_smoothness):
                    all_metrics['linear_smoothness'].append(metric.linear_smoothness)
                if not math.isnan(metric.angular_smoothness):
                    all_metrics['angular_smoothness'].append(metric.angular_smoothness)

        avg_metrics = {}
        for key, values in all_metrics.items():
            if values:
                avg_metrics[key] = sum(values) / len(values)
            else:
                avg_metrics[key] = 'N/A'

        return avg_metrics

    
    def run_tests(self):
        self.get_logger().info(f'Starting automated tests with {len(self.latency_values)} latency values')
        self.get_logger().info(f'Each latency will be tested {self.iterations_per_latency} times')

        for latency in self.latency_values:
            self.current_latency = latency
            self.get_logger().info(f'=== Testing latency: {latency} ===')

            success_count = 0
            timeout_count = 0
            collision_count = 0
            metrics_list = []

            for iteration in range(self.iterations_per_latency):
                self.get_logger().info(f'Starting iteration {iteration+1}/{self.iterations_per_latency}')

                if not self.start_simulation():
                    self.get_logger().error('Failed to start simulation, retrying...')
                    self.cleanup_processes()
                    continue
                
                success = self.wait_for_completion()

                if success:
                    success_count += 1
                    self.get_logger().info(f'Iteration {iteration+1} completed successfully')
                else:
                    if self.failure_type == 'timeout':
                        timeout_count += 1
                        self.get_logger().info(f'Iteration {iteration+1} failed due to timeout')
                    elif self.failure_type == 'collision':
                        collision_count += 1
                        self.get_logger().info(f'Iteration {iteration+1} failed due to collision')
                    else:
                        self.get_logger().info(f'Iteration {iteration+1} failed with unknown reason')

                self.log_iteration_results(latency, iteration, success, self.current_metrics)

                if success and self.current_metrics:  
                    metrics_list.append(self.current_metrics)

                if iteration < self.iterations_per_latency - 1:
                    self.cleanup_processes()
                    time.sleep(2)

            success_rate = success_count / self.iterations_per_latency
            timeout_rate = timeout_count / self.iterations_per_latency
            collision_rate = collision_count / self.iterations_per_latency
            avg_metrics = self.calculate_average_metrics(metrics_list)

            self.log_latency_summary(latency, success_rate, timeout_rate, collision_rate, avg_metrics)

            self.get_logger().info(f'Completed testing for latency {latency}')
            self.get_logger().info(f'Success rate: {success_rate * 100:.2f}%')
            self.get_logger().info(f'Timeout rate: {timeout_rate * 100:.2f}%')
            self.get_logger().info(f'Collision rate: {collision_rate * 100:.2f}%')

            self.cleanup_processes()
            time.sleep(5)

        self.get_logger().info('All tests completed')
        self.get_logger().info(f'Results saved to {self.log_dir}')

def main():
    rclpy.init()
    
    auto_test = AutoTest()
    
    try:
        def signal_handler(sig, frame):
            auto_test.get_logger().info('Received shutdown signal, cleaning up...')
            auto_test.cleanup_processes()
            rclpy.shutdown()
            exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        test_thread = threading.Thread(target=auto_test.run_tests)
        test_thread.daemon = True
        test_thread.start()
        
        rclpy.spin(auto_test)
    except KeyboardInterrupt:
        auto_test.get_logger().info('Keyboard interrupt received, cleaning up...')
    finally:
        auto_test.cleanup_processes()
        rclpy.shutdown()

if __name__ == '__main__':
    main()