import rclpy
import heapq
import numpy as np
import sys
import json
import os
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from time import time
from typing import Dict, List, Tuple, Any, Type

# PyQt5 for GUI
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QCheckBox, QComboBox, QPushButton, QGroupBox
from PyQt5.QtCore import Qt, QTimer

# Import the message types used in your cluster system
from msg_interfaces.msg import (
    ClusterToRobotTrajectory,
    RobotToClusterState,
    ClusterBetweenRobotHeartBeat,
)

class TopicConfig:
    def __init__(self, msg_type, topic_in, topic_out, enabled=True, mean_delay=0.2):
        self.msg_type = msg_type
        self.topic_in = topic_in
        self.topic_out = topic_out
        self.enabled = enabled
        self.mean_delay = mean_delay

class MultiTopicMessageBuffer(Node, QWidget):
    def __init__(self):
        # Initialize ROS2 Node
        Node.__init__(self, 'multi_topic_message_buffer')

        # Parameter declaration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_start_path', ''),  # Path to the robot_start.json file
                ('enable_gui', True),
                ('mean_delay', 0.2)
            ]
        )
        
        # Get robot IDs from JSON file
        robot_start_path = self.get_parameter('robot_start_path').value
        if not robot_start_path:
            self.get_logger().error('robot_start_path parameter is required')
            raise ValueError('robot_start_path parameter is required')
            
        self.robot_ids = self.load_robot_ids(robot_start_path)
        self.get_logger().info(f'Loaded {len(self.robot_ids)} robot IDs: {self.robot_ids}')
        
        self.enable_gui = self.get_parameter('enable_gui').value
        self.mean_delay = self.get_parameter('mean_delay').value

        # Initialize Qt only if GUI is enabled
        if self.enable_gui:
            # Make sure QApplication exists
            if not QApplication.instance():
                self.app = QApplication(sys.argv)
            QWidget.__init__(self)

        self.get_logger().info(f'Initializing message buffer')
        
        # Create callback group for all subscribers
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize buffers and subscriptions
        self._buffers: Dict[str, List[Tuple[float, Any]]] = {}  # topic -> [(release_time, message)]
        self._subscribers = {}
        self._publishers = {}
        self._topic_configs = {}
        
        # QoS profiles
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
        
        # Initialize topic configurations
        self.setup_topic_configs()
        
        # Create publishers and subscribers based on configurations
        self.setup_pub_sub()
        
        # Timer to publish pending messages
        self.timer = self.create_timer(0.01, self.publish_pending_msgs)
        
        # Initialize GUI if enabled
        if self.enable_gui:
            QWidget.__init__(self)
            self.init_gui()
            
            # Qt Timer to update ROS2
            self.qt_timer = QTimer()
            self.qt_timer.timeout.connect(self.spin_once)
            self.qt_timer.start(10)  # Run ROS2 event loop every 10ms
        
        self.get_logger().info("Multi-Topic Message Buffer Started")
    
    def load_robot_ids(self, config_path):
        """Load robot IDs from a JSON file."""
        try:
            # Check if file exists
            if not os.path.exists(config_path):
                self.get_logger().error(f'Robot configuration file not found: {config_path}')
                raise FileNotFoundError(f'Robot configuration file not found: {config_path}')
            
            # Load and parse JSON
            with open(config_path, 'r') as f:
                config = json.load(f)
            
            # Extract and convert robot IDs to integers
            robot_ids = [int(robot_id) for robot_id in config.keys()]
            
            if not robot_ids:
                self.get_logger().error('No robot IDs found in configuration file')
                raise ValueError('No robot IDs found in configuration file')
                
            return robot_ids
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse robot configuration file: {e}')
            raise ValueError(f'Failed to parse robot configuration file: {e}')
        except Exception as e:
            self.get_logger().error(f'Error loading robot configuration: {e}')
            raise
    
    def setup_topic_configs(self):
        # For each robot, set up the topics
        for robot_id in self.robot_ids:
            # Cluster to Robot topics
            self._topic_configs[f"cluster_{robot_id}_trajectory"] = TopicConfig(
                ClusterToRobotTrajectory,
                f"/cluster_{robot_id}/trajectory",
                f"/cluster_{robot_id}/trajectory_delayed",
                True,
                self.mean_delay
            )
            
            self._topic_configs[f"cluster_{robot_id}_heartbeat"] = TopicConfig(
                ClusterBetweenRobotHeartBeat,
                f"/cluster_{robot_id}/heartbeat",
                f"/cluster_{robot_id}/heartbeat_delayed",
                True,
                self.mean_delay
            )
            
            # Robot to Cluster topics
            self._topic_configs[f"robot_{robot_id}_state"] = TopicConfig(
                RobotToClusterState,
                f"/robot_{robot_id}/state",
                f"/robot_{robot_id}/state_delayed",
                True,
                self.mean_delay
            )
            
            self._topic_configs[f"robot_{robot_id}_heartbeat"] = TopicConfig(
                ClusterBetweenRobotHeartBeat,
                f"/robot_{robot_id}/heartbeat",
                f"/robot_{robot_id}/heartbeat_delayed",
                True,
                self.mean_delay
            )
            
            # Gazebo to Manager topics
            self._topic_configs[f"robot_{robot_id}_sim_state"] = TopicConfig(
                ClusterBetweenRobotHeartBeat,
                f"/robot_{robot_id}/sim_state",
                f"/robot_{robot_id}/sim_state_delayed",
                True,
                self.mean_delay
            )
    
    def setup_pub_sub(self):
        # Create a buffer, subscriber and publisher for each topic
        for topic_id, config in self._topic_configs.items():
            if not config.enabled:
                continue
                
            # Initialize buffer for this topic
            self._buffers[topic_id] = []
            
            # Determine appropriate QoS profile
            qos = self.reliable_qos if "heartbeat" not in topic_id else self.best_effort_qos
            
            # Create subscriber with appropriate callback
            self._subscribers[topic_id] = self.create_subscription(
                config.msg_type,
                config.topic_in,
                lambda msg, tid=topic_id: self.message_callback(msg, tid),
                qos,
                callback_group=self.callback_group
            )
            
            # Create publisher
            self._publishers[topic_id] = self.create_publisher(
                config.msg_type,
                config.topic_out,
                qos
            )
            
            self.get_logger().info(f"Set up buffer for {topic_id}: {config.topic_in} -> {config.topic_out}")
    
    def message_callback(self, msg, topic_id):
        """Handles incoming messages and buffers them with a Poisson delay."""
        config = self._topic_configs[topic_id]
        
        if not config.enabled:
            # If buffering is disabled for this topic, publish immediately
            self._publishers[topic_id].publish(msg)
            return
        
        # Generate delay from Poisson distribution
        delay = np.random.exponential(scale=config.mean_delay)
        release_time = time() + delay
        
        # Add to buffer with release time
        heapq.heappush(self._buffers[topic_id], (release_time, msg))
        
        # Log information about buffered message
        self.get_logger().debug(f"Buffered {topic_id} message with delay: {delay:.2f} sec")
    
    def publish_pending_msgs(self):
        """Publishes messages whose delay time has expired for all topics."""
        now = time()
        
        for topic_id, buffer in self._buffers.items():
            # Skip if buffer is empty
            if not buffer:
                continue
                
            # Publish all messages whose release time has passed
            while buffer and buffer[0][0] <= now:
                _, msg = heapq.heappop(buffer)
                self._publishers[topic_id].publish(msg)
                self.get_logger().debug(f"Published delayed {topic_id} message")
    
    def init_gui(self):
        """Initialize the GUI for adjusting delay parameters."""
        self.setWindowTitle("ROS2 Multi-Topic Message Buffer")
        self.setGeometry(100, 100, 800, 600)
        
        # Main layout
        main_layout = QVBoxLayout()
        
        # Add global controls
        global_group = QGroupBox("Global Settings")
        global_layout = QVBoxLayout()
        
        # Global delay control
        global_delay_layout = QHBoxLayout()
        global_delay_layout.addWidget(QLabel("Global Mean Delay:"))
        
        self.global_delay_slider = QSlider(Qt.Horizontal)
        self.global_delay_slider.setMinimum(0)  # 0 sec minimum
        self.global_delay_slider.setMaximum(1000)  # 10 sec max (scaled by 100)
        self.global_delay_slider.setValue(int(self.mean_delay * 100))
        self.global_delay_slider.setTickInterval(100)
        self.global_delay_slider.setTickPosition(QSlider.TicksBelow)
        self.global_delay_slider.valueChanged.connect(self.global_delay_changed)
        global_delay_layout.addWidget(self.global_delay_slider)
        
        self.global_delay_label = QLabel(f"{self.mean_delay:.2f} sec")
        global_delay_layout.addWidget(self.global_delay_label)
        
        global_layout.addLayout(global_delay_layout)
        
        # Apply global delay button
        apply_button = QPushButton("Apply Global Delay to All")
        apply_button.clicked.connect(self.apply_global_delay)
        global_layout.addWidget(apply_button)
        
        global_group.setLayout(global_layout)
        main_layout.addWidget(global_group)
        
        # Topic specific controls
        topics_group = QGroupBox("Topic Specific Settings")
        topics_layout = QVBoxLayout()
        
        # Create a section for each topic category
        categories = {
            "Cluster to Robot": [t for t in self._topic_configs if "cluster" in t],
            "Robot to Cluster": [t for t in self._topic_configs if "robot" in t and not "sim" in t],
            "Gazebo to Manager": [t for t in self._topic_configs if "sim" in t]
        }
        
        # Create widgets for each topic
        self.topic_widgets = {}
        
        for category, topic_ids in categories.items():
            if not topic_ids:
                continue
                
            category_group = QGroupBox(category)
            category_layout = QVBoxLayout()
            
            for topic_id in topic_ids:
                config = self._topic_configs[topic_id]
                
                # Create layout for this topic
                topic_layout = QHBoxLayout()
                
                # Enable checkbox
                enable_checkbox = QCheckBox()
                enable_checkbox.setChecked(config.enabled)
                enable_checkbox.stateChanged.connect(lambda state, tid=topic_id: self.topic_enabled_changed(state, tid))
                topic_layout.addWidget(enable_checkbox)
                
                # Topic label
                topic_label = QLabel(f"{topic_id}:")
                topic_layout.addWidget(topic_label)
                
                # Delay slider
                delay_slider = QSlider(Qt.Horizontal)
                delay_slider.setMinimum(0)
                delay_slider.setMaximum(1000)  # 10 sec max (scaled by 100)
                delay_slider.setValue(int(config.mean_delay * 100))
                delay_slider.setTickInterval(100)
                delay_slider.valueChanged.connect(lambda value, tid=topic_id: self.topic_delay_changed(value, tid))
                topic_layout.addWidget(delay_slider)
                
                # Delay value label
                delay_label = QLabel(f"{config.mean_delay:.2f} sec")
                topic_layout.addWidget(delay_label)
                
                # Store widgets for later reference
                self.topic_widgets[topic_id] = {
                    "enable": enable_checkbox,
                    "slider": delay_slider,
                    "label": delay_label
                }
                
                category_layout.addLayout(topic_layout)
            
            category_group.setLayout(category_layout)
            topics_layout.addWidget(category_group)
        
        topics_group.setLayout(topics_layout)
        main_layout.addWidget(topics_group)
        
        self.setLayout(main_layout)
    
    def global_delay_changed(self, value):
        """Updates the global mean delay when the slider is moved."""
        delay = value / 100.0
        self.global_delay_label.setText(f"{delay:.2f} sec")
    
    def apply_global_delay(self):
        """Applies the global delay setting to all topics."""
        global_delay = self.global_delay_slider.value() / 100.0
        
        for topic_id, config in self._topic_configs.items():
            config.mean_delay = global_delay
            
            # Update UI
            if topic_id in self.topic_widgets:
                self.topic_widgets[topic_id]["slider"].setValue(int(global_delay * 100))
                self.topic_widgets[topic_id]["label"].setText(f"{global_delay:.2f} sec")
        
        self.get_logger().info(f"Applied global delay of {global_delay:.2f} sec to all topics")
    
    def topic_enabled_changed(self, state, topic_id):
        """Updates whether a topic is enabled for buffering."""
        enabled = (state == Qt.Checked)
        self._topic_configs[topic_id].enabled = enabled
        self.get_logger().info(f"Set {topic_id} enabled: {enabled}")
    
    def topic_delay_changed(self, value, topic_id):
        """Updates the mean delay for a specific topic."""
        delay = value / 100.0
        self._topic_configs[topic_id].mean_delay = delay
        self.topic_widgets[topic_id]["label"].setText(f"{delay:.2f} sec")
        self.get_logger().debug(f"Updated {topic_id} mean delay to {delay:.2f} sec")
    
    def spin_once(self):
        """Processes a single ROS2 event."""
        rclpy.spin_once(self, timeout_sec=0.01)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        buffer_node = MultiTopicMessageBuffer()
        
        if buffer_node.enable_gui:
            # Run with Qt event loop if GUI is enabled
            app = QApplication(sys.argv if args is None else args)
            buffer_node.show()
            app.exec_()
        else:
            # Run with regular ROS2 spin if GUI is disabled
            try:
                rclpy.spin(buffer_node)
            except KeyboardInterrupt:
                pass
        
        buffer_node.destroy_node()
    except Exception as e:
        import traceback
        print(f"Error initializing buffer node: {e}")
        print(traceback.format_exc())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()