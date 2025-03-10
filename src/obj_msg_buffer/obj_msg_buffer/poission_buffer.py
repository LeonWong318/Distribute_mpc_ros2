import rclpy
import heapq
import numpy as np
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String  # Replace with a generic message type
from typing import Generic, TypeVar
from time import time

# PyQt5 for GUI
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider
from PyQt5.QtCore import Qt, QTimer

T = TypeVar('T')  # Generic message type

class PoissonMessageBuffer(Node, Generic[T], QWidget):
    def __init__(self, msg_type, topic_in: str, topic_out: str):
        # Initialize ROS2 Node
        Node.__init__(self, 'poisson_message_buffer')
        self.msg_type = msg_type
        self.mean_delay = 1.0  # Default mean delay in seconds
        self.buffer = []  # Priority queue: (release_time, message)

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.sub = self.create_subscription(msg_type, topic_in, self.callback, qos)
        self.pub = self.create_publisher(msg_type, topic_out, qos)
        self.timer = self.create_timer(0.1, self.publish_pending_msgs)

        self.get_logger().info("Poisson Message Buffer Started")

        # Initialize PyQt GUI
        QWidget.__init__(self)
        self.setWindowTitle("Adjust Poisson Mean Delay")
        self.layout = QVBoxLayout()

        self.label = QLabel(f"Mean Delay: {self.mean_delay:.1f} sec")
        self.layout.addWidget(self.label)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)  # 0 sec minimum
        self.slider.setMaximum(500)  # 5 sec max (scaled by 100)
        self.slider.setValue(int(self.mean_delay * 100))  # Default position
        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.valueChanged.connect(self.slider_changed)

        self.layout.addWidget(self.slider)
        self.setLayout(self.layout)

        # Qt Timer to update ROS2
        self.qt_timer = QTimer()
        self.qt_timer.timeout.connect(self.spin_once)
        self.qt_timer.start(10)  # Run ROS2 event loop every 10ms

    def callback(self, msg: T):
        """Handles incoming messages and buffers them with a Poisson delay."""
        delay = np.random.poisson(lam=self.mean_delay)
        release_time = time() + delay
        heapq.heappush(self.buffer, (release_time, msg))
        self.get_logger().info(f"Buffered message with delay: {delay:.2f} sec")

    def publish_pending_msgs(self):
        """Publishes messages whose delay time has expired."""
        now = time()
        while self.buffer and self.buffer[0][0] <= now:
            _, msg = heapq.heappop(self.buffer)
            self.pub.publish(msg)
            self.get_logger().info("Published delayed message.")

    def slider_changed(self, value):
        """Updates the mean delay when the slider is moved."""
        self.mean_delay = value / 100.0  # Convert back from slider scale
        self.label.setText(f"Mean Delay: {self.mean_delay:.1f} sec")
        self.get_logger().info(f"Updated mean delay to {self.mean_delay}")

    def spin_once(self):
        """Processes a single ROS2 event."""
        rclpy.spin_once(self, timeout_sec=0.01)

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = PoissonMessageBuffer(String, 'input_topic', 'output_topic')
    node.show()
    app.exec_()  # Start PyQt event loop
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
