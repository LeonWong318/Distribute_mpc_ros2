import sys
sys.path.append('src')

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
from pkg_local_control.pure_pursuit import PurePursuit

from msg_interfaces.msg import RobotState, RobotStatesQuery, Trajectory


class RobotNode(Node):
    async def initialize(self):
        # Any asynchronous initialization can be done here (e.g., waiting for services)
        pass

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
                ('lookahead_distance', 0.5)
            ]
        )

        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value

        # Compute sampling time
        self.Ts = 1.0 / self.control_frequency

        # Tuning parameter for velocity reduction at high curvature (set as needed)
        alpha = 1.0

        # Instantiate the Pure Pursuit controller with the state-space parameters
        self.pure_pursuit = PurePursuit(self.lookahead_distance, self.Ts, self.max_velocity, alpha)

        # Initialize storage for the latest state and trajectory messages
        self.current_state = None        # Expected to be a RobotState message
        self.current_trajectory = None   # Expected to be a Trajectory message

        # Setup QoS profile for subscribers and publisher
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber: listen for the robot's state (e.g., on 'robot_state' topic)
        self.state_sub = self.create_subscription(
            RobotState,
            'robot_state',
            self.robot_state_callback,
            qos_profile
        )

        # Subscriber: listen for the planned trajectory (e.g., on 'trajectory' topic)
        self.trajectory_sub = self.create_subscription(
            Trajectory,
            'trajectory',
            self.trajectory_callback,
            qos_profile
        )

        # Publisher: send computed control commands (e.g., on 'robot_cmd' topic)
        self.cmd_pub = self.create_publisher(
            RobotStatesQuery,
            'robot_cmd',
            qos_profile
        )

        # Create a timer to execute the control loop at the desired frequency
        timer_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(timer_period, self.control_loop_callback)

    def robot_state_callback(self, msg: RobotState):
        """
        Callback to receive and store the current robot state.
        Expected message fields: x, y, theta.
        """
        self.current_state = msg
        self.get_logger().debug(f"Received robot state: {msg}")

    def trajectory_callback(self, msg: Trajectory):
        """
        Callback to receive and store the planned trajectory.
        Expected that msg.points is a list of RobotState messages.
        """
        self.current_trajectory = msg
        self.get_logger().debug(f"Received trajectory with {len(msg.points)} points")

    def control_loop_callback(self):
        """
        Control loop timer callback: computes and publishes the control command
        based on the current robot state and trajectory.
        """
        if self.current_state is None or self.current_trajectory is None:
            self.get_logger().warn("Waiting for both robot state and trajectory messages")
            return

        # Extract current state (assumes RobotState has fields: x, y, theta)
        current_position = (self.current_state.x, self.current_state.y)
        current_heading = self.current_state.theta

        # Extract the trajectory as a list of waypoints (each a tuple: (x, y, theta))
        trajectory_list = []
        for pt in self.current_trajectory.points:
            trajectory_list.append((pt.x, pt.y, pt.theta))

        # Compute control commands using Pure Pursuit
        v, omega = self.pure_pursuit.compute_control_commands(current_position, current_heading, trajectory_list)

        # Saturate the computed commands with the maximum allowable velocities
        v = np.clip(v, 0, self.max_velocity)
        omega = np.clip(omega, -self.max_angular_velocity, self.max_angular_velocity)

        self.get_logger().info(f"Computed control: v = {v:.2f} m/s, omega = {omega:.2f} rad/s")

        # Construct and publish the control command message
        cmd_msg = RobotStatesQuery()
        cmd_msg.robot_id = self.robot_id
        cmd_msg.v = float(v)
        cmd_msg.omega = float(omega)
        cmd_msg.stamp = self.get_clock().now().to_msg()

        self.cmd_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()

    node = RobotNode()
    executor.add_node(node)

    # Run the executor in a separate thread
    import threading
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Run asynchronous initialization if needed
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


if __name__ == "__main__":
    main()
