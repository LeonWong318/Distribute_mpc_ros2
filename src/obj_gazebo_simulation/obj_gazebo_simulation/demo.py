import os
import rclpy
import json
import math
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

class MultiRobotSpawner(Node):
    def __init__(self):
        """ Initializes the ROS2 node and loads robot positions from JSON """
        super().__init__("multi_robot_spawner")

        # Declare parameter and get JSON path
        self.declare_parameter('robot_setup_path', '')  # Default: empty string
        self.json_file_path = self.get_parameter('robot_setup_path').value

        if not self.json_file_path:
            self.get_logger().error("robot_setup_path parameter is empty! Set it in launch file or CLI.")
            rclpy.shutdown()
            return

        try:
            self.robot_positions = self.load_robot_positions()
        except (FileNotFoundError, ValueError) as e:
            self.get_logger().error(str(e))
            rclpy.shutdown()
            return

        # Create a client to interact with Gazebo's spawn service
        self.client = self.create_client(SpawnEntity, "/spawn_entity")
        self.get_logger().info("Waiting for `/spawn_entity` service...")
        self.client.wait_for_service()
        self.get_logger().info("Connected to `/spawn_entity` service!")

        # Start spawning robots
        self.spawn_all_robots()

    def load_robot_positions(self):
        """ Loads multiple robot initial positions from a JSON file """
        if not os.path.exists(self.json_file_path):
            raise FileNotFoundError(f"JSON file {self.json_file_path} not found!")

        with open(self.json_file_path, 'r') as file:
            data = json.load(file)

        if not isinstance(data, dict) or len(data) == 0:
            raise ValueError("Invalid JSON format! Expected a dictionary of robot names and positions.")

        return data  # Returns a dictionary {robot_name: [x, y, yaw]}

    def sanitize_namespace(self, name):
        """ Ensure the robot name is a valid ROS2 namespace """
        name = name.replace(" ", "_")  # Replace spaces with underscores
        if name[0].isdigit():  # Ensure it doesn't start with a number
            name = "robot_" + name
        return name

    def spawn_robot(self, robot_name, position):
        """ Function to spawn a single robot """
        x, y, yaw = position
        sanitized_name = self.sanitize_namespace(robot_name)  # Ensure valid namespace

        # Construct SDF file path
        sdf_file_path = os.path.join(
            get_package_share_directory("obj_gazebo_simulation"), "models",
            "mobile_robot", "model.sdf"
        )

        if not os.path.exists(sdf_file_path):
            self.get_logger().error(f"SDF file {sdf_file_path} not found for {robot_name}!")
            return

        # Prepare spawn request
        request = SpawnEntity.Request()
        request.name = sanitized_name
        request.xml = open(sdf_file_path, 'r').read()
        request.robot_namespace = sanitized_name  # Use sanitized name
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = 0.0

        # Convert yaw to quaternion (for correct rotation)
        request.initial_pose.orientation.z = math.sin(yaw / 2)
        request.initial_pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info(f"Spawning {robot_name} (namespace: {sanitized_name}) at ({x}, {y}, {yaw})")

        # Call service to spawn
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"{robot_name} spawned successfully!")
        else:
            self.get_logger().error(f"Failed to spawn {robot_name}: {future.exception()}")

    def spawn_all_robots(self):
        """ Spawns all robots defined in the JSON configuration """
        for robot_name, position in self.robot_positions.items():
            self.spawn_robot(robot_name, position)

        self.get_logger().info("All robots spawned successfully!")
        self.destroy_node()
        rclpy.shutdown()

def main():
    """ Entry point for the ROS2 node """
    rclpy.init()  # Ensure ROS2 is initialized

    try:
        node = MultiRobotSpawner()  # Create the node
        rclpy.spin(node)  # Keep it running
    except Exception as e:
        node.get_logger().error(f"Exception in main: {e}")
    finally:
        node.destroy_node()  # Cleanup
        rclpy.shutdown()  # Properly shut down ROS2

if __name__ == "__main__":
    main()

