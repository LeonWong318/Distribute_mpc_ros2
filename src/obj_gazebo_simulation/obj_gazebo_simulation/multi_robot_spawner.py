import os
import rclpy
import json
import math
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import xml.etree.ElementTree as ET
import subprocess

class MultiRobotSpawner(Node):
    def __init__(self):
        """ Initializes the ROS2 node and loads robot positions from JSON """
        super().__init__("multi_robot_spawner")

        # Declare parameter and get JSON path
        self.declare_parameter('robot_setup_path', '')  # Default: empty string
        self.robot_setup_path = self.get_parameter('robot_setup_path').value
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
        
        # Start converter nodes for each robot
        #self.start_converters()
        self.start_converters_with_terminal()

    def load_robot_positions(self):
        """ Loads multiple robot initial positions from a JSON file """
        if not os.path.exists(self.robot_setup_path):
            raise FileNotFoundError(f"JSON file {self.robot_setup_path} not found!")

        with open(self.robot_setup_path, 'r') as file:
            data = json.load(file)

        if not isinstance(data, dict) or len(data) == 0:
            raise ValueError("Invalid JSON format! Expected a dictionary of robot IDs and positions.")

        return data

    def modify_sdf_for_interface(self, sdf_content, robot_namespace):
        """
        Modify the SDF content to include the necessary plugins for ROS2 interfacing
        
        Args:
            sdf_content: Original SDF content
            robot_namespace: Namespace for the robot
            
        Returns:
            Modified SDF content with ROS2 interface plugins
        """
        try:
            root = ET.fromstring(sdf_content)

            model = root.find(".//model")
            if model is None:
                self.get_logger().error("No model element found in SDF file!")
                return sdf_content

            model.set("name", robot_namespace)

            diff_drive = ET.SubElement(model, "plugin")
            diff_drive.set("name", "diff_drive")
            diff_drive.set("filename", "libgazebo_ros_diff_drive.so")

            ros_elem = ET.SubElement(diff_drive, "ros")
            ET.SubElement(ros_elem, "namespace").text = f"/{robot_namespace}"
            ET.SubElement(ros_elem, "remapping").text = f"cmd_vel:=cmd_vel"
            ET.SubElement(ros_elem, "remapping").text = f"odom:=odom"

            ET.SubElement(diff_drive, "left_joint").text = "left_wheel_hinge"
            ET.SubElement(diff_drive, "right_joint").text = "right_wheel_hinge"
            ET.SubElement(diff_drive, "wheel_separation").text = "0.26"
            ET.SubElement(diff_drive, "wheel_diameter").text = "0.2"
            ET.SubElement(diff_drive, "max_wheel_torque").text = "20"
            ET.SubElement(diff_drive, "max_wheel_acceleration").text = "1.0"
            ET.SubElement(diff_drive, "publish_odom").text = "true"
            ET.SubElement(diff_drive, "publish_odom_tf").text = "true"
            ET.SubElement(diff_drive, "publish_wheel_tf").text = "true"
            ET.SubElement(diff_drive, "odometry_frame").text = "odom"
            ET.SubElement(diff_drive, "robot_base_frame").text = f"{robot_namespace}/chassis"

            laser_sensor = root.find(".//sensor[@name='laser']")
            if laser_sensor is not None:
                laser_plugin = ET.SubElement(laser_sensor, "plugin")
                laser_plugin.set("name", "laser")
                laser_plugin.set("filename", "libgazebo_ros_ray_sensor.so")

                laser_ros = ET.SubElement(laser_plugin, "ros")
                ET.SubElement(laser_ros, "namespace").text = f"/{robot_namespace}"
                ET.SubElement(laser_ros, "argument").text = f"--ros-args --remap ~/out:={robot_namespace}/scan"

                ET.SubElement(laser_plugin, "output_type").text = "sensor_msgs/LaserScan"

            return ET.tostring(root, encoding='unicode')

        except Exception as e:
            self.get_logger().error(f"Error modifying SDF: {str(e)}")
            import traceback
            traceback.print_exc()
            return sdf_content

    def spawn_robot(self, robot_id, position):
        """ Function to spawn a single robot with a specific ID """
        x, y, yaw = position
        
        # Create a proper namespace for the robot
        robot_namespace = f"robot_{robot_id}"
        
        # Construct SDF file path
        sdf_file_path = os.path.join(
            get_package_share_directory("obj_gazebo_simulation"), "models",
            "mobile_robot", "model.sdf"
        )

        if not os.path.exists(sdf_file_path):
            self.get_logger().error(f"SDF file {sdf_file_path} not found for robot {robot_id}!")
            return

        # Load and modify SDF content
        with open(sdf_file_path, 'r') as file:
            sdf_content = file.read()
        
        # Modify SDF to add ROS2 interface plugins
        modified_sdf = self.modify_sdf_for_interface(sdf_content, robot_namespace)

        # Prepare spawn request
        request = SpawnEntity.Request()
        request.name = robot_namespace
        request.xml = modified_sdf
        request.robot_namespace = robot_namespace
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = 0.0

        # Convert yaw to quaternion (for correct rotation)
        request.initial_pose.orientation.z = math.sin(yaw / 2)
        request.initial_pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info(f"Spawning robot with ID {robot_id} (namespace: {robot_namespace}) at ({x}, {y}, {yaw})")

        # Call service to spawn
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Robot {robot_id} spawned successfully!")
            return True
        else:
            self.get_logger().error(f"Failed to spawn robot {robot_id}: {future.exception()}")
            return False

    def spawn_all_robots(self):
        """ Spawns all robots defined in the JSON configuration """
        self.spawned_robots = []
        
        for robot_id, position in self.robot_positions.items():
            success = self.spawn_robot(robot_id, position)
            if success:
                self.spawned_robots.append(robot_id)

        if self.spawned_robots:
            self.get_logger().info(f"Successfully spawned {len(self.spawned_robots)} robots!")
        else:
            self.get_logger().error("Failed to spawn any robots!")
    
    def start_converters_with_terminal(self):
        """Start converter nodes for each successfully spawned robot using a terminal and ros2 launch"""
        self.converter_processes = []

        for robot_id in self.spawned_robots:
            try:
                cmd = [
                    'gnome-terminal', '--', 'bash', '-c',
                    f'ros2 launch obj_gazebo_simulation gazebo_converter.launch.py robot_id:={robot_id}; exec bash'
                ]

                process = subprocess.Popen(
                    cmd, 
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )

                self.converter_processes.append(process)
                self.get_logger().info(f"Started converter for robot: {robot_id}")

            except Exception as e:
                self.get_logger().error(f"Failed to start converter for robot {robot_id}: {str(e)}")

        self.get_logger().info("All converters started with launch files")

    def start_converters(self):
        """Start converter nodes for each successfully spawned robot using ros2 launch without terminal"""
        self.converter_processes = []

        for robot_id in self.spawned_robots:
            try:
                cmd = [
                    'ros2', 'launch', 
                    'obj_gazebo_simulation', 'gazebo_converter.launch.py',
                    f'robot_id:={robot_id}'
                ]

                process = subprocess.Popen(
                    cmd, 
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )

                self.converter_processes.append(process)
                self.get_logger().info(f"Started converter for robot: {robot_id}")

            except Exception as e:
                self.get_logger().error(f"Failed to start converter for robot {robot_id}: {str(e)}")

        self.get_logger().info("All converters started with launch files")
        
    def __del__(self):
        """Clean up processes when the node is destroyed"""
        if hasattr(self, 'converter_processes'):
            for process in self.converter_processes:
                try:
                    process.terminate()
                    process.wait(timeout=5)
                except:
                    pass


def main():
    """ Entry point for the ROS2 node """
    rclpy.init()  # Ensure ROS2 is initialized

    try:
        node = MultiRobotSpawner()  # Create the node
        rclpy.spin(node)  # Keep it running
    except Exception as e:
        print(f"Exception in main: {e}")
    finally:
        rclpy.shutdown()  # Properly shut down ROS2

if __name__ == "__main__":
    main()