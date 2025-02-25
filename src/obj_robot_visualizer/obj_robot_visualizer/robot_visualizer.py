import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from msg_interfaces.msg import RobotStatesQuery, RobotState
from geometry_msgs.msg import Point
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import json
import os
from ament_index_python.packages import get_package_share_directory

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = [0, 0, 0, 0]
    q[0] = cj*sc - sj*cs  # x
    q[1] = cj*ss + sj*cc  # y
    q[2] = cj*cs - sj*sc  # z
    q[3] = cj*cc + sj*ss  # w

    return q

class RobotStateVisualizer(Node):
    def __init__(self):
        super().__init__('robot_state_visualizer')
        
        self.declare_parameter('map_path', 'data/test_data/map.json')
        self.declare_parameter('graph_path', 'data/test_data/graph.json')
        self.declare_parameter('robot_start_path', 'data/test_data/robot_start.json')
        
        self.map_path = self.get_parameter('map_path').value
        self.graph_path = self.get_parameter('graph_path').value
        self.robot_start_path = self.get_parameter('robot_start_path').value
        
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/robot_visualization/markers',
            10
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.subscription = self.create_subscription(
            RobotStatesQuery,
            '/manager/robot_states',
            self.robot_states_callback,
            10
        )
        
        self.static_timer = self.create_timer(1.0, self.publish_static_markers)
        
        self.map_data = None
        self.graph_data = None
        self.robot_start_data = None
        self.load_data()
        
        self.get_logger().info('Robot state visualizer initialized')
    
    def load_data(self):
        try:
            package_share_dir = get_package_share_directory('obj_robot_visualizer')
            
            try:
                map_file_path = os.path.join(package_share_dir, self.map_path)
                with open(map_file_path, 'r') as f:
                    self.map_data = json.load(f)
                self.get_logger().info(f'Loaded map data from {map_file_path}')
            except Exception as e:
                self.get_logger().error(f'Error loading map data: {str(e)}')
                try:
                    with open(self.map_path, 'r') as f:
                        self.map_data = json.load(f)
                    self.get_logger().info(f'Loaded map data from {self.map_path}')
                except Exception as e2:
                    self.get_logger().error(f'Error loading map data from relative path: {str(e2)}')
            
            try:
                graph_file_path = os.path.join(package_share_dir, self.graph_path)
                with open(graph_file_path, 'r') as f:
                    self.graph_data = json.load(f)
                self.get_logger().info(f'Loaded graph data from {graph_file_path}')
            except Exception as e:
                self.get_logger().error(f'Error loading graph data: {str(e)}')
                try:
                    with open(self.graph_path, 'r') as f:
                        self.graph_data = json.load(f)
                    self.get_logger().info(f'Loaded graph data from {self.graph_path}')
                except Exception as e2:
                    self.get_logger().error(f'Error loading graph data from relative path: {str(e2)}')
            
            try:
                robot_start_file_path = os.path.join(package_share_dir, self.robot_start_path)
                with open(robot_start_file_path, 'r') as f:
                    self.robot_start_data = json.load(f)
                self.get_logger().info(f'Loaded robot start data from {robot_start_file_path}')
            except Exception as e:
                self.get_logger().error(f'Error loading robot start data: {str(e)}')
                try:
                    with open(self.robot_start_path, 'r') as f:
                        self.robot_start_data = json.load(f)
                    self.get_logger().info(f'Loaded robot start data from {self.robot_start_path}')
                except Exception as e2:
                    self.get_logger().error(f'Error loading robot start data from relative path: {str(e2)}')
                    
        except Exception as e:
            self.get_logger().error(f'General error loading data: {str(e)}')
    
    def publish_static_markers(self):
        marker_array = MarkerArray()

        if self.map_data:
            if "boundary_coords" in self.map_data:
                boundary_marker = Marker()
                boundary_marker.header.frame_id = "map"
                boundary_marker.header.stamp = self.get_clock().now().to_msg()
                boundary_marker.ns = "boundary"
                boundary_marker.id = 0
                boundary_marker.type = Marker.LINE_STRIP
                boundary_marker.action = Marker.ADD
                boundary_marker.scale.x = 0.1
                boundary_marker.color.r = 0.0
                boundary_marker.color.g = 0.0
                boundary_marker.color.b = 0.0
                boundary_marker.color.a = 1.0

                for coord in self.map_data["boundary_coords"]:
                    p = Point()
                    p.x = float(coord[0])
                    p.y = float(coord[1])
                    p.z = 0.0
                    boundary_marker.points.append(p)

                if len(self.map_data["boundary_coords"]) > 0:
                    p = Point()
                    p.x = float(self.map_data["boundary_coords"][0][0])
                    p.y = float(self.map_data["boundary_coords"][0][1])
                    p.z = 0.0
                    boundary_marker.points.append(p)

                marker_array.markers.append(boundary_marker)

            if "obstacle_list" in self.map_data:
                for i, obstacle in enumerate(self.map_data["obstacle_list"]):
                    obstacle_marker = Marker()
                    obstacle_marker.header.frame_id = "map"
                    obstacle_marker.header.stamp = self.get_clock().now().to_msg()
                    obstacle_marker.ns = "obstacles"
                    obstacle_marker.id = i
                    obstacle_marker.type = Marker.LINE_STRIP
                    obstacle_marker.action = Marker.ADD
                    obstacle_marker.scale.x = 0.1
                    obstacle_marker.color.r = 1.0
                    obstacle_marker.color.g = 0.0
                    obstacle_marker.color.b = 0.0
                    obstacle_marker.color.a = 1.0

                    for coord in obstacle:
                        p = Point()
                        p.x = float(coord[0])
                        p.y = float(coord[1])
                        p.z = 0.0
                        obstacle_marker.points.append(p)

                    if len(obstacle) > 0:
                        p = Point()
                        p.x = float(obstacle[0][0])
                        p.y = float(obstacle[0][1])
                        p.z = 0.0
                        obstacle_marker.points.append(p)

                    marker_array.markers.append(obstacle_marker)

                    fill_marker = Marker()
                    fill_marker.header.frame_id = "map"
                    fill_marker.header.stamp = self.get_clock().now().to_msg()
                    fill_marker.ns = "obstacle_fills"
                    fill_marker.id = i
                    fill_marker.type = Marker.TRIANGLE_LIST
                    fill_marker.action = Marker.ADD
                    fill_marker.scale.x = 1.0
                    fill_marker.scale.y = 1.0
                    fill_marker.scale.z = 1.0
                    fill_marker.color.r = 1.0
                    fill_marker.color.g = 0.0
                    fill_marker.color.b = 0.0
                    fill_marker.color.a = 0.3

                    if len(obstacle) >= 3:
                        center_x = float(sum(float(p[0]) for p in obstacle) / len(obstacle))
                        center_y = float(sum(float(p[1]) for p in obstacle) / len(obstacle))

                        for j in range(len(obstacle)):
                            center = Point()
                            center.x = center_x
                            center.y = center_y
                            center.z = 0.0

                            p1 = Point()
                            p1.x = float(obstacle[j][0])
                            p1.y = float(obstacle[j][1])
                            p1.z = 0.0

                            p2 = Point()
                            p2.x = float(obstacle[(j+1) % len(obstacle)][0])
                            p2.y = float(obstacle[(j+1) % len(obstacle)][1])
                            p2.z = 0.0

                            fill_marker.points.append(center)
                            fill_marker.points.append(p1)
                            fill_marker.points.append(p2)

                    marker_array.markers.append(fill_marker)

        if self.graph_data:
            if "node_dict" in self.graph_data:
                for i, (node_id, coords) in enumerate(self.graph_data["node_dict"].items()):
                    node_marker = Marker()
                    node_marker.header.frame_id = "map"
                    node_marker.header.stamp = self.get_clock().now().to_msg()
                    node_marker.ns = "graph_nodes"
                    node_marker.id = i
                    node_marker.type = Marker.SPHERE
                    node_marker.action = Marker.ADD
                    node_marker.pose.position.x = float(coords[0])
                    node_marker.pose.position.y = float(coords[1])
                    node_marker.pose.position.z = 0.1
                    node_marker.scale.x = 0.3
                    node_marker.scale.y = 0.3
                    node_marker.scale.z = 0.3
                    node_marker.color.r = 0.0
                    node_marker.color.g = 0.0
                    node_marker.color.b = 1.0
                    node_marker.color.a = 1.0
                    marker_array.markers.append(node_marker)

                    text_marker = Marker()
                    text_marker.header.frame_id = "map"
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.ns = "graph_node_labels"
                    text_marker.id = i
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    text_marker.pose.position.x = float(coords[0])
                    text_marker.pose.position.y = float(coords[1])
                    text_marker.pose.position.z = 0.4
                    text_marker.text = node_id
                    text_marker.scale.z = 0.2 
                    text_marker.color.r = 1.0
                    text_marker.color.g = 1.0
                    text_marker.color.b = 1.0
                    text_marker.color.a = 1.0
                    marker_array.markers.append(text_marker)

            if "edge_list" in self.graph_data and "node_dict" in self.graph_data:
                for i, edge in enumerate(self.graph_data["edge_list"]):
                    if len(edge) >= 2:
                        node1 = edge[0]
                        node2 = edge[1]

                        if node1 in self.graph_data["node_dict"] and node2 in self.graph_data["node_dict"]:
                            edge_marker = Marker()
                            edge_marker.header.frame_id = "map"
                            edge_marker.header.stamp = self.get_clock().now().to_msg()
                            edge_marker.ns = "graph_edges"
                            edge_marker.id = i
                            edge_marker.type = Marker.LINE_STRIP
                            edge_marker.action = Marker.ADD
                            edge_marker.scale.x = 0.05
                            edge_marker.color.r = 0.0
                            edge_marker.color.g = 0.5
                            edge_marker.color.b = 0.5
                            edge_marker.color.a = 1.0

                            p1 = Point()
                            p1.x = float(self.graph_data["node_dict"][node1][0])
                            p1.y = float(self.graph_data["node_dict"][node1][1])
                            p1.z = 0.05
                            edge_marker.points.append(p1)

                            p2 = Point()
                            p2.x = float(self.graph_data["node_dict"][node2][0])
                            p2.y = float(self.graph_data["node_dict"][node2][1])
                            p2.z = 0.05
                            edge_marker.points.append(p2)

                            marker_array.markers.append(edge_marker)

        if marker_array.markers:
            self.marker_publisher.publish(marker_array)
    
    def robot_states_callback(self, msg):
        marker_array = MarkerArray()
        
        for robot_state in msg.robot_states:
            robot_marker = Marker()
            robot_marker.header.frame_id = "map"
            robot_marker.header.stamp = msg.stamp
            robot_marker.ns = "robots"
            robot_marker.id = robot_state.robot_id
            robot_marker.type = Marker.ARROW
            robot_marker.action = Marker.ADD
            
            robot_marker.pose.position.x = robot_state.x
            robot_marker.pose.position.y = robot_state.y
            robot_marker.pose.position.z = 0.1
            
            q = quaternion_from_euler(0, 0, robot_state.theta)
            robot_marker.pose.orientation.x = q[0]
            robot_marker.pose.orientation.y = q[1]
            robot_marker.pose.orientation.z = q[2]
            robot_marker.pose.orientation.w = q[3]
            
            robot_marker.scale.x = 0.5
            robot_marker.scale.y = 0.2
            robot_marker.scale.z = 0.1
            
            robot_marker.color.a = 1.0
            if robot_state.idle:
                robot_marker.color.r = 0.0
                robot_marker.color.g = 0.0
                robot_marker.color.b = 1.0
            else:
                robot_marker.color.r = 0.0
                robot_marker.color.g = 1.0
                robot_marker.color.b = 0.0
            
            marker_array.markers.append(robot_marker)
            
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = msg.stamp
            text_marker.ns = "robot_ids"
            text_marker.id = robot_state.robot_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = robot_state.x
            text_marker.pose.position.y = robot_state.y
            text_marker.pose.position.z = 0.3
            text_marker.text = f"Robot {robot_state.robot_id}"
            text_marker.scale.z = 0.2
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            marker_array.markers.append(text_marker)
            
            if len(robot_state.pred_states) > 0:
                trajectory_marker = Marker()
                trajectory_marker.header.frame_id = "map"
                trajectory_marker.header.stamp = msg.stamp
                trajectory_marker.ns = "trajectories"
                trajectory_marker.id = robot_state.robot_id
                trajectory_marker.type = Marker.LINE_STRIP
                trajectory_marker.action = Marker.ADD
                
                trajectory_marker.scale.x = 0.05 
                trajectory_marker.color.a = 0.7 
                trajectory_marker.color.r = 1.0
                trajectory_marker.color.g = 0.5
                trajectory_marker.color.b = 0.0
                
                start_point = Point()
                start_point.x = robot_state.x
                start_point.y = robot_state.y
                start_point.z = 0.05
                trajectory_marker.points.append(start_point)
                

                for i in range(0, len(robot_state.pred_states), 3):
                    if i + 1 < len(robot_state.pred_states):
                        p = Point()
                        p.x = robot_state.pred_states[i]
                        p.y = robot_state.pred_states[i + 1]
                        p.z = 0.05
                        trajectory_marker.points.append(p)
                
                marker_array.markers.append(trajectory_marker)
            
            transform = TransformStamped()
            transform.header.frame_id = "map"
            transform.header.stamp = msg.stamp
            transform.child_frame_id = f"robot_{robot_state.robot_id}"
            
            transform.transform.translation.x = robot_state.x
            transform.transform.translation.y = robot_state.y
            transform.transform.translation.z = 0.0
            
            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]
            
            self.tf_broadcaster.sendTransform(transform)
        
        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    visualizer = RobotStateVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()