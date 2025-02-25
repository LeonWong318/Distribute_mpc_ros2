import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from msg_interfaces.msg import RobotStatesQuery, RobotState
from geometry_msgs.msg import Point, Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

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
        
        self.get_logger().info('Robot state visualizer initialized')
    
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
                
                # add predict points
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