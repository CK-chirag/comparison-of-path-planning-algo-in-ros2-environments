"""
Path Visualization Node

ROS2 node for visualizing path planning results with markers in RViz.
Creates colored markers for different algorithms and performance metrics.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
import math

class PathVisualizer(Node):
    """
    ROS2 node for visualizing path planning algorithm results.
    """
    
    def __init__(self):
        """Initialize the path visualizer node."""
        super().__init__('path_visualizer')
        
        # Parameters - avoid conflicts
        self.marker_lifetime = 30.0
        
        # Get use_sim_time parameter if available  
        try:
            self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        except:
            self.use_sim_time = True
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, '/algorithm_markers', 10)
        
        # Subscribe to path topics
        self.dijkstra_sub = self.create_subscription(Path, '/dijkstra_path', self.dijkstra_path_callback, 10)
        self.astar_sub = self.create_subscription(Path, '/astar_path', self.astar_path_callback, 10) 
        self.theta_star_sub = self.create_subscription(Path, '/theta_star_path', self.theta_star_path_callback, 10)
        
        # Timer for periodic updates (if needed)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Marker ID counter
        self.marker_id = 0
        
        self.get_logger().info('Path Visualizer Node initialized')
        
    def timer_callback(self):
        """Periodic timer callback (placeholder for future features)."""
        pass
        
    def dijkstra_path_callback(self, msg):
        """Handle Dijkstra path messages."""
        self.visualize_path(msg, 'dijkstra', [1.0, 0.0, 0.0])  # Red
        
    def astar_path_callback(self, msg):
        """Handle A* path messages."""
        self.visualize_path(msg, 'astar', [0.0, 1.0, 0.0])  # Green
        
    def theta_star_path_callback(self, msg):
        """Handle Theta* path messages."""
        self.visualize_path(msg, 'theta_star', [0.0, 0.0, 1.0])  # Blue
        
    def visualize_path(self, path_msg, algorithm_name, color_rgb):
        """Convert ROS Path message to visualization marker."""
        if not path_msg.poses:
            return
            
        # Extract path points
        path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
        
        # Create color
        color = ColorRGBA()
        color.r, color.g, color.b = color_rgb
        color.a = 1.0
        
        # Create and publish marker
        marker = self.create_path_marker(path_points, color, self.marker_id, algorithm_name)
        self.marker_id += 1
        
        marker_array = MarkerArray()
        marker_array.markers = [marker]
        self.marker_pub.publish(marker_array)
        
        self.get_logger().info(f'Published {algorithm_name} path with {len(path_points)} points')
        
    def create_path_marker(self, path, color, marker_id, namespace):
        """
        Create line strip marker for path visualization.
        
        Args:
            path (list): List of (x, y) world coordinates
            color (ColorRGBA): Marker color
            marker_id (int): Unique marker ID
            namespace (str): Marker namespace
            
        Returns:
            Marker: ROS Marker message
        """
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set marker scale (line width)
        marker.scale.x = 0.05  # Line width
        
        # Set color
        marker.color = color
        
        # Set lifetime
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        
        # Add path points
        for x, y in path:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.1  # Slightly above ground
            marker.points.append(point)
            
        return marker
        
    def create_explored_nodes_marker(self, nodes, color, marker_id, namespace):
        """
        Create sphere markers for explored nodes.
        
        Args:
            nodes (list): List of (x, y) world coordinates of explored nodes
            color (ColorRGBA): Marker color
            marker_id (int): Unique marker ID
            namespace (str): Marker namespace
            
        Returns:
            Marker: ROS Marker message
        """
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        
        # Set marker scale (sphere size)
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        
        # Set color (make it semi-transparent)
        marker.color = color
        marker.color.a = 0.3  # Semi-transparent
        
        # Set lifetime
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        
        # Add node points
        for x, y in nodes:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.05  # Slightly above ground
            marker.points.append(point)
            
        return marker
        
    def create_text_marker(self, text, position, color, marker_id, namespace):
        """
        Create text marker for displaying algorithm metrics.
        
        Args:
            text (str): Text to display
            position (tuple): (x, y) position for text
            color (ColorRGBA): Text color
            marker_id (int): Unique marker ID
            namespace (str): Marker namespace
            
        Returns:
            Marker: ROS Marker message
        """
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.5  # Above other markers
        marker.pose.orientation.w = 1.0
        
        # Set text
        marker.text = text
        
        # Set scale (text size)
        marker.scale.z = 0.1
        
        # Set color
        marker.color = color
        
        # Set lifetime
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        
        return marker
        
    def create_start_goal_markers(self, start_pos, goal_pos):
        """
        Create markers for start and goal positions.
        
        Args:
            start_pos (tuple): (x, y) start position
            goal_pos (tuple): (x, y) goal position
            
        Returns:
            list: List of Marker messages
        """
        markers = []
        
        # Start marker (green sphere)
        start_marker = Marker()
        start_marker.header = Header()
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.header.frame_id = 'map'
        
        start_marker.ns = 'start_goal'
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        
        start_marker.pose.position.x = start_pos[0]
        start_marker.pose.position.y = start_pos[1]
        start_marker.pose.position.z = 0.1
        start_marker.pose.orientation.w = 1.0
        
        start_marker.scale.x = 0.2
        start_marker.scale.y = 0.2
        start_marker.scale.z = 0.2
        
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 1.0
        
        markers.append(start_marker)
        
        # Goal marker (red sphere)
        goal_marker = Marker()
        goal_marker.header = Header()
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.header.frame_id = 'map'
        
        goal_marker.ns = 'start_goal'
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        
        goal_marker.pose.position.x = goal_pos[0]
        goal_marker.pose.position.y = goal_pos[1]
        goal_marker.pose.position.z = 0.1
        goal_marker.pose.orientation.w = 1.0
        
        goal_marker.scale.x = 0.2
        goal_marker.scale.y = 0.2
        goal_marker.scale.z = 0.2
        
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        
        markers.append(goal_marker)
        
        return markers
        
    def get_algorithm_colors(self):
        """
        Get standard colors for each algorithm.
        
        Returns:
            dict: Algorithm name to ColorRGBA mapping
        """
        colors = {}
        
        # Dijkstra - Red
        colors['dijkstra'] = ColorRGBA()
        colors['dijkstra'].r = 1.0
        colors['dijkstra'].g = 0.0
        colors['dijkstra'].b = 0.0
        colors['dijkstra'].a = 1.0
        
        # A* - Green
        colors['astar'] = ColorRGBA()
        colors['astar'].r = 0.0
        colors['astar'].g = 1.0
        colors['astar'].b = 0.0
        colors['astar'].a = 1.0
        
        # Theta* - Blue
        colors['theta_star'] = ColorRGBA()
        colors['theta_star'].r = 0.0
        colors['theta_star'].g = 0.0
        colors['theta_star'].b = 1.0
        colors['theta_star'].a = 1.0
        
        return colors
        
    def publish_algorithm_results(self, results, start_pos, goal_pos):
        """
        Publish visualization markers for algorithm results.
        
        Args:
            results (dict): Algorithm results with paths and metrics
            start_pos (tuple): Start position
            goal_pos (tuple): Goal position
        """
        marker_array = MarkerArray()
        colors = self.get_algorithm_colors()
        
        # Add start/goal markers
        start_goal_markers = self.create_start_goal_markers(start_pos, goal_pos)
        marker_array.markers.extend(start_goal_markers)
        
        # Add path markers for each algorithm
        for i, (algorithm, result) in enumerate(results.items()):
            if result['path'] and algorithm in colors:
                path_marker = self.create_path_marker(
                    result['path'],
                    colors[algorithm],
                    100 + i,  # Unique ID offset
                    f'{algorithm}_path'
                )
                marker_array.markers.append(path_marker)
                
                # Add metrics text
                if 'metrics' in result and result['metrics'].get('success', False):
                    metrics = result['metrics']
                    text = (f"{algorithm.upper()}\n"
                           f"Length: {metrics['path_length']:.2f}m\n"
                           f"Time: {metrics['planning_time']:.3f}s\n"
                           f"Nodes: {metrics['nodes_explored']}")
                    
                    # Position text near goal
                    text_pos = (goal_pos[0] + 0.5, goal_pos[1] + i * 0.3)
                    
                    text_marker = self.create_text_marker(
                        text,
                        text_pos,
                        colors[algorithm],
                        200 + i,  # Unique ID offset
                        f'{algorithm}_metrics'
                    )
                    marker_array.markers.append(text_marker)
        
        # Publish marker array
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(marker_array.markers)} visualization markers')

def main(args=None):
    """Main function for the path visualizer node."""
    rclpy.init(args=args)
    
    try:
        node = PathVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
