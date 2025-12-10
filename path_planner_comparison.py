"""
Path Planner Comparison Node

Main ROS2 node that compares Dijkstra, A*, and Theta* algorithms.
Subscribes to costmap and goal topics, runs all algorithms, and publishes results.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
import json
import time
from datetime import datetime

from .costmap_handler import CostmapHandler
from .algorithms.dijkstra import DijkstraPlanner
from .algorithms.astar import AStarPlanner
from .algorithms.theta_star import ThetaStarPlanner

class PathPlannerComparison(Node):
    """
    ROS2 node for comparing path planning algorithms.
    """
    
    def __init__(self):
        """Initialize the path planner comparison node."""
        super().__init__('path_planner_comparison')
        
        # Initialize costmap handler and algorithms
        self.costmap_handler = CostmapHandler()
        self.dijkstra_planner = DijkstraPlanner(self.costmap_handler)
        self.astar_planner = AStarPlanner(self.costmap_handler)
        self.theta_star_planner = ThetaStarPlanner(self.costmap_handler)
        
        # Current start and goal positions
        self.current_start = None
        self.current_goal = None
        
        # Parameters - use undeclared parameters to avoid conflicts
        self.save_metrics = True
        self.metrics_file = '/tmp/algorithm_comparison.json'
        
        # Get use_sim_time parameter if available
        try:
            self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        except:
            self.use_sim_time = True
        
        # Subscribers - QoS for latched topics
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.costmap_callback,
            qos
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10
        )
        
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )
        
        # Publishers for each algorithm's path
        self.dijkstra_path_pub = self.create_publisher(Path, '/dijkstra_path', 10)
        self.astar_path_pub = self.create_publisher(Path, '/astar_path', 10)
        self.theta_star_path_pub = self.create_publisher(Path, '/theta_star_path', 10)
        
        # Metrics storage
        self.all_metrics = []
        
        self.get_logger().info('Path Planner Comparison Node initialized')
        self.get_logger().info('Waiting for costmap data...')
        
    def costmap_callback(self, msg):
        """
        Handle costmap updates from Nav2.
        
        Args:
            msg (OccupancyGrid): Costmap message
        """
        self.costmap_handler.update_costmap(msg)
        self.get_logger().info(f'Costmap updated: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution}')
        
    def initial_pose_callback(self, msg):
        """
        Handle initial pose from RViz.
        
        Args:
            msg (PoseWithCovarianceStamped): Initial pose message
        """
        self.current_start = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        self.get_logger().info(f'Start position set: {self.current_start}')
        
    def goal_callback(self, msg):
        """
        Handle goal pose from RViz 2D Nav Goal.
        
        Args:
            msg (PoseStamped): Goal pose message
        """
        self.current_goal = (
            msg.pose.position.x,
            msg.pose.position.y
        )
        
        self.get_logger().info(f'Goal received: {self.current_goal}')
        
        # Use current robot position if no start set
        if self.current_start is None:
            # Try to get robot's current position from TF or use origin
            self.current_start = (0.0, 0.0)  # Fallback to origin
            self.get_logger().warn('No start position set, using origin (0,0)')
        
        # Run planning comparison
        self.run_planning_comparison()
        
    def run_planning_comparison(self):
        """Run all three algorithms and compare results."""
        if self.costmap_handler.costmap_data is None:
            self.get_logger().error('No costmap data available')
            return
            
        if self.current_start is None or self.current_goal is None:
            self.get_logger().error('Start or goal position not set')
            return
            
        self.get_logger().info(f'Planning from {self.current_start} to {self.current_goal}')
        
        # Debug: Check grid conversions
        start_grid = self.costmap_handler.world_to_grid(self.current_start[0], self.current_start[1])
        goal_grid = self.costmap_handler.world_to_grid(self.current_goal[0], self.current_goal[1])
        self.get_logger().info(f'Start grid: {start_grid}, Goal grid: {goal_grid}')
        self.get_logger().info(f'Map size: {self.costmap_handler.width}x{self.costmap_handler.height}')
        self.get_logger().info(f'Map origin: ({self.costmap_handler.origin_x}, {self.costmap_handler.origin_y})')
        
        # Check if positions are valid
        start_valid = self.costmap_handler.is_valid_cell(start_grid[0], start_grid[1])
        goal_valid = self.costmap_handler.is_valid_cell(goal_grid[0], goal_grid[1])
        start_obstacle = self.costmap_handler.is_obstacle(start_grid[0], start_grid[1])
        goal_obstacle = self.costmap_handler.is_obstacle(goal_grid[0], goal_grid[1])
        
        self.get_logger().info(f'Start - Valid: {start_valid}, Obstacle: {start_obstacle}')
        self.get_logger().info(f'Goal - Valid: {goal_valid}, Obstacle: {goal_obstacle}')
        
        # Run all algorithms
        algorithms = {
            'dijkstra': self.dijkstra_planner,
            'astar': self.astar_planner,
            'theta_star': self.theta_star_planner
        }
        
        results = {}
        
        for name, planner in algorithms.items():
            self.get_logger().info(f'Running {name} algorithm...')
            
            try:
                path, metrics = planner.plan(self.current_start, self.current_goal)
                results[name] = {'path': path, 'metrics': metrics}
                
                if metrics.get('success', False):
                    self.get_logger().info(
                        f'{name}: Success! Path length: {metrics["path_length"]:.2f}m, '
                        f'Time: {metrics["planning_time"]:.3f}s, '
                        f'Nodes: {metrics["nodes_explored"]}'
                    )
                else:
                    self.get_logger().warn(f'{name}: Failed - {metrics.get("error", "Unknown error")}')
                    
            except Exception as e:
                self.get_logger().error(f'Error running {name}: {str(e)}')
                results[name] = {'path': [], 'metrics': {'success': False, 'error': str(e)}}
        
        # Publish paths
        self.publish_paths(results)
        
        # Save metrics
        self.save_comparison_metrics(results)
        
        # Print comparison summary
        self.print_comparison_summary(results)
        
    def publish_paths(self, results):
        """
        Publish paths for visualization in RViz.
        
        Args:
            results (dict): Results from all algorithms
        """
        current_time = self.get_clock().now()
        
        # Publishers mapping
        publishers = {
            'dijkstra': self.dijkstra_path_pub,
            'astar': self.astar_path_pub,
            'theta_star': self.theta_star_path_pub
        }
        
        for algorithm, pub in publishers.items():
            if algorithm in results and results[algorithm]['path']:
                path_msg = self.create_path_message(results[algorithm]['path'], current_time)
                pub.publish(path_msg)
                
    def create_path_message(self, path_world, timestamp):
        """
        Create ROS Path message from world coordinates.
        
        Args:
            path_world (list): Path as list of (x, y) world coordinates
            timestamp: ROS timestamp
            
        Returns:
            Path: ROS Path message
        """
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = timestamp.to_msg()
        path_msg.header.frame_id = 'map'
        
        for x, y in path_world:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
            
        return path_msg
        
    def save_comparison_metrics(self, results):
        """
        Save comparison metrics to file.
        
        Args:
            results (dict): Results from all algorithms
        """
        if not self.save_metrics:
            return
            
        # Create comparison entry
        comparison_data = {
            'timestamp': datetime.now().isoformat(),
            'start_position': self.current_start,
            'goal_position': self.current_goal,
            'costmap_info': self.costmap_handler.get_costmap_info(),
            'algorithms': {}
        }
        
        for algorithm, result in results.items():
            comparison_data['algorithms'][algorithm] = result['metrics']
            
        self.all_metrics.append(comparison_data)
        
        # Save to file
        try:
            with open(self.metrics_file, 'w') as f:
                json.dump(self.all_metrics, f, indent=2)
            self.get_logger().info(f'Metrics saved to {self.metrics_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save metrics: {str(e)}')
            
    def print_comparison_summary(self, results):
        """
        Print comparison summary to console.
        
        Args:
            results (dict): Results from all algorithms
        """
        self.get_logger().info('=== ALGORITHM COMPARISON SUMMARY ===')
        
        successful_results = {k: v for k, v in results.items() 
                            if v['metrics'].get('success', False)}
        
        if not successful_results:
            self.get_logger().warn('No algorithms found valid paths')
            return
            
        # Find best performers
        fastest = min(successful_results.items(), 
                     key=lambda x: x[1]['metrics']['planning_time'])
        shortest = min(successful_results.items(),
                      key=lambda x: x[1]['metrics']['path_length'])
        most_efficient = min(successful_results.items(),
                           key=lambda x: x[1]['metrics']['nodes_explored'])
        
        self.get_logger().info(f'Fastest planning: {fastest[0]} ({fastest[1]["metrics"]["planning_time"]:.3f}s)')
        self.get_logger().info(f'Shortest path: {shortest[0]} ({shortest[1]["metrics"]["path_length"]:.2f}m)')
        self.get_logger().info(f'Most efficient: {most_efficient[0]} ({most_efficient[1]["metrics"]["nodes_explored"]} nodes)')
        self.get_logger().info('=====================================')

def main(args=None):
    """Main function for the path planner comparison node."""
    rclpy.init(args=args)
    
    try:
        node = PathPlannerComparison()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
