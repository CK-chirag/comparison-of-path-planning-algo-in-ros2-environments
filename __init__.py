"""
All Algorithms Package for ROS2 Navigation

This package implements and compares path planning algorithms:
- Dijkstra's Algorithm
- A* Algorithm  
- Theta* Algorithm

For use with Nav2 costmaps and ROS2 navigation stack.
"""

__version__ = '0.0.1'
__author__ = 'Chirag'
__email__ = 'chiragkhanna@example.com'

# Import main classes for easy access
from .costmap_handler import CostmapHandler
from .path_planner_comparison import PathPlannerComparison
from .visualizer import PathVisualizer

# Import algorithms
from .algorithms.dijkstra import DijkstraPlanner
from .algorithms.astar import AStarPlanner
from .algorithms.theta_star import ThetaStarPlanner

__all__ = [
    'CostmapHandler',
    'PathPlannerComparison', 
    'PathVisualizer',
    'DijkstraPlanner',
    'AStarPlanner',
    'ThetaStarPlanner'
]
