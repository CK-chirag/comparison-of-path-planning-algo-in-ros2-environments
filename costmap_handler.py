"""
Costmap Handler for ROS2 Navigation

Handles costmap data from Nav2 and provides utility functions
for coordinate transformations and cost queries.
"""

import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
import math

class CostmapHandler:
    """
    Handles costmap data and provides utility functions for path planning algorithms.
    """
    
    def __init__(self):
        """Initialize costmap handler."""
        self.costmap_data = None
        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.frame_id = "map"
        
    def update_costmap(self, occupancy_grid_msg):
        """
        Update costmap from Nav2 OccupancyGrid message.
        
        Args:
            occupancy_grid_msg (OccupancyGrid): ROS2 occupancy grid message
        """
        # Extract metadata
        self.width = occupancy_grid_msg.info.width
        self.height = occupancy_grid_msg.info.height
        self.resolution = occupancy_grid_msg.info.resolution
        self.origin_x = occupancy_grid_msg.info.origin.position.x
        self.origin_y = occupancy_grid_msg.info.origin.position.y
        self.frame_id = occupancy_grid_msg.header.frame_id
        
        # Convert 1D array to 2D grid (row-major order)
        self.costmap_data = np.array(occupancy_grid_msg.data, dtype=np.int8).reshape((self.height, self.width))
        
    def world_to_grid(self, world_x, world_y):
        """
        Convert world coordinates to grid indices.
        
        Args:
            world_x (float): X coordinate in world frame
            world_y (float): Y coordinate in world frame
            
        Returns:
            tuple: (grid_x, grid_y) Grid indices
        """
        grid_x = int((world_x - self.origin_x) / self.resolution)
        grid_y = int((world_y - self.origin_y) / self.resolution)
        return grid_x, grid_y
        
    def grid_to_world(self, grid_x, grid_y):
        """
        Convert grid indices to world coordinates.
        
        Args:
            grid_x (int): Grid x index
            grid_y (int): Grid y index
            
        Returns:
            tuple: (world_x, world_y) World coordinates
        """
        world_x = grid_x * self.resolution + self.origin_x + self.resolution / 2.0
        world_y = grid_y * self.resolution + self.origin_y + self.resolution / 2.0
        return world_x, world_y
        
    def get_cost(self, grid_x, grid_y):
        """
        Get cost at grid position with bounds checking.
        
        Args:
            grid_x (int): Grid x index
            grid_y (int): Grid y index
            
        Returns:
            float: Cost value (inf for obstacles/unknown/out-of-bounds)
        """
        if not self.is_valid_cell(grid_x, grid_y):
            return float('inf')
            
        cost = self.costmap_data[grid_y, grid_x]  # Note: y first for numpy array
        
        # Handle Nav2 cost values:
        # -1: Unknown space
        # 0: Free space  
        # 1-99: Low to high cost
        # 100+: Obstacle
        
        if cost == -1:  # Unknown
            return float('inf')
        elif cost >= 100:  # Obstacle
            return float('inf')
        else:
            return float(cost)
            
    def is_valid_cell(self, grid_x, grid_y):
        """
        Check if cell is within bounds and has costmap data.
        
        Args:
            grid_x (int): Grid x index
            grid_y (int): Grid y index
            
        Returns:
            bool: True if cell is valid
        """
        if self.costmap_data is None:
            return False
            
        return (0 <= grid_x < self.width and 0 <= grid_y < self.height)
        
    def is_obstacle(self, grid_x, grid_y):
        """
        Check if cell contains an obstacle.
        
        Args:
            grid_x (int): Grid x index
            grid_y (int): Grid y index
            
        Returns:
            bool: True if cell is an obstacle
        """
        if not self.is_valid_cell(grid_x, grid_y):
            return True
            
        cost = self.costmap_data[grid_y, grid_x]
        return cost >= 100 or cost == -1
        
    def get_neighbors_8_connected(self, grid_x, grid_y):
        """
        Get 8-connected neighbors with movement costs.
        
        Args:
            grid_x (int): Grid x index
            grid_y (int): Grid y index
            
        Returns:
            list: List of (neighbor_x, neighbor_y, movement_cost) tuples
        """
        neighbors = []
        
        # 8-connected movement: 4 cardinal + 4 diagonal
        movements = [
            (-1, -1, math.sqrt(2)),  # Diagonal
            (-1,  0, 1.0),           # Left
            (-1,  1, math.sqrt(2)),  # Diagonal
            ( 0, -1, 1.0),           # Down
            ( 0,  1, 1.0),           # Up
            ( 1, -1, math.sqrt(2)),  # Diagonal
            ( 1,  0, 1.0),           # Right
            ( 1,  1, math.sqrt(2))   # Diagonal
        ]
        
        for dx, dy, cost in movements:
            nx, ny = grid_x + dx, grid_y + dy
            if self.is_valid_cell(nx, ny) and not self.is_obstacle(nx, ny):
                neighbors.append((nx, ny, cost))
                
        return neighbors
        
    def distance_euclidean(self, x1, y1, x2, y2):
        """
        Calculate Euclidean distance between two points.
        
        Args:
            x1, y1 (float): First point coordinates
            x2, y2 (float): Second point coordinates
            
        Returns:
            float: Euclidean distance
        """
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
    def get_costmap_info(self):
        """
        Get costmap metadata information.
        
        Returns:
            dict: Costmap information
        """
        return {
            'width': self.width,
            'height': self.height,
            'resolution': self.resolution,
            'origin_x': self.origin_x,
            'origin_y': self.origin_y,
            'frame_id': self.frame_id,
            'has_data': self.costmap_data is not None
        }
