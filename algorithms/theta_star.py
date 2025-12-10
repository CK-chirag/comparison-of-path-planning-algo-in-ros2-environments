"""
Theta* Algorithm Implementation

Implements Theta* any-angle path planning algorithm.
Produces smoother paths by allowing parent pointers to skip nodes
when line-of-sight exists, avoiding grid-constrained paths.
"""

import heapq
import numpy as np
from typing import List, Tuple, Optional, Dict
import time
import math

class ThetaStarPlanner:
    """
    Theta* algorithm implementation for any-angle path planning on costmaps.
    """
    
    def __init__(self, costmap_handler):
        """
        Initialize Theta* planner.
        
        Args:
            costmap_handler: CostmapHandler instance
        """
        self.costmap_handler = costmap_handler
        self.metrics = {}
        
    def plan(self, start_world, goal_world):
        """
        Plan path from start to goal using Theta* algorithm.
        
        Args:
            start_world (tuple): (x, y) start position in world coordinates
            goal_world (tuple): (x, y) goal position in world coordinates
            
        Returns:
            tuple: (path, metrics) where path is list of (x, y) world coordinates
        """
        start_time = time.time()
        
        # Convert to grid coordinates
        start_grid = self.costmap_handler.world_to_grid(*start_world)
        goal_grid = self.costmap_handler.world_to_grid(*goal_world)
        
        # Validate start and goal
        if not self.costmap_handler.is_valid_cell(*start_grid):
            return [], {'error': 'Invalid start position'}
            
        if not self.costmap_handler.is_valid_cell(*goal_grid):
            return [], {'error': 'Invalid goal position'}
            
        if self.costmap_handler.is_obstacle(*start_grid):
            return [], {'error': 'Start position is obstacle'}
            
        if self.costmap_handler.is_obstacle(*goal_grid):
            return [], {'error': 'Goal position is obstacle'}
        
        # Initialize data structures
        open_set = []  # Priority queue: (f_score, (x, y))
        closed_set = set()  # Explored nodes
        g_score = {}  # Cost from start to each node
        f_score = {}  # g_score + heuristic
        came_from = {}  # Parent tracking for path reconstruction
        
        # Initialize start node
        start_h = self.heuristic(*start_grid, *goal_grid)
        heapq.heappush(open_set, (start_h, start_grid))
        g_score[start_grid] = 0.0
        f_score[start_grid] = start_h
        came_from[start_grid] = None
        nodes_explored = 0
        
        # Main Theta* loop
        while open_set:
            # Get node with lowest f_score
            current_f, current = heapq.heappop(open_set)
            
            # Skip if already processed
            if current in closed_set:
                continue
                
            # Mark as explored
            closed_set.add(current)
            nodes_explored += 1
            
            # Check if goal reached
            if current == goal_grid:
                planning_time = time.time() - start_time
                
                # Reconstruct path
                path_grid = self.reconstruct_path(came_from, start_grid, goal_grid)
                path_world = [self.costmap_handler.grid_to_world(x, y) for x, y in path_grid]
                
                # Calculate metrics
                path_length = self.calculate_path_length(path_world)
                
                self.metrics = {
                    'algorithm': 'theta_star',
                    'planning_time': planning_time,
                    'path_length': path_length,
                    'path_points': len(path_world),
                    'nodes_explored': nodes_explored,
                    'success': True,
                    'start_grid': start_grid,
                    'goal_grid': goal_grid,
                    'start_world': start_world,
                    'goal_world': goal_world
                }
                
                return path_world, self.metrics
            
            # Explore neighbors
            for neighbor_x, neighbor_y, move_cost in self.costmap_handler.get_neighbors_8_connected(*current):
                neighbor = (neighbor_x, neighbor_y)
                
                # Skip if already explored
                if neighbor in closed_set:
                    continue
                
                # Calculate cost to reach neighbor
                terrain_cost = self.costmap_handler.get_cost(neighbor_x, neighbor_y)
                if terrain_cost == float('inf'):
                    continue
                
                # Theta* modification: Check line-of-sight to parent
                self.update_vertex(current, neighbor, goal_grid, g_score, f_score, came_from, open_set)
        
        # No path found
        planning_time = time.time() - start_time
        self.metrics = {
            'algorithm': 'theta_star',
            'planning_time': planning_time,
            'path_length': 0.0,
            'path_points': 0,
            'nodes_explored': nodes_explored,
            'success': False,
            'start_grid': start_grid,
            'goal_grid': goal_grid,
            'start_world': start_world,
            'goal_world': goal_world,
            'error': 'No path found'
        }
        
        return [], self.metrics
    
    def update_vertex(self, current, neighbor, goal, g_score, f_score, came_from, open_set):
        """
        Theta* specific vertex update with line-of-sight optimization.
        
        Args:
            current (tuple): Current node position
            neighbor (tuple): Neighbor node position
            goal (tuple): Goal position
            g_score (dict): Cost from start to each node
            f_score (dict): Total cost estimate for each node
            came_from (dict): Parent pointers
            open_set (list): Priority queue
        """
        # Get parent of current node
        parent = came_from.get(current)
        
        if parent is not None and self.line_of_sight(*parent, *neighbor):
            # Path 2: Direct connection from parent to neighbor
            terrain_cost = self.costmap_handler.get_cost(*neighbor)
            if terrain_cost == float('inf'):
                return
                
            distance = self.costmap_handler.distance_euclidean(*parent, *neighbor)
            tentative_g = g_score[parent] + distance + terrain_cost
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = parent
                g_score[neighbor] = tentative_g
                
                h_cost = self.heuristic(*neighbor, *goal)
                f_cost = tentative_g + h_cost
                f_score[neighbor] = f_cost
                
                heapq.heappush(open_set, (f_cost, neighbor))
        else:
            # Path 1: Standard A* connection from current to neighbor
            terrain_cost = self.costmap_handler.get_cost(*neighbor)
            if terrain_cost == float('inf'):
                return
                
            distance = self.costmap_handler.distance_euclidean(*current, *neighbor)
            tentative_g = g_score[current] + distance + terrain_cost
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                
                h_cost = self.heuristic(*neighbor, *goal)
                f_cost = tentative_g + h_cost
                f_score[neighbor] = f_cost
                
                heapq.heappush(open_set, (f_cost, neighbor))
    
    def line_of_sight(self, x1, y1, x2, y2):
        """
        Check if there's a clear line-of-sight between two grid points.
        Uses Bresenham's line algorithm to check for obstacles.
        
        Args:
            x1, y1 (int): First point grid coordinates
            x2, y2 (int): Second point grid coordinates
            
        Returns:
            bool: True if line-of-sight exists (no obstacles)
        """
        # Bresenham's line algorithm
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        
        # Direction of line
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        
        err = dx - dy
        x, y = x1, y1
        
        while True:
            # Check current cell for obstacle
            if self.costmap_handler.is_obstacle(x, y):
                return False
                
            # Reached end point
            if x == x2 and y == y2:
                break
                
            # Move to next cell
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
                
        return True
    
    def heuristic(self, x1, y1, x2, y2):
        """
        Heuristic function for Theta* (Euclidean distance).
        
        Args:
            x1, y1 (int): Current grid position
            x2, y2 (int): Goal grid position
            
        Returns:
            float: Heuristic cost estimate
        """
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def reconstruct_path(self, came_from, start, goal):
        """
        Reconstruct path from goal to start using parent pointers.
        
        Args:
            came_from (dict): Parent pointers
            start (tuple): Start grid position
            goal (tuple): Goal grid position
            
        Returns:
            list: Path as list of (x, y) grid coordinates
        """
        path = []
        current = goal
        
        while current is not None:
            path.append(current)
            current = came_from.get(current)
            
        path.reverse()  # Reverse to get start-to-goal path
        return path
    
    def calculate_path_length(self, path_world):
        """
        Calculate total path length in world coordinates.
        
        Args:
            path_world (list): Path as list of (x, y) world coordinates
            
        Returns:
            float: Total path length
        """
        if len(path_world) < 2:
            return 0.0
            
        length = 0.0
        for i in range(len(path_world) - 1):
            x1, y1 = path_world[i]
            x2, y2 = path_world[i + 1]
            length += self.costmap_handler.distance_euclidean(x1, y1, x2, y2)
            
        return length
    
    def get_explored_nodes(self, closed_set):
        """
        Get explored nodes for visualization.
        
        Args:
            closed_set (set): Set of explored grid positions
            
        Returns:
            list: List of (x, y) world coordinates of explored nodes
        """
        explored_world = []
        for grid_pos in closed_set:
            world_pos = self.costmap_handler.grid_to_world(*grid_pos)
            explored_world.append(world_pos)
        return explored_world
