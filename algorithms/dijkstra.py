"""
Dijkstra's Algorithm Implementation

Implements Dijkstra's shortest path algorithm for grid-based path planning.
Guarantees optimal paths by exploring all directions equally.
"""

import heapq
import numpy as np
from typing import List, Tuple, Optional, Dict
import time
import math

class DijkstraPlanner:
    """
    Dijkstra's algorithm implementation for path planning on costmaps.
    """
    
    def __init__(self, costmap_handler):
        """
        Initialize Dijkstra planner.
        
        Args:
            costmap_handler: CostmapHandler instance
        """
        self.costmap_handler = costmap_handler
        self.metrics = {}
        
    def plan(self, start_world, goal_world):
        """
        Plan path from start to goal using Dijkstra's algorithm.
        
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
        open_set = []  # Priority queue: (cost, (x, y))
        closed_set = set()  # Explored nodes
        g_score = {}  # Cost from start to each node
        came_from = {}  # Parent tracking for path reconstruction
        
        # Initialize start node
        heapq.heappush(open_set, (0.0, start_grid))
        g_score[start_grid] = 0.0
        nodes_explored = 0
        
        # Main Dijkstra loop
        while open_set:
            # Get node with lowest cost
            current_cost, current = heapq.heappop(open_set)
            
            # Skip if already processed (can happen due to duplicate entries)
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
                    'algorithm': 'dijkstra',
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
                    
                # Total cost = current cost + movement cost + terrain cost
                tentative_g = g_score[current] + move_cost + terrain_cost
                
                # Update if this is a better path to neighbor
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    heapq.heappush(open_set, (tentative_g, neighbor))
        
        # No path found
        planning_time = time.time() - start_time
        self.metrics = {
            'algorithm': 'dijkstra',
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
