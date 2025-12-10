"""
Path Planning Algorithms Module

Contains implementations of various path planning algorithms:
- Dijkstra's Algorithm: Optimal shortest path algorithm
- A* Algorithm: Heuristic-guided optimal search
- Theta* Algorithm: Any-angle path planning with line-of-sight optimization
"""

from .dijkstra import DijkstraPlanner
from .astar import AStarPlanner
from .theta_star import ThetaStarPlanner

__all__ = [
    'DijkstraPlanner',
    'AStarPlanner', 
    'ThetaStarPlanner'
]

# Algorithm metadata for comparison
ALGORITHM_INFO = {
    'dijkstra': {
        'name': 'Dijkstra\'s Algorithm',
        'optimal': True,
        'heuristic': False,
        'any_angle': False,
        'description': 'Guarantees shortest path by exploring all directions equally'
    },
    'astar': {
        'name': 'A* Algorithm',
        'optimal': True,
        'heuristic': True,
        'any_angle': False,
        'description': 'Uses heuristic to guide search toward goal for faster planning'
    },
    'theta_star': {
        'name': 'Theta* Algorithm',
        'optimal': False,
        'heuristic': True,
        'any_angle': True,
        'description': 'Produces smoother any-angle paths using line-of-sight optimization'
    }
}
