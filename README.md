# Comparative Analysis of Grid-Based and Any-Angle Pathfinding Algorithms in ROS2 Nav2

This project presents a detailed performance evaluation of three global path planning algorithms in the ROS2 Navigation Framework (Nav2):  
**Dijkstra**, **A***, and **Theta***.  
The research benchmarks computation speed, path optimality, and waypoint smoothness in a unified Gazebo-SLAM simulation.

---

## Key Research Objectives

- Implement comparison of Dijkstra, A*, and Theta* algorithms within ROS2 Nav2
- Benchmark performance through real-time path planning experiments
- Evaluate search efficiency, smoothness, and resource feasibility
- Provide practical algorithm-selection guidelines for real robots

---

## System Architecture

| Component | Technology Used |
|----------|----------------|
| Middleware | ROS2 Humble |
| Global Planner | Custom Dijkstra / A* / Theta* implementations |
| Simulation | Gazebo 11 |
| Visualization | RViz2 |
| Robot Model | TurtleBot3 Burger |
| Mapping | 2D LiDAR SLAM (SLAM Toolbox) |

Simulation conducted in an **8m × 8m** static environment mapped at **0.05m resolution**.

---

## Algorithm Overview

| Algorithm | Key Characteristics |
|----------|-------------------|
| Dijkstra | Guarantees optimal cost but explores blindly; slowest |
| A* | Heuristic-guided; faster and scalable |
| Theta* | Any-angle path; shortest and smoothest trajectory |

Theta* uses line-of-sight checks to generate smooth and efficient paths.

---

## Performance Results

| Metric | Dijkstra | A* | Theta* |
|--------|---------|-----|--------|
| Path Length | 4.17 m | 4.17 m | **3.87 m** |
| Waypoints | 70 | 70 | **4** |
| Planning Time | 302.56 ms | 70.25 ms | **69.25 ms** |
| Nodes Explored | 7704 | 1404 | **188** |

### Key Insights

- A* is **6.4× faster** than Dijkstra
- Theta* smoothens motion and reduces energy waste
- Theta* is the most efficient for large search spaces

---

![RViz Result Image](https://github.com/user-attachments/assets/b344ed32-ae10-4fb7-bbf2-7f66ba207259)

- Start position set: (-1.938772201538086, -0.30759093165397644)

- Start: (-1.97, -0.28) - Bottom-left area
- Goal: (1.5, 1.5) - Top-right area
- Distance: ~3.7 meters diagonal
- Direction: Northeast diagonal (up and right)

- Red - Dijkstra
- Green - A*
- Blue - Theta*

---

## Real-Time & Memory Feasibility

| Refresh Rate | A* | Theta* | Dijkstra |
|------------|-----|--------|----------|
| 100 Hz (10ms) | Not feasible | Not feasible | Not feasible |
| 20 Hz (50ms) | Borderline | Borderline | Not feasible |
| 5 Hz (200ms) | Feasible | Feasible | Not Feasible |

Memory usage remains under **50 KB** for a 100×100 grid.

---

## Practical Recommendations

| Application | Recommended Planner |
|------------|--------------------|
| Warehouse / Industrial | A* |
| Outdoor / Search & Rescue | Theta* |
| Domestic / Service Robots | A* with smoothing |

---

## Author

- Chirag Khanna
- Daksh Jain
