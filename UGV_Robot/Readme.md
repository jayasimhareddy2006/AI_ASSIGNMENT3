#  UGV Path Planning using A* Algorithm

## Description

This project simulates an **Unmanned Ground Vehicle (UGV)** navigating a battlefield represented as a **70×70 grid**. The objective is to find the **shortest collision-free path** from a given start node to a goal node while avoiding obstacles.

The obstacles are generated randomly with different density levels, and the path is computed using the **A* (A-Star) algorithm**, which is an efficient heuristic-based search algorithm.

##  Objectives

- Model a battlefield as a 2D grid
- Generate obstacles with varying densities
- Implement A* algorithm for path planning
- Find optimal path from start to goal
- Evaluate performance using Measures of Effectiveness (MOE)
- Visualize the path

---

## Algorithm Used

##  A* Algorithm

- Combines **Dijkstra’s Algorithm** and **Greedy Best-First Search**
- Uses a heuristic function to improve efficiency
- Guarantees shortest path if heuristic is admissible

### Heuristic Used

- **Manhattan Distance**
