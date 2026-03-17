# Pathfinding AI Assignments

This repository contains three Python scripts that demonstrate different pathfinding algorithms and traversal techniques on simulated environments.

## 1. Dijkstra's Algorithm on Indian Cities (`dijkstra_india.py`)

This script implements Dijkstra's Algorithm (Uniform Cost Search) to find the shortest path between randomly seeded major cities in India based on rough distance estimates. 

### How it Works:
- **Graph Construction**: Cities are treated as **nodes** and the roads connecting them are **edges** with specific `weights` (distances in kilometers).
- **Algorithm**: The script initializes the starting city with a distance of 0 and all other cities with an infinite distance. It uses a **Priority Queue (Min-Heap)** to constantly evaluate the unvisited city with the lowest cumulative distance.
- **Path Reconstruction**: Once the destination node is reached, it traces back its previous nodes to construct the optimal path array and prints out the total geographical distance.

### How to Run:
```powershell
python AI_Assignment\dijkstra_india.py
```
*(Or if you are already inside the AI_Assignment folder: `python dijkstra_india.py`)*

---

## 2. A* Search - Static UGV Environment (`ugv_static.py`)

This script simulates an Unmanned Ground Vehicle (UGV) trying to navigate a 70x70 grid from point `(0,0)` to `(69,69)` with randomly generated **static** obstacles impeding its path. 

### How it Works:
- **Grid Generation**: A 2D array is generated. Based on the selected density (`Low 10%`, `Medium 20%`, `High 30%`), grid cells are converted into impassable obstacles (represented as `1`, whereas free space is `0`).
- **A* Algorithm**: The search relies heavily on a **Heuristic Function** to find an optimal path faster than Dijkstra. It calculates the total cost `f(n) = g(n) + h(n)` where:
    - `g(n)` = Actual travel cost from the start to the current node (using `1.0` for horizontal/vertical steps and `math.sqrt(2)` for diagonal steps).
    - `h(n)` = Estimated Manhattan or Euclidean distance from the current node to the final goal.
- **Execution**: The UGV executes its simulation on all three density levels and outputs performance metrics like path length and the number of expanded nodes.

### How to Run:
```powershell
python AI_Assignment\ugv_static.py
```
*(Or if you are already inside the AI_Assignment folder: `python ugv_static.py`)*

---

## 3. A* Search - Dynamic UGV Environment (`ugv_dynamic.py`)

This script is an advanced evolution of `ugv_static.py`. The UGV still uses A* to navigate, but the obstacles are now **dynamic**. The environment can change *while* the robot is traversing it.

### How it Works:
- **Initial Planning**: The Grid is generated exactly like the static version with a 10% obstacle density, and the UGV calculates its initial A* path.
- **Simulation Loop**: The UGV begins executing the planned path step-by-step.
- **Dynamic Obstacles**: At each step forward, there is a **5% probability** that an unmapped obstacle spawns directly in the UGV's path. 
- **Replanning (D* Lite Logic)**: When the UGV encounters a blocked path, it marks the grid cell as obstructed and calls the A* function *again* from its current coordinate to calculate a brand-new route to the goal. This continues until it either reaches the destination or gets permanently trapped.

### How to Run:
```powershell
python AI_Assignment\ugv_dynamic.py
```
*(Or if you are already inside the AI_Assignment folder: `python ugv_dynamic.py`)*
