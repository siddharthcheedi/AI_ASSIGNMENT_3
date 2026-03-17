import random
import heapq
import time
import math
from typing import List, Tuple, Dict, Optional, Any

class Node:
    def __init__(self, x: int, y: int, cost: float, heuristic: float, parent: Optional['Node'] = None):
        self.x = x
        self.y = y
        self.cost = cost # g(n)
        self.heuristic = heuristic # h(n)
        self.total_cost = cost + heuristic # f(n)
        self.parent = parent

    def __lt__(self, other: 'Node'):
        return self.total_cost < other.total_cost
        
    def __eq__(self, other: object):
        if not isinstance(other, Node):
            return False
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

class UGVGrid:
    def __init__(self, width: int = 70, height: int = 70, density_level: str = 'medium'):
        self.width = width
        self.height = height
        self.grid: List[List[int]] = [[0 for _ in range(width)] for _ in range(height)] # 0 is free, 1 is obstacle
        self.start: Tuple[int, int] = (0, 0)
        self.goal: Tuple[int, int] = (width - 1, height - 1)
        self.generate_obstacles(density_level)

    def generate_obstacles(self, level: str):
        densities = {
            'low': 0.10,
            'medium': 0.20,
            'high': 0.30
        }
        density: float = densities.get(level.lower(), 0.20)
        
        num_obstacles: int = int(self.width * self.height * density)
        obstacles_placed: int = 0
        
        while obstacles_placed < num_obstacles:
            rx = random.randint(0, self.width - 1)
            ry = random.randint(0, self.height - 1)
            
            # Don't place obstacle on start or goal
            if (rx, ry) == self.start or (rx, ry) == self.goal:
                continue
                
            if self.grid[ry][rx] == 0:
                self.grid[ry][rx] = 1
                obstacles_placed += 1 

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        # Euclidean distance
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, x: int, y: int) -> List[Tuple[Tuple[int, int], float]]:
        # 8-way movement (horizontal, vertical, diagonal)
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (-1, -1), (1, -1), (-1, 1)
        ]
        
        neighbors: List[Tuple[Tuple[int, int], float]] = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self.grid[ny][nx] == 0: # Not an obstacle
                    # Cost is 1 for horiz/vert, sqrt(2) for diagonal
                    cost = 1.0 if dx == 0 or dy == 0 else math.sqrt(2.0)
                    neighbors.append(((nx, ny), cost))
        return neighbors

def astar_search(env: UGVGrid) -> Tuple[Optional[List[Tuple[int, int]]], int, float]:
    """
    A* Search implementation.
    Returns the path, number of nodes expanded, and execution time.
    """
    start_time = time.time()
    
    start_node = Node(env.start[0], env.start[1], 0.0, env.heuristic(env.start, env.goal))
    
    open_set: List[Node] = []
    heapq.heappush(open_set, start_node)
    
    # Keep track of the best g-score for each cell
    best_costs: Dict[Tuple[int, int], float] = {env.start: 0.0}
    
    nodes_expanded: int = 0
    
    while open_set:
        current = heapq.heappop(open_set)
        
        if (current.x, current.y) == env.goal:
            end_time = time.time()
            
            # Reconstruct path
            path: List[Tuple[int, int]] = []
            curr_node: Optional[Node] = current
            while curr_node:
                path.append((curr_node.x, curr_node.y))
                curr_node = curr_node.parent
            path.reverse()
            
            return path, nodes_expanded, end_time - start_time
            
        nodes_expanded += 1
        
        for neighbor_pos, move_cost in env.get_neighbors(current.x, current.y):
            new_cost = best_costs[(current.x, current.y)] + move_cost 
            
            if neighbor_pos not in best_costs or new_cost < best_costs[neighbor_pos]:  
                best_costs[neighbor_pos] = new_cost
                h = env.heuristic(neighbor_pos, env.goal)
                neighbor_node = Node(neighbor_pos[0], neighbor_pos[1], new_cost, h, current)
                heapq.heappush(open_set, neighbor_node)
                
    end_time = time.time()
    return None, nodes_expanded, end_time - start_time # No path found

def main():
    levels = ['low', 'medium', 'high']
    
    print("--- UGV Navigation with Static Obstacles (A* Search) ---")
    print("Grid Size: 70x70")
    print("Start: (0, 0) | Goal: (69, 69)")
    
    for level in levels:
        print(f"\n--- Simulating Density Level: {level.upper()} ---")
        env = UGVGrid(width=70, height=70, density_level=level)
        
        path, nodes_expanded, exec_time = astar_search(env)
        
        if path:
            path_length = sum([1.0 if abs(path[i][0]-path[i-1][0]) + abs(path[i][1]-path[i-1][1]) == 1 else math.sqrt(2.0) for i in range(1, len(path))])
            print(f"Path Found!")
            # Tracing the path - list is too long to print fully, so let's summarize
            path_start = [path[i] for i in range(min(5, len(path)))] 
            path_end = [path[-i-1] for i in range(min(5, len(path)))][::-1]
            print(f"Path starts: {path_start} ... ends: {path_end}")
            print(f"-- Measures of Effectiveness --")
            print(f"Path Length: {path_length:.2f} units")
            print(f"Nodes Expanded: {nodes_expanded}")
            print(f"Execution Time: {exec_time:.5f} seconds")
        else:
            print(f"No path found! Dense obstacles blocked the goal.")
            print(f"-- Measures of Effectiveness --")
            print(f"Nodes Expanded (searched before failing): {nodes_expanded}")
            print(f"Execution Time: {exec_time:.5f} seconds")

if __name__ == "__main__":
    main()
