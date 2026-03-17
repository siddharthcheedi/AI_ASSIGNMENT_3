import random
import heapq
import time
import math
from typing import List, Tuple, Dict, Optional, Any

class Node:
    def __init__(self, x: int, y: int, cost: float, heuristic: float, parent: Optional['Node'] = None):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.total_cost = cost + heuristic
        self.parent = parent

    def __lt__(self, other: 'Node'):
        return self.total_cost < other.total_cost
        
    def __eq__(self, other: object):
        if not isinstance(other, Node):
            return False
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

class DynamicUGVGrid:
    def __init__(self, width: int = 70, height: int = 70, initial_density: float = 0.10):
        self.width = width
        self.height = height
        self.grid: List[List[int]] = [[0 for _ in range(width)] for _ in range(height)]
        self.start: Tuple[int, int] = (0, 0)
        self.goal: Tuple[int, int] = (width - 1, height - 1)
        self.generate_initial_obstacles(initial_density)

    def generate_initial_obstacles(self, density: float) -> None:
        num_obstacles: int = int(self.width * self.height * density)
        obstacles_placed: int = 0
        while obstacles_placed < num_obstacles:
            rx = random.randint(0, self.width - 1)
            ry = random.randint(0, self.height - 1)
            if (rx, ry) == self.start or (rx, ry) == self.goal:
                continue
            if self.grid[ry][rx] == 0:
                self.grid[ry][rx] = 1
                obstacles_placed += 1 

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, x: int, y: int) -> List[Tuple[Tuple[int, int], float]]:
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
        neighbors: List[Tuple[Tuple[int, int], float]] = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self.grid[ny][nx] == 0: # Not an obstacle
                    cost = 1.0 if dx == 0 or dy == 0 else math.sqrt(2.0)
                    neighbors.append(((nx, ny), cost))
        return neighbors

def astar_search(env: DynamicUGVGrid, current_start: Tuple[int, int]) -> Tuple[Optional[List[Tuple[int, int]]], int]:
    """Computes A* path from current_start to goal."""
    start_node = Node(current_start[0], current_start[1], 0.0, env.heuristic(current_start, env.goal))
    open_set: List[Node] = []
    heapq.heappush(open_set, start_node)
    best_costs: Dict[Tuple[int, int], float] = {current_start: 0.0}
    nodes_expanded: int = 0
    
    while open_set:
        current = heapq.heappop(open_set)
        
        if (current.x, current.y) == env.goal:
            path: List[Tuple[int, int]] = []
            curr_node: Optional[Node] = current
            while curr_node:
                path.append((curr_node.x, curr_node.y))
                curr_node = curr_node.parent
            path.reverse()
            return path, nodes_expanded
            
        nodes_expanded += 1
        
        for neighbor_pos, move_cost in env.get_neighbors(current.x, current.y):
            new_cost = best_costs[(current.x, current.y)] + move_cost 
            if neighbor_pos not in best_costs or new_cost < best_costs[neighbor_pos]: 
                best_costs[neighbor_pos] = new_cost
                h = env.heuristic(neighbor_pos, env.goal)
                neighbor_node = Node(neighbor_pos[0], neighbor_pos[1], new_cost, h, current)
                heapq.heappush(open_set, neighbor_node)
                
    return None, nodes_expanded

def simulate_dynamic_environment():
    print("--- UGV Navigation with DYNAMIC Obstacles ---")
    print("Simulating a simple replanning approach.")
    print("Initial Grid Size: 70x70 | Initial Density: 10%")
    
    env = DynamicUGVGrid(width=70, height=70, initial_density=0.10)
    current_pos = env.start
    
    total_path_taken: List[Tuple[int, int]] = [current_pos]
    replans: int = 0
    total_nodes_expanded: int = 0
    
    print(f"\nCalculating initial path from {current_pos} to {env.goal}...")
    current_path, nodes = astar_search(env, current_pos)
    total_nodes_expanded += nodes
    
    if not current_path:
        print("No initial path found!")
        return
        
    print("Initial path found. Starting navigation...")
    
    path_index: int = 1
    
    while current_pos != env.goal:
        if current_path is None:
            break
        next_step = current_path[path_index] 
        
        # Simulate dynamic obstacle appearance with 5% chance at our next step
        # (Only if it's not the goal)
        if next_step != env.goal and random.random() < 0.05: 
            print(f"\n[!] DANGER: Unexpected dynamic obstacle detected at {next_step}!")
            env.grid[next_step[1]][next_step[0]] = 1  # Mark as blocked
            
            print(f"[*] Replanning path from current position {current_pos}...")
            replans += 1 
            current_path, nodes = astar_search(env, current_pos)
            total_nodes_expanded += nodes
            
            if not current_path:
                print(f"FAILURE: UGV is trapped at {current_pos}. No path to goal exists anymore.")
                break
                
            path_index = 1 # Reset path index for the new path
            next_step = current_path[path_index]
            print(f"[*] New path found. Resuming navigation.")
            
        # Move UGV
        current_pos = next_step
        total_path_taken.append(current_pos)
        path_index += 1
        
        # Print progress every 20 steps
        if len(total_path_taken) % 20 == 0:
            print(f"Progress: Currently at {current_pos}...")
            
    if current_pos == env.goal: 
        print(f"\nSUCCESS: UGV reached the goal at {env.goal}!") 
        
    print(f"\n--- Dynamic Navigation Summary ---")
    print(f"Total Steps Taken: {len(total_path_taken) - 1}")
    print(f"Number of Replans Required: {replans}")
    print(f"Total Search Nodes Expanded (Over all replans): {total_nodes_expanded}")

if __name__ == "__main__":
    simulate_dynamic_environment()
