import heapq
from typing import Set, Dict, List, Tuple, Optional, Any

class Graph:
    def __init__(self) -> None:
        self.nodes: Set[str] = set()
        self.edges: Dict[str, List[str]] = {}
        self.distances: Dict[Tuple[str, str], int] = {}

    def add_node(self, value: str) -> None:
        self.nodes.add(value)
        self.edges[value] = []

    def add_edge(self, from_node: str, to_node: str, distance: int) -> None:
        if from_node not in self.edges:
            self.add_node(from_node)
        if to_node not in self.edges:
            self.add_node(to_node)
            
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)  # Assuming bidirectional roads
        self.distances[(from_node, to_node)] = distance
        self.distances[(to_node, from_node)] = distance

def dijkstra(graph: Graph, start: str, goal: str) -> Tuple[Optional[List[str]], float]:
    """
    Finds the shortest path from start to goal using Dijkstra's algorithm (Uniform Cost Search).
    """
    if start not in graph.nodes or goal not in graph.nodes:
        return None, float('inf')

    # Priority queue stores tuples of (cumulative_distance, current_node)
    pq: List[Tuple[float, str]] = [(0.0, start)]
    
    # Dictionary to keep track of the shortest distance to each node
    shortest_distances: Dict[str, float] = {node: float('inf') for node in graph.nodes}
    shortest_distances[start] = 0.0
    
    # Dictionary to reconstruct the path
    previous_nodes: Dict[str, Optional[str]] = {node: None for node in graph.nodes}
    
    # Set to keep track of visited nodes
    visited: Set[str] = set()

    while pq:
        current_distance, current_node = heapq.heappop(pq)
        
        # If we reached the goal, we can reconstruct the path and return
        if current_node == goal:
            path: List[str] = []
            curr: Optional[str] = current_node
            while curr is not None:
                path.insert(0, curr)
                curr = previous_nodes[curr]
            return path, current_distance
            
        if current_node in visited:
            continue
            
        visited.add(current_node)
        
        # Explore neighbors
        for neighbor in graph.edges[current_node]:
            if neighbor in visited:
                continue
                
            distance = graph.distances[(current_node, neighbor)]
            new_distance = current_distance + distance
            
            # If the new distance to the neighbor is shorter, update it
            if new_distance < shortest_distances[neighbor]: 
                shortest_distances[neighbor] = new_distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(pq, (float(new_distance), neighbor))
                
    return None, float('inf') # No path found

def main():
    # Example Graph of Indian Cities and approximate road distances (in km)
    # Distances are rough estimates for demonstration purposes.
    india_map = Graph()
    
    edges = [
        ('Delhi', 'Jaipur', 280),
        ('Delhi', 'Agra', 230),
        ('Delhi', 'Chandigarh', 250),
        ('Jaipur', 'Ahmedabad', 670),
        ('Jaipur', 'Agra', 240),
        ('Ahmedabad', 'Mumbai', 520),
        ('Ahmedabad', 'Pune', 660),
        ('Mumbai', 'Pune', 150),
        ('Mumbai', 'Goa', 590),
        ('Pune', 'Hyderabad', 560),
        ('Pune', 'Bangalore', 840),
        ('Goa', 'Mangalore', 360),
        ('Mangalore', 'Bangalore', 350),
        ('Mangalore', 'Kochi', 420),
        ('Kochi', 'Thiruvananthapuram', 200),
        ('Bangalore', 'Chennai', 350),
        ('Bangalore', 'Hyderabad', 570),
        ('Chennai', 'Hyderabad', 630),
        ('Chennai', 'Madurai', 460),
        ('Madurai', 'Kochi', 270),
        ('Madurai', 'Thiruvananthapuram', 300),
        ('Hyderabad', 'Nagpur', 500),
        ('Nagpur', 'Bhopal', 350),
        ('Nagpur', 'Raipur', 280),
        ('Bhopal', 'Agra', 540),
        ('Bhopal', 'Jaipur', 600),
        ('Agra', 'Kanpur', 280),
        ('Kanpur', 'Lucknow', 90),
        ('Kanpur', 'Varanasi', 330),
        ('Lucknow', 'Varanasi', 300),
        ('Varanasi', 'Patna', 250),
        ('Patna', 'Kolkata', 580),
        ('Kolkata', 'Bhubaneswar', 440),
        ('Bhubaneswar', 'Visakhapatnam', 440),
        ('Visakhapatnam', 'Vijayawada', 350),
        ('Vijayawada', 'Chennai', 450),
        ('Vijayawada', 'Hyderabad', 270),
        ('Raipur', 'Bhubaneswar', 530)
    ]

    for start, end, dist in edges:
        india_map.add_edge(start, end, dist)

    print("--- Dijkstra's Algorithm (Uniform Cost Search) ---")
    print("Available Cities:", ", ".join(sorted(list(india_map.nodes))))
    print("-" * 50)

    queries = [
        ('Delhi', 'Chennai'),
        ('Mumbai', 'Kolkata'),
        ('Jaipur', 'Thiruvananthapuram'),
        ('Chandigarh', 'Goa')
    ]

    for start_city, goal_city in queries:
        print(f"\nFinding shortest path from {start_city} to {goal_city}...")
        path, distance = dijkstra(india_map, start_city, goal_city)
        if path:
            print(f"Path: {' -> '.join(path)}")
            print(f"Total Distance: {distance} km")
        else:
            print(f"No path found between {start_city} and {goal_city}.")

if __name__ == "__main__":
    main()
