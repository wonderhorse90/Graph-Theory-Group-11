# Number 1

# Traveling Salesman Problem (TSP) Solver

This Python code implements a solution to the Traveling Salesman Problem (TSP) using recursion and backtracking. The goal of this program is to find the shortest possible route that visits each city exactly once and returns to the starting city.

## Explanation of the Code

### 1. **Classes and Global Variables**

```python
class Path:
    def __init__(self, label, from_node, to_node, weight):
        self.label = label
        self.from_node = from_node
        self.to_node = to_node
        self.weight = weight
```

Path Class: This class is used to represent an edge between two nodes (or cities).

`label` - A unique identifier for the path (edge).

`from_node` - The starting node of the edge.

`to_node` - The destination node of the edge.

`weight` - The cost (or distance) associated with this path.

```python
node_count = 0
edge_count = 0
all_edges = []
adjacency_list = []
start = 0
lowest_cost = sys.maxsize
optimal_path = []
```

Global Variables:

`node_count` - The total number of nodes (or cities).

`edge_count` - The total number of edges (or paths between cities).

`all_edges` - A list to store all the edges (of type Path).

`adjacency_list` - A list of lists where each node stores its adjacent nodes. This is a typical adjacency list representation for a graph.

`start` - The starting node (city) for the TSP.

`lowest_cost` - Stores the minimum cost (shortest distance) found for completing the tour.

`optimal_path` - Stores the labels of the edges in the optimal tour path.

```python
#solve_tsp Function
def solve_tsp(current_node, accumulated_cost, current_path, visited_nodes, visit_count):
    global lowest_cost, optimal_path
```
This function solves the TSP using recursion and backtracking.

`current_node` - The current city being visited.
`accumulated_cost` - The total cost accumulated so far in the current path.
`current_path` - The list of edges that represent the current tour path.
`visited_nodes` - A boolean array to track the cities that have been visited.
`visit_count` - A counter that tracks how many cities have been visited so far.

BAll Cities Visited
```python
if visit_count == node_count:
    for path in adjacency_list[current_node]:
        neighbor = path.to_node if path.from_node == current_node else path.from_node
        if neighbor == start:
            accumulated_cost += path.weight
            current_path.append(path.label)
            if accumulated_cost < lowest_cost:
                lowest_cost = accumulated_cost
                optimal_path = current_path[:]
            current_path.pop()
            accumulated_cost -= path.weight
    return
```
This is the base case of the recursion. If all cities have been visited `(visit_count == node_count)`, the function checks if there is a path back to the starting city.
If a valid path back exists, it updates the `lowest_cost` and `optimal_path` if the accumulated cost is lower than any previously found tour.

Recursive Case - Exploring Neighboring Cities
```python
for path in adjacency_list[current_node]:
    neighbor = path.to_node if path.from_node == current_node else path.from_node
    if not visited_nodes[neighbor]:
        visited_nodes[neighbor] = True
        accumulated_cost += path.weight
        current_path.append(path.label)
```
This part of the function explores all the adjacent cities (neighbors) that have not been visited yet. For each unvisited neighbor, it:
Marks the neighbor as visited.
- Adds the edge weight to the accumulated cost.
- Appends the edge label to the current path.

Backtracking and Pruning
```python
if accumulated_cost >= lowest_cost:
    current_path.pop()
    accumulated_cost -= path.weight
    visited_nodes[neighbor] = False
    continue
```

Pruning: If the accumulated cost already exceeds the current `lowest_cost`, the function prunes this path by backtracking without further exploration, as this path won't lead to an optimal solution.

After exploring all possible paths from the current city, the function backtracks:

Removes the last edge from the `current_path`.
Subtracts the edge weight from `accumulated_cost`.
Marks the neighbor as unvisited again `(visited_nodes[neighbor] = False)`.

Input Parsing and Initialization
```python
node_count = int(input())
edge_count = int(input())

all_edges = []
adjacency_list = [[] for _ in range(node_count + 1)]
```
First, we read the number of nodes (cities) and edges (paths). Then, we initialize the `adjacency_list` to store the graph in an adjacency list format.

```python
for _ in range(edge_count):
    label, u, v, cost = map(int, input().split())
    new_edge = Path(label, u, v, cost)
    all_edges.append(new_edge)
    adjacency_list[u].append(new_edge)
    adjacency_list[v].append(new_edge)
```
For each edge, the code reads the path label, starting node, ending node, and the cost (weight). This information is stored in the `all_edges` list and added to the `adjacency_list`.

```python
start = int(input())
```
Finally, we read the starting city for the TSP.

Driver Code and Result Output
```python
visited_nodes = [False] * (node_count + 1)
visited_nodes[start] = True
current_path = []

solve_tsp(start, 0, current_path, visited_nodes, 1)

print(f"Cost: {lowest_cost}")
print("Route: ", ", ".join(map(str, optimal_path)))
```
The `visited_nodes array` is initialized with all False values, except for the starting city which is set to True.
The `solve_tsp` function is called with the initial parameters.
After the function completes, the program prints the optimal tour cost `(lowest_cost)` and the route (labels of the paths in `optimal_path`).

Input Format
The program expects the following input:

`node_count` (integer): Number of cities.
`edge_count` (integer): Number of paths (edges) between cities.
For each path:
`label`, `u`, `v`, `cost`: where label is the identifier of the path, u is the starting city, v is the destination city, and cost is the weight of the path.
`start`: The starting city for the TSP.
Output
The program prints:

The lowest cost of the optimal route.
The labels of the paths in the optimal route.

Example:
input:
3 
4 
0 1 2 10 
1 2 3 5
2 3 1 7 
3 3 1 2 
1

output:
Cost: 17
Route: 0, 1, 3


