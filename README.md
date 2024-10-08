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

# Number 2
## Chinese Postman Problem
This code implements a solution for the Chinese Postman Problem. The goal is to find the least-cost circuit on a graph where each edge is traversed at least once, and all vertices are visited.

### Imports and Input Parsing:
```
import itertools, heapq

n, e = int(input()), int(input())  # Number of vertices and edges
graph, adj_list = {}, {i: [] for i in range(1, n + 1)}
```
Imports:

    - itertools: Provides useful iterators, like combinations.
    - heapq: Implements a priority queue used in Dijkstra's algorithm.
    
Input Parsing:

    - n is the number of vertices.
    - e is the number of edges.
    - graph: A dictionary to store edges as (edge_id: (u, v, w)), where u and v are vertices and w is the weight (cost) of the edge.
    - adj_list: Adjacency list to store the graph as a dictionary of vertex i and its neighboring vertices, along with the corresponding edge information.
    
### Adding an Edge (add_edge function):
```
def add_edge(eid, u, v, w):
    old_edges = [(adj, wt, name) for adj, wt, name in adj_list[u] if adj == v and wt > w]
    if old_edges:
        old_name = old_edges[0][2]
        adj_list[u] = [(adj, wt, name) for adj, wt, name in adj_list[u] if adj != v]
        adj_list[v] = [(adj, wt, name) for adj, wt, name in adj_list[v] if adj != u]
        del graph[old_name]

        graph[eid] = (u, v, w)
        adj_list[u].append((v, w, eid))
        adj_list[v].append((u, w, eid))

        graph[old_name] = (u, v, old_edges[0][1])
        adj_list[u].append((v, old_edges[0][1], old_name))
        adj_list[v].append((u, old_edges[0][1], old_name))
    else:
        graph[eid] = (u, v, w)
        adj_list[u].append((v, w, eid))
        adj_list[v].append((u, w, eid))
```
- add_edge adds an edge between two vertices u and v with a weight w.
- If there is already an edge between u and v with a higher weight, it is replaced by the new edge with the lower weight. The graph dictionary and adjacency list are updated to reflect this.
- If no such edge exists, the edge is simply added.

### Building the Graph (Reading edges):
```
for _ in range(e):
    edge_info = input().split()
    eid, u, v, w = int(edge_info[0]), int(edge_info[1]), int(edge_info[2]), int(edge_info[3])
    if u > v: u, v = v, u
    add_edge(eid, u, v, w)
```

- Edge Input: Loops through e edges.
- For each edge, it reads the edge ID (eid), vertices u and v, and weight w.
- If u > v, it swaps them to maintain a consistent order for simplicity.
- Calls add_edge to add the edge to the graph.

### Calculating Degrees (calc_degrees function):
```
def calc_degrees(adj_list):
    return {i: len(adj_list[i]) for i in adj_list}
```

- calc_degrees calculates the degree of each vertex. The degree is the number of edges connected to the vertex.
- Returns a dictionary where the keys are vertices and values are their degrees.

### Finding Odd-Degree Vertices (find_odd_vertices function):
```
def find_odd_vertices(degree):
    return [v for v in degree if degree[v] % 2 != 0]
```
find_odd_vertices identifies all vertices with an odd degree. These vertices need to be paired to make the graph Eulerian

### Dijkstra's Algorithm (dijkstra function):
```
def dijkstra(src, dst):
    dist = {i: float('inf') for i in range(1, n + 1)}
    dist[src], prev, edge_used = 0, {i: None for i in range(1, n + 1)}, {}
    pq = [(0, src)]
    
    while pq:
        cur_dist, u = heapq.heappop(pq)
        if cur_dist > dist[u]: continue
        for v, w, eid in adj_list[u]:
            if cur_dist + w < dist[v]:
                dist[v], prev[v], edge_used[v] = cur_dist + w, u, eid
                heapq.heappush(pq, (dist[v], v))
    
    path, cur = [], dst
    while prev[cur]: path.append(edge_used[cur]); cur = prev[cur]
    return dist[dst], path[::-1]
```

- Dijkstra’s Algorithm is used to find the shortest path between two vertices src (source) and dst (destination).
- It uses a priority queue (heapq) to explore vertices with the smallest current distance.
- After computing the shortest distance, it reconstructs the path from src to dst.
- Returns both the shortest distance and the path of edges.

### Matching Odd-Degree Vertices (match_odd_vertices function):
```
def match_odd_vertices(odd_vertices):
    pairs = itertools.combinations(odd_vertices, 2)
    return {(u, v): dijkstra(u, v) for u, v in pairs}
```

- match_odd_vertices finds the shortest paths between all pairs of odd-degree vertices.
- Uses itertools.combinations to generate pairs of odd-degree vertices, then calls dijkstra to compute the shortest paths between each pair.

### Adding Matching Edges (add_matching_edges function):
```
def add_matching_edges(odd_vertices, pairings):
    total_cost, route = 0, []
    while odd_vertices:
        u = odd_vertices.pop(0)
        best_pair = min((v for v in odd_vertices), key=lambda v: pairings[(u, v)][0])
        odd_vertices.remove(best_pair)
        total_cost += pairings[(u, best_pair)][0]
        route += pairings[(u, best_pair)][1]
    return total_cost, route
```
- add_matching_edges pairs up odd-degree vertices using the shortest paths calculated earlier.
- It selects pairs that minimize the total additional cost, updates the route, and removes the odd-degree vertices as they are paired.
- Returns the total cost of the added edges and the route.

### Eulerian Tour (euler_tour function):
```
def euler_tour(start):
    stack, tour, used = [start], [], set()
    while stack:
        u = stack[-1]
        available = [(v, w, eid) for v, w, eid in adj_list[u] if eid not in used]
        if available:
            v, w, eid = min(available, key=lambda x: x[1])
            used.add(eid)
            stack.append(v)
        else:
            tour.append(stack.pop())
    return tour
```

- euler_tour computes an Eulerian circuit. It starts from a given vertex and finds a path that visits every edge exactly once.
- It uses a stack to track the current path and a set used to mark edges that have been traversed.

### Chinese Postman Solution (chinese_postman function):
```
def chinese_postman():
    deg = calc_degrees(adj_list)
    odd_vertices = find_odd_vertices(deg)
    extra_cost, extra_edges = 0, []
    
    if odd_vertices:
        pairings = match_odd_vertices(odd_vertices)
        extra_cost, extra_edges = add_matching_edges(odd_vertices, pairings)

    total_cost = sum(w for _, (_, _, w) in graph.items()) + extra_cost
    eulerian_path = euler_tour(start)
    print(f"Cost: {total_cost}")
    print("Route:", ', '.join(str(edge) for edge in list(graph.keys()) + extra_edges))
```
chinese_postman solves the Chinese Postman Problem.
    - First, it calculates the degree of all vertices and identifies the odd-degree vertices.
    - If there are odd-degree vertices, it pairs them using the minimum-cost matching and computes the additional cost.
    - Then, it calculates the Eulerian tour and outputs the total cost and the Eulerian path.

# Number 3

# Knight's Problem Solver

```python
FUNCTION is_safe(x, y, board, N, M):
    RETURN (0 <= x < N) AND (0 <= y < M) AND (board[x][y] == -1)

FUNCTION get_degree(x, y, board, moves_x, moves_y, N, M):
    degree = 0
    FOR each move in possible knight moves (8 directions):
        next_x = x + moves_x[i]
        next_y = y + moves_y[i]
        IF is_safe(next_x, next_y, board, N, M):
            degree += 1
    RETURN degree

FUNCTION print_solution(board):
    result = []
    FOR each cell in board:
        result.append((row, column, move_number))
    SORT result by move_number
    FOR each element in sorted result:
        PRINT row, column

FUNCTION solve_knights_tour(N, M, start_x, start_y):
    board = 2D array of size N x M filled with -1
    moves_x = [2, 1, -1, -2, -2, -1, 1, 2]  // Possible knight moves in x direction
    moves_y = [1, 2, 2, 1, -1, -2, -2, -1]  // Possible knight moves in y direction
    board[start_x][start_y] = 0  // Start from the given position

    IF solve_knights_tour_util(start_x, start_y, 1, board, moves_x, moves_y, N, M):
        print_solution(board)
    ELSE:
        PRINT "No solution exists"

FUNCTION solve_knights_tour_util(x, y, move_count, board, moves_x, moves_y, N, M):
    IF move_count == N * M:
        RETURN True  // All cells are visited

    next_moves = []
    FOR each move in possible knight moves:
        next_x = x + moves_x[i]
        next_y = y + moves_y[i]
        IF is_safe(next_x, next_y, board, N, M):
            degree = get_degree(next_x, next_y, board, moves_x, moves_y, N, M)
            next_moves.append((degree, next_x, next_y))
    
    SORT next_moves based on degree (ascending order)

    FOR each move in next_moves:
        board[next_x][next_y] = move_count
        IF solve_knights_tour_util(next_x, next_y, move_count + 1, board, moves_x, moves_y, N, M):
            RETURN True
        board[next_x][next_y] = -1  // Backtracking

    RETURN False

// MAIN program
INPUT N, M  // Size of the board
INPUT start_x, start_y  // Starting position of the knight
solve_knights_tour(N, M, start_x, start_y)
```

1. Input: The program reads the board size `N` (rows) and `M` (columns) and the knight's starting position `(start_x, start_y)`.

2. Initialization:
    a. `board`: A 2D array of size `N x M` is initialized with `-1`, indicating that all squares are unvisited.
    b. The knight's starting position is marked with `0`, representing the first move.
    c. `moves_x` and `moves_y`: Arrays representing the possible moves a knight can make (e.g., `+2, +1`, `+1, +2`, etc.).

3. solve_knights_tour_util Function:
    a. Base Case: If the `move_count` equals `N * M`, it means all squares have been visited, and a solution is found. The function returns `True`.
    b. Generate Possible Moves: For each possible knight move from the current position `(x, y)`, it checks if the next move is safe using `is_safe`.
    c. Degree Calculation: It calculates the number of onward moves (degree) from each safe next position using `get_degree`.
    d. Sort by Degree: The next moves are sorted based on their degree in ascending order, prioritizing moves with fewer onward options.
    e. Backtracking: For each sorted move:
        - Update the `board` with the `move_count` and make a recursive call with the updated move count.
        - If the recursive call succeeds, return `True`.
        - Otherwise, backtrack by resetting the position to `-1`.
    If no valid moves are found, return `False`.

4. print_solution Function: If a solution is found, it prints out the sequence of moves by sorting the positions based on the order they were visited.

5. Output:
    a. If a solution exists, the program prints the sequence of `(row, column)` pairs representing the knight's tour.
    b. If no solution exists, it outputs "No solution exists".

Key Concepts:
    a. Warnsdorff's Rule: The heuristic prioritizes moves that leave the knight with the fewest subsequent options, helping to avoid getting trapped.
    b. Backtracking: The algorithm tries each potential move and backtracks if a move leads to a dead end.
    c. Sorting Moves: Sorting the next possible moves based on degree helps the knight navigate the board more effectively.

