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

### 1. Initialization Functions
### is_safe Function:
```python
def is_safe(x, y, board, N, M):
    return 0 <= x < N and 0 <= y < M and board[x][y] == -1
```
Purpose: Checks if a given position (x, y) is within the boundaries of the board and whether it has been visited.
Parameters:
    - x, y: Coordinates of the position to check.
    - board: The chessboard (2D array) representing visited and unvisited squares.
    - N, M: Dimensions of the board (number of rows N and columns M).
Return: True if the position is valid and unvisited; False otherwise.

### get_degree Function:
```python
def get_degree(x, y, board, moves_x, moves_y, N, M):
    count = 0
    for i in range(8):
        next_x = x + moves_x[i]
        next_y = y + moves_y[i]
        if is_safe(next_x, next_y, board, N, M):
            count += 1
    return count
```
Purpose: Calculates the number of onward moves a knight can make from a given position (x, y).
Parameters: Same as above, plus moves_x and moves_y which contain the possible moves of the knight.
Logic: Iterates over all 8 possible knight moves and counts how many of them lead to valid (safe) positions.
Return: The number of safe onward moves from (x, y).

### 2. Displaying the Solution
### print_solution Function:
```python
def print_solution(board):
    result = []
    for i in range(len(board)):
        for j in range(len(board[i])):
            result.append((i, j, board[i][j]))
    result.sort(key=lambda x: x[2])
    for x, y, move in result:
        print(x, y)
```
Purpose: Outputs the sequence of knight's moves in the order they were visited.
Parameters: board (2D array with move numbers).
Logic:
    - Creates a list called result to store (row, column, move_number) for each position.
    - Sorts the list based on move_number to ensure the output is in the order of the knight's moves.
    - Iterates through the sorted list to print each move (x, y).

### 3. Main Solving Function
### solve_knights_tour Function:
```python
def solve_knights_tour(N, M, start_x, start_y):
    board = [[-1 for _ in range(M)] for _ in range(N)]
    moves_x = [2, 1, -1, -2, -2, -1, 1, 2]
    moves_y = [1, 2, 2, 1, -1, -2, -2, -1]
    board[start_x][start_y] = 0
    if not solve_knights_tour_util(start_x, start_y, 1, board, moves_x, moves_y, N, M):
        print("No solution exists")
    else:
        print_solution(board)
```
Purpose: Sets up the board and starts the process of finding the knight’s tour.
Parameters:
    - N, M: Dimensions of the board.
    - start_x, start_y: The starting position of the knight.
Logic:
    - Initializes a board filled with -1 to represent unvisited squares.
    - Defines moves_x and moves_y to represent the possible movements of a knight (up-right, up-left, etc.).
    - Marks the starting position with 0 (the first move).
    - Calls solve_knights_tour_util to try to find a solution.
    - If the solution exists, it calls print_solution to display it; otherwise, it prints "No solution exists".

### 4. Recursive Utility Function (Core of the Algorithm)
### solve_knights_tour_util Function:
```python
def solve_knights_tour_util(x, y, move_count, board, moves_x, moves_y, N, M):
    if move_count == N * M:
        return True

    next_moves = []
    for i in range(8):
        next_x = x + moves_x[i]
        next_y = y + moves_y[i]
        if is_safe(next_x, next_y, board, N, M):
            degree = get_degree(next_x, next_y, board, moves_x, moves_y, N, M)
            next_moves.append((degree, next_x, next_y))
    next_moves.sort()

    for _, next_x, next_y in next_moves:
        board[next_x][next_y] = move_count
        if solve_knights_tour_util(next_x, next_y, move_count + 1, board, moves_x, moves_y, N, M):
            return True
        board[next_x][next_y] = -1

    return False
```
Purpose: Recursively attempts to build the knight’s tour using backtracking and Warnsdorff’s heuristic.
Parameters:
    - x, y: Current position of the knight.
    - move_count: The current move number.
    - board, moves_x, moves_y, N, M: As described earlier.
Logic:
    - Base Case: If move_count equals N * M, all squares have been visited, so return True.
    - Generate Next Moves: Iterates over all possible knight moves. If a move is safe, it calculates the degree using get_degree and appends the move and its degree to next_moves.
    - Sort Next Moves: Sorts next_moves based on their degree, which means it attempts moves with fewer onward possibilities first.
    - Recursive Call: Iteratively tries each sorted move:
        - Marks the next position with the current move_count.
        - Recursively calls itself with the updated move_count.
        - If the recursive call finds a solution, returns True.
        - If not, backtracks by marking the position as unvisited (-1).
    - Return False: If no valid moves are found, returns False to backtrack.

### 5. Driver Code
This part is responsible for handling user input:
```python
if __name__ == "__main__":
    N, M = map(int, input("Enter the size of the board (N M): ").split())
    start_x, start_y = map(int, input("Enter the starting position of the knight (x y): ").split())
    solve_knights_tour(N, M, start_x, start_y)
```
Purpose: Reads input from the user, including the board size (N and M) and the knight’s starting position (start_x and start_y).
Logic: Uses input() to get values and converts them into integers using map().

### How It All Works Together:
- Input: The user provides the board size and starting position.
- Board Setup: solve_knights_tour sets up the board and attempts to find a tour.
- Recursive Backtracking with Warnsdorff’s Rule: solve_knights_tour_util recursively tries to place the knight on each position while choosing paths with fewer onward options.
- Backtracking: If a path fails to cover all squares, it backtracks and tries a different path.
- Output: If a solution is found, print_solution outputs the move sequence. Otherwise, "No solution exists" is printed.

### Summary:
- Heuristic: Warnsdorff's rule guides the knight to positions with fewer onward options, helping to avoid early dead-ends.
- Backtracking: The recursive function explores all possible paths, backtracking when a dead-end is reached.
- Efficiency: Sorting moves based on the degree improves the likelihood of finding a solution more quickly compared to simple brute-force approaches.

