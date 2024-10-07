Traveling Salesman Problem (TSP) Solver in Python
This project provides a solution to the Traveling Salesman Problem (TSP) using a backtracking approach. The program calculates the lowest-cost route that visits each node (city) exactly once and returns to the starting node.

How It Works
The TSP solver works as follows:

Graph Representation: Cities are represented as nodes, and the paths between them are edges with associated weights (costs).
Recursive Backtracking: A recursive function, solve_tsp, explores all possible paths through the graph. As soon as a path has a higher cost than the best one found so far, it is abandoned (pruned).
Tracking Costs: The total accumulated cost is calculated as each path is explored. If a complete path (that visits all cities and returns to the starting city) has a lower cost than the current optimal solution, it becomes the new best path.
Adjacency List: An adjacency list is used to store the connections between cities, making it easier to access neighboring nodes during the recursive search.
Code Explanation
Class: Path
python
Salin kode
class Path:
    def __init__(self, label, from_node, to_node, weight):
        self.label = label
        self.from_node = from_node
        self.to_node = to_node
        self.weight = weight
This class defines the edges (or paths) between cities.
Each path has:
label: A unique identifier for the path.
from_node and to_node: The two nodes (cities) that this path connects.
weight: The cost or distance between the two nodes.
Global Variables
node_count: The total number of cities.
edge_count: The total number of paths connecting the cities.
all_edges: A list to store all the edges (paths) in the graph.
adjacency_list: A list of lists to store the neighbors of each node, used for exploring the graph efficiently.
start: The starting city for the TSP.
lowest_cost: Keeps track of the lowest cost found during the search.
optimal_path: Stores the path (sequence of edges) that results in the lowest cost.
Function: solve_tsp
python
Salin kode
def solve_tsp(current_node, accumulated_cost, current_path, visited_nodes, visit_count):
This function recursively solves the TSP by:
Base Case: When all cities are visited, it checks if there's a path back to the starting city and updates the lowest cost if the path is cheaper than the current best.
Recursive Case: It explores each neighbor of the current city. If the city hasn't been visited yet, it updates the accumulated cost and recursively calls solve_tsp for the next city.
Pruning: If the current cost exceeds the lowest cost found so far, the recursion stops (pruned).
Input Parsing
The program takes the number of cities (node_count) and the number of paths (edge_count) as input.
Each path (with label, start city, end city, and weight) is stored in both all_edges and adjacency_list.
Main Execution Flow
Parse the number of nodes and edges.
Store the edges in the adjacency list for efficient access.
Call the solve_tsp function starting from the start node.
Print the minimum cost and the optimal path after all possibilities are explored.
Input Format
First line: Integer node_count - the number of cities (nodes).
Second line: Integer edge_count - the number of paths (edges) between the cities.
Next edge_count lines: Each line contains four integers: label, from_node, to_node, and weight, representing an edge between two cities and the cost to travel between them.
Last line: The integer start, representing the starting city.
Example
Input:

Salin kode
4
6
1 1 2 10
2 1 3 15
3 1 4 20
4 2 3 35
5 2 4 25
6 3 4 30
1
Explanation:

The graph has 4 cities and 6 paths between them.
Path 1 connects city 1 and city 2 with a cost of 10.
The start city is city 1.
Output:

makefile
Salin kode
Cost: 80
Route: 1, 5, 6, 2
Explanation:

The minimum cost to visit all cities and return to the starting city (city 1) is 80.
The optimal path follows the sequence of edges with labels 1, 5, 6, and 2.
How to Run
Copy the code into a Python file, e.g., tsp_solver.py.
Run the file using Python in your terminal:
bash
Salin kode
python tsp_solver.py
Enter the input as described in the Input Format section.
Make sure that the input format follows the structure closely for correct results.
