import sys

class Path:
    def __init__(self, label, from_node, to_node, weight):
        self.label = label
        self.from_node = from_node
        self.to_node = to_node
        self.weight = weight

node_count = 0
edge_count = 0
all_edges = []
adjacency_list = []
start = 0
lowest_cost = sys.maxsize
optimal_path = []

def solve_tsp(current_node, accumulated_cost, current_path, visited_nodes, visit_count):
    global lowest_cost, optimal_path

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

    for path in adjacency_list[current_node]:
        neighbor = path.to_node if path.from_node == current_node else path.from_node
        if not visited_nodes[neighbor]:
            visited_nodes[neighbor] = True
            accumulated_cost += path.weight
            current_path.append(path.label)

            if accumulated_cost >= lowest_cost:
                current_path.pop()
                accumulated_cost -= path.weight
                visited_nodes[neighbor] = False
                continue

            solve_tsp(neighbor, accumulated_cost, current_path, visited_nodes, visit_count + 1)

            current_path.pop()
            accumulated_cost -= path.weight
            visited_nodes[neighbor] = False

node_count = int(input())
edge_count = int(input())

all_edges = []
adjacency_list = [[] for _ in range(node_count + 1)]

for _ in range(edge_count):
    label, u, v, cost = map(int, input().split())
    new_edge = Path(label, u, v, cost)
    all_edges.append(new_edge)
    adjacency_list[u].append(new_edge)
    adjacency_list[v].append(new_edge)

start = int(input())

visited_nodes = [False] * (node_count + 1)
visited_nodes[start] = True
current_path = []

solve_tsp(start, 0, current_path, visited_nodes, 1)

print(f"Cost: {lowest_cost}")
print("Route: ", ", ".join(map(str, optimal_path)))
