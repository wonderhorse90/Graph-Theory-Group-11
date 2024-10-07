import itertools, heapq

n, e = int(input()), int(input())  # Number of vertices and edges
graph, adj_list = {}, {i: [] for i in range(1, n + 1)}

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

for _ in range(e):
    edge_info = input().split()
    eid, u, v, w = int(edge_info[0]), int(edge_info[1]), int(edge_info[2]), int(edge_info[3])
    if u > v: u, v = v, u
    add_edge(eid, u, v, w)

def calc_degrees(adj_list):
    return {i: len(adj_list[i]) for i in adj_list}

def find_odd_vertices(degree):
    return [v for v in degree if degree[v] % 2 != 0]

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

def match_odd_vertices(odd_vertices):
    pairs = itertools.combinations(odd_vertices, 2)
    return {(u, v): dijkstra(u, v) for u, v in pairs}

def add_matching_edges(odd_vertices, pairings):
    total_cost, route = 0, []
    while odd_vertices:
        u = odd_vertices.pop(0)
        best_pair = min((v for v in odd_vertices), key=lambda v: pairings[(u, v)][0])
        odd_vertices.remove(best_pair)
        total_cost += pairings[(u, best_pair)][0]
        route += pairings[(u, best_pair)][1]
    return total_cost, route

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

start = int(input())
chinese_postman()
