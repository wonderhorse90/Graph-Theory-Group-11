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
