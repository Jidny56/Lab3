class Maze:
    def __init__(self, grid):
        self.grid = grid
        self.start = self.find_start()
        self.goal = self.find_goal()

    def find_start(self):
        for row in range(len(self.grid)):
            for col in range(len(self.grid[0])):
                if self.grid[row][col] == 'S':
                    return (row, col)
        return None

    def find_goal(self):
        for row in range(len(self.grid)):
            for col in range(len(self.grid[0])):
                if self.grid[row][col] == 'G':
                    return (row, col)
        return None

    def is_valid_move(self, state):
        row, col = state
        if 0 <= row < len(self.grid) and 0 <= col < len(self.grid[0]):
            return self.grid[row][col] != 1  # Not a wall
        return False

    def get_successors(self, state):
        row, col = state
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # up, down, left, right
        successors = []
        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            new_state = (new_row, new_col)
            if self.is_valid_move(new_state):
                successors.append(new_state)
        return successors


class DFS:
    def __init__(self, maze):
        self.maze = maze

    def search(self):
        stack = [(self.maze.start, [])]  # (current_state, path)
        visited = set()
        
        while stack:
            state, path = stack.pop()
            if state == self.maze.goal:
                return path  # Return the path to the goal
            
            if state not in visited:
                visited.add(state)
                for successor in self.maze.get_successors(state):
                    if successor not in visited:
                        stack.append((successor, path + [successor]))
        return []  # No path found


from collections import deque

class BFS:
    def __init__(self, maze):
        self.maze = maze

    def search(self):
        queue = deque([(self.maze.start, [])])  # (current_state, path)
        visited = set()
        
        while queue:
            state, path = queue.popleft()
            if state == self.maze.goal:
                return path  # Return the path to the goal
            
            if state not in visited:
                visited.add(state)
                for successor in self.maze.get_successors(state):
                    if successor not in visited:
                        queue.append((successor, path + [successor]))
        return []  # No path found


import heapq

class UCS:
    def __init__(self, maze):
        self.maze = maze

    def search(self):
        start = self.maze.start
        goal = self.maze.goal
        
        # Priority queue: (cost, state, path)
        frontier = [(0, start, [])]  # Cost, state, path
        visited = {}
        
        while frontier:
            cost, state, path = heapq.heappop(frontier)
            if state == goal:
                return path  # Return the path to the goal
            
            if state not in visited or cost < visited[state]:
                visited[state] = cost
                for successor in self.maze.get_successors(state):
                    new_cost = cost + 1  # All moves have the same cost
                    heapq.heappush(frontier, (new_cost, successor, path + [successor]))
        return []  # No path found


import time

def run_search_algorithm(algorithm, maze):
    start_time = time.time()
    solution = algorithm.search()
    end_time = time.time()
    time_taken = end_time - start_time
    nodes_expanded = len(algorithm.maze.get_successors(algorithm.maze.start))  # This is a rough approximation
    return time_taken, nodes_expanded


tinyMaze = [
    [1, 1, 1, 1, 1],
    [1, 'S', 0, 'G', 1],
    [1, 1, 1, 1, 1]
]

mediumMaze = [
    [1, 1, 1, 1, 1, 1, 1],
    [1, 'S', 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 0, 1],
    [1, 0, 1, 'G', 1, 0, 1],
    [1, 1, 1, 1, 1, 1, 1]
]

bigMaze = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 'S', 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 1, 1, 0, 1],
    [1, 1, 1, 1, 0, 1, 0, 0, 1],
    [1, 0, 0, 1, 0, 1, 0, 0, 1],
    [1, 1, 0, 0, 0, 0, 1, 'G', 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1]
]

mazes = {
    "tinyMaze": Maze(tinyMaze),
    "mediumMaze": Maze(mediumMaze),
    "bigMaze": Maze(bigMaze)
}


algorithms = {
    "DFS": DFS,
    "BFS": BFS,
    "UCS": UCS
}

for maze_name, maze in mazes.items():
    print(f"Running search algorithms on {maze_name}...")
    
    for algorithm_name, algorithm_class in algorithms.items():
        algorithm = algorithm_class(maze)
        time_taken, nodes_expanded = run_search_algorithm(algorithm, maze)
        
        print(f"{algorithm_name}: Time taken = {time_taken:.4f}s, Nodes expanded = {nodes_expanded}")
