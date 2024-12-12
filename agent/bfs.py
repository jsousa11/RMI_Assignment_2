from collections import deque
from time import time
import heapq

class Node():
    """A node class for BFS Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

def bfs(maze, start, end):
    rows, cols = len(maze), len(maze[0])
    i = (27, 13)  # Offset para ajustar coordenadas
    priority_queue = []
    heapq.heappush(priority_queue, (0, start, []))  # (prioridade, posição, caminho)
    visited = set()

    while priority_queue:
        cost, current, path = heapq.heappop(priority_queue)
        path = path + [current]

        if current == end:
            return path, False

        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            neighbor = (current[0] + dx, current[1] + dy)

            if (neighbor[0] + i[0] < 0 or neighbor[0] + i[0] >= cols or
                neighbor[1] + i[1] < 0 or neighbor[1] + i[1] >= rows):
                continue

            if maze[i[1] - neighbor[1]][neighbor[0] + i[0]] == 'X' and neighbor not in visited:
                priority = cost + 1 + abs(neighbor[0] - end[0]) + abs(neighbor[1] - end[1])  # Heurística
                heapq.heappush(priority_queue, (priority, neighbor, path))
                visited.add(neighbor)

    return [], True
