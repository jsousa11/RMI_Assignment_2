from collections import deque

def bfs(maze, start, end):
    """
    Retorna um caminho usando BFS.
    """
    neighbors = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # Cima, baixo, esquerda, direita
    queue = deque([(start, [])])  # Nó atual e caminho até ele
    visited = set()
    visited.add(start)

    while queue:
        current, path = queue.popleft()
        path = path + [current]

        if current == end:
            return path

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)

            # Verifica os limites do labirinto
            if 0 <= neighbor[0] < len(maze) and 0 <= neighbor[1] < len(maze[0]):
                # Verifica se é caminhável ('X' representa parede)
                if maze[neighbor[0]][neighbor[1]] != 'X' and neighbor not in visited:
                    queue.append((neighbor, path))
                    visited.add(neighbor)

    return None  # Retorna None se nenhum caminho for encontrado
