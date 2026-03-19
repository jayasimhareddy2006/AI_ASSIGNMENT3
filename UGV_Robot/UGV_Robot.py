import random
import heapq
import time
import matplotlib.pyplot as plt

GRID_SIZE = 70
def generate_grid(density):
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            if random.random() < density:
                grid[i][j] = 1  # obstacle
    return grid

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid, start, goal):
    rows, cols = GRID_SIZE, GRID_SIZE

    open_list = []
    heapq.heappush(open_list, (0, start))

    came_from = {}
    g_score = {start: 0}

    visited_nodes = 0
    while open_list:
        _, current = heapq.heappop(open_list)
        visited_nodes += 1

        if current == goal:
            # reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, visited_nodes
        neighbors = [
            (current[0]+1, current[1]),
            (current[0]-1, current[1]),
            (current[0], current[1]+1),
            (current[0], current[1]-1)
        ]
        for neighbor in neighbors:
            x, y = neighbor
            if 0 <= x < rows and 0 <= y < cols and grid[x][y] == 0:
                tentative_g = g_score[current] + 1

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score, neighbor))
                    came_from[neighbor] = current

    return None, visited_nodes

def visualize(grid, path, start, goal):
    display = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            if grid[i][j] == 1:
                display[i][j] = 1  # obstacle
    for (x, y) in path:
        display[x][y] = 2  # path

    sx, sy = start
    gx, gy = goal

    display[sx][sy] = 3  # start
    display[gx][gy] = 4  # goal

    plt.imshow(display)
    plt.title("UGV Path Planning (A*)")
    plt.show()

def main():
    print("Select Obstacle Density:")
    print("1. Low (10%)")
    print("2. Medium (30%)")
    print("3. High (45%)")

    choice = int(input("Enter choice: "))

    if choice == 1:
        density = 0.1
    elif choice == 2:
        density = 0.3
    else:
        density = 0.45

    grid = generate_grid(density)

    start = (0, 0)
    goal = (GRID_SIZE-1, GRID_SIZE-1)

    # Ensure start and goal are not blocked
    grid[start[0]][start[1]] = 0
    grid[goal[0]][goal[1]] = 0

    print("\nRunning A* Algorithm...")

    start_time = time.time()
    path, visited = astar(grid, start, goal)
    end_time = time.time()

    print("\n--- Results ---")

    if path:
        print(f"Path Found ✅")
        print(f"Path Length: {len(path)}")
    else:
        print("No Path Found ❌")

    print(f"Nodes Explored: {visited}")
    print(f"Time Taken: {end_time - start_time:.5f} seconds")

    if path:
        visualize(grid, path, start, goal)


if __name__ == "__main__":
    main()
