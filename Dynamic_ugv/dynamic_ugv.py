import random
import heapq
import time
import matplotlib.pyplot as plt
GRID_SIZE = 30   
def generate_grid(density):
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            if random.random() < density:
                grid[i][j] = 1

    return grid
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_list:
        _, current = heapq.heappop(open_list)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        neighbors = [
            (current[0]+1, current[1]),
            (current[0]-1, current[1]),
            (current[0], current[1]+1),
            (current[0], current[1]-1)
        ]
        for neighbor in neighbors:
            x, y = neighbor
            if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE and grid[x][y] == 0:
                tentative_g = g_score[current] + 1

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score, neighbor))
                    came_from[neighbor] = current
    return None
def add_dynamic_obstacles(grid, current, goal, prob=0.005):
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            # Don't block current position or goal
            if (i, j) == current or (i, j) == goal:
                continue

            # Only add obstacle if cell is free
            if grid[i][j] == 0 and random.random() < prob:
                grid[i][j] = 1
def visualize(grid, path, start, goal):
    display = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            if grid[i][j] == 1:
                display[i][j] = 1  # obstacle
    if path:
        for (x, y) in path:
            display[x][y] = 2  # path

    sx, sy = start
    gx, gy = goal

    display[sx][sy] = 3  # start
    display[gx][gy] = 4  # goal

    plt.imshow(display)
    plt.title("Dynamic UGV Path Planning (A*)")
    plt.show()

def main():
    density = 0.1   # low initial density for success
    grid = generate_grid(density)
    start = (0, 0)
    goal = (GRID_SIZE-1, GRID_SIZE-1)
    grid[start[0]][start[1]] = 0
    grid[goal[0]][goal[1]] = 0
    current = start
    full_path = [current]
    start_time = time.time()

    while current != goal:
        path = astar(grid, current, goal)
        if not path:
            print("\n--- Dynamic Results ---")
            print("Failed to reach goal ❌")
            print(f"Time Taken: {time.time() - start_time:.5f} sec")
            return
        # move one step
        next_step = path[1]
        current = next_step
        full_path.append(current)

        # add small number of dynamic obstacles
        add_dynamic_obstacles(grid, current, goal, prob=0.005)

        # ensure current and goal are always free
        grid[current[0]][current[1]] = 0
        grid[goal[0]][goal[1]] = 0
    end_time = time.time()
    print("\n--- Dynamic Results ---")
    print("Goal Reached ✅")
    print(f"Path Length: {len(full_path)}")
    print(f"Time Taken: {end_time - start_time:.5f} sec")
    visualize(grid, full_path, start, goal)

if __name__ == "__main__":
    main()
