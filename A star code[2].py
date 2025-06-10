from queue import PriorityQueue
import serial
import time
# Maze grid: 0 = free cell, 1 = obstacle
maze = [
    [0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0],
    [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0]
]

start = (2, 10)  # Starting position (row, col)
goal = (2, 0)   # Goal position (row, col)

def heuristic(a, b):
    # Manhattan distance heuristic
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(maze, start, goal):
    # Priority queue for A* search
    queue = PriorityQueue()
    queue.put((0, start))
    
    # Tracking where we came from and costs
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not queue.empty():
        _, current = queue.get()

        if current == goal:
            break

        # Explore all 4 possible moves: right, down, left, up
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            next_node = (current[0] + dx, current[1] + dy)

            # Check if next_node is within bounds and not an obstacle
            if (0 <= next_node[0] < len(maze)) and (0 <= next_node[1] < len(maze[0])) and maze[next_node[0]][next_node[1]] == 0:
                new_cost = cost_so_far[current] + 1  # Uniform cost: each step costs 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(goal, next_node)
                    queue.put((priority, next_node))
                    came_from[next_node] = current

    return reconstruct_path(came_from, start, goal)

def reconstruct_path(came_from, start, goal):
    # Reconstruct the path by backtracking from the goal
    directions = ['UP', 'RIGHT', 'DOWN', 'LEFT']
    heading = 'UP'  # Assume robot starts facing 'UP'

    # Movement vectors corresponding to directions
    movement = {'UP': (-1, 0), 'DOWN': (1, 0), 'LEFT': (0, -1), 'RIGHT': (0, 1)}

    current = goal
    path = []

    if goal not in came_from:
        print("Error: Goal is unreachable!")
        return []

    # Reconstruct path by backtracking
    steps = []
    while current != start:
        prev = came_from[current]
        if prev is None:  # Safety check
            break

        # Determine the movement direction
        dx, dy = current[0] - prev[0], current[1] - prev[1]
        if dx == -1 and dy == 0:
            steps.append('UP')  # Moving up
        elif dx == 1 and dy == 0:
            steps.append('DOWN')  # Moving down
        elif dx == 0 and dy == -1:
            steps.append('LEFT')  # Moving left
        elif dx == 0 and dy == 1:
            steps.append('RIGHT')  # Moving right
        else:
            print("Error: Unexpected movement encountered.")
            return []

        current = prev

    steps.reverse()  # Reverse steps to get start -> goal

    # Convert movement steps into F, L, R commands
    for step in steps:
        while heading != step:  # Adjust heading
            current_index = directions.index(heading)
            target_index = directions.index(step)

            # Determine if we need to turn left or right
            if (target_index - current_index) % 4 == 1:
                path.append('R')  # Right turn
                heading = directions[(current_index + 1) % 4]
            elif (target_index - current_index) % 4 == 3:
                path.append('L')  # Left turn
                heading = directions[(current_index - 1) % 4]
            else:
                print("Error: Invalid heading adjustment.")
                return []

        path.append('F')  # Move forward

    return path
# Run A* search and reconstruct the path
path = a_star_search(maze, start, goal)

fpath = []
for i,p in enumerate(path):
    if (i in [4,9,13,18,23]):
        continue
    else:
        fpath.append(p)
arduino = serial.Serial('com12', 9600)  # Replace with your port
time.sleep(2)  # Wait for Arduino to reset

for command in fpath:
    arduino.write(command.encode())
    time.sleep(1)  # Delay between commands

# End path transmission
arduino.write(b'S')  # 'S' indicates the end of the path
arduino.close()
