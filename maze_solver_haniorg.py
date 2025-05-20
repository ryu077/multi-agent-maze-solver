import pygame
import random
import heapq
from collections import defaultdict

pygame.init()#initialising pygame....

SCREEN_WIDTH, SCREEN_HEIGHT = 600, 600
GRID_ROWS, GRID_COLS = 20, 20
CELL_DIM = SCREEN_WIDTH // GRID_COLS
COLORS = {
    "background": (255, 255, 255),
    "wall": (0, 0, 0),
    "start": (0, 255, 0),
    "goal": (255, 0, 0),
    "path": (200, 200, 200),
    "agents": [(0, 0, 255), (255, 165, 0), (128, 0, 128), (255, 255, 0)],
}

# creating the screen(window)....
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Multi-Agent Pathfinding with Conflict-Based Search")

#gris and agent setup--start and end points 
grid = []
for _ in range(GRID_ROWS):
    row = []
    for _ in range(GRID_COLS):
        row.append(0)  # Initialisin each cell with 0 (empty space)
    grid.append(row)
agent_starts = []
goal_position = None
agent_paths = []

# drawing the grids 
def drawing_grid():
    for row in range(GRID_ROWS):#giving color
        for col in range(GRID_COLS):
            color = COLORS["background"]
            if grid[row][col] == 1:#1 is wall so black color
                color = COLORS["wall"]
            pygame.draw.rect(screen, color, (col * CELL_DIM, row * CELL_DIM, CELL_DIM, CELL_DIM))#creating grid cell --cell dimension
            pygame.draw.rect(screen,COLORS["path"],(col * CELL_DIM, row * CELL_DIM, CELL_DIM, CELL_DIM),1, ) #border

    for idx, path in enumerate(agent_paths):# drawing  paths of  agents in grid
        color = COLORS["agents"][idx % len(COLORS["agents"])] #agentpath ---#list in list --agent_paths = [[(1, 2), (1, 3)....]] this for the agent
        for r, c in path:
            pygame.draw.rect(screen, color, (c * CELL_DIM, r * CELL_DIM, CELL_DIM, CELL_DIM))#giving color to cell

    for start in agent_starts:
        pygame.draw.rect(screen, COLORS["start"], (start[1] * CELL_DIM, start[0] * CELL_DIM, CELL_DIM, CELL_DIM))#(start[1] * CELL_DIM converting to pixel
    if goal_position:
        pygame.draw.rect( screen, COLORS["goal"], (goal_position[1] * CELL_DIM, goal_position[0] * CELL_DIM, CELL_DIM, CELL_DIM))
        
# Main Functions

def random_maze(): # Used to create random mazes with different designs
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            grid[row][col] = 1 if random.random() < 0.25 else 0

def manhattan_distance(a, b):  # This is used to determine the distances between two points on the grid
    #Manhattan Distance=∣x2−x1∣+∣y2−y1∣
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(start, goal, restrictions):   # a star algorithm implementation, used to determine the shortest path
    if not start or not goal:
        return []

    open_set = []# priority queue... 
    heapq.heappush(open_set, (0 + manhattan_distance(start, goal), 0, start, []))# (f(n), g(n), currentnode, pathtocurrent_node)
    visited = set()
    g_scores = {start: 0}

    while open_set:
        _, g, current, path = heapq.heappop(open_set)

        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            return path + [current]

        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]# right, down, left, up

        for dr, dc in directions:#neighbour
            nr, nc = current[0] + dr, current[1] + dc
            next_cell = (nr, nc)

            if 0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS and grid[nr][nc] == 0:#checking neighb is within bounds&&not wall...
                restricted = False
                for restriction in restrictions:
                    if restriction == (next_cell, len(path) + 1):
                        restricted = True
                        break
                if restricted:
                    continue

                new_g = g + 1
                new_f = new_g + manhattan_distance(next_cell, goal)

                if next_cell not in g_scores or new_g < g_scores[next_cell]:
                    g_scores[next_cell] = new_g
                    heapq.heappush(open_set, (new_f, new_g, next_cell, path + [current]))

    return []

def find_conflicts(paths):  # Checks the obstacles that obstructs the path
    timeline = defaultdict(list)
    for idx, path in enumerate(paths):
        for time, pos in enumerate(path):
            timeline[(time, pos)].append(idx)#appending the time and pos 

    conflicts = []
    for (time, pos), agents in timeline.items():
        if len(agents) > 1:#if more than 1 append 
            conflicts.append((time, pos, agents))
    return conflicts

def conflict_based_search():  # Main algorithm (Conflict Based Search), used to find the optimised path wtih no obstacles
    global agent_paths
    agent_paths = [a_star_search(start, goal_position, []) for start in agent_starts]## initial path of all agents --A* 
    constraints = []# list for storing restricted cells and times


    while True:
        conflicts = find_conflicts(agent_paths)
        if not conflicts:
            break

        time, position, agents_in_conflict = conflicts[0]
        agent_to_replan = agents_in_conflict[0]
        constraints.append((position, time))
        agent_paths[agent_to_replan] = a_star_search(agent_starts[agent_to_replan], goal_position, constraints)

random_maze() #random_maze() function declaration

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if pygame.mouse.get_pressed()[0]:#left
            x, y = pygame.mouse.get_pos()
            col, row = x // CELL_DIM, y // CELL_DIM
            if grid[row][col] == 0:
                grid[row][col] = 1

        if pygame.mouse.get_pressed()[2]:#right
            x, y = pygame.mouse.get_pos()
            col, row = x // CELL_DIM, y // CELL_DIM
            grid[row][col] = 0

        if event.type == pygame.KEYDOWN:#key pressed
            x, y = pygame.mouse.get_pos()
            col, row = x // CELL_DIM, y // CELL_DIM

            if event.key == pygame.K_s:
                if len(agent_starts) < len(COLORS["agents"]):
                    agent_starts.append((row, col))

            if event.key == pygame.K_e:
                goal_position = (row, col)

            if event.key == pygame.K_SPACE:
                if goal_position:
                    conflict_based_search()

            if event.key == pygame.K_r:
                grid = [[0 for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
                agent_starts.clear()
                agent_paths.clear()
                goal_position = None
                random_maze()

    screen.fill(COLORS["background"])
    drawing_grid()
    pygame.display.flip()

pygame.quit()
