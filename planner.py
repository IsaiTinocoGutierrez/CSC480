import sys
import heapq

#---------------------------------
def parse_world(filename):
    with open(filename, 'r') as f:
        lines = [line.strip() for line in f.readlines() if line.strip()]

    cols = int(lines[0])
    rows = int(lines[1])
    grid = [list(line) for line in lines[2:]]

    if len(grid) != rows or any(len(row) != cols for row in grid):
        raise ValueError("Grid size does not match specified rows and columns.")

    robot_pos = None
    dirty_set = set()

    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == '@':
                if robot_pos is not None:
                    raise ValueError("Multiple robot start positions found.")
                robot_pos = (r, c)
            elif grid[r][c] == '*':
                dirty_set.add((r, c))

    if robot_pos is None:
        raise ValueError("No robot start position '@' found.")

    return cols, rows, grid, robot_pos, dirty_set

#----------------------------------
class State:
    def __init__(self, robot, dirt, path=None, cost=0):
        self.robot = robot            # (row, col)
        self.dirt = frozenset(dirt)   # Set of (row, col)
        self.path = path or []        # List of actions ['E', 'V', ...]
        self.cost = cost              # Integer total cost

    def is_goal(self):
        return len(self.dirt) == 0

    def __eq__(self, other):
        return self.robot == other.robot and self.dirt == other.dirt

    def __hash__(self):
        return hash((self.robot, self.dirt))

    def __lt__(self, other):
        # For priority queue (UCS): lowest cost first
        return self.cost < other.cost
    
#-----------------------------------
def get_successors(state, grid, rows, cols):
    successors = []
    r, c = state.robot
    dirt = set(state.dirt)

    directions = {
        'N': (-1, 0),
        'S': (1, 0),
        'E': (0, 1),
        'W': (0, -1)
    }

    # Try moving in each direction
    for action, (dr, dc) in directions.items():
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] != '#':
            new_robot = (nr, nc)
            successors.append(State(new_robot, dirt, state.path + [action], state.cost + 1))

    # Try vacuuming if current cell is dirty
    if (r, c) in dirt:
        new_dirt = dirt - {(r, c)}
        successors.append(State((r, c), new_dirt, state.path + ['V'], state.cost + 1))

    return successors

#-------------------------------------
def depth_first_search(start_state, grid, rows, cols):
    stack = [start_state]
    visited = set()
    nodes_generated = 0
    nodes_expanded = 0

    while stack:
        state = stack.pop()
        nodes_expanded += 1

        if state.is_goal():
            return state.path, nodes_generated, nodes_expanded

        if state in visited:
            continue
        visited.add(state)

        successors = get_successors(state, grid, rows, cols)
        nodes_generated += len(successors)

        # DFS: push successors onto the stack in reverse order for consistent path ordering
        for succ in reversed(successors):
            stack.append(succ)

    return None, nodes_generated, nodes_expanded

#------------------------------------
def uniform_cost_search(start_state, grid, rows, cols):
    frontier = []
    heapq.heappush(frontier, (start_state.cost, start_state))
    visited = dict()  # Maps (robot, dirt) → lowest cost seen

    nodes_generated = 0
    nodes_expanded = 0

    while frontier:
        cost, state = heapq.heappop(frontier)
        nodes_expanded += 1

        # Check if goal
        if state.is_goal():
            return state.path, nodes_generated, nodes_expanded

        key = (state.robot, state.dirt)
        if key in visited and visited[key] <= state.cost:
            continue
        visited[key] = state.cost

        # Generate successors
        successors = get_successors(state, grid, rows, cols)
        nodes_generated += len(successors)

        for succ in successors:
            heapq.heappush(frontier, (succ.cost, succ))

    return None, nodes_generated, nodes_expanded

#--------------------------------------
if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python3 planner.py [depth-first|uniform-cost] world.txt")
        sys.exit(1)

    algo = sys.argv[1]
    filename = sys.argv[2]

    try:
        cols, rows, grid, robot_pos, dirty_set = parse_world(filename)
    except Exception as e:
        print("Error reading world file:", e)
        sys.exit(1)

    start = State(robot_pos, dirty_set)

    if algo == 'depth-first':
        path, gen, exp = depth_first_search(start, grid, rows, cols)
    elif algo == 'uniform-cost':
        path, gen, exp = uniform_cost_search(start, grid, rows, cols)
    else:
        print("Unknown algorithm:", algo)
        sys.exit(1)

    if path is not None:
        for action in path:
            print(action)
    print(f"{gen} nodes generated")
    print(f"{exp} nodes expanded")
