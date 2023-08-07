from typing import Tuple
import heapq
import copy
import sys

class Node:
    def __init__(self, state, action, parent, path_cost, est_cost=None) -> None:
        self.grid = state           # 4x4 puzzle
        self.parent = parent        # parent node
        self.action = action        # action to get to this node
        self.path_cost = path_cost  # path cost to this node
        self.est_cost = est_cost    # estimated cost to goal (the f_value)

    def __lt__(self, node) -> bool:
        return (self.est_cost < node.est_cost)  # for priority queue

class Solver:
    def __init__(self, initial_state, goal_state, weight) -> None:
        self.weight = weight            # weight of heuristic
        self.goal_state = goal_state    # goal state

        node = Node(initial_state, None, None, 0)
        node.est_cost = self.evaluate(node)
        self.initial_state = node       # initial state

        self.frontier = []              # frontier
        self.reached = []               # reached nodes (visited)

        self.actions = ["L", "R", "U", "D"]  # actions

    # check if node is goal state
    def is_goal(self, node) -> bool:
        return node.grid == self.goal_state

    # search for item in grid
    def search_item(self, search_grid, item, position=None) -> Tuple[int, int]:
        if position and item == search_grid[position[0]][position[1]]:
            return position

        for i, row in enumerate(search_grid):
            for j, col in enumerate(row):
                if col == item:
                    return (i, j)

    # heuristic function that returns sum of chessboard distances
    def heuristic(self, state: Node) -> int:
        h = 0
        for i, row in enumerate(state.grid):
            for j, col in enumerate(row):
                x, y = self.search_item(self.goal_state, col, (i, j))
                h += max(abs(x - i), abs(y - j))

        return h

    # get successor state after an action
    def successor(self, node: Node, action) -> Node:
        state = copy.deepcopy(node)

        b_x, b_y = self.search_item(state.grid, 0)
        x, y = b_x, b_y

        if action == "L":
            x, y = (b_x, b_y - 1)
        elif action == "R":
            x, y = (b_x, b_y + 1)
        elif action == "U":
            x, y = (b_x - 1, b_y)
        elif action == "D":
            x, y = (b_x + 1, b_y)
        else:
            raise Exception("Invalid action.")

        if not self.position_valid(state.grid, (x, y)):
            return None

        _state: Node = state
        _state.grid[b_x][b_y] = _state.grid[x][y]
        _state.grid[x][y] = 0

        return Node(_state.grid, action, state, _state.path_cost + 1, self.evaluate(_state))

    # check if position is valid
    def position_valid(self, grid, position) -> bool:
        size = len(grid)
        x, y = position

        if x < 0 or x >= size:
            return False
        elif y < 0 or y >= size:
            return False
        else:
            return True

    # get the value of evaluation funtion f(n) = g(n) + W * h(n)
    def evaluate(self, state: Node) -> float:
        h = self.weight * self.heuristic(state)
        g = state.path_cost

        return g + h

    # solve the puzzle using A* search algorithm
    def solve(self) -> Node or bool:
        heapq.heappush(self.frontier, self.initial_state)
        self.reached.append(self.initial_state)

        while self.frontier:
            node: Node = heapq.heappop(self.frontier)

            # if goal state is reached
            if self.is_goal(node):
                return node

            # apply actions to get successor states
            for action in self.actions:
                _state = self.successor(node, action)

                # _state is None if action does not generate a new node
                if _state == None:
                    continue

                idx = None
                for i, item in enumerate(self.reached):
                    if item.grid == _state.grid:
                        idx = i
                        break

                if idx != None:
                    # if node is already reached and estimated cost
                    # is less than the one in reached
                    if _state.est_cost < self.reached[idx].est_cost:
                        self.reached[idx] = _state
                        heapq.heappush(self.frontier, _state)
                else:
                    self.reached.append(_state)
                    heapq.heappush(self.frontier, _state)

        return False

    # get metrics to write to file
    def metrics(self, goal_node: Node):
        total_nodes = len(self.reached)     # total number of nodes expanded

        depth = -1
        actions = []                        # actions to reach goal state
        fn_vals = []                        # f(n) values of each node in path

        # traversing from goal node to initial state
        while (goal_node != None):
            depth += 1

            if goal_node.action != None:
                actions.append(goal_node.action)

            fn_vals.append(str(goal_node.est_cost))

            goal_node = goal_node.parent

        # reversed because actions. fn_vals are stored in reverse order
        return (depth, total_nodes, list(reversed(actions)), list(reversed(fn_vals)))


def main():
    # read input file
    args = sys.argv
    filename = args[1]

    input_file = open(filename, "r")
    lines = input_file.readlines()

    # get weight
    W = float(lines[0])

    # initial tiles
    initial_tile_pattern = lines[2:6]
    initial_state = []
    for pattern in initial_tile_pattern:
        row = pattern.strip().split(" ")
        row = [int(i) for i in row]

        initial_state.append(row)

    # goal tiles
    goal_state = []
    goal_tile_pattern = lines[7:11]
    for pattern in goal_tile_pattern:
        row = pattern.strip().split(" ")
        row = [int(i) for i in row]

        goal_state.append(row)

    # solver instance
    solver = Solver(initial_state, goal_state, W)
    goal_node = solver.solve()      # goal node after solving the puzzle

    # get metrics
    metrics = solver.metrics(goal_node)

    # write to output file (if output filename is given, use it, otherwise use "output.txt")
    output_file_name = args[2] if len(args) == 3 else "output.txt"
    output_file = open(output_file_name, "w")

    lines = ["".join(initial_tile_pattern), "".join(goal_tile_pattern), str(W), str(
        metrics[0]), str(metrics[1]), " ".join(metrics[2]), " ".join(metrics[3])]

    output_file.writelines("\n".join(lines))

    # close files
    input_file.close()
    output_file.close()

# run main function
if __name__ == "__main__":
    main()
