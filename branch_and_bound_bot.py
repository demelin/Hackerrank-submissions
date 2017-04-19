""" Uses the branch and bound algorithm to estimate the shortest route for the robot to travel 
in order to reach every dirt spot. """
import os


def next_move(posx, posy, dimx, dimy, board):
    """ Main function. """
    rob_pos = (posx, posy)

    def get_direction(curr_pos, next_pos):
        """ Provides directional and cleaning commands to the robot based on its current location 
        and the next dirt spot. """
        if curr_pos == next_pos:
            return 'CLEAN'

        v_dist = next_pos[0] - curr_pos[0]
        h_dist = next_pos[1] - curr_pos[1]

        if h_dist != 0:
            if h_dist < 0:
                return 'LEFT'
            else:
                return 'RIGHT'
        else:
            if v_dist < 0:
                return 'UP'
            else:
                return 'DOWN'

    # First, check if a solution has already been discovered during the previous time step
    found = os.path.isfile('storage.txt')

    if found:
        with open('storage.txt', 'r') as fr:
            path = fr.readline()
            # Get the next goal coordinates from the path string within the storage file
            next_goal = path[0:6]
            rest = path[6:]

        next_coords = (int(next_goal[1]), int(next_goal[4]))
        instruction = get_direction(rob_pos, next_coords)

        with open('storage.txt', 'w') as f:
            if instruction == 'CLEAN':
                f.write(rest)
            else:
                f.write(path)

        print(instruction)
        # Terminate script after submitting the instruction to the robot
        quit()

    # Calculate path, in case this hasn't been done yet
    vertices = [rob_pos]

    # Collect all locations of dirty spots on the board
    for r in range(dimx):
        for c in range(dimy):
            if board[r][c] == 'd':
                vertices.append((r, c))

    # Compute and store distances between each point of interest (dirty spots, plus the initial position of the robot)
    distances = dict()
    for idx1, coord1 in enumerate(vertices):
        for idx2, coord2 in enumerate(vertices):
            if idx1 != idx2 and (idx2, idx1) not in distances.keys():
                distances[(idx1, idx2)] = abs(coord1[0] - coord2[0]) + abs(coord1[1] - coord2[1])

    # Identify the two shortest edges ending at each vertex for the calculation of the lower bound
    least_cost = dict()
    for v in range(len(vertices)):
        edges = [(distances[k], k) for k in distances.keys() if v in k]
        edges.sort()
        # Format: least_cost[vertex]: ((edge1, cost1), (edge2, cost2))
        least_cost[v] = ((edges[0][1], edges[0][0]), (edges[1][1], edges[1][0]))

    # Compute lower bound of the tour cost (i.e. distance) at the root node
    init_bound = 0.5 * sum(v[0][1] + v[1][1] for v in least_cost.values())

    # Initialize tracking variables and containers (robot's starting point is the root node)
    init_path = [0]
    init_max_distance = dimx * dimy
    visited = [True] + [False] * (len(vertices) - 1)

    # Initialize the adjacency matrix
    adj_mat = [[0.0] * len(vertices) for _ in range(len(vertices))]
    # Fill adjacency matrix with edge distances
    for r in range(len(vertices)):
        for c in range(len(vertices)):
            # Diagonals remain set to 0.0
            if r != c:
                key = (r, c) if (r, c) in distances.keys() else (c, r)
                adj_mat[r][c] = distances[key]

    # Build and search a graph for traversing all points of interest
    def branch_and_bound(bound, distance, step, path, max_distance):
        # Check if all points of interest have been visited
        if step == len(vertices):
            # Return the final path and the estimation of final distance
            if distance < max_distance:
                max_distance = distance
                return path, distance, max_distance

        # Build the traversal graph
        for i in range(1, len(vertices)):
            # Check if candidate node is different from last and hasn't yet been visited
            dist_to_i = adj_mat[path[step - 1]][i]
            if dist_to_i != 0 and not visited[i]:
                # Increment completed distance
                temp = bound
                distance += dist_to_i

                # Calculate the first part of the bound update with one of two methods
                if step == 1:
                    # Use the shortest edge of the vertex last visited
                    bound -= (least_cost[path[step - 1]][0][1] + least_cost[i][0][1]) / 2
                else:
                    # Use the second shortest edge of the vertex last visited
                    bound -= (least_cost[path[step - 1]][1][1] + least_cost[i][0][1]) / 2

                # Check if the current bound is lower than the defined limit; if so, extend tree
                if bound + distance < max_distance:
                    path.append(i)
                    visited[i] = True

                    # Recursively build the next levels
                    final_path, final_distance, max_distance = branch_and_bound(bound, distance, step + 1, path,
                                                                                max_distance)

                else:
                    # Else, prune the node and revert all changes
                    distance -= dist_to_i
                    bound = temp
                    visited[i] = False

        return final_path, final_distance, max_distance

    # Obtain the computed estimations of the optimal path and associated distance
    p, d, _ = branch_and_bound(init_bound, 0, 1, init_path, init_max_distance)

    path_coords = [str(vertices[idx]) for idx in p[1:]]
    path_string = ''.join(path_coords)

    # Obtain first direction to the bot
    first_coords = vertices[p[1]]
    instruction = get_direction(rob_pos, first_coords)

    # Store the path in a file
    with open('storage.txt', 'w') as f:
        f.write(path_string)

    print(instruction)
