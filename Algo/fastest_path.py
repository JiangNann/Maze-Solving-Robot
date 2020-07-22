import sys
from Algo.priority_queue import priority_queue
from Utils.utils import *

"""This module defines the fastest path algorithm."""

def is_valid_move(graph, bounded_row, bounded_col):
    """Check to see if the intended location is valid for the robot to move to."""
    row, col = bounded_row + 1, bounded_col + 1
    for (r, c) in [(row - 1, col - 1), (row - 1, col), (row - 1, col + 1),
                   (row, col - 1), (row, col), (row, col + 1),
                   (row + 1, col - 1), (row + 1, col), (row + 1, col + 1)]:
        if 0 <= r <= 19 and 0 <= c <= 14 and graph[r][c] == 1:
            return False

    return True


def turning_cost(before_point, current_point, next_point):
    """Return the cost of turning in a certain direction to encourage straight-line movement."""
    if before_point is None:
        return 1
    else:
        if (before_point[0] == current_point[0] == next_point[0] or
                before_point[1] == current_point[1] == next_point[1]):
            return 1
        else:
            return 3


def find_fastest_path(graph, start_point=(1, 1), goal_point=(18, 13), before_start_point=None):
    """Calculate the fastest path from a starting position to a goal position."""

    # if goal point is at the border, unable to find valid path since robot center can never touch border
    if is_at_border(goal_point[0], goal_point[1]):
        return False
    # go to bottom left corner of the matrix
    bounded_start_point = (start_point[0] - 1, start_point[1] - 1)
    bounded_goal_point = (goal_point[0] - 1, goal_point[1] - 1)
    if before_start_point is None:
        bounded_before_start_point = bounded_start_point
    else:
        bounded_before_start_point = (before_start_point[0] - 1, before_start_point[1] - 1)

    # var to be returned
    results = []
    # x: 1-13; y:1-18, excluding the borders
    # indicate the true borders for robot center
    bounded_graph = [graph[i][1:14] for i in range(1, 19)]
    # indicate the true borders for robot bottom left corner
    #inits
    discovers = [[0 for i in range(13)] for j in range(18)]
    distances_from_start = [[sys.maxsize for i in range(13)] for j in range(18)]
    distances_to_goal = [[0 for i in range(13)] for j in range(18)]
    cost = [[0 for i in range(13)] for j in range(18)]
    before = [[None for i in range(13)] for j in range(18)]
    q = priority_queue()
    # bounded start point which is the bottom left corner is the real start point for calculating distance
    distances_from_start[bounded_start_point[0]][bounded_start_point[1]] = 0
    before[bounded_start_point[0]][bounded_start_point[1]] = bounded_before_start_point
    for row in range(len(bounded_graph)):
        for col in range(len(bounded_graph[row])):
            distances_to_goal[row][col] = abs(row - bounded_goal_point[0]) + abs(col - bounded_goal_point[1])
            cost[row][col] = distances_from_start[row][col] + distances_to_goal[row][col]
            q[(row, col)] = cost[row][col]

    while q:
        (r, c) = q.pop_smallest()
        discovers[r][c] = 1
        for (adj_r, adj_c) in [(r-1, c), (r+1, c), (r, c-1), (r, c+1)]:
            if 0 > adj_r or adj_r > 17 or 0 > adj_c or adj_c > 12:
                continue

            estimated_cost = distances_from_start[r][c] + \
                             distances_to_goal[adj_r][adj_c] + \
                             turning_cost(before[r][c], (r, c), (adj_r, adj_c))
            if (is_valid_move(graph, adj_r, adj_c) and
                    discovers[adj_r][adj_c] != 1 and
                    cost[adj_r][adj_c] > estimated_cost):
                distances_from_start[adj_r][adj_c] = distances_from_start[r][c] + \
                                                     turning_cost(before[r][c], (r, c), (adj_r, adj_c))
                cost[adj_r][adj_c] = estimated_cost
                q.__setitem__((adj_r, adj_c), estimated_cost)
                before[adj_r][adj_c] = (r, c)

    (trace_r, trace_c) = bounded_goal_point
    while (trace_r, trace_c) != bounded_start_point:
        results.append((trace_r + 1, trace_c + 1))
        try:
            # print('Before: {}'.format(before))
            # print('trace_r, trace_c: {}'.format((trace_r, trace_c)))
            (trace_r, trace_c) = before[trace_r][trace_c]
        except TypeError:
            return False

    return results[::-1]


def get_shortest_path_moves(robot, start, goal, before_start_point=None, is_give_up=False):
    """
    Calculate the list of moves needed to make given a list of coordinates.

    :param robot: The robot currently in the maze.
    :param start: The start position
    :param goal: The goal position
    :param before_start_point: The point the robot was at before it moved to the start point
    :param is_give_up: Whether the robot is giving up exploration.
    :return: The list of moves the robot needs to take.
    """
    limited_map = []
    if is_give_up:
        for row in robot.discovered_map:
            limited_map.append([1 if x == 2 else x for x in row])
    #         mark all unexplored cells as obstacle so that we dont go into any possible obstacle
    else:
        limited_map = robot.discovered_map

    if limited_map[start[0]][start[1]]:
         # if starting point is obstacle => no valid path
        return []

    cells = find_fastest_path(graph=limited_map, start_point=start, goal_point=goal,
                              before_start_point=before_start_point)
    # call find_fastest_path to return a list of cells

    print('=' * 50)
    print('Fast Path Cell List from {} to {}: '.format(start, goal))
    if not cells:
        # if find_fastest_path returns empty list, no path found, return empty list
        print('WARNING: No Path Found!')
    else:
        print(cells)

    prev_cell = (start[0], start[1])
    # init
    move_list = []
    # init
    potential_facing = robot.facing
    # init
    potential_center = start

    if not cells:
        return []

    # calculate the series of robot moves given a list of cells to go to
    for cell in cells:
        # we know that each cell in the list is 1 cell away from the next cell

        y_diff = cell[0] - prev_cell[0]
        print('y_diff: {}'.format(y_diff))
        # next cell is 1 cell away to the south of the current position
        if y_diff == -1:
            abs_dir = SOUTH
        # next cell is 1 cell away to the north of the current position
        elif y_diff == 1:
            abs_dir = NORTH
        elif y_diff == 0:
            x_diff = cell[1] - prev_cell[1]
            print('x_diff: {}'.format(x_diff))
            if x_diff == -1:
                # next cell is 1 cell away to the west of the current position
                abs_dir = WEST
            elif x_diff == 1:
                # next cell is 1 cell away to the east of the current position
                abs_dir = EAST
            # Start (Anqi)
            elif x_diff == 0:
                # the next cell is at the current position
                abs_dir = potential_facing
            # End (Anqi)

        to_move = abs_dir - potential_facing
        #potential facing is current facing
        #abs dir is the direction to move towards
        #   0
        # 3   1
        #   2
        # in the case of 0-3, to go to north and currently facing west=> robot move to its right
        # RIGHT = 1  # Turn right and move forward 1 square
        if to_move == -3:
            to_move = RIGHT
        # go to north and currently facing south=> turn backward
        elif to_move == -2:
            to_move = BACKWARD
        # to go to west and currently facing north=> turn left
        elif to_move == 3:
            to_move = LEFT
        #     else move FORWARD

        # add move to list
        move_list.append(to_move)
        
        potential_facing = (potential_facing + to_move) % 4
        #update potential / current facing
        #update potential center:
        if potential_facing == NORTH:
            # add 1 row's index to the previous potential center then convert to coords
            potential_center = get_matrix_coords(get_grid_index(potential_center[0], potential_center[1])
                                                 + ROW_LENGTH)
        elif potential_facing == EAST:
            potential_center = get_matrix_coords(get_grid_index(potential_center[0], potential_center[1]) + 1)
        elif potential_facing == SOUTH:
            potential_center = get_matrix_coords(get_grid_index(potential_center[0], potential_center[1])
                                                 - ROW_LENGTH)
        elif potential_facing == WEST:
            potential_center = get_matrix_coords(get_grid_index(potential_center[0], potential_center[1]) - 1)
        # update prev_cell
        prev_cell = cell

    print('Fast Path Move List from {} to {}: '.format(start, goal))
    print(move_list)

    return move_list


def get_shortest_valid_path(robot, start_cell, goal_cells):
    """
    Find the shortest path to any cell in the list of possible goal cells.

    :param robot: The robot currently in the maze
    :param start_cell: The start cell
    :param goal_cells: The list of possible goal cells.
    :return: The list of moves.
    """
    center_y, center_x = get_matrix_coords(start_cell)
    for target in goal_cells:
        if is_at_border(center_y, center_x):
            continue
        target_y, target_x = get_matrix_coords(target)
        moves = get_shortest_path_moves(robot, (center_y, center_x), (target_y, target_x))
        if moves:
            return moves
    return moves
