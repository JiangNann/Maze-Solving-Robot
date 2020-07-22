from Algo.fastest_path import *

"""This module defines the Exploration class that handles the exploration algorithm, along with Exceptions used."""
class Exploration:
    """
    This class defines and handles the exploration algorithm.
    """
#    def __init__(self, robot, start_time, is_arrow_scan, exploration_limit=300, time_limit=360):
#        self._robot = robot
#        self._start_time = start_time
#        self.is_arrow_scan = IS_ARROW_SCAN
#        self._exploration_limit = exploration_limit
#        self._time_limit = 100000
#        self._auto_update = True
    def __init__(self, robot, start_time, is_arrow_scan, exploration_limit=300, time_limit=360):
       self._robot = robot
       self._start_time = start_time
       self.is_arrow_scan = IS_ARROW_SCAN
       self._exploration_limit = exploration_limit
       self._time_limit = 100000
       self._auto_update = True

    def _get_nearest_unexplored(self):
        """
        Find the unexplored cell with the shortest Manhattan Distance from the current center of the robot.

        :return: The xy coordinates of the nearest unexplored cell.
        """
        min_dist = NUM_ROWS * NUM_COLS + 1
        nearest = (-1, -1)
        y, x = get_matrix_coords(self._robot.center)
        for i in range(NUM_ROWS):
            for j in range(NUM_COLS):
                if self._robot.discovered_map[i][j] == 2:
                    dist = abs(y - i) + abs(x - j)

                    if dist < min_dist:
                        min_dist = dist
                        nearest = (i, j)

        return nearest[0], nearest[1]

    def _get_unexplored(self):
        unexplored_coors = {}
        y, x = get_matrix_coords(self._robot.center)
        for i in range(NUM_ROWS):
            for j in range(NUM_COLS):
                if self._robot.discovered_map[i][j] == 2:
                    unexplored_coors[(i, j)] = abs(y - i) + abs(x - j)
        return sorted(unexplored_coors.items(), key=lambda kv: kv[1])


    def start(self):
        """
        Simulate exploring a maze with a virtual robot.

        :return: True when exploration is complete and the robot is back in the start zone.
        """
        yield self._robot.mark_robot_standing()  # Mark initial robot space explored

        is_back_at_start = False
        is_leave_start = False
        while True:
            try:
                while not is_back_at_start:
                    updated_cells, is_blind_range_undetected_obstacle = self._robot.get_sensor_readings(self.is_arrow_scan)
                    # call get_sensor_readings to get updated_cells and if need to turn left for possible undetected obstacle
                    yield updated_cells
#                    if is_leave_start and is_back_at_start:

                    # 0, 1, 2
                    # 3, 4, 5
                    # 6, 7, 8

                    # If robot facing == NORTH
                    blind_unexplored_index = get_robot_cells(self._robot.center)[6] - 1

                    if self._robot.facing == SOUTH:
                        blind_unexplored_index = get_robot_cells(self._robot.center)[2] + 1
                    elif self._robot.facing == WEST:
                        blind_unexplored_index = get_robot_cells(self._robot.center)[8] - ROW_LENGTH
                    elif self._robot.facing == EAST:
                        blind_unexplored_index = get_robot_cells(self._robot.center)[0] + ROW_LENGTH

                    # blind_unexplored_index = get_robot_cells(self._robot.cen
                    blind_unexplored_coors = get_matrix_coords(blind_unexplored_index)
                    
                    if blind_unexplored_coors in self._get_unexplored():
                    
                    # if is_blind_range_undetected_obstacle:
                    # turn left for possible undetected obstacle
                        if self._robot.check_free(RIGHT) or self._robot.check_free(FORWARD):
                        # if right or forward is free, the robot will either move right or forward; thus need to turn left for undetected obstacle.
                        # call get_sensor_readings_blind_range to check for the unobserved cells
                            self._robot.turn_robot(LEFT)
                            print('Blind Range Undetected Obstacle for long sensor Observed: Turn left to get sensor reading')
                            yield LEFT, TURN, {}

                            is_complete = False
                            yield is_complete

                            is_back_at_start = False
                            yield is_back_at_start

                            updated_cells = self._robot.get_sensor_readings()
                            # get sensor readings for the blind range
                            # no longer being used
                            
                            yield updated_cells

                            print('Turn right to get back to original track')
                            self._robot.turn_robot(RIGHT)
                            # turn back to original facing
                            yield RIGHT, TURN, {}
                            yield is_complete
                            yield is_back_at_start

                            updated_cells = {}
                            yield updated_cells

                    in_efficiency_limit = self._robot.in_efficiency_limit()
                    # end of if is_blind_range_undetected_obstacle
                    # check if robot is in_efficiency_limit

                    
                    # if not in efficient limit, move to efficient limit in the preference of Right => forward => left
                    # in_efficiency_limit = False
                    
                    if not in_efficiency_limit:

                        if self._robot.check_free(RIGHT):
                            updated_cells = self._robot.move_robot(RIGHT)
                            print('RIGHT Free')
                            yield RIGHT, MOVE, updated_cells
                        elif self._robot.check_free(FORWARD):
                            print('Forward Free')
                            updated_cells = self._robot.move_robot(FORWARD)
                            yield FORWARD, MOVE, updated_cells
                        else:
                            self._robot.turn_robot(LEFT)
                            yield LEFT, TURN, {}
                    else:
                    # if in efficient limit, move in the preference of
                        print('Robot in efficiency limit.... ')
                        if self._robot.check_free(FORWARD):
                            print('Forward Free')
                            updated_cells = self._robot.move_robot(FORWARD)
                            yield FORWARD, MOVE, updated_cells
                        elif self._robot.check_free(RIGHT):
                            updated_cells = self._robot.move_robot(RIGHT)
                            yield RIGHT, MOVE, updated_cells
                        #!!!!!!!!!!!!!!!!!!!!!! to be changed
#                            is_complete = self._robot.is_complete(self._exploration_limit, self._start_time, self._time_limit)
#                            yield is_complete
#
#                            if is_complete:
#                                raise ExploreComplete
#
#                            if self._robot.center == START:
#                                is_back_at_start = True
#                            yield is_back_at_start
#                            if is_back_at_start:
#                                break
#                            updated_cells, is_blind_range_undetected_obstacle = self._robot.get_sensor_readings(self.is_arrow_scan)
#                            yield updated_cells
#
#                            self._robot.turn_robot(BACKWARD)
#                            yield BACKWARD, TURN, {}
                        #!!!!!!!!!!!!!!!!!!!!!!!!
                        else:
                            self._robot.turn_robot(LEFT)
                            yield LEFT, TURN, {}


                    is_complete = self._robot.is_complete(self._exploration_limit, self._start_time, self._time_limit)
                    yield is_complete

                    if is_complete:
                        raise ExploreComplete

                    if self._robot.center != START:
                        is_leave_start = True

                    if self._robot.center == START and is_leave_start and self._robot.get_completion_count() > 100:
                        is_back_at_start = True

                    yield is_back_at_start

                if self._robot.is_complete_after_back_to_start(self._exploration_limit, self._start_time, self._time_limit):
                    raise ExploreComplete
                    # end of while not is_back_at_start loop, if exploration is complete_after_back_to_start=> raise explorecomplete status

                while True:
                    is_complete = self._robot.is_complete(self._exploration_limit, self._start_time, self._time_limit)
                    if is_complete:
                        raise ExploreComplete
                # after robot goes through the first round of exploration and is back to start:
                    try:
                        unexplored_coors = self._get_unexplored()
                        # check for unexplored cells, which is a list ranked from nearest to furthest
                        # get shortest path to nearest unexplored => something we can change
                        for unexplored_coor in unexplored_coors:
                            nearest_unexplored_y, nearest_unexplored_x = unexplored_coor[0]
                            center_y, center_x = get_matrix_coords(self._robot.center)
                            print('nearest_unexplored_y, nearest_unexplored_x: {}'.format((nearest_unexplored_y, nearest_unexplored_x)))

                            print('Finding shortest path moves to nearest unexplored......')
                            moves = get_shortest_path_moves(self._robot,
                                                            (center_y, center_x),
                                                            (nearest_unexplored_y, nearest_unexplored_x))
                            print('Shortest path moaves to nearest unexplored: {}'.format(moves))

                            if not moves:  # Check adjacent cells
                                print('WARNING: Cannot find shortest path moves to nearest unexplored')

                                print('Finding shortest valid path moves to adjacent cells......')
                                robot_cell_index = get_grid_index(nearest_unexplored_y, nearest_unexplored_x)
                                adjacent_cells = get_detectable_cells(robot_cell_index)
                                del adjacent_cells[4]

                                adj_order = [8, 9, 10, 11, 5, 6, 7, 3, 4, 0, 1, 2]
                                adjacent_cells = [adjacent_cells[i] for i in adj_order]

                                moves = get_shortest_valid_path(self._robot,
                                                                self._robot.center, adjacent_cells)

                            if moves:
                                break

                        if not moves:
                            print('WARNING: Cannot find shortest path moves to unexplored')
                            raise PathNotFound

                        for move in moves:
                            updated_cells, is_blind_range_undetected_obstacle = self._robot.get_sensor_readings(self.is_arrow_scan)
                            if updated_cells:
                                is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                        self._time_limit)
                                yield "updated", updated_cells, is_complete

                                if is_complete:
                                    raise ExploreComplete
                                raise CellsUpdated

                            if is_blind_range_undetected_obstacle:
                                self._robot.turn_robot(LEFT)
                                print('-' * 50)
                                print('Blind Range Undetected Obstacle Observed: Turn right to get sensor reading')
                                yield "turned", LEFT, False

                                updated_cells, is_blind_range_undetected_obstacle = self._robot.get_sensor_readings(self.is_arrow_scan)

                                if updated_cells:
                                    is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                            self._time_limit)
                                    yield "updated", updated_cells, is_complete

                                    if is_complete:
                                        raise ExploreComplete
                                    raise CellsUpdated

                                print('Turn Right to get back to original track')
                                self._robot.turn_robot(RIGHT)
                                yield "turned", RIGHT, False

                            updated_cells = self._robot.move_robot(move)
                            yield "moved", move, False

                            if updated_cells:
                                is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                        self._time_limit)
                                yield "updated", updated_cells, is_complete

                                if is_complete:
                                    raise ExploreComplete
                                raise CellsUpdated

                    except CellsUpdated:
                        continue

            except ExploreComplete:
                break
            except PathNotFound:
                yield "invalid", None, None
                break

        # Return to start after completion by taking the shortest path
        center_y, center_x = get_matrix_coords(self._robot.center)
        start_y, start_x = get_matrix_coords(START)

        moves = get_shortest_path_moves(self._robot,
                                        (center_y, center_x), (start_y, start_x), is_give_up=True)

        if not moves:
            return True

        else:
            for move in moves:
                self._robot.move_robot(move)
                yield move

        return True

#   !!!!!
    def start_real(self, sender):
        """
        Explore the maze with the physical robot.

        :param sender: The object that communicates with the RPi.
        :return: True when exploration is complete.
        """
#        become initial_pos of _explore(self) in controller_gui
#       1st yield
        yield self._robot.mark_robot_standing()  # Mark initial robot space explored

        is_back_at_start = False
        is_leave_start = False
        while True:
            try:
                # right-wall-hugging until loop
                while not is_back_at_start:
                    updated_cells, is_blind_range_undetected_obstacle = self._robot.get_sensor_readings(sender, self.is_arrow_scan)
#                    2nd yield
                    enable_print()
                    print("running 2nd yield")
                    yield updated_cells

                    # 0, 1, 2
                    # 3, 4, 5
                    # 6, 7, 8

                    # If robot facing == NORTH
                    blind_unexplored_index = get_robot_cells(self._robot.center)[6] - 1
                    enable_print()
                    print("turn every 3 steps NORTH")
#                    get the index for robot bottom left corner; if it is unexplored, turn left to detect
                    if self._robot.facing == SOUTH:
                        enable_print()
                        print("turn every 3 steps SOUTH")
                        blind_unexplored_index = get_robot_cells(self._robot.center)[2] + 1
                    elif self._robot.facing == WEST:
                        print("turn every 3 steps WEST")
                        blind_unexplored_index = get_robot_cells(self._robot.center)[8] - ROW_LENGTH
                    elif self._robot.facing == EAST:
                        print("turn every 3 steps EAST")
                        blind_unexplored_index = get_robot_cells(self._robot.center)[0] + ROW_LENGTH
                    
#                    enable_print()
#                    print("turn every 3 steps cell index:")
#                    print(blind_unexplored_index)
#                    disable_print()
                    blind_unexplored_coors = get_matrix_coords(blind_unexplored_index)
#                    print("turn every 3 steps cell coords:")
#                    print(blind_unexplored_coors)
#                    print(self._get_unexplored())
#                    if blind_unexplored_coors is unexplored
                    if self._robot.discovered_map[blind_unexplored_coors[0]][blind_unexplored_coors[1]] == 2:
                        enable_print()
                        print("trying to turn left for undetected bottom left corner with index:")
                        print(blind_unexplored_index)
                        disable_print()

                        if self._robot.check_free(RIGHT) or self._robot.check_free(FORWARD):
#                            enable_print()
#                            print("turning left")
#                            print(blind_unexplored_index)
#                            disable_print()
                            self._robot.turn_robot(sender, LEFT, self.is_arrow_scan)
                            print('Bottom left corner Undetected Obstacle Observed: Turn LEFT to get sensor reading')
                            yield LEFT, TURN, {}

                            is_complete = False
                            yield is_complete

                            is_back_at_start = False
                            yield is_back_at_start

                            updated_cells = self._robot.get_sensor_readings(sender)
                            yield updated_cells

                            print('Turn RIGHT to get back to original track')
                            self._robot.turn_robot(sender, RIGHT, self.is_arrow_scan)
                            yield RIGHT, TURN, {}
                            yield is_complete
                            yield is_back_at_start

                            updated_cells = {}
                            yield updated_cells

                    if self._robot.check_free(RIGHT):
                        updated_cells = self._robot.move_robot(sender, RIGHT, self.is_arrow_scan)
                        print('RIGHT Free')
                        yield RIGHT, MOVE, updated_cells
                    elif self._robot.check_free(FORWARD):
                        print('Forward Free')
                        updated_cells = self._robot.move_robot(sender, FORWARD, self.is_arrow_scan)
                        yield FORWARD, MOVE, updated_cells
                    else:
                        self._robot.turn_robot(sender, LEFT, self.is_arrow_scan)
                        yield LEFT, TURN, {}

                    is_complete = self._robot.is_complete(self._exploration_limit, self._start_time, self._time_limit)
                    yield is_complete

                    if is_complete:
                        raise ExploreComplete

                    if self._robot.center != START:
                        is_leave_start = True

                    if self._robot.center == START and is_leave_start and self._robot.get_completion_count() > 100:
                        is_back_at_start = True

                    yield is_back_at_start

                if self._robot.is_complete_after_back_to_start(self._exploration_limit, self._start_time, self._time_limit):
                    raise ExploreComplete

                # Keep finding shortest path to nearest unexplored square
                while True:
                    try:
                        unexplored_coors = self._get_unexplored()
                        for unexplored_coor in unexplored_coors:
                            nearest_unexplored_y, nearest_unexplored_x = unexplored_coor[0]
                            center_y, center_x = get_matrix_coords(self._robot.center)
                            print('nearest_unexplored_y, nearest_unexplored_x: {}'.format((nearest_unexplored_y, nearest_unexplored_x)))

                            print('Finding shortest path moves to nearest unexplored......')
                            moves = get_shortest_path_moves(self._robot,
                                                            (center_y, center_x),
                                                            (nearest_unexplored_y, nearest_unexplored_x))
                            print('Shortest path moves to nearest unexplored: {}'.format(moves))

                            if not moves:  # Check adjacent cells
                                print('WARNING: Cannot find shortest path moves to nearest unexplored')

                                print('Finding shortest valid path moves to adjacent cells......')
                                robot_cell_index = get_grid_index(nearest_unexplored_y, nearest_unexplored_x)
                                adjacent_cells = get_robot_cells(robot_cell_index)
                                del adjacent_cells[4]

                                adj_order = [8, 9, 10, 11, 5, 6, 7, 3, 4, 0, 1, 2]
                                adjacent_cells = [adjacent_cells[i] for i in adj_order]

                                moves = get_shortest_valid_path(self._robot,
                                                                self._robot.center, adjacent_cells)

                            if moves:
                                break

                        if not moves:
                            print('WARNING: Cannot find shortest path moves to unexplored')
                            raise PathNotFound

                        for move in moves:
                            updated_cells, is_blind_range_undetected_obstacle = self._robot.get_sensor_readings(sender, self.is_arrow_scan)
                            if updated_cells:
                                is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                        self._time_limit)
                                yield "updated", updated_cells, is_complete

                                if is_complete:
                                    raise ExploreComplete
                                raise CellsUpdated

                            if is_blind_range_undetected_obstacle:
                                self._robot.turn_robot(sender, LEFT, self.is_arrow_scan)
                                print('-' * 50)
                                print('Blind Range Undetected Obstacle Observed: Turn LEFT to get sensor reading')
                                yield "turned", LEFT, False

                                updated_cells, is_blind_range_undetected_obstacle = self._robot.get_sensor_readings(sender, self.is_arrow_scan)

                                if updated_cells:
                                    is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                            self._time_limit)
                                    yield "updated", updated_cells, is_complete

                                    if is_complete:
                                        raise ExploreComplete
                                    raise CellsUpdated

                                print('Turn RIGHT to get back to original track')
                                self._robot.turn_robot(sender, RIGHT, self.is_arrow_scan)
                                yield "turned", RIGHT, False

                            updated_cells = self._robot.move_robot(sender, move, self.is_arrow_scan)
                            yield "moved", move, False

                            if updated_cells:
                                is_complete = self._robot.is_complete(self._exploration_limit, self._start_time,
                                                        self._time_limit)
                                yield "updated", updated_cells, is_complete

                                if is_complete:
                                    raise ExploreComplete
                                raise CellsUpdated

                    except CellsUpdated:
                        continue

            except ExploreComplete:
                break
            except PathNotFound:
                yield "invalid", None, None
                break


        center_y, center_x = get_matrix_coords(self._robot.center)
        start_y, start_x = get_matrix_coords(START)

        moves = get_shortest_path_moves(self._robot,
                                        (center_y, center_x), (start_y, start_x), is_give_up=True)

        if not moves:
            return True

        else:
            for move in moves:
                self._robot.move_robot(sender, move, self.is_arrow_scan)
                yield move
        return True


class ExploreComplete(Exception):
    """
    This exception is raised when exploration is complete.
    """
    def __init__(self, message="Exploration complete!"):
        print(message)


class CellsUpdated(Exception):
    """
    This exception is raised when there has been an update in the robot's internal map
    """
    pass


class PathNotFound(Exception):
    """
    This exception is raised when a path to the location cannot be found.
    """
    def __init__(self):
        print("Valid path not found!")
