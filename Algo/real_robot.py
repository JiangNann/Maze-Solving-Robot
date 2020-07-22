import re

from Utils.utils import *
from time import time, sleep

"""This module defines the Robot class that represents the robot in a physical run."""


class Robot:
    """
    This class is a representation of the physical robot.
    """

    def __init__(self, exploration_status, facing, discovered_map, sender):
        """
        Initialize the Robot class.

        :param exploration_status: The map that shows whether each cell is explored or unexplored.
        :param facing: The current facing of the robot. (N/S/E/W)
        :param discovered_map: The map built by the robot that shows whether each cell is an obstacle or not.
        """
        #        #        (x,y):[]
        self.sender = sender
        self.is_arrow_scan = IS_ARROW_SCAN
        self.obstacle_image_dic = {}
        self.blind_range_cells = {}
        self.is_fast_path = False
        self.exploration_status = exploration_status
        self.center = START
        self.facing = facing
        self.discovered_map = discovered_map
        self.permanent_non_obstacle = [[-1 for _ in range(ROW_LENGTH)] for _ in range(COL_LENGTH)]
#        number of time being detected as obstacle
        self.probability_map = [[[0, 0] for _ in range(ROW_LENGTH)] for _ in range(COL_LENGTH)]
        self.arrow_taken_status = [[[0, 0, 0, 0] for _ in range(ROW_LENGTH)] for _ in range(COL_LENGTH)]
        self.arrow_taken_positions = []
        self.arrows = []
        self.arrows_arduino = []
        self.image_info_for_Android = ''
        self.arrows_results = []
        self.move_counts = 1
        self.is_calibration_side_time = False
        self.is_calibration_front_time = False
        self.sensors = [
            #   2 3 4
            # 5       1
            #         0
            #
            {"mount_loc": ES, "facing": EAST, "range": 2, "blind_spot": 0},
            {"mount_loc": NES, "facing": EAST, "range": 2, "blind_spot": 0},
            {"mount_loc": NWS, "facing": NORTH, "range": 2, "blind_spot": 0},
            {"mount_loc": NS, "facing": NORTH, "range": 2, "blind_spot": 0},
            {"mount_loc": NES, "facing": NORTH, "range": 2, "blind_spot": 0},
            {"mount_loc": NWS, "facing": WEST, "range": 6, "blind_spot": 2}
        ]
        self.real_map = []

        regex_str = '^(\d*,){%s}$' % (len(self.sensors))
        self._readings_regex_arduino = re.compile(regex_str)

        regex_str = '[01]{2}'
        self._readings_regex_rpi = re.compile(regex_str)


    def _mark_obstacle(self, cell_index, weight=1):
        """
        Mark a cell as a guaranteed obstacle.
        """
        
        y, x = get_matrix_coords(cell_index)
        if self.permanent_non_obstacle[y][x] == 0:
            return False
            

        self.probability_map[y][x][1] = self.probability_map[y][x][1] + weight
        if self.discovered_map[y][x] == 1:
            return False

        if self.exploration_status[y][x] == 0:
            self.exploration_status[y][x] = 1
            self.discovered_map[y][x] = 1
            if self.is_arrow_scan:
                if (y, x) not in self.obstacle_image_dic.keys():
                    self.obstacle_image_dic[y, x] = []
                    cells = [(x - 1, y + 1), (x, y + 1), (x + 1, y + 1), (x - 1, y), (x, y), (x + 1, y), (x - 1, y - 1),
                             (x, y - 1), (x + 1, y - 1)]

                    for (x1, y1) in cells:
                        if ((y1, x1) in self.obstacle_image_dic.keys()) and (-1 in self.obstacle_image_dic[y1, x1]):
                            self._mark_neigbourhood_obsolete(y, x)
                            return True
            return True

        else:
            # if self.probability_map[y][x][0] == self.probability_map[y][x][1]:
            #     self.exploration_status[y][x] = 0
            #     self.discovered_map[y][x] = 2
            if self.probability_map[y][x][1] >= self.probability_map[y][x][0]:
                self.discovered_map[y][x] = 1
                if self.is_arrow_scan:
                    if (y, x) not in self.obstacle_image_dic.keys():
                        self.obstacle_image_dic[y, x] = []
                        cells = [(x - 1, y + 1), (x, y + 1), (x + 1, y + 1), (x - 1, y), (x, y), (x + 1, y), (x - 1, y - 1),
                                 (x, y - 1), (x + 1, y - 1)]
                        for (x1, y1) in cells:
                            if ((y1, x1) in self.obstacle_image_dic.keys()) and (-1 in self.obstacle_image_dic[y1, x1]):
                                self._mark_neigbourhood_obsolete(y, x)
                                return True
                return True

            else:
                return False

    def _mark_permanent_obstacle(self, cell_index):
        """
        Mark a cell as a guaranteed obstacle.
        """
        y, x = get_matrix_coords(cell_index)
        if self.permanent_non_obstacle[y][x] == 0:
            return False

        self.probability_map[y][x][0]=0
        if self.exploration_status[y][x] != 1:
            self.exploration_status[y][x] = 1
            self.discovered_map[y][x] = 1
            if self.is_arrow_scan:
                if (y, x) not in self.obstacle_image_dic.keys():
                    self.obstacle_image_dic[y, x] = []
                    cells = [(x - 1, y + 1), (x, y + 1), (x + 1, y + 1), (x - 1, y), (x, y), (x + 1, y), (x - 1, y - 1),
                             (x, y - 1), (x + 1, y - 1)]
                    for (x1, y1) in cells:
                        if ((y1, x1) in self.obstacle_image_dic.keys()) and (-1 in self.obstacle_image_dic[y1, x1]):
                            self._mark_neigbourhood_obsolete(y, x)
                            return True
            return True
        elif self.discovered_map[y][x] != 1:
            self.discovered_map[y][x] = 1
            if self.is_arrow_scan:
                if (y, x) not in self.obstacle_image_dic.keys():
                    self.obstacle_image_dic[y, x] = []
                    cells = [(x - 1, y + 1), (x, y + 1), (x + 1, y + 1), (x - 1, y), (x, y), (x + 1, y), (x - 1, y - 1),
                             (x, y - 1), (x + 1, y - 1)]
                    for (x1, y1) in cells:
                        if ((y1, x1) in self.obstacle_image_dic.keys()) and (-1 in self.obstacle_image_dic[y1, x1]):
                            self._mark_neigbourhood_obsolete(y, x)
                            return True
            return True
        else:
            return False


    def _mark_neigbourhood_obsolete(self, y, x):
        # recursion
        # x-1y+1, xy+1, x+1y+1
        # x-1,y, xy, x+1,y
        # x-1y-1, xy-1, x+1y-1

        #       if the cell is not marked as obsolete, mark it
        if (y, x) not in self.obstacle_image_dic.keys():
            return
        if -1 in self.obstacle_image_dic[y, x]:
            return
        else:
            self.obstacle_image_dic[y, x].append(-1)
            print("marking obstacle obsolete: ")
            print(x,y)
            cells = [(x - 1, y + 1), (x, y + 1), (x + 1, y + 1), (x - 1, y), (x + 1, y), (x - 1, y - 1), (x, y - 1),
                     (x + 1, y - 1)]
            #        mark its neighbours
            for (x1, y1) in cells:
                self._mark_neigbourhood_obsolete(y1, x1)


    def _mark_non_obstacle(self, cell,weight = 1):
        y, x = get_matrix_coords(cell)
        
        if self.permanent_non_obstacle[y][x] == 0:
            return False
        self.probability_map[y][x][0] = self.probability_map[y][x][0] + weight
        if self.discovered_map[y][x] == 0:
            return False
        #             first time it is being explored
        #             mark it as obstacle & update
        elif self.exploration_status[y][x] == 0:
            self.exploration_status[y][x] = 1
            self.discovered_map[y][x] = 0
            return True
            #        has been marked as non-obstacle
        else:
#            if obstacle outweigh non-obstacle, mark it as obstacle
#             if self.probability_map[y][x][0] == self.probability_map[y][x][1]:
#                 self.exploration_status[y][x] = 0
#                 self.discovered_map[y][x] = 2
            if self.probability_map[y][x][0] >= self.probability_map[y][x][1]:
                self.discovered_map[y][x] = 0
                return True
        #if non-obstacle outweigh obstacle, do nothing and return
            else:
                return False


    def _mark_permanent(self, cell):
        """
        Mark a cell as a guaranteed non-obstacle.

        Called when the robot walks over a cell.

        :param cell: The number of the cell being marked.
        :return: True if success. No definition of failure provided, however it is easy to add if required.
        """
        y, x = get_matrix_coords(cell)

        if self.permanent_non_obstacle[y][x] == 0:
            return False
        else:
            #            if self.exploration_status[y][x] != 1:
            # mark as explored
            self.exploration_status[y][x] = 1
            # if self.discovered_map[y][x] != 0:
            self.discovered_map[y][x] = 0
            #            self._mark_non_obstacle(cell)
            self.permanent_non_obstacle[y][x] = 0
            return True
        return False

#    def _mark_arrow_taken(self, y, x, camera_facing):
#        """
#        Mark the face a cell having its picture taken by the arrow recognizer.
#
#        :param y: The y-coordinate of the cell to be marked.
#        :param x: The x-coordinate of the cell to be marked.
#        :param facing: The facing of the robot when the photo was taken.
#        :return: True if success. No definition of failure provided, however it is easy to add if required.
#        """
#
#        self.arrow_taken_status[y][x][camera_facing] = 1
##        print('Mark Arrow Taken @ {}'.format((x, y, DIRECTIONS[camera_facing])))
#
#        return True


#    def _mark_arrows(self, position, result):
#        #    result is the arrow detection result returned by rpi
#        y, x, facing = tuple([int(_) for _ in position.split(',')])
##        print('Recognizing Image taken with robot position @ {}'.format((x, y, DIRECTIONS[facing])))
#        discovered_map = self.discovered_map
#
#        camera_facing = (facing + CAMERA_FACING) % 4
#        #        e.g. if camera faces north, then arrow_facing is south
#        arrow_direction = (camera_facing + 2) % 4
#
#        try:
#            #           IMPT!!
#            #           algo might need to calculate the distance from camera to obstacle
#            distance = 2
#            #            camera can detect 2 cells in front of the center left and center of the robot
#            if camera_facing == WEST:
#                #            x,y are the coordinates of robot center
#                #            new_x = x-2 i.e. camera can detect 2 cells to the left of the robot
#                new_x = x - distance
#                if new_x < 0:
#                    raise IndexError
#                index = 0
#
#                for i, j in [(new_x, y - 1), (new_x, y)]:
#                    #                2 iterations, new_x,y-1 & new_x,y
#                    #                new_x,y-1: index0; new_x,y: index1
#                    image_info_for_Android = 0
##                    print('Check Arrow @ {}'.format((i, j, DIRECTIONS[arrow_direction])))
#                    if discovered_map[j][i] == 1:
#                        #                    IMPT!!!!
#                        #                    result is a 2 index array; index initialised to 0 and increases from 0 to 1
#                        #                    result = "20" which means first index has image 2 detected whereas second one has no image detected
#                        #                    might need to adopt HEX since more than 10 pic exist
#                        if result[index] == '1':
#                            #                        IMPT!!! MIGHT NEED TO CHANGE TO result[index] != 0
#                            #                         i.e. if nothing detected: 0; if anything detected: index of that pic
#                            self.arrows.append((j, i, arrow_direction))
#                            #                            this returns the location & facing (arrow_direction) of the arrow
#                            self.arrows_arduino.append(','.join([str(i), str(19 - j), str(arrow_direction)]))
#                            self.image_info_for_Android = ','.join(['ID#', str(i), str(19 - j)])
#                            # IMPT str(arrow_direction) is not needed. Need to add image ID
#                            # image_info_for_Android is of type str
#                            #                            IMPT!!!!! 3rd item NEED TO CHANGE FROM str(arrow_direction) TO result[index] which is the image id
##                            print('Detected Arrow @ {}'.format((i, j, DIRECTIONS[arrow_direction])))
#                    index += 1
#            elif camera_facing == NORTH:
#                new_y = y + distance
#                if new_y > 19:
#                    raise IndexError
#                index = 0
#                for i, j in [(x - 1, new_y), (x, new_y)]:
##                    print('Check Arrow @ {}'.format((i, j, DIRECTIONS[arrow_direction])))
#                    if discovered_map[j][i] == 1:
#                        if result[index] == '1':
#                            self.arrows.append((j, i, arrow_direction))
#                            self.arrows_arduino.append(','.join([str(i), str(19 - j), str(arrow_direction)]))
#                            print('Detected Arrow @ {}'.format((i, j, DIRECTIONS[arrow_direction])))
#                    index += 1
#            elif camera_facing == EAST:
#                new_x = x + distance
#                if new_x > 14:
#                    raise IndexError
#                index = 0
#                for i, j in [(new_x, y + 1), (new_x, y)]:
##                    print('Check Arrow @ {}'.format((i, j, DIRECTIONS[arrow_direction])))
#                    if discovered_map[j][i] == 1:
#                        if result[index] == '1':
#                            self.arrows.append((j, i, arrow_direction))
#                            self.arrows_arduino.append(','.join([str(i), str(19 - j), str(arrow_direction)]))
##                            print('Detected Arrow @ {}'.format((i, j, DIRECTIONS[arrow_direction])))
#                    index += 1
#            elif camera_facing == SOUTH:
#                new_y = y - distance
#                if new_y < 0:
#                    raise IndexError
#                index = 0
#                for i, j in [(x + 1, new_y), (x, new_y)]:
##                    print('Check Arrow @ {}'.format((i, j, DIRECTIONS[arrow_direction])))
#                    if discovered_map[j][i] == 1:
#                        if result[index] == '1':
#                            self.arrows.append((j, i, arrow_direction))
#                            self.arrows_arduino.append(','.join([str(i), str(19 - j), str(arrow_direction)]))
##                            print('Detected Arrow @ {}'.format((i, j, DIRECTIONS[arrow_direction])))
#                    index += 1
#        except IndexError:
#            return


#    def in_efficiency_limit(self):
#        """
#        Check if the robot is one grid before the maximum limit of the maze.
#
#        The method is called to check if the robot has just moved past a 1-cell-width obstacle along
#        the wall of the maze.
#
#        :return: True if robot is in the limit, false otherwise.
#        """
#
#        if (self.center in E_LIMITS[SOUTH] and self.facing == EAST) \
#                or (self.center in E_LIMITS[EAST] and self.facing == NORTH) \
#                or (self.center in E_LIMITS[NORTH] and self.facing == WEST) \
#                or (self.center in E_LIMITS[WEST] and self.facing == SOUTH):
#            # if (self.center in E_LIMITS[NORTH] and self.facing == EAST) \
#            #         or (self.center in E_LIMITS[EAST] and self.facing == SOUTH) \
#            #         or (self.center in E_LIMITS[SOUTH] and self.facing == WEST) \
#            #         or (self.center in E_LIMITS[WEST] and self.facing == NORTH):
#            return True
#        return False


    def mark_robot_standing(self):
        """
        Mark the area the robot is standing on as explored and guaranteed non-obstacles.

        :return: The cells that were updated.
        """
        robot_cells = get_robot_cells(self.center)
        updated_cells = {}
        for cell in robot_cells:
            # only when the cell has not been marked as permanent non-obstacle before
            # then mark it as permanent non obstacle & include in updated_cells
            if self._mark_permanent(cell):
                updated_cells[cell] = 0

        return updated_cells


    def get_completion_count(self):
        """
        Calculate how many of the cells the robot has explored in percentage.

        :return: The count of the cells the robot has explored.
        """
        count = 0
        for row in self.exploration_status:
            for i in row:
                count += i

        return count


    def is_complete(self, explore_limit, start_time, time_limit):
        """
        Check if the exploration is complete based on the exploration limit and the time limit.

        :param explore_limit: The percentage of the maze up to which the robot is allowed to explore.
        :param start_time: The start time of the exploration
        :param time_limit: The maximum time that the robot was allowed to explore until.
        :return: True if the exploration should be stopped, false otherwise.
        """
        return self.get_completion_count() >= 300 \
               or float(time() - start_time >= 360)


    def is_complete_after_back_to_start(self, explore_limit, start_time, time_limit):
        """
        Check if the exploration is complete based on the exploration limit and the time limit.

        :param explore_limit: The percentage of the maze up to which the robot is allowed to explore.
        :param start_time: The start time of the exploration
        :param time_limit: The maximum time that the robot was allowed to explore until.
        :return: True if the exploration should be stopped, false otherwise.
        """
        return self.get_completion_count() >= 300 or float(time() - start_time >= 350)
    
    def is_complete_before_back_to_start(self, explore_limit, start_time, time_limit):
        """
        Check if the exploration is complete based on the exploration limit and the time limit.

        :param explore_limit: The percentage of the maze up to which the robot is allowed to explore.
        :param start_time: The start time of the exploration
        :param time_limit: The maximum time that the robot was allowed to explore until.
        :return: True if the exploration should be stopped, false otherwise.
        """
#        count = self.get_completion_count()
#        print("exploration completion count: ")
        
        # print("exploration completion count: "
        return self.get_completion_count() >= 300 or float(time() - start_time >= 340)

    def turn_robot(self, sender, direction):
        """
        Turn the robot in a chosen direction.

        :param sender: The object that communicates with the RPi
        :param direction: The direction to turn (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: Nothing. Stops the method if the direction is FORWARD to save time as the robot does not need to turn.
        """

        if direction == FORWARD:
            return

        sender.send_arduino(get_arduino_cmd(direction))
        #        sender.send_arduino('C')
        self.facing = (self.facing + direction) % 4
        self.move_counts += TURNING_STEP

        #        sender.wait_arduino(ARDUIMO_MOVED)
        sender.wait_arduino
        sender.wait_arduino(ARDUIMO_MOVED)

        #        if self.is_calibrate_side_possible():
        #            self.calibrate_side(sender)

#        if is_arrow_scan and not self.is_fast_path:
#            self.check_arrow(sender)
    def turn_robot_algoo(self, direction):
        """
        Turn the robot in a chosen direction.

        :param sender: The object that communicates with the RPi
        :param direction: The direction to turn (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: Nothing. Stops the method if the direction is FORWARD to save time as the robot does not need to turn.
        """

        if direction == FORWARD:
            return
        self.facing = (self.facing + direction) % 4

#        sender.send_arduino(get_arduino_cmd(direction))
        #        sender.send_arduino('C')
        
#        self.move_counts += TURNING_STEP

        #        sender.wait_arduino(ARDUIMO_MOVED)
#        sender.wait_arduino
#        sender.wait_arduino(ARDUIMO_MOVED)
    def move_robot_algoo(self,direction):
        if self.facing == NORTH:
            self.center += ROW_LENGTH
        elif self.facing == EAST:
            self.center += 1
        elif self.facing == SOUTH:
            self.center -= ROW_LENGTH
        elif self.facing == WEST:
            self.center -= 1
        """
        Move the robot 1 step in a chosen direction.

        Turn the robot towards the chosen direction then move one step forward. Assume step is not obstacle.

        :param sender: The object that communicates with the RPi.
        :param direction: The direction to move (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: Any cells that the robot has stepped on that it had not yet before.
        """
        #        if robot is to move forward, then do nothing in this func
#        self.turn_robot_algoo(direction)

#        sender.send_arduino(get_arduino_cmd(FORWARD))
        #        sender.send_arduino('C')

        #        self.move_counts += STRAIGHT_STEP
#        updated_cells = self.mark_robot_standing()

#        moved = sender.wait_arduino(ARDUIMO_MOVED)
#        moved = sender.wait_arduino(self._readings_regex_arduino, is_regex=False)
#        got a sensor reading instead of M
#        if moved != 'M':
##            self.get_sensor_readings((self.sender, self.is_arrow_scan)
#            return moved
#        else:
        
#            self.mark_robot_standing()
#            return 'M'
            
    def move_robot(self, sender, direction):
        """
        Move the robot 1 step in a chosen direction.

        Turn the robot towards the chosen direction then move one step forward. Assume step is not obstacle.

        :param sender: The object that communicates with the RPi.
        :param direction: The direction to move (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: Any cells that the robot has stepped on that it had not yet before.
        """
        #        if robot is to move forward, then do nothing in this func
        self.turn_robot(sender, direction)

        sender.send_arduino(get_arduino_cmd(FORWARD))
        #        sender.send_arduino('C')

        #        self.move_counts += STRAIGHT_STEP
#        updated_cells = self.mark_robot_standing()

#        moved = sender.wait_arduino(ARDUIMO_MOVED)
        moved = sender.wait_arduino(self._readings_regex_arduino, is_regex=False)
#        got a sensor reading instead of M
        if moved != 'M':
#            self.get_sensor_readings((self.sender, self.is_arrow_scan)
            return moved
        else:
            if self.facing == NORTH:
                self.center += ROW_LENGTH
            elif self.facing == EAST:
                self.center += 1
            elif self.facing == SOUTH:
                self.center -= ROW_LENGTH
            elif self.facing == WEST:
                self.center -= 1
#            self.mark_robot_standing()
            return 'M'
            
        #        sender.wait_arduino(ARDUIMO_MOVED)

#        if is_arrow_scan and not self.is_fast_path:
#            self.check_arrow(sender)

#        return updated_cells


    def calibrate_side(self, sender):
        return
        

    def move_robot_algo(self, direction):
        """
        Turn the algo robot in a chosen direction or forward it

        :param direction: The direction to turn (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: Nothing.
        """
        if direction == FORWARD:
            if self.facing == NORTH:
                self.center += ROW_LENGTH
            elif self.facing == EAST:
                self.center += 1
            elif self.facing == SOUTH:
                self.center -= ROW_LENGTH
            elif self.facing == WEST:
                self.center -= 1
        else:
            self.facing = (self.facing + direction) % 4


    def check_free(self, direction):
        """
        Check if the adjacent cells in the chosen direction have obstacles.

        :param direction: he direction to check (FORWARD, LEFT, RIGHT, BACKWARD)
        :return: true if the robot is able to take one step in that direction, false otherwise
        """

#        print('Checking Free towards {}'.format(MOVEMENTS[direction]))

        true_bearing = (self.facing + direction) % 4
        robot_cells = get_robot_cells(self.center)

        try:
            if true_bearing == NORTH:
                y, x = get_matrix_coords(robot_cells[0])
                y += 1
#                print('Cell to check : {}'.format((x, y, x + 1, y, x + 2, y)))
                if y < 0 or x < 0:
                    raise IndexError
                is_free = not (self.discovered_map[y][x] == 1 or self.discovered_map[y][x + 1] == 1
                               or self.discovered_map[y][x + 2] == 1)
#                print('North: ' + str(is_free))
                return is_free
            elif true_bearing == EAST:
                y, x = get_matrix_coords(robot_cells[2])
                x += 1
#                print('Cell to check : {}'.format((x, y, x, y - 1, x, y - 2)))
                if y < 2 or x < 0:
                    raise IndexError
                is_free = not (self.discovered_map[y][x] == 1 or self.discovered_map[y - 1][x] == 1
                               or self.discovered_map[y - 2][x] == 1)
                print('East: ' + str(is_free))
                return is_free
            elif true_bearing == SOUTH:
                y, x = get_matrix_coords(robot_cells[6])
                y -= 1
                print('Cell to check : {}'.format((x, y, x + 1, y, x + 2, y)))
                if y < 0 or x < 0:
                    raise IndexError
                is_free = not (self.discovered_map[y][x] == 1 or self.discovered_map[y][x + 1] == 1
                               or self.discovered_map[y][x + 2] == 1)
                print('South: ' + str(is_free))
                return is_free
            elif true_bearing == WEST:
                y, x = get_matrix_coords(robot_cells[0])
                x -= 1
                print('Cell to check : {}'.format((x, y, x, y - 1, y - 2, x)))
                if y < 2 or x < 0:
                    raise IndexError
                is_free = not (self.discovered_map[y][x] == 1 or self.discovered_map[y - 1][x] == 1
                               or self.discovered_map[y - 2][x] == 1)
                print('West: ' + str(is_free))
                return is_free
        except IndexError:
            print('ie')
            return False


    def robot_surround_status(self):
        print('Getting cell status surrounding robot...')

        y, x = get_matrix_coords(self.center)
        discovered_map = self.discovered_map
        facing = self.facing

        print('Robot Position @ {}'.format((x, y, facing)))
        surround_status = {NORTH: [], EAST: [], SOUTH: [], WEST: []}

        for i in [x - 1, x, x + 1]:
            if (y + 2) > 19:
                surround_status[NORTH].append(1)
            else:
                surround_status[NORTH].append(discovered_map[y + 2][i])

        for i in [y + 1, y, y - 1]:
            if (x + 2) > 14:
                surround_status[EAST].append(1)
            else:
                surround_status[EAST].append(discovered_map[i][x + 2])

        for i in [x + 1, x, x - 1]:
            if (y - 2) < 0:
                surround_status[SOUTH].append(1)
            else:
                surround_status[SOUTH].append(discovered_map[y - 2][i])

        for i in [y - 1, y, y + 1]:
            if (x - 2) < 0:
                surround_status[WEST].append(1)
            else:
                surround_status[WEST].append(discovered_map[i][x - 2])

        return {(direction - facing) % 4: value for direction, value in surround_status.items()}


    #    def is_calibrate_side_possible(self):
    #        self.get_robot_cells(self.center):
    #             x  x
    #        # 0, 1, 2
    #        # 3, 4, 5 x
    #        # 6, 7, 8 x
    #
    #        check position
    #        self._robot.get_robot_cells()
    #        return False
    #

    
    def is_arrow_possible(self):
        """
        # Camera put on the west of the robot to detect the left and middle image

           # # #
        I2 # # #
        I1 # # #

        Check if it is possible to have arrows in the chosen direction.

        The method also takes into consideration obstacles that have already been scanned for arrows. Will only return
        true if there are faces that have not been scanned that are facing the robot.

        :return: True if there are unscanned faces of obstacles in the path of the RPi camera, false otherwise.
        """
        y, x = get_matrix_coords(self.center)
        discovered_map = self.discovered_map
        arrow_taken_status = self.arrow_taken_status
        facing = self.facing
        camera_facing = (facing + CAMERA_FACING) % 4

        flag = False

        try:
            distance = 2
            if camera_facing == WEST:
                new_x = x - distance
                if new_x < 0:
                    raise IndexError
                for i, j in [(new_x, y - 1), (new_x, y)]:
                    print('Checking arrow @ %s,%s' % (i, j))
                    #                    if there is an obstacle & arrow_taken_status is 0 i.e. havent checked for arrow
                    if discovered_map[j][i] == 1 and not arrow_taken_status[j][i][camera_facing]:
                        flag = True
                if flag:
                    for i, j in [(new_x, y - 1), (new_x, y)]:
                        self._mark_arrow_taken(j, i, camera_facing)
                return flag
            elif camera_facing == NORTH:
                new_y = y + distance
                if new_y > 19:
                    raise IndexError
                for i, j in [(x - 1, new_y), (x, new_y)]:
                    print('Checking arrow @ %s,%s' % (i, j))
                    if discovered_map[j][i] == 1 and not arrow_taken_status[j][i][camera_facing]:
                        flag = True
                if flag:
                    for i, j in [(x - 1, new_y), (x, new_y)]:
                        self._mark_arrow_taken(j, i, camera_facing)
                return flag
            elif camera_facing == EAST:
                new_x = x + distance
                if new_x > 14:
                    raise IndexError
                for i, j in [(new_x, y + 1), (new_x, y)]:
                    print('Checking arrow @ %s,%s' % (i, j))
                    if discovered_map[j][i] == 1 and not arrow_taken_status[j][i][camera_facing]:
                        flag = True
                if flag:
                    for i, j in [(new_x, y + 1), (new_x, y)]:
                        self._mark_arrow_taken(j, i, camera_facing)
                return flag
            elif camera_facing == SOUTH:
                new_y = y - distance
                if new_y < 0:
                    raise IndexError
                for i, j in [(x + 1, new_y), (x, new_y)]:
                    print('Checking arrow @ %s,%s' % (i, j))
                    if discovered_map[j][i] == 1 and not arrow_taken_status[j][i][camera_facing]:
                        flag = True
                if flag:
                    for i, j in [(x + 1, new_y), (x, new_y)]:
                        self._mark_arrow_taken(j, i, camera_facing)
                return flag

        except IndexError:
            return False

        # def _discard_block():
        #
        #


#    def recognize_image_one_cell(self, sender, position):
##        print("Asking RPI for image from 1 cell away")
#        sender.send_rpi('1')
#        result = sender.wait_rpi()
#        if result != '0':
#            self._mark_arrows(position, result)
#            self._discard_block()


    # def recognize_image_two_cells(self, sender):


    #


    #    def check_arrow(self, sender):
    #        """
    #        Send the RPi a message to take a picture to check for arrows.
    #
    #        Check if there are potential unscanned arrows in the field of view of the RPi camera.
    #
    #        :param sender: The object that communicates with the RPi.
    #        :return: N/A
    #        """
    ##        get robot position
    #        y, x = get_matrix_coords(self.center)
    #
    #        if self.is_image_possible_right():
    #            take_photo(sender,1)
    #            print('Arrow Possible @ Robot Position: {}'.format((x, y, DIRECTIONS[self.facing])))
    #            position = '%s,%s,%s' % (y, x, self.facing)
    #
    #        if self.is_image_possible_front():
    #
    #        if self.is_image_possible_left():
    #        else:
    #            print('Arrow Not Possible @ Robot Position: {}'.format((x, y, DIRECTIONS[self.facing])))

    def get_sensor_readings(self, sender, is_arrow_scan=False, readings = None):
        """
        Send a message to the Arduino to take sensor readings.

        The sensors are iterated through and the number of times a cell is detected as an obstacle is added to its
        running count. The total number of scans the cell has received is added to its running count of scans.

        The counts are weighted by distance, with the weight halving for every unit further away from the robot that
        the reading is taken.

        :param sender: The object that communicates with the RPi
        :return: The updated cell values and indexes.
        """
        if readings == None:
            sender.send_arduino(ARDUINO_SENSOR)
            readings = sender.wait_arduino(self._readings_regex_arduino, is_regex=True)
        
        readings = readings.split(',')
        del readings[-1]
#        print("RECEIVED READING WAITING FOR ARM")
        #        sender.wait_arduino(ARDUIMO_MOVED)
        #        print("RECEIVED ARM")

        readings = [int(x) for x in readings]

        robot_cells = get_robot_cells(self.center)
        sensors = self.sensors[:]
        sensor_index = sensors.index
        updated_cells = {}
        is_blind_range_undetected_obstacle = False

        for sensor in sensors:
            true_facing = (sensor["facing"] + self.facing) % 4

            if sensor["mount_loc"] != CS:
                offset = self.facing * 2
                true_mounting = (sensor["mount_loc"] + offset) % 8
            else:
                true_mounting = CS

            if true_mounting == NWS:
                origin = robot_cells[0]
            elif true_mounting == NS:
                origin = robot_cells[1]
            elif true_mounting == NES:
                origin = robot_cells[2]
            elif true_mounting == WS:
                origin = robot_cells[3]
            elif true_mounting == ES:
                origin = robot_cells[5]
            elif true_mounting == SWS:
                origin = robot_cells[6]
            elif true_mounting == SS:
                origin = robot_cells[7]
            elif true_mounting == SES:
                origin = robot_cells[8]
            elif true_mounting == CS:
                origin = robot_cells[4]

            y, x = get_matrix_coords(origin)
            cover_range = list(range(1, sensor["range"] + 1))
            read_range = list(range(sensor["blind_spot"] + 1, sensor["range"] + 1))
            blind_range = list(range(1, sensor["blind_spot"] + 1))
            blind_range_no_obstacle = True
            no_obstacle_long_sensor = False
            reading = readings[sensor_index(sensor)]
#            print('Sensor', sensor_index(sensor))

            #            weight = 4

            #                ---------------------------------------------------------------------------------------------------
            #           For long sensor: obstacle either 1 / 2 cells away:
            if sensor_index(sensor) == LONG_SENSOR_INDEX and (reading == 1 or reading == 2):
#                print("sensor_index(sensor) == LONG_SENSOR_INDEX and (reading == 1 or reading == 2)")
                if true_facing == NORTH:
                    to_explore1 = (y + 1, x)
                    to_explore2 = (y + 2, x)
                elif true_facing == EAST:
                    to_explore1 = (y, x + 1)
                    to_explore2 = (y, x + 2)
                elif true_facing == SOUTH:
                    to_explore1 = (y - 1, x)
                    to_explore2 = (y - 2, x)
                elif true_facing == WEST:
                    to_explore1 = (y, x - 1)
                    to_explore2 = (y, x - 2)
                try:
                    cell_index1 = get_grid_index(to_explore1[0], to_explore1[1])

                    #                   if both out of index, do nothing
                    if self.discovered_map[to_explore1[0]][to_explore1[1]] == 2 and self.discovered_map[to_explore2[0]][
                        to_explore2[1]] == 2:
                        print("Cannot decide")
                    elif to_explore1[0] < 0 or to_explore1[0] > 19 or to_explore1[1] < 0 or to_explore1[1] > 14:
                        #                        enable_print()
#                        print('ie: 1st&2nd cell away from long range sensor')
                        raise IndexError

                    #                   if only 2nd cell out of index=> 1st cell is obstacle
                    #                    elif to_explore2[0] not in range(COL_LENGTH) or to_explore2[1] not in range(ROW_LENGTH):
                    elif to_explore2[0] < 0 or to_explore2[0] > 19 or to_explore2[1] < 0 or to_explore2[1] > 14:
                        #                        enable_print()
#                        print('ie 2nd cell away from long range sensor')
                        if self.discovered_map[to_explore1[0]][to_explore1[1]] == 0:
                            updated = self._mark_obstacle(cell_index1, weight=0.5)
                            if updated:
                                updated_cells[cell_index1] = 1
                    #                   both are not out of index & 1st cell is non-obstacle => 2nd cell is obstacle
                    elif self.discovered_map[to_explore1[0]][to_explore1[1]] == 0 and self.discovered_map[to_explore2[0]][
                        to_explore2[1]] == 2:
                        #                        enable_print()
#                        print("1st cell is non-obstacle => mark 2nd index as obstacle")
                        cell_index2 = get_grid_index(to_explore2[0], to_explore2[1])
                        updated = self._mark_obstacle(cell_index2, weight=0.5)
                        if updated:
                            updated_cells[cell_index2] = 1
                    #                   both are not out of index & 2nd cell is non-obstacle =>mark 1st cell as obstacle
                    elif self.discovered_map[to_explore2[0]][to_explore2[1]] == 0 and self.discovered_map[to_explore1[0]][
                        to_explore1[1]] == 2:
                        #                        enable_print()
#                        print("1st index is marked as obstacle")
                        updated = self._mark_obstacle(cell_index1, weight=0.5)
                        if updated:
                            updated_cells[cell_index1] = 1
                    else:
                        #                        enable_print()
                       print("no update")
                except IndexError:
                    continue
            #               long range cant see obstacle => all 6 cells are clear
            #                elif reading == 0:
            #
            #                    blind_range_index_with_0_reading = {}
            #                    for cell in blind_range:
            #                        try:
            #                            if true_facing == NORTH:
            #    #                          to_explore is the target cell coordinates
            #                                to_explore = (y + cell, x)
            #                            elif true_facing == EAST:
            #                                to_explore = (y, x + cell)
            #                            elif true_facing == SOUTH:
            #                                to_explore = (y - cell, x)
            #                            elif true_facing == WEST:
            #                                to_explore = (y, x - cell)
            #
            #                            if to_explore[0] < 0 or to_explore[0] > 19 or to_explore[1] < 0 or to_explore[1] > 14:
            #                                print('ie')
            #                                raise IndexError

            #                       obstacle in blind range:
            #                       Sol1: long sensor for sure cannot see anything, don’t even bother its readings
            #                         if self.discovered_map[to_explore[0]][to_explore[1]] == 1:
            #                             enable_print()
            #                             print("the blind range cell has been explored and marked as obstacle, long sensor reading is useless")
            #                             disable_print()
            #                             blind_range_index_with_0_reading = {}
            #                             blind_range_no_obstacle = False
            #                             break
            #
            #                       unexplored cell in blind range:
            #                            if self.discovered_map[to_explore[0]][to_explore[1]] == 2:
            #                                enable_print()
            #                                print('unexplored cell in long sensor blind range & long sensor reading == 0')
            #                                disable_print()
            #                                cell_index = get_grid_index(to_explore[0], to_explore[1])
            #                                blind_range_index_with_0_reading[cell_index,true_facing] = cell
            #                                blind_range_no_obstacle = False
            #

            #                       explored and is non-obstacle
            #                            elif self.discovered_map[to_explore[0]][to_explore[1]] == 0:
            #                                if cell == len(blind_range) and blind_range_no_obstacle:
            #                                    blind_range_index_with_0_reading = {}
            #                                    enable_print()
            #                                    print('No obstacle in long sensor blind range & long sensor reading == 0')
            #                                    disable_print()
            #                                    readings[5] = 6
            #                                    reading = 6
            #                                    no_obstacle_long_sensor = True

            #                        blind_range_obstacle_status.append(self.discovered_map[to_explore[0]][to_explore[1]])

            #                        except IndexError:
            #                            break
            #                    print('br')
            #                    self.blind_range_cells = dict(list(self.blind_range_cells.items()) + list(blind_range_index_with_0_reading.items()))
            #                    enable_print()
            #                    print(self.blind_range_cells)
            #                    disable_print()

            #                ---------------------------------------------------------------------------------------------------

            # If reading is 0, means no obstacle in the covered range
            elif reading == 0:
                #               if no obstacle detected, mark all cells in its range as non-obstacle
                #               however if it's the long sensor, we have already dealt with it so do nothing
                #                if sensor_index(sensor) != LONG_SENSOR_INDEX:
#                print('No Obstacle in Covered Range')
                for cell in cover_range:
                    try:
                        if true_facing == NORTH:
                            to_explore = (y + cell, x)
                        elif true_facing == EAST:
                            to_explore = (y, x + cell)
                        elif true_facing == SOUTH:
                            to_explore = (y - cell, x)
                        elif true_facing == WEST:
                            to_explore = (y, x - cell)

                        if to_explore[0] < 0 or to_explore[0] > 19 or to_explore[1] < 0 or to_explore[1] > 14:
#                            print('ie')
                            raise IndexError

                        cell_index = get_grid_index(to_explore[0], to_explore[1])

                        # updated_cell, value = self._mark_probability(cell_index, 0 * weight, 1 * weight)
#                        if cell == 2:
#                            updated = self._mark_permanent(cell_index)
#                        else:
                        if sensor_index(sensor) == LONG_SENSOR_INDEX:
                            updated = self._mark_non_obstacle(cell_index, weight=0.5)
                        else:
                            updated = self._mark_non_obstacle(cell_index)
                        if updated:
                            updated_cells[cell_index] = 0
                    #                            for key in self.blind_range_cells.keys():
                    #                                if cell_index == key[0]:
                    #                                    enable_print()
                    #                                    print('key:')
                    #                                    print(key)
                    #
                    #                                    updated_cells_2 = self._mark_cells_non_obstacle(key)
                    #                                    enable_print()
                    #                                    print('updated cells:')
                    #                                    print(updated_cells_2)
                    #                                    updated_cells = dict(list(updated_cells.items()) + list(updated_cells_2.items()))
                    #                                    break
                    #             need to change

                    # weight = max(weight/2, 1)

                    except IndexError:
                        break
                print('br')

            # if reading!=0 in the read range, mark cells as 0 until the obstacle cell
            elif reading in read_range:
                #                enable_print()
#                print('long sensor detects obstacle')
                #                disable_print()

                #                enable_print()
                print('Has Obstacle in Covered Range or no obstacle in the entire range of long sensor')
                #                disable_print()
                # If the robot is able to observe onstacle in covered range, there is no obstacle in the blind spot.
                for cell in blind_range:
                    try:
                        if true_facing == NORTH:
                            to_explore = (y + cell, x)
                        elif true_facing == EAST:
                            to_explore = (y, x + cell)
                        elif true_facing == SOUTH:
                            to_explore = (y - cell, x)
                        elif true_facing == WEST:
                            to_explore = (y, x - cell)

                        if to_explore[0] < 0 or to_explore[0] > 19 or to_explore[1] < 0 or to_explore[1] > 14:
#                            print('ie')
                            raise IndexError

                        cell_index = get_grid_index(to_explore[0], to_explore[1])
                        #                        updated_cell, value = self._mark_probability(cell_index, 0, 1 * weight)
                        #                        if updated_cell is not None:
                        #                            updated_cells[cell_index] = value
#                        updated = self._mark_non_obstacle(cell_index)
#                        if cell == 2:
                        updated = self._mark_non_obstacle(cell_index, weight=0.5)
#                        else:
#                            updated = self._mark_non_obstacle(cell_index)
                        if updated:
                            updated_cells[cell_index] = 0
                        #                        enable_print()
#                        print('Long sensor has reading, mark cells in blind range as non-obstacle')
                    #                        disable_print()
                    #                        if updated:
                    #                            updated_cells[cell_index] = 0
                    #                            for key in self.blind_range_cells.keys():
                    #                                if cell_index == key[0]:
                    #                                    enable_print()
                    #                                    print('Long sensor has reading, mark cells in front of blind range cells as non-obstacle')
                    #                                    disable_print()
                    #                                    updated_cells_2 = self._mark_cells_non_obstacle(key)
                    #                                    updated_cells = dict(list(updated_cells.items()) + list(updated_cells_2.items()))
                    #                                    break
                    except IndexError:
                        break

                # Check for cells in read range
                for cell in read_range:
                    #                    enable_print()
                    #                    print('long sensor detects obstacle in read range')
                    #                    disable_print()
                    try:
                        if true_facing == NORTH:
                            to_explore = (y + cell, x)
                        elif true_facing == EAST:
                            to_explore = (y, x + cell)
                        elif true_facing == SOUTH:
                            to_explore = (y - cell, x)
                        elif true_facing == WEST:
                            to_explore = (y, x - cell)

                        if to_explore[0] < 0 or to_explore[0] > 19 or to_explore[1] < 0 or to_explore[1] > 14:
                            print('ie')
                            raise IndexError

                        cell_index = get_grid_index(to_explore[0], to_explore[1])

                        #                        if the cell is obstacle
                        if reading == cell:
#                            if cell == 1:
#                                updated = self._mark_permanent_obstacle(cell_index)
#                            else:
                            if sensor_index(sensor) == LONG_SENSOR_INDEX:
                                updated = self._mark_obstacle(cell_index, weight=0.5)
                            else:
                                updated = self._mark_obstacle(cell_index)
#                            updated = self._mark_obstacle(cell_index)
                            if updated:
                                updated_cells[cell_index] = 1
                            raise IndexError
                        #                            enable_print()
                        #                            print('long sensor detects obstacle')
                        #                            disable_print()
                        #                         other sensors

                        #                         long sensor and has obstacle in front
                        #                            elif not blind_range_no_obstacle:
                        #                            elif not no_obstacle_long_sensor:
                        #                                enable_print()
                        #                                print('long sensor detects obstacle')
                        #                                print(blind_range_no_obstacle)
                        #                                disable_print()
                        #                                self._mark_obstacle(cell_index)
                        #                                updated_cells[cell_index] = 1
                        #                                raise IndexError

                        #                        if the cell is before the obstacle cell / if it's long sensor
                        else:
#                            updated = self._mark_non_obstacle(cell_index)
#                            if cell == 2:
#                                updated = self._mark_permanent(cell_index)
#                            else:

                            if sensor_index(sensor) == LONG_SENSOR_INDEX:
                                updated = self._mark_non_obstacle(cell_index, weight=0.5)
                            else:
                                updated = self._mark_non_obstacle(cell_index)
#                            updated = self._mark_non_obstacle(cell_index)
                            if updated:
                                updated_cells[cell_index] = 0
                    #                            for key in self.blind_range_cells.keys():
                    #                                if cell_index == key[0]:
                    #                                    enable_print()
                    #                                    disable_print()
                    #                                    updated_cells_2 = self._mark_cells_non_obstacle(key)
                    #                                    updated_cells = dict(list(updated_cells.items()) + list(updated_cells_2.items()))
                    #                                    break
                    #                        updated_cell, value = self._mark_probability(cell_index, int(reading == cell) * weight, 1 * weight)
                    #                        if updated_cell is not None:
                    #                            updated_cells[cell_index] = value

                    # If the current cell is the one with obstacle, break the loop
                    #                        if self.discovered_map[to_explore[0]][to_explore[1]] == 1:
                    #                            raise IndexError

                    #                        weight /= 2

                    except IndexError:
                        break
#                print('br')
            else:
                print('Unacceptable Reading')

        #            elif sensor_index(sensor) == LONG_SENSOR_INDEX and reading in blind_range:

        #                print('Long Range Sensor: Obstacle observed in blind range')
        #                blind_range_obstacle_status = []
        #                for cell in blind_range:
        #                    try:
        #                        if true_facing == NORTH:
        #                            to_explore = (y + cell, x)
        #                        elif true_facing == EAST:
        #                            to_explore = (y, x + cell)
        #                        elif true_facing == SOUTH:
        #                            to_explore = (y - cell, x)
        #                        elif true_facing == WEST:
        #                            to_explore = (y, x - cell)
        #
        #                        if to_explore[0] < 0 or to_explore[0] > 19 or to_explore[1] < 0 or to_explore[1] > 14:
        #                            print('ie')
        #                            raise IndexError
        #                        blind_range_obstacle_status.append(self.discovered_map[to_explore[0]][to_explore[1]])
        #
        #                    except IndexError:
        #                        break
        #                print('br')
        #                print('blind_range_obstacle_status: {}'.format(blind_range_obstacle_status))
        #                if len(blind_range_obstacle_status) != 0:
        #                    if 2 in blind_range_obstacle_status:
        #                        if 1 not in blind_range_obstacle_status:
        #                            is_blind_range_undetected_obstacle = True
        #                        else:
        #                            is_blind_range_undetected_obstacle = blind_range_obstacle_status.index(1) > blind_range_obstacle_status.index(2)

        #        end of for loop for sensor iterations
        print('-' * 50)
        print('Total move counts: {}'.format(self.move_counts))
        #        if self.move_counts % CALIBRATION_SIDE_STEPS == 0:
        #            self.is_calibration_side_time = True
        #            print('Time to Calibrate')
        #        if self.is_calibration_side_time and self.is_calibrate_side_possible():
        #            self.calibrate_side(sender)
        #            self.is_calibration_side_time = False
        #
        #        if self.move_counts % CALIBRATION_FRONT_STEPS == 0:
        #            self.is_calibration_front_time = True
        #            print('Time to Calibrate')
        #        if self.is_calibration_front_time:
        #            self.is_calibration_front_time = not self.calibrate_front(sender)

        #        if is_arrow_scan and not self.is_fast_path:
        #            self.check_arrow(sender)

        return updated_cells


    #    (cell_index,facing):range_index
    #    def _mark_cells_non_obstacle(self,key):
    #        enable_print()
    #        print('in def _mark_cells_non_obstacle(self,key)')
    #
    #        update_cells = {}
    #        num_of_cells_to_update = 6 - self.blind_range_cells.pop(key)
    #        print("range index: ")
    #        print(num_of_cells_to_update)
    #        facing = key[1]
    #        y, x = get_matrix_coords(key[0])
    #
    #        for cell in range(1,num_of_cells_to_update+1):
    #            print("0")
    #            try:
    #                print("1 ")
    #                if facing == NORTH:
    #                    print("2 ")
    #                    to_mark = (y + cell, x)
    #                elif facing == EAST:
    #                    print("3 ")
    #                    to_mark = (y, x + cell)
    #                elif facing == SOUTH:
    #                    print("4 ")
    #                    to_mark = (y - cell, x)
    #                elif facing == WEST:
    #                    print("5 ")
    #                    to_mark = (y, x - cell)
    #
    #                if to_mark[0] < 0 or to_mark[0] > 19 or to_mark[1] < 0 or to_mark[1] > 14:
    #                    print('ieaaa')
    #                    enable_print()
    #                    print('out of index range error')
    #                    raise IndexError
    #
    #
    #                print("1 ")
    #                cell_index = get_grid_index(to_mark[0], to_mark[1])
    #                if self.discovered_map[to_mark[0]][to_mark[1]] == 1:
    #                    enable_print()
    #                    print('marking in blind range until obstacle')
    #                    break
    #                self._mark_permanent(cell_index)
    #                update_cells[cell_index] = 0
    #
    #
    #            except IndexError:
    #                break
    #        return update_cells


    #    def get_sensor_readings_blind_range(self, sender):
    #
    #        sender.send_arduino(ARDUINO_SENSOR)
    #        readings = sender.wait_arduino(self._readings_regex_arduino, is_regex=True)
    #        readings = readings.split(',')
    #        del readings[-1]
    #
    #        readings = [int(x) for x in readings]
    #
    #        robot_cells = get_robot_cells(self.center)
    #        sensors = self.sensors[:]
    #        sensor_index = sensors.index
    #        updated_cells = {}
    #        is_blind_range_undetected_obstacle = False
    #
    #        for sensor in sensors:
    #            true_facing = (sensor["facing"] + self.facing) % 4
    #
    #            if sensor["mount_loc"] != CS:
    #                offset = self.facing * 2
    #                true_mounting = (sensor["mount_loc"] + offset) % 8
    #            else:
    #                true_mounting = CS
    #
    #            if true_mounting == NWS:
    #                origin = robot_cells[0]
    #            elif true_mounting == NS:
    #                origin = robot_cells[1]
    #            elif true_mounting == NES:
    #                origin = robot_cells[2]
    #            elif true_mounting == WS:
    #                origin = robot_cells[3]
    #            elif true_mounting == ES:
    #                origin = robot_cells[5]
    #            elif true_mounting == SWS:
    #                origin = robot_cells[6]
    #            elif true_mounting == SS:
    #                origin = robot_cells[7]
    #            elif true_mounting == SES:
    #                origin = robot_cells[8]
    #            elif true_mounting == CS:
    #                origin = robot_cells[4]
    #
    #            y, x = get_matrix_coords(origin)
    #            cover_range = range(1, sensor["range"] + 1)
    #            read_range = range(sensor["blind_spot"] + 1, sensor["range"] + 1)
    #            blind_range = range(1, sensor["blind_spot"] + 1)
    #
    #            reading = readings[sensor_index(sensor)]
    #            print('Sensor', sensor_index(sensor))
    #
    #            weight = 4
    #
    #            # If reading is 0, means no obstacle in the covered range
    #            if reading == 0:
    #                print('No Obstacle in Covered Range')
    #                for cell in cover_range:
    #                    try:
    #                        if true_facing == NORTH:
    #                            to_explore = (y + cell, x)
    #                        elif true_facing == EAST:
    #                            to_explore = (y, x + cell)
    #                        elif true_facing == SOUTH:
    #                            to_explore = (y - cell, x)
    #                        elif true_facing == WEST:
    #                            to_explore = (y, x - cell)
    #
    #                        if to_explore[0] < 0 or to_explore[0] > 19 or to_explore[1] < 0 or to_explore[1] > 14:
    #                            print('ie')
    #                            raise IndexError
    #
    #                        cell_index = get_grid_index(to_explore[0], to_explore[1])
    #
    #                        updated_cell, value = self._mark_probability(cell_index, 0 * weight, 1 * weight)
    #                        if updated_cell is not None:
    #                            updated_cells[updated_cell] = value
    #
    #                        weight = max(weight/2, 1)
    #
    #                    except IndexError:
    #                        break
    #                print('br')
    #
    #            # if reading in the read range, mark cells as 0 until the obstacle cell
    #            elif reading in read_range:
    #                print('Has Obstacle in Covered Range')
    #
    #
    #                # If the robot is able to observe onstacle in covered range, there is no obstacle in the blind spot.
    #                for cell in blind_range:
    #                    try:
    #                        if true_facing == NORTH:
    #                            to_explore = (y + cell, x)
    #                        elif true_facing == EAST:
    #                            to_explore = (y, x + cell)
    #                        elif true_facing == SOUTH:
    #                            to_explore = (y - cell, x)
    #                        elif true_facing == WEST:
    #                            to_explore = (y, x - cell)
    #
    #                        if to_explore[0] < 0 or to_explore[0] > 19 or to_explore[1] < 0 or to_explore[1] > 14:
    #                            print('ie')
    #                            raise IndexError
    #
    #                        cell_index = get_grid_index(to_explore[0], to_explore[1])
    #                        updated_cell, value = self._mark_probability(cell_index, 0, 1 * weight)
    #                        if updated_cell is not None:
    #                            updated_cells[updated_cell] = value
    #                    except IndexError:
    #                        break
    #                print('br')
    #
    #                # Check for cells in read range
    #                for cell in read_range:
    #                    try:
    #                        if true_facing == NORTH:
    #                            to_explore = (y + cell, x)
    #                        elif true_facing == EAST:
    #                            to_explore = (y, x + cell)
    #                        elif true_facing == SOUTH:
    #                            to_explore = (y - cell, x)
    #                        elif true_facing == WEST:
    #                            to_explore = (y, x - cell)
    #
    #                        if to_explore[0] < 0 or to_explore[0] > 19 or to_explore[1] < 0 or to_explore[1] > 14:
    #                            print('ie')
    #                            raise IndexError
    #
    #                        cell_index = get_grid_index(to_explore[0], to_explore[1])
    #
    #                        updated_cell, value = self._mark_probability(cell_index, int(reading == cell) * weight, 1 * weight)
    #                        if updated_cell is not None:
    #                            updated_cells[updated_cell] = value
    #
    #                        # If the current cell is the one with obstacle, break the loop
    #                        if self.discovered_map[to_explore[0]][to_explore[1]] == 1:
    #                            raise IndexError
    #
    #                        weight /= 2
    #
    #                    except IndexError:
    #                        break
    #                print('br')
    #            else:
    #                print('Unacceptable Reading')
    #
    #            if sensor_index(sensor) == 2:
    #                if reading not in [1,2]:
    #                    cell = 3
    #                    if true_facing == NORTH:
    #                        to_explore = (y + cell, x)
    #                    elif true_facing == EAST:
    #                        to_explore = (y, x + cell)
    #                    elif true_facing == SOUTH:
    #                        to_explore = (y - cell, x)
    #                    elif true_facing == WEST:
    #                        to_explore = (y, x - cell)
    #
    #                    if to_explore[0] < 0 or to_explore[0] > 19 or to_explore[1] < 0 or to_explore[1] > 14:
    #                        print('ie')
    #                    else:
    #                        cell_index = get_grid_index(to_explore[0], to_explore[1])
    #                        updated_cell, value = self._mark_probability(cell_index, 1, 1 * 2)
    #                        if updated_cell is not None:
    #                            updated_cells[updated_cell] = value
    #
    #        return updated_cells

    def get_explore_string(self):
        """ Build and return the MDF string of the exploration status at the time of calling this function. """
        exploration_status = self.exploration_status[:]
        explore_str = ''.join(str(grid) for row in exploration_status for grid in row)
        explore_status_string = '11%s11' % explore_str
        explore_status_string = str(hex(int(explore_status_string, 2)))
        return explore_status_string[2:]


    def get_map_string(self):
        """ Build and return the MDF string of the robot's internal map at the time of calling this function. """
        discovered_map = self.discovered_map[:]
        map_str = ''.join(str(grid) for row in discovered_map for grid in row if grid != 2)
        pad_length = (4 - ((len(map_str) + 4) % 4)) % 4
        pad = '0' * pad_length
        map_string = '1111%s%s' % (map_str, pad)
        map_string = str(hex(int(map_string, 2)))
        map_string = map_string[3:]
        return map_string


    def is_calibrate_right_possible(self):
        #   W  W
        #   0, 1, 2 N
        # S 3, 4, 5 N
        # S 6, 7, 8
        #      E  E

        if self.facing == NORTH:
            #            print("turn every 3 steps NORTH")
            y1, x1 = get_matrix_coords(get_robot_cells(self.center)[5])
            x1 += 1
            y2, x2 = get_matrix_coords(get_robot_cells(self.center)[2])
            x2 += 1
            if x1 > 19:
                return True
        #             obstacle
        elif self.facing == SOUTH:
            #            print("turn every 3 steps SOUTH")
            #            y1,x1 = get_matrix_coords(get_robot_cells(self._robot.center)[2])
            y1, x1 = get_matrix_coords(get_robot_cells(self.center)[6])
            x1 -= 1
            y2, x2 = get_matrix_coords(get_robot_cells(self.center)[3])
            x2 -= 1
            if x1 < 0:
                return True
        elif self.facing == WEST:
            #            print("turn every 3 steps WEST")
            #            y1,x1 = get_matrix_coords(get_robot_cells(self._robot.center)[8] - ROW_LENGTH)
            y1, x1 = get_matrix_coords(get_robot_cells(self.center)[1])
            y1 += 1
            y2, x2 = get_matrix_coords(get_robot_cells(self.center)[0])
            y2 += 1
            if y1 > 14:
                return True
        #            blind_unexplored_coors = (y-1,x)
        elif self.facing == EAST:
            #            print("turn every 3 steps EAST")
            #            y1,x1 = get_matrix_coords(get_robot_cells(self._robot.center)[0])
            #            y1,x1 = get_matrix_coords(get_robot_cells(self._robot.center)[6] - ROW_LENGTH)
            y1, x1 = get_matrix_coords(get_robot_cells(self.center)[8])
            y1 -= 1
            y2, x2 = get_matrix_coords(get_robot_cells(self.center)[7])
            y2 -= 1
            if y1 < 0:
                return True
        #                obstacle
        if self.discovered_map[y1][x1] == self.discovered_map[y2][x2]:
            if self.discovered_map[y1][x1] == 1:
                return True
        return False
