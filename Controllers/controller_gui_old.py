from tkinter import *
from tkinter.filedialog import askopenfilename
import re
from time import time, sleep

from Utils.utils import *
from Algo.exploration import Exploration
from Algo.fastest_path import *
from Utils.constants import *

import threading
from ast import literal_eval
from Connections.connection_client import Message_Handler

"""This module defines the main GUI window for the robot simulation."""

__author__ = 'MDPTeam8'


class TimeUp(Exception):
    """Raised when the time limit has reached."""

    def __init__(self):
        print("TIME'S UP (GUI)")


class Window(Frame):
    """
    This class is the main GUI window.
    """

    _grid_size = 30  # size of one grid square in pixels

    def __init__(self, master):
        #        enable_print()

        """Initializes the GUI."""
        Frame.__init__(self, master)

        self._master = master
        self._filename = ''

        print("Init window starting")
        self._init_window()
        print("Init window completed")

        """
        Initialize the Controller class.
        """
        self._sender = Message_Handler(self._receive_handler)
        from Algo.real_robot import Robot
        self._robot = Robot(exploration_status=[[0] * ROW_LENGTH for _ in range(COL_LENGTH)],
                            facing=NORTH,
                            discovered_map=[[2] * ROW_LENGTH for _ in range(COL_LENGTH)],
                            sender = self._sender)

        self._paint_map()

        self._explore_limit = COMPLETION_THRESHOLD
        self._time_limit = TIME_LIMITE

        # Initialize connention client thread

        self._auto_update = True

        print('Init complete!')
        # self._sender.send_rpi("Hello from PC to RPi\n")
        # self._sender.send_arduino("Hello from PC to Arduino\n")
        # self._sender.send_android("Hello from PC to Android\n")

        self._facing = self._robot.facing
        self._draw_robot(START, self._facing)

        self.is_arrow_scan = IS_ARROW_SCAN

        disable_print()

        self._way_point = (17, 3)
        regex_str = '^(\d*,){%s}$' % (6)
        self._readings_regex_arduino = re.compile(regex_str)

    def _init_window(self):
        """
        Load all window elements.
        """
        self._master.title("MDP Team 8 Robot Simulation")

        self.pack(fill=BOTH, expand=1)

        bg_frame = Frame(self)

        bg_frame.pack(fill=X, padx=90)

        up_frame = Frame(bg_frame)
        top_frame = Frame(bg_frame)
        bottom_frame = Frame(bg_frame)
        middle_frame = Frame(bg_frame)

        up_frame.pack()
        top_frame.pack()
        middle_frame.pack()
        bottom_frame.pack(side='bottom')

        self._up_label = Label(up_frame, text="LITTLE TOMATO IS THE BEST", justify='center', fg='tomato2',
                               font=('Helvetica', 18))
        self._up_label.pack()
        self._blank_label = Label(up_frame, text="LITTLE TOMATO IS THE BEST", justify='center', fg='white',
                                  font=('Helvetica', 1))
        self._blank_label.pack()

        self._completion_label = Label(top_frame, text="0")
        self._completion_label.grid(row=0, column=2)

        self._timestep_label = Label(top_frame, text="Timestep(seconds):")
        self._timestep_label.grid(row=0, column=0)

        self._timestep_entry = Entry(top_frame, width=5, justify='center')
        self._timestep_entry.insert(END, "0.1")
        self._timestep_entry.grid(row=0, column=1)

        self._explore_label = Label(top_frame, text="Explore Cutoff (percentage):")
        self._explore_label.grid(row=1, column=0)

        self._explore_entry = Entry(top_frame, width=5, justify='center')
        self._explore_entry.insert(END, '100')
        self._explore_entry.grid(row=1, column=1)

        self._completion_label = Label(top_frame, text="0")
        self._completion_label.grid(row=1, column=2)

        self._time_limit_label = Label(top_frame, text="Time Limit(seconds):")
        self._time_limit_label.grid(row=2, column=0)

        self._time_limit_entry = Entry(top_frame, width=5, justify='center')
        self._time_limit_entry.insert(END, "10000000")
        self._time_limit_entry.grid(row=2, column=1)

        self._time_spent_label = Label(top_frame, text="0.0s")
        self._time_spent_label.grid(row=2, column=2)

        self._canvas = Canvas(middle_frame, height=COL_LENGTH * self._grid_size + 1,
                              width=ROW_LENGTH * self._grid_size + 1,
                              borderwidth=0, highlightthickness=0, background='#ffffff')

        self._canvas.pack(padx=20, pady=20)

        self._loadBtn = Button(bottom_frame, text="Load Map", command=self._load_map)
        self._loadBtn.grid(row=0, column=0)

        self._button = Button(bottom_frame, text="Explore", command=self._explore)
        self._button.grid(row=0, column=1)

        self._fp_button = Button(bottom_frame, text="Move Fastest Path", command=self._move_fastest_path)
        self._fp_button.grid(row=0, column=2)

        # Draw grid
        self._draw_grid()

    def _receive_handler(self, msg):
        """
        Parse and handle messages from the Android device.

        :param msg: The message received from the Android device.
        :return: N/A
        """
        if msg[0:8] == ANDROID_WAYPOINT:
            self._set_way_point(msg[8:])
        elif msg == ANDROID_CALIBRATE:
            thread = threading.Thread(target=self._explore)
            thread.daemon = True
            thread.start()
            enable_print()
            print('Start CALIBRATION')
            disable_print()
        elif msg == ANDROID_EXPLORE:
            thread = threading.Thread(target=self._explore)
            thread.daemon = True
            thread.start()
            enable_print()
            print('Start EXPLORATION')
            disable_print()
        elif msg == ANDROID_MOVE_FASTEST_PATH:
            thread = threading.Thread(target=self._move_fastest_path)
            thread.daemon = True
            thread.start()
            enable_print()
            print('Start FAST PATH')
            disable_print()
        elif msg == ANDROID_LOAD_EXPLORE_MAP:
            thread = threading.Thread(target=self._load_explore_map)
            thread.daemon = True
            thread.start()
            enable_print()
            print('LOAD EXPLORE MAP')
            disable_print()
        elif msg == ANDROID_FORWARD:
            self._sender.send_arduino(ARDUINO_FORWARD)
        elif msg == ANDROID_TURN_LEFT:
            self._sender.send_arduino(ARDUINO_TURN_LEFT)
        elif msg == ANDROID_TURN_RIGHT:
            self._sender.send_arduino(ARDUINO_TURN_RIGHT)
        elif msg == ANDROID_TURN_TO_BACKWARD:
            self._sender.send_arduino(ARDUINO_TURN_TO_BACKWARD)
#        elif msg == 'arrow_on':
#            self.is_arrow_scan = True
#        elif msg == 'arrow_off':
#            self.is_arrow_scan = False
#        elif msg == 'S':
#            self._sender.send_arduino(ARDUINO_SENSOR)
#        elif msg == ANDROID_BATTERY_DRAINER:
#            thread = threading.Thread(target=self._battery_drainer)
#            thread.daemon = True
#            thread.start()
#            enable_print()
#            print('START BATTERY DRAINER')
#            disable_print()
        else:
            enable_print()
            print('Unknown command from Android')
            disable_print()
            return

    def _load_explore_map(self):
        from Algo.real_robot import Robot
        self._robot = Robot(exploration_status=EXPLORE_STATUS_MAP,
                            facing=NORTH,
                            discovered_map=EXPLORATION_OBSTACLE_MAP)

        cells = [item for sublist in EXPLORATION_OBSTACLE_MAP for item in sublist]
        updated_cells = {i + 1: cells[i] for i in range(len(cells))}
        self._update_cells(updated_cells)
        self._update_android()

        self._calibrate()
        sleep(1)

        self._calibrate_after_exploration()
        sleep(1)

    def _set_way_point(self, coordinate):
        """
        Set the waypoint coordinates.

        :param coordinate: The coordinates received from the Android device.
        :return: N/A
        """
        #        enable_print()
        (row, col) = literal_eval(coordinate)
#        _way_point is in reverse order
        self._way_point = (col, row)
        print('Set Waypoint: {}'.format(self._way_point))
        self._mark_way_point(get_grid_index(col, row))

    #        disable_print()
#    def _calibrate_(self):
#
#        self._sender.send_arduino('T')
#        self._sender.wait_arduino(ARDUIMO_MOVED)
#        self._sender.send_arduino('A')
#        self._sender.wait_arduino(ARDUIMO_MOVED)
        
        
        
    def _calibrate(self):
        """
        Calibrate the robot.

        :return: N/A
        """
        while self._robot.facing != SOUTH:
            self._robot.turn_robot(self._sender, RIGHT)
            self._turn_head(self._facing, RIGHT)
            self._update_android()
#            self._sender.send_arduino('T')
#            self._sender.wait_arduino(ARDUIMO_MOVED)
        #           in the last iteration, robot turns to face SOUTH WALL => Do Clibration

        #       robot is facing SOUTH: turn right to face WEST WALL => Do calibration
        self._sender.send_arduino('T')
        self._sender.wait_arduino(ARDUIMO_MOVED)
        self._robot.turn_robot(self._sender, RIGHT)
        self._turn_head(self._facing, RIGHT)
        self._update_android()
        self._sender.send_arduino('T')
        self._sender.wait_arduino(ARDUIMO_MOVED)
        #       robot is facing West: turn right to face NORTH
        self._robot.turn_robot(self._sender, RIGHT)
        self._turn_head(self._facing, RIGHT)
        self._update_android()
#        self._sender.send_arduino('C')
#        self._sender.wait_arduino(ARDUIMO_MOVED)
        #       robot ends up facing NORTH, calibration done!

        enable_print()
        print('Calibration Done!')
        disable_print()

    #    def _battery_drainer(self):
    #        for j in range(2):
    #            for i in range(min(BATTERY_DRAINER_STEP_Y, 17)):
    #                self._robot.move_robot(self._sender, FORWARD)
    #            self._sender.send_arduino(BATTERY_DRAINER_TURN)
    #            self._sender.wait_arduino(ARDUIMO_MOVED)
    #            for i in range(min(BATTERY_DRAINER_STEP_X, 12)):
    #                self._robot.move_robot(self._sender, FORWARD)
    #            self._sender.send_arduino(BATTERY_DRAINER_TURN)
    #            self._sender.wait_arduino(ARDUIMO_MOVED)

    def _update_android(self):
        """
        Send the latest updates to the Android device.

        :return: N/A
        """
        msgs = []
        # Send the latest MDF strings to the Android device.
        #        msgs.append('"explore":"%s"'%self._robot.get_explore_string())
        #        msgs.append('"obstacleMap":"%s"'%self._robot.get_map_string())
        #        y, x = get_matrix_coords(self._robot.center)
        #        msgs.append('"robotPosition":"%s,%s,%s"' % (str(x), str(19 - y), str(self._robot.facing)))
        #        msgs.append('"arrowPosition":"{}"'.format(';'.join(self._robot.arrows_arduino)))
        #        msg =
        msgs = ['grid']

        msgs.append(self._robot.get_explore_string())
        msgs.append(self._robot.get_map_string())
        y, x = get_matrix_coords(self._robot.center)
        msgs.append(str(x))
        msgs.append(str(y))
        msgs.append(str(self._robot.facing))
        self._sender.send_android(','.join(msgs))

    def _update_android_camera(self):
        """
        Send the latest updates to the Android device.

        :return: N/A
        """
        msgs = ['ID']
        image_ID = 1
        msgs.append(str(image_ID))
        y, x = 10, 11
        msgs.append(str(x))
        msgs.append(str(y))

        self._sender.send_android(','.join(msgs))

    def _explore(self):
        print("starting calibration")
        continuous_right = 0
        explore_complete = False
        ''' Initialisation '''
        start_time = time()
        self.start_time = time()

#        time_limit = float(self._time_limit_entry.get().strip())
#        time_limit = 340
        time_limit = 300
        explore_limit = float(self._explore_entry.get().strip()) * 3
        steps_since_last_right_calibration = 0
#        right_staircase_cali = 0
#        mark start as explored
        updated_cells = self._robot.mark_robot_standing()
        self._update_cells(updated_cells)
#        facing north, get sensor readings
        updated_cells = self._robot.get_sensor_readings(self._sender, self.is_arrow_scan)
        self._update_cells(updated_cells)
        
#        self._sender.send_arduino('D')
#        self._sender.wait_arduino(ARDUIMO_MOVED)
#       turn right to face east
        self._robot.turn_robot(self._sender, RIGHT)
        self._turn_head(self._facing, RIGHT)
##        turn to face north
#        elf._robot.turn_robot(self._sender, RIGHT)
#        self._turn_head(self._facing, RIGHT)
#        updated_cells = self._robot.get_sensor_readings(self._sender, self.is_arrow_scan)
#        self._update_cells(updated_cells)
        
        
        
#        steps_since_last_side_calibration = 0
        
#        steps_since_last_front_calibration = 0
        
#        self._sender.send_arduino('T')
#        self._sender.wait_arduino(ARDUIMO_MOVED)
#        self._sender.send_arduino('A')
#        self._sender.wait_arduino(ARDUIMO_MOVED)
#        self._sender.send_arduino('C')
#        self._sender.wait_arduino(ARDUIMO_MOVED)
        
#        self._sender.wait_arduino(ARDUINO_SENSOR)
#        facing east, ask for sensor reading but not being processed
        self._sender.send_arduino(ARDUINO_SENSOR)
        sensor_readings = self._sender.wait_arduino(self._readings_regex_arduino, is_regex=True)
#        send android cali-done
        self._sender.send_android('ca_done')
        
        
        #         wait for ANex
        self._sender.wait_android()
        self._update_android()
        

        
        
        

        #       1st round of exploration
#        sensor_readings = None
        while True:
            try:
                #           ask robot for sensor reading & process sensor readings & update cell status
                self._robot.mark_robot_standing()
#                updated_cells =
#                if updated_cells:
#                    self._update_cells(updated_cells)
#                    self._update_android()
                    
                if sensor_readings == None:
                    updated_cells = self._robot.get_sensor_readings(self._sender, self.is_arrow_scan)
                else:
                    print("got sensor reading alr, not asking arduino again")
                    updated_cells = self._robot.get_sensor_readings(self._sender, self.is_arrow_scan, readings = sensor_readings)
                sensor_readings = None
                # send_rpi
                if updated_cells:
                    #       update robot cells in UI as non-obstacle
                    self._update_cells(updated_cells)
                    #       update android
                    self._update_android()
                ''' Turn right & turn back to Calibrate side distance using front sensors '''
                ''' Calibrate front distance using front sensors '''
# CALIBRATION_FRONT_STEPSsteps_since_last_front_calibration >= CALIBRATION_FRONT_STEPS and
                if self.is_calibrate_front_possible():
                    self._sender.send_arduino('T')
                    self._sender.wait_arduino(ARDUIMO_MOVED)
                    steps_since_last_front_calibration = 0
                    steps_since_last_side_calibration = 0
                
#                if steps_since_last_side_calibration >= 5 and self.is_calibrate_side_possible() and (not self.is_calibrate_front_possible()):
##                    print("turning right to calibrate distance")
#                    #                     turn right
#                    self._robot.turn_robot(self._sender, RIGHT)
#                    #                    update UI
#                    self._turn_head(self._facing, RIGHT)
#                    self._update_android()
#
#                    self._sender.send_arduino('T')
#                    self._sender.wait_arduino(ARDUIMO_MOVED)
#                    steps_since_last_side_calibration = 0
##                        steps_since_last_right_calibration = 0
#
#                    #                    turn left
#                    self._robot.turn_robot(self._sender, LEFT, self.is_arrow_scan)
#                    self._turn_head(self._facing, LEFT)
#                    self._update_android()
#
#                    if self.is_calibrate_right_possible():
#                        self._sender.send_arduino('C')
#                        self._sender.wait_arduino(ARDUIMO_MOVED)
#                        self.steps_since_last_right_calibration = 0
#

                ''' Calibrate right alignment using right sensors '''
                if self.is_calibrate_right_possible() and steps_since_last_right_calibration>= 2:
                    self._sender.send_arduino('C')
                    self._sender.wait_arduino(ARDUIMO_MOVED)
                    steps_since_last_right_calibration = 0
                else:
                    steps_since_last_right_calibration = steps_since_last_right_calibration + 1

                ''' Turn left and turn back for blind range undetected obstacle at bottom left '''
                front_and_left_image_detection = True
                right_free = self.check_free_right()
                front_free = self.check_free_front()
                if self.turn_left_for_blind_range(right_free,front_free) == True:
                    print("turning left for blind range")
    #                   1. turn left for sensor readings
                    self._robot.turn_robot(self._sender, LEFT)
                    #                   change algo UI
                    self._turn_head(self._facing, LEFT)
                    self._update_android()
                    #                            steps_since_last_side_calibration += 1

                    #                       2. ask robot for sensor reading & process sensor readings & update cell status
                    updated_cells = self._robot.get_sensor_readings(self._sender, self)
                    
                    if updated_cells:
                        #       update robot cells in UI as non-obstacle
                        self._update_cells(updated_cells)
                        #       update android
                        self._update_android()
                    if self.is_arrow_scan:
    #                            equivalent to check front and left if without the turn
                        self._check_obstacle_image_right()
                        self._check_obstacle_image_front()
                        front_and_left_image_detection = False

                    #                   3. turn right
                    self._robot.turn_robot(self._sender, RIGHT)
                    self._turn_head(self._facing, RIGHT)
                    self._update_android()
#                y,x=get_matrix_coords(self._robot.center)
#                #                 x
#                #           0, 1, 2 N
#                #           3, 4, 5
#                #         x 6, 7, 8
#                #              E  E
#                if self._robot.facing == NORTH:
##                    6
#                    y= y-1
#                    x = x-1
#                    blind_unexplored_coors = (y, x - 1)
#                elif self._robot.facing == SOUTH:
##                    2
#                    y= y+1
#                    x = x+1
#                    blind_unexplored_coors = (y, x + 1)
#                elif self._robot.facing == WEST:
#                    #8
#                    y= y-1
#                    x = x+1
#                    blind_unexplored_coors = (y - 1, x)
#                elif self._robot.facing == EAST:
##                    print("1111turn every 3 steps EAST")
#                    #0
#                    y= y+1
#                    x = x-1
#                    blind_unexplored_coors = (y + 1, x)
#                #                blind_unexplored_coors = get_matrix_coords(blind_unexplored_index)
#
#                if blind_unexplored_coors[0] >= 0 and blind_unexplored_coors[0] <= 19 and blind_unexplored_coors[
#                    1] >= 0 and blind_unexplored_coors[1] <= 14:
#                    #               to detect bottom left cell
#                    #                 unexplored_coors = self._get_unexplored()
#                    if self._robot.discovered_map[blind_unexplored_coors[0]][blind_unexplored_coors[1]] == 2:
##                        right_free = self.check_free_right()
##                        front_free = self.check_free_front()
#                        if right_free or front_free:
    
#                            #                   1. turn left for sensor readings
#                            self._robot.turn_robot(self._sender, LEFT)
#                            #                   change algo UI
#                            self._turn_head(self._facing, LEFT)
#                            self._update_android()
#                            #                            steps_since_last_side_calibration += 1
#
#                            #                       2. ask robot for sensor reading & process sensor readings & update cell status
#                            updated_cells = self._robot.get_sensor_readings(self._sender, self)
#
#                            if updated_cells:
#                                #       update robot cells in UI as non-obstacle
#                                self._update_cells(updated_cells)
#                                #       update android
#                                self._update_android()
#                            if self.is_arrow_scan:
##                            equivalent to check front and left if without the turn
#                                self._check_obstacle_image_right()
#                                self._check_obstacle_image_front()
#                                front_and_left_image_detection = False
#
#                            #                   3. turn right
#                            self._robot.turn_robot(self._sender, RIGHT)
#                            self._turn_head(self._facing, RIGHT)
#                            self._update_android()
                            
                            
                #                            steps_since_last_side_calibration += 1
                if self.is_arrow_scan:
#                    front_and_left_image_detection
                    self._check_obstacle_image_right()
                    if front_and_left_image_detection == True:
                        self._check_obstacle_image_front()
#                        self._check_obstacle_image_left()
                    
                    
#                if not right_free:
#                    command = self.check_right_staircase()
#                    if command:
#                        right_staircase_cali = 0
#                        self._sender.send_arduino(command)
#                        self._sender.wait_arduino(ARDUIMO_MOVED)
#                    else:
#                        right_staircase_cali = right_staircase_cali + 1
                        
                updated_cells = {}


#                is_free_right = self.check_free_right()
#                right_free = self.check_free_right()
#                front_free = self.check_free_front()
#                is_free_front
#                going to move to the right
                if right_free:
#                   if robot goes into a loop
                    if continuous_right >= 4:
                        print("more than 4 continous_right")
                        y, x = get_matrix_coords(self._robot.center)
#                        can solve the loop alr
                        if not front_free:
                            self._robot.turn_robot(self._sender, LEFT)
                            self._turn_head(self._facing, LEFT)
                            
                            self._update_android()
                            continuous_right = 0
#                        when it's more towards left and facing left=> go find the left wall; when it's towards right and facing right=> go find the right wall
#                       required fulfilled, keep moving forward until it is near to the wall/obstacle
                        elif (x<=7 and self._robot.facing == WEST) or (x>=8 and self._robot.facing == EAST) or (y>=10 and self._robot.facing == NORTH) or (y<=9 and self._robot.facing == SOUTH):
                            print("more than 4 continous_right1")
#                            is_free_front = self.check_free_front()
#                            move front if front is free
#                            if front_free:
                            print("more than 4 continous_right2")
                            moved = self._robot.move_robot(self._sender, FORWARD)
#                                expect a move, but there is a phantom => got sensor readings instead
                            if moved!= 'M':
                                sensor_readings = moved
                            else:
#                                UI
                                self._move_robot()
                                self._update_android()
#                       condition for moving forward to find wall not met, keep turning
                        else:
                            self._robot.turn_robot(self._sender,RIGHT)
                            self._turn_head(self._facing, RIGHT)

                            self._update_android()
#                                until it is back on track, turn to left and restart count
#                            else:
#                                print("more than 4 continous_right3")
#                                self._robot.turn_robot(self._sender, LEFT)
#                                self._turn_head(self._facing, LEFT)
#
#                                self._update_android()
#                                continuous_right = 0
#                        else:
#                            print("more than 4 continous_right4")
##                        not in the right direction, keep turning right
#
#
#                            moved = self._robot.move_robot(self._sender, RIGHT)
##                                expect a move, but there is a phantom => got sensor readings instead
#                            if moved!= 'M':
#                                sensor_readings = moved
#                            else:
##                                UI
#                                self._turn_head(self._facing, RIGHT)
#                                self._move_robot()
#                                self._update_android()
#                                continuous_right = continuous_right + 1

#                            self._robot.move_robot(self._sender, RIGHT)
#                            self._move_robot()
#                            self._update_android()
#                            continuous_right = continuous_right + 1

#                            continuous right<4, keep going
                    else:
#                    not in a loop, go to right
#                        self._turn_head(self._facing, RIGHT)
#                        self._update_android()
                        
                        moved = self._robot.move_robot(self._sender, RIGHT)
                        #                                expect a move, but there is a phantom => got sensor readings instead
                        if moved!= 'M':
                            sensor_readings = moved
                        else:
#                                UI
                            self._turn_head(self._facing, RIGHT)
                            self._move_robot()
                            self._update_android()
                            continuous_right = continuous_right + 1
                    
                #                ''' Move forward or turn left '''
                else:
#                    continuous right problem has been solved, restart counter
                    continuous_right = 0
                    print("more than 4 continous_right6")
#                     move forward
#                    is_free_front = self.check_free_front()
                    if front_free:
                        moved = self._robot.move_robot(self._sender, FORWARD)
                            #                                expect a move, but there is a phantom => got sensor readings instead
                        if moved!= 'M':
                            sensor_readings = moved
                        else:
#                                UI
                            self._move_robot()
                            self._update_android()
                    #                ''' Turn left & move forward '''
                    else:
                        self._robot.turn_robot(self._sender, LEFT)
                        self._turn_head(self._facing, LEFT)
                        self._update_android()
                        

#                steps_since_last_side_calibration += 1
#                steps_since_last_front_calibration += 1
#
#                print("end of the while loop movement")
                
#                count = self._robot.get_completion_count()
                    

                if self._robot.center == START:
#                   arena is fully explored
                    if self._robot.is_complete_after_back_to_start(explore_limit, start_time, time_limit):
                        explore_complete = True
#                        print("Exploration complete, heading back to START")
                        break

                    elif self._robot.get_completion_count() >= 200:
#                        print("fastest path explorations")
                        break
                    else:
                        continue
                elif self._robot.is_complete_before_back_to_start(explore_limit, start_time, time_limit):
                    explore_complete = True
                    break
#                print("end of the while loop")

            except IndexError:
                enable_print()
                print("ie!!!!!")
                disable_print()
                continue
            ''' End of 1st round Exploration  '''

        # fastest path exploration
#        steps_since_last_side_calibration = 0
#        steps_since_last_front_calibration = 0
#        steps_since_last_right_calibration = 0
        
        while not explore_complete:
            try:
                unexplored_coors = self._get_unexplored()
                for unexplored_coor in unexplored_coors:
                    nearest_unexplored_y, nearest_unexplored_x = unexplored_coor[0]
                    center_y, center_x = get_matrix_coords(self._robot.center)

                    print('Finding shortest path moves to nearest unexplored......')
                    print('nearest_unexplored_y, nearest_unexplored_x: {}'.format(
                        (nearest_unexplored_y, nearest_unexplored_x)))

                    moves = get_shortest_path_moves(self._robot,
                                                    (center_y, center_x),
                                                    (nearest_unexplored_y, nearest_unexplored_x))

                    if not moves:  # Check adjacent cells
                        print('Cannot find shortest path moves to nearest unexplored')

                        print('Finding shortest valid path moves to its adjacent cells......')
                        robot_cell_index = get_grid_index(nearest_unexplored_y, nearest_unexplored_x)
                        adjacent_cells = get_detectable_cells(robot_cell_index)
                        del adjacent_cells[4]

                        adj_order = [8, 9, 10, 11, 5, 6, 7, 3, 4, 0, 1, 2]
                        adjacent_cells = [adjacent_cells[i] for i in adj_order]

                        moves = get_shortest_valid_path(self._robot,
                                                        self._robot.center, adjacent_cells)

                    if moves:
                        print('Foundhortest path moves to nearest accessible unexplored: {}'.format(moves))
                        break

                if not moves:
                    print('WARNING: Cannot find shortest path moves to all unexplored cells')
                    print('Stopping fastest path exploration')
                    explore_complete = True
                    break


                # found a valid move to any of the accessible unexplored cells:
                steps_since_last_right_calibration = 0
                for move in moves:
                    # steps_since_last_side_calibration += 1
                    # steps_since_last_right_calibration += 1
                    print('Starting shortest path moves to the nearest accessible unexplored......')
                    #                turn to do calibration against right wall with front sensors
#                    if self.is_calibrate_side_possible() and steps_since_last_side_calibration >= CALIBRATION_SIDE_DISTANCE_STEPS:
#                        self._robot.turn_robot(self._sender, RIGHT)
#                        self._robot.turn_robot(self._sender, LEFT)
#                        steps_since_last_side_calibration = 0
#                        steps_since_last_right_calibration = 0
                    ''' Calibrate front distance using front sensors '''
                    if self.is_calibrate_front_possible():
                        print("Calibrating front distance")
                        self._sender.send_arduino('T')
                        self._sender.wait_arduino(ARDUIMO_MOVED)
                        steps_since_last_front_calibration = 0
                        steps_since_last_right_calibration = 0
                    if self.is_calibrate_right_possible() and steps_since_last_right_calibration>= 2:
                        self._sender.send_arduino('C')
                        self._sender.wait_arduino(ARDUIMO_MOVED)
                        steps_since_last_right_calibration = 0
                    else:
                        steps_since_last_right_calibration = steps_since_last_right_calibration + 1
                        
#                    if self.is_calibrate_right_possible() and steps_since_last_right_calibration >= CALIBRATION_RIGHT_STEPS:
#                        self._sender.send_arduino('C')
#                        self._sender.wait_arduino(ARDUIMO_MOVED)
#                        steps_since_last_right_calibration = 0

#                               #                    check bottom left cell
#                                    if move!=LEFT:
#                                        if self._robot.facing == NORTH:
#                                            print("turn every 3 steps NORTH")
#                                            blind_unexplored_index = get_robot_cells(self._robot.center)[6] - 1
#                                        if self._robot.facing == SOUTH:
#                                            print("turn every 3 steps SOUTH")
#                                            blind_unexplored_index = get_robot_cells(self._robot.center)[2] + 1
#                                        elif self._robot.facing == WEST:
#                                            print("turn every 3 steps WEST")
#                                            blind_unexplored_index = get_robot_cells(self._robot.center)[8] - ROW_LENGTH
#                                        elif self._robot.facing == EAST:
#                                            print("turn every 3 steps EAST")
#                                            blind_unexplored_index = get_robot_cells(self._robot.center)[0] + ROW_LENGTH
#
#                                        blind_unexplored_coors = get_matrix_coords(blind_unexplored_index)
#                               #               to detect bottom left cell
#                                        if self._robot.discovered_map[blind_unexplored_coors[0]][blind_unexplored_coors[1]] == 2:
#                                            print("trying to turn left for undetected bottom left corner with index:")
#                                            print(blind_unexplored_index)
#                               #                    turn left for sensor readings
#                                            self._robot.turn_robot(self._sender, LEFT)
#                                            #           ask robot for sensor reading & process sensor readings & update cell status
#                                            updated_cells = self._robot.get_sensor_readings(self._sender, self.is_arrow_scan)
#                                            if updated_cells:
#                                                #       update robot cells in UI as non-obstacle
#                                                self._update_cells(updated_cells)
#                                   #                   change algo UI
#                                                self._turn_head(self._facing, LEFT)
#                                                #       update android
#                                                self._update_android()
#                                                is_complete = self._robot.is_complete(explore_limit, start_time, time_limit)
#                                                if is_complete:
#                                                    explore_complete = true
#                                                    print("Exploration complete")
#                                                    self._sender.send_android('ex_done')
#                                                    break
#                                                raise CellsUpdated
#
#                                turn right
#                                            self._robot.turn_robot(self._sender, RIGHT, self.is_arrow_scan)
#                                            self._turn_head(self._facing, RIGHT)
#                                            self._update_android()

                    #                       move robot and mark robot standing cells as non-obstacle
                    
#                    if updated_cells1:
#                        #       update robot cells in UI as non-obstacle
#                        self._update_cells(updated_cells1)
#                        #       update android
#                        self._update_android()
#                        is_complete = self._robot.is_complete(explore_limit, start_time, time_limit)
#                        if is_complete:
#                            explore_complete = True
#                            print("Exploration complete, heading back to START")
#                            #                            self._sender.send_android('ex_done')
#                            break

                    #                        get robot sensor readings
                    self._robot.move_robot(self._sender, move)
                    
                    updated_cells2 = self._robot.get_sensor_readings(self._sender, self.is_arrow_scan)
                    if updated_cells2:
                        #       update robot cells in UI as non-obstacle
                        self._update_cells(updated_cells2)
                        #                   change algo UI
                        self._turn_head(self._facing, LEFT)
                        #       update android
                        self._update_android()
                        is_complete = self._robot.is_complete(explore_limit, start_time, time_limit)
                        if is_complete:
                            explore_complete = True
                            print("Exploration complete, heading back to START")
                            #                            self._sender.send_android('ex_done')
                            break

                    if updated_cells2:
                        continue

            except CellsUpdated:
                continue
            except PathNotFound:
                continue

        if self._robot.center == START:
            self._sender.send_android('ex_done')
            self._calibrate()
            self._calibrate_after_exploration()
            return

        center_y, center_x = get_matrix_coords(self._robot.center)
        start_y, start_x = get_matrix_coords(START)
        moves = get_shortest_path_moves(self._robot,
                                        (center_y, center_x), (start_y, start_x), is_give_up=True)
        if not moves:
            print("Something wrong, couldn't find path back to START")
        #            self._sender.send_android('ex_done')
        else:
            for move in moves:
                self._robot.move_robot(self._sender, move)
                self._update_android()
            self._sender.send_android('ex_done')

        print_map_info(self._robot)

        self._calibrate()
        self._calibrate_after_exploration()

    def _calibrate_after_exploration(self):
        """
        Post-exploration calibration to prepare the robot for the fastest path.

        :return: N/A
        """
        #        enable_print()
#        print('Calibrating for fast path...')
        #        disable_print()
        ''' finding fastest path '''
        print("finding fastest path")
        self._fastest_path = self._find_fastest_path()

        #       change robot facing according to fastest path
        if self._fastest_path[0] != FORWARD:
            if self._fastest_path[0] != RIGHT:
                print("Error in fastest path: first move is either forward or right")
            print('Turning Robot')
            self._robot.turn_robot(self._sender, self._fastest_path[0])
            print('Robot Turned')
            self._turn_head(self._facing, self._fastest_path[0])
        #       each command is direction followed by forward => forward is not explicitly stated
        self._fastest_path[0] = FORWARD
        self._update_android()

        #        enable_print()
        print('Fastest path preparation done!')

    #        disable_print()
    
    
    def _find_fastest_path(self):
        """Calculate and return the set of moves required for the fastest path."""
        print("waypoint used for fastest path: ")
        print(self._way_point)
#        self._way_point is row,col
        from Algo.sim_robot import Robot
        clone_robot = Robot(exploration_status=self._robot.exploration_status,
                            facing = NORTH,
                            discovered_map=self._robot.discovered_map,
                            real_map=[[0] * ROW_LENGTH for _ in range(COL_LENGTH)])

        fastest_path_start_way_point = get_shortest_path_moves(clone_robot,
                                                               start=(1, 1),
                                                               goal=self._way_point)
        print("finised fastest path from start to waypoint")

        if fastest_path_start_way_point:
            for move in fastest_path_start_way_point:
                clone_robot.move_robot(move)
        
        before_way_point = previous_cell(clone_robot.center, clone_robot.facing)
        
        fastest_path_way_point_goal = get_shortest_path_moves(clone_robot,
                                                              start=self._way_point,
                                                              goal=(18, 13),
                                                              before_start_point=before_way_point)

        return fastest_path_start_way_point + fastest_path_way_point_goal

    
    def _move_fastest_path_test(self):
        
        """Move the robot along the fastest path."""
        self._fastest_path = self._find_fastest_path()
        if self._fastest_path[0] != FORWARD:
            if self._fastest_path[0] != RIGHT:
                print("Error in fastest path: first move is either forward or right")
            print('Turning Robot')
            self._robot.turn_robot(self._sender, self._fastest_path[0])
            print('Robot Turned')
            self._turn_head(self._facing, self._fastest_path[0])
        #       each command is direction followed by forward => forward is not explicitly stated
        self._fastest_path[0] = FORWARD
        self._update_android()

        #        enable_print()
        print('Fastest path preparation done!')
        
        if self._fastest_path:
            self._robot.is_fast_path = True
    #           moves contain numbers but got append the missing foward after turn
            moves = []
            for index, move in enumerate(self._fastest_path):
                if move != 0:
                    moves.append(move)
                moves.append(FORWARD)
            print("after appending the missing foward movements after turns:")
            print(moves)
            
    #            add calibration to moves
            moves_calibration = []
    #            FORWARD = 0                                             # Move forward 1 square
    #            LEFT = -1                                               # Turn left and move forward 1 square
    #            RIGHT = 1                                               # Turn right and move forward 1 square
    #            BACKWARD = 2
            for index, move in enumerate(moves):
    #                for the last step
                if index == len(moves) - 1:
                    moves_calibration.append(move)
                    break
    #                append move
                moves_calibration.append(move)
    #                simulate robot movement
                if move == FORWARD:
                    self._robot.move_robot_algoo(FORWARD)
    #                    update UI
                else:
                    self._robot.turn_robot_algoo(move)
    #                    update UI
    #                check if can do calibration
                if self.is_calibrate_front_possible():
                    moves_calibration.append('T')

                elif self.is_calibrate_right_possible():
                    moves_calibration.append('C')
    #                else:
    #                    command =self.check_right_staircase()
    #                    if command:
    #                       moves_calibration.append(command)
    #                    self._sender.send_arduino(command)
    #                    self._sender.wait_arduino(ARDUIMO_MOVED)
                    
    #            print
    #            command = self.check_right_staircase()
    #            if command:
    #                right_staircase_cali = 0
    #                self._sender.send_arduino(command)
    #                self._sender.wait_arduino(ARDUIMO_MOVED)
    #            moves = moves_calibration
    #            print("moves with calibration")
    #            print(moves)
    #
                
            
                
    #                ar_command = get_arduino_cmd(move)
    #                ar_command =
            

            moves_arduino = []
            arduino_forward_move_instructions = ['z', 'W', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j']
            forward_count = 0
            for index, move in enumerate(moves):
    #            change here to change the max number of continuous forward movements
                if move == 'T' or move == 'C':
                    moves_arduino.append(move)
                    continue
                if forward_count >= 3:
                    moves_arduino.append(arduino_forward_move_instructions[forward_count])
                    forward_count = 0
                if move == 0:
                    forward_count = forward_count + 1
                    if index == len(moves) - 1:
                        moves_arduino.append(arduino_forward_move_instructions[forward_count])
                else:
                    if forward_count != 0:
                        moves_arduino.append(arduino_forward_move_instructions[forward_count])
                        forward_count = 0
                    moves_arduino.append(get_arduino_cmd(move))
            print("after converting to arduino commands:")
            print(moves_arduino)
            #

            #            send fastest path moves to update Android UI
    #            thread = threading.Thread(target=self._update_android_fast_path)
    #            thread.daemon = True
    #            thread.start()

            #            self._sender.send_arduino(''.join(self._moves_arduino))

            for moves in moves_arduino:
                self._sender.send_arduino(moves)
                self._sender.wait_arduino(ARDUIMO_MOVED)
                

            #            enable_print()
            self._sender.send_android('fp_done')
            print('Reached GOAL!')
        #            disable_print()
        else:
            #            enable_print()
            print("No valid path")
            
    def _move_fastest_path(self):
        """Move the robot along the fastest path."""
        if self._fastest_path:
            self._robot.is_fast_path = True
#           moves contain numbers but got append the missing foward after turn
            moves = []
            for index, move in enumerate(self._fastest_path):
                if move != 0:
                    moves.append(move)
                moves.append(FORWARD)
            print("after appending the missing foward movements after turns:")
            print(moves)
            
#            add calibration to moves
            moves_calibration = []
#            FORWARD = 0                                             # Move forward 1 square
#            LEFT = -1                                               # Turn left and move forward 1 square
#            RIGHT = 1                                               # Turn right and move forward 1 square
#            BACKWARD = 2
            for index, move in enumerate(moves):
#                for the last step
                if index == len(moves) - 1:
                    moves_calibration.append(move)
                    break
#                append move
                moves_calibration.append(move)
#                simulate robot movement
                if move == FORWARD:
                    self._robot.move_robot_algoo(FORWARD)
#                    update UI
                else:
                    self._robot.turn_robot_algoo(move)
#                    update UI
#                check if can do calibration
                if self.is_calibrate_front_possible():
                    moves_calibration.append('T')

                elif self.is_calibrate_right_possible():
                    moves_calibration.append('C')
#                else:
#                    command =self.check_right_staircase()
#                    if command:
#                       moves_calibration.append(command)
#                    self._sender.send_arduino(command)
#                    self._sender.wait_arduino(ARDUIMO_MOVED)
                    
#            print
#            command = self.check_right_staircase()
#            if command:
#                right_staircase_cali = 0
#                self._sender.send_arduino(command)
#                self._sender.wait_arduino(ARDUIMO_MOVED)
#            moves = moves_calibration
#            print("moves with calibration")
#            print(moves)
#
                
            
                
#                ar_command = get_arduino_cmd(move)
#                ar_command =
            

            moves_arduino = []
            arduino_forward_move_instructions = ['z', 'W', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j']
            forward_count = 0
            for index, move in enumerate(moves):
#            change here to change the max number of continuous forward movements
                if move == 'T' or move == 'C':
                    moves_arduino.append(move)
                    continue
                if forward_count >= 3:
                    moves_arduino.append(arduino_forward_move_instructions[forward_count])
                    forward_count = 0
                if move == 0:
                    forward_count = forward_count + 1
                    if index == len(moves) - 1:
                        moves_arduino.append(arduino_forward_move_instructions[forward_count])
                else:
                    if forward_count != 0:
                        moves_arduino.append(arduino_forward_move_instructions[forward_count])
                        forward_count = 0
                    moves_arduino.append(get_arduino_cmd(move))
            print("after converting to arduino commands:")
            print(moves_arduino)
            #

            #            send fastest path moves to update Android UI
#            thread = threading.Thread(target=self._update_android_fast_path)
#            thread.daemon = True
#            thread.start()

            #            self._sender.send_arduino(''.join(self._moves_arduino))

            for moves in moves_arduino:
                self._sender.send_arduino(moves)
                self._sender.wait_arduino(ARDUIMO_MOVED)
                

            #            enable_print()
            self._sender.send_android('fp_done')
            print('Reached GOAL!')
        #            disable_print()
        else:
            #            enable_print()
            print("No valid path")

    #            disable_print()

    def _update_android_fast_path(self):
        for move in ''.join(self._moves_arduino):
            sleep(ANDROID_FAST_PATH_SLEEP_SEC)
#             change actual robot position and facing based on fastest path algo
            self._robot.move_robot_algo(convert_arduino_cmd_to_direction(move))
#
            if convert_arduino_cmd_to_direction(move) == FORWARD:
                self._move_robot(convert_arduino_cmd_to_direction(move))
            else:
                self._turn_head(self._facing, convert_arduino_cmd_to_direction(move))

            self._update_android()

    def _draw_grid(self):
        """Draw the virtual maze."""
        self._grid_squares = []

        for y in range(COL_LENGTH):
            temp_row = []
            for x in range(ROW_LENGTH):
                temp_square = self._canvas.create_rectangle(x * self._grid_size, (COL_LENGTH - 1 - y) * self._grid_size,
                                                            (x + 1) * self._grid_size,
                                                            (COL_LENGTH - y) * self._grid_size, width=3, outline='gold',
                                                            fill='light goldenrod yellow')
                temp_row.append(temp_square)

            self._grid_squares.append(temp_row)

    def _draw_robot(self, location, facing):
        """Draw the robot in a given location with a given facing."""
        if location in BORDERS[NORTH] or location in BORDERS[SOUTH] \
                or location in BORDERS[EAST] or location in BORDERS[WEST]:
            print("invalid location")
            return

        top_left_grid = self._canvas.coords(location + ROW_LENGTH - 1)
        x = top_left_grid[0]
        y = top_left_grid[1]

        self._robot_graphic = self._canvas.create_oval(x, y, x + (3 * self._grid_size), y + (3 * self._grid_size),
                                                       width=2, fill="tomato2", outline="tomato3")
        self._draw_head(location, facing)

    def _draw_head(self, location, facing):
        """Draw the head of the robot based on the robot's location and facing."""
        if facing == NORTH:
            corner = self._canvas.coords(location + ROW_LENGTH)
        elif facing == SOUTH:
            corner = self._canvas.coords(location - ROW_LENGTH)
        elif facing == EAST:
            corner = self._canvas.coords(location + 1)
        else:
            corner = self._canvas.coords(location - 1)
        x, y = corner[0], corner[1]
        self._head = self._canvas.create_oval(x + (self._grid_size // 3), y + (self._grid_size // 3),
                                              x + (2 * (self._grid_size // 3)) + 1,
                                              y + (2 * (self._grid_size // 3)) + 1,
                                              width=0, fill="dark green")

    def _turn_head(self, facing, direction):
        """Move the graphical head of the robot in a certain direction."""
        if facing == NORTH:
            if direction == LEFT:
                self._canvas.move(self._head, -self._grid_size, self._grid_size)
                self._facing = WEST
            elif direction == RIGHT:
                self._canvas.move(self._head, self._grid_size, self._grid_size)
                self._facing = EAST
            elif direction == BACKWARD:
                self._canvas.move(self._head, 0, self._grid_size * 2)
                self._facing = SOUTH
        elif facing == SOUTH:
            if direction == LEFT:
                self._canvas.move(self._head, self._grid_size, -self._grid_size)
                self._facing = EAST
            elif direction == RIGHT:
                self._canvas.move(self._head, -self._grid_size, -self._grid_size)
                self._facing = WEST
            elif direction == BACKWARD:
                self._canvas.move(self._head, 0, -self._grid_size * 2)
                self._facing = NORTH
        elif facing == EAST:
            if direction == LEFT:
                self._canvas.move(self._head, -self._grid_size, -self._grid_size)
                self._facing = NORTH
            elif direction == RIGHT:
                self._canvas.move(self._head, -self._grid_size, self._grid_size)
                self._facing = SOUTH
            elif direction == BACKWARD:
                self._canvas.move(self._head, -self._grid_size * 2, 0)
                self._facing = WEST
        else:
            if direction == LEFT:
                self._canvas.move(self._head, self._grid_size, self._grid_size)
                self._facing = SOUTH
            elif direction == RIGHT:
                self._canvas.move(self._head, self._grid_size, -self._grid_size)
                self._facing = NORTH
            elif direction == BACKWARD:
                self._canvas.move(self._head, self._grid_size * 2, 0)
                self._facing = EAST

        self.update()

    def _move_robot(self, direction=FORWARD):
        """Move the graphical robot in a certain direction."""
        switcher = {
            NORTH: self._north_facing_move,
            SOUTH: self._south_facing_move,
            EAST: self._east_facing_move,
            WEST: self._west_facing_move
        }

        f = switcher.get(self._facing)

        if direction == BACKWARD:
            self._turn_head(self._facing, RIGHT)
            self._turn_head(self._facing, RIGHT)
        elif direction != FORWARD:
            self._turn_head(self._facing, direction)

        f(direction)

    def _north_facing_move(self, direction):
        """Move the graphical robot as it is facing north."""
        switcher = {
            FORWARD: self._move_up,
            LEFT: self._move_left,
            RIGHT: self._move_right,
            BACKWARD: self._move_down
        }

        f = switcher.get(direction)

        f()

    def _south_facing_move(self, direction):
        """Move the graphical robot as it is facing south."""
        switcher = {
            FORWARD: self._move_down,
            LEFT: self._move_right,
            RIGHT: self._move_left,
            BACKWARD: self._move_up
        }

        f = switcher.get(direction)

        f()

    def _east_facing_move(self, direction):
        """Move the graphical robot as it is facing east."""
        switcher = {
            FORWARD: self._move_right,
            LEFT: self._move_up,
            RIGHT: self._move_down,
            BACKWARD: self._move_left
        }

        f = switcher.get(direction)

        f()

    def _west_facing_move(self, direction):
        """Move the graphical robot as it is facing west."""
        switcher = {
            FORWARD: self._move_left,
            LEFT: self._move_down,
            RIGHT: self._move_up,
            BACKWARD: self._move_right
        }

        f = switcher.get(direction)

        f()

    def _move_up(self):
        """Move the graphical robot toward the top of the maze regardless of its facing."""
        self._canvas.move(self._robot_graphic, 0, -self._grid_size)
        self._canvas.move(self._head, 0, -self._grid_size)
        self.update()

    def _move_left(self):
        """Move the graphical robot toward the left of the maze regardless of its facing."""
        self._canvas.move(self._robot_graphic, -self._grid_size, 0)
        self._canvas.move(self._head, -self._grid_size, 0)
        self.update()

    def _move_right(self):
        """Move the graphical robot toward the right of the maze regardless of its facing."""
        self._canvas.move(self._robot_graphic, self._grid_size, 0)
        self._canvas.move(self._head, self._grid_size, 0)
        self.update()

    def _move_down(self):
        """Move the graphical robot toward the right of the maze regardless of its facing."""
        self._canvas.move(self._robot_graphic, 0, self._grid_size)
        self._canvas.move(self._head, 0, self._grid_size)
        self.update()

    def _update_cells(self, updated_cells):
        """Repaint the cells that have been updated."""
        if type(updated_cells) != dict:
            updated_cells = updated_cells[0]
        start_cells = get_robot_cells(START)
        goal_cells = get_robot_cells(GOAL)
        for cell, value in updated_cells.items():
            if cell in start_cells:
                print("start cells:")
                print(updated_cells)
                self.mark_cell(cell, 'goldenrod2')
            elif cell in goal_cells:
                self.mark_cell(cell, 'olive drab')
            elif not value:
                self.mark_cell(cell, 'light goldenrod yellow')
            else:
                self.mark_cell(cell, 'tomato3')

        if self.is_arrow_scan:
            for y, x, a in self._robot.arrows:
                self._draw_arrow(get_grid_index(y, x), a)
        time_delta = float(time() - self.start_time)
        self._time_spent_label.config(text="%.2f" % time_delta + "s")
        self._completion_label.config(text=(str(self._robot.get_completion_count() // 3)))
    def turn_left_for_blind_range(self,right_free,front_free):
        y,x=get_matrix_coords(self._robot.center)
        #                 x
                        #           0, 1, 2 N
                        #           3, 4, 5
                        #         x 6, 7, 8
                        #              E  E
        if self._robot.facing == NORTH:
#                    6
            y1= y-1
            x1 = x-2
            blind_unexplored_coors1 = (y1, x1)
            if blind_unexplored_coors1[0] < 0 or blind_unexplored_coors1[0] > 19 or blind_unexplored_coors1[1] < 0 or blind_unexplored_coors1[1] > 14:
                return False
#                    3
            y2= y
            x2 = x-2
#            blind_unexplored_coors2 = (y2, x2)
#                    0
            y3= y+1
            x3 = x-2
#            blind_unexplored_coors3 = (y3, x3)
            
        elif self._robot.facing == SOUTH:
#                    2
            y1= y+1
            x1 = x+2
            blind_unexplored_coors1= (y1, x1)
            if blind_unexplored_coors1[0] < 0 or blind_unexplored_coors1[0] > 19 or blind_unexplored_coors1[1] < 0 or blind_unexplored_coors1[1] > 14:
                return False
#                    5
            y2= y
            x2 = x+2
#            blind_unexplored_coors2 = (y2, x2)
#                    8
            y3= y-1
            x3 = x+2
#            blind_unexplored_coors3 = (y3, x3)
        elif self._robot.facing == WEST:
            #8
            y1= y-2
            x1 = x+1
            blind_unexplored_coors1= (y1, x1)
            if blind_unexplored_coors1[0] < 0 or blind_unexplored_coors1[0] > 19 or blind_unexplored_coors1[1] < 0 or blind_unexplored_coors1[1] > 14:
                return False
#                    7
            y2= y-2
            x2 = x
#            blind_unexplored_coors2 = (y2, x2)
#                    6
            y3= y-2
            x3 = x-1
#            blind_unexplored_coors3 = (y3, x3)
        elif self._robot.facing == EAST:
#                    print("1111turn every 3 steps EAST")
            #0
            y1= y+2
            x1 = x-1
            blind_unexplored_coors1= (y1, x1)
            if blind_unexplored_coors1[0] < 0 or blind_unexplored_coors1[0] > 19 or blind_unexplored_coors1[1] < 0 or blind_unexplored_coors1[1] > 14:
                return False
#                    2
            y2= y+2
            x2 = x
#            blind_unexplored_coors2 = (y2, x2)
#                    2
            y3= y+2
            x3 = x+1
#            blind_unexplored_coors3 = (y3, x3)
        
        if right_free:
            if self._robot.discovered_map[y1][x1] == 2 or self._robot.discovered_map[y2][x2] == 2 or self._robot.discovered_map[y3][x3] == 2:
                return True
            return False
        if front_free:
            if self._robot.discovered_map[y1][x1] == 2:
                return True
            return False

    def _load_map(self):
        """
        Load a map descriptor text file.

        :return: True if the file exists and is able to be successfully parsed, false otherwise.
        """
        self._filename = askopenfilename(title="Select Map Descriptor", filetypes=[("Text Files (*.txt)", "*.txt")])

        if self._filename:
            print(self._filename)
            if self._parse_map(self._filename):
                self._paint_map()
                return True
            print("File %s cannot be parsed" % self._filename)
            return False
        print("File %s does not exist" % self._filename)
        return False

    def _mark_way_point(self, grid_num):
        """
        Mark a grid as obstructed

        :param grid_num: ID of the grid to be marked
        :return: N/A
        """
        self._canvas.itemconfig(grid_num, fill="#ffc700")
        self.update()

    def _parse_map(self, filename):
        """
        Parse a map descriptor text file

        :param filename: The name of the map descriptor file
        :return: True if the file is able to be successfully parsed, false otherwise.
        """
        file = open(filename, mode="r")
        map_str = file.read()

        match = re.fullmatch("[012345\n]*", map_str)
        if match:
            self._grid_map = []
            row_strings = map_str.split("\n")
            for row_string in row_strings[:NUM_ROWS]:
                grid_row = []
                for char in row_string:
                    bit = int(char)
                    grid_row.append(bit)
                self._grid_map.append(grid_row)
            return True

        return False

    def _paint_map(self):
        """Paint the unexplored map on the grid."""
        for i in range(NUM_ROWS):
            for j in range(NUM_COLS):
                grid_num = i * ROW_LENGTH + j + 1
                self.mark_cell(grid_num, 'gray55')

    def mark_cell(self, cell_index, cell_type):
        """Mark a cell as a certain type."""
        self._canvas.itemconfig(cell_index, fill=cell_type)
        self.update()

    def _draw_arrow(self, location, facing):
        top_left_grid = self._canvas.coords(location)
        x, y = top_left_grid[0], top_left_grid[1]

        if facing == NORTH:
            points = [x, y + self._grid_size, x + self._grid_size, y + self._grid_size, x + self._grid_size / 2, y]
        elif facing == SOUTH:
            points = [x, y, x + self._grid_size, y, x + self._grid_size / 2, y + self._grid_size]
        elif facing == EAST:
            points = [x, y, x, y + self._grid_size, x + self._grid_size, y + self._grid_size / 2]
        else:
            points = [x + self._grid_size, y, x + self._grid_size, y + self._grid_size, x, y + self._grid_size / 2]

        self._canvas.create_polygon(points, fill='gold', width=3)

    def get_time_elapsed(start_time):
        """Get the elapsed time."""
        return float(time() - start_time)

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

    def is_calibrate_front_possible(self):
        # W  N    x  x E
        # x  # 0, 1, 2
        # x  # 3, 4, 5 x
        #    # 6, 7, 8 x
        #    S x  x
        
        y,x=get_matrix_coords(self._robot.center)
        
        if self._robot.facing == NORTH:
#        0,1
            y1= y+2
            x1 = x-1
            if y1 > 19:
                return True
                
            y2= y+2
            x2 = x


        elif self._robot.facing == SOUTH:
#         6,7
            y1= y-2
            x1 = x+1
            if y1 < 0:
                return True

            y2= y-2
            x2 = x

        elif self._robot.facing == WEST:
#        3,6
            y1= y
            x1 = x-2
            if x1 < 0:
                return True

            y2= y-1
            x2 = x-2

        #            blind_unexplored_coors = (y-1,x)
        elif self._robot.facing == EAST:
#        2,5
            y1= y+1
            x1 = x+2
            if x1 > 14:
                return True
                
            y2= y
            x2 = x+2

        if self._robot.discovered_map[y1][x1] == self._robot.discovered_map[y2][x2]:
            if self._robot.discovered_map[y1][x1] == 1:
                return True
        return False
# '''  NOT BEING USED '''
    def is_calibrate_right_possible(self):
        #           W  W
        #           0, 1, 2 N
        #         S 3, 4, 5 N
        #         S 6, 7, 8
        #              E  E
        y,x=get_matrix_coords(self._robot.center)
        if self._robot.facing == NORTH:
#            print("calibrate right possible NORTH")
#2,5
            y1= y+1
            x1 = x+2
            if x1 > 11:
                return False
                
            y2= y
            x2 = x+2
            

        elif self._robot.facing == SOUTH:
#            print("calibrate right possible SOUTH")
#3,6
            y1= y
            x1 = x-2
            if x1 < 3:
                return False

            y2= y-1
            x2 = x-2

        elif self._robot.facing == WEST:
#            print("calibrate right possible WEST")
#0,1
            y1= y+2
            x1 = x-1
            if y1 > 16:
                return False
                
            y2= y+2
            x2 = x

        elif self._robot.facing == EAST:
#            print("calibrate right possible EAST")
#8,7
            y1= y-2
            x1 = x+1
            if y1 < 3:
                return False

            y2= y-2
            x2 = x
#        print("doing right sensor calibration")

        if self._robot.discovered_map[y1][x1] == self._robot.discovered_map[y2][x2]:
            if self._robot.discovered_map[y1][x1] == 1:
                print("!!!LOOK HERE can calibrate right!!!!")
                return True
        return False

    def is_calibrate_side_possible(self):
        return False
#        return False
        #         W  W
        #    S 0, 1, 2
        #    S 3, 4, 5 N
        #      6, 7, 8 N
        #      E  E
        y,x=get_matrix_coords(self._robot.center)

        if self._robot.facing == NORTH:
#                5,2
            y1= y+1
            x1 = x+2
            if x1 > 14:
                return True
                
            y2= y
            x2 = x+2
 
        elif self._robot.facing == SOUTH:
#            6,3
            y1= y
            x1 = x-2
            if x1 < 0:
                return True

            y2= y-1
            x2 = x-2
            
        elif self._robot.facing == WEST:
          #  1,0
            y1= y+2
            x1 = x-1
            if y1 > 19:
                return True
                
            y2= y+2
            x2 = x

        elif self._robot.facing == EAST:
#            8,7
            y1= y-2
            x1 = x+1
            if y1 < 0:
                return True

            y2= y-2
            x2 = x
        
#        if self._robot.discovered_map[y1][x1] == 1 and self._robot.discovered_map[y2][x2] == 1:
##            print("checked: can calibrate side distance facing EAST")
#            return True
#        print("checked: cannot calibrate side distance facing EAST")
        return False
        
    def check_right_staircase(self):
        #           W  W
        #           0, 1, 2 N
        #         S 3, 4, 5 N
        #         S 6, 7, 8
        #              E  E
        
    #        print("facing: ")
    #        print(self._robot.facing)
        y,x=get_matrix_coords(self._robot.center)
        if self._robot.facing == NORTH:
            y1= y+1
            x1 = x+2
            if x1 > 13:
                return False
            y11 =y+1
            x11 = x+3
            
            y2= y
            x2 = x1
            y22 =y
            x22 = x+3
#            both must be explored
#            if self._robot.discovered_map[y1][x1] != 2 and self._robot.discovered_map[y2][x2] != 2:
#            middle cell got ostacle

#            y3= y-1
#            x3 = x1
#            if self._robot.discovered_map[y3][x3] != 0:
#                return False
#            return False


        elif self._robot.facing == SOUTH:
            y1= y-1
            x1 = x-2
            if x1 < 1:
                return False
            y11 = y-1
            x11 = x-3
#            if self._robot.discovered_map[y1][x1] != 0:
#                return False
                
            y2= y
            x2 = x1
            y22 = y
            x22 = x-3
#            if self._robot.discovered_map[y2][x2] != 0:
#                return False

#            y3= y+1
#            x3 = x1
#            if self._robot.discovered_map[y3][x3] != 0:
#                return False
#            return True

        elif self._robot.facing == WEST:
            y1= y+2
            x1 = x-1
            if y1 > 18:
                return False
            y11 = y+3
            x11 = x-1
#            if self._robot.discovered_map[y1][x1] != 0:
#                return False
                
            y2= y1
            x2 = x
            y22 = y+3
            x22 = x
#            if self._robot.discovered_map[y2][x2] != 0:
#                return False

#            y3= y1
#            x3 = x+1
#            if self._robot.discovered_map[y3][x3] != 0:
#                return False
#            return True

        elif self._robot.facing == EAST:
            y1= y-2
            x1 = x+1
            if y1 < 1:
                return False
            y11 = y-3
            x11 = x+1
#            if self._robot.discovered_map[y1][x1] != 0:
#                return False
                
            y2= y1
            x2 = x
            y22 = y-3
            x22 = x
#            if self._robot.discovered_map[y2][x2] != 0:
#                return False

#            y3= y1
#            x3 = x+1
#            if self._robot.discovered_map[y3][x3] != 0:
#                return False
#            return True
#            middle cell got obstacle
        if self._robot.discovered_map[y2][x2] == 1 and self._robot.discovered_map[y1][x1] == 0 and self._robot.discovered_map[y11][x11] == 1:
            return 'U'
            #            front cell got obstacle
        if self._robot.discovered_map[y1][x1] == 1 and self._robot.discovered_map[y2][x2] == 0 and self._robot.discovered_map[y22][x22] == 1:
            return 'V'
#            self._robot.discovered_map[y1][x1] != 0
        return False

#        return True

    def check_free_right(self):
        #           W  W
        #           0, 1, 2 N
        #         S 3, 4, 5 N
        #         S 6, 7, 8
        #              E  E
        
#        print("facing: ")
#        print(self._robot.facing)
        y,x=get_matrix_coords(self._robot.center)
        if self._robot.facing == NORTH:
            y1= y-1
            x1 = x+2
            if x1 > 14:
                return False
            if self._robot.discovered_map[y1][x1] != 0:
                return False
                
            y2= y
            x2 = x1
            if self._robot.discovered_map[y2][x2] != 0:
                return False

            y3= y+1
            x3 = x1
            if self._robot.discovered_map[y3][x3] != 0:
                return False
            return True


        elif self._robot.facing == SOUTH:
            y1= y-1
            x1 = x-2
            if x1 < 0:
                return False
            if self._robot.discovered_map[y1][x1] != 0:
                return False
                
            y2= y
            x2 = x1
            if self._robot.discovered_map[y2][x2] != 0:
                return False

            y3= y+1
            x3 = x1
            if self._robot.discovered_map[y3][x3] != 0:
                return False
            return True

        elif self._robot.facing == WEST:
            y1= y+2
            x1 = x-1
            if y1 > 19:
                return False
            if self._robot.discovered_map[y1][x1] != 0:
                return False
                
            y2= y1
            x2 = x
            if self._robot.discovered_map[y2][x2] != 0:
                return False

            y3= y1
            x3 = x+1
            if self._robot.discovered_map[y3][x3] != 0:
                return False
            return True

        elif self._robot.facing == EAST:
            y1= y-2
            x1 = x-1
            if y1 < 0:
                return False
            if self._robot.discovered_map[y1][x1] != 0:
                return False
                
            y2= y1
            x2 = x
            if self._robot.discovered_map[y2][x2] != 0:
                return False

            y3= y1
            x3 = x+1
            if self._robot.discovered_map[y3][x3] != 0:
                return False
            return True

        return True

    def check_free_front(self):
        #           N  N
        #           0, 1, 2 E
        #         W 3, 4, 5 E
        #         W 6, 7, 8
        #              S  S

        if self._robot.facing == EAST:
            #            print("calibrate right possible NORTH")
            y,x=get_matrix_coords(self._robot.center)
            y1= y-1
            x1 = x+2
            if x1 > 14:
                return False
            if self._robot.discovered_map[y1][x1] != 0:
                return False
                
            y2= y
            x2 = x+2
            if self._robot.discovered_map[y2][x2] != 0:
                return False

            y3= y+1
            x3 = x+2
            if self._robot.discovered_map[y3][x3] != 0:
                return False


        elif self._robot.facing == WEST:
            y,x=get_matrix_coords(self._robot.center)
            y1= y-1
            x1 = x-2
            if x1 < 0:
                return False
            if self._robot.discovered_map[y1][x1] != 0:
                return False
                
            y2= y
            x2 = x-2
            if self._robot.discovered_map[y2][x2] != 0:
                return False

            y3= y+1
            x3 = x-2
            if self._robot.discovered_map[y3][x3] != 0:
                return False

        elif self._robot.facing == NORTH:
            y,x=get_matrix_coords(self._robot.center)
            y1= y+2
            x1 = x-1
            if y1 > 19:
                return False
            if self._robot.discovered_map[y1][x1] != 0:
                return False
                
            y2= y+2
            x2 = x
            if self._robot.discovered_map[y2][x2] != 0:
                return False

            y3= y+2
            x3 = x+1
            if self._robot.discovered_map[y3][x3] != 0:
                return False

        elif self._robot.facing == SOUTH:
            y,x=get_matrix_coords(self._robot.center)
            y1= y-2
            x1 = x-1
            if y1 < 0:
                return False
            if self._robot.discovered_map[y1][x1] != 0:
                return False
                
            y2= y-2
            x2 = x
            if self._robot.discovered_map[y2][x2] != 0:
                return False

            y3= y-2
            x3 = x+1
            if self._robot.discovered_map[y3][x3] != 0:
                return False

        return True
    def left_image_detection_worthy(y,x):
        if self._robot.facing == EAST:
            if y<=7:
                return True
            return False
        elif self._robot.facing == NORTH:
            if x>=9:
                return True
            return False
        elif self._robot.facing == WEST:
            if y>=12:
                return True
            return False
        elif self._robot.facing == SOUTH:
            if x<=5:
                return True
            return False
        return False
        
#    def front_image_detection_worthy(self,y,x):
#        return True
#        if self._robot.facing == NORTH:
#            if y<=7:
#                return True
#            return False
#        elif self._robot.facing == WEST:
#            if x>=9:
#                return True
#            return False
#        elif self._robot.facing == SOUTH:
#            if y>=12:
#                return True
#            return False
#        elif self._robot.facing == EAST:
#            if x<=5:
#                return True
#            return False
#        return False
            


    ''' image detection at left '''

    def is_image_possible_left(self):
        #         E  E
        #    N 0, 1, 2
        #    N 3, 4, 5 S
        #      6, 7, 8 S
        #      W  W
        #         y, x = get_matrix_coords(self._robot.center)
        y,x=get_matrix_coords(self._robot.center)
        worthy = self.left_image_detection_worthy(y=y,x=x)
        if worthy == False:
            return False
        if self._robot.facing == SOUTH:
            #            print("turn every 3 steps NORTH")
            #5,8
            y1= y
            x1 = x+3
            if x1 > 14:
                return False
                
            y2= y-1
            x2 = x+3

        elif self._robot.facing == NORTH:
#            3,0
            y1= y
            x1 = x-3
            if x1 < 0:
              return False

            y2= y+1
            x2 = x-3

        elif self._robot.facing == EAST:
#            1,2
            y1= y+3
            x1 = x
            if y1 > 19:
              return False
              
            y2= y+3
            x2 = x+1

        elif self._robot.facing == WEST:
#            7,6
            y1= y-3
            x1 = x
            if y1 < 0:
             return False

            y2= y-3
            x2 = x-1

        facing = (self._robot.facing + LEFT) % 4
        obstacle1 = 0,0,0,2
        obstacle2 = 0,0,0,2
        if (self._robot.discovered_map[y1][x1] == 1):
            if (facing not in self._robot.obstacle_image_dic[y1, x1]) and (-1 not in self._robot.obstacle_image_dic[y1, x1]):
                obstacle1 = y1,x1,facing, 2
        if (self._robot.discovered_map[y2][x2] == 1):
            if (facing not in self._robot.obstacle_image_dic[y2, x2]) and (-1 not in self._robot.obstacle_image_dic[y2, x2]):
                obstacle2 = y2,x2,facing, 2
        if obstacle1 == (0, 0, 0, 2) and obstacle2 == (0, 0, 0, 2):
            return False
        elif obstacle1 != (0, 0, 0, 2) and obstacle2 != (0, 0, 0, 2):
            return obstacle1,obstacle2
        elif obstacle1 != (0, 0, 0, 2):
            return obstacle1
        else:
            return obstacle2


    ''' image detection at front '''

    def is_image_possible_left(self):
        #         N  N
        #    W 0, 1, 2
        #    W 3, 4, 5 E
        #      6, 7, 8 E
        #      S  S
        #         y, x = get_matrix_coords(self._robot.center)
        y,x=get_matrix_coords(self._robot.center)
        worthy=self.left_image_detection_worthy(y=y,x=x)
        if not worthy:
            return False
        if self._robot.facing == SOUTH:
            #5,8
            y1= y
            x1 = x+3
            if x1 > 14:
                return False
                
            y2= y-1
            x2 = x+3

        elif self._robot.facing == NORTH:
#        3,0
            y1= y
            x1 = x-3
            if x1 < 0:
              return False

            y2= y+1
            x2 = x-3

        elif self._robot.facing == EAST:
#        1,2
            y1= y+3
            x1 = x
            if y1 > 19:
              return False
              
            y2= y+3
            x2 = x+1


        elif self._robot.facing == WEST:
            #7,6
            y1= y-3
            x1 = x
            if y1 < 0:
             return False

            y2= y-3
            x2 = x-1
            
        #              either one is obstacle & that face is not scanned:
        facing = self._robot.facing
        obstacle1 = 0,0,0
        obstacle2 = 0,0,0
        obstacles = [(0,0,0),(0,0,0)]
        if (self._robot.discovered_map[y1][x1] == 1):
            if (facing not in self._robot.obstacle_image_dic[y1, x1]) and (-1 not in self._robot.obstacle_image_dic[y1, x1]):
                obstacles[0] = y1,x1,facing
        if (self._robot.discovered_map[y2][x2] == 1):
            if (facing not in self._robot.obstacle_image_dic[y2, x2]) and (-1 not in self._robot.obstacle_image_dic[y2, x2]):
                obstacles[1] = y2,x2,facing
        if obstacles == [(0,0,0),(0,0,0)]:
            obstacles = self.is_image_possible_left_3cells_away(facing=facing,y=y,x=x)
            if obstacles != False:
                return obstacles
            return False
        return obstacles
    def is_image_possible_front(self):
        #         N  N
        #    W 0, 1, 2
        #    W 3, 4, 5 E
        #      6, 7, 8 E
        #      S  S
        #         y, x = get_matrix_coords(self._robot.center)
        y,x=get_matrix_coords(self._robot.center)
        worthy=self.front_image_detection_worthy(y=y,x=x)
        if worthy == False:
            return False
        if self._robot.facing == EAST:
            #5,8
            y1= y
            x1 = x+3
            if x1 > 14:
                return False
                
            y2= y-1
            x2 = x+3

        elif self._robot.facing == WEST:
#        3,0
            y1= y
            x1 = x-3
            if x1 < 0:
              return False

            y2= y+1
            x2 = x-3

        elif self._robot.facing == NORTH:
#        1,2
            y1= y+3
            x1 = x
            if y1 > 19:
              return False
              
            y2= y+3
            x2 = x+1


        elif self._robot.facing == SOUTH:
            #7,6
            y1= y-3
            x1 = x
            if y1 < 0:
             return False

            y2= y-3
            x2 = x-1
            
        #              either one is obstacle & that face is not scanned:
        facing = self._robot.facing
        obstacle1 = 0,0,0
        obstacle2 = 0,0,0
        obstacles = [(0,0,0),(0,0,0)]
        if (self._robot.discovered_map[y1][x1] == 1):
            if (facing not in self._robot.obstacle_image_dic[y1, x1]) and (-1 not in self._robot.obstacle_image_dic[y1, x1]):
                obstacles[0] = y1,x1,facing
        if (self._robot.discovered_map[y2][x2] == 1):
            if (facing not in self._robot.obstacle_image_dic[y2, x2]) and (-1 not in self._robot.obstacle_image_dic[y2, x2]):
                obstacles[1] = y2,x2,facing
        if obstacles == [(0,0,0),(0,0,0)]:
            obstacles = self.is_image_possible_front_3cells_away(facing=facing,y=y,x=x)
            if obstacles != False:
                return obstacles
            return False
        return obstacles
    
    def is_image_possible_left_3cells_away(self,facing,y,x):
        print("inside is_image_possible_front_3cells_away func")
            #         N  N
            #    W 0, 1, 2
            #    W 3, 4, 5 E
            #      6, 7, 8 E
            #      S  S

        if self._robot.facing == SOUTH:
            #5,8
            y1= y
            x1 = x+4
            if x1 > 14:
                return False
                
            y2= y-1
            x2 = x+4
            
            y3= y+1
            x3 = x+4
            

        elif self._robot.facing == NORTH:
#        3,0
            y1= y
            x1 = x-4
            if x1 < 0:
              return False

            y2= y+1
            x2 = x-4
            
            y3= y-1
            x3 = x-4

        elif self._robot.facing == EAST:
#        1,2
            y1= y+4
            x1 = x
            if y1 > 19:
              return False
              
            y2= y+4
            x2 = x+1
            
            y3= y+4
            x3 = x-1


        elif self._robot.facing == WEST:
            #7,6
            y1= y-4
            x1 = x
            if y1 < 0:
             return False

            y2= y-4
            x2 = x-1
            
            y3= y-4
            x3 = x+1
            
        #              either one is obstacle & that face is not scanned:
#        facing = self._robot.facing
        obstacle1 = 0,0,0
        obstacle2 = 0,0,0
        obstacle3 = 0,0,0
        obstacles = [(0,0,0),(0,0,0),(0,0,0)]
        if (self._robot.discovered_map[y1][x1] == 1):
            if (facing not in self._robot.obstacle_image_dic[y1, x1]) and (-1 not in self._robot.obstacle_image_dic[y1, x1]):
                obstacles[0] = y1,x1,facing
                print("here there is obstacle 3 cells away")
#                return obstacle1
        if (self._robot.discovered_map[y2][x2] == 1):
            if (facing not in self._robot.obstacle_image_dic[y2, x2]) and (-1 not in self._robot.obstacle_image_dic[y2, x2]):
                obstacles[1] = y2,x2,facing
#                return obstacle2
        if (self._robot.discovered_map[y3][x3] == 1):
            if (facing not in self._robot.obstacle_image_dic[y3, x3]) and (-1 not in self._robot.obstacle_image_dic[y3, x3]):
                obstacles[2] = y3,x3,facing
#                return obstacle3
        if obstacles == [(0,0,0),(0,0,0),(0,0,0)]:
            return False
        return obstacles
    
    def is_image_possible_front_3cells_away(self,facing,y,x):
        print("inside is_image_possible_front_3cells_away func")
            #         N  N
            #    W 0, 1, 2
            #    W 3, 4, 5 E
            #      6, 7, 8 E
            #      S  S

        if self._robot.facing == EAST:
            #5,8
            y1= y
            x1 = x+4
            if x1 > 14:
                return False
                
            y2= y-1
            x2 = x+4
            
            y3= y+1
            x3 = x+4
            

        elif self._robot.facing == WEST:
#        3,0
            y1= y
            x1 = x-4
            if x1 < 0:
              return False

            y2= y+1
            x2 = x-4
            
            y3= y-1
            x3 = x-4

        elif self._robot.facing == NORTH:
#        1,2
            y1= y+4
            x1 = x
            if y1 > 19:
              return False
              
            y2= y+4
            x2 = x+1
            
            y3= y+4
            x3 = x-1


        elif self._robot.facing == SOUTH:
            #7,6
            y1= y-4
            x1 = x
            if y1 < 0:
             return False

            y2= y-4
            x2 = x-1
            
            y3= y-4
            x3 = x+1
            
        #              either one is obstacle & that face is not scanned:
#        facing = self._robot.facing
        obstacle1 = 0,0,0
        obstacle2 = 0,0,0
        obstacle3 = 0,0,0
        obstacles = [(0,0,0),(0,0,0),(0,0,0)]
        if (self._robot.discovered_map[y1][x1] == 1):
            if (facing not in self._robot.obstacle_image_dic[y1, x1]) and (-1 not in self._robot.obstacle_image_dic[y1, x1]):
                obstacles[0] = y1,x1,facing
                print("here there is obstacle 3 cells away")
#                return obstacle1
        if (self._robot.discovered_map[y2][x2] == 1):
            if (facing not in self._robot.obstacle_image_dic[y2, x2]) and (-1 not in self._robot.obstacle_image_dic[y2, x2]):
                obstacles[1] = y2,x2,facing
#                return obstacle2
        if (self._robot.discovered_map[y3][x3] == 1):
            if (facing not in self._robot.obstacle_image_dic[y3, x3]) and (-1 not in self._robot.obstacle_image_dic[y3, x3]):
                obstacles[2] = y3,x3,facing
#                return obstacle3
        if obstacles == [(0,0,0),(0,0,0),(0,0,0)]:
            return False
        return obstacles
            

    def recognize_image(self,numOfCells):
        self._sender.send_rpi(str(numOfCells))
        result = self._sender.wait_rpi()
#        if result[
        return result


    ''' func to check image for 1 cell '''
    def check_single_obstacle_for_image(self,obstacle):
        #        ask rpi to detect
        y, x, facing,num = obstacle
        print("Asking RPI for single image from 1/2 cell away")
        image_ID = self.recognize_image(num)
        #        if image is detected
        if image_ID != '0':
            #            ID,11,9,10;1,18,0;2,18,5
            self._sender.send_android('ID' + ',' + image_ID + ',' + str(x) + ',' + str(y))
            #                mark its neighourhood as obsolete , wont check anymore
            self._robot._mark_neigbourhood_obsolete(y, x)
        else:
            self._robot.obstacle_image_dic[y, x].append(facing)
            
            ''' func to check image for 2 neighbour cells '''
    def check_obstacles_for_image(self,obstacles):
        #        ask rpi to detect
    #    y, x, facing = obstacle
        print("Asking RPI for images from 2 cells away")
        image_ID = self.recognize_image(len(obstacles))
        #        if image is detected
        if image_ID != '0':
        
            obstacle_index,image_ID =image_ID.split(',')
            obstacle_index = int(obstacle_index)
#            single_obstacle = False
#           no obstacle at the index, use its neighbour obstacle
            if obstacles[obstacle_index] == (0,0,0):
                for (y,x,facing) in obstacles:
                    if (y,x,facing) != (0,0,0):
                        self._sender.send_android('ID' + ',' + image_ID + ',' + str(x) + ',' + str(y))
                        self._robot._mark_neigbourhood_obsolete(y, x)
            else:
                y,x,facing = obstacles[obstacle_index]
                self._sender.send_android('ID' + ',' + image_ID + ',' + str(x) + ',' + str(y))
                self._robot._mark_neigbourhood_obsolete(y, x)
    #        image not detected
        else:
            for y,x,facing in obstacles:
                if (y,x,facing)!=(0,0,0):
                    self._robot.obstacle_image_dic[y, x].append(facing)
        
        
    def _check_obstacle_image_right(self):
#                ''' right '''
        #       image_possible: 1.False; 2.y,x,facing
        useful_obstacle_right = self.is_image_possible_right()
        #        possible image at obstacle surface, ask rpi to detect
        if useful_obstacle_right != False:
            self.check_single_obstacle_for_image(useful_obstacle_right)

    def _check_obstacle_image_front(self):
        print("start of check image front")
#            ''' front '''
        obstacles = self.is_image_possible_front()
        if obstacles != False:
            self._robot.turn_robot(self._sender, LEFT)
            self._turn_head(self._facing, LEFT)
            self._update_android()
#            print('Obstacle(s) detected')
#            print(obstacles)
            self.check_obstacles_for_image(obstacles)
            
            self._robot.turn_robot(self._sender, RIGHT)
            self._turn_head(self._facing, RIGHT)
            self._update_android()
        print("end of check image front")
            
#            if len(useful_obstacles_front) == 3:
#                print("length == 3")
#            self.check_single_obstacle_for_image(useful_obstacles_front)
#            else:
#                print("length != 3")
##                print(useful_obstacles_front[0])
##                print(useful_obstacles_front[1])
#                self.check_obstacles_for_image(useful_obstacles_front[0],useful_obstacles_front[1])
        
#        else:
#            image_possible_front_3cells_away = self.is_image_possible_front_3cells_away()
#            if self.
#
                
            
            
    def _check_obstacle_image_left(self):
#           ''' left '''
        useful_obstacles_left = self.is_image_possible_left()
        if useful_obstacles_left != False:
            self._robot.turn_robot(self._sender, BACKWARD)
            self._turn_head(self._facing, BACKWARD)
            self._update_android()
            
#            if len(useful_obstacles_left) == 3:
            self.check_single_obstacle_for_image(useful_obstacles_left)
#            else:
#                self.check_obstacles_for_image(useful_obstacles_left[0],useful_obstacles_left[1])
                
            self._robot.turn_robot(self._sender, BACKWARD)
            self._turn_head(self._facing, BACKWARD)
            self._update_android()


    ''' image detection at right only 1 obstacle'''
    def is_image_possible_right(self):
        #         W  W
        #    S 0, 1, 2
        #    S 3, 4, 5 N
        #      6, 7, 8 N
        #      E  E
        y, x = get_matrix_coords(self._robot.center)
        
        if self._robot.facing == NORTH:
#        5,8
            y1= y
            x1 = x+2
            if x1 > 14:
                return False
            y2= y-1
            x2 = x+2
  
        elif self._robot.facing == SOUTH:
#        3,0
            y1= y
            x1 = x-2
            if x1 < 0:
                return False
            y2= y+1
            x2 = x-2
        #         y2,x2 = get_matrix_coords(get_robot_cells(self._robot.center)[0])
        #         x2 -= 1

        elif self._robot.facing == WEST:
#           1,2
            y1= y+2
            x1 = x
            if y1 > 19:
                return False
            y2= y+2
            x2 = x+1

        elif self._robot.facing == EAST:
#            7,6
            y1= y-2
            x1 = x
            if y1 < 0:
                return False
            y2= y-2
            x2 = x-1
                
        #         y2,x2 = get_matrix_coords(get_robot_cells(self._robot.center)[6])
        #         y2 -= 1

        #                obstacle
        #     if self._robot.discovered_map[y1][x1] == 1:
        ##         if self._robot.discovered_map[y1][x1] == 1:
        #         return True,
        facing = (self._robot.facing + RIGHT) % 4
        #     obstacle & surface has not been scanned & no image detected so far in its neighbourhood
#        if (self._robot.discovered_map[y1][x1] == 1) and (facing not in self._robot.obstacle_image_dic[y1, x1]) and (
#                -1 not in self._robot.obstacle_image_dic[y1, x1]):
#            return y1,x1,facing
        if (self._robot.discovered_map[y1][x1] == 1):
            if (facing not in self._robot.obstacle_image_dic[y1, x1]) and (-1 not in self._robot.obstacle_image_dic[y1, x1]):
                return y1,x1,facing,1
        elif (self._robot.discovered_map[y2][x2] == 1):
            if (facing not in self._robot.obstacle_image_dic[y2, x2]) and (-1 not in self._robot.obstacle_image_dic[y2, x2]):
                return y2,x2,facing,1
        return False

    #    if self.is_image_possible_right():
    #        take_photo(sender,1)
    #        print('Arrow Possible @ Robot Position: {}'.format((x, y, DIRECTIONS[self.facing])))
    #        position = '%s,%s,%s' % (y, x, self.facing)
    #
    #    if self.is_image_possible_front():
    #
    #    if self.is_image_possible_left():
    #    else:
    #    print('Arrow Not Possible @ Robot Position: {}'.format((x, y, DIRECTIONS[self.facing])))
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
