"""
  FRE2025 - TASK2
"""
import math
import numpy as np
import csv

from osgar.node import Node
from osgar.bus import BusShutdownException
from task3 import Task3


class Task5GREEN(Task3):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.fruit_coordinate = None
        self.enabled = True

    def on_fruit_coordinate(self,data):
        self.fruit_coordinate = data

        
    def wait_for_fruit_coordinate(self):
        while self.fruit_coordinate is None:
            print('čekám na data')
            self.wait(0.5)

    def go_straight(self, dist):
        print(self.time, 'go_straight')
        self.end_of_row = self.pose_xy
        while True:
            if self.update() == 'pose2d':
                if math.hypot(self.end_of_row[0] - self.pose_xy[0],
                              self.end_of_row[1] - self.pose_xy[1]) < dist:
                    self.send_speed_cmd(0.3, 0)
                else:
                    self.send_speed_cmd(0, 0)
                    break 

            
    def run(self):
            try:
                self.wait_for_fruit_coordinate()
                self.drive_to_point(self.fruit_coordinate, 0.5)
                self.send_sprayer(True, False, False)
                self.go_straight(1)
                self.send_speed_cmd(0, 0)
                self.send_sprayer(False, False, False)
            except BusShutdownException:

                self.send_speed_cmd(0, 0)
            self.send_sprayer(False, False, False)



# vim: expandtab sw=4 ts=4
