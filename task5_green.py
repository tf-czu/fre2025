"""
  FRE2025 - TASK2
"""
import math
import numpy as np
import csv

from osgar.node import Node
from osgar.bus import BusShutdownException
from task2 import Task2


class Task5GREEN(Task2):

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

            
    def run(self):
            try:
                self.wait_for_fruit_coordinate()
                self.drive_to_point(self.fruit_coordinate, 0.5)
                self.send_sprayer(True, True, True)
                self.go_straight(1)
                self.send_speed_cmd(0, 0)
            except BusShutdownException:
                self.send_speed_cmd(0, 0)



# vim: expandtab sw=4 ts=4
