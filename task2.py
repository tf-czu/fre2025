"""
  FRE2025 - TASK2
"""
import math

from osgar.node import Node
from osgar.bus import BusShutdownException
from task1 import Task1


class Task2(Task1):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.detections = None

    def on_detections(self, data):
        fruit = []
        for det in data:
            if det [0] == "banana":
                print (det)
                fruit.append(det)
        self.detections = fruit
   



# vim: expandtab sw=4 ts=4
