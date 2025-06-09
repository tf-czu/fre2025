import math
import numpy as np
from osgar.node import Node
from osgar.bus import BusShutdownException
from task1 import Task1  # zdědí základní pohybové schopnosti

class Task5(Task1):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering', 'sprayer')

    def on_detections(self, data):
        pass

    def send_sprayer(self, sirene, left, right):
        if sirene:
            self.publish('sprayer', b'*B1OS1H\r')
        else:
            self.publish('sprayer', b'*B1OS1L\r')
        if left:
            self.publish('sprayer', b'*B1OS2H\r')
        else:
            self.publish('sprayer', b'*B1OS2L\r')
        if right:
            self.publish('sprayer', b'*B1OS3H\r')
        else:
            self.publish('sprayer', b'*B1OS3L\r')

    def run(self):
        try:
            self.send_speed_cmd(0, 0)
            self.wait(1.0)
            self.send_sprayer(True, False, False)
            self.wait(1.0)
            self.send_sprayer(False, False, False)
            self.wait(1.0)
        except BusShutdownException:
            self.send_speed_cmd(0, 0)
