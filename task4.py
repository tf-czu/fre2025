import math
import numpy as np
from osgar.node import Node
from osgar.bus import BusShutdownException
from task1switch import Task1  # zdědí základní pohybové schopnosti

class Task4(Task1):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        bus.register('detections')
        self.detections = None
        self.fruit_types = {"apple", "banana", "lemon", "orange", "grape"}
        self.detected_fruits = []

    def on_detections(self, data):
        if self.time.total_seconds() < 5:
            return
        for det in data:
            fruit_type = det[0]
            if fruit_type in self.fruit_types:
                print(f"{self.time} Detekováno ovoce: {fruit_type}")
                self.detected_fruits.append((self.time.total_seconds(), fruit_type))

    def go_straight(self, dist):
        print(self.time, f'Jedu rovně {dist} m')
        start = self.pose_xy
        while True:
            if self.update() == 'pose2d':
                dx = self.pose_xy[0] - start[0]
                dy = self.pose_xy[1] - start[1]
                if math.hypot(dx, dy) < dist:
                    self.send_speed_cmd(self.max_speed, 0)
                else:
                    self.send_speed_cmd(0, 0)
                    break

    def turn_left_90deg(self):
        print(self.time, 'Otáčím se doleva o 90°')
        start_angle = self.pose_angle
        target = (start_angle + math.radians(90)) % (2 * math.pi)
        while True:
            if self.update() == 'pose2d':
                diff = (target - self.pose_angle + math.pi) % (2 * math.pi) - math.pi
                if abs(diff) < math.radians(3):
                    break
                self.send_speed_cmd(0, 1.0 * diff)
        self.send_speed_cmd(0, 0)

    def run(self):
        try:
            length = 9.0
            while length > 0:
                for _ in range(3):
                    self.go_straight(length)
                    self.turn_left_90deg()
                length -= 1.0
            self.send_speed_cmd(0, 0)
        except BusShutdownException:
            self.send_speed_cmd(0, 0)