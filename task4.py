import math
import numpy as np
from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.platforms.matty import FRONT_REAR_AXIS_DISTANCE
from task1switch import Task1  # zdědí základní pohybové schopnosti

class Task4(Task1):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        bus.register('detections')
        self.side_length = config.get('side_length', 3.0)
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
        print(self.time, 'Zatáčím vlevo o 90° během jízdy')
        self.send_speed_cmd(0, 0)
        self.wait(2)
        dist = 0
        prev = self.pose_xy
        joint_angle = math.radians(45)
        radius = (FRONT_REAR_AXIS_DISTANCE/2) / math.tan(joint_angle/2)
        arc_length = (math.pi * radius / 2)  # původně pro 180°, teď polovina pro 90°
        while dist < arc_length:
            if self.update() == 'pose2d':
                self.send_speed_cmd(self.max_speed, joint_angle)
                dist += math.hypot(prev[0] - self.pose_xy[0],
                                   prev[1] - self.pose_xy[1])
                prev = self.pose_xy
        self.send_speed_cmd(0, 0)
        self.wait(2)

    def wait(self,duration):
        self.update()
        start_time=self.time
        while self.time - start_time < timedelta(seconds=duration):
            self.update()

    def run(self):
        try:
            length = self.side_length
            while length > 0:
                for _ in range(3):
                    self.go_straight(length)
                    self.turn_left_90deg()
                length -= 1.0
            self.send_speed_cmd(0, 0)
        except BusShutdownException:
            self.send_speed_cmd(0, 0)
