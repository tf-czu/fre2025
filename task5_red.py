import math
import numpy as np
from osgar.node import Node
from osgar.bus import BusShutdownException
from task1 import Task1  # zdědí základní pohybové schopnosti

class Task5RED(Task1):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("fruit_coordinate")

    def on_detections(self, data):
        if self.time.total_seconds() < 15:
            return
        for det in data:
            if det[0] == "strawberry":
                x1, y1, x2, y2 = det[2]
                if x1 < 0.05 or x2 > 0.95 or y1 < 0.5 or y2 > 0.99:
                    continue
                x_center = (x1 + x2) / 2
                beta = (0.5 - x_center) * math.radians(69)
                robot_heading = self.pose_angle
                mask = self.depth[int(y1 * 400): int(y2 * 400), int(x1 * 640): int(x2 * 640)] != 0
                if not np.any(mask):
                    continue
                dist = np.median(self.depth[int(y1 * 400): int(y2 * 400), int(x1 * 640): int(x2 * 640)][mask]) / 1000
                if dist < 1.0:
                    x_fruit = self.pose_xy[0] + dist * math.cos(robot_heading + beta)
                    y_fruit = self.pose_xy[1] + dist * math.sin(robot_heading + beta)

                    print("Fruit detected!! ", x_fruit, y_fruit)
                    self.publish("fruit_coordinate", [x_fruit, y_fruit])

    def run(self):
        try:
            self.go_straight(4)

        except BusShutdownException:
            self.send_speed_cmd(0, 0)
