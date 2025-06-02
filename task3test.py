import math
import numpy as np
import csv
from osgar.node import Node
from osgar.bus import BusShutdownException


def cluster(points, radius=0.3):
    s_points = []
    for p in points:
        for sp in s_points:
            if math.hypot(p[1] - sp[1], p[2] - sp[2]) < radius:
                break
        else:
            s_points.append(p)
    return s_points


class Task3test(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.bus.register('desired_steering')
        self.bus.register('pose2d')
        self.bus.register('depth')
        self.bus.register('detections')

        self.pose_xy = (0, 0)
        self.pose_angle = 0
        self.depth = None
        self.fruits = []
        self.trajectory = []  # seznam pozic pro vykreslení
        self.output_csv_enabled = config.get('outputcsv', True)
        self.max_speed = config.get('max_speed', 0.2)

    def on_pose2d(self, data):
        self.pose_xy = data[0] / 1000, data[1] / 1000
        self.pose_angle = math.radians(data[2] / 100)
        self.trajectory.append(self.pose_xy)

    def on_depth(self, data):
        self.depth = np.array(data).reshape((400, 640))

    def on_detections(self, data):
        if self.time.total_seconds() < 5 or self.depth is None:
            return

        fruit_types = {"apple", "banana", "lemon", "orange", "grape"}
        new_detections = []

        for det in data:
            fruit_type = det[0]
            if fruit_type in fruit_types:
                x1, y1, x2, y2 = det[2]
                x_center = (x1 + x2) / 2
                y_center = (y1 + y2) / 2
                beta = (0.5 - x_center) * math.radians(69)
                alpha = self.pose_angle

                roi = self.depth[int(y1 * 400): int(y2 * 400), int(x1 * 640): int(x2 * 640)]
                mask = roi != 0
                if not np.any(mask):
