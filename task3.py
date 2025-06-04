"""
  FRE2025 - TASK3
"""
import math
import numpy as np
import csv

from osgar.node import Node
from osgar.bus import BusShutdownException
from task1 import Task1

def cluster(points, radius=0.3):
    s_points = []
    for p in points:
        for sp in s_points:
            if math.hypot(p[1] - sp[1], p[2] - sp[2]) < radius:
                break
        else:
            s_points.append(p)
    return s_points


class Task3(Task1):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')  # výstup
        self.detections = None
        self.fruits = []
        self.output_csv_enabled = config.get('outputcsv', True)
        self.save_csv_if_enabled([]) #create empty file

    def on_detections(self, data):
        if self.time.total_seconds() < 5:
            return

        camera_height = 0.25  # výška kamery nad zemí v metrech, upravit dle potřeby
        vertical_fov = math.radians(55)  # vertikální zorné pole kamery
        camera_tilt = math.radians(40) #naklonění kamery, upravit úhel

        fruit_types = {"apple", "banana", "lemon", "orange", "grape"}
        fruit = []
        for det in data:
            fruit_type = det[0]
            if fruit_type in fruit_types:
                x1, y1, x2, y2 = det[2]
                x_center = (x1 + x2) / 2
                y_center = (y1 + y2) / 2
                beta = (0.5 - x_center) * math.radians(69)
                alpha = self.pose_angle
                mask = self.depth[int(y1 * 400): int(y2 * 400), int(x1 * 640): int(x2 * 640)] != 0
                if not np.any(mask):
                    continue
                dist = np.median(self.depth[int(y1 * 400): int(y2 * 400), int(x1 * 640): int(x2 * 640)][mask]) / 1000
                x_fruit = self.pose_xy[0] + dist * math.cos(alpha + beta)
                y_fruit = self.pose_xy[1] + dist * math.sin(alpha + beta)
                theta = camera_tilt + (0.5 - y_center) * vertical_fov
                z_fruit = camera_height - dist * math.sin(theta)
                self.fruits.append((fruit_type, x_fruit, y_fruit, z_fruit))
                print(self.time, fruit_type, x_fruit, y_fruit, z_fruit)
                fruit.append(det)

        self.detections = fruit
        if len(self.detections) > 0:
            radius = 0.2
            center = cluster(self.fruits, radius)
            self.save_csv_if_enabled(center)

    def drive_full_circle(self):
        print(self.time, 'drive_full_circle: rovně → kružnice (otáčení o 360°)')
        steering_angle = math.radians(16.7)
        straight_distance = 1.0  # rovná jízda: 1 m

        dist = 0.0
        prev = self.pose_xy

        while dist < straight_distance:
            channel = self.update()
            if channel == 'pose2d':
                self.send_speed_cmd(self.max_speed, 0.0)
                dx = self.pose_xy[0] - prev[0]
                dy = self.pose_xy[1] - prev[1]
                dist += math.hypot(dx, dy)
                prev = self.pose_xy

        start_angle = self.pose_angle
        current_angle = start_angle
        turned = 0.0

        while abs(turned) < 2 * math.pi: 
            channel = self.update()
            if channel == 'pose2d':
                self.send_speed_cmd(self.max_speed, steering_angle)
                prev_angle = current_angle
                current_angle = self.pose_angle
                delta = (current_angle - prev_angle + math.pi) % (2 * math.pi) - math.pi
                turned += delta

        self.send_speed_cmd(0, 0)

    def save_csv_if_enabled(self, centroid):
        if self.output_csv_enabled:
            filename = "CULS-Robotics-task3.csv"
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['fruit_type', 'x', 'y', 'z'])
                writer.writerows(centroid)
            print(f"Uloženo do souboru: {filename}")

    def run(self):
        try:
            self.drive_full_circle()
        except BusShutdownException:
            self.send_speed_cmd(0, 0)

# vim: expandtab sw=4 ts=4
