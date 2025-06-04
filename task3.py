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
        bus.register('desired_steering')
        self.detections = None
        self.fruits = []
        self.output_csv_enabled = config.get('outputcsv', True)
        self.start_angle = None
        self.save_csv_if_enabled([])  # vytvoř prázdný soubor

    def on_detections(self, data):
        if self.time.total_seconds() < 5:
            return

        camera_height = 0.25
        vertical_fov = math.radians(55)
        camera_tilt = math.radians(40)

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

    def drive_to_point(self, target, tolerance=0.05):
        tx, ty = target
        while True:
            self.update()
            x, y = self.pose_xy
            dx = tx - x
            dy = ty - y
            dist = math.hypot(dx, dy)
            if dist < tolerance:
                break
            heading = math.atan2(dy, dx)
            self.send_speed_cmd(self.max_speed, heading - self.pose_angle)
        self.send_speed_cmd(0, 0)

    def turn_to_angle(self, target_angle, tolerance=math.radians(3)):
        while True:
            self.update()
            diff = target_angle - self.pose_angle
            diff = (diff + math.pi) % (2 * math.pi) - math.pi
            if abs(diff) < tolerance:
                break
            angular_speed = 0.5 * diff
            self.send_speed_cmd(0, angular_speed)
        self.send_speed_cmd(0, 0)

    def drive_circle_around_point(self, center):
        print(self.time, f"Objíždím strom na {center}")
        steering_angle = math.radians(16.7)
        radius = self.max_speed / steering_angle

        cx, cy = center
        sx, sy = self.pose_xy
        dx = sx - cx
        dy = sy - cy
        dist_to_center = math.hypot(dx, dy)

        if dist_to_center < 1e-3:
            print("Robot je ve stredu kružnice, nelze určit vstupní bod.")
            return

        ux = dx / dist_to_center
        uy = dy / dist_to_center
        entry_x = cx + ux * radius
        entry_y = cy + uy * radius
        entry_point = (entry_x, entry_y)

        self.drive_to_point(entry_point)
        tangent_angle = math.atan2(ux, -uy)
        self.turn_to_angle(tangent_angle)

        self.start_angle = self.pose_angle
        total_rotated = 0.0
        prev_angle = self.start_angle

        while total_rotated < 2 * math.pi:
            channel = self.update()
            if channel == 'pose2d':
                self.send_speed_cmd(self.max_speed, steering_angle)
                current = self.pose_angle
                diff = (current - prev_angle + math.pi) % (2 * math.pi) - math.pi
                total_rotated += abs(diff)
                prev_angle = current

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
            trees = [(1.0, 1.0), (3.0, 1.0)]  # pozice stromů vůči robotovi
            for tree in trees:
                self.drive_circle_around_point(tree)
        except BusShutdownException:
            self.send_speed_cmd(0, 0)
