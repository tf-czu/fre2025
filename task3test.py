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
                    continue  # ← zde byla chyba, nyní opraveno

                dist = np.median(roi[mask]) / 1000
                x_fruit = self.pose_xy[0] + dist * math.cos(alpha + beta)
                y_fruit = self.pose_xy[1] + dist * math.sin(alpha + beta)

                self.fruits.append((fruit_type, x_fruit, y_fruit))
                print(self.time, fruit_type, x_fruit, y_fruit)
                new_detections.append(det)

        if new_detections:
            clustered = cluster(self.fruits, radius=0.2)
            self.save_csv_if_enabled(clustered)

    def save_csv_if_enabled(self, fruits):
        if self.output_csv_enabled:
            filename = "CULS-Robotics-task3.csv"
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['fruit_type', 'x', 'y'])
                writer.writerows(fruits)
            print(f"Uloženo do souboru: {filename}")

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )

    def drive_full_circle(self, radius):
        print(self.time, f'drive_full_circle r={radius}')
        steering_angle = self.max_speed / radius
        total_distance = 2 * math.pi * radius
        dist = 0
        prev = self.pose_xy

        while dist < total_distance:
            channel = self.update()
            if channel == 'pose2d':
                self.send_speed_cmd(self.max_speed, steering_angle)
                dist += math.hypot(prev[0] - self.pose_xy[0], prev[1] - self.pose_xy[1])
                prev = self.pose_xy

        self.send_speed_cmd(0, 0)

    def draw(self):
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots()

        if self.trajectory:
            x_vals = [p[0] for p in self.trajectory]
            y_vals = [p[1] for p in self.trajectory]
            ax.plot(x_vals, y_vals, linestyle='-', marker='.', label='Trajektorie robota')

        ax.set_title("Trajektorie jízdy robota")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.axis('equal')
        ax.grid(True)
        plt.show()

    def run(self):
        try:
            self.drive_full_circle(1.0)
        except BusShutdownException:
            self.send_speed_cmd(0, 0)
        finally:
            self.draw()
