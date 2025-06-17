"""
  FRE2025 - TASK2
"""
import math
import numpy as np
import csv

from osgar.node import Node
from osgar.bus import BusShutdownException
from task1 import Task1


def cluster(points, radius = 0.3):
    s_points = []
    for p in points:
        for sp in s_points:
            if math.hypot(p[0] - sp[0], p[1] - sp[1]) < radius:
                break
        else:
            s_points.append(p)
    return s_points



class TaskPrezentace(Task1):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('sprayer')
        self.detections = None
        self.fruits = []
        self.output_csv_enabled = config.get('outputcsv', True)
        self.save_csv_if_enabled([]) #create empty file
        self.center = None
        self.prev_pose = (0, 0)

    def on_detections(self, data):
        if self.time.total_seconds() < 5:
            return
        fruit = []
        for det in data:
            if det [0] == "strawberry":
                x1, y1, x2, y2 = det[2]
                x_center = (x1 + x2) / 2
                y_center = (y1 + y2) / 2
                beta = (0.5 - x_center) * math.radians(69)
                alpha = self.pose_angle
                mask = self.depth[int(y1 * 400) : int(y2 * 400), int(x1 * 640) : int(x2 * 640)] != 0
                if mask.shape[0] > 0 and mask.shape[1] > 0 and mask.max():
                    dist = np.median(self.depth[int(y1 * 400) : int(y2 * 400), int(x1 * 640) : int(x2 * 640)][mask])/1000
                    if dist < 1.0:
                        x_fruit = self.pose_xy [0] + dist * math.cos(alpha + beta)
                        y_fruit = self.pose_xy [1] + dist * math.sin(alpha + beta)
                        self.fruits.append ((x_fruit, y_fruit))
                        print (self.time, x_fruit, y_fruit)
                        fruit.append(det)
        self.detections = fruit
        if len(self.detections) > 0:
            radius = 0.2
            self.center = cluster(self.fruits, radius)
            self.save_csv_if_enabled(self.center)

    def on_tick(self, data):
        if self.center:
            honk = False
            travelled_dist = self.pose_xy[0] - self.prev_pose[0]

            if abs(travelled_dist) > 0.01:
                if travelled_dist > 0:
                    pose_leftspray = (self.pose_xy[0] - 0.30, self.pose_xy[1] + 0.3)
                    pose_rightspray = (self.pose_xy[0] - 0.30, self.pose_xy[1] - 0.3)
                else:
                    pose_leftspray = (self.pose_xy[0] + 0.30, self.pose_xy[1] - 0.3)
                    pose_rightspray = (self.pose_xy[0] + 0.30, self.pose_xy[1] + 0.3)

                self.prev_pose = self.pose_xy

            else:
                return

            for c in self.center:
                distance_left = math.hypot(c[0] - pose_leftspray[0], c[1] - pose_leftspray[1])
                distance_right = math.hypot(c[0] - pose_rightspray[0], c[1] - pose_rightspray[1])
                if distance_left < 0.2:
                    honk = True
                if distance_right < 0.2:
                    honk = True
                if self.verbose:
                    print("Honk!", honk, distance_left, distance_right)

            self.send_sprayer(honk, False, False)

    def save_csv_if_enabled(self, centroid):
        if self.output_csv_enabled:
            filename = "CULS-Robotics-task2.csv"
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['x', 'y'])  # hlavička
                writer.writerows(centroid)
            print(f"Uloženo do souboru: {filename}")

    def send_sprayer(self, sirene, left, right):
        if sirene:
            self.publish('sprayer', b'*B1OS1H\r')
        else:
            self.publish('sprayer', b'*B1OS1L\r')
        if left:
            self.publish('sprayer', b'*B1OS4H\r')
        else:
            self.publish('sprayer', b'*B1OS4L\r')
        if right:
            self.publish('sprayer', b'*B1OS3H\r')
        else:
            self.publish('sprayer', b'*B1OS3L\r')

    def draw(self):
        from matplotlib.patches import Circle
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots()
        for x_center, y_center in self.fruits:
            ax.scatter(x_center, y_center)

        if not self.fruits:
            print("Žádné body k zobrazení nebo exportu.")
            return

       #  points = np.array(self.fruits)
       #  center = points.mean(axis=0)  # centroid (průměrný bod)
        radius = 0.2
        center = cluster(self.fruits, radius)
        for c in center:
            circle = Circle(c, radius, fill=False, edgecolor='r', linestyle='--')
            ax.add_patch(circle)
            ax.set_aspect('equal')
            ax.scatter(x_center, y_center)
        #assert 0, center

        # Uložení do CSV (pokud povoleno)
       # self.save_csv_if_enabled(center)

        # Vykreslení centroidu
        #plt.scatter(center[0], center[1], color='red', label='Reprezentant (průměr)')
        plt.legend()
        plt.title('Reprezentativní bod shluku')
        plt.grid(True, linestyle='--', color='gray', alpha=0.6)
        plt.show()

# vim: expandtab sw=4 ts=4
