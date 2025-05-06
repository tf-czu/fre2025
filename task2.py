"""
  FRE2025 - TASK2
"""
import math
import numpy as np
import csv

from osgar.node import Node
from osgar.bus import BusShutdownException
from task1 import Task1


class Task2(Task1):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.detections = None
        self.fruits = []
        self.output_csv_enabled = config.get('outputcsv', False) 

    def on_detections(self, data):
        if self.time.total_seconds() < 5:
            return
        fruit = []
        for det in data:
            if det [0] == "banana":
                x1, y1, x2, y2 = det[2]
                x_center = (x1 + x2) / 2
                y_center = (y1 + y2) / 2
                beta = (0.5 - x_center) * math.radians(69)
                alpha = self.pose_angle
                mask = self.depth[int(y1 * 400) : int(y2 * 400), int(x1 * 640) : int(x2 * 640)] != 0
                dist = np.median(self.depth[int(y1 * 400) : int(y2 * 400), int(x1 * 640) : int(x2 * 640)][mask])/1000
                x_fruit = self.pose_xy [0] + dist * math.cos(alpha + beta)
                y_fruit = self.pose_xy [1] + dist * math.sin(alpha + beta)
                self.fruits.append ((x_fruit, y_fruit))
                print (self.time, x_fruit, y_fruit)
                fruit.append(det)
        self.detections = fruit

    def save_csv_if_enabled(self, centroid):
        if self.output_csv_enabled:
            filename = "CULS-Robotics-task2.csv"
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['x', 'y'])  # hlavička
                writer.writerow(centroid.tolist())
            print(f"Uloženo do souboru: {filename}")

    def draw(self):
        import matplotlib.pyplot as plt
        for x_center, y_center in self.fruits:
            plt.scatter(x_center, y_center)

        if not self.fruits:
            print("Žádné body k zobrazení nebo exportu.")
            return

        points = np.array(self.fruits)
        center = points.mean(axis=0)  # centroid (průměrný bod)

        # Uložení do CSV (pokud povoleno)
        self.save_csv_if_enabled(center)

        # Vykreslení centroidu
        plt.scatter(center[0], center[1], color='red', label='Reprezentant (průměr)')
        plt.legend()
        plt.title('Reprezentativní bod shluku')
        plt.grid(True, linestyle='--', color='gray', alpha=0.6)
        plt.show()



# vim: expandtab sw=4 ts=4
