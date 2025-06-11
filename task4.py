import math
import numpy as np
import csv

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.platforms.matty import FRONT_REAR_AXIS_DISTANCE
from task1 import Task1
from task2 import cluster
from datetime import timedelta# zdědí základní pohybové schopnosti


class Task4(Task1):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.side_length = config.get('side_length', 7.0)
        self.detections = None
        self.fruits = []
        self.detect_radius = 0.5
        self.output_csv_enabled = config.get('outputcsv', True)
        self.save_csv_if_enabled([])  # vytvoř prázdný soubor
        self.dist_right = None
        

    def on_depth(self, data):
        self.depth = data
        
        line = 400//2
        line_end = 400//2 + 30             
        mask = data[line:line_end, 480:640] != 0
        if mask.max():
            self.dist_right = data[line:line_end, 480:640][mask].min()
        else:
            self.dist_right = 0
        if self.verbose:
            print(self.time, self.dist_right)

    def on_detections(self, data):
        if self.time.total_seconds() < 5:
            return
        fruit = []
        for det in data:
            if det [0] == "orange":
                x1, y1, x2, y2 = det[2]
                x_center = (x1 + x2) / 2
                y_center = (y1 + y2) / 2
                if y_center < 0.5:
                    continue
                beta = (0.5 - x_center) * math.radians(69)
                alpha = self.pose_angle
                mask = self.depth[int(y1 * 400) : int(y2 * 400), int(x1 * 640) : int(x2 * 640)] != 0
                if mask.shape[0] > 0 and mask.shape[1] > 0 and mask.max():
                    dist = np.median(self.depth[int(y1 * 400) : int(y2 * 400), int(x1 * 640) : int(x2 * 640)][mask])/1000
                    if dist < 2.0:
                        x_fruit = self.pose_xy [0] + dist * math.cos(alpha + beta) + 1
                        y_fruit = self.pose_xy [1] + dist * math.sin(alpha + beta) + 1
                        self.fruits.append ((x_fruit, y_fruit))
##                        print (self.time, x_fruit, y_fruit)
                        fruit.append(det)
        self.detections = fruit
        if len(self.detections) > 0:
            self.center = cluster(self.fruits, self.detect_radius)
            self.save_csv_if_enabled(self.center)

    def go_straight(self, dist):
        print(self.time, f'Jedu rovně {dist} m')
        start = self.pose_xy
        start_heading = self.pose_angle
        while True:
            if self.update() == 'pose2d':
                dx = self.pose_xy[0] - start[0]
                dy = self.pose_xy[1] - start[1]
                if math.hypot(dx, dy) < dist:
                    diff = self.pose_angle - start_heading
##                    assert abs(diff) < math.radians(1), self.time
                    direction = 0
                    if abs(diff) < math.radians(1):
                        direction = 0
                    else:
                        if diff > 0:
                            direction = -math.radians(self.turn_angle)
                        else:
                            direction = math.radians(self.turn_angle)
                    if (self.dist_right is not None and self.dist_right < 1000
                        and self.time.total_seconds() > 10):
                        direction += math.radians(self.turn_angle) * 2 
                    self.send_speed_cmd(self.max_speed, direction)
                else:
                    self.send_speed_cmd(0, 0)
                    break

    def turn_left_90deg(self):
        print(self.time, 'Zatáčím vlevo o 90° během jízdy')
        dist = 0
        prev = self.pose_xy
        start_heading = self.pose_angle
        joint_angle = math.radians(45)
        radius = (FRONT_REAR_AXIS_DISTANCE/2) / math.tan(joint_angle/2)
        arc_length = (math.pi * radius / 2)  # původně pro 180°, teď polovina pro 90°
        while self.pose_angle - start_heading < math.pi / 2:
            if self.update() == 'pose2d':
                self.send_speed_cmd(0.2, joint_angle)
                dist += math.hypot(prev[0] - self.pose_xy[0],
                                   prev[1] - self.pose_xy[1])
                prev = self.pose_xy
        self.send_speed_cmd(0, 0)
##        assert 0, (start_heading, self.pose_angle, self.time)

    def wait(self,duration):
        self.update()
        start_time=self.time
        while self.time - start_time < timedelta(seconds=duration):
            self.update()
            
    def save_csv_if_enabled(self, centroid):
        if self.output_csv_enabled:
            filename = "CULS-Robotics-task4.csv"
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['x', 'y'])  # hlavička
                writer.writerows(centroid)
            if len (centroid) == 0:
                print(f"Uloženo do souboru: {filename}")
                
    def wait_for_camera(self):
        self.update()
        print(self.time, "Čekám na kameru")
        while self.depth is None:
            self.update()
        print(self.time, "Kamera k dispozici")
     
    def run(self):
        try:
            self.wait_for_camera() 
            length = self.side_length
            while length > 0:
                for _ in range(3):
                    self.go_straight(length)
                    self.turn_left_90deg()
                length -= 1.0
            self.send_speed_cmd(0, 0)
        except BusShutdownException:
            self.send_speed_cmd(0, 0)

    def draw(self):
        from matplotlib.patches import Circle
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots()
        for x_center, y_center in self.fruits:
            ax.scatter(x_center, y_center)
        # playground 10x10             
        for x_center, y_center in [[0, 0], [10, 10]]:
            ax.scatter(x_center, y_center)
            
        if not self.fruits:
            print("Žádné body k zobrazení nebo exportu.")
            return

       #  points = np.array(self.fruits)
       #  center = points.mean(axis=0)  # centroid (průměrný bod)
        center = cluster(self.fruits, self.detect_radius)
        for c in center:
            circle = Circle(c, self.detect_radius, fill=False, edgecolor='r', linestyle='--')
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
