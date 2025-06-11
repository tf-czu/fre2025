import math
import numpy as np
import csv

from collections import Counter

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.platforms.matty import FRONT_REAR_AXIS_DISTANCE
from task2 import Task2

def cluster(points, radius=0.3):
    s_points = []
    for p in points:
        for sp in s_points:
            if math.hypot(p[1] - sp[1], p[2] - sp[2]) < radius:
                break
        else:
            s_points.append(p)
    return s_points


class Task3(Task2):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.detections = None
        self.fruits = []
        self.output_csv_enabled = config.get('outputcsv', True)
        self.start_angle = None
        self.save_csv_if_enabled([])  # vytvoř prázdný soubor
        self.trees = config.get('trees', [])
        self.fruits_on_trees = []
        self.tree_id = None
        self.radius = 1.20
        self.steering_angle_rad = 2 * math.atan((FRONT_REAR_AXIS_DISTANCE / 2) / self.radius)
        self.honk_dic = {"apple": 1, "banana": 2, "lemon": 3, "orange": 4, "grape": 5}

    def on_detections(self, data):
        if self.time.total_seconds() < 5:
            return

        if self.tree_id is None:
            return

        camera_height = 0.25
        vertical_fov = math.radians(40)
        camera_tilt = math.radians(68)

        # to be removed
        assert self.depth.shape == (400, 640), self.depth.shape
        my_depth = self.depth
        my_depth[my_depth > 3000] = 0  # remove too far values

        fruit_types = {"apple", "banana", "lemon", "orange", "grape"}
        fruit = []
        for det in data:
            if self.verbose:
                pass
                # print("Detections: ", det)
            fruit_type = det[0]
            if fruit_type in fruit_types:
                x1, y1, x2, y2 = det[2]  # values between 0 and 1
                if x1 < 0.15 or x2 > 0.85 or y1 < 0.1 or y2 > 0.9:
                    continue
                x_center = (x1 + x2) / 2
                y_center = (y1 + y2) / 2
                beta = (0.5 - x_center) * math.radians(69)  # 69 horizontal FOV
                alpha = self.pose_angle # heading
                mask = my_depth[int(y1 * 400): int(y2 * 400), int(x1 * 640): int(x2 * 640)] != 0
                if not np.any(mask):
                    continue
                dist = np.median(my_depth[int(y1 * 400): int(y2 * 400), int(x1 * 640): int(x2 * 640)][mask]) / 1000
                x_fruit = self.pose_xy[0] + dist * math.sin(alpha + beta) + 1  # corection for the robot initial pose
                theta = camera_tilt + (0.5 - y_center) * vertical_fov
                y_fruit = self.pose_xy[1] + ((dist / math.cos(theta))*math.cos(camera_tilt+theta+alpha)) + 1  # corection for the robot initial pose
                z_fruit = camera_height + (dist / math.cos(theta)) * math.sin(camera_tilt + theta)
                self.fruits.append((fruit_type, x_fruit, y_fruit, z_fruit))  # old part

                self.fruits_on_trees[self.tree_id]["fruits"].append([fruit_type, x_fruit, y_fruit])  # to be filtered later

                # print(self.time, fruit_type, x_fruit, y_fruit, z_fruit)
                fruit.append(det)

        self.detections = fruit
        if len(self.detections) > 0:
            radius = 0.2
            center = cluster(self.fruits, radius)
            self.save_csv_if_enabled(center)

    def drive_to_point(self, target, tolerance=0.2):
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
            angle_diff = heading - self.pose_angle
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            self.send_speed_cmd(self.max_speed, angle_diff)
        self.send_speed_cmd(0, 0)

    def turn_to_angle(self, target_angle, tolerance=math.radians(2)):
        while True:
            self.update()
            diff = target_angle - self.pose_angle
            diff = (diff + math.pi) % (2 * math.pi) - math.pi
            if abs(diff) < tolerance:
                break
            angular_speed = math.copysign(math.radians(45), diff)
            self.send_speed_cmd(0.1, angular_speed)
            
        self.send_speed_cmd(0, 0)

    def drive_circle_by_angle(self, steering_angle, target_angle_deg):
        start_angle = self.pose_angle
        swept_angle = 0.0
        prev_angle = start_angle

        while swept_angle < math.radians(target_angle_deg):
            channel = self.update()
            if channel == 'pose2d':
                self.send_speed_cmd(self.max_speed, steering_angle)
                current_angle = self.pose_angle
                delta = (current_angle - prev_angle + math.pi) % (2 * math.pi) - math.pi
                swept_angle += abs(delta)
                prev_angle = current_angle
        self.send_speed_cmd(0, 0)

    def go_to_circle_and_drive(self, center):
        print(self.time, f'Jízda ke stromu v {center}')
        cx, cy = center
        sx, sy = self.pose_xy

        # Najet na obvod kruhu
        dx = cx - sx
        dy = cy - sy
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            print("Robot je ve středu stromu, nelze určit směr.")
            return
        ux = dx / dist
        uy = dy / dist
        entry_x = cx - ux * self.radius
        entry_y = cy - uy * self.radius
        self.drive_to_point((entry_x, entry_y))

        # Otočit se tangenciálně proti směru hodinových ručiček
        tangent_angle = math.atan2(-ux, uy)
        print(self.time, f'Otáčení do tangenciálního směru: {math.degrees(tangent_angle):.1f}°')
        self.turn_to_angle(tangent_angle)

        # Obkroužit celý strom
        print(self.time, 'Začínám opisovat kružnici...')
        self.drive_circle_by_angle(self.steering_angle_rad, 360)

        print(self.time, 'Popojíždím rovně o 1 metr od stromu...')
        direction = self.pose_angle
        x, y = self.pose_xy
        target_x = x + math.cos(direction) * 1.0
        target_y = y + math.sin(direction) * 1.0
        self.drive_to_point((target_x, target_y))

    def save_csv_if_enabled(self, centroid):
        if self.output_csv_enabled:
            filename = "CULS-Robotics-task3.csv"
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['fruit_type', 'x', 'y', 'z'])
                writer.writerows(centroid)
            # print(f"Uloženo do souboru: {filename}")

    def filter_output(self):
        curennt_tree = self.fruits_on_trees[self.tree_id]
        detections = [fruit_type[0] for fruit_type in curennt_tree["fruits"]]
        print(detections)

        counts = Counter(detections)
        if self.verbose:
            print(f"NUmber of fruit type: {counts}")
        most_common_type = counts.most_common(1)[0][0]

        self.honk_fruits(most_common_type)


    def honk_fruits(self, fruit_type):
        print(fruit_type)
        nummber_of_honks = self.honk_dic[fruit_type]
        for ii in range(nummber_of_honks):
            self.send_sprayer(True, False, False)
            self.wait(1)
            self.send_sprayer(False, False, False)
            self.wait(1)

    def run(self):
        try:
            for ii, tree in enumerate(self.trees):
                self.tree_id = ii
                self.fruits_on_trees.append({"pose": tree, "fruits": []})
                self.go_to_circle_and_drive(tree)
                self.send_speed_cmd(0, 0)
                self.filter_output()
                # if self.verbose:
                #    print("FRUITS: ", self.fruits_on_trees)
            self.send_speed_cmd(0, 0)
        except BusShutdownException:
            self.send_speed_cmd(0, 0)
