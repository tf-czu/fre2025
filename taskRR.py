"""
  FRE2025 - TASK1 - Navigace podle barvy (zelená = tráva, šedá = cesta)
"""

import math
import numpy as np
import cv2

from osgar.node import Node
from osgar.bus import BusShutdownException


class TaskRR(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.max_speed = config.get('max_speed', 0.2)
        self.turn_angle = config.get('turn_angle', 20)
        self.verbose = False
        self.debug_arr = []
        self.pose_xy = (0, 0)
        self.pose_angle = 0
        self.color = None  # RGB obraz z OAK-D

    def on_color(self, data):
        # Předpokládáme, že `data` je BGR obraz jako NumPy array
        nparr = np.frombuffer(data, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        self.color = image

    def on_pose2d(self, data):
        self.pose_xy = data[0]/1000, data[1]/1000
        self.pose_angle = math.radians(data[2]/100)

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )

    def navigate_by_color_step(self, rgb):
        # Vyber dolní část obrazu (oblast před robotem)
        roi = rgb[300:480, :, :]  # spodní třetina

        # Převod do HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Maska pro zelenou (tráva)
        lower_green = np.array([35, 60, 40])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # Rozdělíme na 5 sektorů a spočítáme zelené pixely
        height, width = green_mask.shape
        N = 5
        column_width = width // N
        green_amounts = []

        for i in range(N):
            col = green_mask[:, i*column_width:(i+1)*column_width]
            green_pixels = cv2.countNonZero(col)
            green_amounts.append(green_pixels)

        # Najdi směr s nejmenším množstvím zelené
        best_index = np.argmin(green_amounts)
        direction = (best_index - N // 2) * self.turn_angle

        if self.verbose:
            print(self.time, "green amounts:", green_amounts, "→", direction)

        self.send_speed_cmd(self.max_speed, math.radians(direction))
        return True  # pokračuj v jízdě

    def navigate_by_color(self):
        print(self.time, 'navigate_by_color')
        while True:
            if self.update() == 'color':
                if self.color is not None:
                    if not self.navigate_by_color_step(self.color):
                        break

    def run(self):
        try:
            while True:
                self.navigate_by_color()
        except BusShutdownException:
            pass
