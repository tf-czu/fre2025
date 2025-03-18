"""
  FRE2025 - TASK1
"""
import math

from osgar.node import Node


class Task1(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.max_speed = config.get('max_speed', 0.2)
        self.turn_angle = config.get('turn_angle', 20)
        self.verbose = False

    def on_depth(self, data):
        line = 400//2
        line_end = 400//2 + 1
        mask = data[line:line_end][160:480] != 0
        if mask.max():
            dist = data[line:line_end][160:480][mask].min()
        else:
            dist = 0
        mask = data[line:line_end][0:160] != 0
        if mask.max():
            dist_left = data[line:line_end][0:160][mask].min()
        else:
            dist_left = 0
        mask = data[line:line_end][480:640] != 0
        if mask.max():
            dist_right = data[line:line_end][480:640][mask].min()
        else:
            dist_right = 0
        print(self.time, dist_left, dist, dist_right)
        if abs(dist_left - dist_right) < 200:
            direction = 0
        else:
            if dist_left < dist_right:
                direction = -self.turn_angle
            else:
                direction = self.turn_angle
        if dist == 0:
            return
        if 0 < dist <= 330:
            self.send_speed_cmd(0, 0)
        else:
            self.send_speed_cmd(self.max_speed, math.radians(direction))

    def on_pose2d(self, data):
        pass

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed*1000), round(math.degrees(steering_angle)*100)]
        )

# vim: expandtab sw=4 ts=4
