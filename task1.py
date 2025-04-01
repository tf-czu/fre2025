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
        self.debug_arr = []
        self.navigate_in_row = True
        self.end_of_row = None
        self.pose_xy = (0, 0)

    def on_depth(self, data):
        if not self.navigate_in_row:
            if math.hypot(self.end_of_row[0] - self.pose_xy[0],
                          self.end_of_row[1] - self.pose_xy[1]) < 1.0:
                self.send_speed_cmd(self.max_speed, 0)
            else:
                self.send_speed_cmd(0, 0)
            return
        line = 400//2
        line_end = 400//2 + 30
        mask = data[line:line_end, 160:480] != 0
        if mask.max():
            dist = data[line:line_end, 160:480][mask].min()
        else:
            dist = 0
        mask = data[line:line_end, 0:160] != 0
        if mask.max():
            dist_left = data[line:line_end, 0:160][mask].min()
        else:
            dist_left = 0
        mask = data[line:line_end, 480:640] != 0
        if mask.max():
            dist_right = data[line:line_end, 480:640][mask].min()
        else:
            dist_right = 0
        print(self.time, dist_left, dist, dist_right)
        if self.verbose:
            self.debug_arr.append((self.time.total_seconds(), dist_left, dist, dist_right))
        if abs(int(dist_left) - int(dist_right)) < 200:
            direction = 0
        else:
            if dist_left < dist_right:
                direction = -self.turn_angle
            else:
                direction = self.turn_angle
        if dist == 0:
            return
        if 0 < dist <= 330 or (dist_left > 1000 and dist_right > 1000):
            if (dist_left > 1000 and dist_right > 1000) and self.time.total_seconds() > 5:
                self.navigate_in_row = False
                self.end_of_row = self.pose_xy
            self.send_speed_cmd(0, 0)
        else:
            self.send_speed_cmd(self.max_speed, math.radians(direction))

    def on_pose2d(self, data):
        self.pose_xy = data[0]/1000, data[1]/1000

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed*1000), round(math.degrees(steering_angle)*100)]
        )

    def draw(self):
        import matplotlib.pyplot as plt

        for selection in range(3):
            t = [a[0] for a in self.debug_arr]
            x = [a[1 + selection] for a in self.debug_arr]
            line = plt.plot(t, x, '-o', linewidth=2, label=f'{selection + 1}')

        plt.xlabel('time (s)')
        plt.ylabel('dist')
        plt.legend()
        plt.show()



# vim: expandtab sw=4 ts=4
