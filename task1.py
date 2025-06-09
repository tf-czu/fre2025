"""
  FRE2025 - TASK1
"""
import math
import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException
from datetime import timedelta


class Task1(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.max_speed = config.get('max_speed', 0.2)
        self.turn_angle = config.get('turn_angle', 20)
        self.verbose = False
        self.debug_arr = []
        self.end_of_row = None
        self.pose_xy = (0, 0)
        self.pose_angle = 0
        self.depth = None

    def on_depth(self, data):
        self.depth = data
        
    def navigate_row_step(self, data):
        """
        Navigate single step in the row of maize
        :param data: depth data
        :return: True if still in row
        """
        line = 400//2
        line_end = 400//2 + 30
        box_width = 160
        arr = []
        for index in range(0 , 641 - box_width, 20):
            mask = data[line:line_end, index:box_width + index] != 0
            if mask.max():
                dist = int(np.percentile( data[line:line_end, index:box_width + index][mask], 5))
            else:
                dist = 0
##            if self.time.total_seconds() > 17.366:
##                print(index, dist)
            arr.append(dist)
        center = len(arr) // 2
        direction = None
        if arr [center] > 1000:
            direction = 0
        else:
            #nemůže jet rovně 
            for i in range(1, center):
                if arr [center - i] > 1000 and arr [center + i] <= 1000:
                    # volno vlevo                    
                    direction = self.turn_angle
                elif arr [center - i] <= 1000 and arr [center + i] > 1000:
                    # volno pravo                   
                    direction = -self.turn_angle    
##            print(self.time)
        assert direction is not None, (self.time, arr)
        self.send_speed_cmd(self.max_speed, math.radians(direction))
        return self.navigate_in_row
            

    def old_navigate_row_step(self, data):
        """
        Navigate single step in the row of maize
        :param data: depth data
        :return: True if still in row
        """
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
        if self.verbose:
            print(self.time, dist_left, dist, dist_right)
            self.debug_arr.append((self.time.total_seconds(), dist_left, dist, dist_right))
        if abs(int(dist_left) - int(dist_right)) < 200:
            direction = 0
        else:
            if dist_left < dist_right:
                direction = -self.turn_angle
            else:
                direction = self.turn_angle
        if dist == 0:
            return self.navigate_in_row
        if 0 < dist <= 330 or (dist_left > 1000 and dist_right > 1000):
            if (dist_left > 1000 and dist_right > 1000) and self.time.total_seconds() > 5:
                self.navigate_in_row = False
                self.end_of_row = self.pose_xy
            self.send_speed_cmd(0, 0)
        else:
            self.send_speed_cmd(self.max_speed, math.radians(direction))
        return self.navigate_in_row

    def on_pose2d(self, data):
        self.pose_xy = data[0]/1000, data[1]/1000
        self.pose_angle = math.radians(data[2]/100)

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed*1000), round(math.degrees(steering_angle)*100)]
        )

    def navigate_row(self):
        print(self.time, 'navigate_row')
        self.navigate_in_row = True
        while True:
            if self.update() == 'depth':
                if not self.navigate_row_step(self.depth):
                    break

    def go_straight(self, dist):
        print(self.time, 'go_straight')
        self.end_of_row = self.pose_xy
        while True:
            if self.update() == 'depth':
                if math.hypot(self.end_of_row[0] - self.pose_xy[0],
                              self.end_of_row[1] - self.pose_xy[1]) < dist:
                    self.send_speed_cmd(self.max_speed, 0)
                else:
                    self.send_speed_cmd(0, 0)
                    break
    def turn_deg_left(self, angle):
        print(self.time, 'turn_deg_left')
        dist = 0
        prev = self.pose_xy
        while dist < 3.14*0.75/2:
            if self.update() == 'pose2d':
                 self.send_speed_cmd(self.max_speed, math.radians (45))
                 dist += math.hypot(prev[0] - self.pose_xy[0],
                              prev[1] - self.pose_xy[1])
                 prev = self.pose_xy
        self.send_speed_cmd(0, 0)

    def turn_deg_right(self, angle):
        print(self.time, 'turn_deg_right')
        dist = 0
        prev = self.pose_xy
        while dist < 3.14*0.75/2:
            if self.update() == 'pose2d':
                 self.send_speed_cmd(self.max_speed, math.radians (-45))
                 dist += math.hypot(prev[0] - self.pose_xy[0],
                              prev[1] - self.pose_xy[1])
                 prev = self.pose_xy
        self.send_speed_cmd(0, 0)

    def wait(self,duration):
        self.update()
        start_time=self.time
        while self.time - start_time < timedelta(seconds=duration):
            self.update()
        
    def run(self):
        try:
            self.wait(15)
            for num in range(10):
                self.navigate_row()
                self.go_straight(1.0)
                self.turn_deg_left(180)
                self.navigate_row()
                self.go_straight(1.0)
                self.turn_deg_right(180)
        except BusShutdownException:
            pass

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
