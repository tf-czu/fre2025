import math
from osgar.node import Node
from osgar.bus import BusShutdownException


class Task3c(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.max_speed = config.get('max_speed', 0.2)  # rychlost v m/s
        self.verbose = False
        self.pose_xy = (0, 0)
        self.pose_angle = 0

    def on_pose2d(self, data):
        self.pose_xy = data[0] / 1000, data[1] / 1000
        self.pose_angle = math.radians(data[2] / 100)

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )

    def drive_full_circle(self, radius):
        print(self.time, f'drive_full_circle r={radius}')
        steering_angle = self.max_speed / radius  # rad/s
        total_distance = 2 * math.pi * radius     # celková délka kruhu
        dist = 0
        prev = self.pose_xy

        while dist < total_distance:
            if self.update() == 'pose2d':
                self.send_speed_cmd(self.max_speed, steering_angle)
                dist += math.hypot(prev[0] - self.pose_xy[0],
                                   prev[1] - self.pose_xy[1])
                prev = self.pose_xy

        self.send_speed_cmd(0, 0)  # zastaví robota

    def run(self):
        try:
            self.drive_full_circle(1.2)  # opisuje celý kruh o r = 1.2 m
        except BusShutdownException:
            self.send_speed_cmd(0, 0)

