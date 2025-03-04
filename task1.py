"""
  FRE2025 - TASK1
"""
from osgar.node import Node


class Task1(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.max_speed = config.get('max_speed', 0.2)
        self.verbose = False

    def on_depth(self, data):
        self.send_speed_cmd(self.max_speed, 0)

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed*1000), round(math.degrees(steering_angle)*100)]
        )

# vim: expandtab sw=4 ts=4
