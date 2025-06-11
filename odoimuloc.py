"""
  This is revised localization from Virtual to be used in System K2 as backup
"""

import collections
import logging
import math
from osgar.node import Node

from osgar.lib.quaternion import heading

g_logger = logging.getLogger(__name__)

Pose2d = collections.namedtuple("Pose2d", ("x", "y", "heading"))

class Localization(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        # outputs
        bus.register('pose2d')
        # inputs: origin, orientation, odom
        self.x = 0
        self.y = 0
        self.heading = None
        self.start_heading = None
        self.last_odom = None

    def on_orientation(self, orientation_list):
        orientation = orientation_list[-1][2:]  # take the last value
        if self.start_heading is None:
            self.start_heading = heading(orientation)
            self.heading = 0
        else:
            self.heading = heading(orientation) - self.start_heading
            # print(orientation)

    def on_odom(self, pose2d):
        x, y, heading = pose2d
        if self.heading is None:
            return

        odom = Pose2d(x / 1000.0, y / 1000.0, math.radians(heading / 100.0))
        if self.last_odom is not None:
            dist = math.hypot(odom.x - self.last_odom.x, odom.y - self.last_odom.y)
            direction = ((odom.x - self.last_odom.x) * math.cos(self.last_odom.heading) +
                         (odom.y - self.last_odom.y) * math.sin(self.last_odom.heading))
            if direction < 0:
                dist = -dist
        else:
            dist = 0.0

        self.last_odom = odom
        self.x = self.x + dist * math.cos(self.heading)
        self.y = self.y + dist * math.sin(self.heading)
        self.bus.publish('pose2d', [int(self.x*1000), int(self.y*1000), math.degrees(self.heading)*100])
        if self.verbose:
            print("pose2d: ", self.x, self.y, math.degrees(self.heading), "Odo heading", self.last_odom.heading)
