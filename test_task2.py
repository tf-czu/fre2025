import unittest
from unittest.mock import MagicMock

from task2 import Task2, cluster


class Task2Test(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        app = Task2(bus=bus, config={
            # required parameters
            'max_speed': 0.5,
            'dist': 1.0,
            'timeout': 10
        })
        app.on_pose2d([0, 0, 0])
#        bus.publish.assert_called_with('emergency_stop', True)

    def test_cluster(self):
        self.assertEqual (cluster([(1,2)]),[(1,2)])
        self.assertEqual (cluster([(1,2), (1.1, 2)]),[(1,2)])
        self.assertEqual (cluster([(1,2), (1.1, 2), (3,4)]),[(1,2), (3,4)])

if __name__ == "__main__":
    unittest.main()

# vim: expandtab sw=4 ts=4
