from math import atan2, pi

from turtlesim.msg import Pose


class TurtlePose(Pose):

    def __str__(self):
        return 'x: {:.2f}, y: {:.2f}'.format(self.x, self.y)

    def __sub__(self, other):
        delta_pose = self.x - other.x, self.y - other.y
        return (delta_pose[0]**2 + delta_pose[1]**2)**0.5

    def angle(self, other):
        return (atan2(self.y - other.y, self.x - other.x) - other.theta + pi) % (2*pi) - pi
