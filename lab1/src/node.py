#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import LaserScan


class MAP:
    laser_scan = None
    last_stamp = None
    map = None
    scale = 10
    shape = (0, 0)

    @classmethod
    def update_scan(cls, msg):
        cls.laser_scan = msg


def create_map(msg):
    rospy.loginfo('create_map for %s' % str(MAP.last_stamp))
    scale = MAP.scale
    angle_min, angle_max, angle_increment = msg.angle_min, msg.angle_max, msg.angle_increment
    ranges, range_max = msg.ranges, msg.range_max

    range_max = np.max(ranges)
    range_min = np.min(ranges)
    rospy.logdebug(str(range_max))
    range_max = int(range_max + 1) * scale
    angles = np.arange(angle_min, angle_max, angle_increment)

    map_size = 2*int(range_max)
    center = [map_size/2, map_size/2]

    map = np.full((map_size, map_size), fill_value=-1, dtype=np.int8)

    for i, laser_range in enumerate(ranges):
        x = laser_range * np.sin(angles[i]) * scale + center[0]
        y = laser_range * np.cos(angles[i]) * scale + center[1]

        _x = abs(int(x))
        _y = abs(int(y))
        if _x < map.shape[0] and _y < map.shape[1]:
            #_x = (shape_x - _x - 1) % shape_x
            coord = np.array([_x, _y])
            _center = np.array(center)

            diff = coord - _center
            dist = np.sqrt(np.sum(diff*diff))
            direction = diff / np.linalg.norm(diff)
            if dist > range_min and dist < np.inf:
                dist = int(dist)
                for i in range(dist):
                    point = (center + direction * i).astype(int)
                    if point[0] < map.shape[0] and point[1] < map.shape[1]:
                        map[point[0]][point[1]] = 0
                    else:
                        break
            map[_x][_y] = 100

    MAP.shape = map.shape
    MAP.map = map.flatten()


if __name__ == "__main__":
    rospy.init_node('map_creator')

    scan_topic = rospy.get_param('~scan_topic', '/base_scan')
    MAP.scale = rospy.get_param('~map_scale', 10)
    rate = rospy.Rate(1)

    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
    sub_scan = rospy.Subscriber(scan_topic, LaserScan, MAP.update_scan)

    rospy.loginfo('Start node')
    while not rospy.is_shutdown():
        if MAP.laser_scan and MAP.last_stamp != MAP.laser_scan.header.stamp:
            create_map(MAP.laser_scan)
            MAP.last_stamp = MAP.laser_scan.header.stamp

            m = MapMetaData()
            m.resolution = 1.0/MAP.scale
            m.width = MAP.shape[1]
            m.height = MAP.shape[0]

            map_msg = OccupancyGrid()
            map_msg.data = MAP.map
            map_msg.header.frame_id = '/base_laser_link'

            m.origin = Pose()
            m.origin.position.x, m.origin.position.y = -m.width * m.resolution / 2, -m.height * m.resolution / 2
            map_msg.info = m

            pub.publish(map_msg)
        rate.sleep()
