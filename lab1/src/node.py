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
    rospy.loginfo('create_map')
    scale = MAP.scale
    angle_min, angle_max, angle_increment = msg.angle_min, msg.angle_max, msg.angle_increment
    ranges, range_max = msg.ranges, msg.range_max

    range_max = max((el for el in ranges if el != np.inf))
    # rospy.logerr(str(range_max))
    range_max = int(range_max + 1) * scale
    angles = np.arange(angle_min, angle_max, angle_increment)

    map_size = 2*int(range_max)
    center = [map_size/2, map_size/2]

    map = np.full((map_size, map_size), fill_value=-1)

    for i, laser_range in enumerate(ranges):
        #if i % 2 == 0:
        #    continue
        x = laser_range * np.sin(angles[i]) * scale + center[0]
        y = laser_range * np.cos(angles[i]) * scale + center[1]

        _x = abs(int(x))
        _y = abs(int(y))
        if _x < map.shape[0] and _y < map.shape[1]:
            #_x = (shape_x - _x - 1) % shape_x
            coord = np.array([_x, _y])
            _center = np.array(center)

            diff = coord - _center
            dist = np.sqrt(np.sum(np.power(diff, 2)))
            direction = diff / np.linalg.norm(diff)
            if dist > 1 and np.abs(dist) < np.inf:
                dist = int(dist)
                for i in range(dist * 2):
                    point = (center + direction * i / 2).astype(int)
                    if point[0] < map.shape[0] and point[1] < map.shape[1]:
                        map[point[0]][point[1]] = 0
                    else:
                        break
            map[_x][_y] = 100

            continue

            if end[0] == start[0] or end[1] == start[1]:
                continue

            if start[0] > end[0]:
                start, end = end, start

            k = (end[0] - start[1]) / (end[0] - start[0])
            b = start[1] - k*start[0]
            def f_y(x): return int(k*x + b) % map.shape[1]
            def f_x(y): return int(y/k - start[1]/k + start[0]) % map.shape[0]

            for new_x in range(start[0], end[0]):
                map[new_x][f_y(new_x)] = 0
            continue

            if abs(end[0] - start[0]) > abs(end[1] - start[1]):
                for new_x in range(start[0], end[0]):
                    map[new_x][f_y(new_x)] = 0
            else:
                for new_y in range(start[1], end[1]):
                    map[f_x(new_y)][new_y] = 0

            continue
            start, end = end, start
            if start[0]-end[0]+start[1]-end[1] == 0:
                continue

            steep = abs(end[1]-start[1]) > abs(end[0]-start[0])
            toggle = 0
            if steep:
                start = start[::-1]
                end = end[::-1]

            if start[0] > end[0]:
                toggle = 1
                _x0 = int(start[0])
                _x1 = int(end[0])
                start[0] = _x1
                end[0] = _x0

                _y0 = int(start[1])
                _y1 = int(end[1])
                start[1] = _y1
                end[1] = _y0

            dx = end[0] - start[0]
            dy = abs(end[1] - start[1])
            error = 0
            derr = dy/float(dx)

            ystep = 0
            y = start[1]

            if start[1] < end[1]:
                ystep = 1
            else:
                ystep = -1

            for x in range(start[0], end[0]+1):
                if steep:
                    map[y][x] = 0
                    # path.append((y,x))
                else:
                    map[x][y] = 0
                    # path.append((x,y))

                error += derr

                if error >= 0.5:
                    y += ystep
                    error -= 1.0

    MAP.shape = map.shape
    map = map[::-1]
    MAP.map = np.flipud(map).flatten().astype(np.int8).tolist()


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

            pos = np.array([-m.width * m.resolution / 2, -
                           m.height * m.resolution / 2, 0])
            m.origin = Pose()
            m.origin.position.x, m.origin.position.y = pos[:2]
            map_msg.info = m

            pub.publish(map_msg)
        rate.sleep()
