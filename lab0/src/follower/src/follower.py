#!/usr/bin/python3
# roslaunch pursue pursue.launch distance_tolerance:=.3 pursue_step_rate:=1
from time import sleep

import rospy
from geometry_msgs.msg import Twist

from turtle_pose import TurtlePose
from utils import create_msg


class Configuration:
    current_pose = TurtlePose()
    target_pose = TurtlePose()

    def update_current_pose(cls, new_pose):
        cls.current_pose = new_pose

    def update_target_pose(cls, new_pose):
        cls.target_pose = new_pose

    def distance(cls):
        return cls.target_pose - cls.current_pose

    def angle(self):
        return self.target_pose.angle(self.current_pose)

    def __str__(cls):
        return 'current_pose: {}, target_pose: {}, distance: {:.3f}, angle: {:.3f}'.format(
            cls.current_pose, cls.target_pose, cls.distance(), cls.angle())


if __name__ == "__main__":
    rospy.init_node('turtle_follower')
    rate = rospy.Rate(4/5)

    conf = Configuration()
    
    pub = rospy.Publisher('/turtle_follower/cmd_vel', Twist, queue_size=1)
    sub_follower = rospy.Subscriber(
        '/turtle_follower/pose', TurtlePose, conf.update_current_pose)
    sub_following = rospy.Subscriber(
        '/turtle1/pose', TurtlePose, conf.update_target_pose)

    min_distance = 1
    while not rospy.is_shutdown():
        dist = conf.distance()
        rospy.loginfo(str(conf))
        if dist >= min_distance:
            angle = conf.angle()
            tw = create_msg(dist, angle)
            pub.publish(tw)
        rate.sleep()
