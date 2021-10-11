from geometry_msgs.msg import Twist


def create_msg(linear_x, angle_z):
    tw = Twist()
    tw.linear.x = linear_x
    tw.angular.z = angle_z
    return tw
