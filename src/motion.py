#!/usr/bin/env python

import rospy
from A53070542_assignment_4.srv import KeyboardService


def printer(request):
    if request.mode not in 'rfm':
        return 'Invalid mode {0}'.format(request.mode)
    print request.mode
    return ''


class Motion:

    def __init__(self, node_name):
        rospy.init_node(node_name)
        s = rospy.Service('motion_mode_keyboard', KeyboardService, printer)
        s.spin()


if __name__ == "__main__":
    motion = Motion('motion_node')
