#!/usr/bin/env python

import rospy

# TODO: Generate service.
from somewhere import KeyboardService


def printer(request):
    print request.msg
    return True,


class Motion:

    def __init__(self, node_name):
        rospy.init_node(node_name)
        s = rospy.Service('motion_mode_keyboard', KeyboardService, printer)
        s.spin()


if __name__ == "__main__":
    motion = Motion('motion_node')
