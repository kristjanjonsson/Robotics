#!/usr/bin/env python

import rospy
from rospy import ServiceException
from A53070542_assignment_4.srv import KeyboardService


class Keyboard:

    def __init__(self, service_name):
        rospy.init_node(service_name)
        rospy.wait_for_service(service_name)
        self.change_mode_handle = rospy.ServiceProxy(service_name, KeyboardService)

    def change_mode(self, mode):
        response = self.change_mode_handle(mode.lower())

        if not response.error:
            print('Mode changed to: {0}'.format(mode))
        else:
            raise ServiceException(response.error)


input_msg = '''
Enter:
r for raw video feed.
f for Farneback.
m for MOG2.
'''


if __name__ == "__main__":
    keyboard = Keyboard('motion_mode_keyboard')
    while not rospy.is_shutdown():
        try:
            mode = raw_input(input_msg)
            keyboard.change_mode(mode)
        except ServiceException as e:
            print(e)
