#!/usr/bin/env python

import rospy
from A53070542_assignment_4.srv import KeyboardService


class Keyboard:

    def __init__(self, service_name):
        rospy.init_node(service_name)
        self.service_name = service_name
        rospy.wait_for_service(service_name)
        try:
            self.change_mode_handle = rospy.ServiceProxy(service_name, KeyboardService)
        except rospy.ServiceException as e:
            print 'Service call failed: {}'.format(str(e))

    def change_mode(self, mode):
        try:
            response = self.change_mode_handle(mode.lower())
        except rospy.ServiceException as e:
            print 'Service call failed: {}'.format(str(e))

        if not response.error:
            print('Mode changed to: {0}'.format(mode))
        else:
            raise ValueError('Failed to change to mode: {0}\n{1}'.format(mode, response.error))


input_msg = '''
Enter:
r for raw video feed.
f for Farneback.
m for MOG2.
'''


if __name__ == "__main__":
    keyboard = Keyboard('motion_mode_keyboard')
    while True:
        try:
            mode = raw_input(input_msg)
            keyboard.change_mode(mode)
        except Exception as e:
            print(e)
