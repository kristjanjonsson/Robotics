#!/usr/bin/env python

import rospy
from A53070542_assignment_4.srv import KeyboardService
from sensor_msgs.msg import Image


class Motion:

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.pub = rospy.Publisher('/camera/visible/image', Image, queue_size=10)
        rospy.Subscriber("camera/image_raw", Image, self.imageCallback)
        s = rospy.Service('motion_mode_keyboard', KeyboardService, self.printer)
        self.mode = None
        s.spin()

    def printer(self, request):
        if request.mode not in 'rfm':
            return 'Invalid mode {0}'.format(request.mode)
        print "Mode set to ", request.mode
        self.mode = request.mode

    def imageCallback(self, image):
        print "Get the image."

        if self.mode == 'r':
            print "Publish image to /camera/visible/image"
            self.pub.publish(image)
        
        elif self.mode == 'f':
            # TODO
            pass
        
        elif self.mode == 'm':
            # TODO
            pass
        
if __name__ == "__main__":
    motion = Motion('motion_node')
