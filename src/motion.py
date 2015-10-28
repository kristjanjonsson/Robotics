#!/usr/bin/env python

import rospy
from A53070542_assignment_4.srv import KeyboardService
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from bbox import OpticalFlow, MOG2


class Motion:

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/camera/visible/image', Image, queue_size=10)
        rospy.Subscriber("camera/image_raw", Image, self.imageCallback)
        s = rospy.Service('motion_mode_keyboard', KeyboardService, self.change_mode)
        self.motion_detector = None
        s.spin()

    def change_mode(self, request):
        try:
            if request.mode == 'r':
                self.motion_detector = None
            elif request.mode == 'f':
                self.motion_detector = OpticalFlow()
            elif request.mode == 'm':
                self.motion_detector = MOG2()
            else:
                raise ValueError('Invalid mode: {0}'.format(request.mode))
        except Exception as e:
            return str(e)

        print 'Mode set to ' + request.mode
        return ''

    def imageCallback(self, image):
        if self.motion_detector:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            self.motion_detector.apply(cv_image)
            self.motion_detector.draw_bboxes(cv_image)
            image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

        self.pub.publish(image)

if __name__ == "__main__":
    motion = Motion('motion_node')
