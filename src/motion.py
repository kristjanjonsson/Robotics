#!/usr/bin/env python

import rospy
from A53070542_assignment_4.srv import KeyboardService
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

# Kernels for dilate erode filters.
kernel15 = np.ones((15, 15), np.uint8)
kernel_elliptic_7 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
kernel_elliptic_15 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))


class BBox:
    '''Parent class for motion bounding box.'''

    def apply(self, frame):
        '''Applies a motion detection algorithms on frame.'''
        raise NotImplementedError()

    def binary_image(self):
        '''Returns a binary image of pixels that are moving.'''
        raise NotImplementedError()

    def bounding_boxes(self, area_threshold=2000):
        '''Returns: List of the corner points of motion bounding boxes.
        Returns the 3 largest if that many.'''
        contours = cv2.findContours(self.binary_image().astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        area_box = ((cv2.contourArea(contour), cv2.boundingRect(contour)) for contour in contours[0])
        area_box = [(area, box) for (area, box) in area_box if area > area_threshold]
        area_box.sort(reverse=True)
        return [((x, y), (x+w, y+h)) for _, (x, y, w, h) in area_box[:3]]

    def draw_bboxes(self, frame, **kwargs):
        '''Draws bounding boxes on frame.'''
        for p1, p2 in self.bounding_boxes(**kwargs):
            cv2.rectangle(frame, p1, p2, (0, 255, 0), **kwargs)


class OpticalFlow(BBox):

    def __init__(self):
        self.prev_frame = None

    def apply(self, frame):
        '''Takes in a greyscale img and calculates the flow, a binary image from the flow
        and bounding boxes of the binary image.'''
        # To gray scale.
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.prev_frame is None:
            self.prev_frame = frame

        self.flow_magnitude_img = self.flow_magnitude(self.prev_frame, frame)
        self.binary_img = self.to_binary(self.flow_magnitude_img)
        self.prev_frame = frame

    def binary_image(self):
        return self.binary_img

    def flow_magnitude(self, prev_frame, frame):
        '''Calculates the magnitude of the Farneback Optical flow vectors.
        Both frames are greyscale.'''
        flow = cv2.calcOpticalFlowFarneback(prev_frame, frame,
                                            pyr_scale=0.5,
                                            levels=3,
                                            winsize=15,
                                            iterations=2,
                                            poly_n=5,
                                            poly_sigma=1.1,
                                            flags=0)
        return cv2.magnitude(flow[..., 0], flow[..., 1])

    def to_binary(self, dest, threshold=2.7, kernel=kernel15):
        '''Applies a threshold and does erosion and dilation on a grey scale image.'''
        cv2.threshold(dest, threshold, 1, 0, dst=dest)
        cv2.morphologyEx(dest, cv2.MORPH_OPEN, kernel, dst=dest)
        cv2.dilate(dest, kernel, dst=dest, iterations=1)
        return dest


class MOG2(BBox):

    def __init__(self):
        self.mog2 = cv2.BackgroundSubtractorMOG2(history=150, varThreshold=16, bShadowDetection=False)

    def apply(self, frame):
        '''Applies mog2 backround subtraction algorithm on frame.'''
        fgmask = self.mog2.apply(frame, learningRate=0.01)
        cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel_elliptic_7, dst=fgmask)
        cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel_elliptic_15, dst=fgmask)
        self.binary_img = fgmask

    def binary_image(self):
        return self.binary_img

class Motion:

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.pub = rospy.Publisher('/camera/visible/image', Image, queue_size=10)
        rospy.Subscriber("camera/image_raw", Image, self.imageCallback)
        s = rospy.Service('motion_mode_keyboard', KeyboardService, self.mode_selector)
        
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.OpticalFlow_bbox = OpticalFlow()
        self.MOG2_bbox = MOG2()
        self.mode = None
        s.spin()

    def mode_selector(self, request):
        '''For choosing processing modes:
        r: Raw Video
        f: Farneback Optical Flow Algorithm
        m: Improved Mixture of Gaussians (MOG2) Background Subtraction Algorithm
        '''
        if request.mode not in 'rfm':
            return 'Invalid mode {0}'.format(request.mode)
        print "Mode set to ", request.mode
        self.mode = request.mode

    def imageCallback(self, image):
        '''For image processing under different mode and publication to viewer'''
        print "Get the image."
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError, e:
            print e
        
        if self.mode == 'r':
            print "Publish image to /camera/visible/image"    
            self.pub.publish(image)

        elif self.mode == 'f':
            print "Publish image processed by Optical Flow Algorithm to /camera/visible/image"

            cv_image = cv2.resize(cv_image, (640, 480))
            self.OpticalFlow_bbox.apply(cv_image)
            self.OpticalFlow_bbox.draw_bboxes(cv_image)

            image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.pub.publish(image)        
        
        elif self.mode == 'm':
            print "Publish image processed by MOG2 Algorithm to /camera/visible/image"

            cv_image = cv2.resize(cv_image, (640, 480))
            self.MOG2_bbox.apply(cv_image)
            self.MOG2_bbox.draw_bboxes(cv_image)
            
            image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.pub.publish(image)        

if __name__ == "__main__":
    motion = Motion('motion_node')
