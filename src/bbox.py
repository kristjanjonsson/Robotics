#!/usr/bin/env python

import sys
import numpy as np
import cv2


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
        area_box = ((cv2.contourArea(contour), cv2.boundingRect(contour)) for contour in contours[1])
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
        flow = cv2.calcOpticalFlowFarneback(prev_frame, frame, None,
                                            pyr_scale=0.5,
                                            levels=3,
                                            winsize=17,
                                            iterations=2,
                                            poly_n=7,
                                            poly_sigma=1.5,
                                            flags=0)
        return cv2.magnitude(flow[..., 0], flow[..., 1])

    def to_binary(self, flow_magnitude, threshold=2.7, kernel=kernel15):
        '''Applies a threshold and does erosion and dilation on a grey scale image.'''
        dest = np.zeros_like(flow_magnitude)
        cv2.threshold(flow_magnitude, threshold, 1, 0, dst=dest)
        cv2.morphologyEx(dest, cv2.MORPH_OPEN, kernel, dst=dest)
        cv2.dilate(dest, kernel, dst=dest, iterations=1)
        return dest


class MOG2(BBox):

    def __init__(self):
        self.mog2 = cv2.createBackgroundSubtractorMOG2(history=150)

    def apply(self, frame):
        '''Applies mog2 backround subtraction algorithm on frame.'''
        fgmask = self.mog2.apply(frame)
        cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel_elliptic_7, dst=fgmask)
        cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel_elliptic_15, dst=fgmask)
        self.binary_img = fgmask

    def binary_image(self):
        return self.binary_img


def main(mode, video_fname=None):
    if video_fname:
        cap = cv2.VideoCapture(video_fname)
    else:
        cap = cv2.VideoCapture(0)

    cv2.namedWindow('original', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('binary', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('bounding_box', cv2.WINDOW_AUTOSIZE)
    cv2.startWindowThread()

    if mode == 'f':
        bbox = OpticalFlow()
    elif mode == 'm':
        bbox = MOG2()
    else:
        raise ValueError('Invalid filter: ' + mode)

    key = None
    print('Press q to exit.')
    try:
        while key != ord('q'):
            success, frame = cap.read()
            frame = cv2.resize(frame, (640, 480))
            cv2.imshow('original', frame)

            bbox.apply(frame)
            cv2.imshow('binary', bbox.binary_image())
            bbox.draw_bboxes(frame)
            cv2.imshow('bounding_box', frame)

            key = cv2.waitKey(30) & 0xff
    finally:
        cap.release()
        cv2.waitKey(1)
        cv2.destroyAllWindows()
        cv2.waitKey(1)

if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'f'
    source = sys.argv[2] if len(sys.argv) > 2 else 0
    main(mode, source)
