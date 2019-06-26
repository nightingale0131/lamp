#!/usr/bin/env python

import logging
logger = logging.getLogger(__name__)
import os
import rospy, rospkg

import cv2
import numpy as np

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    pkgdir = rospack.get_path('policy')
    mapdir = pkgdir + '/tests/maps/'

    img = cv2.imread(mapdir + 'problem.pgm')
    img[np.where(img == 0)] = 255
    thresh = cv2.threshold(img, 125, 255, cv2.THRESH_BINARY)[1]

    cv2.imshow("Thresholded", thresh)
    cv2.waitKey(0)

