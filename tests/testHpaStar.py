#!/usr/bin/env python
import logging
logger = logging.getLogger(__name__)
import os
from matplotlib import pyplot as plt
import cv2
import numpy as np

# from policy.hpa_star import *
from policy import timing

TESTPATH = os.path.dirname(os.path.abspath(__file__))

def test_find_segments(border):
    # border: 2xn array
    logger.info("Checking border:\n{}".format(border))
    entrances = []
    new_seg = True
    seg_start = None
    seg_end = None
    for i,col in enumerate(border.T):
        if not col.any():
            if new_seg == True: 
                seg_start = i
                new_seg = False

            if i == len(border.T) - 1:
                logger.info("Entrance segment: ({},{})".format(seg_start, i))
                entrances.append((i - seg_start)/2 + seg_start)
        else:
            new_seg = True
            if seg_start != None:
                logger.info("Entrance segment: ({},{})".format(seg_start, i-1))
                entrances.append((i - 1 - seg_start)/2 + seg_start)

        logger.info("i: {}, col: {}, new_seg: {}, seg_start: {}"
                    .format(i, col, new_seg, seg_start))

    logger.info(entrances)

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)

    imgfile = TESTPATH + '/00.pgm'
    img = cv2.imread(imgfile, cv2.IMREAD_GRAYSCALE)

    # hpa = HpaStar(img, 20, 10)
    # hpa.draw_clusters()

    test_find_segments(np.array([[0,0,0,99,0],[0,0,0,0,0]]))
    test_find_segments(np.array([[0,0,0,99,0,0,0,99],[99,0,0,99,0,0,0,0]]))
