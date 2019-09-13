#!/usr/bin/env python
"""
File to test BoundingBox methods
"""

import logging
logger = logging.getLogger(__name__)
import os

from policy.gridgraph import BoundingBox

def test_mod_bound():
    box = BoundingBox(1, (1,1), (2,1))

    # mod_bound input: (minx, maxx, miny, maxy) 
    # case 1: inside bounds
    modded = box.mod_bounds([-1, 5, -2, 2])
    print(modded)

    # case 2: outside bounds
    modded = box.mod_bounds([-1, 5, 3, 5])
    print(modded)

    # case 3: bottom right corner outside bounds
    modded = box.mod_bounds([-1, 1, 1, 5])
    print(modded)

    # case
    box = BoundingBox(1, (-2,0), (0, -2))
    modded = box.mod_bounds([-5, 0, 0, 5])
    print(modded)

    box = BoundingBox(1, (-1, -4), (4, 1))
    modded = box.mod_bounds([-5, 0, 0, 5])
    print(modded)


if __name__ == '__main__':
    testpath = os.path.dirname(os.path.abspath(__file__))
    logging.basicConfig(filename=testpath + '/debug.log', filemode='w',level=logging.INFO)

    test_mod_bound()
