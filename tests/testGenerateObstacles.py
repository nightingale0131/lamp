#!/usr/bin/env python

from policy.generate_obstacles import *

cmd = spawn_obstacles()
raw_input('Press any key to delete obstacles')
delete_obstacles(cmd)
