# use MoveBaseSeq class from 
# - reminder: move base seq takes in a path and executes it
#       but if path is blocked it will use openloop to try to get to the goal (map.goal)

# for now just do one task execution given maps

from openloop import MoveBaseSeq
import rospy, rospkg 

def import_maps():

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    pkgdir = rospack.get_path('policy')
    mapdir = pkgdir + '/maps/'

    # import maps
    # run rpp
    # execute policy
        # follow path
        # once path is done, make observation
        # set next leg of outcome as next path
    
