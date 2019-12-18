#!/usr/bin/env python

import sys
import rospy
from agent.srv import *

def move_arm_client(loc):
    rospy.wait_for_service('move_arm')
    try:
        move_arm = rospy.ServiceProxy('move_arm', MoveArm)
        resp1 = move_arm(loc)
        return resp1.success_bool
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [loc]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        loc = int(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s"%(loc)
    print "%s"%(loc, move_arm(loc))
