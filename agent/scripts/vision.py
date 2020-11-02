#!/usr/bin/env python

import rospy

from environment.srv import * 
from agent.srv import *

from util.image_converter import ImageConverter

IC = None

def is_visible_callback(req):
    return IC.is_visible(req.object)

################################################################################

def main():
    rospy.init_node("vision_node")
    # rospy.wait_for_message("/models_loaded", Bool) 
    
    global IC
    IC = ImageConverter()
    
    rospy.Service("is_visible_srv", IsVisibleSrv, is_visible_callback)

    rospy.spin()

    return 0 
################################################################################

if __name__ == "__main__":
    main()
