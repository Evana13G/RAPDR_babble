#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from pddl.srv import *
from pddl.msg import *
from util.knowledge_base.knowledge_base import KnowledgeBase

KB = KnowledgeBase()

def handle_domain_req(req):
    domainDict = KB.getDomainData()
    domainName = domainDict['domain']
    types = domainDict['types']
    predicates = domainDict['predicates']
    requirements = domainDict['requirements']
    actions = domainDict['actions']
    return Domain(domainName, requirements, types, predicates, actions)

def handle_pddlLocs_req(req):
    domainDict = KB.getDomainData()
    return(domainDict['pddlLocs'])

def handle_action_locs_req(req):
    return KB.getActionsLocs()

################################################################################

def main():
    rospy.init_node("knowledge_base_node")

    # Could do just one general get request, all using the same srv file... 
    # well their return would be diff so maybe no
    s1 = rospy.Service("get_KB_domain_srv", GetKBDomainSrv, handle_domain_req)
    # s2 = rospy.Service("get_KB_actions", GetKBDataSrv, get_actions)
    s3 = rospy.Service("get_KB_action_locs", GetKBActionLocsSrv, handle_action_locs_req)
    s4 = rospy.Service("get_KB_pddl_locs", GetKBPddlLocsSrv, handle_pddlLocs_req)

    rospy.spin()

    return 0 
################################################################################

if __name__ == "__main__":
    main()
