#!/usr/bin/env python

import rospy

# import argparse
# import struct
# import sys
# import copy
# import numpy as np

# import rospkg

# from gazebo_msgs.srv import (
#     SpawnModel,
#     DeleteModel,
# )
# from geometry_msgs.msg import (
#     PoseStamped,
#     Pose,
#     Point,
#     Quaternion,
# )
# from std_msgs.msg import (
#     Header,
#     Empty,
# )

# from util.data_conversion import *
from environment.srv import * 
from pddl.srv import *
from pddl.msg import *
from agent.srv import *

from util.knowledge_base.knowledge_base import KnowledgeBase

KB = KnowledgeBase()
getObjLoc = rospy.ServiceProxy('object_location_srv', ObjectLocationSrv)
executionInfo = rospy.ServiceProxy('get_offset', GetHardcodedOffsetSrv)
orientationSolver = rospy.ServiceProxy('calc_gripper_orientation_pose', CalcGripperOrientationPoseSrv)

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

def get_action_info(req):
    name = req.actionName
    action = KB.getAction(name)
    argNames = action.getExecutionArgNames()
    paramNames = [x.getName() for x in action.getParams()]
    paramDefaults = [x.getDefaultVal() for x in action.getParams()]
    paramMins = [x.getMin() for x in action.getParams()]
    paramMaxs = [x.getMax() for x in action.getParams()]

    return ActionInfo(name, 
                      argNames, 
                      paramNames, 
                      paramDefaults, 
                      paramMins, 
                      paramMaxs)

def handle_get_pddl_instatiations(req):    
    name = req.actionName
    action = KB.getAction(name)
    args = req.orderedArgs 

    # info = executionInfo('cover', 'left')
    # info2 = orientationSolver('left_gripper', 'cover', 'left')

    # This is not a correct assumption to make
    # locs = [poseStampedToString(getObjLoc(x).location) for x in args]
    locs = ['A', 'B']
    preConds = action.get_instatiated_preconditions(args, locs)
    effects = action.get_instatiated_effects(args, locs)

    return ActionPDDLBinding(name, preConds, effects)


def add_action_to_KB(req):    
    new_name = req.new_action_name
    param_names = req.param_names
    param_assignments = req.param_assignments
    new_effects = req.new_effects
    
    try: 
        assert(len(param_names) == len(param_assignments))
        new_action = copy.deepcopy(KB.getAction(req.orig_action_name))
        new_action.setName(new_name)
        new_action.setEffects(effects)
        for i in range(len(param_names)):
            new_action.setParam(param_names[i], param_assignments[i])
        KB.addAction(new_action)
        return AddActionToKBSrvResponse(True)
    except:
        return AddActionToKBSrvResponse(False)

################################################################################

def main():
    rospy.init_node("knowledge_base_node")

    rospy.Service("get_KB_domain_srv", GetKBDomainSrv, handle_domain_req)
    rospy.Service("get_KB_action_info_srv", GetKBActionInfoSrv, get_action_info)
    rospy.Service("get_KB_action_locs", GetKBActionLocsSrv, handle_action_locs_req)
    rospy.Service("get_KB_pddl_locs", GetKBPddlLocsSrv, handle_pddlLocs_req)
    rospy.Service("get_pddl_instatiations_srv", GetActionPDDLBindingSrv, handle_get_pddl_instatiations)
    rospy.ServiceProxy("add_action_to_KB_srv", AddActionToKBSrv, add_action_to_KB)
    rospy.spin()

    return 0 
################################################################################

if __name__ == "__main__":
    main()


